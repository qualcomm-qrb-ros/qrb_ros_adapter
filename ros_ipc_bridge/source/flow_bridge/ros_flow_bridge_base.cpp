// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_flow_bridge_base.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"
#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"
#include "qrb_ros_adapter/ipc_runtime/transport_codec.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <string_view>

#include <unistd.h>

namespace qrb::adapter
{

namespace
{

bool ParseConfiguredDirection( std::string_view text, Direction &out )
{
    if ( text == "ros_to_hcm" )
    {
        out = Direction::kRosToHcm;
        return true;
    }
    if ( text == "hcm_to_ros" )
    {
        out = Direction::kHcmToRos;
        return true;
    }
    return false;
}

/**
 * @brief How long the service thread waits for a frame before re-checking the
 * stop flag.
 *
 * Long enough that an idle Flow costs nothing measurable, short enough that
 * destroying a Flow is not perceptibly delayed.
 */
constexpr std::chrono::milliseconds kServicePollInterval{ 200 };
using Clock = std::chrono::steady_clock;

std::chrono::milliseconds Remaining( Clock::time_point deadline )
{
    const auto now = Clock::now();
    if ( now >= deadline ) return std::chrono::milliseconds( 0 );
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>( deadline - now );
    return remaining.count() > 0 ? remaining : std::chrono::milliseconds( 1 );
}

Status QueryReceivedBufferSize( int fd, std::uint64_t &outSize )
{
    if ( fd < 0 ) return Status::kInvalidArgument;
    const off_t end = ::lseek( fd, 0, SEEK_END );
    if ( end < 0 ) return Status::kResourceError;
    ::lseek( fd, 0, SEEK_SET );
    outSize = static_cast<std::uint64_t>( end );
    return Status::kOk;
}

}   // namespace

RosFlowBridgeBase::RosFlowBridgeBase(
    const std::string &nodeName, const rclcpp::NodeOptions &options,
    std::string_view contractTypeName, std::unique_ptr<RosFlowHandler> handler,
    FlowTerminalCallback terminalCallback )
    : rclcpp::Node( nodeName, options ), terminalCallback_( std::move( terminalCallback ) ),
      handler_( std::move( handler ) )
{
    try
    {
        const ContractDescriptor *descriptor = FindContractDescriptor( contractTypeName );
        Status initialized = descriptor == nullptr
            ? Status::kUnsupportedContract
            : Initialize( *descriptor );
        if ( initialized != Status::kOk )
        {
            if ( descriptor == nullptr ) diagnostic_ = "Contract Descriptor is not registered; ";
            throw std::runtime_error(
                nodeName + ": " + ToString( initialized ) + ": " + diagnostic_ );
        }
    }
    catch ( ... )
    {
        Close();
        throw;
    }
}

RosFlowBridgeBase::~RosFlowBridgeBase()
{
    Close();
}

Status RosFlowBridgeBase::DeclareAndReadParameters(
    FlowConfig &config, FlowRegistration &registration )
{
    // Contract type and payload class are fixed by the shared mapping. The
    // generated typed factory fixes the ROS endpoint strategy.
    config.flowId = this->declare_parameter<std::string>( "flow_id", "" );
    config.rosTopic = this->declare_parameter<std::string>( "ros_topic", "" );
    registration.dataSocketPath =
        this->declare_parameter<std::string>( "data_socket_path", "" );
    registration.instanceToken =
        this->declare_parameter<std::string>( "instance_token", "" );

    config.contractTypeName =
        this->declare_parameter<std::string>( "contract_type_name", "" );
    const std::int64_t timeoutMs = this->declare_parameter<std::int64_t>(
        "request_timeout_ms", static_cast<std::int64_t>( config.requestTimeoutMs ) );
    const std::int64_t sampleTimeoutMs = this->declare_parameter<std::int64_t>(
        "sample_timeout_ms", static_cast<std::int64_t>( config.sampleTimeoutMs ) );
    const std::int64_t startupTimeoutMs = this->declare_parameter<std::int64_t>(
        "startup_timeout_ms", static_cast<std::int64_t>( config.startupTimeoutMs ) );
    const std::int64_t qosDepth = this->declare_parameter<std::int64_t>(
        "qos_depth", static_cast<std::int64_t>( config.qosDepth ) );
    const std::string qosProfileText = this->declare_parameter<std::string>(
        "qos_profile", ToString( config.qosProfile ) );

    const std::string directionText = this->declare_parameter<std::string>( "direction", "" );
    if ( config.contractTypeName.empty() )
    {
        diagnostic_ += "contract_type_name must not be empty; ";
        return Status::kInvalidArgument;
    }
    constexpr std::int64_t kUint32Max = std::numeric_limits<std::uint32_t>::max();
    if ( timeoutMs < 0 || timeoutMs > kUint32Max || sampleTimeoutMs < 0
        || sampleTimeoutMs > kUint32Max || startupTimeoutMs < 0
        || startupTimeoutMs > kUint32Max || qosDepth < 0 || qosDepth > kUint32Max )
    {
        diagnostic_ += "timeout and qos_depth parameters must fit uint32; ";
        return Status::kInvalidArgument;
    }

    config.requestTimeoutMs = static_cast<std::uint32_t>( timeoutMs );
    config.sampleTimeoutMs = static_cast<std::uint32_t>( sampleTimeoutMs );
    config.startupTimeoutMs = static_cast<std::uint32_t>( startupTimeoutMs );
    config.qosDepth = static_cast<std::uint32_t>( qosDepth );

    if ( !ParseConfiguredDirection( directionText, config.direction ) )
    {
        diagnostic_ += "direction must be \"ros_to_hcm\" or \"hcm_to_ros\", got \"" + directionText
            + "\"; ";
        return Status::kInvalidArgument;
    }
    if ( !ParseQosProfile( qosProfileText, config.qosProfile ) )
    {
        diagnostic_ += "qos_profile is not supported: \"" + qosProfileText + "\"; ";
        return Status::kInvalidArgument;
    }
    return Status::kOk;
}

Status RosFlowBridgeBase::Initialize( const ContractDescriptor &descriptor )
{
    if ( handler_ == nullptr )
    {
        diagnostic_ = "ROS Flow handler must not be null; ";
        return Status::kInvalidArgument;
    }

    FlowConfig config;
    FlowRegistration registration;
    Status read = DeclareAndReadParameters( config, registration );
    if ( read != Status::kOk ) return read;
    if ( registration.dataSocketPath.empty() || registration.instanceToken.empty() )
    {
        diagnostic_ += "data_socket_path and instance_token must not be empty; ";
        return Status::kInvalidArgument;
    }

    // The startup gate runs before any socket or ROS entity exists.
    Status validated = ValidateFlowConfig( config, descriptor, diagnostic_ );
    if ( validated != Status::kOk ) return validated;

    config_ = config;
    registration_ = registration;
    descriptor_ = &descriptor;

    const auto *messageDescriptor =
        google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(
            config.contractTypeName );
    const auto *messagePrototype = messageDescriptor == nullptr
        ? nullptr
        : google::protobuf::MessageFactory::generated_factory()->GetPrototype(
              messageDescriptor );
    if ( messagePrototype == nullptr )
    {
        diagnostic_ += "contract type is not registered in the Protobuf generated pool; ";
        return Status::kUnsupportedContract;
    }
    contractMessage_.reset( messagePrototype->New() );

    Status entity = handler_->Open( *this, config_, diagnostic_ );
    if ( entity != Status::kOk )
    {
        diagnostic_ += "failed to open ROS Flow handler; ";
        return entity;
    }

    RCLCPP_INFO( this->get_logger(),
        "flow '%s': contract '%s', %s, topic '%s' (%s), socket '%s'",
        config.flowId.c_str(), config.contractTypeName.c_str(), ToString( config.direction ),
        config.rosTopic.c_str(), ToString( descriptor.payloadClass ),
        registration.dataSocketPath.c_str() );

    Status listened = session_.Listen( registration.dataSocketPath );
    if ( listened != Status::kOk )
    {
        diagnostic_ += "failed to listen on data socket; ";
        return listened;
    }
    serviceRunning_.store( true );
    serviceThread_ = std::thread( &RosFlowBridgeBase::ServiceLoop, this );

    return Status::kOk;
}

void RosFlowBridgeBase::ServiceLoop()
{
    std::string diagnostic;
    const auto deadline = Clock::now() + std::chrono::milliseconds( config_.startupTimeoutMs );

    Status accepted = Status::kTimeout;
    while ( serviceRunning_.load() && Remaining( deadline ).count() > 0 )
    {
        accepted = session_.Accept( std::min( kServicePollInterval, Remaining( deadline ) ) );
        if ( accepted == Status::kOk ) break;
        if ( accepted != Status::kTimeout )
        {
            Fail( accepted, "data socket accept failed" );
            return;
        }
    }
    if ( !serviceRunning_.load() ) return;
    if ( accepted != Status::kOk )
    {
        Fail( Status::kTimeout, "data socket accept timed out" );
        return;
    }

    const auto handshakeTimeout = Remaining( deadline );
    if ( handshakeTimeout.count() == 0 )
    {
        Fail( Status::kTimeout, "startup timed out before data handshake" );
        return;
    }
    Status handshake = session_.PerformServerHandshake(
        config_.flowId, registration_, handshakeTimeout, diagnostic );
    if ( handshake != Status::kOk )
    {
        Fail( handshake, "handshake failed: " + diagnostic );
        return;
    }

    RCLCPP_INFO(
        this->get_logger(), "flow '%s': data Session established", config_.flowId.c_str() );

    TransportHeader header;
    std::vector<std::uint8_t> payload;
    std::vector<OwnedFd> fds;
    while ( serviceRunning_.load() )
    {
        Status received = session_.ReceiveFrame( header, payload, fds, kServicePollInterval );

        if ( received == Status::kTimeout ) continue;   // re-check the stop flag
        if ( received != Status::kOk )
        {
            Fail( received, "receive failed" );
            return;
        }

        ++framesIn_;
        Status handled = HandleFrame( header, payload, fds );
        if ( handled != Status::kOk && IsFatalForFlow( handled ) )
        {
            // Centralize terminal reporting so no protocol-error branch can
            // stop this thread without giving Manager a reclamation event.
            Fail( handled, "fatal frame handling failure" );
            return;
        }
    }

}

Status RosFlowBridgeBase::HandleFrame( const TransportHeader &header,
    const std::vector<std::uint8_t> &payload, std::vector<OwnedFd> &fds )
{
    switch ( static_cast<Operation>( header.operation ) )
    {
        case Operation::kFetchRequest:
            if ( config_.direction != Direction::kRosToHcm )
            {
                Fail( Status::kProtocolError, "FETCH_REQUEST on an hcm_to_ros flow" );
                return Status::kProtocolError;
            }
            if ( !payload.empty() || !fds.empty() || header.flags != 0
                || header.payloadOffset != 0 )
            {
                Fail( Status::kProtocolError, "malformed FETCH_REQUEST" );
                return Status::kProtocolError;
            }
            return HandleFetchRequest( header.requestId );

        case Operation::kPublishRequest:
            if ( config_.direction != Direction::kHcmToRos )
            {
                Fail( Status::kProtocolError, "PUBLISH_REQUEST on a ros_to_hcm flow" );
                return Status::kProtocolError;
            }
            return HandlePublishRequest( header, payload, fds );

        default:
            Fail( Status::kProtocolError,
                "unexpected operation "
                    + std::string( ToString( static_cast<Operation>( header.operation ) ) ) );
            return Status::kProtocolError;
    }
}

Status RosFlowBridgeBase::HandleFetchRequest( std::uint64_t requestId )
{
    OutboundPayload outbound;
    std::string diagnostic;
    Status taken = handler_->Fetch( *contractMessage_, outbound, diagnostic,
        std::chrono::milliseconds( config_.sampleTimeoutMs ) );

    if ( taken == Status::kNoData )
    {
        // Report the exhausted sample wait without terminating the Flow. The
        // next GF invocation may issue another FETCH_REQUEST on this Session.
        ++noDataResponses_;
        Status sent = session_.SendStatusResponse(
            Operation::kFetchResponse, requestId, Status::kNoData, "" );
        if ( sent != Status::kOk ) Fail( sent, "NO_DATA FETCH_RESPONSE send failed" );
        return sent;
    }

    if ( taken != Status::kOk )
    {
        ++conversionErrors_;
        RCLCPP_WARN( this->get_logger(), "flow '%s': conversion failed: %s",
            config_.flowId.c_str(), diagnostic.c_str() );
        // Report the conversion result before the HCM client closes the
        // terminal Flow. The server keeps ownership of teardown via its
        // terminal event path.
        Status sent = session_.SendStatusResponse(
            Operation::kFetchResponse, requestId, taken, diagnostic );
        if ( sent != Status::kOk ) Fail( sent, "FETCH_RESPONSE send failed" );
        return sent;
    }

    const bool hasFd = outbound.hasPayload && outbound.fd >= 0;
    if ( hasFd && !outbound.keepAlive )
    {
        Fail( Status::kInternalError, "DMA-BUF payload has no ROS owner" );
        return Status::kInternalError;
    }
    Status sent = session_.SendProtoFrame( Operation::kFetchResponse, requestId, *contractMessage_,
        hasFd ? &outbound.fd : nullptr, hasFd ? 1 : 0, transport_flags::kHasData,
        hasFd ? outbound.offset : 0 );
    if ( sent != Status::kOk )
    {
        Fail( sent, "FETCH_RESPONSE send failed" );
        return sent;
    }
    ++framesOut_;

    if ( hasFd )
    {
        // Keep outbound.keepAlive held until HCM confirms that it has copied
        // the producer Buffer into GF-owned storage. Releasing after sendmsg()
        // alone would let a ROS-side pool reuse the same DMA-BUF too early.
        TransportHeader completionHeader;
        std::vector<std::uint8_t> completionPayload;
        std::vector<OwnedFd> completionFds;
        Status received = session_.ReceiveFrame( completionHeader, completionPayload,
            completionFds, std::chrono::milliseconds( config_.requestTimeoutMs ) );
        if ( received != Status::kOk )
        {
            Fail( received, "no FETCH_COMPLETE within request_timeout_ms" );
            return received;
        }
        ++framesIn_;

        if ( completionHeader.operation
                != static_cast<std::uint16_t>( Operation::kFetchComplete )
            || completionHeader.requestId != requestId || completionHeader.fdCount != 0 )
        {
            Fail( Status::kProtocolError, "malformed FETCH_COMPLETE" );
            return Status::kProtocolError;
        }

        ipc_proto::OperationStatus completion;
        if ( IpcSession::ParseProtoPayload( completionPayload, completion ) != Status::kOk )
        {
            Fail( Status::kProtocolError, "malformed FETCH_COMPLETE status" );
            return Status::kProtocolError;
        }
        const Status peerStatus = FromProtoStatusCode( completion.status() );
        if ( peerStatus != Status::kOk )
        {
            Fail( peerStatus,
                "HCM rejected fetched payload: " + completion.diagnostic_message() );
            return peerStatus;
        }
    }

    return Status::kOk;
}

Status RosFlowBridgeBase::HandlePublishRequest( const TransportHeader &header,
    const std::vector<std::uint8_t> &payload, std::vector<OwnedFd> &fds )
{
    Status parsed = IpcSession::ParseProtoPayload( payload, *contractMessage_ );
    if ( parsed != Status::kOk )
    {
        session_.SendStatusResponse(
            Operation::kPublishResponse, header.requestId, parsed, "malformed contract message" );
        Fail( parsed, "malformed PUBLISH_REQUEST payload" );
        return parsed;
    }

    const bool isDmaBuf = descriptor_->payloadClass == PayloadClass::kDmaBuf;

    InboundPayload inbound;
    if ( isDmaBuf )
    {
        if ( header.fdCount != 1 || fds.size() != 1 )
        {
            session_.SendStatusResponse( Operation::kPublishResponse, header.requestId,
                Status::kProtocolError, "DMA-BUF contract requires exactly one fd" );
            Fail( Status::kProtocolError, "PUBLISH_REQUEST fd count != 1" );
            return Status::kProtocolError;
        }
        inbound.fd = std::move( fds[0] );
        inbound.hasPayload = true;
        std::uint64_t bufferSize = 0;
        if ( QueryReceivedBufferSize( inbound.fd.Get(), bufferSize ) != Status::kOk
            || bufferSize == 0 || bufferSize > static_cast<std::uint64_t>( SIZE_MAX ) )
        {
            session_.SendStatusResponse( Operation::kPublishResponse, header.requestId,
                Status::kProtocolError, "invalid received DMA-BUF capacity" );
            Fail( Status::kProtocolError, "invalid received DMA-BUF capacity" );
            return Status::kProtocolError;
        }
        inbound.capacity = static_cast<std::size_t>( bufferSize );
        if ( header.payloadOffset > bufferSize
            || header.payloadOffset > static_cast<std::uint64_t>( SIZE_MAX ) )
        {
            session_.SendStatusResponse( Operation::kPublishResponse, header.requestId,
                Status::kProtocolError, "DMA-BUF payload offset exceeds capacity" );
            Fail( Status::kProtocolError, "DMA-BUF payload offset exceeds capacity" );
            return Status::kProtocolError;
        }
        inbound.offset = static_cast<std::size_t>( header.payloadOffset );
    }
    else if ( header.fdCount != 0 )
    {
        session_.SendStatusResponse( Operation::kPublishResponse, header.requestId,
            Status::kProtocolError, "inline contract must not carry an fd" );
        Fail( Status::kProtocolError, "inline PUBLISH_REQUEST carried an fd" );
        return Status::kProtocolError;
    }
    else if ( header.payloadOffset != 0 )
    {
        session_.SendStatusResponse( Operation::kPublishResponse, header.requestId,
            Status::kProtocolError, "inline contract must not carry a DMA-BUF offset" );
        Fail( Status::kProtocolError, "inline PUBLISH_REQUEST carried a DMA-BUF offset" );
        return Status::kProtocolError;
    }

    std::string diagnostic;
    Status published = handler_->Publish( *contractMessage_, inbound, diagnostic );

    if ( published != Status::kOk )
    {
        ++conversionErrors_;
        RCLCPP_WARN( this->get_logger(), "flow '%s': publish failed: %s",
            config_.flowId.c_str(), diagnostic.c_str() );
    }

    // Publish() returns only after any HCM payload has been copied
    // into ROS-owned storage. Therefore this response is also the synchronous
    // signal that Graph Framework may reclaim the HCM source buffer.
    Status sent = session_.SendStatusResponse(
        Operation::kPublishResponse, header.requestId, published, diagnostic );
    if ( sent != Status::kOk )
    {
        Fail( sent, "PUBLISH_RESPONSE send failed" );
        return sent;
    }
    ++framesOut_;

    // The response reaches the HCM client before it closes a terminal Flow.
    return Status::kOk;
}

void RosFlowBridgeBase::Fail( Status reason, const std::string &diagnostic )
{
    Status expected = Status::kOk;
    if ( !terminalReason_.compare_exchange_strong( expected, reason ) ) return;

    RCLCPP_ERROR( this->get_logger(), "flow '%s': %s (%s)", config_.flowId.c_str(),
        diagnostic.c_str(), ToString( reason ) );
    serviceRunning_.store( false );
    session_.Close();
    NotifyTerminal( reason, diagnostic );
}

void RosFlowBridgeBase::NotifyTerminal( Status reason, const std::string &diagnostic )
{
    if ( !terminalCallback_ ) return;
    bool expected = false;
    if ( !terminalNotified_.compare_exchange_strong( expected, true ) ) return;
    terminalCallback_( FlowTerminalEvent{
        config_.flowId, registration_.instanceToken, reason, diagnostic } );
}

void RosFlowBridgeBase::Close() noexcept
{
    if ( stopped_ ) return;
    stopped_ = true;

    // Flip the flag and let the service thread notice within one poll interval,
    // rather than closing the socket under a thread that may be blocked in
    // poll()/recvmsg() on it -- that would race two threads on the same fd.
    serviceRunning_.store( false );
    if ( handler_ != nullptr ) handler_->CancelPendingFetch();
    if ( serviceThread_.joinable() ) serviceThread_.join();

    // Only now that no service thread can touch them: the session, then the handler.
    session_.Close();
    if ( handler_ != nullptr ) handler_->Close();

}

}   // namespace qrb::adapter
