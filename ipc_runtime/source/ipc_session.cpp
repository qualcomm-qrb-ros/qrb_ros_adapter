// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/ipc_session.hpp"

#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"

#include <algorithm>
#include <thread>

namespace qrb::adapter
{

Status IpcSession::Listen( const std::string &socketPath )
{
    return endpoint_.Listen( socketPath );
}

Status IpcSession::Accept( std::chrono::milliseconds timeout )
{
    return endpoint_.Accept( timeout );
}

Status IpcSession::AcceptConnection( IpcSession &connectionOut,
    std::chrono::milliseconds timeout, int wakeFd, bool &interrupted )
{
    return endpoint_.AcceptConnection(
        connectionOut.endpoint_, timeout, wakeFd, interrupted );
}

Status IpcSession::Connect(
    const std::string &socketPath, std::chrono::milliseconds timeout )
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + timeout;

    for ( ;; )
    {
        Status connected = endpoint_.ConnectOnce( socketPath );
        if ( connected != Status::kTimeout ) return connected;

        const auto now = Clock::now();
        if ( now >= deadline ) return Status::kTimeout;
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>( deadline - now );
        if ( remaining.count() <= 0 ) return Status::kTimeout;
        std::this_thread::sleep_for(
            std::min( std::chrono::milliseconds( 10 ), remaining ) );
    }
}

Status IpcSession::SendSerialized( Operation operation, std::uint64_t requestId,
    const google::protobuf::MessageLite &message, const int *fds, std::uint32_t fdCount,
    std::uint32_t flags, std::uint64_t payloadOffset )
{
    const std::size_t size = message.ByteSizeLong();
    if ( size > kMaxPayloadSize ) return Status::kInvalidArgument;

    scratch_.resize( size );
    if ( size > 0 && !message.SerializeToArray( scratch_.data(), static_cast<int>( size ) ) )
    {
        return Status::kInternalError;
    }

    return endpoint_.SendFrame(
        operation, requestId, scratch_.data(), size, fds, fdCount, flags, payloadOffset );
}

Status IpcSession::SendProtoFrame( Operation operation, std::uint64_t requestId,
    const google::protobuf::MessageLite &message, const int *fds, std::uint32_t fdCount,
    std::uint32_t flags, std::uint64_t payloadOffset )
{
    return SendSerialized( operation, requestId, message, fds, fdCount, flags, payloadOffset );
}

Status IpcSession::SendEmptyFrame( Operation operation, std::uint64_t requestId )
{
    return endpoint_.SendFrame( operation, requestId, nullptr, 0 );
}

Status IpcSession::SendStatusResponse(
    Operation operation, std::uint64_t requestId, Status status, const std::string &diagnostic )
{
    ipc_proto::OperationStatus message;
    message.set_status( ToProtoStatusCode( status ) );
    if ( !diagnostic.empty() ) message.set_diagnostic_message( diagnostic );
    return SendSerialized( operation, requestId, message, nullptr, 0, 0 );
}

Status IpcSession::ReceiveFrame( TransportHeader &headerOut,
    std::vector<std::uint8_t> &payloadOut,
    std::vector<OwnedFd> &fdsOut, std::chrono::milliseconds timeout )
{
    return endpoint_.RecvFrame( headerOut, payloadOut, fdsOut, timeout );
}

Status IpcSession::ParseProtoPayload(
    const std::vector<std::uint8_t> &payload, google::protobuf::MessageLite &out )
{
    out.Clear();
    // An empty payload is a valid encoding of a proto3 message with every
    // field at its default, so this is not special-cased.
    if ( !out.ParseFromArray( payload.data(), static_cast<int>( payload.size() ) ) )
    {
        return Status::kProtocolError;
    }
    return Status::kOk;
}

Status IpcSession::PerformServerHandshake( const std::string &flowId,
    const FlowRegistration &registration, std::chrono::milliseconds timeout,
    std::string &diagnostic )
{
    if ( handshakeDone_ ) return Status::kBadState;

    TransportHeader header;
    std::vector<std::uint8_t> payload;
    std::vector<OwnedFd> fds;
    Status received = endpoint_.RecvFrame( header, payload, fds, timeout );
    if ( received != Status::kOk ) return received;

    if ( header.operation != static_cast<std::uint16_t>( Operation::kConnectRequest ) )
    {
        diagnostic += "expected CONNECT_REQUEST, got ";
        diagnostic += ToString( static_cast<Operation>( header.operation ) );
        diagnostic += "; ";
        return Status::kProtocolError;
    }
    // The handshake never carries a payload fd; `fds` closes them on scope
    // exit, but a peer sending one is speaking a protocol we do not implement.
    if ( header.fdCount != 0 )
    {
        diagnostic += "CONNECT_REQUEST must not carry file descriptors; ";
        return Status::kProtocolError;
    }

    ipc_proto::ConnectRequest request;
    Status parsed = ParseProtoPayload( payload, request );
    if ( parsed != Status::kOk )
    {
        diagnostic += "malformed ConnectRequest; ";
        return parsed;
    }

    // Registration already validated the Flow configuration and created this
    // dedicated endpoint. The handshake only binds the peer to that exact
    // registration instance.
    Status result = Status::kOk;
    if ( request.flow_id() != flowId )
    {
        diagnostic += "flow_id '" + request.flow_id() + "' != expected '" + flowId + "'; ";
        result = Status::kInvalidArgument;
    }
    else if ( request.instance_token() != registration.instanceToken )
    {
        diagnostic += "instance_token does not match the active registration; ";
        result = Status::kInvalidArgument;
    }

    ipc_proto::ConnectResponse response;
    response.set_status( ToProtoStatusCode( result ) );
    if ( !diagnostic.empty() ) response.set_diagnostic_message( diagnostic );

    // Sent even on rejection so the peer learns why, then the caller closes.
    // A send failure here does not change the outcome: a refused handshake
    // fails either way.
    Status sent = SendSerialized( Operation::kConnectResponse, header.requestId, response, nullptr,
        0, 0 );
    if ( result != Status::kOk ) return result;
    if ( sent != Status::kOk ) return sent;

    handshakeDone_ = true;
    return Status::kOk;
}

Status IpcSession::PerformClientHandshake( const std::string &flowId,
    const FlowRegistration &registration, std::chrono::milliseconds timeout,
    std::string &diagnostic )
{
    if ( handshakeDone_ ) return Status::kBadState;

    ipc_proto::ConnectRequest request;
    request.set_flow_id( flowId );
    request.set_instance_token( registration.instanceToken );

    Status sent = SendSerialized( Operation::kConnectRequest, 1, request, nullptr, 0, 0 );
    if ( sent != Status::kOk ) return sent;

    TransportHeader header;
    std::vector<std::uint8_t> payload;
    std::vector<OwnedFd> fds;
    Status received = endpoint_.RecvFrame( header, payload, fds, timeout );
    if ( received != Status::kOk ) return received;

    if ( header.operation != static_cast<std::uint16_t>( Operation::kConnectResponse ) )
    {
        diagnostic += "expected CONNECT_RESPONSE, got ";
        diagnostic += ToString( static_cast<Operation>( header.operation ) );
        diagnostic += "; ";
        return Status::kProtocolError;
    }

    ipc_proto::ConnectResponse response;
    Status parsed = ParseProtoPayload( payload, response );
    if ( parsed != Status::kOk )
    {
        diagnostic += "malformed ConnectResponse; ";
        return parsed;
    }

    Status peerStatus = FromProtoStatusCode( response.status() );
    if ( peerStatus != Status::kOk )
    {
        diagnostic += "peer refused handshake: " + response.diagnostic_message() + "; ";
        return peerStatus;
    }

    handshakeDone_ = true;
    return Status::kOk;
}

void IpcSession::Close() noexcept
{
    endpoint_.Close();
    handshakeDone_ = false;
}

}   // namespace qrb::adapter
