// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_node.hpp"

#include <chrono>
#include <cstring>

namespace qrb::adapter
{

namespace
{

QC::QCStatus_e ToQcStatus( Status status )
{
    switch ( status )
    {
        case Status::kOk: return QC::QC_STATUS_OK;
        case Status::kNoData:
        case Status::kTimeout: return QC::QC_STATUS_TIMEOUT;
        case Status::kBadState: return QC::QC_STATUS_BAD_STATE;
        case Status::kInvalidArgument: return QC::QC_STATUS_BAD_ARGUMENTS;
        case Status::kUnsupportedContract:
        case Status::kVersionMismatch: return QC::QC_STATUS_UNSUPPORTED;
        case Status::kResourceError: return QC::QC_STATUS_NO_RESOURCE;
        case Status::kProtocolError:
        case Status::kConversionError:
        case Status::kInternalError: return QC::QC_STATUS_FAIL;
    }
    return QC::QC_STATUS_FAIL;
}

}   // namespace

QC::QCStatus_e HcmIpcAdapterNodeConfig::VerifyAndSet(
    const std::string config, std::string &errors )
{
    HcmIpcAdapterConfig parsed;
    Status status = ParseHcmIpcAdapterConfigJson( config, parsed, errors );
    if ( status != Status::kOk ) return ToQcStatus( status );

    config_.adapter = std::move( parsed );
    config_.nodeId.name = config_.adapter.nodeName;
    config_.nodeId.id = static_cast<std::uint8_t>( config_.adapter.nodeId );
    config_.nodeId.type = QC::QC_NODE_TYPE_CUSTOM_0;
    config_.numOfEntries = 1;
    return QC::QC_STATUS_OK;
}

const std::string &HcmIpcAdapterNodeConfig::GetOptions()
{
    if ( options_.empty() ) options_ = HcmIpcAdapterConfigOptions();
    return options_;
}

const QC::QCNodeConfigBase_t &HcmIpcAdapterNodeConfig::Get() { return config_; }

HcmIpcAdapterNodeMonitoring::HcmIpcAdapterNodeMonitoring()
{
    data_.numOfEntries = 6;
}

QC::QCStatus_e HcmIpcAdapterNodeMonitoring::VerifyAndSet(
    const std::string /*config*/, std::string & /*errors*/ )
{
    return QC::QC_STATUS_OK;
}

const std::string &HcmIpcAdapterNodeMonitoring::GetOptions()
{
    if ( options_.empty() ) options_ = R"({"format":"HcmIpcAdapterMonitoringData"})";
    return options_;
}

const QC::QCNodeMonitoringBase_t &HcmIpcAdapterNodeMonitoring::Get() { return data_; }

std::uint32_t HcmIpcAdapterNodeMonitoring::GetMaximalSize()
{
    return sizeof( HcmIpcAdapterMonitoringData );
}

std::uint32_t HcmIpcAdapterNodeMonitoring::GetCurrentSize()
{
    return sizeof( HcmIpcAdapterMonitoringData );
}

QC::QCStatus_e HcmIpcAdapterNodeMonitoring::Place( void *data, std::uint32_t &size )
{
    if ( data == nullptr ) return QC::QC_STATUS_BAD_ARGUMENTS;
    if ( size < sizeof( HcmIpcAdapterMonitoringData ) )
    {
        size = sizeof( HcmIpcAdapterMonitoringData );
        return QC::QC_STATUS_OUT_OF_BOUND;
    }

    std::lock_guard<std::mutex> lock( mutex_ );
    std::memcpy( data, &data_, sizeof( data_ ) );
    size = sizeof( data_ );
    return QC::QC_STATUS_OK;
}

void HcmIpcAdapterNodeMonitoring::UpdateSession( bool sessionConnected )
{
    std::lock_guard<std::mutex> lock( mutex_ );
    data_.sessionConnected = sessionConnected ? 1u : 0u;
}

void HcmIpcAdapterNodeMonitoring::RecordProcess( QC::QCStatus_e status,
    bool sessionConnected, bool latencyMeasured, std::uint64_t latencyNs )
{
    std::lock_guard<std::mutex> lock( mutex_ );
    data_.sessionConnected = sessionConnected ? 1u : 0u;
    ++data_.frameCount;
    if ( status != QC::QC_STATUS_OK ) ++data_.errorCount;
    if ( latencyMeasured )
    {
        totalLatencyNs_ += latencyNs;
        data_.lastLatencyNs = latencyNs;
        data_.averageLatencyNs = totalLatencyNs_ / data_.frameCount;
        if ( latencyNs > data_.maxLatencyNs ) data_.maxLatencyNs = latencyNs;
    }
}

void HcmIpcAdapterNodeMonitoring::Reset()
{
    std::lock_guard<std::mutex> lock( mutex_ );
    data_ = {};
    data_.numOfEntries = 6;
    totalLatencyNs_ = 0;
}

HcmIpcAdapterNode::~HcmIpcAdapterNode()
{
    std::lock_guard<std::mutex> lock( mutex_ );
    client_.Close();
}

QC::QCStatus_e HcmIpcAdapterNode::Initialize( QC::QCNodeInit_t &config )
{
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( state_.load() != QC::QC_OBJECT_STATE_INITIAL ) return QC::QC_STATUS_BAD_STATE;

    monitoring_.Reset();
    state_.store( QC::QC_OBJECT_STATE_INITIALIZING );
    std::string diagnostic;
    QC::QCStatus_e configStatus = config_.VerifyAndSet( config.config, diagnostic );
    if ( configStatus != QC::QC_STATUS_OK )
    {
        state_.store( QC::QC_OBJECT_STATE_INITIAL );
        UpdateMonitoring();
        return configStatus;
    }

    Status initialized = client_.Initialize( config_.Adapter().flowConfig,
        config_.Adapter().bootstrapSocketPath, diagnostic );
    QC::QCStatus_e result = ToQcStatus( initialized );
    if ( initialized != Status::kOk )
    {
        client_.Close();
    }
    sessionConnected_ = initialized == Status::kOk;
    state_.store( initialized == Status::kOk ? QC::QC_OBJECT_STATE_READY
                                             : QC::QC_OBJECT_STATE_INITIAL );
    UpdateMonitoring();
    return result;
}

QC::QCStatus_e HcmIpcAdapterNode::Start()
{
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( state_.load() != QC::QC_OBJECT_STATE_READY ) return QC::QC_STATUS_BAD_STATE;

    state_.store( QC::QC_OBJECT_STATE_STARTING );
    state_.store( QC::QC_OBJECT_STATE_RUNNING );
    UpdateMonitoring();
    return QC::QC_STATUS_OK;
}

QC::QCStatus_e HcmIpcAdapterNode::ProcessFrameDescriptor(
    QC::QCFrameDescriptorNodeIfs &frameDesc )
{
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( state_.load() != QC::QC_OBJECT_STATE_RUNNING ) return QC::QC_STATUS_BAD_STATE;

    const bool measureLatency = config_.Adapter().enableLatencyMonitoring;
    std::chrono::steady_clock::time_point start;
    if ( measureLatency ) start = std::chrono::steady_clock::now();
    const QC::QCStatus_e result =
        config_.Adapter().flowConfig.direction == Direction::kRosToHcm
        ? ProcessRosToHcm( frameDesc )
        : ProcessHcmToRos( frameDesc );
    std::uint64_t latencyNs = 0;
    if ( measureLatency )
    {
        const auto latency = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start );
        latencyNs = static_cast<std::uint64_t>( latency.count() );
    }
    monitoring_.RecordProcess( result, sessionConnected_, measureLatency, latencyNs );
    return result;
}

QC::QCStatus_e HcmIpcAdapterNode::ProcessRosToHcm(
    QC::QCFrameDescriptorNodeIfs &frameDesc )
{
    const HcmFrameBufferLayout &layout = config_.Adapter().buffers;
    QC::Memory::QCBufferDescriptorBase_t &descriptor =
        frameDesc.GetBuffer( layout.descriptorLocalSlot );
    return FinishOperation( client_.Fetch( descriptor ) );
}

QC::QCStatus_e HcmIpcAdapterNode::ProcessHcmToRos(
    QC::QCFrameDescriptorNodeIfs &frameDesc )
{
    const HcmFrameBufferLayout &layout = config_.Adapter().buffers;
    QC::Memory::QCBufferDescriptorBase_t &descriptor =
        frameDesc.GetBuffer( layout.descriptorLocalSlot );
    return FinishOperation( client_.Publish( descriptor ) );
}

QC::QCStatus_e HcmIpcAdapterNode::FinishOperation( Status status )
{
    if ( IsFatalForFlow( status ) )
    {
        client_.Close();
        sessionConnected_ = false;
        state_.store( QC::QC_OBJECT_STATE_ERROR );
    }
    return ToQcStatus( status );
}

QC::QCStatus_e HcmIpcAdapterNode::Stop()
{
    std::lock_guard<std::mutex> lock( mutex_ );
    const QC::QCObjectState_e current = state_.load();
    if ( current != QC::QC_OBJECT_STATE_RUNNING && current != QC::QC_OBJECT_STATE_ERROR )
    {
        return QC::QC_STATUS_BAD_STATE;
    }
    if ( current == QC::QC_OBJECT_STATE_ERROR )
    {
        UpdateMonitoring();
        return QC::QC_STATUS_OK;
    }

    state_.store( QC::QC_OBJECT_STATE_STOPING );
    state_.store( QC::QC_OBJECT_STATE_READY );
    UpdateMonitoring();
    return QC::QC_STATUS_OK;
}

QC::QCStatus_e HcmIpcAdapterNode::DeInitialize()
{
    std::lock_guard<std::mutex> lock( mutex_ );
    const QC::QCObjectState_e current = state_.load();
    if ( current == QC::QC_OBJECT_STATE_INITIAL ) return QC::QC_STATUS_BAD_STATE;
    if ( current == QC::QC_OBJECT_STATE_RUNNING || current == QC::QC_OBJECT_STATE_STARTING
        || current == QC::QC_OBJECT_STATE_STOPING )
    {
        return QC::QC_STATUS_BAD_STATE;
    }

    state_.store( QC::QC_OBJECT_STATE_DEINITIALIZING );
    client_.Close();
    sessionConnected_ = false;
    QC::QCStatus_e result = QC::QC_STATUS_OK;
    state_.store( QC::QC_OBJECT_STATE_INITIAL );
    UpdateMonitoring();
    return result;
}

QC::QCObjectState_e HcmIpcAdapterNode::GetState() { return state_.load(); }

QC::QCNodeConfigIfs &HcmIpcAdapterNode::GetConfigurationIfs() { return config_; }

QC::QCNodeMonitoringIfs &HcmIpcAdapterNode::GetMonitoringIfs() { return monitoring_; }

void HcmIpcAdapterNode::UpdateMonitoring()
{
    monitoring_.UpdateSession( sessionConnected_ );
}

QC::QCStatus_e CreateHcmIpcAdapterNode(
    QC::QCNodeType_e type, std::unique_ptr<QC::QCNodeIfs> &outNode )
{
    outNode.reset();
    if ( type != QC::QC_NODE_TYPE_CUSTOM_0 ) return QC::QC_STATUS_UNSUPPORTED;
    outNode = std::make_unique<HcmIpcAdapterNode>();
    return outNode != nullptr ? QC::QC_STATUS_OK : QC::QC_STATUS_NO_RESOURCE;
}

}   // namespace qrb::adapter
