// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"

namespace qrb::adapter
{

namespace
{

/** @brief ROS topic names must be non-empty and absolute-or-relative, but
 * never contain whitespace; a typo here otherwise surfaces much later as an
 * obscure rclcpp exception during entity creation. */
bool IsPlausibleTopicName( const std::string &topic )
{
    if ( topic.empty() ) return false;
    for ( char c : topic )
    {
        if ( c == ' ' || c == '\t' || c == '\n' ) return false;
    }
    return true;
}

Status ValidateContractCapabilities(
    const ContractDescriptor &descriptor, Direction direction, std::string &diagnostic )
{
    if ( descriptor.typeName.empty() )
    {
        diagnostic += "registry contains an incomplete Contract Descriptor; ";
        return Status::kInternalError;
    }

    if ( direction == Direction::kUnspecified )
    {
        diagnostic += "direction unspecified; ";
        return Status::kUnsupportedContract;
    }

    return Status::kOk;
}

}   // namespace

Status ValidateFlowConfig(
    const FlowConfig &config, const ContractDescriptor &descriptor, std::string &diagnostic )
{
    if ( config.flowId.empty() )
    {
        diagnostic += "flow_id must not be empty; ";
        return Status::kInvalidArgument;
    }

    // A generated typed Flow Bridge implements exactly one Contract. Pointing
    // an instance at another type is a deployment error, not something to
    // reconcile at runtime.
    if ( config.contractTypeName != descriptor.typeName )
    {
        diagnostic += "contract_type_name '" + config.contractTypeName
            + "' does not match this Flow Bridge's descriptor '"
            + std::string( descriptor.typeName ) + "'; ";
        return Status::kInvalidArgument;
    }

    if ( !IsPlausibleTopicName( config.rosTopic ) )
    {
        diagnostic += "ros_topic '" + config.rosTopic + "' is empty or contains whitespace; ";
        return Status::kInvalidArgument;
    }

    if ( config.requestTimeoutMs < kMinRequestTimeoutMs
        || config.requestTimeoutMs > kMaxRequestTimeoutMs )
    {
        diagnostic += "request_timeout_ms " + std::to_string( config.requestTimeoutMs )
            + " outside [" + std::to_string( kMinRequestTimeoutMs ) + ", "
            + std::to_string( kMaxRequestTimeoutMs ) + "]; ";
        return Status::kInvalidArgument;
    }

    if ( config.sampleTimeoutMs < kMinSampleTimeoutMs
        || config.sampleTimeoutMs > kMaxSampleTimeoutMs )
    {
        diagnostic += "sample_timeout_ms " + std::to_string( config.sampleTimeoutMs )
            + " outside [" + std::to_string( kMinSampleTimeoutMs ) + ", "
            + std::to_string( kMaxSampleTimeoutMs ) + "]; ";
        return Status::kInvalidArgument;
    }

    if ( config.startupTimeoutMs < kMinStartupTimeoutMs
        || config.startupTimeoutMs > kMaxStartupTimeoutMs )
    {
        diagnostic += "startup_timeout_ms " + std::to_string( config.startupTimeoutMs )
            + " outside [" + std::to_string( kMinStartupTimeoutMs ) + ", "
            + std::to_string( kMaxStartupTimeoutMs ) + "]; ";
        return Status::kInvalidArgument;
    }

    switch ( config.qosProfile )
    {
        case QosProfile::kRosDefault:
        case QosProfile::kReliable:
        case QosProfile::kSensorData:
        case QosProfile::kTransientLocal: break;
        default:
            diagnostic += "qos_profile is not supported; ";
            return Status::kInvalidArgument;
    }

    if ( config.qosDepth > kMaxQosDepth )
    {
        diagnostic += "qos_depth " + std::to_string( config.qosDepth )
            + " outside [0, " + std::to_string( kMaxQosDepth ) + "]; ";
        return Status::kInvalidArgument;
    }

    return ValidateContractCapabilities( descriptor, config.direction, diagnostic );
}

}   // namespace qrb::adapter
