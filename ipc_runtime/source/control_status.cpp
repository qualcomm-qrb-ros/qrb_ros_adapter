// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"

namespace qrb::adapter
{

ipc_proto::StatusCode ToProtoStatusCode( Status status )
{
    switch ( status )
    {
        case Status::kOk: return ipc_proto::OK;
        case Status::kNoData: return ipc_proto::NO_DATA;
        case Status::kInvalidArgument: return ipc_proto::INVALID_ARGUMENT;
        case Status::kUnsupportedContract: return ipc_proto::UNSUPPORTED_CONTRACT;
        case Status::kProtocolError: return ipc_proto::PROTOCOL_ERROR;
        case Status::kConversionError: return ipc_proto::CONVERSION_ERROR;
        case Status::kResourceError: return ipc_proto::RESOURCE_ERROR;
        case Status::kInternalError: return ipc_proto::INTERNAL_ERROR;

        // Local-only: these describe transport or local call failures.
        case Status::kVersionMismatch: return ipc_proto::PROTOCOL_ERROR;
        case Status::kTimeout: return ipc_proto::PROTOCOL_ERROR;
        case Status::kBadState: return ipc_proto::INTERNAL_ERROR;
    }
    return ipc_proto::INTERNAL_ERROR;
}

Status FromProtoStatusCode( ipc_proto::StatusCode code )
{
    switch ( code )
    {
        case ipc_proto::OK: return Status::kOk;
        case ipc_proto::NO_DATA: return Status::kNoData;
        case ipc_proto::INVALID_ARGUMENT: return Status::kInvalidArgument;
        case ipc_proto::UNSUPPORTED_CONTRACT: return Status::kUnsupportedContract;
        case ipc_proto::PROTOCOL_ERROR: return Status::kProtocolError;
        case ipc_proto::CONVERSION_ERROR: return Status::kConversionError;
        case ipc_proto::RESOURCE_ERROR: return Status::kResourceError;
        case ipc_proto::INTERNAL_ERROR: return Status::kInternalError;

        case ipc_proto::STATUS_CODE_UNSPECIFIED: break;
        default: break;
    }
    // Never fall through to kOk: an unspecified or unknown code from a peer
    // must not be read as success.
    return Status::kProtocolError;
}

ipc_proto::Direction ToProtoDirection( Direction direction )
{
    switch ( direction )
    {
        case Direction::kRosToHcm: return ipc_proto::ROS_TO_HCM;
        case Direction::kHcmToRos: return ipc_proto::HCM_TO_ROS;
        case Direction::kUnspecified: break;
    }
    return ipc_proto::DIRECTION_UNSPECIFIED;
}

Direction FromProtoDirection( ipc_proto::Direction direction )
{
    switch ( direction )
    {
        case ipc_proto::ROS_TO_HCM: return Direction::kRosToHcm;
        case ipc_proto::HCM_TO_ROS: return Direction::kHcmToRos;
        case ipc_proto::DIRECTION_UNSPECIFIED: break;
        default: break;
    }
    return Direction::kUnspecified;
}

ipc_proto::QosProfile ToProtoQosProfile( QosProfile profile )
{
    switch ( profile )
    {
        case QosProfile::kReliable: return ipc_proto::QOS_PROFILE_RELIABLE;
        case QosProfile::kSensorData: return ipc_proto::QOS_PROFILE_SENSOR_DATA;
        case QosProfile::kTransientLocal: return ipc_proto::QOS_PROFILE_TRANSIENT_LOCAL;
        case QosProfile::kRosDefault: break;
    }
    return ipc_proto::QOS_PROFILE_ROS_DEFAULT;
}

QosProfile FromProtoQosProfile( ipc_proto::QosProfile profile )
{
    switch ( profile )
    {
        case ipc_proto::QOS_PROFILE_RELIABLE: return QosProfile::kReliable;
        case ipc_proto::QOS_PROFILE_SENSOR_DATA: return QosProfile::kSensorData;
        case ipc_proto::QOS_PROFILE_TRANSIENT_LOCAL: return QosProfile::kTransientLocal;
        case ipc_proto::QOS_PROFILE_ROS_DEFAULT:
            return QosProfile::kRosDefault;
        default: break;
    }
    return static_cast<QosProfile>( 0xFF );
}

}   // namespace qrb::adapter
