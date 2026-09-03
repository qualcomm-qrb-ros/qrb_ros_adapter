// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"

namespace qrb::adapter
{

const char *ToString( PayloadClass payloadClass )
{
    switch ( payloadClass )
    {
        case PayloadClass::kInline: return "inline";
        case PayloadClass::kDmaBuf: return "dma_buf";
    }
    return "unknown";
}

const char *ToString( Direction direction )
{
    switch ( direction )
    {
        case Direction::kRosToHcm: return "ros_to_hcm";
        case Direction::kHcmToRos: return "hcm_to_ros";
        case Direction::kUnspecified: return "unspecified";
    }
    return "unknown";
}

const char *ToString( QosProfile profile )
{
    switch ( profile )
    {
        case QosProfile::kRosDefault: return "ros_default";
        case QosProfile::kReliable: return "reliable";
        case QosProfile::kSensorData: return "sensor_data";
        case QosProfile::kTransientLocal: return "transient_local";
    }
    return "unknown";
}

bool ParseQosProfile( std::string_view text, QosProfile &out )
{
    if ( text == "ros_default" ) out = QosProfile::kRosDefault;
    else if ( text == "reliable" ) out = QosProfile::kReliable;
    else if ( text == "sensor_data" ) out = QosProfile::kSensorData;
    else if ( text == "transient_local" ) out = QosProfile::kTransientLocal;
    else return false;
    return true;
}

}   // namespace qrb::adapter
