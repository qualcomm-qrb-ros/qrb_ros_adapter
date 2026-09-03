// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_TYPES_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_TYPES_HPP

#include <cstdint>
#include <string_view>

namespace qrb::adapter
{

enum class PayloadClass : std::uint8_t
{
    kInline,
    kDmaBuf,
};

enum class Direction : std::uint8_t
{
    kUnspecified = 0,
    kRosToHcm = 1,
    kHcmToRos = 2,
};

/**
 * @brief Restricted ROS QoS profiles carried by dynamic Flow registration.
 *
 * KEEP_LAST is intentionally fixed by the Adapter. A Flow may select the
 * reliability/durability profile and optionally override its bounded depth.
 */
enum class QosProfile : std::uint8_t
{
    kRosDefault = 0,
    kReliable = 1,
    kSensorData = 2,
    kTransientLocal = 3,
};

struct ContractDescriptor
{
    std::string_view typeName;
    PayloadClass payloadClass{ PayloadClass::kInline };
};

const char *ToString( PayloadClass payloadClass );
const char *ToString( Direction direction );
const char *ToString( QosProfile profile );
bool ParseQosProfile( std::string_view text, QosProfile &out );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_TYPES_HPP
