// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_QOS_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_QOS_HPP

#include <cstddef>
#include <rclcpp/qos.hpp>

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"

namespace qrb::adapter
{

/** Standard ROS 2 publisher/subscription QoS uses KEEP_LAST depth 10. */
constexpr std::size_t kRosDefaultQosDepth = 10;

/** Keep one spare slot beyond the default DDS queue depth. */
constexpr int kDefaultRosDmaBufferPoolSize = 12;

inline std::size_t ResolveQosDepth(std::uint32_t requested)
{
  return requested == 0 ? kRosDefaultQosDepth : static_cast<std::size_t>(requested);
}

inline rclcpp::QoS MakeRosQos(const FlowConfig & config)
{
  rclcpp::QoS qos{ rclcpp::KeepLast(ResolveQosDepth(config.qosDepth)) };
  switch (config.qosProfile) {
    case QosProfile::kRosDefault:
      break;
    case QosProfile::kSensorData:
      qos.best_effort().durability_volatile();
      break;
    case QosProfile::kTransientLocal:
      qos.reliable().transient_local();
      break;
    case QosProfile::kReliable:
      qos.reliable().durability_volatile();
      break;
  }
  return qos;
}

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_QOS_HPP
