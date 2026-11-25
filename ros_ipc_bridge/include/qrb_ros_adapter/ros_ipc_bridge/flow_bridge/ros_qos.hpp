// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_QOS_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_QOS_HPP

#include <cstddef>
#include <rclcpp/qos.hpp>

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"

namespace qrb::adapter
{

/**
 * Host storage pool of one HCM_TO_ROS Flow: the publisher's KEEP_LAST history, one buffer being
 * copied into and one spare.
 */
inline std::size_t RosDmaBufferPoolSize(const QosProfile & profile)
{
  return static_cast<std::size_t>(profile.depth) + 2;
}

inline rclcpp::QoS MakeRosQos(const FlowConfig & config)
{
  const QosProfile & profile = config.qosProfile;
  rclcpp::QoS qos{rclcpp::KeepLast(profile.depth)};
  if (profile.reliability == QosReliability::kBestEffort)
    qos.best_effort();
  else
    qos.reliable();
  if (profile.durability == QosDurability::kTransientLocal)
    qos.transient_local();
  else
    qos.durability_volatile();
  return qos;
}

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_QOS_HPP
