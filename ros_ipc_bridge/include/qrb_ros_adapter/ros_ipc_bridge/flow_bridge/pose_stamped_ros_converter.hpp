// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_STAMPED_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_STAMPED_ROS_CONVERTER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "qrb/contract/geometry/v1/pose_stamped.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using PoseStampedRosMessage = geometry_msgs::msg::PoseStamped;
using PoseStampedRosContract = ::qrb::contract::geometry::v1::PoseStamped;

template <>
struct RosConversion<PoseStampedRosMessage, PoseStampedRosContract>
{
  using RosType = PoseStampedRosMessage;
  using ContractType = PoseStampedRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using PoseStampedRosConversion = RosConversion<PoseStampedRosMessage, PoseStampedRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_STAMPED_ROS_CONVERTER_HPP
