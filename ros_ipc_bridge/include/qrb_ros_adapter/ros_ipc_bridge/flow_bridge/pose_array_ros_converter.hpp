// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_ARRAY_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_ARRAY_ROS_CONVERTER_HPP

#include <geometry_msgs/msg/pose_array.hpp>

#include "qrb/contract/geometry/v1/pose_array.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using PoseArrayRosMessage = geometry_msgs::msg::PoseArray;
using PoseArrayRosContract = ::qrb::contract::geometry::v1::PoseArray;

template <>
struct RosConversion<PoseArrayRosMessage, PoseArrayRosContract>
{
  using RosType = PoseArrayRosMessage;
  using ContractType = PoseArrayRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using PoseArrayRosConversion = RosConversion<PoseArrayRosMessage, PoseArrayRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_ARRAY_ROS_CONVERTER_HPP
