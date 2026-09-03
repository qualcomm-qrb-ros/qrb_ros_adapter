// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_ROS_CONVERTER_HPP

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using PoseRosMessage = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseRosContract = ::qrb::contract::geometry::v1::PoseWithCovarianceStamped;

template <>
struct RosConversion<PoseRosMessage, PoseRosContract>
{
  using RosType = PoseRosMessage;
  using ContractType = PoseRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using PoseRosConversion = RosConversion<PoseRosMessage, PoseRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_POSE_ROS_CONVERTER_HPP
