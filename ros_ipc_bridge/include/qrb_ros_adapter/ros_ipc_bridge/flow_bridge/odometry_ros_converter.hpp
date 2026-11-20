// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ODOMETRY_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ODOMETRY_ROS_CONVERTER_HPP

#include <nav_msgs/msg/odometry.hpp>

#include "qrb/contract/nav/v1/odometry.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using OdometryRosMessage = nav_msgs::msg::Odometry;
using OdometryRosContract = ::qrb::contract::nav::v1::Odometry;

template <>
struct RosConversion<OdometryRosMessage, OdometryRosContract>
{
  using RosType = OdometryRosMessage;
  using ContractType = OdometryRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using OdometryRosConversion = RosConversion<OdometryRosMessage, OdometryRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ODOMETRY_ROS_CONVERTER_HPP
