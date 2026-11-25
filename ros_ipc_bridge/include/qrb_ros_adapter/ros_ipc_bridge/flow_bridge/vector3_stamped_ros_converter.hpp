// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_VECTOR3_STAMPED_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_VECTOR3_STAMPED_ROS_CONVERTER_HPP

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "qrb/contract/geometry/v1/vector3_stamped.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using Vector3StampedRosMessage = geometry_msgs::msg::Vector3Stamped;
using Vector3StampedRosContract = ::qrb::contract::geometry::v1::Vector3Stamped;

template <>
struct RosConversion<Vector3StampedRosMessage, Vector3StampedRosContract>
{
  using RosType = Vector3StampedRosMessage;
  using ContractType = Vector3StampedRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using Vector3StampedRosConversion =
    RosConversion<Vector3StampedRosMessage, Vector3StampedRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_VECTOR3_STAMPED_ROS_CONVERTER_HPP
