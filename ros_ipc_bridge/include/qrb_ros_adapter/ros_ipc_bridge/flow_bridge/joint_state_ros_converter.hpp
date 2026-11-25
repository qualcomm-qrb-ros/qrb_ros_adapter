// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_JOINT_STATE_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_JOINT_STATE_ROS_CONVERTER_HPP

#include <sensor_msgs/msg/joint_state.hpp>

#include "qrb/contract/sensor/v1/joint_state.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using JointStateRosMessage = sensor_msgs::msg::JointState;
using JointStateRosContract = ::qrb::contract::sensor::v1::JointState;

/**
 * @brief sensor_msgs/JointState <-> Contract JointState.
 *
 * Each measurement array is empty (not supplied) or as long as `name`; an empty
 * array is preserved as empty in both directions and never filled with zeros.
 * On failure the output target is left unchanged.
 */
template <>
struct RosConversion<JointStateRosMessage, JointStateRosContract>
{
  using RosType = JointStateRosMessage;
  using ContractType = JointStateRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using JointStateRosConversion = RosConversion<JointStateRosMessage, JointStateRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_JOINT_STATE_ROS_CONVERTER_HPP
