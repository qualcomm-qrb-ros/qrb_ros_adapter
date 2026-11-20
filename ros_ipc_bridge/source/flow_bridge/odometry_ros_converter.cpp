// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/odometry_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult OdometryRosConversion::RosToContract(const OdometryRosMessage & source,
    OdometryRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  target.set_child_frame_id(source.child_frame_id);
  PoseWithCovarianceToContract(source.pose, *target.mutable_pose());
  TwistWithCovarianceToContract(source.twist, *target.mutable_twist());

  return ValidateOdometry(target);
}

ConversionResult OdometryRosConversion::ContractToRos(const OdometryRosContract & source,
    OdometryRosMessage & target)
{
  ConversionResult validation = ValidateOdometry(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);
  target.child_frame_id = source.child_frame_id();
  PoseWithCovarianceToRos(source.pose(), target.pose);
  TwistWithCovarianceToRos(source.twist(), target.twist);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
