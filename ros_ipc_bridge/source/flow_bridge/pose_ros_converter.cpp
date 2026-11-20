// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult PoseRosConversion::RosToContract(const PoseRosMessage & source,
    PoseRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  PoseWithCovarianceToContract(source.pose, *target.mutable_pose());

  return ValidatePoseWithCovarianceStamped(target);
}

ConversionResult PoseRosConversion::ContractToRos(const PoseRosContract & source,
    PoseRosMessage & target)
{
  ConversionResult validation = ValidatePoseWithCovarianceStamped(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);
  PoseWithCovarianceToRos(source.pose(), target.pose);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
