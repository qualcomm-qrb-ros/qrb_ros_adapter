// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_stamped_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult PoseStampedRosConversion::RosToContract(const PoseStampedRosMessage & source,
    PoseStampedRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  PoseToContract(source.pose, *target.mutable_pose());

  return ValidatePoseStamped(target);
}

ConversionResult PoseStampedRosConversion::ContractToRos(const PoseStampedRosContract & source,
    PoseStampedRosMessage & target)
{
  ConversionResult validation = ValidatePoseStamped(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);
  PoseToRos(source.pose(), target.pose);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
