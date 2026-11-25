// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/pose_array_ros_converter.hpp"

#include <cstddef>

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult PoseArrayRosConversion::RosToContract(const PoseArrayRosMessage & source,
    PoseArrayRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  target.mutable_poses()->Reserve(static_cast<int>(source.poses.size()));
  for (const auto & pose : source.poses)
    PoseToContract(pose, *target.add_poses());

  return ValidatePoseArray(target);
}

ConversionResult PoseArrayRosConversion::ContractToRos(const PoseArrayRosContract & source,
    PoseArrayRosMessage & target)
{
  ConversionResult validation = ValidatePoseArray(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);
  target.poses.resize(static_cast<std::size_t>(source.poses_size()));
  for (int i = 0; i < source.poses_size(); ++i)
    PoseToRos(source.poses(i), target.poses[static_cast<std::size_t>(i)]);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
