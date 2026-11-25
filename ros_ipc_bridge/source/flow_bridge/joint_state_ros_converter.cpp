// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/joint_state_ros_converter.hpp"

#include <utility>

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult JointStateRosConversion::RosToContract(const JointStateRosMessage & source,
    JointStateRosContract & target)
{
  // Bound ROS vectors before converting sizes to protobuf's int or copying data.
  if (source.name.size() > contract_limits::kMaxJointCount)
    return ConversionResult::Failure("JointState holds more joints than the contract limit");
  if ((!source.position.empty() && source.position.size() != source.name.size()) ||
      (!source.velocity.empty() && source.velocity.size() != source.name.size()) ||
      (!source.effort.empty() && source.effort.size() != source.name.size()))
    return ConversionResult::Failure("JointState measurements must be empty or match name size");
  for (const auto & name : source.name) {
    if (name.empty() || name.size() > contract_limits::kMaxJointNameLength)
      return ConversionResult::Failure("joint name is outside the contract limit");
  }
  ConversionResult frame = ValidateFrameId(source.header.frame_id);
  if (!frame)
    return frame;

  JointStateRosContract converted;
  HeaderToContract(source.header, *converted.mutable_header());
  converted.mutable_name()->Reserve(static_cast<int>(source.name.size()));
  for (const auto & name : source.name)
    converted.add_name(name);
  converted.mutable_position()->Assign(source.position.begin(), source.position.end());
  converted.mutable_velocity()->Assign(source.velocity.begin(), source.velocity.end());
  converted.mutable_effort()->Assign(source.effort.begin(), source.effort.end());

  ConversionResult validation = ValidateJointState(converted);
  if (!validation)
    return validation;

  target = std::move(converted);
  return ConversionResult::Success();
}

ConversionResult JointStateRosConversion::ContractToRos(const JointStateRosContract & source,
    JointStateRosMessage & target)
{
  ConversionResult validation = ValidateJointState(source);
  if (!validation)
    return validation;

  JointStateRosMessage converted;
  HeaderToRos(source.header(), converted.header);
  converted.name.assign(source.name().begin(), source.name().end());
  converted.position.assign(source.position().begin(), source.position().end());
  converted.velocity.assign(source.velocity().begin(), source.velocity().end());
  converted.effort.assign(source.effort().begin(), source.effort().end());

  target = std::move(converted);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
