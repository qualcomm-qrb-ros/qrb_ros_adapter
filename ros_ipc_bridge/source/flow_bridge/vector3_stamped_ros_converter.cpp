// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/vector3_stamped_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult Vector3StampedRosConversion::RosToContract(const Vector3StampedRosMessage & source,
    Vector3StampedRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  Vector3ToContract(source.vector, *target.mutable_vector());

  return ValidateVector3Stamped(target);
}

ConversionResult Vector3StampedRosConversion::ContractToRos(
    const Vector3StampedRosContract & source,
    Vector3StampedRosMessage & target)
{
  ConversionResult validation = ValidateVector3Stamped(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);
  Vector3ToRos(source.vector(), target.vector);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
