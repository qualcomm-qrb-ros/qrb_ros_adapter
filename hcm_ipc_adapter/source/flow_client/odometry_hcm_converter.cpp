// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/odometry_hcm_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

template <>
ConversionResult ConvertContractToHcm<OdometryHcmContract, OdometryHcmDescriptor>(
    const OdometryHcmContract & odometry,
    const DmaBufView & /*payload*/,
    OdometryHcmDescriptor & descriptor)
{
  // Everything that can fail is checked before the descriptor is written, so a
  // rejected message leaves the host's descriptor untouched.
  ConversionResult validation = ValidateOdometry(odometry);
  if (!validation)
    return validation;
  if (odometry.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
    return ConversionResult::Failure("frame_id is longer than the HCM Odometry descriptor holds");
  if (odometry.child_frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
    return ConversionResult::Failure(
        "child_frame_id is longer than the HCM Odometry descriptor holds");

  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(odometry.header().stamp(), timestampNs);
  if (!time)
    return time;

  if (!AssignHcmString(odometry.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM Odometry descriptor");
  if (!AssignHcmString(odometry.child_frame_id(), descriptor.childFrameId))
    return ConversionResult::Failure("child_frame_id does not fit the HCM Odometry descriptor");
  descriptor.timestampNs = timestampNs;
  PoseWithCovarianceToHcm(odometry.pose(), descriptor.poseWithCovariance);
  TwistWithCovarianceToHcm(odometry.twist(), descriptor.twistWithCovariance);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<OdometryHcmContract, OdometryHcmDescriptor>(
    const OdometryHcmDescriptor & descriptor,
    OdometryHcmContract & odometry)
{
  odometry.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *odometry.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  odometry.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  odometry.set_child_frame_id(HcmStringToContract(descriptor.childFrameId));
  PoseWithCovarianceToContract(descriptor.poseWithCovariance, *odometry.mutable_pose());
  TwistWithCovarianceToContract(descriptor.twistWithCovariance, *odometry.mutable_twist());

  // A non-finite value produced on the HCM side must not be published to ROS.
  return ValidateOdometry(odometry);
}

}  // namespace qrb::adapter
