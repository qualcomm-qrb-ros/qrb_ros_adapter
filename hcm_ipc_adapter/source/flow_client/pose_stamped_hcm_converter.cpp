// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_stamped_hcm_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

template <>
ConversionResult ConvertContractToHcm<PoseStampedHcmContract, PoseStampedHcmDescriptor>(
    const PoseStampedHcmContract & pose,
    const DmaBufView & /*payload*/,
    PoseStampedHcmDescriptor & descriptor)
{
  // Everything that can fail is checked before the descriptor is written, so a
  // rejected message leaves the host's descriptor untouched.
  ConversionResult validation = ValidatePoseStamped(pose);
  if (!validation)
    return validation;
  if (pose.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN) {
    return ConversionResult::Failure(
        "frame_id is longer than the HCM PoseStamped descriptor holds");
  }

  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(pose.header().stamp(), timestampNs);
  if (!time)
    return time;

  if (!AssignHcmString(pose.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM PoseStamped descriptor");
  descriptor.timestampNs = timestampNs;
  PoseToHcm(pose.pose(), descriptor.pose);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<PoseStampedHcmContract, PoseStampedHcmDescriptor>(
    const PoseStampedHcmDescriptor & descriptor,
    PoseStampedHcmContract & pose)
{
  pose.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *pose.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  pose.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  PoseToContract(descriptor.pose, *pose.mutable_pose());

  // A non-finite value produced on the HCM side must not be published to ROS.
  return ValidatePoseStamped(pose);
}

}  // namespace qrb::adapter
