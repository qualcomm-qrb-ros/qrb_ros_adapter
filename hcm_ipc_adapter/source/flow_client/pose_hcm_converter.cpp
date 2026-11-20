// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"

#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace geometry_proto = ::qrb::contract::geometry::v1;

template <>
ConversionResult ConvertContractToHcm<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmContract & pose,
    const DmaBufView & /*payload*/,
    PoseHcmDescriptor & descriptor)
{
  ConversionResult validation = ValidatePoseWithCovarianceStamped(pose);
  if (!validation)
    return validation;
  if (pose.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
    return ConversionResult::Failure("frame_id is longer than the HCM Pose descriptor holds");

  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(pose.header().stamp(), timestampNs);
  if (!time)
    return time;

  if (!AssignHcmString(pose.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM Pose descriptor");
  descriptor.timestampNs = timestampNs;
  PoseWithCovarianceToHcm(pose.pose(), descriptor.poseWithCovariance);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmDescriptor & descriptor,
    PoseHcmContract & pose)
{
  pose.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *pose.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  pose.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  PoseWithCovarianceToContract(descriptor.poseWithCovariance, *pose.mutable_pose());

  return ValidatePoseWithCovarianceStamped(pose);
}

}  // namespace qrb::adapter
