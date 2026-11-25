// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/vector3_stamped_hcm_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

template <>
ConversionResult ConvertContractToHcm<Vector3StampedHcmContract, Vector3StampedHcmDescriptor>(
    const Vector3StampedHcmContract & vector,
    const DmaBufView & /*payload*/,
    Vector3StampedHcmDescriptor & descriptor)
{
  // Everything that can fail is checked before the descriptor is written, so a
  // rejected message leaves the host's descriptor untouched.
  ConversionResult validation = ValidateVector3Stamped(vector);
  if (!validation)
    return validation;
  if (vector.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN) {
    return ConversionResult::Failure(
        "frame_id is longer than the HCM Vector3Stamped descriptor holds");
  }

  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(vector.header().stamp(), timestampNs);
  if (!time)
    return time;

  if (!AssignHcmString(vector.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM Vector3Stamped descriptor");
  descriptor.timestampNs = timestampNs;
  Vector3ToHcm(vector.vector(), descriptor.vector);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<Vector3StampedHcmContract, Vector3StampedHcmDescriptor>(
    const Vector3StampedHcmDescriptor & descriptor,
    Vector3StampedHcmContract & vector)
{
  vector.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *vector.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  vector.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  Vector3ToContract(descriptor.vector, *vector.mutable_vector());

  // A non-finite value produced on the HCM side must not be published to ROS.
  return ValidateVector3Stamped(vector);
}

}  // namespace qrb::adapter
