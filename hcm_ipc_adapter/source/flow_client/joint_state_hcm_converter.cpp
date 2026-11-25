// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/joint_state_hcm_converter.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace
{

using Joint = QRB::Memory::JointState_t;

/** @brief Validate host records; the flow client owns DMA-BUF CPU synchronization. */
bool JointBuffer(const JointStateHcmDescriptor & descriptor,
    std::size_t bytes,
    bool reading,
    void *& data)
{
  int fd = -1;
  const HcmBufferAccess access = InspectHcmBuffer(descriptor, fd);
  if (access != HcmBufferAccess::kHostMemory && access != HcmBufferAccess::kLocalDmaBuf)
    return false;
  const std::size_t available = descriptor.size - descriptor.offset;
  if (bytes > available)
    return false;
  if (reading && (descriptor.validSize > available || descriptor.validSize < bytes ||
                     descriptor.GetDataSize() < bytes))
    return false;
  data = descriptor.GetDataPtr();
  return data != nullptr && reinterpret_cast<std::uintptr_t>(data) % alignof(Joint) == 0;
}

}  // namespace

template <>
ConversionResult ConvertContractToHcm<JointStateHcmContract, JointStateHcmDescriptor>(
    const JointStateHcmContract & source,
    const DmaBufView & /*payload*/,
    JointStateHcmDescriptor & target)
{
  ConversionResult validation = ValidateJointState(source);
  if (!validation)
    return validation;
  const int count = source.name_size();
  if (source.position_size() != count || source.velocity_size() != count ||
      source.effort_size() != count)
    return ConversionResult::Failure(
        "HCM JointState requires position, velocity and effort for every joint");

  QRB::Memory::FrameId_t frameId;
  if (!AssignHcmString(source.header().frame_id(), frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM JointState descriptor");
  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(source.header().stamp(), timestampNs);
  if (!time)
    return time;

  const std::size_t bytes =
      JointStateHcmDescriptor::CalcBufferSize(static_cast<std::uint32_t>(count));
  void * data = nullptr;
  if (count > 0 && (bytes == 0 || !JointBuffer(target, bytes, false, data)))
    return ConversionResult::Failure("HCM JointState buffer is missing, too small or misaligned");

  // Prepare every record before touching the host's buffer, including all string assignments.
  std::vector<Joint> joints(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    Joint & joint = joints[static_cast<std::size_t>(i)];
    if (!AssignHcmString(source.name(i), joint.name))
      return ConversionResult::Failure("joint name does not fit the HCM JointState record");
    joint.position = source.position(i);
    joint.velocity = source.velocity(i);
    joint.effort = source.effort(i);
  }
  if (count > 0)
    std::memcpy(data, joints.data(), bytes);
  target.frameId = frameId;
  target.timestampNs = timestampNs;
  target.jointCount = static_cast<std::uint32_t>(count);
  target.validSize = bytes;
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<JointStateHcmContract, JointStateHcmDescriptor>(
    const JointStateHcmDescriptor & source,
    JointStateHcmContract & target)
{
  const std::uint32_t count = source.jointCount;
  if (count > contract_limits::kMaxJointCount)
    return ConversionResult::Failure("HCM JointState holds more joints than the contract limit");
  if (source.frameId.Size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
    return ConversionResult::Failure("HCM JointState frame_id size is invalid");

  JointStateHcmContract message;
  ConversionResult time =
      NanosToContractTime(source.timestampNs, *message.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  message.mutable_header()->set_frame_id(HcmStringToContract(source.frameId));
  const std::size_t bytes = JointStateHcmDescriptor::CalcBufferSize(count);
  void * data = nullptr;
  if (count > 0 && (bytes == 0 || !JointBuffer(source, bytes, true, data)))
    return ConversionResult::Failure("HCM JointState buffer is missing, incomplete or misaligned");

  std::vector<Joint> joints(count);
  if (count > 0)
    std::memcpy(joints.data(), data, bytes);
  for (const Joint & joint : joints) {
    // Reject malformed fixed-string metadata before reading its reported length.
    if (joint.name.Size() > QRB::Memory::QRB_JOINT_NAME_MAX_LEN)
      return ConversionResult::Failure("HCM JointState joint name size is invalid");
    message.add_name(HcmStringToContract(joint.name));
    message.add_position(joint.position);
    message.add_velocity(joint.velocity);
    message.add_effort(joint.effort);
  }
  ConversionResult validation = ValidateJointState(message);
  if (!validation)
    return validation;
  target.Swap(&message);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
