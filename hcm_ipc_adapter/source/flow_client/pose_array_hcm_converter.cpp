// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_array_hcm_converter.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace
{

/**
 * @brief How many poses fit the descriptor's buffer from its offset on. The flow client syncs the
 * buffer around the conversion. InspectHcmBuffer() reports an offset past the size as kUnusable,
 * so the subtraction cannot wrap.
 */
bool BufferCapacity(const PoseArrayHcmDescriptor & descriptor, std::size_t & capacity)
{
  int fd = -1;
  const HcmBufferAccess access = InspectHcmBuffer(descriptor, fd);
  if (access != HcmBufferAccess::kLocalDmaBuf && access != HcmBufferAccess::kHostMemory)
    return false;
  capacity = (descriptor.size - descriptor.offset) / sizeof(QRB::Memory::Pose_t);
  return true;
}

}  // namespace

template <>
ConversionResult ConvertContractToHcm<PoseArrayHcmContract, PoseArrayHcmDescriptor>(
    const PoseArrayHcmContract & poses,
    const DmaBufView & /*payload*/,
    PoseArrayHcmDescriptor & descriptor)
{
  // Everything that can fail is checked before the buffer is written, so a rejected message
  // leaves the host's descriptor and buffer untouched.
  ConversionResult validation = ValidatePoseArray(poses);
  if (!validation)
    return validation;
  if (poses.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN) {
    return ConversionResult::Failure("frame_id is longer than the HCM PoseArray descriptor holds");
  }
  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(poses.header().stamp(), timestampNs);
  if (!time)
    return time;

  const std::size_t count = static_cast<std::size_t>(poses.poses_size());
  std::size_t capacity = 0;
  if (count > 0 && (!BufferCapacity(descriptor, capacity) || count > capacity)) {
    return ConversionResult::Failure(
        "HCM PoseArray descriptor buffer is missing or holds fewer than " + std::to_string(count) +
        " poses");
  }

  // memcpy, since the host's offset need not align the array for direct access.
  auto * bytes = static_cast<std::uint8_t *>(count > 0 ? descriptor.GetDataPtr() : nullptr);
  for (std::size_t i = 0; i < count; ++i) {
    QRB::Memory::Pose_t out{};
    PoseToHcm(poses.poses(static_cast<int>(i)), out);
    std::memcpy(bytes + i * sizeof(out), &out, sizeof(out));
  }

  (void)AssignHcmString(poses.header().frame_id(), descriptor.frameId);  // fits, checked above
  descriptor.timestampNs = timestampNs;
  descriptor.poseCount = static_cast<std::uint32_t>(count);
  descriptor.validSize = PoseArrayHcmDescriptor::CalcBufferSize(descriptor.poseCount);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<PoseArrayHcmContract, PoseArrayHcmDescriptor>(
    const PoseArrayHcmDescriptor & descriptor,
    PoseArrayHcmContract & poses)
{
  poses.Clear();
  const std::size_t count = descriptor.poseCount;
  // poseCount is the host's: bound it before it sizes a copy.
  if (count > contract_limits::kMaxPoseCount)
    return ConversionResult::Failure("HCM PoseArray poseCount exceeds the contract limit");
  std::size_t capacity = 0;
  if (count > 0 && (!BufferCapacity(descriptor, capacity) || count > capacity)) {
    return ConversionResult::Failure("HCM PoseArray poseCount exceeds the descriptor's buffer");
  }

  std::vector<QRB::Memory::Pose_t> hcmPoses(count);
  if (count > 0)
    std::memcpy(hcmPoses.data(), descriptor.GetDataPtr(), count * sizeof(QRB::Memory::Pose_t));

  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *poses.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  poses.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  poses.mutable_poses()->Reserve(static_cast<int>(count));
  for (const QRB::Memory::Pose_t & pose : hcmPoses)
    PoseToContract(pose, *poses.add_poses());

  // A non-finite value produced on the HCM side must not be published to ROS.
  return ValidatePoseArray(poses);
}

}  // namespace qrb::adapter
