// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_LIMITS_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_LIMITS_HPP

#include <cstddef>
#include <cstdint>

namespace qrb::adapter::contract_limits
{

inline constexpr std::size_t kMaxFrameIdLength = 255;
inline constexpr std::size_t kMaxPointFieldCount = 64;
inline constexpr std::size_t kMaxPointFieldNameLength = 63;
inline constexpr std::uint32_t kMaxPointFieldElementCount = 1024;
/**
 * Transforms in one TFMessage: with frame ids at kMaxFrameIdLength, 200 serialize to about
 * 118 KiB, within the IPC payload limit (kMaxPayloadSize, 128 KiB).
 */
inline constexpr std::size_t kMaxTransformCount = 200;
/** @brief Bounded JointState frames stay below the 128 KiB inline IPC payload limit. */
inline constexpr std::size_t kMaxJointCount = 1024;
inline constexpr std::size_t kMaxJointNameLength = 63;

/**
 * Poses in one PoseArray: a pose serializes to at most 69 bytes in the array, so 1000 of them
 * with a frame id at kMaxFrameIdLength take about 68 KiB, within the IPC payload limit
 * (kMaxPayloadSize, 128 KiB).
 */
inline constexpr std::size_t kMaxPoseCount = 1000;

}  // namespace qrb::adapter::contract_limits

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_LIMITS_HPP
