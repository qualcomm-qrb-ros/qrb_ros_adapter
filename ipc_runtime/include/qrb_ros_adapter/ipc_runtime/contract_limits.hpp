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

}   // namespace qrb::adapter::contract_limits

#endif   // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_LIMITS_HPP
