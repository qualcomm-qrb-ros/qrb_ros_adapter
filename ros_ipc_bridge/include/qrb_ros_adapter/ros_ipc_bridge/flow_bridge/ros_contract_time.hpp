// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_TIME_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_TIME_HPP

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>

#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

/**
 * @brief Overflow-safe conversion between a ROS timestamp and the Contract's
 * `timestamp_ns`.
 *
 * HLD §7.2 requires this explicitly ("ROS 时间戳与 timestamp_ns 的无溢出转换"),
 * and it is not a formality in either direction:
 *
 *   - ROS `sec` is *signed*. A negative stamp cannot be represented in an
 *     unsigned nanosecond count, and casting it would silently produce a
 *     timestamp roughly 292 years in the future.
 *   - The reverse conversion divides by 1e9 into a signed 32-bit `sec`, which
 *     saturates just past the year 2038. A Contract timestamp beyond that
 *     cannot be represented in a ROS message at all.
 *
 * Both are reported rather than clamped: a conversion function must not silently modify
 * its input (HLD §6.3).
 */

constexpr std::uint64_t kNanosPerSecond = 1000000000ULL;

/**
 * @brief ROS stamp -> Contract nanoseconds.
 * @return Success, or a diagnostic if `stamp.sec` is negative or the product
 *         overflows.
 */
ConversionResult StampToContractNanos(const builtin_interfaces::msg::Time & stamp,
    std::uint64_t & outNanos);

/**
 * @brief Contract nanoseconds -> ROS stamp.
 * @return Success, or a diagnostic if the seconds part exceeds int32.
 */
ConversionResult ContractNanosToStamp(std::uint64_t nanos,
    builtin_interfaces::msg::Time & outStamp);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_TIME_HPP
