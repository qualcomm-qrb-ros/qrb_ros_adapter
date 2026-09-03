// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_time.hpp"

#include <limits>

namespace qrb::adapter
{

ConversionResult StampToContractNanos(const builtin_interfaces::msg::Time & stamp,
    std::uint64_t & outNanos)
{
  if (stamp.sec < 0) {
    return ConversionResult::Failure("ROS timestamp seconds must not be negative");
  }

  const std::uint64_t secondsPart = static_cast<std::uint64_t>(stamp.sec);
  if (secondsPart > std::numeric_limits<std::uint64_t>::max() / kNanosPerSecond) {
    return ConversionResult::Failure("ROS timestamp nanoseconds overflow uint64");
  }
  const std::uint64_t seconds = secondsPart * kNanosPerSecond;

  if (stamp.nanosec > std::numeric_limits<std::uint64_t>::max() - seconds) {
    return ConversionResult::Failure("ROS timestamp nanoseconds overflow uint64");
  }

  outNanos = seconds + stamp.nanosec;
  return ConversionResult::Success();
}

ConversionResult ContractNanosToStamp(std::uint64_t nanos, builtin_interfaces::msg::Time & outStamp)
{
  const std::uint64_t seconds = nanos / kNanosPerSecond;
  if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    return ConversionResult::Failure("Contract timestamp seconds exceed ROS int32 range");
  }

  outStamp.sec = static_cast<std::int32_t>(seconds);
  outStamp.nanosec = static_cast<std::uint32_t>(nanos % kNanosPerSecond);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
