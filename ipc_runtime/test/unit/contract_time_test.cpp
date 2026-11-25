// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

using namespace qrb::adapter;

using ContractTime = ::qrb::contract::common::v1::Time;

// The HCM side keeps a uint64 nanosecond stamp; conversion to the
// Contract sec/nanosec pair is exact and rejects what the other side cannot hold.
TEST(ContractTime, ConvertsExactlyAndRejectsOutOfRangeStamps)
{
  ContractTime time;
  ASSERT_TRUE(NanosToContractTime(100 * kNanosPerSecond + 500, time));
  EXPECT_EQ(time.sec(), 100);
  EXPECT_EQ(time.nanosec(), 500u);
  std::uint64_t nanos = 0;
  ASSERT_TRUE(ContractTimeToNanos(time, nanos));
  EXPECT_EQ(nanos, 100 * kNanosPerSecond + 500);

  const std::uint64_t maxNanos =
      static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) * kNanosPerSecond +
      (kNanosPerSecond - 1);
  ASSERT_TRUE(NanosToContractTime(maxNanos, time));
  EXPECT_EQ(time.sec(), std::numeric_limits<std::int32_t>::max());
  EXPECT_FALSE(NanosToContractTime(maxNanos + 1, time));

  ContractTime negative;
  negative.set_sec(-1);
  nanos = 7;
  EXPECT_FALSE(ContractTimeToNanos(negative, nanos));
  EXPECT_EQ(nanos, 7u);

  ContractTime overflow;
  overflow.set_nanosec(static_cast<std::uint32_t>(kNanosPerSecond));
  EXPECT_FALSE(ContractTimeToNanos(overflow, nanos));
}
