// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/latest_sample.hpp"

#include <gtest/gtest.h>

using namespace qrb::adapter;

// Only the newest sample is kept and each one is handed out once;
// Take never waits, so a fetch without a fresh sample answers NO_DATA at once.
TEST(LatestSample, HandsOutTheNewestSampleOnce)
{
  LatestSample<int> latest;
  int value = 0;
  EXPECT_FALSE(latest.HasSample());
  EXPECT_FALSE(latest.Take(value));

  latest.Store(1);
  latest.Store(2);
  EXPECT_TRUE(latest.HasSample());
  ASSERT_TRUE(latest.Take(value));
  EXPECT_EQ(value, 2);
  EXPECT_FALSE(latest.HasSample());
  EXPECT_FALSE(latest.Take(value));

  latest.Store(3);
  latest.Clear();
  EXPECT_FALSE(latest.Take(value));

  latest.Store(4);
  ASSERT_TRUE(latest.Take(value));
  EXPECT_EQ(value, 4);
}
