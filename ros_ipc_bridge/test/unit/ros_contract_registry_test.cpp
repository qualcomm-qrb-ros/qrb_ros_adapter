// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/manager/ros_contract_registry.hpp"

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <string_view>

using namespace qrb::adapter;

// Every shared Contract Descriptor has exactly one ROS binding with a
// factory and a unique node name, so adding a Contract cannot silently leave the
// ROS side unimplemented.
TEST(RosContractRegistry, BindsEveryContractDescriptorExactlyOnce)
{
  ASSERT_EQ(RosContractBindingCount(), ContractDescriptorCount());

  std::set<std::string_view> typeNames;
  std::set<std::string_view> nodeNames;
  for (std::size_t i = 0; i < RosContractBindingCount(); ++i) {
    const RosContractBinding * binding = RosContractBindingAt(i);
    ASSERT_NE(binding, nullptr);
    ASSERT_NE(binding->descriptor, nullptr) << "binding " << i;
    EXPECT_NE(binding->createBridge, nullptr) << binding->descriptor->typeName;
    EXPECT_FALSE(binding->nodeName.empty()) << binding->descriptor->typeName;
    EXPECT_TRUE(typeNames.insert(binding->descriptor->typeName).second);
    EXPECT_TRUE(nodeNames.insert(binding->nodeName).second);
    EXPECT_EQ(FindContractDescriptor(binding->descriptor->typeName), binding->descriptor);
    EXPECT_EQ(FindRosContractBinding(binding->descriptor->typeName), binding);
  }
  EXPECT_EQ(RosContractBindingAt(RosContractBindingCount()), nullptr);
  EXPECT_EQ(FindRosContractBinding("qrb.contract.unknown.v1.Message"), nullptr);

  // A binding without a factory is reported, not dereferenced.
  const RosContractBinding empty{};
  std::shared_ptr<RosFlowBridgeBase> bridge;
  std::string diagnostic;
  EXPECT_EQ(CreateFlowBridge(empty, rclcpp::NodeOptions(), nullptr, bridge, diagnostic),
      Status::kUnsupportedContract);
  EXPECT_EQ(bridge, nullptr);
  EXPECT_FALSE(diagnostic.empty());
}
