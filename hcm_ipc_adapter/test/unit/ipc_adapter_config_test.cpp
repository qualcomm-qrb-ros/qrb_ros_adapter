// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"

using namespace qrb::adapter;

namespace
{

/** The required fields only. */
nlohmann::json MakeConfig()
{
  nlohmann::json root;
  root["static"] = {
      {"name", "pose_adapter"},
      {"contract_type", std::string(kPoseWithCovarianceStampedContractType)},
      {"direction", kDirectionCodeRosToHcm},
      {"ros_topic", "/localization/pose"},
  };
  return root;
}

}  // namespace

// The QCNode static block is the HCM side's only Flow configuration; a bad
// one fails with a reason and is never half-applied.
TEST(HcmIpcAdapterConfig, ParsesTheStaticBlockAndRejectsInvalidOnesUntouched)
{
  HcmIpcAdapterConfig config;
  std::string diagnostic;
  ASSERT_EQ(ParseHcmIpcAdapterConfigJson(MakeConfig().dump(), config, diagnostic), Status::kOk)
      << diagnostic;

  // The QCNode name is the Flow identity; omitted optional fields and the
  // fixed timeouts take the same defaults the ROS side uses.
  const FlowConfig defaults;
  EXPECT_EQ(config.nodeName, "pose_adapter");
  EXPECT_EQ(config.nodeId, 0u);
  EXPECT_EQ(config.flowConfig.flowId, "pose_adapter");
  EXPECT_EQ(config.flowConfig.direction, Direction::kRosToHcm);
  EXPECT_EQ(config.flowConfig.rosTopic, "/localization/pose");
  EXPECT_EQ(config.flowConfig.timeoutMs, defaults.timeoutMs);
  EXPECT_EQ(config.flowConfig.startupTimeoutMs, defaults.startupTimeoutMs);
  EXPECT_TRUE(config.flowConfig.qosProfile == QosProfile{});
  EXPECT_EQ(config.globalBufferId, kHcmDefaultGlobalBufferId);

  nlohmann::json full = MakeConfig();
  full["static"]["direction"] = kDirectionCodeHcmToRos;
  full["static"]["id"] = 7;
  full["static"]["qos_profile"] = {
      {"reliability", "best_effort"}, {"durability", "transient_local"}, {"depth", 1}};
  full["static"]["globalBufferIdMap"] = {{{"name", "Input"}, {"id", 5}}};
  ASSERT_EQ(ParseHcmIpcAdapterConfigJson(full.dump(), config, diagnostic), Status::kOk)
      << diagnostic;
  EXPECT_EQ(config.nodeId, 7u);
  EXPECT_EQ(config.globalBufferId, 5u);
  EXPECT_EQ(config.flowConfig.direction, Direction::kHcmToRos);
  EXPECT_EQ(config.flowConfig.qosProfile.reliability, QosReliability::kBestEffort);
  EXPECT_EQ(config.flowConfig.qosProfile.durability, QosDurability::kTransientLocal);
  EXPECT_EQ(config.flowConfig.qosProfile.depth, 1u);

  // Each QoS member is optional on its own.
  nlohmann::json partial = MakeConfig();
  partial["static"]["qos_profile"] = {{"depth", kMaxQosDepth}};
  ASSERT_EQ(ParseHcmIpcAdapterConfigJson(partial.dump(), config, diagnostic), Status::kOk)
      << diagnostic;
  EXPECT_EQ(config.flowConfig.qosProfile.reliability, QosReliability::kReliable);
  EXPECT_EQ(config.flowConfig.qosProfile.durability, QosDurability::kVolatile);
  EXPECT_EQ(config.flowConfig.qosProfile.depth, kMaxQosDepth);

  // The advertised option schema lists every required field.
  const nlohmann::json options = nlohmann::json::parse(HcmIpcAdapterConfigOptions());
  const nlohmann::json required = MakeConfig();
  EXPECT_EQ(options["required"].size(), required["static"].size());
  for (const auto & item : required["static"].items()) {
    const std::string key = "static." + item.key();
    EXPECT_NE(std::find(options["required"].begin(), options["required"].end(), key),
        options["required"].end())
        << key;
  }

  const auto with = [](const char * key, nlohmann::json value) {
    nlohmann::json root = MakeConfig();
    root["static"][key] = std::move(value);
    return root.dump();
  };
  const auto withQos = [](const char * key, nlohmann::json value) {
    nlohmann::json root = MakeConfig();
    root["static"]["qos_profile"][key] = std::move(value);
    return root.dump();
  };
  nlohmann::json withoutTopic = MakeConfig();
  withoutTopic["static"].erase("ros_topic");

  const struct
  {
    const char * name;
    std::string json;
    Status expected;
  } cases[] = {
      {"MalformedJson", "{\"static\":", Status::kInvalidArgument},
      {"MissingRosTopic", withoutTopic.dump(), Status::kInvalidArgument},
      // direction is 0 or 1; the old string names are rejected.
      {"UnknownDirection", with("direction", 2), Status::kInvalidArgument},
      {"DirectionString", with("direction", "ros_to_hcm"), Status::kInvalidArgument},
      {"IdOutOfRange", with("id", 256), Status::kInvalidArgument},
      // qos_profile is an object of raw ROS policies, not a preset name.
      {"QosProfileNotObject", with("qos_profile", "sensor_data"), Status::kInvalidArgument},
      {"UnknownQosReliability", withQos("reliability", "system_default"), Status::kInvalidArgument},
      {"UnknownQosDurability", withQos("durability", "persistent"), Status::kInvalidArgument},
      {"QosDepthZero", withQos("depth", 0), Status::kInvalidArgument},
      {"QosDepthTooLarge", withQos("depth", kMaxQosDepth + 1), Status::kInvalidArgument},
      {"UnsupportedQosPolicy", withQos("history", "keep_all"), Status::kInvalidArgument},
      // The node has one buffer, so the QCNode globalBufferIdMap has one full entry.
      {"BufferMapNotArray", with("globalBufferIdMap", {{"name", "Output"}, {"id", 1}}),
          Status::kInvalidArgument},
      {"BufferMapEmpty", with("globalBufferIdMap", nlohmann::json::array()),
          Status::kInvalidArgument},
      {"BufferMapTwoEntries",
          with("globalBufferIdMap",
              {{{"name", "Input"}, {"id", 0}}, {{"name", "Output"}, {"id", 1}}}),
          Status::kInvalidArgument},
      {"BufferMapNoId", with("globalBufferIdMap", {{{"name", "Output"}}}),
          Status::kInvalidArgument},
      {"BufferMapUnsetId",
          with("globalBufferIdMap",
              {{{"name", "Output"}, {"id", std::numeric_limits<std::uint32_t>::max()}}}),
          Status::kInvalidArgument},
      {"BufferMapEmptyName", with("globalBufferIdMap", {{{"name", ""}, {"id", 1}}}),
          Status::kInvalidArgument},
      {"BufferMapUnknownMember",
          with("globalBufferIdMap", {{{"name", "Output"}, {"id", 1}, {"type", "x"}}}),
          Status::kInvalidArgument},
      // Misspelled, fixed and Manager-owned fields are all unknown.
      {"UnknownField", with("flow_id", "pose_to_hcm"), Status::kInvalidArgument},
      {"FixedTimeout", with("timeout_ms", 250), Status::kInvalidArgument},
      {"FixedBootstrapPath", with("bootstrap_socket_path", "/tmp/qrb/bootstrap.sock"),
          Status::kInvalidArgument},
      {"FixedQosDepth", with("qos_depth", 4), Status::kInvalidArgument},
      {"FixedDescriptorSlot", with("descriptor_local_slot", 0), Status::kInvalidArgument},
      {"UnknownContract", with("contract_type", "qrb.contract.unknown.v1.Missing"),
          Status::kUnsupportedContract},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    HcmIpcAdapterConfig untouched;
    untouched.nodeName = "untouched";
    diagnostic.clear();
    EXPECT_EQ(ParseHcmIpcAdapterConfigJson(c.json, untouched, diagnostic), c.expected);
    EXPECT_FALSE(diagnostic.empty());
    EXPECT_EQ(untouched.nodeName, "untouched");
  }
}
