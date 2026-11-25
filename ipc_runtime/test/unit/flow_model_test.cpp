// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <string>

#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"
#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"

using namespace qrb::adapter;

namespace
{

FlowConfig MakeValidConfig()
{
  FlowConfig config;
  config.flowId = "flow_a";
  config.contractTypeName = std::string(kPoseWithCovarianceStampedContractType);
  config.direction = Direction::kRosToHcm;
  config.rosTopic = "/robot/pose";
  return config;
}

const ContractDescriptor & PoseDescriptor()
{
  return *FindContractDescriptor(kPoseWithCovarianceStampedContractType);
}

}  // namespace

// One validator guards both ends of a Flow, and control-plane enums
// survive the wire without an unknown value ever passing as success or a valid
// setting.
TEST(FlowConfigValidation, RejectsOutOfRangeFieldsAndUnknownWireValues)
{
  std::string diagnostic;
  EXPECT_EQ(ValidateFlowConfig(MakeValidConfig(), PoseDescriptor(), diagnostic), Status::kOk)
      << diagnostic;

  FlowConfig limits = MakeValidConfig();
  limits.timeoutMs = kMaxTimeoutMs;
  limits.startupTimeoutMs = kMaxStartupTimeoutMs;
  EXPECT_EQ(ValidateFlowConfig(limits, PoseDescriptor(), diagnostic), Status::kOk) << diagnostic;

  const struct
  {
    const char * name;
    void (*mutate)(FlowConfig &);
    Status expected;
  } cases[] = {
      {"EmptyFlowId", [](FlowConfig & c) { c.flowId.clear(); }, Status::kInvalidArgument},
      {"OtherContract",
          [](FlowConfig & c) { c.contractTypeName = std::string(kPointCloudContractType); },
          Status::kInvalidArgument},
      {"TopicWithSpace", [](FlowConfig & c) { c.rosTopic = "/a b"; }, Status::kInvalidArgument},
      {"TimeoutBelowMin", [](FlowConfig & c) { c.timeoutMs = kMinTimeoutMs - 1; },
          Status::kInvalidArgument},
      {"UnknownQosReliability",
          [](FlowConfig & c) { c.qosProfile.reliability = static_cast<QosReliability>(2); },
          Status::kInvalidArgument},
      {"UnknownQosDurability",
          [](FlowConfig & c) { c.qosProfile.durability = static_cast<QosDurability>(2); },
          Status::kInvalidArgument},
      {"QosDepthZero", [](FlowConfig & c) { c.qosProfile.depth = 0; }, Status::kInvalidArgument},
      {"QosDepthAboveMax", [](FlowConfig & c) { c.qosProfile.depth = kMaxQosDepth + 1; },
          Status::kInvalidArgument},
      {"UnspecifiedDirection", [](FlowConfig & c) { c.direction = Direction::kUnspecified; },
          Status::kUnsupportedContract},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    FlowConfig config = MakeValidConfig();
    c.mutate(config);
    diagnostic.clear();
    EXPECT_EQ(ValidateFlowConfig(config, PoseDescriptor(), diagnostic), c.expected);
    EXPECT_FALSE(diagnostic.empty());
  }

  const Status all[] = {Status::kOk, Status::kNoData, Status::kInvalidArgument,
      Status::kUnsupportedContract, Status::kProtocolError, Status::kConversionError,
      Status::kResourceError, Status::kInternalError, Status::kVersionMismatch, Status::kTimeout,
      Status::kBadState};
  for (Status status : all) {
    const ipc_proto::StatusCode code = ToProtoStatusCode(status);
    EXPECT_NE(code, ipc_proto::STATUS_CODE_UNSPECIFIED) << ToString(status);
    const Status back = FromProtoStatusCode(code);
    EXPECT_EQ(IsOk(back), IsOk(status)) << ToString(status);
    EXPECT_EQ(IsFatalForFlow(back), IsFatalForFlow(status)) << ToString(status);
    EXPECT_EQ(IsFrameError(back), IsFrameError(status)) << ToString(status);
    // Every Status is exactly one of: success or no data, a frame error, fatal.
    const int kinds = (status == Status::kOk || status == Status::kNoData) + IsFrameError(status) +
                      IsFatalForFlow(status);
    EXPECT_EQ(kinds, 1) << ToString(status);
  }
  // kConversionError and kInvalidArgument are frame errors: they drop only the affected frame.
  EXPECT_TRUE(IsFrameError(Status::kConversionError));
  EXPECT_TRUE(IsFrameError(Status::kInvalidArgument));
  EXPECT_TRUE(IsFatalForFlow(Status::kProtocolError));
  EXPECT_TRUE(IsFatalForFlow(Status::kResourceError));
  EXPECT_EQ(FromProtoStatusCode(static_cast<ipc_proto::StatusCode>(99)), Status::kProtocolError);

  // Unknown wire values must fail validation rather than silently become defaults.
  FlowConfig config = MakeValidConfig();
  config.direction = FromProtoDirection(static_cast<ipc_proto::Direction>(99));
  EXPECT_NE(ValidateFlowConfig(config, PoseDescriptor(), diagnostic), Status::kOk);
  ipc_proto::QosProfile wireQos;
  ToProtoQosProfile(
      QosProfile{QosReliability::kBestEffort, QosDurability::kTransientLocal, 3}, wireQos);
  config = MakeValidConfig();
  config.qosProfile = FromProtoQosProfile(wireQos);
  EXPECT_EQ(config.qosProfile.reliability, QosReliability::kBestEffort);
  EXPECT_EQ(config.qosProfile.durability, QosDurability::kTransientLocal);
  EXPECT_EQ(config.qosProfile.depth, 3u);
  wireQos.set_reliability(static_cast<ipc_proto::QosReliability>(99));
  config.qosProfile = FromProtoQosProfile(wireQos);
  EXPECT_EQ(ValidateFlowConfig(config, PoseDescriptor(), diagnostic), Status::kInvalidArgument);
  wireQos = ipc_proto::QosProfile{};
  wireQos.set_durability(static_cast<ipc_proto::QosDurability>(99));
  wireQos.set_depth(10);
  config.qosProfile = FromProtoQosProfile(wireQos);
  EXPECT_EQ(ValidateFlowConfig(config, PoseDescriptor(), diagnostic), Status::kInvalidArgument);
}
