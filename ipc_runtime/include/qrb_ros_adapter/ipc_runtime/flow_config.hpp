// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_CONFIG_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_CONFIG_HPP

#include <cstdint>
#include <string>

#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"

namespace qrb::adapter
{

/**
 * @brief Immutable configuration of one Flow.
 *
 * One Flow instance means one Contract, one ROS message type, one Topic,
 * one Direction and one IPC Session.
 *
 * `payloadClass` is absent on purpose -- it is fixed by the Contract Descriptor
 * and must never be user-configurable.
 */
struct FlowConfig
{
  std::string flowId;
  std::string contractTypeName;
  Direction direction{Direction::kUnspecified};
  std::string rosTopic;
  QosProfile qosProfile;
  /** End-to-end HCM initialization deadline for register, connect and handshake. */
  std::uint32_t startupTimeoutMs{10000};
  /**
   * Longest one processed frame waits for the ROS IPC Bridge's response:
   * the publish round trip for HCM_TO_ROS, the fetch round trip for
   * ROS_TO_HCM. A fetch never waits for a ROS sample; without a fresh one
   * the bridge answers NO_DATA at once.
   */
  std::uint32_t timeoutMs{1000};
};

/** Bootstrap endpoint shared by the ROS Bridge Manager and the HCM IPC Adapter. */
constexpr const char * kDefaultBootstrapSocketPath = "/run/qrb_ros_adapter/bootstrap.sock";

/** @brief Bounds on `timeoutMs`, validated at startup. */
constexpr std::uint32_t kMinTimeoutMs = 1;
constexpr std::uint32_t kMaxTimeoutMs = 60000;

/** @brief Bounds on the end-to-end startup deadline. */
constexpr std::uint32_t kMinStartupTimeoutMs = 1;
constexpr std::uint32_t kMaxStartupTimeoutMs = 60000;

/**
 * @brief Validate a FlowConfig and cross-check it against the
 * immutable Contract Descriptor selected by `contractTypeName`.
 *
 * This is the whole of a Flow's startup gate: configuration parsing and
 * Descriptor lookup happen at creation, and nothing about the Flow changes
 * afterwards. It must run before any ROS entity or socket exists, so
 * that an unsupported combination fails the Flow rather than half-
 * starting it.
 *
 * @param descriptor The Flow's fixed Descriptor. Its type name must match
 *        `config.contractTypeName` -- a mismatch means the instance was pointed at
 *        a Contract its generated binding does not implement.
 * @param diagnostic Appended with a human-readable reason on failure.
 * @return kOk, kInvalidArgument (a malformed or missing field), or
 *         kUnsupportedContract.
 */
Status ValidateFlowConfig(const FlowConfig & config,
    const ContractDescriptor & descriptor,
    std::string & diagnostic);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_CONFIG_HPP
