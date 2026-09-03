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
 * one Direction and one IPC Session (HLD §4.3).
 *
 * `payloadClass` is absent on purpose -- it is fixed by the Contract Descriptor
 * and must never be user-configurable (HLD §15).
 */
struct FlowConfig
{
  std::string flowId;
  std::string contractTypeName;
  Direction direction{ Direction::kUnspecified };
  std::string rosTopic;
  QosProfile qosProfile{ QosProfile::kRosDefault };
  /** Zero selects the standard ROS default depth. */
  std::uint32_t qosDepth{ 0 };
  /** End-to-end HCM initialization deadline for register, connect and handshake. */
  std::uint32_t startupTimeoutMs{ 3000 };
  /** Maximum wait for a fresh ROS sample on ROS_TO_HCM source Flows. */
  std::uint32_t sampleTimeoutMs{ 1000 };
  std::uint32_t requestTimeoutMs{ 100 };
};

/** @brief Bounds on `requestTimeoutMs`, validated at startup. */
constexpr std::uint32_t kMinRequestTimeoutMs = 1;
constexpr std::uint32_t kMaxRequestTimeoutMs = 5000;

/** @brief Bounds on the ROS_TO_HCM fresh-sample wait. */
constexpr std::uint32_t kMinSampleTimeoutMs = 1;
constexpr std::uint32_t kMaxSampleTimeoutMs = 60000;

/** @brief Bounds on the HCM-owned end-to-end startup deadline. */
constexpr std::uint32_t kMinStartupTimeoutMs = 1;
constexpr std::uint32_t kMaxStartupTimeoutMs = 60000;

/** @brief Bound dynamic DDS queue allocation while allowing burst-oriented endpoints. */
constexpr std::uint32_t kMaxQosDepth = 1024;

/**
 * @brief Validate a FlowConfig and cross-check it against the
 * immutable Contract Descriptor selected by `contractTypeName`.
 *
 * This is the whole of a Flow's startup gate (HLD §4.2: configuration
 * parsing and Descriptor lookup happen at creation, and nothing about the Flow
 * changes afterwards). It must run before any ROS entity or socket exists, so
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
