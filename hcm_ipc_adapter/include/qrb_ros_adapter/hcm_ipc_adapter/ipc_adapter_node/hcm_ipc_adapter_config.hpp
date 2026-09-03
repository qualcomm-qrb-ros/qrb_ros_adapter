// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_CONFIG_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_CONFIG_HPP

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"

#include <cstdint>
#include <string>

namespace qrb::adapter
{

/**
 * @brief Mapping from the GF-local frame descriptor slots to one Flow.
 *
 * GFNodeAdaptor converts global buffer IDs into a dense local
 * QCFrameDescriptorNodeIfs before invoking the QCNode. The IPC Adapter must
 * therefore index the local descriptor, not the graph-global buffer table.
 */
struct HcmFrameBufferLayout
{
    std::uint32_t descriptorLocalSlot{ 0 };
};

/** @brief HCM-only configuration layered on top of the shared FlowConfig. */
struct HcmIpcAdapterConfig
{
    FlowConfig flowConfig;
    std::string bootstrapSocketPath;
    std::string nodeName;
    std::uint32_t nodeId{ 0 };
    HcmFrameBufferLayout buffers;
    /** Enable per-frame clock sampling for latency fields in Monitoring data. */
    bool enableLatencyMonitoring{ false };
};

/** QCNode configuration contract generated from the parser's canonical field lists. */
const std::string &HcmIpcAdapterConfigOptions();

/**
 * @brief Parse the one configured FlowConfig from GF JSON configuration.
 *
 * The ROS IPC Bridge gets its Flow from ROS parameters (HLD §7.5); the HCM IPC Adapter gets
 * the equivalent from the Graph Framework's own config string, which is JSON.
 * Parsing is kept separate from the QCNode interface methods so it can be
 * unit tested independently.
 *
 * Expected shape -- the `"static"` object is the GF static configuration block:
 *
 *   {
 *     "static": {
 *       "flow_id": "pose_to_hcm",
 *       "contract_type_name": "qrb.contract.geometry.v1.PoseWithCovarianceStamped",
 *       "direction": "ros_to_hcm",
 *       "ros_topic": "/localization/pose",
 *       "bootstrap_socket_path": "/run/qrb_ros_adapter/bootstrap.sock",
 *       "startup_timeout_ms": 3000,
 *       "request_timeout_ms": 100,
 *       "sample_timeout_ms": 1000,
 *       "enable_latency_monitoring": false,
 *       "descriptor_local_slot": 0
 *     }
 *   }
 *
 * The Manager assigns the per-Flow data socket. Payload class and ROS endpoint
 * strategy are resolved from the Contract registries, not configured.
 *
 * @param diagnostic Filled in with a human-readable reason on failure.
 * @return kOk, or kInvalidArgument if the JSON is malformed or a required
 *         field is missing, of the wrong type, or has an unrecognized value.
 */
Status ParseFlowConfigJson( const std::string &json, FlowConfig &out, std::string &diagnostic );

/**
 * @brief Parse the complete GF-facing QCNode configuration.
 *
 * Every Contract uses exactly one local slot containing its native
 * qrb_contracts descriptor. DMA payload storage is inherited by that descriptor.
 */
Status ParseHcmIpcAdapterConfigJson(
    const std::string &json, HcmIpcAdapterConfig &out, std::string &diagnostic );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_CONFIG_HPP
