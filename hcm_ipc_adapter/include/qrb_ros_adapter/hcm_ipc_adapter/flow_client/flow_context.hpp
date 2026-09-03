// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_FLOW_CONTEXT_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_FLOW_CONTEXT_HPP

#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_registration.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"

#include <cstdint>
#include <string>

namespace qrb::adapter
{

/**
 * @brief Runtime data of one Flow: validated configuration, Manager-issued
 * registration identity, Contract Descriptor, and counters.
 *
 * QCNode owns the only lifecycle state machine. This context stores resources
 * and does not maintain a second Client state.
 */
class FlowContext
{
public:
    FlowContext() = default;

    /**
     * @brief Adopt validated configuration and Descriptor.
     */
    Status Configure( const FlowConfig &config, const ContractDescriptor &descriptor );

    /** Assign the Manager-selected endpoint and registration identity. */
    Status AssignRegistration( const FlowRegistration &registration );

    /** @brief Release all resources and counters. */
    void Reset();

    const FlowConfig &Config() const { return config_; }
    const FlowRegistration &Registration() const { return registration_; }
    const ContractDescriptor *Descriptor() const { return descriptor_; }

    /** @brief Next request ID for an outbound request frame. */
    std::uint64_t NextRequestId() { return ++lastRequestId_; }

    // Counters used by monitoring and resource-lifecycle tests.
    std::uint64_t framesIn{ 0 };
    std::uint64_t framesOut{ 0 };
    std::uint64_t noDataResponses{ 0 };
    std::uint64_t conversionErrors{ 0 };

private:
    FlowConfig config_{};
    FlowRegistration registration_{};
    const ContractDescriptor *descriptor_{ nullptr };
    std::uint64_t lastRequestId_{ 0 };
};

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_FLOW_CONTEXT_HPP
