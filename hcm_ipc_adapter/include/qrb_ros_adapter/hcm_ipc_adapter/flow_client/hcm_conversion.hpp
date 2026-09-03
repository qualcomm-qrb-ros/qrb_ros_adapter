// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP

#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

#include <cstddef>
namespace qrb::adapter
{

/** @brief Read-only mapping of a received DMA-BUF. */
struct DmaBufView
{
    const void *data{ nullptr };
    std::size_t size{ 0 };
};

/** @brief Convert a Contract and optional external payload to an HCM descriptor. */
template <typename ContractT, typename DescriptorT>
ConversionResult ConvertContractToHcm(
    const ContractT &source, const DmaBufView &payload, DescriptorT &target ) = delete;

/** @brief Convert an HCM descriptor to its transport Contract. */
template <typename ContractT, typename DescriptorT>
ConversionResult ConvertHcmToContract(
    const DescriptorT &source, ContractT &target ) = delete;

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP
