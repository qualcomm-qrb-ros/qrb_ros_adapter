// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_JOINT_STATE_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_JOINT_STATE_HCM_CONVERTER_HPP

#include "QRB/Memory/Sensor/JointStateBufferDescriptor.hpp"
#include "qrb/contract/sensor/v1/joint_state.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using JointStateHcmContract = ::qrb::contract::sensor::v1::JointState;
using JointStateHcmDescriptor = QRB::Memory::JointStateBufferDescriptor_t;

/**
 * Joint records live in aligned, constructed host storage at pBuf + offset;
 * the Contract travels inline and needs no external IPC DMA-BUF payload.
 * The flow client brackets local DMA-BUF access with CPU sync, as for TFMessage.
 * HCM carries all three measurements: missing arrays are rejected, not zero-filled.
 * timestampNs uses the same clock domain as header.stamp, without clock conversion.
 */
template <>
ConversionResult ConvertContractToHcm<JointStateHcmContract, JointStateHcmDescriptor>(
    const JointStateHcmContract & source,
    const DmaBufView & payload,
    JointStateHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<JointStateHcmContract, JointStateHcmDescriptor>(
    const JointStateHcmDescriptor & source,
    JointStateHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_JOINT_STATE_HCM_CONVERTER_HPP
