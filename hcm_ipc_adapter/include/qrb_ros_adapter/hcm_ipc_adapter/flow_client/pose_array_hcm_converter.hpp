// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_ARRAY_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_ARRAY_HCM_CONVERTER_HPP

#include "QRB/Memory/Geometry/PoseArrayDescriptor.hpp"
#include "qrb/contract/geometry/v1/pose_array.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using PoseArrayHcmContract = ::qrb::contract::geometry::v1::PoseArray;
using PoseArrayHcmDescriptor = QRB::Memory::PoseArrayDescriptor_t;

/**
 * The descriptor's poses live in the host's buffer (`pBuf` + `offset`), an array of `poseCount`
 * Pose_t. The Contract carries them inline, so the IPC transfer needs no DMA-BUF. The
 * converters bracket their access to a local DMA-BUF with CPU sync themselves.
 *
 * A Contract that does not fit the descriptor (a frame_id longer than FrameId_t, more poses than
 * the buffer holds) is rejected whole and leaves the descriptor and its buffer untouched.
 */
template <>
ConversionResult ConvertContractToHcm<PoseArrayHcmContract, PoseArrayHcmDescriptor>(
    const PoseArrayHcmContract & source,
    const DmaBufView & payload,
    PoseArrayHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<PoseArrayHcmContract, PoseArrayHcmDescriptor>(
    const PoseArrayHcmDescriptor & source,
    PoseArrayHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_ARRAY_HCM_CONVERTER_HPP
