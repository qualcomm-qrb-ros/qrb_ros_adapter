// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_STAMPED_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_STAMPED_HCM_CONVERTER_HPP

#include "QRB/Memory/Geometry/PoseDescriptor.hpp"
#include "qrb/contract/geometry/v1/pose_stamped.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using PoseStampedHcmContract = ::qrb::contract::geometry::v1::PoseStamped;
using PoseStampedHcmDescriptor = QRB::Memory::PoseDescriptor_t;

template <>
ConversionResult ConvertContractToHcm<PoseStampedHcmContract, PoseStampedHcmDescriptor>(
    const PoseStampedHcmContract & source,
    const DmaBufView & payload,
    PoseStampedHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<PoseStampedHcmContract, PoseStampedHcmDescriptor>(
    const PoseStampedHcmDescriptor & source,
    PoseStampedHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_STAMPED_HCM_CONVERTER_HPP
