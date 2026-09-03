// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_HCM_CONVERTER_HPP

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

#include "QRB/Memory/Geometry/PoseDescriptor.hpp"
#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"

namespace qrb::adapter
{

using PoseHcmContract = ::qrb::contract::geometry::v1::PoseWithCovarianceStamped;
using PoseHcmDescriptor = QRB::Memory::PoseDescriptor_t;

template <>
ConversionResult ConvertContractToHcm<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmContract &source, const DmaBufView &payload, PoseHcmDescriptor &target );

template <>
ConversionResult ConvertHcmToContract<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmDescriptor &source, PoseHcmContract &target );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POSE_HCM_CONVERTER_HPP
