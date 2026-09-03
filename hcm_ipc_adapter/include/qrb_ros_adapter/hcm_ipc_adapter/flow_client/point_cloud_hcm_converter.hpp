// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POINT_CLOUD_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POINT_CLOUD_HCM_CONVERTER_HPP

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

#include "QRB/Memory/Sensor/PointCloudDescriptor.hpp"
#include "qrb/contract/sensor/v1/point_cloud2.pb.h"

namespace qrb::adapter
{

using PointCloudHcmContract = ::qrb::contract::sensor::v1::PointCloud2;
using PointCloudHcmDescriptor = QRB::Memory::PointCloudDescriptor_t;

template <>
ConversionResult ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
    const PointCloudHcmContract &source, const DmaBufView &payload,
    PointCloudHcmDescriptor &target );

template <>
ConversionResult ConvertHcmToContract<PointCloudHcmContract, PointCloudHcmDescriptor>(
    const PointCloudHcmDescriptor &source, PointCloudHcmContract &target );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_POINT_CLOUD_HCM_CONVERTER_HPP
