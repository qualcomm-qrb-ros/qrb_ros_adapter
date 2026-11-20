// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_ODOMETRY_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_ODOMETRY_HCM_CONVERTER_HPP

#include "QRB/Memory/Navigation/OdometryDescriptor.hpp"
#include "qrb/contract/nav/v1/odometry.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using OdometryHcmContract = ::qrb::contract::nav::v1::Odometry;
using OdometryHcmDescriptor = QRB::Memory::OdometryDescriptor_t;

template <>
ConversionResult ConvertContractToHcm<OdometryHcmContract, OdometryHcmDescriptor>(
    const OdometryHcmContract & source,
    const DmaBufView & payload,
    OdometryHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<OdometryHcmContract, OdometryHcmDescriptor>(
    const OdometryHcmDescriptor & source,
    OdometryHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_ODOMETRY_HCM_CONVERTER_HPP
