// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_TF_MESSAGE_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_TF_MESSAGE_HCM_CONVERTER_HPP

#include "QRB/Memory/Geometry/TFMessageDescriptor.hpp"
#include "qrb/contract/tf2/v1/tf_message.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using TFMessageHcmContract = ::qrb::contract::tf2::v1::TFMessage;
using TFMessageHcmDescriptor = QRB::Memory::TFMessageDescriptor_t;

/**
 * The descriptor's transforms live in host storage (`pBuf` + `offset`), an array of
 * `transformCount` TransformStamped_t. The Contract carries them Inline, so the IPC
 * transfer needs no DMA-BUF. The flow client does the CPU sync around the converter
 * call, so the converter does not sync.
 */
template <>
ConversionResult ConvertContractToHcm<TFMessageHcmContract, TFMessageHcmDescriptor>(
    const TFMessageHcmContract & source,
    const DmaBufView & payload,
    TFMessageHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<TFMessageHcmContract, TFMessageHcmDescriptor>(
    const TFMessageHcmDescriptor & source,
    TFMessageHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_TF_MESSAGE_HCM_CONVERTER_HPP
