// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERTERS_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERTERS_HPP

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"

#include <cstddef>
#include <memory>
#include <string_view>

#include "QC/Infras/Memory/Ifs/QCBufferDescriptorBase.hpp"
#include <google/protobuf/message_lite.h>

namespace qrb::adapter
{

using HcmDescriptorMatcher = bool ( * )(
    const QC::Memory::QCBufferDescriptorBase_t &descriptor );
using ContractToHcmFunction = ConversionResult ( * )(
    const google::protobuf::MessageLite &source, const DmaBufView &payload,
    QC::Memory::QCBufferDescriptorBase_t &target );
using HcmToContractFunction = ConversionResult ( * )(
    const QC::Memory::QCBufferDescriptorBase_t &source,
    google::protobuf::MessageLite &target );
using HcmDescriptorFactory =
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> ( * )( QC::QCBufferType_e );

/** @brief HCM-side implementation registered for one shared Contract Descriptor. */
struct HcmContractBinding
{
    const ContractDescriptor *descriptor{ nullptr };
    HcmDescriptorMatcher acceptsDescriptor{ nullptr };
    ContractToHcmFunction contractToHcm{ nullptr };
    HcmToContractFunction hcmToContract{ nullptr };
    QC::QCBufferType_e descriptorType{ QC::QC_BUFFER_TYPE_MAX };
    HcmDescriptorFactory createDescriptor{ nullptr };
};

template <typename DescriptorT>
bool AcceptsHcmDescriptor( const QC::Memory::QCBufferDescriptorBase_t &descriptor )
{
    return dynamic_cast<const DescriptorT *>( &descriptor ) != nullptr;
}

template <typename ContractT, typename DescriptorT>
ConversionResult InvokeContractToHcm( const google::protobuf::MessageLite &source,
    const DmaBufView &payload, QC::Memory::QCBufferDescriptorBase_t &target )
{
    const auto *contract = dynamic_cast<const ContractT *>( &source );
    if ( contract == nullptr )
    {
        return ConversionResult::Failure(
            "HCM conversion received an unexpected Contract source type" );
    }
    auto *descriptor = dynamic_cast<DescriptorT *>( &target );
    if ( descriptor == nullptr )
    {
        return ConversionResult::Failure(
            "HCM conversion received an unexpected descriptor target type" );
    }
    return ConvertContractToHcm<ContractT, DescriptorT>( *contract, payload, *descriptor );
}

template <typename ContractT, typename DescriptorT>
ConversionResult InvokeHcmToContract( const QC::Memory::QCBufferDescriptorBase_t &source,
    google::protobuf::MessageLite &target )
{
    const auto *descriptor = dynamic_cast<const DescriptorT *>( &source );
    if ( descriptor == nullptr )
    {
        return ConversionResult::Failure(
            "HCM conversion received an unexpected descriptor source type" );
    }
    auto *contract = dynamic_cast<ContractT *>( &target );
    if ( contract == nullptr )
    {
        return ConversionResult::Failure(
            "HCM conversion received an unexpected Contract target type" );
    }
    return ConvertHcmToContract<ContractT, DescriptorT>( *descriptor, *contract );
}

template <typename DescriptorT>
std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> CreateHcmDescriptor(
    QC::QCBufferType_e descriptorType )
{
    auto descriptor = std::make_unique<DescriptorT>();
    descriptor->type = descriptorType;
    return descriptor;
}

template <typename ContractT, typename DescriptorT>
HcmContractBinding MakeHcmContractBinding()
{
    return { FindContractDescriptor( ContractT::descriptor()->full_name() ),
        &AcceptsHcmDescriptor<DescriptorT>, &InvokeContractToHcm<ContractT, DescriptorT>,
        &InvokeHcmToContract<ContractT, DescriptorT>, QC::QC_BUFFER_TYPE_MAX,
        &CreateHcmDescriptor<DescriptorT> };
}

std::size_t HcmContractBindingCount();
const HcmContractBinding *HcmContractBindingAt( std::size_t index );
const HcmContractBinding *FindHcmContractBinding( std::string_view contractTypeName );

/** @brief Return the automatically assigned GF buffer type for a Contract. */
QC::QCBufferType_e GetQrbBufferDescriptorType( std::string_view contractTypeName );

/** @brief GF descriptor factory covering every registered qrb_contracts type. */
QC::QCStatus_e CreateQrbBufferDescriptor( QC::QCBufferType_e type,
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> &outDescriptor );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERTERS_HPP
