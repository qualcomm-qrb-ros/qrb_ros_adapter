// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_contract_registry.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"

#include <array>
#include <iterator>

namespace qrb::adapter
{

namespace
{

constexpr std::size_t kCustomDescriptorCapacity =
    static_cast<std::size_t>( QC::QC_BUFFER_TYPE_CUSTOM_END )
    - static_cast<std::size_t>( QC::QC_BUFFER_TYPE_CUSTOM_BEGIN );

static_assert( QC::QC_BUFFER_TYPE_CUSTOM_BEGIN < QC::QC_BUFFER_TYPE_CUSTOM_END );
static_assert( QC::QC_BUFFER_TYPE_CUSTOM_END <= QC::QC_BUFFER_TYPE_MAX );
static_assert( ContractDescriptorCount() <= kCustomDescriptorCapacity,
    "registered Contracts exceed the GF custom descriptor type capacity" );

std::array<HcmContractBinding, ContractDescriptorCount()> MakeHcmBindings()
{
    std::array<HcmContractBinding, ContractDescriptorCount()> bindings{ {
#define QRB_CONTRACT_MAPPING( Id, TypeName, Payload, ContractT, DescriptorT, ... ) \
    MakeHcmContractBinding<ContractT, DescriptorT>(),
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
    } };

    for ( std::size_t index = 0; index < bindings.size(); ++index )
    {
        bindings[index].descriptorType = static_cast<QC::QCBufferType_e>(
            static_cast<std::size_t>( QC::QC_BUFFER_TYPE_CUSTOM_BEGIN ) + index );
    }
    return bindings;
}

const auto kHcmBindings = MakeHcmBindings();

}   // namespace

std::size_t HcmContractBindingCount()
{
    return std::size( kHcmBindings );
}

const HcmContractBinding *HcmContractBindingAt( std::size_t index )
{
    return index < std::size( kHcmBindings ) ? &kHcmBindings[index] : nullptr;
}

const HcmContractBinding *FindHcmContractBinding( std::string_view contractTypeName )
{
    for ( const HcmContractBinding &binding : kHcmBindings )
    {
        if ( binding.descriptor != nullptr
            && binding.descriptor->typeName == contractTypeName )
        {
            return &binding;
        }
    }
    return nullptr;
}

QC::QCBufferType_e GetQrbBufferDescriptorType( std::string_view contractTypeName )
{
    const HcmContractBinding *binding = FindHcmContractBinding( contractTypeName );
    return binding == nullptr ? QC::QC_BUFFER_TYPE_MAX : binding->descriptorType;
}

QC::QCStatus_e CreateQrbBufferDescriptor( QC::QCBufferType_e type,
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> &outDescriptor )
{
    outDescriptor.reset();
    for ( const HcmContractBinding &binding : kHcmBindings )
    {
        if ( binding.descriptorType == type && binding.createDescriptor != nullptr )
        {
            outDescriptor = binding.createDescriptor( binding.descriptorType );
            return outDescriptor == nullptr ? QC::QC_STATUS_NOMEM : QC::QC_STATUS_OK;
        }
    }
    return QC::QC_STATUS_UNSUPPORTED;
}

}   // namespace qrb::adapter
