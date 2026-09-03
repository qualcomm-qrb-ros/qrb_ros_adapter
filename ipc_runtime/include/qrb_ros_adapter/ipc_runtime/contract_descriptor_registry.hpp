// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_DESCRIPTOR_REGISTRY_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_DESCRIPTOR_REGISTRY_HPP

#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"

#include <cstddef>
#include <iterator>
#include <string_view>

namespace qrb::adapter
{

#define QRB_CONTRACT_MAPPING( Id, TypeName, Payload, ... ) \
    inline constexpr std::string_view k##Id##ContractType = TypeName;
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING

inline constexpr ContractDescriptor kContractDescriptors[] = {
#define QRB_CONTRACT_MAPPING( Id, TypeName, Payload, ... ) \
    { TypeName, PayloadClass::k##Payload },
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
};

inline constexpr std::size_t ContractDescriptorCount()
{
    return std::size( kContractDescriptors );
}

inline constexpr const ContractDescriptor *ContractDescriptorAt( std::size_t index )
{
    return index < std::size( kContractDescriptors ) ? &kContractDescriptors[index] : nullptr;
}

inline constexpr const ContractDescriptor *FindContractDescriptor( std::string_view typeName )
{
    for ( const ContractDescriptor &descriptor : kContractDescriptors )
    {
        if ( descriptor.typeName == typeName ) return &descriptor;
    }
    return nullptr;
}

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_DESCRIPTOR_REGISTRY_HPP
