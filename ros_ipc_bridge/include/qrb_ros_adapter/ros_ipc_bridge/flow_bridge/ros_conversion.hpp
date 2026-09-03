// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONVERSION_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONVERSION_HPP

#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

#include <cstdint>
#include <memory>
#include <utility>

#include <lib_mem_dmabuf/dmabuf.hpp>

namespace qrb::adapter
{

/** @brief Payload selected by a ROS-to-Contract DMA conversion. */
class RosToContractPayload final
{
public:
    RosToContractPayload() = default;
    RosToContractPayload( const RosToContractPayload & ) = delete;
    RosToContractPayload &operator=( const RosToContractPayload & ) = delete;

    ConversionResult Bind( std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer,
        std::uint64_t size, std::uint64_t offset = 0 )
    {
        if ( buffer_ != nullptr )
        {
            return ConversionResult::Failure( "DMA-BUF source payload is already bound" );
        }
        if ( buffer == nullptr || buffer->fd() < 0 || buffer->size() <= 0 )
        {
            return ConversionResult::Failure( "ROS message has no valid DMA-BUF" );
        }
        const std::uint64_t capacity = static_cast<std::uint64_t>( buffer->size() );
        if ( size == 0 )
        {
            return ConversionResult::Failure( "DMA-BUF payload is empty" );
        }
        if ( offset > capacity || size > capacity - offset )
        {
            return ConversionResult::Failure(
                "DMA-BUF payload range exceeds the ROS buffer capacity" );
        }
        buffer_ = std::move( buffer );
        size_ = size;
        offset_ = offset;
        return ConversionResult::Success();
    }

    bool IsBound() const noexcept { return buffer_ != nullptr; }
    const std::shared_ptr<lib_mem_dmabuf::DmaBuffer> &Buffer() const noexcept
    {
        return buffer_;
    }
    std::uint64_t Size() const noexcept { return size_; }
    std::uint64_t Offset() const noexcept { return offset_; }

private:
    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer_;
    std::uint64_t size_{ 0 };
    std::uint64_t offset_{ 0 };
};

/** @brief Destination selected by a Contract-to-ROS DMA conversion. */
class ContractToRosPayload final
{
public:
    explicit ContractToRosPayload( std::uint64_t availableCapacity )
        : availableCapacity_( availableCapacity )
    {
    }
    ContractToRosPayload( const ContractToRosPayload & ) = delete;
    ContractToRosPayload &operator=( const ContractToRosPayload & ) = delete;

    ConversionResult Bind( std::shared_ptr<lib_mem_dmabuf::DmaBuffer> &destination,
        std::uint64_t size )
    {
        if ( destination_ != nullptr )
        {
            return ConversionResult::Failure(
                "DMA-BUF destination payload is already bound" );
        }
        if ( size == 0 )
        {
            return ConversionResult::Failure( "DMA-BUF payload is empty" );
        }
        if ( size > availableCapacity_ )
        {
            return ConversionResult::Failure(
                "Contract payload exceeds the received DMA-BUF range" );
        }
        destination_ = &destination;
        size_ = size;
        return ConversionResult::Success();
    }

    bool IsBound() const noexcept { return destination_ != nullptr; }
    std::uint64_t Size() const noexcept { return size_; }
    std::uint64_t AvailableCapacity() const noexcept { return availableCapacity_; }

    bool Assign( std::shared_ptr<lib_mem_dmabuf::DmaBuffer> buffer )
    {
        if ( destination_ == nullptr || buffer == nullptr ) return false;
        *destination_ = std::move( buffer );
        return true;
    }

private:
    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> *destination_{ nullptr };
    std::uint64_t availableCapacity_{ 0 };
    std::uint64_t size_{ 0 };
};

/**
 * @brief Message-specific ROS/Contract conversion policy.
 *
 * Each registered message pair provides one full specialization containing
 * exactly two static methods, one for each conversion direction. DMA-BUF
 * policies additionally receive the corresponding payload context.
 */
template <typename RosMessageT, typename ContractT>
struct RosConversion;

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONVERSION_HPP
