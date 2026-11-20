// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file ContractFrame.hpp
 * @brief The Contract descriptor a sample hands to the IPC Adapter, and a pool of them.
 *
 * A ContractFrame owns one qrb_contracts descriptor. A DMA-BUF Contract gets a buffer of its own
 * from the system DMA heap, mapped for the CPU; an inline Contract needs none.
 * Samples pass frames to each other as std::shared_ptr<ContractFrame> over the DataBroker: a frame
 * is free again once the only reference left is the pool's.
 */

#ifndef QRB_ROS_ADAPTER_SAMPLE_CONTRACT_FRAME_HPP
#define QRB_ROS_ADAPTER_SAMPLE_CONTRACT_FRAME_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"

#include <atomic>
#include <fcntl.h>
#include <linux/dma-heap.h>
#include <memory>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace QC
{
namespace sample
{

class ContractFrame
{
public:
    ContractFrame() = default;
    ~ContractFrame()
    {
        m_descriptor.reset();
        if ( nullptr != m_data )
        {
            (void) munmap( m_data, m_size );
        }
        if ( m_fd >= 0 )
        {
            (void) close( m_fd );
        }
    }
    ContractFrame( const ContractFrame & ) = delete;
    ContractFrame &operator=( const ContractFrame & ) = delete;

    /**
     * @brief Create the descriptor of Contract @p contractType; a DMA-BUF Contract gets a
     * buffer of @p bufferBytes.
     */
    QCStatus_e Init( const std::string &contractType, uint32_t bufferBytes )
    {
        std::unique_ptr<Memory::QCBufferDescriptorBase_t> created;
        QCStatus_e ret = qrb::adapter::CreateQrbBufferDescriptor(
                qrb::adapter::GetQrbBufferDescriptorType( contractType ), created );
        if ( ( QC_STATUS_OK != ret ) || ( nullptr == created ) )
        {
            return ( QC_STATUS_OK != ret ) ? ret : QC_STATUS_UNSUPPORTED;
        }

        auto *buffer = dynamic_cast<Memory::BufferDescriptor_t *>( created.get() );
        if ( nullptr != buffer )
        {
            if ( 0 == bufferBytes )
            {
                return QC_STATUS_BAD_ARGUMENTS;
            }
            m_fd = AllocateDmaBuf( bufferBytes );
            if ( m_fd < 0 )
            {
                return QC_STATUS_NOMEM;
            }
            void *addr = mmap( nullptr, bufferBytes, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0 );
            if ( MAP_FAILED == addr )
            {
                return QC_STATUS_NOMEM;
            }
            m_data = addr;
            m_size = bufferBytes;
            buffer->pBuf = m_data;
            buffer->size = bufferBytes;
            buffer->offset = 0;
            buffer->validSize = 0;
            buffer->allocatorType = Memory::QC_MEMORY_ALLOCATOR_DMA;
            buffer->dmaHandle = (uint64_t) m_fd;
            buffer->pid = (uint64_t) getpid();
        }
        m_descriptor = std::move( created );

        return QC_STATUS_OK;
    }

    Memory::QCBufferDescriptorBase_t &Descriptor() { return *m_descriptor; }
    bool IsDmaBuf() const { return m_fd >= 0; }

private:
    static int AllocateDmaBuf( uint32_t bytes )
    {
        int heap = open( "/dev/dma_heap/system", O_RDONLY | O_CLOEXEC );
        if ( heap < 0 )
        {
            return -1;
        }
        dma_heap_allocation_data allocation = {};
        allocation.len = bytes;
        allocation.fd_flags = O_RDWR | O_CLOEXEC;
        int result = ioctl( heap, DMA_HEAP_IOCTL_ALLOC, &allocation );
        (void) close( heap );
        return ( 0 == result ) ? (int) allocation.fd : -1;
    }

private:
    std::unique_ptr<Memory::QCBufferDescriptorBase_t> m_descriptor;
    int m_fd = -1;
    void *m_data = nullptr;
    size_t m_size = 0;
};

/** @brief What samples publish and receive over the DataBroker. */
typedef std::shared_ptr<ContractFrame> ContractFrameRef_t;

/**
 * @brief A fixed set of ContractFrames, handed out in turn.
 *
 * Take() returns a frame that nobody downstream still holds, or nullptr when every frame is
 * held. A subscriber of a pooled topic is FIFO with a depth of kMaxSize: the DataBroker then
 * never drops a frame, since no more than the pool can be in flight.
 */
class ContractFramePool
{
public:
    static constexpr uint32_t kMaxSize = 32;

    QCStatus_e Init( const std::string &contractType, uint32_t bufferBytes, uint32_t poolSize )
    {
        QCStatus_e ret = QC_STATUS_OK;

        m_frames.clear();
        m_next = 0;
        for ( uint32_t i = 0; ( i < poolSize ) && ( QC_STATUS_OK == ret ); i++ )
        {
            auto frame = std::make_shared<ContractFrame>();
            ret = frame->Init( contractType, bufferBytes );
            if ( QC_STATUS_OK == ret )
            {
                m_frames.push_back( std::move( frame ) );
            }
        }
        if ( QC_STATUS_OK != ret )
        {
            m_frames.clear();
        }

        return ret;
    }

    ContractFrameRef_t Take()
    {
        for ( size_t n = 0; n < m_frames.size(); n++ )
        {
            ContractFrameRef_t &frame = m_frames[m_next];
            m_next = ( m_next + 1 ) % m_frames.size();
            if ( 1 == frame.use_count() )
            {
                // Pairs with the release of the last downstream reference.
                std::atomic_thread_fence( std::memory_order_acquire );
                return frame;
            }
        }
        return nullptr;
    }

    void Clear() { m_frames.clear(); }

private:
    std::vector<ContractFrameRef_t> m_frames;
    size_t m_next = 0;
};

}   // namespace sample
}   // namespace QC

#endif   // QRB_ROS_ADAPTER_SAMPLE_CONTRACT_FRAME_HPP
