// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_DMA_BUFFER_POOL_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_DMA_BUFFER_POOL_HPP

#include <chrono>
#include <cstddef>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <memory>
#include <string>

namespace qrb::adapter
{

/**
 * @brief Bounded pool for DMA-BUFs owned by one ROS Flow Bridge.
 *
 * Acquire() returns a shared_ptr whose last release returns the backing buffer
 * to the pool. This matches qrb_ros_transport's message ownership model while
 * keeping HCM buffers outside the ROS-side lifetime domain.
 */
class RosDmaBufferPool
{
public:
  explicit RosDmaBufferPool(std::size_t maxBuffers, std::string heapName = "/dev/dma_heap/system");
  ~RosDmaBufferPool();

  RosDmaBufferPool(const RosDmaBufferPool &) = delete;
  RosDmaBufferPool & operator=(const RosDmaBufferPool &) = delete;

  /**
   * @brief Acquire an exact-sized buffer within `timeout`.
   *
   * When every bounded slot is held by ROS messages, the caller waits for a
   * subscriber to release one. Returns nullptr on timeout, shutdown or an
   * allocation failure.
   */
  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> Acquire(std::size_t size,
      std::chrono::milliseconds timeout);

  /** @brief Stop accepting returned buffers and release all currently idle buffers. */
  void Shutdown();

private:
  struct State;
  std::shared_ptr<State> state_;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_DMA_BUFFER_POOL_HPP
