// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_dma_buffer_pool.hpp"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <utility>
#include <vector>

namespace qrb::adapter
{

struct RosDmaBufferPool::State
{
  explicit State(std::size_t max, std::string heap) : maxBuffers(max), heapName(std::move(heap)) {}

  std::mutex mutex;
  std::condition_variable availableCondition;
  std::vector<std::shared_ptr<lib_mem_dmabuf::DmaBuffer>> available;
  std::size_t allocated{ 0 };
  const std::size_t maxBuffers;
  const std::string heapName;
  bool acceptingReturns{ true };
};

RosDmaBufferPool::RosDmaBufferPool(std::size_t maxBuffers, std::string heapName)
  : state_(std::make_shared<State>(maxBuffers, std::move(heapName)))
{
}

RosDmaBufferPool::~RosDmaBufferPool()
{
  Shutdown();
}

std::shared_ptr<lib_mem_dmabuf::DmaBuffer> RosDmaBufferPool::Acquire(std::size_t size,
    std::chrono::milliseconds timeout)
{
  if (size == 0 || timeout.count() < 0 || state_ == nullptr || state_->maxBuffers == 0) {
    return nullptr;
  }

  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> backing;
  bool allocate = false;
  {
    std::unique_lock<std::mutex> lock(state_->mutex);
    if (!state_->availableCondition.wait_for(lock, timeout, [this]() {
          return !state_->acceptingReturns || !state_->available.empty() ||
                 state_->allocated < state_->maxBuffers;
        })) {
      return nullptr;
    }
    if (!state_->acceptingReturns)
      return nullptr;

    auto exact = std::find_if(
        state_->available.begin(), state_->available.end(), [size](const auto & buffer) {
          return buffer != nullptr && buffer->size() >= 0 &&
                 static_cast<std::size_t>(buffer->size()) == size;
        });
    if (exact != state_->available.end()) {
      backing = std::move(*exact);
      state_->available.erase(exact);
    } else {
      // Payload size is fixed for most Flows. If it changes, replace one
      // idle allocation rather than letting stale size classes consume
      // the bounded pool forever.
      if (!state_->available.empty()) {
        backing = std::move(state_->available.back());
        state_->available.pop_back();
        --state_->allocated;
      }

      if (state_->allocated < state_->maxBuffers) {
        ++state_->allocated;
        allocate = true;
      }
    }
  }

  if (allocate) {
    backing = lib_mem_dmabuf::DmaBuffer::alloc(size, state_->heapName);
    if (backing == nullptr) {
      {
        std::lock_guard<std::mutex> lock(state_->mutex);
        --state_->allocated;
      }
      state_->availableCondition.notify_one();
      return nullptr;
    }
  }

  if (backing == nullptr)
    return nullptr;

  std::weak_ptr<State> weakState = state_;
  return std::shared_ptr<lib_mem_dmabuf::DmaBuffer>(backing.get(),
      [weakState, backing = std::move(backing)](lib_mem_dmabuf::DmaBuffer *) mutable {
        std::shared_ptr<State> state = weakState.lock();
        if (state == nullptr)
          return;

        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->acceptingReturns) {
          state->available.push_back(std::move(backing));
        }
        state->availableCondition.notify_one();
      });
}

void RosDmaBufferPool::Shutdown()
{
  if (state_ == nullptr)
    return;

  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->acceptingReturns = false;
    state_->available.clear();
  }
  state_->availableCondition.notify_all();
}

}  // namespace qrb::adapter
