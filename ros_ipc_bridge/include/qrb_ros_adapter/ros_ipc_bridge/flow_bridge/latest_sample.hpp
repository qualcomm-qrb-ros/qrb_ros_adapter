// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_LATEST_SAMPLE_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_LATEST_SAMPLE_HPP

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <utility>

namespace qrb::adapter
{

/**
 * @brief Holds the most recent message from a Subscription until the HCM IPC Adapter
 * asks for it.
 *
 * ### Why this exists rather than Subscription::take()
 *
 * A ros_to_hcm Flow is pull-driven: the HCM IPC Adapter sends
 * FETCH_REQUEST and the ROS IPC Bridge waits for one fresh sample within the
 * configured bound. The obvious
 * implementation is a non-blocking `Subscription::take()` at request time, and
 * that is what an earlier revision did.
 *
 * It does not work. `take()` reads the *middleware* queue, and messages
 * published intra-process never enter it -- rclcpp routes those through a
 * separate intra-process buffer delivered only via the executor's callback path.
 * So a `take()`-based bridge silently sees nothing at all from an
 * intra-process publisher, which is precisely the deployment supported by the
 * qrb_ros_transport input path. The failure mode is the worst kind:
 * a correctly configured zero-copy Flow that never observes a sample.
 *
 * Taking delivery through a normal callback works for both transports, so that
 * is what the Components do, and this class is where the callback parks the
 * message until it is asked for.
 *
 * ### Latest-sample semantics
 *
 * Only the newest message is kept. That is deliberate and matches the
     * latest-sample policy in HLD §4.3: the HCM IPC Adapter pulls at its own cadence, so when ROS
 * publishes faster than it pulls, something must be dropped -- and for sensor
 * and pose data the newest sample is the useful one. The middleware QoS depth
 * still bounds what the subscription itself buffers.
 *
 * Thread-safe: the executor stores, the Flow's service thread takes.
 */
template <typename MessageT>
class LatestSample
{
public:
    /** @brief Replace the held sample. Called from the subscription callback. */
    void Store( MessageT message )
    {
        {
            std::lock_guard<std::mutex> lock( mutex_ );
            if ( stopped_ ) return;
            message_ = std::move( message );
            fresh_ = true;
        }
        condition_.notify_one();
    }

    /**
     * @brief Consume the held sample, if one has arrived since the last take.
     *
     * @return false if nothing new is immediately available.
     */
    bool Take( MessageT &out )
    {
        std::lock_guard<std::mutex> lock( mutex_ );
        if ( !fresh_ ) return false;
        out = std::move( message_ );
        message_ = MessageT();
        fresh_ = false;
        return true;
    }

    /** @brief Wait for and consume one fresh sample within a bounded interval. */
    bool WaitAndTake( MessageT &out, std::chrono::milliseconds timeout )
    {
        std::unique_lock<std::mutex> lock( mutex_ );
        if ( !condition_.wait_for( lock, timeout, [this]() { return fresh_ || stopped_; } )
            || !fresh_ )
        {
            return false;
        }
        out = std::move( message_ );
        message_ = MessageT();
        fresh_ = false;
        return true;
    }

    /** @brief True if a sample is waiting. */
    bool HasSample() const
    {
        std::lock_guard<std::mutex> lock( mutex_ );
        return fresh_;
    }

    /** @brief Drop any held sample, releasing whatever it owns. */
    void Clear()
    {
        std::lock_guard<std::mutex> lock( mutex_ );
        message_ = MessageT();
        fresh_ = false;
    }

    /** @brief Wake a pending WaitAndTake() during component teardown. */
    void Shutdown()
    {
        {
            std::lock_guard<std::mutex> lock( mutex_ );
            stopped_ = true;
        }
        condition_.notify_all();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    MessageT message_;
    bool fresh_{ false };
    bool stopped_{ false };
};

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_LATEST_SAMPLE_HPP
