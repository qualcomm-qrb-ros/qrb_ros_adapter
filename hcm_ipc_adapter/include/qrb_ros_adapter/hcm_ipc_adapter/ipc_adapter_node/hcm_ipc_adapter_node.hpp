// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_NODE_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_NODE_HPP

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_flow_client.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_config.hpp"

#include "QC/Node/Ifs/QCNodeConfigIfs.hpp"
#include "QC/Node/Ifs/QCNodeIfs.hpp"
#include "QC/Node/Ifs/QCNodeMonitoringIfs.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace qrb::adapter
{

struct HcmIpcAdapterNodeConfigData : public QC::QCNodeConfigBase_t
{
    HcmIpcAdapterConfig adapter;
};

class HcmIpcAdapterNodeConfig final : public QC::QCNodeConfigIfs
{
public:
    QC::QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    const std::string &GetOptions() override;
    const QC::QCNodeConfigBase_t &Get() override;

    const HcmIpcAdapterConfig &Adapter() const { return config_.adapter; }

private:
    HcmIpcAdapterNodeConfigData config_{};
    std::string options_;
};

struct HcmIpcAdapterMonitoringData : public QC::QCNodeMonitoringBase_t
{
    std::uint32_t sessionConnected{ 0 };
    std::uint64_t frameCount{ 0 };
    std::uint64_t errorCount{ 0 };
    std::uint64_t lastLatencyNs{ 0 };
    std::uint64_t averageLatencyNs{ 0 };
    std::uint64_t maxLatencyNs{ 0 };
};

class HcmIpcAdapterNodeMonitoring final : public QC::QCNodeMonitoringIfs
{
public:
    HcmIpcAdapterNodeMonitoring();

    QC::QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    const std::string &GetOptions() override;
    const QC::QCNodeMonitoringBase_t &Get() override;
    std::uint32_t GetMaximalSize() override;
    std::uint32_t GetCurrentSize() override;
    QC::QCStatus_e Place( void *data, std::uint32_t &size ) override;

    void UpdateSession( bool sessionConnected );
    void RecordProcess( QC::QCStatus_e status, bool sessionConnected,
        bool latencyMeasured, std::uint64_t latencyNs );
    void Reset();

private:
    std::mutex mutex_;
    HcmIpcAdapterMonitoringData data_{};
    std::uint64_t totalLatencyNs_{ 0 };
    std::string options_;
};

/**
 * @brief GF-managed synchronous QCNode implementation for one HCM IPC Flow.
 *
 * GF owns scheduling, threads, frame descriptors, and buffer lifetime. This
 * node implements the standard QCNodeIfs extension point directly and keeps
 * no second lifecycle wrapper. It borrows local frame buffers only for the
 * duration of ProcessFrameDescriptor().
 */
class HcmIpcAdapterNode final : public QC::QCNodeIfs
{
public:
    HcmIpcAdapterNode() = default;
    ~HcmIpcAdapterNode();

    QC::QCStatus_e Initialize( QC::QCNodeInit_t &config ) override;
    QC::QCStatus_e DeInitialize() override;
    QC::QCStatus_e Start() override;
    QC::QCStatus_e Stop() override;
    QC::QCStatus_e ProcessFrameDescriptor( QC::QCFrameDescriptorNodeIfs &frameDesc ) override;
    QC::QCObjectState_e GetState() override;
    QC::QCNodeConfigIfs &GetConfigurationIfs() override;
    QC::QCNodeMonitoringIfs &GetMonitoringIfs() override;

private:
    QC::QCStatus_e ProcessRosToHcm( QC::QCFrameDescriptorNodeIfs &frameDesc );
    QC::QCStatus_e ProcessHcmToRos( QC::QCFrameDescriptorNodeIfs &frameDesc );
    QC::QCStatus_e FinishOperation( Status status );
    void UpdateMonitoring();

    std::mutex mutex_;
    std::atomic<QC::QCObjectState_e> state_{ QC::QC_OBJECT_STATE_INITIAL };
    HcmIpcAdapterNodeConfig config_;
    HcmIpcAdapterNodeMonitoring monitoring_;
    HcmFlowClient client_;
    bool sessionConnected_{ false };
};

/** @brief NodeFactoryFn-compatible factory for GFGraphManager. */
QC::QCStatus_e CreateHcmIpcAdapterNode(
    QC::QCNodeType_e type, std::unique_ptr<QC::QCNodeIfs> &outNode );

}   // namespace qrb::adapter

#endif   // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_NODE_HPP
