<div align="center">
  <h1>QRB ROS Adapter</h1>
  <p>HCM and ROS 2 Bridge for Qualcomm Robotics Platforms</p>

  <img src="https://img.shields.io/badge/Qualcomm%20Linux-3253DC?style=for-the-badge&logo=qualcomm&logoColor=white" alt="Qualcomm Linux">
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
</div>

---

## Overview

QRB ROS Adapter bridges Contract messages between HCM and ROS 2 over Unix
Domain Sockets. The HCM IPC Adapter is a standard synchronous QCNode; the ROS
IPC Bridge creates one ROS Flow Bridge for each Flow an Adapter registers.

- `ipc_runtime`: IPC definitions, transport and Contract validation over the
  `qrb_contracts_proto` messages.
- `hcm_ipc_adapter`: the IPC Adapter Node. Needs QCNode, not ROS.
- `ros_ipc_bridge`: the Bridge Manager and ROS Flow Bridge. Needs ROS, not
  QCNode.
- `test`: the end-to-end stream probe. The ROS side is a ROS package; the HCM
  side is a QCNode sample built with `hcm_ipc_adapter`.

## Table of contents

- [APIs](#apis)
- [Supported targets](#supported-targets)
- [Installation](#installation)
- [Usage](#usage)
- [Build from source](#build-from-source)
- [Test](#test)
- [License](#license)

## APIs

The IPC Adapter Node is built into `libQCNode.so` by the qcnode build. An HCM
application, or a graph framework hosting QCNodes, creates one node per Flow
with `QCNodeFactory::CreateNode(QC_NODE_TYPE_IPC_ADAPTER, ...)` and drives it
through `Initialize()`, `Start()`, `ProcessFrameDescriptor()`, `Stop()` and
`DeInitialize()`. The host owns the frame descriptors and their buffers.

The Flow is set in the QCNode static config:

| Field | Required | Description |
| --- | --- | --- |
| `contract_type` | yes | Contract full name, e.g. `qrb.contract.sensor.v1.PointCloud2`. |
| `direction` | yes | `0`: HCM to ROS (the node publishes the descriptor); `1`: ROS to HCM (the node fills it in). |
| `ros_topic` | yes | ROS topic. |
| `qos_profile` | no | `reliability` (`reliable`/`best_effort`), `durability` (`volatile`/`transient_local`), `depth` (1–32); defaults to `rclcpp::QoS(10)`. |

The supported Contracts, each with its HCM descriptor, ROS message and payload
class, are the entries of
[contract_mappings.def](ipc_runtime/include/qrb_ros_adapter/ipc_runtime/contract_mappings.def).
A DmaBuf Contract carries its payload in a DMA-BUF behind the descriptor. The
host allocates that storage: for HCM to ROS (`direction` 0) the source must be a
local DMA-BUF; for ROS to HCM (`direction` 1) the host receives into a DMA-BUF it
offers, or into heap storage. A TFMessage is Inline but variable-length, so its
transforms need host storage in the HCM descriptor (heap or a local DMA-BUF). A
frame name longer than the HCM descriptor holds is rejected, not truncated.

Monitoring reports `sessionConnected`, `frameCount`, `errorCount`,
`last/average/min/maxLatencyNs` and `noDataCount`.

The ROS IPC Bridge takes no parameters and adds one ROS node for each
registered Flow.

## Supported targets

| Development hardware | Operating system | ROS |
| --- | --- | --- |
| Qualcomm IQ-10 Ride | Qualcomm Linux (IQ10.LE.2.0) | Jazzy |

## Installation

Installation packages and setup instructions are not ready yet.

## Usage

Start the ROS IPC Bridge:

```bash
bash -c 'source /usr/ros/jazzy/setup.bash && ROS_HOME=/data/ros ros2 launch qrb_ros_adapter ros_ipc_bridge.launch.py'
```

On the HCM side, create one IPC Adapter Node per Flow with `QCNodeFactory` and
drive it like any other
[QCNode](https://github.com/qualcomm/QcNode/blob/main/docs/QCNode.md). Buffer 0
of the frame is the Contract descriptor from `CreateQrbBufferDescriptor()`:
direction 0 publishes it, direction 1 fills it with the newest ROS message.

`SampleIpcAdapter` is such a host:

```bash
qcrun QCNodeSampleApp -n adapter -t IpcAdapter [-k <setting> -v <value> ...]
```

### Sample settings

The `IpcAdapter` and `StreamProbe` samples take `-k <setting> -v <value>`; the
ROS probe of the [Test](#test) takes `<setting>:=<value>` on its launch.

| Setting | Default | Description |
| --- | --- | --- |
| `contract_type`, `direction`, `ros_topic` | `PointCloud2`, `0`, per Contract | IpcAdapter: the static config fields above. |
| `reliability`, `durability`, `depth` | `rclcpp::QoS(10)` | IpcAdapter: the `qos_profile` fields above. |
| `adapter_config` | StreamProbe: required; ROS probe: `adapter_h2r.json` | All: a static config file; on IpcAdapter it replaces the two rows above. |
| `input_topic` / `output_topic` | | IpcAdapter, StreamProbe: takes frames from / gives frames to another sample. |
| `fps` | `30` | All: frames per second a sender sends, `0` for no pause. |
| `payload_bytes` | `1048576` | All: DMA-BUF payload of a DmaBuf Contract a sender sends. |
| `duration` | `0` | ROS probe: seconds to run, `0` until stopped. |
| `log` | | ROS probe: text log file. |

## Build from source

Source build instructions are not ready yet.

## Test

The stream probe runs one Flow end to end: the `StreamProbe` sample feeds or
drains the `IpcAdapter` sample, and the ROS probe runs in the Bridge container.
The image needs the packages and ros2cli.

1. Start the Bridge with the ROS probe:

   ```bash
   bash -c 'source /usr/ros/jazzy/setup.bash && ROS_HOME=/data/ros ros2 launch qrb_ros_adapter_stream_probe qrb_ros_adapter_stream_probe.launch.py'
   ```

2. In another terminal, send 1 MiB PointCloud2 frames from HCM to ROS at 30 FPS
   for 10 seconds:

   ```bash
   CFG=/usr/ros/jazzy/share/qrb_ros_adapter_stream_probe/probe_configs
   qcrun QCNodeSampleApp -T 10 \
     -n adapter -t IpcAdapter -k adapter_config -v $CFG/adapter_h2r.json -k input_topic -v /frames \
     -n probe -t StreamProbe -k adapter_config -v $CFG/adapter_h2r.json -k output_topic -v /frames
   ```

For ROS to HCM, use `adapter_r2h.json` on both sides and list the probe first:

```bash
# Terminal 1
bash -c 'source /usr/ros/jazzy/setup.bash && ROS_HOME=/data/ros ros2 launch qrb_ros_adapter_stream_probe qrb_ros_adapter_stream_probe.launch.py adapter_config:=/usr/ros/jazzy/share/qrb_ros_adapter_stream_probe/probe_configs/adapter_r2h.json'
# Terminal 2, with CFG set as above
qcrun QCNodeSampleApp -T 10 \
  -n probe -t StreamProbe -k adapter_config -v $CFG/adapter_r2h.json -k input_topic -v /frames \
  -n adapter -t IpcAdapter -k adapter_config -v $CFG/adapter_r2h.json -k output_topic -v /frames
```

For another Contract, use its `adapter_<contract>_{h2r,r2h}.json` from the same
directory, on both sides and in the same commands.

The run passes when the HCM side prints `probe PASS`. The probe settings are
in [Sample settings](#sample-settings).

## License

This project is licensed under the BSD-3-Clause-Clear license.
