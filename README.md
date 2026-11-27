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
| `name` | yes | Flow name, unique among the Flows of a Bridge, also with each character other than a letter or digit read as `_`. |
| `id` | no | QCNode id (0–255), default 0; `SampleIpcAdapter` defaults it to its sample id. |
| `contract_type` | yes | Contract full name, e.g. `qrb.contract.sensor.v1.PointCloud2`. |
| `direction` | yes | `0`: HCM to ROS (the node publishes the descriptor); `1`: ROS to HCM (the node fills it in). |
| `ros_topic` | yes | ROS topic. |
| `qos_profile` | no | `reliability` (`reliable`/`best_effort`), `durability` (`volatile`/`transient_local`), `depth` (1–32); defaults to `rclcpp::QoS(10)`. |

The supported Contracts are listed in
[contract_mappings.def](ipc_runtime/include/qrb_ros_adapter/ipc_runtime/contract_mappings.def).
A DmaBuf Contract carries its payload in a DMA-BUF the host allocates; from HCM
to ROS it must be a local DMA-BUF. A TFMessage needs host storage for its
transforms in the descriptor.

Monitoring reports `sessionConnected`, `frameCount`, `errorCount`,
`last/average/min/maxLatencyNs` and `noDataCount`, the ROS to HCM calls that
found no new message. Errors are logged through the QCNode default logger.

One Bridge runs per device and serves up to 32 Flows, each with its own `name`.
Flows of one direction may share a `ros_topic`; the Bridge rejects a Flow of the
same Contract in the other direction on that topic.

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
direction 0 publishes it, direction 1 fills it with the oldest ROS message
waiting. Up to `depth` messages wait, so a late call gets the next message
instead of missing it.

`SampleIpcAdapter` is such a host. It passes the frames of another sample, so
run it together with one, such as the `StreamProbe` of the [Test](#test):

```bash
qcrun QCNodeSampleApp -n adapter -t IpcAdapter [-k <setting> -v <value> ...]
```

<details>
<summary>IpcAdapter settings</summary>

| Setting | Default | Description |
| --- | --- | --- |
| `adapter_config` | | A static config file; it replaces the six settings below. |
| `contract_type`, `direction`, `ros_topic` | `PointCloud2`, `0`, per Contract | The static config fields above. |
| `reliability`, `durability`, `depth` | `rclcpp::QoS(10)` | The `qos_profile` fields above. |
| `input_topic`, `output_topic` | | DataBroker topics of the frames it takes from and gives to another sample, such as the StreamProbe. From HCM to ROS, it sends the frames of `input_topic`. From ROS to HCM, it fills the frames of `input_topic` and gives them on `output_topic`. `input_topic` is required, and `output_topic` too from ROS to HCM. |
| `log_file` | | Text log file of what it prints and a monitor line every 10 s. |

From HCM to ROS, it sends each frame as it takes it. From ROS to HCM, it fetches
every waiting message at once and tries again 1 ms later when none waits.

Each line of the log file starts with the CLOCK_MONOTONIC time and the sample
name. `fps` is the rate of the last 10000 frames; the monitor line printed at
Stop also gives `sessionConnected` and the last, min and max latency.

```text
[28087.995013214] [adapter]: monitor: frameCount=300, errorCount=0, averageLatencyNs=89399, noDataCount=0, fps=30.00
```

</details>

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
     -n adapter -t IpcAdapter -k adapter_config -v $CFG/adapter_point_cloud_h2r.json -k input_topic -v /adapter_in \
     -n probe -t StreamProbe -k adapter_config -v $CFG/adapter_point_cloud_h2r.json -k output_topic -v /adapter_in
   ```

For ROS to HCM, use `adapter_point_cloud_r2h.json` on both sides and list the
probe first:

```bash
# Terminal 1
bash -c 'source /usr/ros/jazzy/setup.bash && ROS_HOME=/data/ros ros2 launch qrb_ros_adapter_stream_probe qrb_ros_adapter_stream_probe.launch.py adapter_config:=/usr/ros/jazzy/share/qrb_ros_adapter_stream_probe/probe_configs/adapter_point_cloud_r2h.json'
# Terminal 2, with CFG set as above
qcrun QCNodeSampleApp -T 10 \
  -n probe -t StreamProbe -k adapter_config -v $CFG/adapter_point_cloud_r2h.json -k output_topic -v /adapter_in -k input_topic -v /adapter_out \
  -n adapter -t IpcAdapter -k adapter_config -v $CFG/adapter_point_cloud_r2h.json -k input_topic -v /adapter_in -k output_topic -v /adapter_out
```

For another Contract, use its `adapter_<contract>_{h2r,r2h}.json` from the same
directory, on both sides and in the same commands.

The run passes when the HCM side prints `probe PASS`. Each probe either sends
or receives, by the `direction` of the static config.

<details>
<summary>StreamProbe settings</summary>

| Setting | Default | Description |
| --- | --- | --- |
| `adapter_config` | required | The static config, the same file as the IpcAdapter's. |
| `output_topic`, `input_topic` | required | DataBroker topics. From HCM to ROS, it gives the frames it sends on `output_topic` and takes no `input_topic`. From ROS to HCM, it gives empty frames on `output_topic` and checks the filled ones of `input_topic`. |
| `fps` | `30` | Frames per second it sends from HCM to ROS, `0` for no pause. |
| `point_cloud_bytes` | `1048576` | Bytes of each PointCloud2 it sends. From ROS to HCM, it sizes the buffers of its frames by it. |
| `image_width`, `image_height` | `1920`, `1080` | Size of each rgb8 ImageLayout it sends. From ROS to HCM, it sizes the buffers of its frames by them. |

```bash
qcrun QCNodeSampleApp -n probe -t StreamProbe [-k <setting> -v <value> ...]
```

</details>

<details>
<summary>ROS probe parameters</summary>

| Parameter | Default | Description |
| --- | --- | --- |
| `adapter_config` | `adapter_point_cloud_h2r.json` in `probe_configs` | The static config, the same file as the HCM side's. |
| `fps` | `30` | Frames per second it sends from ROS to HCM, `0` for no pause. |
| `point_cloud_bytes` | `1048576` | Bytes of each PointCloud2 it sends from ROS to HCM. |
| `image_width`, `image_height` | `1920`, `1080` | Size of each rgb8 ImageLayout it sends from ROS to HCM. |
| `duration` | `0` | Seconds to run, `0` until stopped. |
| `log_file` | | Text log file of each frame, a monitor line every 5 s and the summary. |

```bash
ros2 launch qrb_ros_adapter_stream_probe qrb_ros_adapter_stream_probe.launch.py [<parameter>:=<value> ...]
ros2 component load /qrb_ros_adapter_container qrb_ros_adapter_stream_probe qrb::adapter::stream_probe::RosStreamProbeNode [-p <parameter>:=<value> ...]
ros2 run qrb_ros_adapter_stream_probe ros_stream_probe_node --ros-args [-p <parameter>:=<value> ...]
```

</details>

### Several Flows at once

One Bridge serves every Flow; each config in `probe_configs` has its own `name`
and `ros_topic`, so the Flows run side by side. PointCloud2 from HCM to ROS
and PoseStamped from ROS to HCM:

1. Start the Bridge alone:

   ```bash
   bash -c 'source /usr/ros/jazzy/setup.bash && ROS_HOME=/data/ros ros2 launch qrb_ros_adapter ros_ipc_bridge.launch.py'
   ```

2. In another terminal, run `bash`, then load one ROS probe per Flow into the
   Bridge container:

   ```bash
   source /usr/ros/jazzy/setup.bash && export ROS_HOME=/data/ros
   CFG=/usr/ros/jazzy/share/qrb_ros_adapter_stream_probe/probe_configs PROBE=qrb::adapter::stream_probe::RosStreamProbeNode
   ros2 component load /qrb_ros_adapter_container qrb_ros_adapter_stream_probe $PROBE -n probe_point_cloud_h2r -p adapter_config:=$CFG/adapter_point_cloud_h2r.json
   ros2 component load /qrb_ros_adapter_container qrb_ros_adapter_stream_probe $PROBE -n probe_pose_stamped_r2h -p adapter_config:=$CFG/adapter_pose_stamped_r2h.json
   ```

3. Run the HCM side of each Flow in its own terminal, at the same time. In the
   terminal of step 2:

   ```bash
   qcrun QCNodeSampleApp -T 10 \
     -n adapter -t IpcAdapter -k adapter_config -v $CFG/adapter_point_cloud_h2r.json -k input_topic -v /adapter_in \
     -n probe -t StreamProbe -k adapter_config -v $CFG/adapter_point_cloud_h2r.json -k output_topic -v /adapter_in
   ```

   In a third terminal:

   ```bash
   CFG=/usr/ros/jazzy/share/qrb_ros_adapter_stream_probe/probe_configs
   qcrun QCNodeSampleApp -T 10 \
     -n probe -t StreamProbe -k adapter_config -v $CFG/adapter_pose_stamped_r2h.json -k output_topic -v /adapter_in -k input_topic -v /adapter_out \
     -n adapter -t IpcAdapter -k adapter_config -v $CFG/adapter_pose_stamped_r2h.json -k input_topic -v /adapter_in -k output_topic -v /adapter_out
   ```

Each terminal prints `probe PASS`. The probes stay loaded until the Bridge
stops (Ctrl-C in its terminal).

## License

This project is licensed under the BSD-3-Clause-Clear license.
