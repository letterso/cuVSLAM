# Tutorial: Tracking ROS1/ROS2 Bags with PyCuVSLAM

This tutorial shows how to parse ROS1/ROS2 bags with `rosbags` and run cuVSLAM odometry from recorded topics.
ROS1 runtime packages are not required because `rosbags` handles decoding directly.

## Supported Inputs

The script supports the following ROS message types:

- `sensor_msgs/msg/Image` for camera images (`mono8`, `8UC1`, `rgb8`, `bgr8`)
- `sensor_msgs/msg/Image` for depth (`mono16`, `16UC1`, `32FC1`)
- `sensor_msgs/msg/Imu` for inertial mode

## Quick Start

### 1) Sync Python dependencies

```bash
cd <path-to-cuvslam>
uv sync
```

### 2) Install PyCuVSLAM

Install a pre-built wheel from the
[cuVSLAM releases page](https://github.com/nvidia-isaac/cuVSLAM/releases),
or build and install from source as described in the
[root README](../../README.md).

1. Build manually

    ```bash
    mkdir build
    cd build
    cmake ..
    make -j
    ```

2. install PyCuVSLAM from repository 

    ```bash
    CUVSLAM_BUILD_DIR=<path-to-cuvslam-build> uv pip install python/
    ```

### 3) Run tracking

```bash
cd <path-to-cuvslam>/examples/rosbag
uv run track_rosbag.py --config config/rosbag_config.yaml
```

Trajectory output is written to:

- `config/output/trajectory_tum.txt`

## Parameter Reference

### CLI Argument

| Argument | Default | Description |
| --- | --- | --- |
| `--config` | `examples/rosbag/rosbag_config.yaml` (script default) | Path to YAML configuration file. Use a concrete file such as `examples/rosbag/config/rosbag_config.yaml`. |

### `rosbag` section

| Key | Required | Default | Description |
| --- | --- | --- | --- |
| `rosbag.bag_path` | Yes | None | ROS1 `.bag` file path or ROS2 bag directory. Relative paths are resolved from the config file directory. |
| `rosbag.typestore` | No | `ROS2_FOXY` | Rosbags typestore used for decoding and bag-type inference, for example `ROS1_NOETIC`, `ROS2_FOXY`. |
| `rosbag.sync_tolerance_ms` | No | `10.0` | Timestamp tolerance (milliseconds) for multi-topic synchronization. |
| `rosbag.depth_float_to_uint16_scale` | No | `1000.0` | Scale used when converting `float32` depth to `uint16`. |
| `rosbag.max_frames` | No | None | Stop after this many tracked frames. Must be positive when set. |

### `topics` section

| Key | Required | Default | Description |
| --- | --- | --- | --- |
| `topics.image_topics` | Yes | None | Ordered image topics. Order must match `rig.cameras` order. |
| `topics.depth_topic` | RGBD mode only | None | Depth image topic used in `RGBD` mode. |
| `topics.imu_topic` | INERTIAL mode only | None | IMU topic used in `INERTIAL` mode. |

### `tracker` section

| Key | Required | Default | Description |
| --- | --- | --- | --- |
| `tracker.odometry_mode` | Yes | None | One of `MULTICAMERA`, `INERTIAL`, `RGBD`, `MONO`. |
| `tracker.async_sba` | No | `false` | Enable asynchronous SBA pipeline. |
| `tracker.enable_observations_export` | No | `true` | Export feature observations from tracker for debugging/visualization. |
| `tracker.enable_final_landmarks_export` | No | `true` | Export final landmarks at the end of tracking. |
| `tracker.rectified_stereo_camera` | No | `true` | Mark stereo input as rectified when applicable. |
| `tracker.rgbd_settings.depth_scale_factor` | RGBD mode only | `1000.0` | Depth scale passed to RGBD tracker settings. |
| `tracker.rgbd_settings.depth_camera_id` | RGBD mode only | `0` | Camera index used for depth in RGBD mode. |
| `tracker.rgbd_settings.enable_depth_stereo_tracking` | RGBD mode only | `false` | Enable depth-assisted stereo behavior. |

### `rig` section

| Key | Required | Default | Description |
| --- | --- | --- | --- |
| `rig.cameras` | Yes | None | Camera list; must contain at least one camera. |
| `rig.cameras[].size` | Yes | None | `[width, height]`. |
| `rig.cameras[].focal` | Yes | None | `[fx, fy]`. |
| `rig.cameras[].principal` | Yes | None | `[cx, cy]`. |
| `rig.cameras[].rig_from_camera` | No | Identity if omitted | Extrinsic pose from camera to rig (`translation`, `rotation`). |
| `rig.cameras[].distortion.model` | No | None | `BROWN` or `FISHEYE`. |
| `rig.cameras[].distortion.params` | No | `[]` | Distortion parameters for the selected model. |
| `rig.cameras[].border_top/bottom/left/right` | No | `0` | Per-edge crop borders. |
| `rig.imus` | INERTIAL mode only | `[]` | IMU calibration list; at least one entry is required in inertial mode. |
| `rig.imus[].rig_from_imu` | Yes (when IMU is used) | None | Extrinsic pose from IMU to rig (`translation`, `rotation`). |
| `rig.imus[].gyroscope_noise_density` | Yes (when IMU is used) | None | Gyroscope noise density. |
| `rig.imus[].gyroscope_random_walk` | Yes (when IMU is used) | None | Gyroscope random walk. |
| `rig.imus[].accelerometer_noise_density` | Yes (when IMU is used) | None | Accelerometer noise density. |
| `rig.imus[].accelerometer_random_walk` | Yes (when IMU is used) | None | Accelerometer random walk. |
| `rig.imus[].frequency` | Yes (when IMU is used) | None | IMU sample frequency in Hz. |

### `visualization` section

| Key | Required | Default | Description |
| --- | --- | --- | --- |
| `visualization.enabled` | No | `true` | Enable/disable rerun logging. |
| `visualization.app_name` | No | `rosbag_cuvslam` | Rerun application name. |
| `visualization.mode` | No | `spawn` | `spawn` starts viewer; `connect` sends to an existing rerun server. |
| `visualization.grpc_url` | No | `127.0.0.1:9876` | Target endpoint for `connect` mode. |
| `visualization.strict` | No | `true` | Forwarded to `rr.init(strict=...)`. |
| `visualization.spawn` | No | `true` | Forwarded to `rr.init(spawn=...)` in spawn mode. |
| `visualization.log_trajectory` | No | `true` | Log 3D trajectory line strip. |
| `visualization.log_pose` | No | `true` | Log camera transform. |
| `visualization.log_pose_axes` | No | `true` | Log XYZ axis arrows. |
| `visualization.log_images` | No | `true` | Log image streams. |
| `visualization.log_observations` | No | `true` | Log 2D feature observations. |
| `visualization.log_depth` | No | `true` | Log depth stream when `topics.depth_topic` is set. |

### `output` section

| Key | Required | Default | Description |
| --- | --- | --- | --- |
| `output.trajectory_path` | No | None | Output trajectory file path. Relative paths are resolved from the config file directory. |

## Validation Rules

- `len(topics.image_topics)` must match `len(rig.cameras)`.
- `RGBD` mode requires exactly one image topic and `topics.depth_topic`.
- `INERTIAL` mode requires `topics.imu_topic` and at least one `rig.imus` entry.
