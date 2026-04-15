# Tutorial: Tracking a ROS2 Bag with PyCuVSLAM

This tutorial shows how to parse a ROS2 bag with `rosbags` and run cuVSLAM odometry from recorded topics.

## Supported Inputs

The script supports the following ROS message types:

- `sensor_msgs/msg/Image` for camera images (`mono8`, `8UC1`, `rgb8`, `bgr8`)
- `sensor_msgs/msg/Image` for depth (`mono16`, `16UC1`, `32FC1`)
- `sensor_msgs/msg/Imu` for inertial mode

## Install Dependencies

```bash
cd /home/robot/develop/cpp/cuVSLAM/examples
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -r rosbag/requirements.txt
```

## Prepare the Bag Path

The default config points to `examples/rosbag/dataset/ros2_bag`.

```bash
cd /home/robot/develop/cpp/cuVSLAM/examples
mkdir -p rosbag/dataset
SOURCE_BAG_DIR="${SOURCE_BAG_DIR:-/data/rosbags/demo_office}"; ln -sfn "$SOURCE_BAG_DIR" rosbag/dataset/ros2_bag
```

## Configure Tracking Parameters

Edit [rosbag_config.yaml](config/rosbag_config.yaml) and set:

- `rosbag.bag_path`: ROS2 bag directory
- `topics.image_topics`: image topic list in rig-camera order
- `topics.depth_topic`: depth topic for RGBD mode
- `topics.imu_topic`: IMU topic for Inertial mode
- `rig.cameras`: camera intrinsics and extrinsics
- `tracker.odometry_mode`: one of `RGBD`, `MONO`, `MULTICAMERA`, `INERTIAL`

## Run Tracking

```bash
cd /home/robot/develop/cpp/cuVSLAM/examples/rosbag
source ../.venv/bin/activate
python3 track_rosbag.py --config config/rosbag_config.yaml
```

After execution, the trajectory is saved to the configured path:

- `output/trajectory_tum.txt`

## EuRoC ROS2 Bag Quick Start

This repository includes [euroc.yaml](config/euroc/euroc.yaml) configured for the dataset at:

- `/home/robot/datasets/euroc/MH_03_medium`

Run it directly:

```bash
cd /home/robot/develop/cpp/cuVSLAM
uv run python examples/rosbag/track_rosbag.py --config examples/rosbag/config/euroc/euroc.yaml
```

The quick-start config limits processing to the first 120 synchronized frames (`rosbag.max_frames`) to speed up validation.

## EuRoC Stereo-Inertial Quick Start

For stereo-inertial odometry (`INERTIAL` mode with `/imu0`), use [euroc_inertial.yaml](config/euroc/euroc_inertial.yaml):

```bash
cd /home/robot/develop/cpp/cuVSLAM
uv run python examples/rosbag/track_rosbag.py --config examples/rosbag/euroc_inertial.yaml
```

The output trajectory is written to:

- `examples/rosbag/output/euroc_mh03_inertial_trajectory_tum.txt`

## TUM Stereo-Inertial ROS2 Bag Quick Start

For the TUM ROS2 bag under `/home/robot/datasets/tum/dataset-slides1_512_16`, use [tum.yaml](config/tum/tum.yaml):

```bash
cd /home/robot/develop/cpp/cuVSLAM
uv run python examples/rosbag/track_rosbag.py --config examples/rosbag/config/tum/tum.yaml
```

The output trajectory is written to:

- `examples/rosbag/output/tum_slides1_inertial_trajectory_tum.txt`

## Visualization Options

The `visualization` block is fully configurable in YAML:

- `enabled`: enable/disable rerun logging
- `app_name`: rerun application name
- `strict`: pass through to `rr.init(strict=...)`
- `spawn`: pass through to `rr.init(spawn=...)`
- `log_trajectory`: toggle 3D trajectory logging
- `log_pose`: toggle camera pose transform logging
- `log_pose_axes`: toggle XYZ axes logging
- `log_images`: toggle image stream logging
- `log_observations`: toggle feature observations logging
- `log_depth`: toggle depth image logging

## Stereo / Inertial Notes

For stereo or multi-camera tracking, update:

- `topics.image_topics` to include one topic per camera in the rig order
- `rig.cameras` with matching camera count and extrinsics

For inertial tracking, also set:

- `topics.imu_topic`
- `rig.imus` with IMU extrinsics and noise parameters
