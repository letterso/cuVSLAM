# bag2edex — Convert ROS 2 Bag to EDEX

Converts a ROS 2 bag into an EDEX sequence directory (images + `edex` + `frame_metadata.jsonl` + optional `imu.jsonl`) for offline cuVSLAM analysis with the `tracker` tool.

This is the `cuvslam-tools` package. It supports multiple cameras, config-driven topic/frame configuration, all ROS 2 distros (Foxy through Jazzy/Kilted), optional image resizing, and automatic extrinsic extraction from TF.

---

## Install

```bash
cd tools/rosbag_extraction
python3 -m venv .env
source .env/bin/activate
pip install -e .
```

Or use the provided script:

```bash
cd tools/rosbag_extraction
./create_env.sh
source .env/bin/activate
```

---

## Configuration

Create a YAML config file (see `cuvslam_tools/bag2edex/configs/` for examples):

```yaml
# Topic names for CameraInfo messages
camera_info_topics:
  - /camera/infra1/camera_info
  - /camera/infra2/camera_info

# Topic names for image messages (same order as camera_info_topics)
image_topics:
  - /camera/infra1/image_rect_raw
  - /camera/infra2/image_rect_raw

# Rig frame for extrinsic extraction (run rosbag_extract_urdf to see available frames)
rig_frame: camera_link

# IMU topic (optional — omit for VO-only mode)
imu_topic: /camera/imu

# ROS distribution of the bag (foxy, humble, iron, jazzy, kilted — default: humble)
ros_distribution: humble
```

---

## Usage

Extract a full EDEX dataset from a rosbag:

```bash
rosbag_extract_edex \
    --config cuvslam_tools/bag2edex/configs/realsense.yaml \
    --rosbag_path path/to/bag_folder \
    --output_path path/to/edex_folder
```

List available TF frames (to find the right `rig_frame`):

```bash
rosbag_extract_urdf \
    --rosbag_path path/to/bag_folder \
    --output_path /tmp/urdf_out \
    --ros_distribution humble
```

Extract images only (without extrinsics):

```bash
rosbag_extract_images \
    --config cuvslam_tools/bag2edex/configs/realsense.yaml \
    --rosbag_path path/to/bag_folder \
    --output_path path/to/output_folder
```

---

## What it produces

| File | Contents |
|---|---|
| `edex` | Camera intrinsics, extrinsics, IMU transform, sequence frame list |
| `images/<topic>/NNNNN.png` | Extracted camera frames |
| `frame_metadata.jsonl` | Per-frame metadata with filenames and timestamps |
| `imu.jsonl` | IMU samples (if `imu_topic` is set in config) |
| `robot.urdf` | URDF extracted from `/tf_static` |

---

## Sensor configs

| Config | Sensor |
|---|---|
| `cuvslam_tools/bag2edex/configs/realsense.yaml` | RealSense (configurable topics) |
| `cuvslam_tools/bag2edex/configs/nova_hawk.yaml` | NVIDIA Nova + HAWK stereo |
| `cuvslam_tools/bag2edex/configs/oak6.yaml` | OAK-6 camera |

---

## Troubleshooting

**`AssertionError: Failed to parse <type>`** — set `ros_distribution: jazzy` (or the correct distro) in the config. The `rosbags` library uses distro-specific type stores to parse message definitions.

**`Rig frame 'X' not found`** — run `rosbag_extract_urdf` to list all TF frames in the bag, then update `rig_frame` in the config.

**Missing topics** — the tool warns and skips topics not present in the bag. Check topic names with `ros2 bag info <bag_folder>`.

**Wrong extrinsics** — extrinsics are derived from `/tf_static` in the bag. Verify the TF tree is complete and the `rig_frame` is correct.
