# Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA software released under the NVIDIA Community License is intended to be used to enable
# the further development of AI and robotics technologies. Such software has been designed, tested,
# and optimized for use with NVIDIA hardware, and this License grants permission to use the software
# solely with such hardware.
# Subject to the terms of this License, NVIDIA confirms that you are free to commercially use,
# modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership of any
# outputs generated using the software or derivative works thereof. Any code contributions that you
# share with NVIDIA are licensed to NVIDIA as feedback under this License and may be incorporated
# in future releases without notice or attribution.
# By using, reproducing, modifying, distributing, performing, or displaying any portion or element
# of the software or derivative works thereof, you agree to be bound by this License.

"""Run cuVSLAM odometry on a ROS2 bag using rosbags."""

from __future__ import annotations

import argparse
import os
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import yaml

import cuvslam
from dataloader import FrameBatch, ImuBatch, RosbagDataLoader, RosbagDataLoaderError

try:
    import rerun as rr
except ImportError:  # pragma: no cover - optional visualization dependency
    rr = None

from rosbags.typesys import Stores

DEFAULT_CONFIG_PATH = Path(__file__).with_name("rosbag_config.yaml")
DEFAULT_TYPESTORE = Stores.ROS2_FOXY

ODOMETRY_MODE_BY_NAME = {
    "MULTICAMERA": cuvslam.Tracker.OdometryMode.Multicamera,
    "INERTIAL": cuvslam.Tracker.OdometryMode.Inertial,
    "RGBD": cuvslam.Tracker.OdometryMode.RGBD,
    "MONO": cuvslam.Tracker.OdometryMode.Mono,
}

DISTORTION_MODEL_BY_NAME = {
    "BROWN": cuvslam.Distortion.Model.Brown,
    "FISHEYE": cuvslam.Distortion.Model.Fisheye,
}


class RosbagTrackerError(RuntimeError):
    """Raised when configuration or bag content is invalid for tracking."""


def _load_yaml_config(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        raise RosbagTrackerError(f"Config file does not exist: {config_path}")

    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict):
        raise RosbagTrackerError("Config file must contain a YAML mapping at the root level.")

    return config


def _resolve_path(base_path: Path, path_value: str) -> Path:
    candidate = Path(path_value)
    if candidate.is_absolute():
        return candidate
    return (base_path / candidate).resolve()


def _get_required_mapping(config: dict[str, Any], key: str) -> dict[str, Any]:
    value = config.get(key)
    if not isinstance(value, dict):
        raise RosbagTrackerError(f"'{key}' section is required and must be a mapping.")
    return value


def _get_required_list(config: dict[str, Any], key: str) -> list[Any]:
    value = config.get(key)
    if not isinstance(value, list):
        raise RosbagTrackerError(f"'{key}' must be a list.")
    return value


def _get_required_pose(pose_cfg: dict[str, Any], key: str) -> cuvslam.Pose:
    translation = pose_cfg.get("translation")
    rotation = pose_cfg.get("rotation")
    if not isinstance(translation, list) or len(translation) != 3:
        raise RosbagTrackerError(f"'{key}.translation' must be a list with 3 elements.")
    if not isinstance(rotation, list) or len(rotation) != 4:
        raise RosbagTrackerError(f"'{key}.rotation' must be a list with 4 elements.")
    return cuvslam.Pose(translation=translation, rotation=rotation)


def _create_camera(camera_cfg: dict[str, Any], camera_index: int) -> cuvslam.Camera:
    size = camera_cfg.get("size")
    focal = camera_cfg.get("focal")
    principal = camera_cfg.get("principal")

    if not isinstance(size, list) or len(size) != 2:
        raise RosbagTrackerError(f"rig.cameras[{camera_index}].size must be [width, height].")
    if not isinstance(focal, list) or len(focal) != 2:
        raise RosbagTrackerError(f"rig.cameras[{camera_index}].focal must be [fx, fy].")
    if not isinstance(principal, list) or len(principal) != 2:
        raise RosbagTrackerError(
            f"rig.cameras[{camera_index}].principal must be [cx, cy]."
        )

    camera = cuvslam.Camera()
    camera.size = size
    camera.focal = focal
    camera.principal = principal

    rig_from_camera_cfg = camera_cfg.get("rig_from_camera")
    if isinstance(rig_from_camera_cfg, dict):
        camera.rig_from_camera = _get_required_pose(
            rig_from_camera_cfg,
            f"rig.cameras[{camera_index}].rig_from_camera",
        )

    distortion_cfg = camera_cfg.get("distortion")
    if isinstance(distortion_cfg, dict):
        model_name = str(distortion_cfg.get("model", "")).strip().upper()
        params = distortion_cfg.get("params", [])
        if model_name not in DISTORTION_MODEL_BY_NAME:
            supported_models = ", ".join(DISTORTION_MODEL_BY_NAME.keys())
            raise RosbagTrackerError(
                f"Unsupported distortion model '{model_name}' in rig.cameras[{camera_index}]. "
                f"Supported values: {supported_models}"
            )
        if not isinstance(params, list):
            raise RosbagTrackerError(
                f"rig.cameras[{camera_index}].distortion.params must be a list."
            )
        camera.distortion = cuvslam.Distortion(DISTORTION_MODEL_BY_NAME[model_name], params)

    camera.border_top = int(camera_cfg.get("border_top", 0))
    camera.border_bottom = int(camera_cfg.get("border_bottom", 0))
    camera.border_left = int(camera_cfg.get("border_left", 0))
    camera.border_right = int(camera_cfg.get("border_right", 0))

    return camera


def _create_imu_calibration(imu_cfg: dict[str, Any], imu_index: int) -> cuvslam.ImuCalibration:
    imu = cuvslam.ImuCalibration()

    rig_from_imu_cfg = imu_cfg.get("rig_from_imu")
    if not isinstance(rig_from_imu_cfg, dict):
        raise RosbagTrackerError(
            f"rig.imus[{imu_index}].rig_from_imu is required and must be a mapping."
        )

    imu.rig_from_imu = _get_required_pose(
        rig_from_imu_cfg,
        f"rig.imus[{imu_index}].rig_from_imu",
    )

    required_fields = [
        "gyroscope_noise_density",
        "gyroscope_random_walk",
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "frequency",
    ]
    for field in required_fields:
        if field not in imu_cfg:
            raise RosbagTrackerError(f"rig.imus[{imu_index}].{field} is required.")

    imu.gyroscope_noise_density = float(imu_cfg["gyroscope_noise_density"])
    imu.gyroscope_random_walk = float(imu_cfg["gyroscope_random_walk"])
    imu.accelerometer_noise_density = float(imu_cfg["accelerometer_noise_density"])
    imu.accelerometer_random_walk = float(imu_cfg["accelerometer_random_walk"])
    imu.frequency = float(imu_cfg["frequency"])

    return imu


def _create_rig(config: dict[str, Any]) -> cuvslam.Rig:
    rig_cfg = _get_required_mapping(config, "rig")
    camera_cfgs = _get_required_list(rig_cfg, "cameras")
    if not camera_cfgs:
        raise RosbagTrackerError("rig.cameras must contain at least one camera.")

    cameras = [
        _create_camera(camera_cfg, camera_index)
        for camera_index, camera_cfg in enumerate(camera_cfgs)
    ]

    rig = cuvslam.Rig()
    rig.cameras = cameras

    imu_cfgs = rig_cfg.get("imus", [])
    if imu_cfgs is None:
        imu_cfgs = []

    if not isinstance(imu_cfgs, list):
        raise RosbagTrackerError("rig.imus must be a list when provided.")

    if imu_cfgs:
        rig.imus = [
            _create_imu_calibration(imu_cfg, imu_index)
            for imu_index, imu_cfg in enumerate(imu_cfgs)
        ]

    return rig


def _create_tracker(config: dict[str, Any], rig: cuvslam.Rig) -> tuple[cuvslam.Tracker, Any]:
    tracker_cfg = _get_required_mapping(config, "tracker")
    mode_name = str(tracker_cfg.get("odometry_mode", "")).strip().upper()
    if mode_name not in ODOMETRY_MODE_BY_NAME:
        valid_modes = ", ".join(ODOMETRY_MODE_BY_NAME.keys())
        raise RosbagTrackerError(
            f"Unsupported tracker.odometry_mode '{mode_name}'. Valid modes: {valid_modes}"
        )

    odometry_mode = ODOMETRY_MODE_BY_NAME[mode_name]

    rgbd_settings = None
    if odometry_mode == cuvslam.Tracker.OdometryMode.RGBD:
        rgbd_cfg = _get_required_mapping(tracker_cfg, "rgbd_settings")
        rgbd_settings = cuvslam.Tracker.OdometryRGBDSettings()
        rgbd_settings.depth_scale_factor = float(rgbd_cfg.get("depth_scale_factor", 1000.0))
        rgbd_settings.depth_camera_id = int(rgbd_cfg.get("depth_camera_id", 0))
        rgbd_settings.enable_depth_stereo_tracking = bool(
            rgbd_cfg.get("enable_depth_stereo_tracking", False)
        )

    odom_cfg_kwargs: dict[str, Any] = {
        "async_sba": bool(tracker_cfg.get("async_sba", False)),
        "enable_observations_export": bool(tracker_cfg.get("enable_observations_export", True)),
        "enable_final_landmarks_export": bool(
            tracker_cfg.get("enable_final_landmarks_export", True)
        ),
        "rectified_stereo_camera": bool(tracker_cfg.get("rectified_stereo_camera", True)),
        "odometry_mode": odometry_mode,
    }
    if rgbd_settings is not None:
        odom_cfg_kwargs["rgbd_settings"] = rgbd_settings

    odom_cfg = cuvslam.Tracker.OdometryConfig(**odom_cfg_kwargs)

    tracker = cuvslam.Tracker(rig, odom_cfg)
    return tracker, odometry_mode


def _parse_visualization_options(config: dict[str, Any]) -> dict[str, Any]:
    visualization_cfg = config.get("visualization", {})
    if visualization_cfg is None:
        visualization_cfg = {}
    if not isinstance(visualization_cfg, dict):
        raise RosbagTrackerError("visualization section must be a mapping when provided.")

    app_name = visualization_cfg.get("app_name", "rosbag_cuvslam")
    if not isinstance(app_name, str) or not app_name:
        raise RosbagTrackerError("visualization.app_name must be a non-empty string.")

    mode = str(visualization_cfg.get("mode", "spawn")).strip().lower()
    if mode not in {"spawn", "connect"}:
        raise RosbagTrackerError("visualization.mode must be either 'spawn' or 'connect'.")

    grpc_url = visualization_cfg.get("grpc_url", "127.0.0.1:9876")
    if not isinstance(grpc_url, str) or not grpc_url:
        raise RosbagTrackerError("visualization.grpc_url must be a non-empty string.")

    options: dict[str, Any] = {
        "enabled": bool(visualization_cfg.get("enabled", True)),
        "app_name": app_name,
        "mode": mode,
        "grpc_url": grpc_url,
        "strict": bool(visualization_cfg.get("strict", True)),
        "spawn": bool(visualization_cfg.get("spawn", True)),
        "log_trajectory": bool(visualization_cfg.get("log_trajectory", True)),
        "log_pose": bool(visualization_cfg.get("log_pose", True)),
        "log_pose_axes": bool(visualization_cfg.get("log_pose_axes", True)),
        "log_images": bool(visualization_cfg.get("log_images", True)),
        "log_observations": bool(visualization_cfg.get("log_observations", True)),
        "log_depth": bool(visualization_cfg.get("log_depth", True)),
    }
    return options


def _init_visualization_if_enabled(visualization_options: dict[str, Any]) -> bool:
    enabled = visualization_options["enabled"]
    if not enabled:
        return False

    if rr is None:
        raise RosbagTrackerError(
            "visualization.enabled is true but rerun is not installed. "
            "Install rerun-sdk or set visualization.enabled to false."
        )

    if visualization_options["mode"] == "connect":
        rr.init(
            visualization_options["app_name"],
            strict=visualization_options["strict"],
            spawn=False,
        )
        grpc_url = visualization_options["grpc_url"]
        if grpc_url.startswith(("http://", "https://")):
            # Rerun proxy endpoints commonly use HTTP URLs and require the rerun+ prefix.
            grpc_url = f"rerun+{grpc_url}"
        rr.connect_grpc(grpc_url)
    else:
        rr.init(
            visualization_options["app_name"],
            strict=visualization_options["strict"],
            spawn=visualization_options["spawn"],
        )
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
    return True


def _log_visualization(
    frame_id: int,
    odom_pose: Any,
    trajectory: list[list[float]],
    image_topic_order: list[str],
    frame_bundle: dict[str, np.ndarray],
    observations: list[Any],
    depth_topic: str | None,
    visualization_options: dict[str, Any],
    visualization_stats: dict[str, int],
) -> None:
    if rr is None:
        return

    rr.set_time("frame", sequence=frame_id)
    visualization_stats["frames_visualized"] += 1
    if visualization_options["log_trajectory"]:
        rr.log("trajectory", rr.LineStrips3D(trajectory), static=True)
        visualization_stats["log_trajectory_calls"] += 1

    if visualization_options["log_pose"]:
        rr.log(
            "world/camera_0",
            rr.Transform3D(
                translation=odom_pose.translation,
                quaternion=odom_pose.rotation,
            ),
        )
        visualization_stats["log_pose_calls"] += 1

    if visualization_options["log_pose_axes"]:
        rr.log(
            "world/camera_0/axes",
            rr.Arrows3D(
                vectors=np.eye(3) * 0.2,
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            ),
        )
        visualization_stats["log_pose_axes_calls"] += 1

    if visualization_options["log_images"]:
        for camera_index, topic_name in enumerate(image_topic_order):
            image = frame_bundle[topic_name]
            rr.log(f"world/camera_{camera_index}/image", rr.Image(image))
            visualization_stats["log_images_calls"] += 1

    if visualization_options["log_observations"] and observations:
        obs_uv = [[obs.u, obs.v] for obs in observations]
        obs_colors = [
            [
                (int(obs.id) * 17) % 256,
                (int(obs.id) * 31) % 256,
                (int(obs.id) * 47) % 256,
            ]
            for obs in observations
        ]
        rr.log(
            "world/camera_0/image/observations",
            rr.Points2D(obs_uv, radii=5, colors=obs_colors),
        )
        if visualization_options["log_depth"] and depth_topic is not None:
            rr.log(
                "world/camera_0/depth/observations",
                rr.Points2D(obs_uv, radii=5, colors=obs_colors),
            )
        visualization_stats["log_observations_calls"] += 1
    elif visualization_options["log_observations"]:
        visualization_stats["log_observations_empty_frames"] += 1

    if visualization_options["log_depth"] and depth_topic is not None:
        depth_frame = frame_bundle[depth_topic]
        if np.issubdtype(depth_frame.dtype, np.floating) or depth_frame.dtype == np.uint16:
            rr.log("world/camera_0/depth", rr.DepthImage(depth_frame))
        else:
            rr.log("world/camera_0/depth", rr.Image(depth_frame))
        visualization_stats["log_depth_calls"] += 1


def _save_trajectory(
    output_path: Path,
    trajectory_rows: list[tuple[int, list[float], list[float]]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as output_file:
        for timestamp_ns, translation, rotation in trajectory_rows:
            output_file.write(
                f"{timestamp_ns} "
                f"{translation[0]:.9f} {translation[1]:.9f} {translation[2]:.9f} "
                f"{rotation[0]:.9f} {rotation[1]:.9f} {rotation[2]:.9f} {rotation[3]:.9f}\n"
            )


def _validate_topics_and_mode(
    mode: Any,
    image_topics: list[str],
    depth_topic: str | None,
    imu_topic: str | None,
    rig: cuvslam.Rig,
) -> None:
    if len(image_topics) != len(rig.cameras):
        raise RosbagTrackerError(
            "topics.image_topics size must match rig.cameras size. "
            f"Got {len(image_topics)} image topics and {len(rig.cameras)} cameras."
        )

    if mode == cuvslam.Tracker.OdometryMode.RGBD:
        if len(image_topics) != 1:
            raise RosbagTrackerError("RGBD mode requires exactly one image topic.")
        if not depth_topic:
            raise RosbagTrackerError("RGBD mode requires topics.depth_topic.")

    if mode == cuvslam.Tracker.OdometryMode.Inertial:
        if not imu_topic:
            raise RosbagTrackerError("Inertial mode requires topics.imu_topic.")
        if not getattr(rig, "imus", []):
            raise RosbagTrackerError("Inertial mode requires at least one IMU in rig.imus.")


def run_tracking(config_path: Path) -> None:
    config = _load_yaml_config(config_path)

    rosbag_cfg = _get_required_mapping(config, "rosbag")
    topics_cfg = _get_required_mapping(config, "topics")

    bag_path_value = rosbag_cfg.get("bag_path")
    if not isinstance(bag_path_value, str) or not bag_path_value:
        raise RosbagTrackerError("rosbag.bag_path is required and must be a non-empty string.")

    bag_path = _resolve_path(config_path.parent, bag_path_value)
    if not bag_path.exists():
        raise RosbagTrackerError(f"ROS2 bag path does not exist: {bag_path}")

    typestore_name = str(rosbag_cfg.get("typestore", DEFAULT_TYPESTORE.name)).strip().upper()
    if typestore_name not in Stores.__members__:
        valid_typestores = ", ".join(sorted(Stores.__members__.keys()))
        raise RosbagTrackerError(
            f"Unknown rosbag.typestore '{typestore_name}'. Valid values: {valid_typestores}"
        )

    sync_tolerance_ns = int(float(rosbag_cfg.get("sync_tolerance_ms", 10.0)) * 1e6)
    if sync_tolerance_ns < 0:
        raise RosbagTrackerError("rosbag.sync_tolerance_ms must be non-negative.")
    depth_float_to_uint16_scale = float(rosbag_cfg.get("depth_float_to_uint16_scale", 1000.0))
    if depth_float_to_uint16_scale <= 0:
        raise RosbagTrackerError("rosbag.depth_float_to_uint16_scale must be positive.")
    max_frames_cfg = rosbag_cfg.get("max_frames")
    max_frames: int | None = None
    if max_frames_cfg is not None:
        max_frames = int(max_frames_cfg)
        if max_frames <= 0:
            raise RosbagTrackerError("rosbag.max_frames must be positive when provided.")

    image_topics = topics_cfg.get("image_topics")
    if not isinstance(image_topics, list) or not image_topics:
        raise RosbagTrackerError("topics.image_topics must be a non-empty list of topic names.")
    if not all(isinstance(topic, str) and topic for topic in image_topics):
        raise RosbagTrackerError("topics.image_topics must contain non-empty strings.")

    depth_topic = topics_cfg.get("depth_topic")
    if depth_topic is not None and not isinstance(depth_topic, str):
        raise RosbagTrackerError("topics.depth_topic must be a string when provided.")

    imu_topic = topics_cfg.get("imu_topic")
    if imu_topic is not None and not isinstance(imu_topic, str):
        raise RosbagTrackerError("topics.imu_topic must be a string when provided.")

    rig = _create_rig(config)
    tracker, odometry_mode = _create_tracker(config, rig)
    _validate_topics_and_mode(odometry_mode, image_topics, depth_topic, imu_topic, rig)

    tracker_cfg = _get_required_mapping(config, "tracker")
    visualization_options = _parse_visualization_options(config)

    if visualization_options["enabled"]:
        if visualization_options["log_depth"] and depth_topic is None:
            print(
                "Warning: visualization.log_depth=true but topics.depth_topic is not set. "
                "Depth logging will be disabled."
            )
            visualization_options["log_depth"] = False

        if visualization_options["log_observations"] and not bool(
            tracker_cfg.get("enable_observations_export", True)
        ):
            print(
                "Warning: visualization.log_observations=true but "
                "tracker.enable_observations_export=false. Observations logging will be disabled."
            )
            visualization_options["log_observations"] = False

        if visualization_options["log_observations"] and not visualization_options["log_images"]:
            print(
                "Note: visualization.log_observations=true while visualization.log_images=false. "
                "Observation points will be logged without image overlay."
            )

    visualization_stats: dict[str, int] = {
        "frames_visualized": 0,
        "log_trajectory_calls": 0,
        "log_pose_calls": 0,
        "log_pose_axes_calls": 0,
        "log_images_calls": 0,
        "log_observations_calls": 0,
        "log_observations_empty_frames": 0,
        "log_depth_calls": 0,
    }

    visualization_enabled = _init_visualization_if_enabled(visualization_options)

    output_cfg = config.get("output", {})
    trajectory_path_value = output_cfg.get("trajectory_path")
    trajectory_path = (
        _resolve_path(config_path.parent, trajectory_path_value)
        if isinstance(trajectory_path_value, str)
        else None
    )

    data_loader = RosbagDataLoader(
        bag_path=bag_path,
        typestore_name=typestore_name,
        image_topics=image_topics,
        depth_topic=depth_topic,
        imu_topic=imu_topic,
        sync_tolerance_ns=sync_tolerance_ns,
    )

    trajectory_rows: list[tuple[int, list[float], list[float]]] = []
    trajectory_points: list[list[float]] = []
    tracked_frames = 0
    failed_frames = 0
    imu_messages_read = 0
    imu_messages_registered = 0
    discarded_out_of_order_imu_messages = 0
    discarded_out_of_order_frames = 0
    last_tracked_timestamp_ns: int | None = None
    last_api_timestamp_ns: int | None = None
    imu_buffer: deque[tuple[int, cuvslam.ImuMeasurement]] = deque()

    print("Tracking with topic mapping:")
    for topic in data_loader.required_topics:
        print(f"  - {topic}")

    try:
        for event in data_loader.iter_events():
            if isinstance(event, ImuBatch):
                imu_messages_read += 1
                imu_measurement = cuvslam.ImuMeasurement()
                imu_measurement.timestamp_ns = event.timestamp_ns
                imu_measurement.linear_accelerations = event.linear_accelerations
                imu_measurement.angular_velocities = event.angular_velocities
                imu_buffer.append((event.timestamp_ns, imu_measurement))
                continue

            if not isinstance(event, FrameBatch):
                continue

            synced_timestamp_ns = event.timestamp_ns
            frame_bundle = event.frame_bundle

            if (
                last_tracked_timestamp_ns is not None
                and synced_timestamp_ns <= last_tracked_timestamp_ns
            ):
                discarded_out_of_order_frames += 1
                continue

            images = [frame_bundle[topic] for topic in image_topics]
            depths = [frame_bundle[depth_topic]] if depth_topic is not None else None

            while imu_buffer and imu_buffer[0][0] < synced_timestamp_ns:
                imu_timestamp_ns, imu_measurement = imu_buffer.popleft()
                if last_api_timestamp_ns is not None and imu_timestamp_ns <= last_api_timestamp_ns:
                    discarded_out_of_order_imu_messages += 1
                    continue
                tracker.register_imu_measurement(0, imu_measurement)
                imu_messages_registered += 1
                last_api_timestamp_ns = imu_timestamp_ns

            if last_api_timestamp_ns is not None and synced_timestamp_ns <= last_api_timestamp_ns:
                discarded_out_of_order_frames += 1
                continue

            if depths is not None and depths[0].dtype == np.float32:
                depth_float = np.nan_to_num(depths[0], nan=0.0, posinf=0.0, neginf=0.0)
                depth_uint16 = np.clip(
                    depth_float * depth_float_to_uint16_scale,
                    0,
                    np.iinfo(np.uint16).max,
                ).astype(np.uint16)
                depths = [np.ascontiguousarray(depth_uint16)]

            odom_pose_estimate, _ = tracker.track(
                synced_timestamp_ns,
                images=images,
                depths=depths,
            )
            last_api_timestamp_ns = synced_timestamp_ns

            if odom_pose_estimate.world_from_rig is None:
                failed_frames += 1
                continue

            odom_pose = odom_pose_estimate.world_from_rig.pose
            trajectory_points.append(list(odom_pose.translation))
            trajectory_rows.append(
                (
                    int(synced_timestamp_ns),
                    list(odom_pose.translation),
                    list(odom_pose.rotation),
                )
            )

            observations = tracker.get_last_observations(0)
            if visualization_enabled:
                _log_visualization(
                    tracked_frames,
                    odom_pose,
                    trajectory_points,
                    image_topics,
                    frame_bundle,
                    observations,
                    depth_topic,
                    visualization_options,
                    visualization_stats,
                )

            tracked_frames += 1
            last_tracked_timestamp_ns = synced_timestamp_ns

            if max_frames is not None and tracked_frames >= max_frames:
                break
    except RosbagDataLoaderError as error:
        raise RosbagTrackerError(str(error)) from error

    print("Tracking summary:")
    print(f"  tracked_frames: {tracked_frames}")
    print(f"  failed_frames: {failed_frames}")
    print(f"  dropped_unsynced_reference_frames: {data_loader.stats.dropped_reference_frames}")
    print(f"  discarded_out_of_order_frames: {discarded_out_of_order_frames}")
    print(f"  discarded_out_of_order_imu_messages: {discarded_out_of_order_imu_messages}")
    print(f"  imu_messages_read: {imu_messages_read}")
    print(f"  imu_messages_registered: {imu_messages_registered}")

    if visualization_enabled:
        print("Visualization summary:")
        print(f"  mode: {visualization_options['mode']}")
        print(f"  frames_visualized: {visualization_stats['frames_visualized']}")
        print(f"  log_trajectory_calls: {visualization_stats['log_trajectory_calls']}")
        print(f"  log_pose_calls: {visualization_stats['log_pose_calls']}")
        print(f"  log_pose_axes_calls: {visualization_stats['log_pose_axes_calls']}")
        print(f"  log_images_calls: {visualization_stats['log_images_calls']}")
        print(f"  log_observations_calls: {visualization_stats['log_observations_calls']}")
        print(
            "  log_observations_empty_frames: "
            f"{visualization_stats['log_observations_empty_frames']}"
        )
        print(f"  log_depth_calls: {visualization_stats['log_depth_calls']}")

    if trajectory_path is not None:
        _save_trajectory(trajectory_path, trajectory_rows)
        print(f"Saved trajectory to {trajectory_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Track a ROS2 bag using cuVSLAM with YAML configuration.",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to YAML config file.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(os.path.expanduser(args.config)).resolve()
    run_tracking(config_path)


if __name__ == "__main__":
    main()
