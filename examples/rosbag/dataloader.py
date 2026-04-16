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

"""ROS bag dataloader utilities for synchronized image/depth and IMU streams."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


class RosbagDataLoaderError(RuntimeError):
    """Raised when ROS bag loading or decoding fails."""


@dataclass
class ImuBatch:
    """One IMU measurement sample from ROS bag."""

    timestamp_ns: int
    linear_accelerations: np.ndarray
    angular_velocities: np.ndarray


@dataclass
class FrameBatch:
    """One synchronized frame bundle for image topics (and optional depth topic)."""

    timestamp_ns: int
    frame_bundle: dict[str, np.ndarray]


@dataclass
class RosbagLoaderStats:
    """Basic loading statistics for diagnostics."""

    dropped_reference_frames: int = 0
    imu_messages_read: int = 0
    image_messages_read: int = 0
    synced_frames: int = 0


class _SyncState:
    """Internal synchronization queues for image/depth streams."""

    def __init__(self, reference_topic: str, secondary_topics: list[str]) -> None:
        self.reference_topic = reference_topic
        self.secondary_topics = secondary_topics
        self.pending_reference: deque[tuple[int, np.ndarray]] = deque()
        self.secondary_queues: dict[str, deque[tuple[int, np.ndarray]]] = {
            topic: deque() for topic in secondary_topics
        }
        self.dropped_reference_frames = 0


class RosbagDataLoader:
    """Iterate over a ROS2 bag and emit IMU or synchronized frame batches."""

    def __init__(
        self,
        bag_path: Path,
        typestore_name: str,
        image_topics: list[str],
        depth_topic: str | None,
        imu_topic: str | None,
        sync_tolerance_ns: int,
    ) -> None:
        if not bag_path.exists():
            raise RosbagDataLoaderError(f"ROS bag path does not exist: {bag_path}")
        if not image_topics:
            raise RosbagDataLoaderError("image_topics must be a non-empty list.")
        if sync_tolerance_ns < 0:
            raise RosbagDataLoaderError("sync_tolerance_ns must be non-negative.")

        typestore_key = typestore_name.strip().upper()
        if typestore_key not in Stores.__members__:
            valid_typestores = ", ".join(sorted(Stores.__members__.keys()))
            raise RosbagDataLoaderError(
                f"Unknown typestore '{typestore_name}'. Valid values: {valid_typestores}"
            )

        self._bag_path = bag_path
        self._typestore_key = typestore_key
        self._image_topics = image_topics
        self._depth_topic = depth_topic
        self._imu_topic = imu_topic
        self._sync_tolerance_ns = sync_tolerance_ns

        self._reference_topic = image_topics[0]
        self._secondary_topics = list(image_topics[1:])
        if depth_topic is not None:
            self._secondary_topics.append(depth_topic)

        required_topics = list(image_topics)
        if depth_topic is not None:
            required_topics.append(depth_topic)
        if imu_topic is not None:
            required_topics.append(imu_topic)
        self.required_topics = required_topics

        self.stats = RosbagLoaderStats()

    def iter_events(self) -> Iterator[ImuBatch | FrameBatch]:
        """Yield IMU samples and synchronized image/depth frame bundles."""
        typestore = get_typestore(Stores[self._typestore_key])
        sync_state = _SyncState(self._reference_topic, self._secondary_topics)

        try:
            with AnyReader([self._bag_path], default_typestore=typestore) as reader:
                connections_by_topic = self._create_connections_by_topic(reader, self.required_topics)
                selected_connections = [connections_by_topic[topic] for topic in self.required_topics]

                for connection, recorded_timestamp_ns, raw_data in reader.messages(
                    connections=selected_connections
                ):
                    message = reader.deserialize(raw_data, connection.msgtype)
                    topic_name = connection.topic

                    if self._imu_topic is not None and topic_name == self._imu_topic:
                        imu = _decode_imu_measurement(message, recorded_timestamp_ns)
                        self.stats.imu_messages_read += 1
                        yield imu
                        continue

                    frame_timestamp_ns = _get_timestamp_ns(message, recorded_timestamp_ns)
                    frame = _decode_image(message, topic_name)
                    if topic_name != self._depth_topic:
                        frame = _normalize_camera_image_for_tracker(frame, topic_name)
                    self.stats.image_messages_read += 1

                    if topic_name == self._reference_topic:
                        sync_state.pending_reference.append((frame_timestamp_ns, frame))
                    else:
                        sync_state.secondary_queues[topic_name].append((frame_timestamp_ns, frame))

                    ready_frames = _pop_synced_frames(sync_state, self._sync_tolerance_ns)
                    for synced_timestamp_ns, frame_bundle in ready_frames:
                        self.stats.synced_frames += 1
                        yield FrameBatch(synced_timestamp_ns, frame_bundle)
        finally:
            self.stats.dropped_reference_frames = sync_state.dropped_reference_frames

    @staticmethod
    def _create_connections_by_topic(reader: AnyReader, topics: list[str]) -> dict[str, Any]:
        connections_by_topic: dict[str, Any] = {}
        for connection in reader.connections:
            if connection.topic in topics and connection.topic not in connections_by_topic:
                connections_by_topic[connection.topic] = connection

        missing_topics = [topic for topic in topics if topic not in connections_by_topic]
        if missing_topics:
            missing = ", ".join(missing_topics)
            raise RosbagDataLoaderError(f"Requested topics are missing in bag: {missing}")

        return connections_by_topic


def _get_timestamp_ns(message: Any, fallback_timestamp_ns: int) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return int(fallback_timestamp_ns)
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _decode_image(message: Any, topic_name: str) -> np.ndarray:
    encoding = str(getattr(message, "encoding", "")).lower()
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    is_bigendian = bool(message.is_bigendian)

    if width <= 0 or height <= 0:
        raise RosbagDataLoaderError(
            f"Invalid image size from topic '{topic_name}': {width}x{height}"
        )

    if encoding in {"mono8", "8uc1"}:
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
        return np.ascontiguousarray(row[:, :width])

    if encoding in {"rgb8", "bgr8"}:
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
        image = row[:, : width * 3].reshape(height, width, 3)
        if encoding == "rgb8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    if encoding in {"mono16", "16uc1"}:
        dtype = np.dtype(">u2") if is_bigendian else np.dtype("<u2")
        row = np.frombuffer(message.data, dtype=dtype).reshape(height, step // 2)
        image = row[:, :width]
        if is_bigendian:
            image = image.byteswap().view(image.dtype.newbyteorder("="))
        return np.ascontiguousarray(image)

    if encoding == "32fc1":
        row = np.frombuffer(message.data, dtype=np.float32).reshape(height, step // 4)
        image = row[:, :width]
        return np.ascontiguousarray(image)

    raise RosbagDataLoaderError(
        f"Unsupported encoding '{encoding}' on topic '{topic_name}'. "
        "Supported: mono8, 8UC1, rgb8, bgr8, mono16, 16UC1, 32FC1."
    )


def _decode_imu_measurement(message: Any, fallback_timestamp_ns: int) -> ImuBatch:
    return ImuBatch(
        timestamp_ns=_get_timestamp_ns(message, fallback_timestamp_ns),
        linear_accelerations=np.asarray(
            [
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ]
        ),
        angular_velocities=np.asarray(
            [
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ]
        ),
    )


def _normalize_camera_image_for_tracker(image: np.ndarray, topic_name: str) -> np.ndarray:
    if image.dtype == np.uint8:
        return np.ascontiguousarray(image)

    if image.dtype == np.uint16:
        # Preserve relative brightness while mapping 16-bit intensity into 8-bit range.
        return np.ascontiguousarray((image >> 8).astype(np.uint8))

    if np.issubdtype(image.dtype, np.floating):
        finite = np.nan_to_num(image, nan=0.0, posinf=0.0, neginf=0.0)
        max_value = float(np.max(finite)) if finite.size else 0.0
        if max_value <= 1.0:
            scaled = np.clip(finite * 255.0, 0.0, 255.0)
        else:
            scaled = np.clip(finite, 0.0, 255.0)
        return np.ascontiguousarray(scaled.astype(np.uint8))

    raise RosbagDataLoaderError(
        f"Unsupported image dtype '{image.dtype}' for tracker input on topic '{topic_name}'."
    )


def _pop_synced_frames(
    sync_state: _SyncState,
    sync_tolerance_ns: int,
) -> list[tuple[int, dict[str, np.ndarray]]]:
    ready_frames: list[tuple[int, dict[str, np.ndarray]]] = []

    while sync_state.pending_reference:
        reference_timestamp_ns, reference_frame = sync_state.pending_reference[0]
        frame_bundle: dict[str, np.ndarray] = {sync_state.reference_topic: reference_frame}
        matched_candidates: dict[str, tuple[int, int, np.ndarray]] = {}
        drop_reference = False

        for secondary_topic in sync_state.secondary_topics:
            queue = sync_state.secondary_queues[secondary_topic]

            while queue and queue[0][0] < reference_timestamp_ns - sync_tolerance_ns:
                queue.popleft()

            if not queue:
                break

            best_index = -1
            best_delta = 2**63 - 1
            for index, (candidate_timestamp_ns, candidate_frame) in enumerate(queue):
                if candidate_timestamp_ns > reference_timestamp_ns + sync_tolerance_ns:
                    break
                delta = abs(candidate_timestamp_ns - reference_timestamp_ns)
                if delta < best_delta:
                    best_delta = delta
                    best_index = index

            if best_index < 0:
                if queue[0][0] > reference_timestamp_ns + sync_tolerance_ns:
                    drop_reference = True
                break

            matched_timestamp_ns, matched_frame = queue[best_index]
            matched_candidates[secondary_topic] = (best_index, matched_timestamp_ns, matched_frame)

        if drop_reference:
            sync_state.pending_reference.popleft()
            sync_state.dropped_reference_frames += 1
            continue

        if len(matched_candidates) != len(sync_state.secondary_topics):
            break

        sync_state.pending_reference.popleft()

        for secondary_topic in sync_state.secondary_topics:
            matched_index, _matched_timestamp_ns, matched_frame = matched_candidates[secondary_topic]
            queue = sync_state.secondary_queues[secondary_topic]
            for _ in range(matched_index + 1):
                queue.popleft()
            frame_bundle[secondary_topic] = matched_frame

        ready_frames.append((reference_timestamp_ns, frame_bundle))

    return ready_frames
