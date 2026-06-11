import logging
import pathlib

import pydantic
from rosbags import highlevel
from rosbags.typesys import (
    get_typestore,
    Stores,
)
from rosbags.typesys.store import Typestore


ROS_TYPESTORES = {
    "empty": Stores.EMPTY,
    "latest": Stores.LATEST,
    "noetic": Stores.ROS1_NOETIC,
    "dashing": Stores.ROS2_DASHING,
    "eloquent": Stores.ROS2_ELOQUENT,
    "foxy": Stores.ROS2_FOXY,
    "galactic": Stores.ROS2_GALACTIC,
    "humble": Stores.ROS2_HUMBLE,
    "iron": Stores.ROS2_IRON,
    "jazzy": Stores.ROS2_JAZZY,
    "kilted": Stores.ROS2_KILTED,
}


class Config(pydantic.BaseModel):
    """Configuration for the bag to edex converter."""

    # Path of the rosbag used for extraction.
    rosbag_path: pathlib.Path
    # Path of the generated edex, urdf, images, videos, etc.
    output_path: pathlib.Path
    # Topics used to get the camera's intrinsics (and extrinsics if frames are not set explicitly).
    camera_info_topics: list[str]
    # Topics used to extract images. Must be the same length as camera_info_topics.
    image_topics: list[str]
    # Topic used to get IMU measurements.
    imu_topic: str | None = None
    # Frames used to acquire the extrinsics. If not set the frames from the messages will be used:
    rig_frame: str
    camera_optical_frames: list[str] | None = None
    imu_frame: str | None = None
    # Number of workers used in image extraction.
    num_workers: int = -1
    # Threshold used for syncing images in the same frame.
    sync_threshold_ns: int = int(0.001 * 10**9)
    # Width and height used to resize the extracted images.
    output_width: int | None = None
    output_height: int | None = None
    output_format: str | None = None
    # ROS distribution used to extract the rosbag.
    ros_distribution: str = "humble"

    @pydantic.model_validator(mode="after")
    def check_fields(self):
        """Preprocess the values and then validate that all members are valid."""
        if not self.rosbag_path.exists():
            raise ValueError(f"Path '{self.rosbag_path}' does not exist")
        if len(self.image_topics) != len(self.camera_info_topics):
            raise ValueError("Need same number of image topics as camera info topics.")
        if self.camera_optical_frames:
            if len(self.camera_optical_frames) != len(self.camera_info_topics):
                raise ValueError(
                    "Need same number of camera optical frames as camera info topics."
                )
        return self


def get_typestore_from_ros_distribution(ros_distribution: str) -> Typestore:
    """Get the typestore from the ROS distribution."""
    if ros_distribution not in ROS_TYPESTORES:
        raise ValueError(f"Unknown ROS distribution: {ros_distribution}")
    return get_typestore(ROS_TYPESTORES[ros_distribution])


def get_first_message(reader: highlevel.AnyReader, topics: list[str]) -> list[object]:
    """Get the first message of every topic."""
    connections = [c for c in reader.connections if c.topic in topics]
    topic_and_first_msg = {}
    for connection, _, rawdata in reader.messages(connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        topic_and_first_msg[connection.topic] = msg
        if len(topic_and_first_msg) == len(topics):
            break

    # Raise a clear error for any topic that had no messages.
    missing = [t for t in topics if t not in topic_and_first_msg]
    if missing:
        raise ValueError(
            f"get_first_message: no messages found for topic(s) {missing}. "
            "Cannot build camera/IMU metadata. Check that the topic names in "
            "your config match those in the bag (run 'ros2 bag info <bag>' to list topics)."
        )

    # Generate the list in the same order as the input topics.
    return [topic_and_first_msg[topic] for topic in topics]


def log_rosbag_info(reader: highlevel.AnyReader):
    """Log the topics and message types of all message channels in the rosbag."""
    logs = [f"\t- {c.topic}: {c.msgtype}" for c in reader.connections]
    logs = sorted(logs)
    # pylint: disable=logging-not-lazy
    logging.info("Found the following topics in rosbag:\n" + "\n".join(logs))
