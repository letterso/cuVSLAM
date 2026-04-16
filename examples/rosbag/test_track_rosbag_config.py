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

"""Unit tests for rosbag source configuration parsing."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROSBAG_EXAMPLE_DIR = Path(__file__).resolve().parent
if str(ROSBAG_EXAMPLE_DIR) not in sys.path:
    sys.path.insert(0, str(ROSBAG_EXAMPLE_DIR))

import track_rosbag


class TestRosbagSourceConfig(unittest.TestCase):
    def test_ros1_typestore_infers_ros1_format(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            base_path = Path(temporary_directory)
            bag_path = base_path / "sample_ros1.bag"
            bag_path.touch()

            parsed = track_rosbag._parse_rosbag_source_config(
                config_path=base_path / "config.yaml",
                rosbag_cfg={
                    "bag_path": str(bag_path),
                    "typestore": "ROS1_NOETIC",
                },
            )

            self.assertEqual(parsed.bag_format, "ros1")
            self.assertEqual(parsed.typestore_name, "ROS1_NOETIC")

    def test_ros2_typestore_infers_ros2_format(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            base_path = Path(temporary_directory)
            bag_directory = base_path / "dataset" / "ros2_bag"
            bag_directory.mkdir(parents=True)

            parsed = track_rosbag._parse_rosbag_source_config(
                config_path=base_path / "config.yaml",
                rosbag_cfg={
                    "bag_path": "dataset/ros2_bag",
                    "typestore": "ROS2_FOXY",
                },
            )

            self.assertEqual(parsed.bag_format, "ros2")
            self.assertEqual(parsed.typestore_name, "ROS2_FOXY")

    def test_unknown_typestore_raises_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            base_path = Path(temporary_directory)
            bag_path = base_path / "sample_ros1.bag"
            bag_path.touch()

            with self.assertRaises(track_rosbag.RosbagTrackerError):
                track_rosbag._parse_rosbag_source_config(
                    config_path=base_path / "config.yaml",
                    rosbag_cfg={
                        "bag_path": str(bag_path),
                        "typestore": "ROS2_FAKE",
                    },
                )

    def test_missing_typestore_defaults_to_ros2_foxy(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            base_path = Path(temporary_directory)
            bag_directory = base_path / "dataset" / "ros2_bag"
            bag_directory.mkdir(parents=True)

            parsed = track_rosbag._parse_rosbag_source_config(
                config_path=base_path / "config.yaml",
                rosbag_cfg={
                    "bag_path": "dataset/ros2_bag",
                },
            )

            self.assertEqual(parsed.bag_format, "ros2")
            self.assertEqual(parsed.typestore_name, "ROS2_FOXY")


if __name__ == "__main__":
    unittest.main()
