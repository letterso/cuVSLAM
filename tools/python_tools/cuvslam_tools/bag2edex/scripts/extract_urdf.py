"""Extract a URDF from the /tf_static topic in a rosbag.

The generated URDF is minimal and only contains transforms. Physical parameters like mass, inertia
etc. are not contained.
"""

import argparse
import logging
import pathlib

from cuvslam_tools.bag2edex import rosbag_urdf_extraction


def main():
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-r",
        "--rosbag_path",
        type=pathlib.Path,
        required=True,
        help="Path to the input rosbag directory.",
    )
    parser.add_argument(
        "-o",
        "--output_path",
        type=pathlib.Path,
        required=True,
        help="Path where the URDF is generated.",
    )
    parser.add_argument(
        "-d",
        "--ros_distribution",
        type=str,
        required=False,
        default="humble",
        help="ROS distribution",
    )
    args = parser.parse_args()

    rosbag_urdf_extraction.extract_urdf(
        args.rosbag_path,
        args.ros_distribution,
        args.output_path,
    )


if __name__ == "__main__":
    main()
