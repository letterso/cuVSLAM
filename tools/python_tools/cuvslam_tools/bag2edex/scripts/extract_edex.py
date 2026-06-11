"""Extract an EDEX dataset from a rosbag.

The generated EDEX dataset contains images, frame metadata, EDEX metadata, and URDF.
"""

import argparse
import logging
import os
import pathlib

import yaml

from cuvslam_tools.bag2edex import (
    Config,
    extract_edex,
)


def main():
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--config_path",
        type=pathlib.Path,
        required=True,
        help="Path to the config file.",
    )
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
        help="Path where the EDEX dataset is generated.",
    )

    args = parser.parse_args()

    assert (
        args.config_path.exists()
    ), f"Config path '{args.config_path}' does not exist."
    yaml_string = args.config_path.read_text()
    yaml_dict = yaml.safe_load(yaml_string)

    # We override some arguments from the CLI.
    if "rosbag_path" in yaml_dict:
        logging.warning("rosbag_path in the config is overridden by --rosbag_path.")
    if "output_path" in yaml_dict:
        logging.warning("output_path in the config is overridden by --output_path.")
    yaml_dict["rosbag_path"] = args.rosbag_path.absolute()
    yaml_dict["output_path"] = args.output_path.absolute()
    config = Config(**yaml_dict)

    extract_edex(config)


if __name__ == "__main__":
    main()
