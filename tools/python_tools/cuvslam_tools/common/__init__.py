"""
Common utilities shared between tracker and undistort tools.

This module provides shared functionality including:
- camera: Camera model implementations (pinhole, fisheye, brown5k, etc.)
- conversions: Conversion utilities between different formats
- dataset_reader: Base dataset reader class
- edex_reader: EDEX dataset reader implementation
- video_reader: Video dataset reader implementation
"""

__all__ = [
    "camera",
    "conversions",
    "dataset_reader",
    "edex_reader",
    "video_reader",
]
