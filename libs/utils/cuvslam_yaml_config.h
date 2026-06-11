/*
 * Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA software released under the NVIDIA Community License is intended to be used to enable
 * the further development of AI and robotics technologies. Such software has been designed, tested,
 * and optimized for use with NVIDIA hardware, and this License grants permission to use the software
 * solely with such hardware.
 * Subject to the terms of this License, NVIDIA confirms that you are free to commercially use,
 * modify, and distribute the software or derivative works thereof. Any code contributions that you
 * share with NVIDIA are licensed to NVIDIA as feedback under this License and may be incorporated
 * in future releases without notice or attribution.
 * By using, reproducing, modifying, distributing, performing, or displaying any portion or element
 * of the software or derivative works thereof, you agree to be bound by this License.
 */

#pragma once

#include "cuvslam/cuvslam2.h"

namespace cuvslam {

/**
 * @file cuvslam_yaml_config.h
 * @brief Optional YAML file loaders.
 *
 * Implemented in the `utils` static library under libs/utils (links yaml-cpp). Tools and
 * bindings should link `utils` alongside libcuvslam if they want file-based config.
 */

/// @brief Load per-frame track options from the `track_options:` section of a YAML file.
/// @param filepath Path to the YAML file.
/// @param options Output TrackOptions to populate.
/// @return true if the `track_options` section was found and applied, false if the section is absent.
/// @throws std::runtime_error if the file cannot be opened or parsed.
bool LoadTrackOptionsFromFile(const char* filepath, Odometry::TrackOptions& options);

/// @brief Load odometry configuration from the `odometry:` section of a YAML file.
/// @param filepath Path to the YAML file.
/// @param config Output Config to populate.
/// @return true if the `odometry` section was found and applied, false if the section is absent.
/// @throws std::runtime_error if the file cannot be opened or parsed, or if `multicam_mode` /
///         `odometry_mode` contains an unrecognised string value.
bool LoadOdometryConfigFromFile(const char* filepath, Odometry::Config& config);

/// @brief Load SLAM configuration from the `slam:` section of a YAML file.
/// @param filepath Path to the YAML file.
/// @param config Output Config to populate.
/// @return true if the `slam` section was found and applied, false if the section is absent.
/// @throws std::runtime_error if the file cannot be opened or parsed.
bool LoadSlamConfigFromFile(const char* filepath, Slam::Config& config);

}  // namespace cuvslam
