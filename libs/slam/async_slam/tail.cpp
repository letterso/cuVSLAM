
/*
 * Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA software released under the NVIDIA Community License is intended to be used to enable
 * the further development of AI and robotics technologies. Such software has been designed, tested,
 * and optimized for use with NVIDIA hardware, and this License grants permission to use the software
 * solely with such hardware.
 * Subject to the terms of this License, NVIDIA confirms that you are free to commercially use,
 * modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership of any
 * outputs generated using the software or derivative works thereof. Any code contributions that you
 * share with NVIDIA are licensed to NVIDIA as feedback under this License and may be incorporated
 * in future releases without notice or attribution.
 * By using, reproducing, modifying, distributing, performing, or displaying any portion or element
 * of the software or derivative works thereof, you agree to be bound by this License.
 */

#include "slam/async_slam/tail.h"

namespace cuvslam::slam {

std::optional<std::pair<int64_t, Isometry3T>> Tail::GetTip() const {
  std::lock_guard locker(tail_guard_);

  if (tail_.empty()) {
    return {};
  }
  return tail_.back();
}

bool Tail::UpdateTimeByOdometry(int64_t timestamp_ns, const Isometry3T& pose) {
  std::lock_guard locker(tail_guard_);
  if (!tail_.empty()) {
    const int64_t last_timestamp = tail_.back().first;
    if (last_timestamp > timestamp_ns) {
      TraceWarning("Ignore odometry frame.");
      return false;  // we already have more fresh data
    }
  }
  tail_.emplace_back(timestamp_ns, pose);
  return true;
}

void Tail::Clear() {
  std::lock_guard locker(tail_guard_);
  tail_.clear();
}

// called from slam thread
void Tail::UpdatePoseBySLAM(int64_t timestamp_ns, const Isometry3T& pose) {
  std::lock_guard locker(tail_guard_);

  if (tail_.empty() || tail_.back().first < timestamp_ns) {
    // SLAM has the latest known pose. For example if user load map with timestamp in future
    tail_.clear();
    tail_.emplace_back(timestamp_ns, pose);
    return;
  }
  // else SLAM updates Tail in the past

  auto it = tail_.begin();
  while (it != tail_.end() && it->first < timestamp_ns) {
    ++it;
  }
  const Isometry3T& old_pose = it->second;

  if (old_pose.isApprox(pose, 0.01f)) {  // neither pose should have scale
    return;
  }

  const Isometry3T delta = old_pose.inverse() * pose;
  while (it != tail_.end()) {
    Isometry3T updated = it->second * delta;
    RemoveScaleFromTransform(updated);  // remove scale caused by floating-point errors
    it->second = updated;
    ++it;
  }
}

}  // namespace cuvslam::slam
