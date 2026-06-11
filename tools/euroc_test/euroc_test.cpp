
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

/*
 * Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA software released under the NVIDIA Open Software License is intended to be used permissively and enable the
 * further development of AI technologies. Subject to the terms of this License, NVIDIA confirms that you are free to
 * commercially use, modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership to any
 * outputs generated using the software or derivative works thereof. By using, reproducing, modifying, distributing,
 * performing or displaying any portion or element of the software or derivative works thereof, you agree to be bound by
 * this License.
 */

// Run cuVSLAM stereo (+IMU) tracking on a EuRoC MAV dataset sequence.
//
// Usage:
//   euroc_test --dataset /path/to/V1_01_easy/V1_01_easy
//              --poses_file poses.txt [--use_imu=true]
//
// Output: TUM-format pose file  (timestamp_s tx ty tz qx qy qz qw)

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "common/log.h"
#include "cuvslam/cuvslam2.h"
#include "gflags/gflags.h"
#include "utils/image_io.h"

DEFINE_string(dataset, "", "Path to EuRoC sequence root (the directory that contains mav0/)");
DEFINE_string(poses_file, "poses.txt", "Output poses file (TUM format: timestamp tx ty tz qx qy qz qw)");
DEFINE_string(
    state_file, "",
    "Output IMU state file (timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz). Empty = no output.");
DEFINE_bool(use_imu, true, "Enable IMU fusion (OdometryMode::Inertial). Disable for pure stereo.");
DEFINE_bool(use_slam, false, "Enable SLAM map system for loop-closure correction (default: on).");
DEFINE_int32(verbosity, 3, "Trace verbosity: 0=None, 1=Error, 2=Warning, 3=Message, 4=Debug");

namespace {

// ---------------------------------------------------------------------------
// Minimal EuRoC YAML helpers
// ---------------------------------------------------------------------------

std::string ReadFileToString(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) throw std::runtime_error("Cannot open file: " + path);
  return {std::istreambuf_iterator<char>(f), {}};
}

// Parse "[a, b, c, ...]" array that may span multiple lines.
// Searches for 'key' in 'content', then captures everything between '[' and ']'.
std::vector<double> ParseBracketArray(const std::string& content, const std::string& key) {
  auto pos = content.find(key);
  if (pos == std::string::npos) throw std::runtime_error("YAML key not found: " + key);
  auto start = content.find('[', pos);
  auto end = content.find(']', start);
  if (start == std::string::npos || end == std::string::npos)
    throw std::runtime_error("YAML array brackets not found for key: " + key);

  std::string raw = content.substr(start + 1, end - start - 1);
  // Flatten newlines
  for (auto& c : raw) {
    if (c == '\n' || c == '\r') c = ' ';
  }

  std::vector<double> result;
  std::istringstream ss(raw);
  std::string token;
  while (std::getline(ss, token, ',')) {
    size_t a = token.find_first_not_of(" \t");
    size_t b = token.find_last_not_of(" \t");
    if (a == std::string::npos) continue;
    result.push_back(std::stod(token.substr(a, b - a + 1)));
  }
  return result;
}

// Parse a scalar value: "key: value  # optional comment"
double ParseScalar(const std::string& content, const std::string& key) {
  auto pos = content.find(key);
  if (pos == std::string::npos) throw std::runtime_error("YAML key not found: " + key);
  auto colon = content.find(':', pos);
  auto nl = content.find('\n', colon);
  if (nl == std::string::npos) nl = content.size();  // Handle EOF without newline
  std::string val = content.substr(colon + 1, nl - colon - 1);
  // Strip inline comment
  auto hash = val.find('#');
  if (hash != std::string::npos) val = val.substr(0, hash);
  size_t a = val.find_first_not_of(" \t");
  size_t b = val.find_last_not_of(" \t");
  if (a == std::string::npos) throw std::runtime_error("Empty value for key: " + key);
  return std::stod(val.substr(a, b - a + 1));
}

// Parse the 4x4 row-major T_BS matrix from a sensor.yaml.
// "T_BS: \n  ... \n  data: [...]"
Eigen::Matrix4d ParseT_BS(const std::string& content) {
  auto tbs_pos = content.find("T_BS:");
  if (tbs_pos == std::string::npos) throw std::runtime_error("T_BS not found in YAML");
  auto data_pos = content.find("data:", tbs_pos);
  if (data_pos == std::string::npos) throw std::runtime_error("data: not found after T_BS");

  auto values = ParseBracketArray(content.substr(data_pos), "data:");
  if (values.size() < 16) throw std::runtime_error("T_BS data has fewer than 16 elements");

  Eigen::Matrix4d T;
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++) T(r, c) = values[r * 4 + c];
  return T;
}

// Convert a 4x4 Eigen transform to a cuvslam::Pose (rotation as xyzw quaternion).
cuvslam::Pose ToCuvslam(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  cuvslam::Pose pose;
  pose.rotation = {static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
                   static_cast<float>(q.w())};
  pose.translation = {static_cast<float>(T(0, 3)), static_cast<float>(T(1, 3)), static_cast<float>(T(2, 3))};
  return pose;
}

// ---------------------------------------------------------------------------
// EuRoC calibration parsers
// ---------------------------------------------------------------------------

// Parse cam{0,1}/sensor.yaml → cuvslam::Camera.
// EuRoC distortion model "radial-tangential" uses [k1, k2, p1, p2];
// mapped to Brown [k1, k2, k3=0, p1, p2].
// Parse "distortion_model: <value>" from YAML content.
std::string ParseDistortionModel(const std::string& content) {
  auto pos = content.find("distortion_model:");
  if (pos == std::string::npos) return "";
  auto colon = content.find(':', pos);
  auto nl = content.find('\n', colon);
  std::string val = content.substr(colon + 1, nl - colon - 1);
  // Strip inline comment
  auto hash = val.find('#');
  if (hash != std::string::npos) val = val.substr(0, hash);
  size_t a = val.find_first_not_of(" \t");
  size_t b = val.find_last_not_of(" \t");
  if (a == std::string::npos) return "";
  return val.substr(a, b - a + 1);
}

cuvslam::Camera ParseCameraYaml(const std::string& yaml_path) {
  auto content = ReadFileToString(yaml_path);

  auto res = ParseBracketArray(content, "resolution:");   // [width, height]
  auto intr = ParseBracketArray(content, "intrinsics:");  // [fu, fv, cu, cv]

  if (res.size() < 2) throw std::runtime_error("resolution: expected 2 elements in " + yaml_path);
  if (intr.size() < 4) throw std::runtime_error("intrinsics: expected 4 elements in " + yaml_path);

  cuvslam::Camera cam{};
  cam.size = {static_cast<int32_t>(res[0]), static_cast<int32_t>(res[1])};
  cam.focal = {static_cast<float>(intr[0]), static_cast<float>(intr[1])};
  cam.principal = {static_cast<float>(intr[2]), static_cast<float>(intr[3])};
  cam.rig_from_camera = ToCuvslam(ParseT_BS(content));

  const std::string dist_model = ParseDistortionModel(content);
  if (dist_model == "pinhole") {
    cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
  } else {
    // Default: radial-tangential → Brown [k1, k2, k3=0, p1, p2]
    auto d_raw = ParseBracketArray(content, "distortion_coefficients:");  // [k1,k2,p1,p2]
    if (d_raw.size() < 4) throw std::runtime_error("distortion_coefficients: expected 4 elements in " + yaml_path);
    cam.distortion.model = cuvslam::Distortion::Model::Brown;
    cam.distortion.parameters = {static_cast<float>(d_raw[0]), static_cast<float>(d_raw[1]),
                                 0.0f,  // k3 = 0
                                 static_cast<float>(d_raw[2]), static_cast<float>(d_raw[3])};
  }
  return cam;
}

cuvslam::ImuCalibration ParseImuYaml(const std::string& yaml_path) {
  auto content = ReadFileToString(yaml_path);

  cuvslam::ImuCalibration imu{};
  imu.rig_from_imu = ToCuvslam(ParseT_BS(content));
  imu.frequency = static_cast<float>(ParseScalar(content, "rate_hz"));
  imu.gyroscope_noise_density = static_cast<float>(ParseScalar(content, "gyroscope_noise_density"));
  imu.gyroscope_random_walk = static_cast<float>(ParseScalar(content, "gyroscope_random_walk"));
  imu.accelerometer_noise_density = static_cast<float>(ParseScalar(content, "accelerometer_noise_density"));
  imu.accelerometer_random_walk = static_cast<float>(ParseScalar(content, "accelerometer_random_walk"));
  return imu;
}

// ---------------------------------------------------------------------------
// CSV readers
// ---------------------------------------------------------------------------

struct ImageEntry {
  int64_t timestamp_ns;
  std::string filename;  // e.g. "1403715273262142976.png"
};

std::vector<ImageEntry> ParseImageCsv(const std::string& csv_path) {
  std::ifstream f(csv_path);
  if (!f.is_open()) throw std::runtime_error("Cannot open: " + csv_path);
  std::vector<ImageEntry> entries;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string ts, fn;
    std::getline(ss, ts, ',');
    std::getline(ss, fn);
    while (!fn.empty() && (fn.back() == '\r' || fn.back() == ' ')) fn.pop_back();
    entries.push_back({std::stoll(ts), fn});
  }
  return entries;
}

struct ImuEntry {
  int64_t timestamp_ns;
  float wx, wy, wz;  // angular velocity  [rad/s]
  float ax, ay, az;  // linear acceleration [m/s²]
};

std::vector<ImuEntry> ParseImuCsv(const std::string& csv_path) {
  std::ifstream f(csv_path);
  if (!f.is_open()) throw std::runtime_error("Cannot open: " + csv_path);
  std::vector<ImuEntry> entries;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string tok;
    ImuEntry e;
    std::getline(ss, tok, ',');
    e.timestamp_ns = std::stoll(tok);
    std::getline(ss, tok, ',');
    e.wx = std::stof(tok);
    std::getline(ss, tok, ',');
    e.wy = std::stof(tok);
    std::getline(ss, tok, ',');
    e.wz = std::stof(tok);
    std::getline(ss, tok, ',');
    e.ax = std::stof(tok);
    std::getline(ss, tok, ',');
    e.ay = std::stof(tok);
    std::getline(ss, tok, ',');
    e.az = std::stof(tok);
    entries.push_back(e);
  }
  return entries;
}

// ---------------------------------------------------------------------------
// Pose output (TUM format)
// ---------------------------------------------------------------------------

void PrintPoseTum(std::ostream& stream, int64_t timestamp_ns, const cuvslam::Pose& pose) {
  stream << std::fixed << std::setprecision(9) << static_cast<double>(timestamp_ns) * 1e-9 << " " << pose.translation[0]
         << " " << pose.translation[1] << " " << pose.translation[2] << " " << pose.rotation[0] << " "  // qx
         << pose.rotation[1] << " "                                                                     // qy
         << pose.rotation[2] << " "                                                                     // qz
         << pose.rotation[3] << "\n";                                                                   // qw
}

}  // namespace

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "Run cuVSLAM stereo(+IMU) on a EuRoC sequence.\n"
      "  --dataset  path/to/V1_01_easy  (directory containing mav0/)\n"
      "  --poses_file  output_poses.txt\n"
      "  --use_imu  true|false");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cuvslam::SetVerbosity(FLAGS_verbosity);

  if (FLAGS_dataset.empty()) {
    std::cerr << "Error: --dataset is required.\n";
    gflags::ShowUsageWithFlags(argv[0]);
    return EXIT_FAILURE;
  }

  namespace fs = std::filesystem;
  const fs::path mav0 = fs::path(FLAGS_dataset) / "mav0";

  try {
    // Build camera rig from EuRoC calibration files
    cuvslam::Rig rig;
    rig.cameras.push_back(ParseCameraYaml((mav0 / "cam0" / "sensor.yaml").string()));
    rig.cameras.push_back(ParseCameraYaml((mav0 / "cam1" / "sensor.yaml").string()));

    if (FLAGS_use_imu) {
      rig.imus.push_back(ParseImuYaml((mav0 / "imu0" / "sensor.yaml").string()));
    }

    // Configure and construct tracker
    cuvslam::Odometry::Config cfg;
    cfg.odometry_mode =
        FLAGS_use_imu ? cuvslam::Odometry::OdometryMode::Inertial : cuvslam::Odometry::OdometryMode::Multicamera;
    cfg.async_sba = false;  // Disable async SBA since euroc_test feeds frames without real-time delays
    if (FLAGS_use_slam) {
      // GetState() requires observations and landmarks to be exported.
      cfg.enable_observations_export = true;
      cfg.enable_landmarks_export = true;
    }

    cuvslam::WarmUpGPU();
    cuvslam::Odometry tracker(rig, cfg);

    std::unique_ptr<cuvslam::Slam> slam;
    if (FLAGS_use_slam) {
      cuvslam::Slam::Config slam_cfg;
      slam_cfg.sync_mode = true;  // Run SLAM synchronously since euroc_test feeds frames without real-time delays
      slam = std::make_unique<cuvslam::Slam>(rig, tracker.GetPrimaryCameras(), slam_cfg);
    }

    // Read image and IMU lists
    auto cam0_entries = ParseImageCsv((mav0 / "cam0" / "data.csv").string());
    auto cam1_entries = ParseImageCsv((mav0 / "cam1" / "data.csv").string());

    std::vector<ImuEntry> imu_data;
    if (FLAGS_use_imu) {
      imu_data = ParseImuCsv((mav0 / "imu0" / "data.csv").string());
    }

    std::ofstream out(FLAGS_poses_file);
    if (!out.is_open()) throw std::runtime_error("Cannot open output file: " + FLAGS_poses_file);

    std::ofstream state_out;
    if (!FLAGS_state_file.empty()) {
      state_out.open(FLAGS_state_file);
      if (!state_out.is_open()) throw std::runtime_error("Cannot open state file: " + FLAGS_state_file);
      state_out << "# timestamp_s tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz\n";
    }

    // Match cam0 and cam1 frames by timestamp (nearest-neighbor within 5 ms).
    // EuRoC cameras are hardware-synced so timestamps should be identical or
    // differ by only a few microseconds due to clock jitter.
    struct FramePair {
      int64_t timestamp_ns;  // cam0 timestamp used for both (authoritative)
      size_t idx0, idx1;
    };
    std::vector<FramePair> frame_pairs;
    {
      if (cam1_entries.empty()) {
        std::cerr << "Warning: cam1 has no entries; cannot match stereo frames\n";
      }

      size_t j = 0;
      for (size_t i = 0; i < cam0_entries.size(); i++) {
        if (cam1_entries.empty()) break;  // Skip matching if cam1 is empty
        int64_t t0 = cam0_entries[i].timestamp_ns;
        // Advance cam1 pointer to the closest timestamp
        while (j + 1 < cam1_entries.size() &&
               std::abs(cam1_entries[j + 1].timestamp_ns - t0) < std::abs(cam1_entries[j].timestamp_ns - t0)) {
          j++;
        }
        int64_t diff_ns = std::abs(cam1_entries[j].timestamp_ns - t0);
        if (diff_ns > 5'000'000) {  // 5 ms threshold — skip unmatched frames
          std::cerr << "Warning: no cam1 match for cam0 frame " << i << " (closest diff=" << diff_ns / 1000
                    << " us), skipping\n";
          continue;
        }
        frame_pairs.push_back({t0, i, j});
      }
    }

    const size_t n_frames = frame_pairs.size();
    size_t imu_idx = 0;
    size_t tracked = 0;

    std::cout << "Dataset : " << FLAGS_dataset << "\n"
              << "Frames  : " << n_frames << "\n"
              << "IMU     : " << (FLAGS_use_imu ? "enabled" : "disabled") << "\n"
              << "SLAM    : " << (FLAGS_use_slam ? "enabled" : "disabled") << "\n"
              << "Output  : " << FLAGS_poses_file << "\n\n";

    for (size_t i = 0; i < n_frames; i++) {
      const auto& e0 = cam0_entries[frame_pairs[i].idx0];
      const auto& e1 = cam1_entries[frame_pairs[i].idx1];
      const int64_t frame_ts = frame_pairs[i].timestamp_ns;

      // Register all IMU measurements that arrived before this camera frame
      if (FLAGS_use_imu) {
        while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp_ns < frame_ts) {
          const auto& imu = imu_data[imu_idx];
          cuvslam::ImuMeasurement m;
          m.timestamp_ns = imu.timestamp_ns;
          m.angular_velocities = {imu.wx, imu.wy, imu.wz};
          m.linear_accelerations = {imu.ax, imu.ay, imu.az};
          tracker.RegisterImuMeasurement(0, m);
          imu_idx++;
        }
      }

      // Load PNG images
      cuvslam::ImageMatrix<uint8_t> img0, img1;
      const auto path0 = (mav0 / "cam0" / "data" / e0.filename).string();
      const auto path1 = (mav0 / "cam1" / "data" / e1.filename).string();
      if (!cuvslam::LoadPng(path0, img0)) {
        std::cerr << "Failed to load image: " << path0 << "\n";
        continue;
      }
      if (!cuvslam::LoadPng(path1, img1)) {
        std::cerr << "Failed to load image: " << path1 << "\n";
        continue;
      }

      // Build image set for Track()
      cuvslam::Odometry::ImageSet images = {
          {{img0.data(), static_cast<int32_t>(img0.cols()), static_cast<int32_t>(img0.rows()),
            static_cast<int32_t>(img0.cols()), cuvslam::ImageData::Encoding::MONO, cuvslam::ImageData::DataType::UINT8,
            /*is_gpu_mem=*/false},
           frame_ts,
           /*camera_index=*/0},
          {{img1.data(), static_cast<int32_t>(img1.cols()), static_cast<int32_t>(img1.rows()),
            static_cast<int32_t>(img1.cols()), cuvslam::ImageData::Encoding::MONO, cuvslam::ImageData::DataType::UINT8,
            /*is_gpu_mem=*/false},
           frame_ts,  // same timestamp for both; EuRoC cameras are hardware-synced
           /*camera_index=*/1},
      };

      auto result = tracker.Track(images);
      if (result.world_from_rig.has_value()) {
        cuvslam::Pose out_pose = result.world_from_rig->pose;
        if (slam) {
          cuvslam::Odometry::State state;
          tracker.GetState(state);
          out_pose = slam->Track(state);
        }

        PrintPoseTum(out, frame_ts, out_pose);
        if (state_out.is_open() && FLAGS_use_imu) {
          auto imu_state = tracker.GetImuState();
          state_out << std::fixed << std::setprecision(9) << static_cast<double>(frame_ts) * 1e-9 << " "
                    << out_pose.translation[0] << " " << out_pose.translation[1] << " " << out_pose.translation[2]
                    << " " << out_pose.rotation[0] << " " << out_pose.rotation[1] << " " << out_pose.rotation[2] << " "
                    << out_pose.rotation[3];
          if (imu_state.has_value()) {
            state_out << " " << imu_state->velocity[0] << " " << imu_state->velocity[1] << " " << imu_state->velocity[2]
                      << " " << imu_state->gyro_bias[0] << " " << imu_state->gyro_bias[1] << " "
                      << imu_state->gyro_bias[2] << " " << imu_state->acc_bias[0] << " " << imu_state->acc_bias[1]
                      << " " << imu_state->acc_bias[2];
          } else {
            state_out << " 0 0 0 0 0 0 0 0 0";
          }
          state_out << "\n";
        }
        tracked++;
      } else {
        std::cerr << "Tracking lost at frame " << i << "\n";
      }

      if ((i + 1) % 100 == 0) {
        std::cout << "  frame " << (i + 1) << "/" << n_frames << "  tracked=" << tracked << "\n";
      }
    }

    std::cout << "\nDone. Tracked " << tracked << "/" << n_frames << " frames.\n"
              << "Poses written to: " << FLAGS_poses_file << "\n";

  } catch (const std::exception& ex) {
    std::cerr << "Fatal error: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
