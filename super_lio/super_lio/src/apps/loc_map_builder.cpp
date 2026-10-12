/**
 * @file loc_map_builder.cpp
 * @brief Offline tool: turn a voxel-slam session into the compact loc_map/ the
 *        board actually runs on.
 *
 * Deliberately free of ROS (PCL + Eigen only) so it can run anywhere the map
 * lives, and so it can be pointed at a session without a workspace.
 *
 * Input  (written by voxel-slam-ros2):
 *   <session>/alidarState.txt   26 columns per line, see below
 *   <session>/<i>.pcd           frame i, undistorted, already in the IMU frame
 * Output (see design doc section 3.3):
 *   <out>/meta.yaml  keyframes.txt  kf/%06d.pcd  global_ds.pcd  edges.txt
 *
 * Usage:
 *   loc_map_builder --session DIR --out DIR [--edges FILE]
 *                   [--kf_dist 1.0] [--kf_angle 15] [--kf_range 60]
 *                   [--kf_voxel 0.25] [--global_voxel 0.4] [--max_frames N]
 */
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace fs = std::filesystem;

using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

namespace {

struct Options {
  std::string session;
  std::string out;
  std::string edges;
  double kf_dist = 1.0;
  double kf_angle_deg = 15.0;
  double kf_range = 60.0;
  double kf_voxel = 0.25;
  double global_voxel = 0.4;
  int    max_frames = -1;   ///< debug aid: stop after N source frames
  // The extrinsic recorded into meta.yaml so the runtime can compare it against
  // lio.extrinsic.lidar_imu. Map clouds already have it baked in.
  double ext_t[3] = {-0.011, -0.02329, 0.04412};
};

/// One line of alidarState.txt:
/// t px py pz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz gx gy gz u0..u5
struct RawPose {
  double t = 0;
  Eigen::Isometry3d T_M_B = Eigen::Isometry3d::Identity();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
};

bool parseArgs(int argc, char** argv, Options* o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](double* dst) {
      if (i + 1 >= argc) return false;
      *dst = std::atof(argv[++i]);
      return true;
    };
    if (a == "--session" && i + 1 < argc)      o->session = argv[++i];
    else if (a == "--out" && i + 1 < argc)     o->out = argv[++i];
    else if (a == "--edges" && i + 1 < argc)   o->edges = argv[++i];
    else if (a == "--kf_dist")                 { if (!next(&o->kf_dist)) return false; }
    else if (a == "--kf_angle")                { if (!next(&o->kf_angle_deg)) return false; }
    else if (a == "--kf_range")                { if (!next(&o->kf_range)) return false; }
    else if (a == "--kf_voxel")                { if (!next(&o->kf_voxel)) return false; }
    else if (a == "--global_voxel")            { if (!next(&o->global_voxel)) return false; }
    else if (a == "--max_frames" && i + 1 < argc) o->max_frames = std::atoi(argv[++i]);
    else if (a == "-h" || a == "--help")       return false;
    else {
      std::cerr << "unknown argument: " << a << "\n";
      return false;
    }
  }
  return !o->session.empty() && !o->out.empty();
}

std::vector<RawPose> loadPoses(const std::string& path, bool* monotonic) {
  std::vector<RawPose> out;
  std::ifstream f(path);
  if (!f.is_open()) return out;

  *monotonic = true;
  double last_t = -1.0;
  std::string line;
  int line_no = 0;
  while (std::getline(f, line)) {
    ++line_no;
    if (line.empty()) continue;
    std::istringstream is(line);
    RawPose p;
    double px, py, pz, qx, qy, qz, qw;
    double vx, vy, vz, bg[3], ba[3], g[3];
    if (!(is >> p.t >> px >> py >> pz >> qx >> qy >> qz >> qw
             >> vx >> vy >> vz >> bg[0] >> bg[1] >> bg[2]
             >> ba[0] >> ba[1] >> ba[2] >> g[0] >> g[1] >> g[2])) {
      std::cerr << "alidarState.txt line " << line_no << " is malformed, stopping\n";
      break;
    }
    // Quaternion order in the file is x y z w.
    p.T_M_B.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    p.T_M_B.translation() = Eigen::Vector3d(px, py, pz);
    p.gravity = Eigen::Vector3d(g[0], g[1], g[2]);
    if (last_t >= 0.0 && p.t < last_t) *monotonic = false;
    last_t = p.t;
    out.push_back(p);
  }
  return out;
}

double rotationAngle(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  return Eigen::AngleAxisd(A.transpose() * B).angle();
}

CloudXYZ::Ptr voxelize(const CloudXYZ::Ptr& in, double leaf) {
  if (leaf <= 0.0 || in->empty()) return in;
  auto out = CloudXYZ::Ptr(new CloudXYZ());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(in);
  vg.filter(*out);
  return out;
}

/// Mean nearest-neighbour distance between two clouds, the self-check that says
/// whether consecutive keyframes actually land on each other in the map frame.
double meanNearestDistance(const CloudXYZ::Ptr& a, const CloudXYZ::Ptr& b, int sample) {
  if (a->empty() || b->empty()) return -1.0;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(b);
  std::vector<int> idx(1);
  std::vector<float> sq(1);
  double sum = 0.0;
  int n = 0;
  const int step = std::max<int>(1, static_cast<int>(a->size()) / std::max(1, sample));
  for (std::size_t i = 0; i < a->size(); i += step) {
    if (tree.nearestKSearch((*a)[i], 1, idx, sq) <= 0) continue;
    sum += std::sqrt(sq[0]);
    ++n;
  }
  return n > 0 ? sum / n : -1.0;
}

/// edge.txt: sessionA sessionB idA idB tx ty tz qx qy qz qw, constraining
/// idA <- idB. alidarState is already PGO-optimized so this is only used to
/// prove that; the runtime never reads it.
void checkEdges(const std::string& path, const std::vector<RawPose>& poses) {
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cout << "  (no edge file given, skipping the loop-closure residual check)\n";
    return;
  }
  std::string line;
  int n = 0, bad = 0;
  double sum_t = 0.0, sum_r = 0.0, max_t = 0.0, max_r = 0.0;
  while (std::getline(f, line)) {
    std::istringstream is(line);
    std::string sa, sb;
    int ia, ib;
    double tx, ty, tz, qx, qy, qz, qw;
    if (!(is >> sa >> sb >> ia >> ib >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) continue;
    if (ia < 0 || ib < 0 || ia >= (int)poses.size() || ib >= (int)poses.size()) { ++bad; continue; }

    Eigen::Isometry3d meas = Eigen::Isometry3d::Identity();
    meas.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    meas.translation() = Eigen::Vector3d(tx, ty, tz);

    const Eigen::Isometry3d actual = poses[ia].T_M_B.inverse() * poses[ib].T_M_B;
    const Eigen::Isometry3d err = meas.inverse() * actual;
    const double et = err.translation().norm();
    const double er = Eigen::AngleAxisd(err.linear()).angle() * 180.0 / M_PI;
    sum_t += et; sum_r += er;
    max_t = std::max(max_t, et); max_r = std::max(max_r, er);
    ++n;
  }
  if (n == 0) {
    std::cout << "  edge check: no usable edges (" << bad << " out of range)\n";
    return;
  }
  std::printf("  edge check: %d edges, mean residual %.4f m / %.4f deg, max %.4f m / %.4f deg\n",
              n, sum_t / n, sum_r / n, max_t, max_r);
  if (sum_t / n > 0.1) {
    std::cout << "  WARNING: mean edge residual is large; alidarState may not be "
                 "the optimized trajectory\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, &opt)) {
    std::cout <<
        "usage: loc_map_builder --session DIR --out DIR [--edges FILE]\n"
        "                      [--kf_dist 1.0] [--kf_angle 15] [--kf_range 60]\n"
        "                      [--kf_voxel 0.25] [--global_voxel 0.4] [--max_frames N]\n";
    return 1;
  }

  const auto t_start = std::chrono::steady_clock::now();
  std::string session = opt.session;
  if (!session.empty() && session.back() == '/') session.pop_back();

  // ---- 1. poses ----------------------------------------------------------
  bool monotonic = true;
  auto poses = loadPoses(session + "/alidarState.txt", &monotonic);
  if (poses.empty()) {
    std::cerr << "error: no poses read from " << session << "/alidarState.txt\n";
    return 2;
  }
  std::cout << "read " << poses.size() << " poses, timestamps "
            << (monotonic ? "monotonic" : "NOT MONOTONIC (suspicious)") << "\n";

  std::size_t n_pcd = 0;
  for (const auto& e : fs::directory_iterator(session)) {
    if (e.path().extension() == ".pcd") ++n_pcd;
  }
  if (n_pcd != poses.size()) {
    std::cout << "WARNING: " << poses.size() << " poses but " << n_pcd
              << " pcd files; using min(poses, available)\n";
  }

  std::size_t n_frames = poses.size();
  if (opt.max_frames > 0) n_frames = std::min<std::size_t>(n_frames, opt.max_frames);

  // ---- 2. keyframe thinning ---------------------------------------------
  // Greedy: keep frame 0, then keep whenever we have moved kf_dist or turned
  // kf_angle since the last kept frame. Stationary stretches collapse into one
  // keyframe, which is where most of the 8x reduction comes from.
  const double kf_angle = opt.kf_angle_deg * M_PI / 180.0;
  std::vector<int> kept;
  kept.push_back(0);
  for (std::size_t i = 1; i < n_frames; ++i) {
    const auto& ref = poses[kept.back()];
    const double dt = (poses[i].T_M_B.translation() - ref.T_M_B.translation()).norm();
    const double dr = rotationAngle(ref.T_M_B.linear(), poses[i].T_M_B.linear());
    if (dt >= opt.kf_dist || dr >= kf_angle) kept.push_back(static_cast<int>(i));
  }
  std::cout << "thinned " << n_frames << " frames to " << kept.size() << " keyframes\n";

  fs::create_directories(opt.out + "/kf");

  // ---- 3-5. accumulate, crop, downsample, write --------------------------
  std::ofstream kf_txt(opt.out + "/keyframes.txt");
  if (!kf_txt.is_open()) {
    std::cerr << "error: cannot write " << opt.out << "/keyframes.txt\n";
    return 3;
  }
  kf_txt << "# id t x y z qx qy qz qw src_begin src_end n_points\n";
  kf_txt.precision(9);
  kf_txt << std::fixed;

  auto global = CloudXYZ::Ptr(new CloudXYZ());
  const double range_sq = opt.kf_range * opt.kf_range;
  long long total_points = 0;
  Eigen::Vector3d bmin = Eigen::Vector3d::Constant(1e18);
  Eigen::Vector3d bmax = Eigen::Vector3d::Constant(-1e18);
  std::vector<CloudXYZ::Ptr> recent_for_check;
  std::vector<int> recent_ids;
  int missing_pcd = 0;

  for (std::size_t k = 0; k < kept.size(); ++k) {
    const int ref_idx = kept[k];
    // Every raw frame after the previous keyframe up to and including this one,
    // so no point in the session is thrown away.
    const int begin = (k == 0) ? 0 : kept[k - 1] + 1;
    const int end   = ref_idx;

    const Eigen::Isometry3d T_M_Bk = poses[ref_idx].T_M_B;
    const Eigen::Isometry3d T_Bk_M = T_M_Bk.inverse();

    auto acc = CloudXYZ::Ptr(new CloudXYZ());
    for (int j = begin; j <= end; ++j) {
      CloudXYZ raw;
      const std::string path = session + "/" + std::to_string(j) + ".pcd";
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, raw) < 0) { ++missing_pcd; continue; }
      const Eigen::Isometry3d T_Bk_Bj = T_Bk_M * poses[j].T_M_B;
      const Eigen::Matrix4f T = T_Bk_Bj.matrix().cast<float>();
      CloudXYZ moved;
      pcl::transformPointCloud(raw, moved, T);
      *acc += moved;
    }

    // Crop to the sensor's useful range about the keyframe origin, then voxelize.
    auto cropped = CloudXYZ::Ptr(new CloudXYZ());
    cropped->reserve(acc->size());
    for (const auto& p : *acc) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.x * p.x + p.y * p.y + p.z * p.z > range_sq) continue;
      cropped->push_back(p);
    }
    auto ds = voxelize(cropped, opt.kf_voxel);

    char name[64];
    std::snprintf(name, sizeof(name), "kf/%06d.pcd", static_cast<int>(k));
    if (!ds->empty()) {
      ds->width = ds->size();
      ds->height = 1;
      ds->is_dense = true;
      pcl::io::savePCDFileBinary(opt.out + "/" + name, *ds);
    }

    const Eigen::Quaterniond q(T_M_Bk.linear());
    const Eigen::Vector3d t = T_M_Bk.translation();
    kf_txt << k << " " << poses[ref_idx].t << " "
           << t.x() << " " << t.y() << " " << t.z() << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
           << begin << " " << end << " " << ds->size() << "\n";

    total_points += static_cast<long long>(ds->size());
    bmin = bmin.cwiseMin(t);
    bmax = bmax.cwiseMax(t);

    // Global display cloud, downsampled incrementally to keep memory bounded.
    CloudXYZ in_map;
    pcl::transformPointCloud(*ds, in_map, T_M_Bk.matrix().cast<float>());
    *global += in_map;
    if (global->size() > 4000000) global = voxelize(global, opt.global_voxel);

    // Keep a couple of consecutive keyframes in the map frame for the overlap check.
    if (recent_for_check.size() < 2 && k > 0 && k < 3) {
      auto in_map_ptr = CloudXYZ::Ptr(new CloudXYZ(in_map));
      recent_for_check.push_back(in_map_ptr);
      recent_ids.push_back(static_cast<int>(k));
    }

    if (k % 200 == 0 || k + 1 == kept.size()) {
      std::printf("  keyframe %zu/%zu  (%zu points)\r", k + 1, kept.size(), ds->size());
      std::fflush(stdout);
    }
  }
  std::cout << "\n";
  kf_txt.close();

  global = voxelize(global, opt.global_voxel);
  if (!global->empty()) {
    global->width = global->size();
    global->height = 1;
    global->is_dense = true;
    pcl::io::savePCDFileBinary(opt.out + "/global_ds.pcd", *global);
  }

  if (!opt.edges.empty()) {
    std::error_code ec;
    fs::copy_file(opt.edges, opt.out + "/edges.txt", fs::copy_options::overwrite_existing, ec);
  }

  // ---- meta.yaml ---------------------------------------------------------
  // Flat on purpose: KeyframeMap reads it with a two-line parser rather than
  // taking a yaml-cpp dependency onto the board.
  {
    std::ofstream m(opt.out + "/meta.yaml");
    m.precision(6);
    m << std::fixed;
    m << "# Written by loc_map_builder. Consumed by LI2Sup::loc::KeyframeMap.\n";
    m << "format_version: 1\n";
    m << "source_session: " << session << "\n";
    m << "source_frames: " << n_frames << "\n";
    m << "kf_dist: " << opt.kf_dist << "\n";
    m << "kf_angle: " << opt.kf_angle_deg << "\n";
    m << "kf_range: " << opt.kf_range << "\n";
    m << "kf_voxel: " << opt.kf_voxel << "\n";
    m << "global_voxel: " << opt.global_voxel << "\n";
    m << "num_keyframes: " << kept.size() << "\n";
    m << "total_points: " << total_points << "\n";
    m << "gravity: [" << poses[0].gravity.x() << ", " << poses[0].gravity.y()
      << ", " << poses[0].gravity.z() << "]\n";
    // t(3) then R row-major(9), the same layout as lio.extrinsic.lidar_imu.
    m << "extrinsic_lidar_imu: [" << opt.ext_t[0] << ", " << opt.ext_t[1] << ", "
      << opt.ext_t[2] << ", 1, 0, 0, 0, 1, 0, 0, 0, 1]\n";
    m << "bbox_min: [" << bmin.x() << ", " << bmin.y() << ", " << bmin.z() << "]\n";
    m << "bbox_max: [" << bmax.x() << ", " << bmax.y() << ", " << bmax.z() << "]\n";
    m << "cloud_frame: IMU body frame of each keyframe\n";
    m << "intensity: absent (voxel-slam writes zeros; never use it)\n";
  }

  // ---- 6. self-check -----------------------------------------------------
  const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  std::cout << "\nself-check:\n";
  std::printf("  keyframes: %zu, points: %lld, mean %.0f points/keyframe\n",
              kept.size(), total_points,
              kept.empty() ? 0.0 : double(total_points) / kept.size());
  std::printf("  bbox: x[%.1f %.1f] y[%.1f %.1f] z[%.1f %.1f]\n",
              bmin.x(), bmax.x(), bmin.y(), bmax.y(), bmin.z(), bmax.z());
  std::printf("  gravity from session: [%.4f %.4f %.4f] (tilt %.3f deg)\n",
              poses[0].gravity.x(), poses[0].gravity.y(), poses[0].gravity.z(),
              std::atan2(poses[0].gravity.head<2>().norm(),
                         std::abs(poses[0].gravity.z())) * 180.0 / M_PI);
  if (missing_pcd > 0) {
    std::cout << "  WARNING: " << missing_pcd << " source pcd files were missing\n";
  }
  if (recent_for_check.size() == 2) {
    const double d = meanNearestDistance(recent_for_check[0], recent_for_check[1], 2000);
    std::printf("  keyframe %d vs %d mean NN distance in map frame: %.4f m "
                "(expect ~the voxel size, %.2f m)\n",
                recent_ids[0], recent_ids[1], d, opt.kf_voxel);
    if (d > 4.0 * opt.kf_voxel) {
      std::cout << "  WARNING: neighbouring keyframes do not overlap well; "
                   "check that alidarState really is the optimized trajectory\n";
    }
  }
  checkEdges(opt.edges, poses);

  // Disk footprint, the number that decides preload_all vs the LRU.
  std::uintmax_t bytes = 0;
  for (const auto& e : fs::recursive_directory_iterator(opt.out)) {
    if (e.is_regular_file()) bytes += e.file_size();
  }
  std::printf("  on disk: %.1f MB   (built in %.1f s)\n", bytes / 1048576.0, secs);
  std::cout << "\nwrote " << opt.out << "\n";
  return 0;
}
