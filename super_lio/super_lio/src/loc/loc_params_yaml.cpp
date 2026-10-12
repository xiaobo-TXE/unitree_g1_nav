#include "loc/loc_params_yaml.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace LI2Sup {
namespace loc {

namespace {

constexpr double kDeg = M_PI / 180.0;

std::string trim(std::string s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  const auto e = s.find_last_not_of(" \t\r\n");
  return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

/// Strips the quotes a ROS params file puts around strings.
std::string unquote(std::string s) {
  if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front()) {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

}  // namespace

bool LoadLocParamsFromYaml(const std::string& path, LocParams* p) {
  std::ifstream f(path);
  if (!f.is_open()) return false;

  std::unordered_map<std::string, std::string> kv;
  std::string line;
  while (std::getline(f, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trim(line.substr(0, colon));
    const std::string val = trim(line.substr(colon + 1));
    if (key.rfind("lio.", 0) == 0 && !val.empty()) kv[key] = val;
  }

  auto num = [&kv](const char* k, double def) {
    auto it = kv.find(k);
    if (it == kv.end()) return def;
    try { return std::stod(it->second); } catch (...) { return def; }
  };
  auto integer = [&kv](const char* k, int def) {
    auto it = kv.find(k);
    if (it == kv.end()) return def;
    try { return std::stoi(it->second); } catch (...) { return def; }
  };
  auto boolean = [&kv](const char* k, bool def) {
    auto it = kv.find(k);
    if (it == kv.end()) return def;
    const std::string v = it->second;
    return v == "true" || v == "True" || v == "1";
  };
  auto str = [&kv](const char* k, const std::string& def) {
    auto it = kv.find(k);
    return it == kv.end() ? def : unquote(it->second);
  };

  p->map.map_dir      = str("lio.maploc.map_dir", p->map.map_dir);
  p->map.preload_all  = boolean("lio.maploc.preload_all", p->map.preload_all);
  p->map.lru_capacity = integer("lio.maploc.lru_capacity", p->map.lru_capacity);

  p->submap.radius       = num("lio.maploc.submap_radius", p->submap.radius);
  p->submap.max_k        = integer("lio.maploc.submap_max_k", p->submap.max_k);
  p->submap.refresh_dist = num("lio.maploc.submap_refresh_dist", p->submap.refresh_dist);
  p->submap.prefetch_max_files =
      integer("lio.maploc.submap_prefetch_max_files", p->submap.prefetch_max_files);

  p->scan.accum_frames = integer("lio.maploc.accum_frames", p->scan.accum_frames);
  p->scan.source_voxel = num("lio.maploc.source_voxel", p->scan.source_voxel);

  p->track.target_voxel  = num("lio.maploc.target_voxel", p->track.target_voxel);
  p->track.source_voxel  = p->scan.source_voxel;
  p->track.max_corr_dist = num("lio.maploc.max_corr_dist", p->track.max_corr_dist);
  p->track.max_iter      = integer("lio.maploc.max_iter", p->track.max_iter);
  p->track.num_threads   = integer("lio.maploc.num_threads", p->track.num_threads);
  p->track.cov_neighbors = integer("lio.maploc.cov_neighbors", p->track.cov_neighbors);
  p->track.cov_floor_t   = num("lio.maploc.cov_floor_t", p->track.cov_floor_t);
  p->track.cov_floor_r   = num("lio.maploc.cov_floor_r", p->track.cov_floor_r / kDeg) * kDeg;

  p->coarse.target_voxel  = num("lio.maploc.init_target_voxel", p->coarse.target_voxel);
  p->coarse.source_voxel  = num("lio.maploc.init_source_voxel", p->coarse.source_voxel);
  p->coarse.max_corr_dist = num("lio.maploc.init_max_corr_dist", p->coarse.max_corr_dist);
  p->coarse.max_iter      = integer("lio.maploc.init_max_iter", p->coarse.max_iter);
  p->coarse.num_threads   = p->track.num_threads;
  p->coarse.cov_neighbors = p->track.cov_neighbors;

  p->trigger.match_min_dist =
      num("lio.maploc.match_min_dist", p->trigger.match_min_dist);
  p->trigger.match_max_interval =
      num("lio.maploc.match_max_interval", p->trigger.match_max_interval);

  p->est.track_inlier_min = num("lio.maploc.track_inlier_min", p->est.track_inlier_min);
  p->est.track_rmse_max   = num("lio.maploc.track_rmse_max", p->est.track_rmse_max);
  p->est.gate_body_t      = num("lio.maploc.gate_body_t", p->est.gate_body_t);
  p->est.gate_body_r      = num("lio.maploc.gate_body_r", p->est.gate_body_r / kDeg) * kDeg;
  p->est.chi2_gate        = num("lio.maploc.chi2_gate", p->est.chi2_gate);
  p->est.sigma_t_per_m    = num("lio.maploc.sigma_t_per_m", p->est.sigma_t_per_m);
  p->est.sigma_r_per_m    = num("lio.maploc.sigma_r_per_m", p->est.sigma_r_per_m);
  p->est.sigma_r_per_tilt = num("lio.maploc.sigma_r_per_tilt", p->est.sigma_r_per_tilt);
  p->est.sigma_t_per_s    = num("lio.maploc.sigma_t_per_s", p->est.sigma_t_per_s);
  p->est.sigma_r_per_s    = num("lio.maploc.sigma_r_per_s", p->est.sigma_r_per_s);
  p->est.estimate_roll_pitch =
      boolean("lio.maploc.estimate_roll_pitch", p->est.estimate_roll_pitch);
  p->est.rp_pseudo_sigma =
      num("lio.maploc.rp_pseudo_sigma", p->est.rp_pseudo_sigma / kDeg) * kDeg;
  p->est.rp_anchor_rate =
      num("lio.maploc.rp_anchor_rate", p->est.rp_anchor_rate / kDeg) * kDeg;
  p->est.max_degenerate_t_dims =
      integer("lio.maploc.max_degenerate_t_dims", p->est.max_degenerate_t_dims);
  p->est.max_degenerate_r_dims =
      integer("lio.maploc.max_degenerate_r_dims", p->est.max_degenerate_r_dims);
  p->est.unconverged_cov_scale =
      num("lio.maploc.unconverged_cov_scale", p->est.unconverged_cov_scale);
  p->est.rate_gate_enable = boolean("lio.maploc.rate_gate_enable", p->est.rate_gate_enable);
  p->est.chi2_confirm_enable = boolean("lio.maploc.chi2_confirm_enable", p->est.chi2_confirm_enable);
  p->est.rate_gate_t = num("lio.maploc.rate_gate_t", p->est.rate_gate_t);
  p->est.rate_gate_r = num("lio.maploc.rate_gate_r", p->est.rate_gate_r);
  p->est.rate_gate_floor_t =
      num("lio.maploc.rate_gate_floor_t", p->est.rate_gate_floor_t);
  p->est.rate_gate_floor_r =
      num("lio.maploc.rate_gate_floor_r", p->est.rate_gate_floor_r / kDeg) * kDeg;

  p->watch.enable = boolean("lio.maploc.watch_enable", p->watch.enable);
  p->watch.max_speed = num("lio.maploc.watch_max_speed", p->watch.max_speed);
  p->watch.speed_frames =
      static_cast<int>(num("lio.maploc.watch_speed_frames", p->watch.speed_frames));
  p->watch.max_gap_s = num("lio.maploc.watch_max_gap", p->watch.max_gap_s);
  p->watch.odom_radius_margin =
      num("lio.maploc.watch_odom_radius_margin", p->watch.odom_radius_margin);
  p->watch.recover_frames =
      static_cast<int>(num("lio.maploc.watch_recover_frames", p->watch.recover_frames));

  p->composer.v_max  = num("lio.maploc.v_max", p->composer.v_max);
  p->composer.w_max  = num("lio.maploc.w_max", p->composer.w_max / kDeg) * kDeg;
  p->composer.tau    = num("lio.maploc.tau", p->composer.tau);
  p->composer.snap_t = num("lio.maploc.snap_t", p->composer.snap_t);
  p->composer.snap_r = num("lio.maploc.snap_r", p->composer.snap_r / kDeg) * kDeg;

  p->init.yaw_known     = boolean("lio.maploc.init_yaw_known", p->init.yaw_known);
  p->init.yaw_range     = num("lio.maploc.init_yaw_range", p->init.yaw_range / kDeg) * kDeg;
  p->init.yaw_step      = num("lio.maploc.init_yaw_step", p->init.yaw_step / kDeg) * kDeg;
  p->init.xy_step       = num("lio.maploc.init_xy_step", p->init.xy_step);
  p->init.z_from_map    = boolean("lio.maploc.init_z_from_map", p->init.z_from_map);
  p->init.submap_radius = num("lio.maploc.init_submap_radius", p->init.submap_radius);
  p->init.submap_max_k  = integer("lio.maploc.init_submap_max_k", p->init.submap_max_k);
  p->init.inlier_min    = num("lio.maploc.init_inlier_min", p->init.inlier_min);
  p->init.rmse_max      = num("lio.maploc.init_rmse_max", p->init.rmse_max);
  p->init.max_attempts  = integer("lio.maploc.init_max_attempts", p->init.max_attempts);
  p->init.eager_attempts =
      integer("lio.maploc.init_eager_attempts", p->init.eager_attempts);
  p->init.attempt_min_dist =
      num("lio.maploc.init_attempt_min_dist", p->init.attempt_min_dist);
  p->init.attempt_min_interval =
      num("lio.maploc.init_attempt_min_interval", p->init.attempt_min_interval);
  p->init.refine_top_k  = integer("lio.maploc.init_refine_top_k", p->init.refine_top_k);
  p->init.consistency_t = num("lio.maploc.init_consistency_t", p->init.consistency_t);
  p->init.consistency_r =
      num("lio.maploc.init_consistency_r", p->init.consistency_r / kDeg) * kDeg;

  p->lost.reject_count = integer("lio.maploc.lost_reject_count", p->lost.reject_count);
  p->lost.timeout      = num("lio.maploc.lost_timeout", p->lost.timeout);
  p->lost.yaw_range    = num("lio.maploc.lost_yaw_range", p->lost.yaw_range / kDeg) * kDeg;
  p->lost.xy_range     = num("lio.maploc.lost_xy_range", p->lost.xy_range);
  p->lost.attempt_min_dist =
      num("lio.maploc.lost_attempt_min_dist", p->lost.attempt_min_dist);
  p->lost.attempt_min_interval =
      num("lio.maploc.lost_attempt_min_interval", p->lost.attempt_min_interval);

  p->output.match_log = str("lio.maploc.match_log", p->output.match_log);

  p->log.debug           = boolean("lio.maploc.debug", p->log.debug);
  p->log.heartbeat_s     = num("lio.maploc.log_heartbeat_s", p->log.heartbeat_s);
  p->log.summary_on_exit = boolean("lio.maploc.log_summary_on_exit", p->log.summary_on_exit);
  p->log.warn_rmse       = num("lio.maploc.log_warn_rmse", p->log.warn_rmse);
  p->log.warn_inlier     = num("lio.maploc.log_warn_inlier", p->log.warn_inlier);
  p->log.warn_corr_t     = num("lio.maploc.log_warn_corr_t", p->log.warn_corr_t);
  p->log.warn_corr_r     = num("lio.maploc.log_warn_corr_r", p->log.warn_corr_r / kDeg) * kDeg;
  p->log.warn_match_ms   = num("lio.maploc.log_warn_match_ms", p->log.warn_match_ms);
  p->log.warn_submap_ms  = num("lio.maploc.log_warn_submap_ms", p->log.warn_submap_ms);
  p->log.warn_every_n    = integer("lio.maploc.log_warn_every_n", p->log.warn_every_n);
  p->log.expected_frame_hz = num("lio.maploc.expected_frame_hz", p->log.expected_frame_hz);
  return true;
}

std::string DescribeLocParams(const LocParams& p) {
  const double r2d = 180.0 / M_PI;
  std::ostringstream o;
  o << std::fixed << std::setprecision(4);
  o << "map_dir=" << p.map.map_dir
    << "\n  submap radius=" << p.submap.radius << " max_k=" << p.submap.max_k
    << " refresh=" << p.submap.refresh_dist
    << "\n  scan accum=" << p.scan.accum_frames << " source_voxel=" << p.scan.source_voxel
    << "\n  track voxel=" << p.track.target_voxel << " corr=" << p.track.max_corr_dist
    << " iter=" << p.track.max_iter
    << "\n  coarse voxel=" << p.coarse.target_voxel << " corr=" << p.coarse.max_corr_dist
    << " iter=" << p.coarse.max_iter
    << "\n  trigger min_dist=" << p.trigger.match_min_dist
    << " max_interval=" << p.trigger.match_max_interval
    << "\n  gates inlier=" << p.est.track_inlier_min << " rmse=" << p.est.track_rmse_max
    << " body_t=" << p.est.gate_body_t << " body_r_deg=" << p.est.gate_body_r * r2d
    << " chi2=" << p.est.chi2_gate
    << " deg_t=" << p.est.max_degenerate_t_dims << " deg_r=" << p.est.max_degenerate_r_dims
    << " rate_gate=" << p.est.rate_gate_enable
    << " chi2_confirm=" << p.est.chi2_confirm_enable << " rate_t=" << p.est.rate_gate_t
    << " rate_r=" << p.est.rate_gate_r
    << " floor_t=" << p.est.rate_gate_floor_t
    << " floor_r_deg=" << p.est.rate_gate_floor_r * r2d
    << "\n  watch enable=" << p.watch.enable << " max_speed=" << p.watch.max_speed
    << " frames=" << p.watch.speed_frames
    << " odom_radius_margin=" << p.watch.odom_radius_margin
    << "\n  noise t/m=" << p.est.sigma_t_per_m << " r/m=" << p.est.sigma_r_per_m
    << " r/tilt=" << p.est.sigma_r_per_tilt
    << " t/s=" << p.est.sigma_t_per_s << " r/s=" << p.est.sigma_r_per_s
    << "\n  roll/pitch est=" << p.est.estimate_roll_pitch
    << " sigma_deg=" << p.est.rp_pseudo_sigma * r2d
    << " anchor_rate_deg_s=" << p.est.rp_anchor_rate * r2d
    << "\n  composer v_max=" << p.composer.v_max << " w_max_deg=" << p.composer.w_max * r2d
    << " tau=" << p.composer.tau << " snap_t=" << p.composer.snap_t
    << " snap_r_deg=" << p.composer.snap_r * r2d
    << "\n  log debug=" << p.log.debug << " heartbeat_s=" << p.log.heartbeat_s
    << " warn rmse=" << p.log.warn_rmse << " inlier=" << p.log.warn_inlier
    << " corr_t=" << p.log.warn_corr_t << " corr_r_deg=" << p.log.warn_corr_r * r2d
    << " match_ms=" << p.log.warn_match_ms << " submap_ms=" << p.log.warn_submap_ms;
  return o.str();
}

}  // namespace loc
}  // namespace LI2Sup
