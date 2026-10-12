#include "loc/match_log.h"

#include <iomanip>

#include "basic/logs.h"
#include "loc/drift_estimator.h"
#include "loc/registrar.h"

namespace LI2Sup {
namespace loc {

namespace {

void writePose(std::ofstream& f, const BASIC::SE3& T) {
  const BASIC::Quat q = T.quaternion();
  f << ',' << T.t_.x() << ',' << T.t_.y() << ',' << T.t_.z()
    << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}

}  // namespace

MatchLogger::MatchLogger(const std::string& path) {
  if (path.empty()) return;
  file_.open(path);
  if (!file_.is_open()) {
    LOG(ERROR) << " ---> [MapLoc] cannot open match log " << path;
    return;
  }
  file_ << std::fixed << std::setprecision(6);
  file_ << "t,state,accepted,reason,n_source,n_target,inlier,rmse,iters,converged,"
           "eig_min_t,eig_max_t,eig_min_r,eig_max_r,deg_t,deg_r,"
           "innov_t,innov_r,body_corr_t,body_corr_r,chi2,match_ms,"
           "submap_version,submap_ids,"
           "mx,my,mz,mqx,mqy,mqz,mqw,ox,oy,oz,oqx,oqy,oqz,oqw\n";
  LOG(INFO) << " ---> [MapLoc] match log -> " << path;
}

MatchLogger::~MatchLogger() {
  if (file_.is_open()) file_.close();
}

void MatchLogger::write(double t, const char* state, bool accepted, const char* reason,
                        const RegResult& reg, const UpdateReport& rep,
                        const BASIC::SE3& T_O_B, std::uint64_t submap_version,
                        const std::vector<int>& submap_ids) {
  if (!file_.is_open()) return;

  file_ << t << ',' << state << ',' << (accepted ? 1 : 0) << ',' << reason
        << ',' << reg.num_source << ',' << reg.num_target
        << ',' << reg.inlier_ratio << ',' << (std::isfinite(reg.rmse) ? reg.rmse : -1.0)
        << ',' << reg.iterations << ',' << (reg.converged ? 1 : 0)
        << ',' << reg.eig_min_t << ',' << reg.eig_max_t
        << ',' << reg.eig_min_r << ',' << reg.eig_max_r
        << ',' << reg.degenerate_t_dims << ',' << reg.degenerate_r_dims
        << ',' << rep.innovation_t << ',' << rep.innovation_r
        << ',' << rep.body_correction_t << ',' << rep.body_correction_r
        << ',' << rep.chi2 << ',' << reg.time_ms
        << ',' << submap_version << ',';

  // Semicolon-separated so the field survives a comma-split.
  for (std::size_t i = 0; i < submap_ids.size(); ++i) {
    if (i) file_ << ';';
    file_ << submap_ids[i];
  }

  writePose(file_, reg.T);    // T_match: the registrar's answer, pre-EKF
  writePose(file_, T_O_B);    // odometry pose snapshotted with the scan
  // Flushed per row: a few rows per second is nothing, and a 39-minute run that
  // gets killed should still leave every match it managed to make.
  file_ << std::endl;
}

}  // namespace loc
}  // namespace LI2Sup
