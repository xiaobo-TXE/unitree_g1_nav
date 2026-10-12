/**
 * @file registrar.cpp
 * @brief IRegistrar adapters. Both concrete classes are file-local: nothing
 *        outside this translation unit needs to name them, and keeping them
 *        here is what keeps small_gicp and PCL registration headers out of the
 *        rest of the build.
 */
#include "loc/registrar.h"

#include <chrono>
#include <cmath>

#include <Eigen/Eigenvalues>

#include <tbb/task_arena.h>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree_tbb.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_tbb.hpp>
#include <small_gicp/registration/registration.hpp>

#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include "basic/logs.h"

namespace LI2Sup {
namespace loc {

namespace {

Eigen::Isometry3d toIsometry(const BASIC::SE3& T) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear()      = T.R_.template cast<double>();
  out.translation() = T.t_.template cast<double>();
  return out;
}

BASIC::SE3 fromIsometry(const Eigen::Isometry3d& T) {
  BASIC::M3 R = T.linear().template cast<BASIC::scalar>();
  BASIC::V3 t = T.translation().template cast<BASIC::scalar>();
  return BASIC::SE3(R, t);
}

/// Turn sigma^2 * H^-1 into a covariance we are willing to act on: symmetrise,
/// reject non-PSD results, and apply the configured floor so an over-confident
/// fit cannot dominate the EKF. Ordering is [omega; rho], so the rotation floor
/// goes on 0..2 and the translation floor on 3..5.
Eigen::Matrix<double, 6, 6> covarianceFrom(const Eigen::Matrix<double, 6, 6>& H,
                                           double error, int num_inliers,
                                           const RegParams& p) {
  Eigen::Matrix<double, 6, 6> floor = Eigen::Matrix<double, 6, 6>::Zero();
  floor.topLeftCorner<3, 3>()     = Eigen::Matrix3d::Identity() * (p.cov_floor_r * p.cov_floor_r);
  floor.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * (p.cov_floor_t * p.cov_floor_t);

  const int dof = num_inliers - 6;
  if (dof <= 0) return floor * 1e4;  // hopeless fit: enormous covariance, zero gain

  Eigen::Matrix<double, 6, 6> Hs = 0.5 * (H + H.transpose());
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(Hs);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return floor * 1e4;

  const double sigma2 = std::max(error / static_cast<double>(dof), 1e-9);
  Eigen::Matrix<double, 6, 6> cov =
      sigma2 * ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());
  cov = 0.5 * (cov + cov.transpose());

  if (!cov.allFinite()) return floor * 1e4;
  return cov + floor;
}

/// Count directions in which this alignment carries essentially no information.
/// The rotation and translation blocks are examined separately, as the design
/// doc specifies; each is compared against its own largest eigenvalue.
void analyseDegeneracy(const Eigen::Matrix<double, 6, 6>& H, const RegParams& p,
                       RegResult* out) {
  auto block = [&](const Eigen::Matrix3d& B, double* lo, double* hi, int* degenerate) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(0.5 * (B + B.transpose()));
    if (es.info() != Eigen::Success) { *lo = *hi = 0.0; *degenerate = 3; return; }
    const Eigen::Vector3d ev = es.eigenvalues();
    *lo = ev.minCoeff();
    *hi = ev.maxCoeff();
    const double thresh = p.degenerate_eig_ratio * std::max(*hi, 1e-12);
    *degenerate = static_cast<int>((ev.array() < thresh).count());
  };
  block(H.topLeftCorner<3, 3>(),     &out->eig_min_r, &out->eig_max_r, &out->degenerate_r_dims);
  block(H.bottomRightCorner<3, 3>(), &out->eig_min_t, &out->eig_max_t, &out->degenerate_t_dims);
}

// ---------------------------------------------------------------------------
// small_gicp adapter
// ---------------------------------------------------------------------------

class SmallGicpRegistrar final : public IRegistrar {
public:
  explicit SmallGicpRegistrar(const RegParams& params)
      : params_(params), arena_(std::max(1, params.num_threads)) {}

  void setTarget(const CloudXYZ::ConstPtr& cloud_M, std::uint64_t version) override {
    if (!cloud_M || cloud_M->empty()) return;
    if (version != 0 && version == target_version_) return;  // already built

    arena_.execute([&] {
      auto points = std::make_shared<small_gicp::PointCloud>();
      points->resize(cloud_M->size());
      for (std::size_t i = 0; i < cloud_M->size(); ++i) {
        const auto& p = (*cloud_M)[i];
        points->point(i) = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
      }
      // A voxel leaf of 0 would collapse the cloud; treat it as "no downsample".
      if (params_.target_voxel > 0.0f) {
        points = small_gicp::voxelgrid_sampling_tbb(*points, params_.target_voxel * 0.5);
      }

      target_points_ = points;
      target_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
          target_points_, small_gicp::KdTreeBuilderTBB());

      // Required even for the voxel-map target: GaussianVoxel::add accumulates
      // traits::cov(points, i), so a target without per-point covariances yields
      // singular voxel covariances and the GICP factor degenerates.
      small_gicp::estimate_covariances_tbb(*target_points_, *target_tree_, params_.cov_neighbors);

      if (params_.type == RegistrationType::VGICP) {
        voxelmap_ = std::make_shared<small_gicp::GaussianVoxelMap>(params_.target_voxel);
        voxelmap_->insert(*target_points_);
      } else {
        voxelmap_.reset();
      }
    });
    target_version_ = version;
  }

  std::uint64_t targetVersion() const override { return target_version_; }
  bool hasTarget() const override {
    return target_points_ && !target_points_->empty() &&
           (params_.type != RegistrationType::VGICP || voxelmap_ != nullptr);
  }

  RegResult align(const CloudXYZ& source_B, const BASIC::SE3& T_init_M_B) override {
    RegResult out;
    out.T = T_init_M_B;
    if (!hasTarget() || source_B.empty()) return out;

    const auto t0 = std::chrono::steady_clock::now();
    arena_.execute([&] {
      auto source = std::make_shared<small_gicp::PointCloud>();
      source->resize(source_B.size());
      for (std::size_t i = 0; i < source_B.size(); ++i) {
        const auto& p = source_B[i];
        source->point(i) = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
      }
      if (params_.source_voxel > 0.0f) {
        source = small_gicp::voxelgrid_sampling_tbb(*source, params_.source_voxel);
      }
      if (source->size() < 20) return;

      auto source_tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
          source, small_gicp::KdTreeBuilderTBB());
      small_gicp::estimate_covariances_tbb(*source, *source_tree, params_.cov_neighbors);

      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionTBB> reg;
      // ParallelReductionTBB takes its concurrency from the enclosing arena,
      // it has no num_threads field of its own.
      reg.rejector.max_dist_sq = params_.max_corr_dist * params_.max_corr_dist;
      reg.optimizer.max_iterations = params_.max_iter;

      small_gicp::RegistrationResult res;
      if (params_.type == RegistrationType::VGICP) {
        res = reg.align(*voxelmap_, *source, *voxelmap_, toIsometry(T_init_M_B));
      } else {
        res = reg.align(*target_points_, *source, *target_tree_, toIsometry(T_init_M_B));
      }

      out.converged   = res.converged;
      out.iterations  = static_cast<int>(res.iterations);
      out.T           = fromIsometry(res.T_target_source);
      out.H           = res.H;
      out.num_inliers = static_cast<int>(res.num_inliers);
      out.num_source  = static_cast<int>(source->size());
      out.num_target  = static_cast<int>(target_points_ ? target_points_->size() : 0);
      out.inlier_ratio = out.num_source > 0
                             ? static_cast<double>(out.num_inliers) / out.num_source
                             : 0.0;
      out.cov = covarianceFrom(res.H, res.error, out.num_inliers, params_);
      analyseDegeneracy(res.H, params_, &out);
      out.rmse = metricRmse(*source, res.T_target_source);
    });

    out.time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
  }

  const char* name() const override {
    return params_.type == RegistrationType::VGICP ? "small_gicp/VGICP" : "small_gicp/GICP";
  }

private:
  /// small_gicp's `error` is Mahalanobis-weighted, but every gate in the design
  /// doc is in metres. Recompute a plain point-to-point RMS over the points that
  /// actually found a correspondence.
  ///
  /// This always searches the target POINT cloud, never the Gaussian voxel map,
  /// even in VGICP mode. Searching the voxel map returns the distance to a voxel
  /// centroid, which has a floor of roughly a quarter of the voxel size however
  /// perfect the alignment is: with target_voxel = 0.5 m a flawless fit still
  /// reports ~0.2 m and trips a 0.2 m gate. Measuring against the points keeps
  /// `rmse` meaning the same thing for every backend, which is what lets the
  /// gate thresholds be written once in metres.
  double metricRmse(const small_gicp::PointCloud& source, const Eigen::Isometry3d& T) const {
    const double max_sq = params_.max_corr_dist * params_.max_corr_dist;
    double sum_sq = 0.0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < source.size(); ++i) {
      const Eigen::Vector4d q = T * source.point(i);
      std::size_t idx = 0;
      double sq = 0.0;
      if (small_gicp::traits::nearest_neighbor_search(*target_tree_, q, &idx, &sq) <= 0) continue;
      if (sq > max_sq) continue;
      sum_sq += sq;
      ++n;
    }
    return n > 0 ? std::sqrt(sum_sq / static_cast<double>(n))
                 : std::numeric_limits<double>::infinity();
  }

  RegParams params_;
  tbb::task_arena arena_;
  std::uint64_t target_version_ = 0;
  std::shared_ptr<small_gicp::PointCloud> target_points_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;
  std::shared_ptr<small_gicp::GaussianVoxelMap> voxelmap_;
};

// ---------------------------------------------------------------------------
// PCL adapter (reference / fallback)
// ---------------------------------------------------------------------------

/// pcl::GeneralizedIterativeClosestPoint does not expose its linearized system,
/// so this adapter cannot report a real information matrix. It reports
/// information_is_real=false and a conservative covariance scaled by the
/// observed RMSE, which is enough to A-B the pose accuracy of the two backends
/// but not to weight them identically in the EKF.
class PclGicpRegistrar final : public IRegistrar {
public:
  explicit PclGicpRegistrar(const RegParams& params) : params_(params) {}

  void setTarget(const CloudXYZ::ConstPtr& cloud_M, std::uint64_t version) override {
    if (!cloud_M || cloud_M->empty()) return;
    if (version != 0 && version == target_version_) return;

    auto target = CloudXYZ::Ptr(new CloudXYZ());
    if (params_.target_voxel > 0.0f) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      const float leaf = params_.target_voxel * 0.5f;
      vg.setLeafSize(leaf, leaf, leaf);
      vg.setInputCloud(cloud_M);
      vg.filter(*target);
    } else {
      *target = *cloud_M;
    }
    target_ = target;
    target_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    target_tree_->setInputCloud(target_);
    target_version_ = version;
  }

  std::uint64_t targetVersion() const override { return target_version_; }
  bool hasTarget() const override { return target_ && !target_->empty(); }

  RegResult align(const CloudXYZ& source_B, const BASIC::SE3& T_init_M_B) override {
    RegResult out;
    out.T = T_init_M_B;
    out.information_is_real = false;
    if (!hasTarget() || source_B.empty()) return out;

    const auto t0 = std::chrono::steady_clock::now();

    auto source = CloudXYZ::Ptr(new CloudXYZ());
    if (params_.source_voxel > 0.0f) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(params_.source_voxel, params_.source_voxel, params_.source_voxel);
      vg.setInputCloud(source_B.makeShared());
      vg.filter(*source);
    } else {
      *source = source_B;
    }
    if (source->size() < 20) return out;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target_);
    // GICP builds its own pcl::search::KdTree internally; target_tree_ is a
    // FLANN tree kept only for the RMSE/inlier pass below, which must use the
    // same definition as the small_gicp adapter for the two to be comparable.
    gicp.setMaxCorrespondenceDistance(params_.max_corr_dist);
    gicp.setMaximumIterations(params_.max_iter);
    gicp.setCorrespondenceRandomness(params_.cov_neighbors);

    Eigen::Matrix4f init = Eigen::Matrix4f::Identity();
    init.block<3, 3>(0, 0) = T_init_M_B.R_.template cast<float>();
    init.block<3, 1>(0, 3) = T_init_M_B.t_.template cast<float>();

    CloudXYZ aligned;
    gicp.align(aligned, init);

    out.converged  = gicp.hasConverged();
    out.iterations = params_.max_iter;  // PCL does not report the count it used
    Eigen::Matrix4f T = gicp.getFinalTransformation();
    out.T = BASIC::SE3(BASIC::M3(T.block<3, 3>(0, 0).cast<BASIC::scalar>()),
                       BASIC::V3(T.block<3, 1>(0, 3).cast<BASIC::scalar>()));
    out.num_source = static_cast<int>(source->size());
    out.num_target = static_cast<int>(target_ ? target_->size() : 0);

    // Metric RMSE and inlier count over the aligned source, same definition the
    // small_gicp adapter uses so the two are comparable.
    const double max_sq = params_.max_corr_dist * params_.max_corr_dist;
    CloudXYZ transformed;
    pcl::transformPointCloud(*source, transformed, T);
    std::vector<int> idx(1);
    std::vector<float> sq(1);
    double sum_sq = 0.0;
    int n = 0;
    for (const auto& p : transformed) {
      if (target_tree_->nearestKSearch(p, 1, idx, sq) <= 0) continue;
      if (sq[0] > max_sq) continue;
      sum_sq += sq[0];
      ++n;
    }
    out.num_inliers  = n;
    out.inlier_ratio = out.num_source > 0 ? static_cast<double>(n) / out.num_source : 0.0;
    out.rmse = n > 0 ? std::sqrt(sum_sq / n) : std::numeric_limits<double>::infinity();

    // Conservative isotropic stand-in, floored and inflated by the fit quality.
    const double s_t = std::max(params_.cov_floor_t, out.rmse);
    const double s_r = std::max(params_.cov_floor_r, out.rmse * 0.1);
    out.cov.setZero();
    out.cov.topLeftCorner<3, 3>()     = Eigen::Matrix3d::Identity() * (s_r * s_r);
    out.cov.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * (s_t * s_t);
    out.H = out.cov.inverse();
    // No real Hessian means no meaningful eigen-analysis; report "not degenerate"
    // rather than inventing observability the backend does not have.
    out.eig_min_t = out.eig_max_t = out.eig_min_r = out.eig_max_r = 0.0;

    out.time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
  }

  const char* name() const override { return "pcl/GICP"; }

private:
  RegParams params_;
  std::uint64_t target_version_ = 0;
  CloudXYZ::Ptr target_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr target_tree_;
};

}  // namespace

std::unique_ptr<IRegistrar> IRegistrar::Create(const RegParams& params) {
  switch (params.type) {
    case RegistrationType::PCL_GICP:
      LOG(WARNING) << " ---> [MapLoc] using the PCL GICP fallback: it reports no "
                      "information matrix, so observations are weighted conservatively";
      return std::make_unique<PclGicpRegistrar>(params);
    case RegistrationType::GICP:
    case RegistrationType::VGICP:
    default:
      return std::make_unique<SmallGicpRegistrar>(params);
  }
}

}  // namespace loc
}  // namespace LI2Sup
