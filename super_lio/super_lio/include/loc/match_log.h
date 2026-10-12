/**
 * @file match_log.h
 * @brief Per-match CSV trace of the backend, for offline diagnosis.
 *
 * Why this exists: /loc/status carries only the latest match, and the
 * interesting questions ("is the jitter in the LIO or in the match?", "what did
 * H look like at the eight bad spots?") need every match, with the raw
 * registration pose BEFORE the EKF touches it. Reconstructing that from a topic
 * recording is lossy; writing it here is exact.
 *
 * `T_match` is `reg.T` straight from the registrar — no gating, no filtering.
 * Comparing it against the PGO trajectory at the same timestamp is what
 * separates "the match is wrong" from "the estimator smoothed it wrong".
 *
 * Off by default (empty path). Cost when on: one formatted line per match,
 * i.e. a few per second, on the backend thread.
 */
#ifndef LOC_MATCH_LOG_H_
#define LOC_MATCH_LOG_H_

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "basic/alias.h"
#include "basic/Manifold.h"

namespace LI2Sup {
namespace loc {

struct RegResult;
struct UpdateReport;

class MatchLogger {
public:
  /// Opens `path` and writes the header. An empty path disables logging and
  /// every later call becomes a no-op.
  explicit MatchLogger(const std::string& path);
  ~MatchLogger();

  bool enabled() const { return file_.is_open(); }

  /// One row. `state` is the state the match was attempted in; `T_O_B` is the
  /// odometry pose snapshotted with the scan.
  void write(double t, const char* state, bool accepted, const char* reason,
             const RegResult& reg, const UpdateReport& rep,
             const BASIC::SE3& T_O_B, std::uint64_t submap_version,
             const std::vector<int>& submap_ids);

private:
  std::ofstream file_;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_MATCH_LOG_H_
