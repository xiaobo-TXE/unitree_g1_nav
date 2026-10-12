/**
 * @file odom_history.h
 * @brief A short, bounded history of frontend poses, interpolatable by time.
 *
 * Why this exists rather than reusing something: a GNSS epoch is timestamped at
 * the MEASUREMENT instant, and by the time it reaches the backend the robot has
 * moved. Fusing it against "the newest odometry pose" silently bakes in
 * however far the robot walked in the meantime. This answers the only question
 * that matters — "where was the body at time t?" — and answers it with a
 * reason when it cannot.
 *
 * Deliberately NOT the frame queue: FrameQueue drops its oldest entry under
 * load and ScanAccumulator::build() empties the accumulator, so both have holes
 * exactly where a time lookup needs continuity. This keeps poses only, so ten
 * seconds of them costs a few kilobytes and nothing is ever dropped for load.
 *
 * Threading: T0 writes (SuperLIOMapLoc::Output), T1 reads (drainGnss). One
 * mutex, same contract as FrameQueue. Neither side blocks on the other's work.
 *
 * Interpolation: linear in translation, SLERP in rotation, in double. At 10 Hz
 * and 1.5 m/s the samples are 15 cm apart, so the interpolation error is far
 * below the measured 0.47 s of GNSS timestamp uncertainty; there is no reason
 * to hang this off the IMU-rate hook.
 */
#ifndef LOC_ODOM_HISTORY_H_
#define LOC_ODOM_HISTORY_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "basic/alias.h"
#include "basic/Manifold.h"

namespace LI2Sup {
namespace loc {

class OdomHistory {
public:
  /// Why a lookup did not produce a pose. The distinction between kTooNew and
  /// everything else is the one that matters to a caller: kTooNew means WAIT
  /// (the frontend has not caught up yet and will), the rest mean give up.
  /// Substituting the nearest pose for any of these is the bug this enum
  /// exists to make impossible to write by accident.
  enum class Sample : std::uint8_t {
    kOk = 0,
    kEmpty,    ///< nothing recorded yet
    kTooOld,   ///< t precedes the oldest retained pose
    kTooNew,   ///< t is still in the future; retry later
    kGap,      ///< the bracketing samples are further apart than max_gap_s
    kReset     ///< t falls before a detected clock reset (bag replay)
  };

  static const char* toString(Sample s);

  /// span_s bounds the age of what is kept; capacity bounds the memory when the
  /// frontend runs faster than expected. Whichever binds first wins.
  explicit OdomHistory(double span_s = 10.0, double max_gap_s = 0.5,
                       std::size_t capacity = 400);

  /// T0. Never blocks, never allocates beyond the deque, never drops for load.
  ///
  /// A stamp that goes BACKWARDS by more than kResetJumpS is read as a clock
  /// reset (a bag replay looping, or a second `ros2 bag play`), not as an
  /// out-of-order frame: the buffer is emptied and every later lookup of an
  /// earlier time answers kReset. Associating a fix from the previous replay
  /// pass with a pose from this one produces a confident, wrong answer.
  void push(double t, const BASIC::SE3& T_O_B);

  /// T1. Fills *T_O_B only on kOk; leaves it untouched otherwise.
  Sample sample(double t, BASIC::SE3* T_O_B) const;

  /// Forget everything. `reset_stamp` marks the boundary: lookups before it
  /// answer kReset rather than kTooOld, so the caller can tell "your data is
  /// from another run" from "your data is merely late".
  void clear(double reset_stamp);

  double newest() const;
  double oldest() const;
  std::size_t size() const;
  /// How many clock resets push() has detected. Diagnostic.
  std::uint32_t resets() const;

  /// Backwards stamp jump read as a clock reset rather than reordering [s].
  static constexpr double kResetJumpS = 1.0;

private:
  struct Entry {
    double t;
    BASIC::SE3 T;
  };

  mutable std::mutex mutex_;
  std::deque<Entry> q_;
  double span_s_;
  double max_gap_s_;
  std::size_t capacity_;
  double reset_stamp_ = 0.0;
  bool   have_reset_ = false;
  std::uint32_t resets_ = 0;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_ODOM_HISTORY_H_
