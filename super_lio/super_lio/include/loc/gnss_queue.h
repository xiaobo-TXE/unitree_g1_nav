/**
 * @file gnss_queue.h
 * @brief Bounded T0 -> T1 handover for GNSS epochs.
 *
 * Same contract as FrameQueue, with one deliberate difference: this one does
 * NOT block T1. The backend loop is driven by frontend frames, and a GNSS drain
 * is a thing it does on the way past, not a thing it waits for.
 *
 * Overflow drops the OLDEST entry. For a positioning fix, the newest epoch is
 * the only one with a chance of still being inside max_latency_s; keeping a
 * backlog of stale fixes and dropping fresh ones would be exactly backwards.
 */
#ifndef LOC_GNSS_QUEUE_H_
#define LOC_GNSS_QUEUE_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "loc/gnss_types.h"

namespace LI2Sup {
namespace loc {

class GnssQueue {
public:
  explicit GnssQueue(std::size_t capacity = 10)
      : capacity_(capacity > 0 ? capacity : 1) {}

  /// T0. Returns false when an older fix had to be shed to make room.
  bool push(const GnssFix& fix);

  /// T1. Moves at most `max_n` oldest entries into *out (appending). Returns
  /// how many were taken. Never blocks and never waits for a producer.
  std::size_t popSome(std::vector<GnssFix>* out, std::size_t max_n);

  /// T1. Puts entries back at the FRONT, in order, for epochs the backend
  /// cannot judge yet (the odometry has not reached their measurement time).
  /// Appending them instead would reorder the queue behind fixes that arrived
  /// while the batch was being examined.
  void requeueFront(const std::vector<GnssFix>& fixes);

  void clear();
  std::size_t size() const;
  std::uint32_t dropped() const;

private:
  mutable std::mutex mutex_;
  std::deque<GnssFix> q_;
  std::size_t capacity_;
  std::uint32_t dropped_ = 0;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_GNSS_QUEUE_H_
