# GNSS health for the map-localization module.
#
# Deliberately a SEPARATE message from LocStatus, and LocStatus gains nothing.
# Its consumers live outside this workspace and adding a field to it would
# force them to rebuild; more importantly, lidar health and GNSS health must
# stay independent quantities. LocStatus/consecutive_rejects and last_match_age
# keep meaning "the map match", whatever GNSS is doing. A GNSS outage cannot
# create a LOST, and a GNSS success cannot conceal one.
#
# Published at the same cadence as /loc/status, from the node thread, so it
# keeps ticking even when the backend thread has nothing to do.

std_msgs/Header header

# 0 disabled  1 record_only  2 assist_init  3 fuse_xy
# Only 0 and 1 are implemented. 2 and 3 are parsed and refused, not silently
# accepted: a mode that claims to fuse without a map<->ENU calibration would be
# fusing a position that does not exist.
uint8 mode

# 0 disabled  1 no_georef  2 no_signal  3 probation  4 admitted
# Orthogonal to LocStatus/state. Slow to promote, fast to demote.
uint8 admission

bool georef_valid      # a map<->ENU calibration is loaded and matches this map
bool receiving         # a fix arrived within fix_timeout_s

float64 last_fix_stamp    # measurement time of the newest enqueued epoch
float64 last_fuse_stamp   # newest accepted update; 0 while nothing fuses
float32 fix_age           # [s] since last_fix_stamp, on the node clock
float32 fuse_age          # [s]; NaN while nothing fuses
float32 latency           # [s] arrival minus measurement. Live only: under bag
                          # replay the node clock is not the bag's clock.

# Why the newest epoch did not become an observation. In record_only the
# steady-state answer is no_georef, which is a statement about this build, not
# about the receiver -- anything else names a real problem with the data.
#  0 none        1 disabled       2 no_fix       3 no_satellites  4 not_finite
#  5 out_of_range 6 bad_covariance 7 stale_clock 8 duplicate_epoch 9 rate_limited
# 10 frozen      11 queue_full    12 no_georef  13 too_old      14 odom_gap
# 15 time_jump
uint8 reject_reason
int32 reject_streak

# ---- ingest counters, from the ROS adapter (node thread) ----
uint32 n_in            # NavSatFix callbacks entered
uint32 n_no_fix        # status.status < STATUS_FIX
uint32 n_not_finite    # NaN/Inf or out of range -- these must never reach a gate
uint32 n_bad_cov       # covariance entries replaced by the configured floor
uint32 n_dedup         # bit-identical repeats of the previous epoch
uint32 n_rate_limited
uint32 n_enqueued
uint32 n_dropped       # shed at a full queue
uint32 n_extended      # /extended_fix callbacks entered

# ---- backend counters (localization thread) ----
uint32 n_examined      # epochs the backend drained
uint32 n_waiting       # queued right now
uint32 n_fused         # 0 until a fusing rung ships

# ---- quality ----
# -1 means the field was never supplied, which is itself the finding: without
# /extended_fix there is no way to tell a healthy receiver from a disconnected
# one, because NavSatFix carries no satellite count.
int32   satellites_visible
int32   satellites_used
float32 hdop                  # -1 = unknown

# Consecutive epochs reporting a bit-identical latitude/longitude, and the
# worst run this session. A moving receiver must break these. A large value
# while the robot is driving means the position is a fallback constant, not a
# measurement -- observed at 2095 on 2026-09-22 with the antenna disconnected.
int32 frozen_run
int32 frozen_run_max

# Horizontal 1 sigma [m] that a fusing rung WOULD use, and where it came from.
# sigma_h_from_receiver is broken out because on this hardware it is always
# false: both horizontal covariance entries are NaN on every epoch, so the
# number is the configured floor. "We do not actually know the accuracy" has to
# be visible on the topic rather than inferred from a config file.
float32 sigma_h
bool    sigma_h_from_receiver

# ---- fusion diagnostics; NaN until a fusing rung ships ----
float32 residual_h          # [m] observed minus predicted, horizontal
float32 chi2
float32 body_correction_t   # [m] how far this observation would pull the robot
