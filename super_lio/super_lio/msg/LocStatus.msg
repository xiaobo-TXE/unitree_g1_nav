# Status of the map-localization backend (MapLoc).
# Published on every match attempt plus a 1 Hz heartbeat.

std_msgs/Header header

# 0 WAIT_ODOM  1 WAIT_INIT  2 INITIALIZING  3 TRACKING  4 LOST  5 FAILED
#
# TRACKING and LOST are both statements about the MAP, and both mean the
# frontend odometry is sound: LOST deliberately keeps publishing, dead-reckoning
# from the last good drift. FAILED means the FRONTEND is unusable -- it reported
# a pose the robot cannot be in -- and in that state nothing is published at
# all: no /loc/odom, no /loc/robo/odom, no /loc/imu/odom, and no map->world
# transform. A consumer that keeps planning on the last pose it saw will be
# planning on a pose that is leaving; treat the absence as a stop condition,
# not as "the last value still holds".
uint8 state

bool valid          # the published /loc pose is usable
bool jump           # this output snapped instead of ramping
uint32 jump_seq     # increments on each confirmed (re)initialization

geometry_msgs/Pose t_map_odom   # T_M_O, the drift being applied

float32 inlier_ratio
float32 rmse                # metric, inliers only [m]
# Body-frame correction the LAST match asked for -- i.e. how far that
# observation would have moved the robot. This is the gating quantity, reported
# whether the observation was accepted or rejected. It is NOT the amount the
# output limiter still has left to absorb.
float32 correction_t        # [m]
float32 correction_r        # [deg]
float32 last_match_age      # [s] since the last ACCEPTED match
float32 match_time_ms
float32 submap_build_ms
int32   consecutive_rejects
string  last_reject         # gate that refused the last observation
int32[] submap_ids
int32   queue_dropped       # frontend frames the backend could not keep up with
