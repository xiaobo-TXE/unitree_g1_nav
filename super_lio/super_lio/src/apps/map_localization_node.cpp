/**
 * @file map_localization_node.cpp
 * @brief Super-LIO with map localization. Same shape as relocation_node.cpp:
 *        one node, one 2 ms timer, a single-threaded executor. The localization
 *        backend runs on its own thread, started inside SuperLIOMapLoc::init().
 */

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros/ROSWrapper.h"
#include "ros/loc_publisher.h"
#include "lio/super_lio_maploc.h"

using namespace LI2Sup;

namespace {

/// Send glog to a real file as well as the terminal, and prune old runs.
///
/// Why this exists: glog before InitGoogleLogging() writes to stderr and
/// nowhere else, and the launch file runs the node with output='screen', which
/// makes ros2 keep no per-process file either. A two-hour board run therefore
/// left ~/.ros/log/<run>/launch.log and not one line of what MapLoc did. The
/// board is reached through a pasted log; a log that does not survive the run
/// is the same as no log at all.
///
/// The directory comes from the environment rather than a ROS parameter
/// because the first LOG() happens inside the node constructor, before any
/// parameter can be read.
///
/// @return the directory logs are being written to.
std::string setupFileLogging(const char* argv0) {
  namespace fs = std::filesystem;

  const char* env  = std::getenv("MAPLOC_LOG_DIR");
  const char* home = std::getenv("HOME");
  std::string dir = (env && *env) ? std::string(env)
                                  : std::string(home ? home : "/tmp") + "/.ros/log/maploc";

  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) dir = "/tmp";   // never let logging config stop the node from running

  // Prune old runs. glog rotates by size but never deletes, and glog 0.4 (what
  // Ubuntu 20.04 ships, i.e. the board) has no EnableLogCleaner, so do it here.
  int keep_days = 7;
  if (const char* k = std::getenv("MAPLOC_LOG_KEEP_DAYS")) {
    const int v = std::atoi(k);
    if (v > 0) keep_days = v;
  }
  const auto cutoff = fs::file_time_type::clock::now() - std::chrono::hours(24 * keep_days);
  for (const auto& e : fs::directory_iterator(dir, ec)) {
    if (ec) break;
    std::error_code e2;
    if (!e.is_regular_file(e2) || e2) continue;
    if (fs::last_write_time(e.path(), e2) < cutoff && !e2) fs::remove(e.path(), e2);
  }

  FLAGS_log_dir = dir;
  FLAGS_alsologtostderr = true;   // the terminal keeps behaving exactly as before
  // Flush every line. A backend killed with SIGKILL (which is how a stuck
  // launch ends) must not take the last minutes of the log with it -- and the
  // last minutes are the interesting ones.
  FLAGS_logbufsecs = 0;
  FLAGS_max_log_size = 64;                  // MB per file, then glog rolls
  FLAGS_stop_logging_if_full_disk = true;   // a full board disk must not wedge the node
  google::InitGoogleLogging(argv0);
  return dir;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string log_dir = setupFileLogging(argv[0]);
  // Deliberately no std::signal(SIGINT, ...) here. Installing one after
  // rclcpp::init replaces rclcpp's own handler, so spin() never returns on
  // Ctrl-C: the process has to be killed, finish() never runs, and the
  // trajectory file is never written. Let rclcpp handle the signal and shut
  // the executor down cleanly.
  rclcpp::init(argc, argv);
  // First line of every run, so the log says where to find itself.
  LOG(INFO) << GREEN << " ---> [MapLoc] logging to " << log_dir
            << " (map_localization_node.INFO is a symlink to this run; "
               "set MAPLOC_LOG_DIR to move it, MAPLOC_LOG_KEEP_DAYS to keep more)"
            << RESET;

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();

  // The maploc parameters live on the same node as the LIO ones, and must be
  // read before the localizer is constructed.
  const loc::LocParams params = LoadLocParamFromRos(*data_wrapper);

  auto lio = std::make_shared<SuperLIOMapLoc>(params);
  lio->setROSWrapper(data_wrapper);
  lio->init();

  auto timer = data_wrapper->create_wall_timer(
    std::chrono::milliseconds(2),
    [lio]() { lio->process(); },
    data_wrapper->getSensorCallbackGroup()
  );

  rclcpp::spin(data_wrapper);

  lio->finish();
  lio->saveMap();
  lio->printTimeRecord();

  // After every summary and the timing table, so the last thing in the file is
  // where the file is.
  LOG(INFO) << " ---> [MapLoc] log written to " << log_dir;
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
