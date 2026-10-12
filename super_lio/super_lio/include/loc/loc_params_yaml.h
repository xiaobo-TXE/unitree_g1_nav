/**
 * @file loc_params_yaml.h
 * @brief Read lio.maploc.* out of a params yaml without ROS.
 *
 * Why this exists (H1): the node reads its parameters through rclcpp, while
 * loc_replay and loc_probe_reg used LocParams::Defaults(). That divergence
 * already cost a round of testing -- a yaml that restated stale values meant
 * every offline test reported a fix working while the node ran without it. One
 * file, both paths.
 *
 * The parser is deliberately minimal: the config is a flat map of
 * `lio.maploc.<name>: <scalar>` under a fixed two-level header, which is all
 * these tools need, and it avoids putting yaml-cpp on the board's dependency
 * list. Unknown keys are ignored; anything absent keeps the C++ default, so the
 * two sources cannot silently disagree about which knobs exist.
 */
#ifndef LOC_PARAMS_YAML_H_
#define LOC_PARAMS_YAML_H_

#include <string>

#include "loc/loc_params.h"

namespace LI2Sup {
namespace loc {

/// Overlays `lio.maploc.*` entries from `path` onto `p`. Angles in the yaml are
/// degrees (as in the ROS path) and are converted here. Returns false if the
/// file cannot be opened; `p` is then untouched.
bool LoadLocParamsFromYaml(const std::string& path, LocParams* p);

/// One line per knob, in the same order and wording the node logs at startup,
/// so the two can be diffed.
std::string DescribeLocParams(const LocParams& p);

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_PARAMS_YAML_H_
