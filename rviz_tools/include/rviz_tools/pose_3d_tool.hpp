#pragma once

#include <memory>

#include <OgreVector3.h>

#include "rviz_common/tool.hpp"
#include "rviz_rendering/objects/arrow.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/publisher.hpp"

namespace rviz_tools {

class Pose3DTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  bool get3DPoint(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & out);
  bool getDirectionPoint(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & out);

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  std::shared_ptr<rviz_rendering::Arrow> arrow_;

  bool got_position_;
  Ogre::Vector3 pos_;
};

}  // namespace rviz_tools
