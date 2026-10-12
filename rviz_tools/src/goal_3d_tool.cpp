#include "rviz_tools/goal_3d_tool.hpp"

#include <cmath>

#include <OgreCamera.h>
#include <OgreMatrix4.h>
#include <OgrePlane.h>
#include <OgreQuaternion.h>
#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#include <OgreRenderTarget.h>

#include <GL/gl.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_rendering/render_window.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace rviz_tools {

Goal3DTool::Goal3DTool()
: arrow_(nullptr), got_position_(false)
{
  shortcut_key_ = 'g';
}

Goal3DTool::~Goal3DTool() = default;

void Goal3DTool::onInitialize()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_3d", 1);

  arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 0.8f, 0.05f, 0.2f, 0.1f);
  arrow_->setColor(1.0f, 0.5f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void Goal3DTool::activate() {}

void Goal3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
  got_position_ = false;
}

bool Goal3DTool::get3DPoint(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & out)
{
  using rviz_rendering::RenderWindowOgreAdapter;

  rviz_rendering::RenderWindow * rw = event.panel->getRenderWindow();
  Ogre::Camera *       cam = RenderWindowOgreAdapter::getOgreCamera(rw);
  Ogre::Viewport *     vp  = RenderWindowOgreAdapter::getOgreViewport(rw);
  Ogre::RenderTarget * rt  = vp->getTarget();

  int vp_left = vp->getActualLeft();
  int vp_top  = vp->getActualTop();
  int vp_w    = vp->getActualWidth();
  int vp_h    = vp->getActualHeight();
  int rt_h    = static_cast<int>(rt->getHeight());

  // OpenGL origin is bottom-left; screen/RViz origin is top-left.
  int gl_x = vp_left + event.x;
  int gl_y = rt_h - (vp_top + event.y) - 1;

  GLint prev_read = GL_BACK;
  glGetIntegerv(GL_READ_BUFFER, &prev_read);
  glReadBuffer(GL_FRONT);

  GLfloat depth = 1.0f;
  glReadPixels(gl_x, gl_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  GLenum err = glGetError();

  glReadBuffer(prev_read);

  if (err == GL_NO_ERROR && depth < 1.0f - 1e-4f) {
    float ndcX =  2.0f * static_cast<float>(event.x) / vp_w - 1.0f;
    float ndcY =  1.0f - 2.0f * static_cast<float>(event.y) / vp_h;
    float ndcZ =  depth * 2.0f - 1.0f;

    Ogre::Matrix4 VP    = cam->getProjectionMatrixWithRSDepth() * cam->getViewMatrix();
    Ogre::Matrix4 VPinv = VP.inverse();

    Ogre::Vector4 clip(ndcX, ndcY, ndcZ, 1.0f);
    Ogre::Vector4 world = VPinv * clip;

    if (std::fabs(static_cast<float>(world.w)) > 1e-6f) {
      world /= world.w;
      out = Ogre::Vector3(world.x, world.y, world.z);
      return true;
    }
  }

  // Fallback: intersect camera ray with the horizontal plane at z = 0.
  float screen_x = static_cast<float>(event.x) / vp_w;
  float screen_y = static_cast<float>(event.y) / vp_h;
  Ogre::Ray ray = cam->getCameraToViewportRay(screen_x, screen_y);
  Ogre::Plane ground(Ogre::Vector3::UNIT_Z, 0.0f);
  auto result = ray.intersects(ground);
  if (result.first) {
    out = ray.getPoint(result.second);
    return true;
  }
  return false;
}

bool Goal3DTool::getDirectionPoint(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & out)
{
  using rviz_rendering::RenderWindowOgreAdapter;

  rviz_rendering::RenderWindow * rw = event.panel->getRenderWindow();
  Ogre::Camera * cam = RenderWindowOgreAdapter::getOgreCamera(rw);
  Ogre::Viewport * vp = RenderWindowOgreAdapter::getOgreViewport(rw);

  float screen_x = static_cast<float>(event.x) / vp->getActualWidth();
  float screen_y = static_cast<float>(event.y) / vp->getActualHeight();
  Ogre::Ray ray = cam->getCameraToViewportRay(screen_x, screen_y);
  Ogre::Plane plane(Ogre::Vector3::UNIT_Z, pos_.z);
  auto result = ray.intersects(plane);
  if (result.first) {
    out = ray.getPoint(result.second);
    return true;
  }
  return false;
}

int Goal3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  // rviz_rendering::Arrow points along local -Z; rotate to align with ROS heading (+X).
  static const Ogre::Quaternion orient_correction(
    Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

  if (!got_position_) {
    if (event.leftDown()) {
      if (get3DPoint(event, pos_)) {
        got_position_ = true;
        arrow_->setPosition(pos_);
        arrow_->getSceneNode()->setVisible(true);
      }
    }
    return Render;
  }

  Ogre::Vector3 cur = pos_;
  getDirectionPoint(event, cur);

  Ogre::Vector3 dir = cur - pos_;
  if (dir.squaredLength() > 1e-6f) {
    double yaw = std::atan2(dir.y, dir.x);
    Ogre::Quaternion q;
    q.FromAngleAxis(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z);
    arrow_->setOrientation(q * orient_correction);
  }

  if (event.leftUp()) {
    double yaw = std::atan2(static_cast<double>(dir.y), static_cast<double>(dir.x));

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = context_->getClock()->now();
    msg.header.frame_id = context_->getFixedFrame().toStdString();

    msg.pose.position.x = pos_.x;
    msg.pose.position.y = pos_.y;
    msg.pose.position.z = pos_.z;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.orientation.w = std::cos(yaw / 2.0);

    pub_->publish(msg);
    RCLCPP_INFO(
      rclcpp::get_logger("Goal3DTool"),
      "[Goal3DTool] Published goal: (%.2f, %.2f, %.2f) yaw=%.2f",
      pos_.x, pos_.y, pos_.z, yaw);

    arrow_->getSceneNode()->setVisible(false);
    got_position_ = false;
    return Render | Finished;
  }

  return Render;
}

}  // namespace rviz_tools

PLUGINLIB_EXPORT_CLASS(rviz_tools::Goal3DTool, rviz_common::Tool)
