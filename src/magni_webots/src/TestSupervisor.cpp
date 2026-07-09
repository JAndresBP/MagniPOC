/*
Ground-truth pose publisher for local/integration testing. Mirrors NodeRemover's
root-children traversal (see NodeRemover.cpp) but looks up nodes by name instead
of hardcoding "magni", and does not filter by node type since tracked nodes may
be Robot (the base) or Solid (a manipulable object) in future tests.
*/
#include "magni_webots/TestSupervisor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <webots/supervisor.h>
#include <cmath>
#include <sstream>

namespace magni_webots
{
  TestSupervisor::TestSupervisor()
      : logger_(rclcpp::get_logger("magni_webots::TestSupervisor")), node_(nullptr)
  {
  }

  void TestSupervisor::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    node_ = node;
    logger_ = node->get_logger();
    RCLCPP_INFO(logger_, "starting test supervisor plugin");

    std::string track_nodes_param;
    auto it = parameters.find("track_nodes");
    if (it != parameters.end())
    {
      track_nodes_param = it->second;
    }

    std::stringstream ss(track_nodes_param);
    std::string name;
    while (std::getline(ss, name, ','))
    {
      // trim surrounding whitespace
      const auto first = name.find_first_not_of(" \t");
      const auto last = name.find_last_not_of(" \t");
      if (first == std::string::npos)
        continue;
      name = name.substr(first, last - first + 1);

      TrackedNode tracked;
      tracked.name = name;
      tracked.node_ref = nullptr;
      tracked.pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/test_supervisor/" + name + "/pose", 10);
      tracked_nodes_.push_back(tracked);
      RCLCPP_INFO(logger_, "tracking node '%s'", name.c_str());
    }

    if (tracked_nodes_.empty())
    {
      RCLCPP_WARN(logger_, "no 'track_nodes' parameter provided; test supervisor will publish nothing");
    }
  }

  void TestSupervisor::step()
  {
    for (auto &tracked : tracked_nodes_)
    {
      if (!tracked.node_ref)
      {
        tracked.node_ref = find_node_by_name(tracked.name);
      }
      if (tracked.node_ref)
      {
        publish_pose(tracked);
      }
    }
  }

  WbNodeRef TestSupervisor::find_node_by_name(const std::string &name)
  {
    WbNodeRef root = wb_supervisor_node_get_root();
    if (!root)
    {
      RCLCPP_WARN(logger_, "unable to get supervisor root node");
      return nullptr;
    }

    WbFieldRef children_field = wb_supervisor_node_get_field(root, "children");
    if (!children_field)
    {
      RCLCPP_WARN(logger_, "unable to get children field from root");
      return nullptr;
    }

    const int child_count = wb_supervisor_field_get_count(children_field);
    for (auto i = 0; i < child_count; ++i)
    {
      WbNodeRef child = wb_supervisor_field_get_mf_node(children_field, i);
      if (!child)
        continue;

      WbFieldRef name_field = wb_supervisor_node_get_field(child, "name");
      if (!name_field)
        continue;

      const char *child_name = wb_supervisor_field_get_sf_string(name_field);
      if (child_name && name == child_name)
      {
        RCLCPP_INFO(logger_, "found tracked node '%s'", name.c_str());
        return child;
      }
    }

    return nullptr;
  }

  void TestSupervisor::publish_pose(TrackedNode &tracked)
  {
    const double *position = wb_supervisor_node_get_position(tracked.node_ref);
    const double *orientation = wb_supervisor_node_get_orientation(tracked.node_ref);
    if (!position || !orientation)
      return;

    // orientation is a row-major 3x3 rotation matrix; convert to quaternion.
    const double m00 = orientation[0], m01 = orientation[1], m02 = orientation[2];
    const double m10 = orientation[3], m11 = orientation[4], m12 = orientation[5];
    const double m20 = orientation[6], m21 = orientation[7], m22 = orientation[8];

    const double trace = m00 + m11 + m22;
    double qw, qx, qy, qz;
    if (trace > 0.0)
    {
      const double s = std::sqrt(trace + 1.0) * 2.0;
      qw = 0.25 * s;
      qx = (m21 - m12) / s;
      qy = (m02 - m20) / s;
      qz = (m10 - m01) / s;
    }
    else if (m00 > m11 && m00 > m22)
    {
      const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
      qw = (m21 - m12) / s;
      qx = 0.25 * s;
      qy = (m01 + m10) / s;
      qz = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
      const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
      qw = (m02 - m20) / s;
      qx = (m01 + m10) / s;
      qy = 0.25 * s;
      qz = (m12 + m21) / s;
    }
    else
    {
      const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
      qw = (m10 - m01) / s;
      qx = (m02 + m20) / s;
      qy = (m12 + m21) / s;
      qz = 0.25 * s;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    if (node_)
      msg.header.stamp = node_->get_clock()->now();
    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];
    msg.pose.orientation.w = qw;
    msg.pose.orientation.x = qx;
    msg.pose.orientation.y = qy;
    msg.pose.orientation.z = qz;
    tracked.pose_publisher->publish(msg);
  }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(magni_webots::TestSupervisor, webots_ros2_driver::PluginInterface)
