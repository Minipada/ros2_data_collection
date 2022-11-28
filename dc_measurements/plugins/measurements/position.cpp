#include "position.hpp"

#include <chrono>
#include <memory>

namespace dc_measurements
{

Position::Position() : dc_measurements::Measurement()
{
}

Position::~Position() = default;

void Position::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".robot_base_frame",
                                               rclcpp::ParameterValue("base_link"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".transform_tolerance",
                                               rclcpp::ParameterValue(static_cast<float>(0.1)));
  node->get_parameter(measurement_name_ + ".global_frame", global_frame_);
  node->get_parameter(measurement_name_ + ".robot_base_frame", robot_base_frame_);
  node->get_parameter(measurement_name_ + ".transform_timeout", transform_timeout_);
}

void Position::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "position.json");
  }
}

dc_interfaces::msg::StringStamped Position::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(pose, *tf_, global_frame_, robot_base_frame_, transform_timeout_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
  }
  else
  {
    tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                      pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    data_json["x"] = pose.pose.position.x;
    data_json["y"] = pose.pose.position.y;
    data_json["yaw"] = yaw;
  }
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Position, dc_core::Measurement)
