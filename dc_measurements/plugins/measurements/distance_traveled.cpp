#include "distance_traveled.hpp"

namespace dc_measurements
{

DistanceTraveled::DistanceTraveled() : dc_measurements::Measurement()
{
}

DistanceTraveled::~DistanceTraveled() = default;

void DistanceTraveled::onConfigure()
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

void DistanceTraveled::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "distance_traveled.json");
  }
}

dc_interfaces::msg::StringStamped DistanceTraveled::collect()
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
    data_json["distance_traveled"] =
        sqrt(pow(pose.pose.position.x - this->last_x_, 2) + pow(pose.pose.position.y - this->last_y_, 2) * 1.0);
    this->last_x_ = pose.pose.position.x;
    this->last_y_ = pose.pose.position.y;
  }
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::DistanceTraveled, dc_core::Measurement)
