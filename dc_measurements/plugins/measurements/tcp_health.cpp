#include "dc_measurements/plugins/measurements/tcp_health.hpp"

namespace dc_measurements
{

TCPHealth::TCPHealth() : dc_measurements::Measurement()
{
}

TCPHealth::~TCPHealth() = default;

void TCPHealth::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".host", rclcpp::ParameterValue("127.0.0.1"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".port", rclcpp::ParameterValue(80));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".name", rclcpp::PARAMETER_STRING);
  node->get_parameter(measurement_name_ + ".host", host_);
  node->get_parameter(measurement_name_ + ".port", port_);
  node->get_parameter(measurement_name_ + ".name", name_);
}

void TCPHealth::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "tcp_health.json");
  }
}

bool TCPHealth::getPortHealth(unsigned short port)
{
  using namespace boost::asio;
  using ip::tcp;

  bool result = false;

  try
  {
    io_service svc;
    tcp::acceptor a(svc);

    boost::system::error_code ec;
    a.open(tcp::v4(), ec) || a.bind({ tcp::v4(), port }, ec);

    return ec == error::address_in_use;
  }
  catch (...)
  {
  }

  return result;
}

dc_interfaces::msg::StringStamped TCPHealth::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  data_json["port"] = port_;
  data_json["host"] = host_;
  data_json["server_name"] = name_;
  data_json["active"] = getPortHealth(port_);

  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::TCPHealth, dc_core::Measurement)
