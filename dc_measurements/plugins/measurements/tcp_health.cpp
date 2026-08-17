// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

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
  host_ = dc_util::get_str_type_param(node, measurement_name_, "host", "127.0.0.1");
  port_ = dc_util::get_int_type_param(node, measurement_name_, "port", 80);
  name_ = dc_util::get_str_type_param(node, measurement_name_, "name");
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
