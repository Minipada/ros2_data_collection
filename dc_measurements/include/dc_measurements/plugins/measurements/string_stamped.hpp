// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_

#include "dc_core/measurement.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/node_utils.hpp"

namespace dc_measurements
{

// Republishes another Measurement's Record verbatim, so no schema can describe its data: run it
// with enable_validator: false, or configure() fails on the schema it would otherwise look for.
class StringStamped : public dc_measurements::Measurement
{
public:
  StringStamped();
  ~StringStamped() override;
  dc_interfaces::msg::StringStamped collect() override;
  void dataCb(const dc_interfaces::msg::StringStamped& msg);
  void onConfigure() override;

protected:
  dc_interfaces::msg::StringStamped last_data_;
  std::string topic_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr subscription_;
  bool timer_based_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_
