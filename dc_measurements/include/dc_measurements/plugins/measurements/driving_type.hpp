// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DRIVING_TYPE_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DRIVING_TYPE_HPP_

#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/driving_mode_source.hpp"
#include "dc_measurements/measurement.hpp"

namespace dc_measurements
{

class DrivingType : public dc_measurements::Measurement
{
public:
  DrivingType();
  ~DrivingType() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  DrivingModeSource mode_source_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DRIVING_TYPE_HPP_
