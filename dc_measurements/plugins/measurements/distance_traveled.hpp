#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DISTANCE_TRAVELED_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DISTANCE_TRAVELED_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "nav2_util/robot_utils.hpp"

namespace dc_measurements
{

class DistanceTraveled : public dc_measurements::Measurement
{
public:
  DistanceTraveled();
  ~DistanceTraveled();
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;

  std::string global_frame_;
  std::string robot_base_frame_;
  float transform_timeout_;
  long double distance_traveled_ = 0.0;
  float last_x_ = 0.0;
  float last_y_ = 0.0;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DISTANCE_TRAVELED_HPP_
