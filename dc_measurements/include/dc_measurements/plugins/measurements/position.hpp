
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__POSITION_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__POSITION_HPP_

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace dc_measurements
{

class Position : public dc_measurements::Measurement
{
public:
  Position();
  ~Position() override;
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
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__POSITION_HPP_
