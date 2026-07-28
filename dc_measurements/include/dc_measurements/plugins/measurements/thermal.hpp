
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__THERMAL_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__THERMAL_HPP_

#include <string>
#include <vector>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_measurements
{

class Thermal : public dc_measurements::Measurement
{
public:
  Thermal();
  ~Thermal() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  std::vector<std::string> discoverZones();

  std::string base_path_;
  std::vector<std::string> zones_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__THERMAL_HPP_
