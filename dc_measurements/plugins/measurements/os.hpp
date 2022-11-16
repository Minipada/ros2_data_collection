#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__OS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__OS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "system/linux_parser.hpp"
#include "system/system.hpp"

namespace dc_measurements
{

class OS : public dc_measurements::Measurement
{
public:
  OS();
  ~OS();
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__OS_HPP_
