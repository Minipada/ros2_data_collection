#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__UPTIME_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__UPTIME_HPP_

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

class Uptime : public dc_measurements::Measurement
{
public:
  Uptime();
  ~Uptime();
  dc_interfaces::msg::StringStamped collect() override;

private:
  System system_;

protected:
  /**
   * @brief Set validation schema used to confirm data before collecting it
   */
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__UPTIME_HPP_
