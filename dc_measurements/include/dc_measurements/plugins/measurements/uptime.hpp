#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__UPTIME_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__UPTIME_HPP_

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/system/linux_parser.hpp"
#include "dc_measurements/system/system.hpp"

namespace dc_measurements
{

class Uptime : public dc_measurements::Measurement
{
public:
  Uptime();
  ~Uptime() override;
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Set validation schema used to confirm data before collecting it
   */
  void setValidationSchema() override;
  System system_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__UPTIME_HPP_
