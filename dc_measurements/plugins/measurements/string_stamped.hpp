#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"

namespace dc_measurements
{

class StringStamped : public dc_measurements::Measurement
{
public:
  StringStamped();
  ~StringStamped();
  dc_interfaces::msg::StringStamped collect() override;
  void dataCb(dc_interfaces::msg::StringStamped::SharedPtr msg);

protected:
  /**
   * @brief Set validation schema used to confirm data before collecting it
   */
  void setValidationSchema() override;
  dc_interfaces::msg::StringStamped last_data_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_
