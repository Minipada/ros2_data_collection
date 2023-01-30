#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"

namespace dc_measurements
{

class StringStamped : public dc_measurements::Measurement
{
public:
  StringStamped();
  ~StringStamped();
  dc_interfaces::msg::StringStamped collect() override;
  void dataCb(dc_interfaces::msg::StringStamped::SharedPtr msg);
  // void onConfigure();

protected:
  /**
   * @brief Set validation schema used to confirm data before collecting it
   */
  void setValidationSchema() override;
  dc_interfaces::msg::StringStamped last_data_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STRING_STAMPED_HPP_
