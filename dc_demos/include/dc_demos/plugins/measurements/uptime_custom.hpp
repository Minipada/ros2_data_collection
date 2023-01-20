#ifndef DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_
#define DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_

#include "dc_measurements/measurement.hpp"
#include "dc_measurements/plugins/measurements/uptime.hpp"

namespace dc_demos
{

class UptimeCustom : public dc_measurements::Uptime
{
protected:
  void onFailedValidation() override;
  void setValidationSchema() override;
};

}  // namespace dc_demos

#endif  // DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_
