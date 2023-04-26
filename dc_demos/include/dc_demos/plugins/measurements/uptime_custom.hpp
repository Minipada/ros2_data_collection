#ifndef DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_
#define DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_

#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>

#include "dc_measurements/measurement.hpp"
#include "dc_measurements/plugins/measurements/uptime.hpp"

namespace dc_demos
{
using json = nlohmann::json;

class UptimeCustom : public dc_measurements::Uptime
{
protected:
  void onFailedValidation(json data_json) override;
  void setValidationSchema() override;
};

}  // namespace dc_demos

#endif  // DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_
