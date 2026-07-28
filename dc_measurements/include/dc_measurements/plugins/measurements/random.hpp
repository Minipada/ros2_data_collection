#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__RANDOM_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__RANDOM_HPP_

#include <cstdint>
#include <random>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_measurements
{

class Random : public dc_measurements::Measurement
{
public:
  Random();
  ~Random() override;
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Set validation schema used to confirm data before collecting it
   */
  void setValidationSchema() override;
  void onConfigure() override;

  std::string value_type_;
  double min_;
  double max_;
  int64_t seed_;
  std::mt19937_64 generator_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__RANDOM_HPP_
