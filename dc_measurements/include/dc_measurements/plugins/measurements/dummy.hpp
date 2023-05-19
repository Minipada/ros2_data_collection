#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DUMMY_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DUMMY_HPP_

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_measurements
{

class Dummy : public dc_measurements::Measurement
{
public:
  Dummy();
  ~Dummy() override;
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Set validation schema used to confirm data before collecting it
   */
  void setValidationSchema() override;
  void onConfigure() override;
  std::string record_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__DUMMY_HPP_
