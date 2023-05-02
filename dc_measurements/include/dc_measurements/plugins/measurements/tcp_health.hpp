#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__TCP_HEALTH_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__TCP_HEALTH_HPP_

#include <boost/asio.hpp>
#include <iostream>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_measurements
{

class TCPHealth : public dc_measurements::Measurement
{
public:
  TCPHealth();
  ~TCPHealth() override;
  dc_interfaces::msg::StringStamped collect() override;
  bool getPortHealth(unsigned short port);

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
  std::string host_;
  std::string name_;
  unsigned short port_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__TCP_HEALTH_HPP_
