
#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__RCL_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__RCL_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_core/destination.hpp"
#include "dc_destinations/destination.hpp"
#include "dc_destinations/destination_server.hpp"

namespace dc_destinations
{

class Rcl : public dc_destinations::Destination
{
public:
  Rcl();
  ~Rcl() override;

protected:
  /**
   * @brief Configuration of destination
   */
  void sendData(const dc_interfaces::msg::StringStamped& msg) override;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__RCL_HPP_
