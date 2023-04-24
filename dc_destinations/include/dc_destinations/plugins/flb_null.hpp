#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_NULL_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_NULL_HPP_

#include <boost/algorithm/string/replace.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "dc_destinations/destination_server.hpp"
#include "dc_destinations/flb_destination.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <fluent-bit.h>
#pragma GCC diagnostic pop

namespace dc_destinations
{

class FlbNull : public dc_destinations::FlbDestination
{
public:
  FlbNull();
  ~FlbNull() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_NULL_HPP_
