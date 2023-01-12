#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_FILE_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_FILE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_destinations/destination_server.hpp"
#include "dc_destinations/flb_destination.hpp"

namespace dc_destinations
{

class Flbfile : public dc_destinations::FlbDestination
{
public:
  Flbfile();
  ~Flbfile();

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string path_;
  std::string file_;
  std::string format_;
  std::string mkdir_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_FILE_HPP_
