#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_INFLUXDB_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_INFLUXDB_HPP_

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

class FlbInfluxDB : public dc_destinations::FlbDestination
{
public:
  FlbInfluxDB();
  ~FlbInfluxDB() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string host_;
  int port_;
  std::string database_;
  std::string bucket_;
  std::string org_;
  std::string sequence_tag_;
  std::string http_user_;
  std::string http_password_;
  std::string http_token_;
  std::string tag_keys_;
  bool auto_tags_;
  bool tags_list_enabled_;
  std::vector<std::string> tags_list_key_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_INFLUXDB_HPP_
