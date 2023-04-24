#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_HTTP_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_HTTP_HPP_

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

class FlbHTTP : public dc_destinations::FlbDestination
{
public:
  FlbHTTP();
  ~FlbHTTP() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string format_;
  std::string host_;
  std::string body_key_;
  int port_;
  std::string uri_;
  std::string header_;
  std::string headers_key_;
  std::string header_tag_;
  std::string json_date_key_;
  std::string http_user_;
  std::string http_passwd_;
  bool log_response_payload_;
  bool allow_duplicated_headers_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_HTTP_HPP_
