#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_TCP_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_TCP_HPP_

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

class FlbTCP : public dc_destinations::FlbDestination
{
public:
  FlbTCP();
  ~FlbTCP() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string host_;
  int port_;
  std::string format_;
  std::string json_date_key_;
  std::string json_date_format_;
  int workers_;
  bool tls_active_;
  bool tls_verify_;
  int tls_debug_;
  std::string tls_ca_file_;
  std::string tls_crt_file_;
  std::string tls_key_file_;
  std::string tls_key_passwd_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_TCP_HPP_
