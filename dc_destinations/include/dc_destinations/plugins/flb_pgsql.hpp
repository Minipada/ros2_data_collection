#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_PGSQL_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_PGSQL_HPP_

#include <boost/algorithm/string/replace.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "dc_destinations/destination_server.hpp"
#include "dc_destinations/flb_destination.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_util/string_utils.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <fluent-bit.h>
#pragma GCC diagnostic pop

#define MAX_USERID_LENGTH 32

namespace dc_destinations
{

class FlbPgSQL : public dc_destinations::FlbDestination
{
public:
  FlbPgSQL();
  ~FlbPgSQL();

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string host_;
  int port_;
  std::string user_;
  std::string password_;
  std::string database_;
  std::string table_;
  std::string timestamp_key_;
  bool async_;
  std::string min_pool_size_;
  std::string max_pool_size_;
  bool cockroachdb_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_PGSQL_HPP_
