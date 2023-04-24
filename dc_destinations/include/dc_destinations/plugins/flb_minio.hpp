#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_MINIO_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_MINIO_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
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
// #include <fluent-bit/flb_plugin.h>
#include <fluent-bit.h>
#pragma GCC diagnostic pop

namespace dc_destinations
{

class FlbMinIO : public dc_destinations::FlbDestination
{
public:
  FlbMinIO();
  ~FlbMinIO() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string endpoint_;
  std::string access_key_id_;
  std::string secret_access_key_;
  bool use_ssl_;
  bool create_bucket_;
  std::string bucket_;
  std::string plugin_path_;
  std::string plugin_path_default_;
  std::vector<std::string> upload_fields_;
  std::vector<std::string> src_fields_;
  std::vector<std::string> groups_;
  bool verbose_plugin_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_MINIO_HPP_
