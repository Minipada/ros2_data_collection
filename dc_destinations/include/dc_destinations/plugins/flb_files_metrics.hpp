#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_FILESMETRICS_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_FILESMETRICS_HPP_

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

#define MAX_USERID_LENGTH 32

namespace dc_destinations
{

class FlbFilesMetrics : public dc_destinations::FlbDestination
{
public:
  FlbFilesMetrics();
  ~FlbFilesMetrics();

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string plugin_path_;
  std::string plugin_path_default_;
  std::string db_type_;
  std::vector<std::string> file_storage_;
  bool delete_when_sent_;

  std::string minio_endpoint_;
  std::string minio_access_key_id_;
  std::string minio_secret_access_key_;
  bool minio_use_ssl_;
  std::string minio_create_bucket_;
  std::string minio_bucket_;
  std::vector<std::string> minio_upload_fields_;
  std::vector<std::string> minio_src_fields_;
  std::vector<std::string> minio_input_names_;

  std::string s3_endpoint_;
  std::string s3_access_key_id_;
  std::string s3_secret_access_key_;
  std::string s3_create_bucket_;
  std::string s3_bucket_;
  std::vector<std::string> s3_upload_fields_;
  std::vector<std::string> s3_src_fields_;

  std::string pgsql_host_;
  std::string pgsql_port_;
  std::string pgsql_user_;
  std::string pgsql_password_;
  std::string pgsql_database_;
  std::string pgsql_table_;
  std::string pgsql_time_key_;
  std::string pgsql_async_;
  bool pgsql_use_ssl_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_FILESMETRICS_HPP_
