#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_S3_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_S3_HPP_

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

class FlbS3 : public dc_destinations::FlbDestination
{
public:
  FlbS3();
  ~FlbS3();

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string region_;
  std::string bucket_;
  std::string json_date_key_;
  std::string json_date_format_;
  std::string total_file_size_;
  std::string upload_chunk_size_;
  std::string upload_timeout_;
  std::string store_dir_;
  std::string store_dir_limit_size_;
  std::string s3_key_format_;
  std::string s3_key_format_tag_delimiters_;
  bool static_file_path_;
  bool use_put_object_;
  std::string role_arn_;
  std::string endpoint_;
  std::string sts_endpoint_;
  std::string canned_acl_;
  std::string compression_;
  std::string content_type_;
  bool send_content_md5_;
  bool auto_retry_requests_;
  std::string log_key_;
  bool preserve_data_ordering_;
  std::string storage_class_;
  std::string retry_limit_;
  std::string external_id_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_S3_HPP_
