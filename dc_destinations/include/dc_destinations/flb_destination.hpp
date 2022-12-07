#ifndef DC_DESTINATIONS__FLB_DESTINATION_HPP_
#define DC_DESTINATIONS__FLB_DESTINATION_HPP_

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <boost/algorithm/string/replace.hpp>
#include <any>
#include <map>

#include "dc_destinations/destination.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_util/node_utils.hpp"
#include "dc_util/string_utils.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <fluent-bit.h>
#pragma GCC diagnostic pop

namespace dc_destinations
{

using namespace std::chrono_literals;  // NOLINT

class FlbDestination : public dc_destinations::Destination
{
public:
  /**
   * @brief A Destination constructor
   */
  FlbDestination()
  {
  }

  virtual void initFlbOutputPlugin() = 0;

  // configure the server on lifecycle setup
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                 const std::vector<std::string>& inputs, flb_ctx_t* ctx, const bool& debug,
                 const std::string& flb_in_storage_type, const std::string& time_format, const std::string& time_key,
                 const json& custom_params, const std::string& run_id, const bool& run_id_enabled) override
  {
    node_ = parent;
    auto node = node_.lock();

    ctx_ = ctx;
    debug_ = debug;
    destination_name_ = name;
    inputs_ = inputs;
    flb_in_storage_type_ = flb_in_storage_type;
    time_format_ = time_format;
    time_key_ = time_key;
    run_id_ = run_id;
    run_id_enabled_ = run_id_enabled;
    custom_params_ = custom_params;

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring Flb plugin %s", destination_name_.c_str());

    onConfigure();
    onDestinationConfigure();

    RCLCPP_INFO(logger_, "Done configuring Flb plugin %s", destination_name_.c_str());
  }

  ~FlbDestination()
  {
  }

  std::string formatString(const dc_interfaces::msg::StringStamped::SharedPtr msg)
  {
    std::string data_str = msg->data.c_str();
    boost::replace_all(data_str, "'", "\"");
    json data_json = json::parse(data_str);

    data_json["date"] =
        std::stod(std::to_string(msg->header.stamp.sec) + std::string(".") + std::to_string(msg->header.stamp.nanosec));

    std::string formatted_str = nlohmann::to_string(data_json);
    boost::replace_all(formatted_str, "'", "\"");
    return std::string("[") + std::to_string(msg->header.stamp.sec) + std::string(".") +
           std::to_string(msg->header.stamp.nanosec) + "," + formatted_str + std::string("]");
  }

  void initTimestampFilter()
  {
    /* Filter for timestamp*/
    int f_ffd = flb_filter(ctx_, (char*)"lua", NULL);
    if (f_ffd == -1)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot start lua timestamp filter");
    }
    std::string date_record = "";
    if (time_format_ == "double")
    {
      date_record = "record[\"" + time_key_ + "\"] = timestamp ";
    }
    else if (time_format_ == "iso8601")
    {
      date_record = "record[\"" + time_key_ +
                    "\"] = os.date('!%Y-%m-%dT%H:%M:%S', timestamp[\"sec\"]) .. \".\" .. "
                    "tostring(math.floor(timestamp[\"nsec\"])) ";
    }
    std::string ts_lua_code =
        std::string("function replace_ts(tag, timestamp, record) ") + date_record + " return 2, timestamp, record end";
    int ret = flb_filter_set(ctx_, f_ffd, "code", ts_lua_code.c_str(), NULL);
    ret += flb_filter_set(ctx_, f_ffd, "call", "replace_ts", NULL);
    ret += flb_filter_set(ctx_, f_ffd, "Match", destination_name_.c_str(), NULL);
    if (time_format_ != "double")
    {
      ret += flb_filter_set(ctx_, f_ffd, "time_as_table", "true", NULL);
    }
    if (ret != 0)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot set lua timestamp filter");
    }
  }

  void initRewriteTagFilter()
  {
    int f_ffd = flb_filter(ctx_, (char*)"rewrite_tag", NULL);
    if (f_ffd == -1)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot start rewrite_tag filter");
    }
    std::string rule = "$tags .*(" + destination_name_ + ").* " + destination_name_ + " true";

    int ret = flb_filter_set(ctx_, f_ffd, "Rule", rule.c_str(), NULL);
    ret += flb_filter_set(ctx_, f_ffd, "Match", "ros2", NULL);
    ret += flb_filter_set(ctx_, f_ffd, "emitter_storage.type", flb_in_storage_type_.c_str(), NULL);
    ret += flb_filter_set(ctx_, f_ffd, "emitter_mem_buf_limit", "5M", NULL);
    if (ret != 0)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot set rule for rewrite_tag filter");
    }

    RCLCPP_INFO(logger_, "Loaded rewrite_tag filter. Match=ros2, Rule=%s", rule.c_str());
  }

  void initConcatenateTags()
  {
    /* Filter rewrite tags as string configuration */
    /* ["tag1", "tag2"] -> "tag1,tag2" */
    /* and add timestamp in the field - Not sure this is needed */
    int f_ffd = flb_filter(ctx_, (char*)"lua", NULL);
    if (f_ffd == -1)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot start lua filter");
    }

    std::string lua_code = std::string("function concatenate(tag, timestamp, record) ") +
                           "if (type(record[\"tags\"]) == \"table\") then " +
                           "record[\"tags\"] = table.concat(record[\"tags\"], \",\") " + "end " +
                           " return 2, timestamp, record end";

    int ret = flb_filter_set(ctx_, f_ffd, "code", lua_code.c_str(), NULL);
    ret += flb_filter_set(ctx_, f_ffd, "call", "concatenate", NULL);
    ret += flb_filter_set(ctx_, f_ffd, "Match", "ros2", NULL);

    if (ret != 0)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot set lua filter");
    }
    RCLCPP_INFO(logger_, "Loaded lua filter. Match=ros2, code=%s", lua_code.c_str());
  }

  void initRemoveTagsFilter()
  {
    /* Filter modify configuration */
    // Remove the tag from the json message
    int f_ffd = flb_filter(ctx_, (char*)"modify", NULL);
    if (f_ffd == -1)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot start modify filter");
    }

    int ret = flb_filter_set(ctx_, f_ffd, "Remove", "tags", NULL);
    ret += flb_filter_set(ctx_, f_ffd, "Match", destination_name_.c_str(), NULL);

    for (auto param = custom_params_.begin(); param != custom_params_.end(); ++param)
    {
      std::string key = param.key();
      std::string value = param.value();
      ret += flb_filter_set(ctx_, f_ffd, "Add", (key + " " + value).c_str(), NULL);
    }
    if (ret != 0)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot set rule for modify filter");
    }
  }

  void initAddRunId()
  {
    if (run_id_enabled_)
    {
      int f_ffd = flb_filter(ctx_, (char*)"modify", NULL);
      int ret = flb_filter_set(ctx_, f_ffd, "Add", (std::string("run_id ") + run_id_).c_str(), NULL);
      ret += flb_filter_set(ctx_, f_ffd, "Match", destination_name_.c_str(), NULL);
      if (f_ffd == -1)
      {
        flb_destroy(ctx_);
        throw std::runtime_error("Cannot start modify filter (add run_id)");
      }
      if (ret != 0)
      {
        flb_destroy(ctx_);
        throw std::runtime_error("Cannot set rule for modify filter (add run_id)");
      }
    }
  }

  void initAddCustomParamsFilter()
  {
    int ret = 0;
    int f_ffd = flb_filter(ctx_, (char*)"modify", NULL);
    ret += flb_filter_set(ctx_, f_ffd, "Match", destination_name_.c_str(), NULL);
    for (auto param = custom_params_.begin(); param != custom_params_.end(); ++param)
    {
      std::string key = param.key();
      std::string value = param.value();
      ret += flb_filter_set(ctx_, f_ffd, "Add", (key + " " + value).c_str(), NULL);
    }
    if (f_ffd == -1)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot start modify filter (add custom params)");
    }
    if (ret != 0)
    {
      flb_destroy(ctx_);
      throw std::runtime_error("Cannot set rule for modify filter (add custom params)");
    }
  }

  void initFlbFilters()
  {
    initTimestampFilter();

    initConcatenateTags();

    initRewriteTagFilter();

    initRemoveTagsFilter();

    initAddRunId();
  }

  void initFlbDebug()
  {
    /* Enable fluent bit debug */
    if (debug_)
    {
      int f_ffd = flb_output(ctx_, (char*)"stdout", NULL);
      int ret = flb_output_set(ctx_, f_ffd, "Match", destination_name_.c_str(), NULL);
      if (f_ffd == -1)
      {
        flb_destroy(ctx_);
        throw std::runtime_error("Cannot start stdout filter");
      }
      if (ret != 0)
      {
        flb_destroy(ctx_);
        throw std::runtime_error("Cannot enable debug for plugin");
      }
    }
  }

  void onDestinationConfigure()
  {
    initFlbFilters();
    initFlbDebug();
    initFlbOutputPlugin();
  }

protected:
  int in_ffd_;
  int out_ffd_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__FLB_DESTINATION_HPP_
