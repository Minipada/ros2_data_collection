#ifndef DC_CORE_DESTINATION_HPP_
#define DC_CORE_DESTINATION_HPP_

#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <fluent-bit.h>
#pragma GCC diagnostic pop

namespace dc_core
{
class Destination
{
public:
  using Ptr = std::shared_ptr<Destination>;
  using json = nlohmann::json;

  /**
   * @brief Virtual destructor
   */
  virtual ~Destination()
  {
  }

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this destination
   * @param  inputs The name of the inputs matching this destination
   * @param  debug Display debug messages
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                         const std::vector<std::string>& inputs, flb_ctx_t* ctx, const bool& debug,
                         const std::string& flb_in_storage_type, const std::string& time_format,
                         const std::string& time_key, const json& custom_params, const std::string& run_id,
                         const bool& run_id_enabled) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate Destination and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate Destination and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  virtual void sendData(const dc_interfaces::msg::StringStamped& msg) = 0;
};
}  // namespace dc_core

#endif  // DC_CORE_DESTINATION_HPP_
