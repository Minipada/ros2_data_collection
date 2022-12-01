#ifndef DC_CORE_DESTINATION_HPP_
#define DC_CORE_DESTINATION_HPP_

#include <memory>
#include <string>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <nlohmann/json.hpp>

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
                         const std::vector<std::string>& inputs, const bool& debug) = 0;

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

  virtual void sendData(dc_interfaces::msg::StringStamped::SharedPtr msg) = 0;
};
}  // namespace dc_core

#endif  // DC_CORE_DESTINATION_HPP_
