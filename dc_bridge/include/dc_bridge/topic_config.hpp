// Bridge configuration: which topics to subscribe to and the Fluent Forward tag each
// one is sent under. Kept separate from `forwarder` so the wire protocol code never has
// to know about ROS topic names.
#ifndef DC_BRIDGE__TOPIC_CONFIG_HPP_
#define DC_BRIDGE__TOPIC_CONFIG_HPP_

#include <optional>
#include <string>

namespace dc_bridge
{

/// One subscribed topic and the Fluent Forward tag its Records are sent under.
struct TopicConfig
{
  std::string topic;
  std::string tag;

  /// Build a topic config, deriving the Fluent Forward tag from the topic name (leading
  /// `/` stripped, remaining `/` replaced with `.`) when no explicit tag is given.
  static TopicConfig make(const std::string& topic, std::optional<std::string> tag = std::nullopt);

  /// The tag derivation on its own (leading `/` stripped, `/`→`.`, `/` alone → "dc").
  static std::string derive_tag(const std::string& topic);
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__TOPIC_CONFIG_HPP_
