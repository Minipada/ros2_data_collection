// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/raw_subscriptions.hpp"

#include <algorithm>
#include <chrono>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dc_bridge
{

namespace introspection = rosidl_typesupport_introspection_cpp;
using introspection::MessageMember;
using introspection::MessageMembers;
using nlohmann::json;

namespace
{

/// How often a repeated raw-mode warning (oversize, undecodable, shipper drop) is
/// allowed through. A firehose topic hits these paths at the topic's own rate, so an
/// unthrottled log line would itself become the volume problem.
constexpr int RAW_WARN_THROTTLE_MS = 5000;

std::string utf16_to_utf8(const std::u16string& input)
{
  std::string out;
  for (std::size_t i = 0; i < input.size(); ++i)
  {
    char32_t code_point = input[i];
    // Surrogate pair: the code point is split across two UTF-16 units.
    if (code_point >= 0xD800 && code_point <= 0xDBFF && i + 1 < input.size() && input[i + 1] >= 0xDC00 &&
        input[i + 1] <= 0xDFFF)
    {
      code_point = 0x10000 + ((code_point - 0xD800) << 10) + (input[i + 1] - 0xDC00);
      ++i;
    }
    if (code_point < 0x80)
    {
      out.push_back(static_cast<char>(code_point));
    }
    else if (code_point < 0x800)
    {
      out.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
      out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
    }
    else if (code_point < 0x10000)
    {
      out.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
      out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
    }
    else
    {
      out.push_back(static_cast<char>(0xF0 | (code_point >> 18)));
      out.push_back(static_cast<char>(0x80 | ((code_point >> 12) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
    }
  }
  return out;
}

json message_to_json(const void* message, const MessageMembers* members);

json scalar_to_json(const void* field, const MessageMember& member)
{
  switch (member.type_id_)
  {
    case introspection::ROS_TYPE_FLOAT:
      return *static_cast<const float*>(field);
    case introspection::ROS_TYPE_DOUBLE:
      return *static_cast<const double*>(field);
    case introspection::ROS_TYPE_LONG_DOUBLE:
      // JSON has one number type; long double's extra range/precision has nowhere to go.
      return static_cast<double>(*static_cast<const long double*>(field));
    case introspection::ROS_TYPE_CHAR:
      return static_cast<std::int64_t>(*static_cast<const unsigned char*>(field));
    case introspection::ROS_TYPE_WCHAR:
      return static_cast<std::int64_t>(*static_cast<const char16_t*>(field));
    case introspection::ROS_TYPE_BOOLEAN:
      return *static_cast<const bool*>(field);
    case introspection::ROS_TYPE_OCTET:
      // `std::byte` in the C++ mapping; layout-compatible with unsigned char.
      return static_cast<std::int64_t>(*static_cast<const unsigned char*>(field));
    case introspection::ROS_TYPE_UINT8:
      return static_cast<std::int64_t>(*static_cast<const std::uint8_t*>(field));
    case introspection::ROS_TYPE_INT8:
      return static_cast<std::int64_t>(*static_cast<const std::int8_t*>(field));
    case introspection::ROS_TYPE_UINT16:
      return static_cast<std::int64_t>(*static_cast<const std::uint16_t*>(field));
    case introspection::ROS_TYPE_INT16:
      return static_cast<std::int64_t>(*static_cast<const std::int16_t*>(field));
    case introspection::ROS_TYPE_UINT32:
      return static_cast<std::int64_t>(*static_cast<const std::uint32_t*>(field));
    case introspection::ROS_TYPE_INT32:
      return static_cast<std::int64_t>(*static_cast<const std::int32_t*>(field));
    case introspection::ROS_TYPE_UINT64:
      return *static_cast<const std::uint64_t*>(field);
    case introspection::ROS_TYPE_INT64:
      return *static_cast<const std::int64_t*>(field);
    case introspection::ROS_TYPE_STRING:
      return *static_cast<const std::string*>(field);
    case introspection::ROS_TYPE_WSTRING:
      return utf16_to_utf8(*static_cast<const std::u16string*>(field));
    case introspection::ROS_TYPE_MESSAGE:
      return message_to_json(field, static_cast<const MessageMembers*>(member.members_->data));
    default:
      break;
  }
  throw std::runtime_error("field '" + std::string(member.name_) + "': unsupported introspection type id " +
                           std::to_string(static_cast<int>(member.type_id_)));
}

/// In-memory size of one element, used only for the fallback path below.
std::size_t element_size(const MessageMember& member)
{
  switch (member.type_id_)
  {
    case introspection::ROS_TYPE_FLOAT:
      return sizeof(float);
    case introspection::ROS_TYPE_DOUBLE:
      return sizeof(double);
    case introspection::ROS_TYPE_LONG_DOUBLE:
      return sizeof(long double);
    case introspection::ROS_TYPE_CHAR:
    case introspection::ROS_TYPE_OCTET:
    case introspection::ROS_TYPE_UINT8:
    case introspection::ROS_TYPE_INT8:
      return 1;
    case introspection::ROS_TYPE_WCHAR:
    case introspection::ROS_TYPE_UINT16:
    case introspection::ROS_TYPE_INT16:
      return 2;
    case introspection::ROS_TYPE_BOOLEAN:
      return sizeof(bool);
    case introspection::ROS_TYPE_UINT32:
    case introspection::ROS_TYPE_INT32:
      return 4;
    case introspection::ROS_TYPE_UINT64:
    case introspection::ROS_TYPE_INT64:
      return 8;
    case introspection::ROS_TYPE_STRING:
      return sizeof(std::string);
    case introspection::ROS_TYPE_WSTRING:
      return sizeof(std::u16string);
    case introspection::ROS_TYPE_MESSAGE:
      return static_cast<const MessageMembers*>(member.members_->data)->size_of_;
    default:
      break;
  }
  throw std::runtime_error("field '" + std::string(member.name_) + "': unsupported introspection type id " +
                           std::to_string(static_cast<int>(member.type_id_)));
}

json array_to_json(const void* field, const MessageMember& member)
{
  // The generated accessors are the correct way to walk both `std::array` (fixed) and
  // `std::vector` (bounded/unbounded sequence) members without knowing which one this
  // is. The pointer-arithmetic fallback only applies to a fixed array whose accessors a
  // type support generator declined to emit — contiguous storage, so the arithmetic is
  // valid there and nowhere else.
  const std::size_t count = member.size_function != nullptr ? member.size_function(field) : member.array_size_;
  json array = json::array();
  for (std::size_t i = 0; i < count; ++i)
  {
    const void* element =
        member.get_const_function != nullptr ?
            member.get_const_function(field, i) :
            static_cast<const void*>(static_cast<const std::uint8_t*>(field) + i * element_size(member));
    array.push_back(scalar_to_json(element, member));
  }
  return array;
}

json message_to_json(const void* message, const MessageMembers* members)
{
  json out = json::object();
  for (std::uint32_t i = 0; i < members->member_count_; ++i)
  {
    const MessageMember& member = members->members_[i];
    const void* field = static_cast<const std::uint8_t*>(message) + member.offset_;
    out[member.name_] = member.is_array_ ? array_to_json(field, member) : scalar_to_json(field, member);
  }
  return out;
}

/// A default-constructed instance of a runtime-known message type. `::operator new`
/// rather than a `std::vector<uint8_t>` because the buffer holds real C++ objects
/// (std::string, std::vector) whose alignment requirements the introspection data does
/// not report — operator new is guaranteed suitably aligned for any type that size.
class MessageStorage
{
public:
  explicit MessageStorage(const MessageMembers* members) : members_(members), buffer_(::operator new(members->size_of_))
  {
    members_->init_function(buffer_, rosidl_runtime_cpp::MessageInitialization::ALL);
  }

  ~MessageStorage()
  {
    members_->fini_function(buffer_);
    ::operator delete(buffer_);
  }

  MessageStorage(const MessageStorage&) = delete;
  MessageStorage& operator=(const MessageStorage&) = delete;

  void* get() noexcept
  {
    return buffer_;
  }

private:
  const MessageMembers* members_;
  void* buffer_;
};

}  // namespace

RawMessageConverter::RawMessageConverter(const std::string& type_name) : type_name_(type_name)
{
  try
  {
    // Two libraries, two jobs: `rosidl_typesupport_cpp` knows how to turn the wire bytes
    // back into an in-memory message, `rosidl_typesupport_introspection_cpp` knows what
    // that message's fields are called and where they live. Both must outlive every
    // handle taken from them, hence the members.
    typesupport_library_ = rclcpp::get_typesupport_library(type_name, "rosidl_typesupport_cpp");
    const rosidl_message_type_support_t* type_support =
        rclcpp::get_message_typesupport_handle(type_name, "rosidl_typesupport_cpp", *typesupport_library_);

    introspection_library_ = rclcpp::get_typesupport_library(type_name, introspection::typesupport_identifier);
    const rosidl_message_type_support_t* introspection_support = rclcpp::get_message_typesupport_handle(
        type_name, introspection::typesupport_identifier, *introspection_library_);

    members_ = introspection_support->data;
    serializer_ = std::make_unique<rclcpp::SerializationBase>(type_support);
  }
  catch (const std::exception& e)
  {
    throw RawConfigError("cannot load type support for message type '" + type_name + "': " + e.what());
  }
}

RawMessageConverter::~RawMessageConverter() = default;

json RawMessageConverter::to_json(const rclcpp::SerializedMessage& serialized) const
{
  const auto* members = static_cast<const MessageMembers*>(members_);
  MessageStorage storage(members);
  serializer_->deserialize_message(&serialized, storage.get());
  return message_to_json(storage.get(), members);
}

std::optional<RawStamp> stamp_from_payload(const json& payload)
{
  if (!payload.is_object() || !payload.contains("header"))
  {
    return std::nullopt;
  }
  const json& header = payload.at("header");
  if (!header.is_object() || !header.contains("stamp"))
  {
    return std::nullopt;
  }
  const json& stamp = header.at("stamp");
  if (!stamp.is_object() || !stamp.contains("sec") || !stamp.contains("nanosec"))
  {
    return std::nullopt;
  }
  if (!stamp.at("sec").is_number_integer() || !stamp.at("nanosec").is_number_integer())
  {
    return std::nullopt;
  }
  const std::int64_t secs = stamp.at("sec").get<std::int64_t>();
  RawStamp out;
  // A message published before its clock was initialised carries stamp 0 (or, with
  // sim time misconfigured, a negative one) — clamp rather than wrap into 1970-adjacent
  // nonsense on the unsigned protocol field.
  out.secs = secs > 0 ? static_cast<std::uint64_t>(secs) : 0;
  out.nanos = static_cast<std::uint32_t>(stamp.at("nanosec").get<std::uint64_t>() % 1000000000ULL);
  return out;
}

rclcpp::QoS raw_qos_for_topic(rclcpp::Node& node, const std::string& topic, std::size_t depth)
{
  // Braces, not parentheses: `QoS qos(KeepLast(depth))` is a function declaration.
  rclcpp::QoS qos{ rclcpp::KeepLast(depth) };
  const auto publishers = node.get_publishers_info_by_topic(topic);
  if (publishers.empty())
  {
    return qos;
  }

  bool any_best_effort = false;
  bool all_transient_local = true;
  for (const auto& publisher : publishers)
  {
    const rclcpp::QoS& profile = publisher.qos_profile();
    if (profile.reliability() == rclcpp::ReliabilityPolicy::BestEffort)
    {
      any_best_effort = true;
    }
    if (profile.durability() != rclcpp::DurabilityPolicy::TransientLocal)
    {
      all_transient_local = false;
    }
  }
  if (any_best_effort)
  {
    qos.best_effort();
  }
  if (all_transient_local)
  {
    qos.transient_local();
  }
  return qos;
}

RawSubscriptionManager::RawSubscriptionManager(rclcpp::Node* node, RawConfig config, RecordSink sink, RawStats* stats)
  : node_(node)
  , config_(std::move(config))
  , filter_(config_)
  , sink_(std::move(sink))
  , stats_(stats)
  , limiter_(config_.max_rate_hz)
{
  if (config_.destination.empty())
  {
    throw RawConfigError("raw.enabled is true but raw.destination is empty; name the `receives: records` "
                         "destination raw Records should be routed to");
  }
}

void RawSubscriptionManager::start()
{
  scan();
  if (config_.rescan_interval_secs > 0.0)
  {
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(config_.rescan_interval_secs));
    rescan_timer_ = node_->create_wall_timer(period, [this]() { this->scan(); });
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "raw: rescan disabled (raw.rescan_interval_secs = 0); topics appearing "
                                     "after startup will not be picked up");
  }
}

void RawSubscriptionManager::scan()
{
  for (const auto& [topic, types] : node_->get_topic_names_and_types())
  {
    bool already_subscribed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      already_subscribed = subscriptions_.count(topic) > 0;
    }
    if (already_subscribed || types.empty())
    {
      continue;
    }

    // Every skipped topic is re-evaluated on the next scan — some reasons to skip are
    // transient (a second publisher advertising a different type goes away; a message
    // package's type support becomes loadable) and a permanent skip list would never
    // notice. Only the *log line* is suppressed after the first time, so a 5-second
    // rescan doesn't reprint the same verdict forever.
    auto report = [this, &topic](const std::string& message, bool warn) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!reported_.insert(topic).second)
      {
        return;
      }
      if (warn)
      {
        RCLCPP_WARN(node_->get_logger(), "raw: cannot subscribe to %s: %s", topic.c_str(), message.c_str());
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "raw: skipping %s (%s)", topic.c_str(), message.c_str());
      }
    };

    if (types.size() > 1)
    {
      // Genuinely ambiguous: one generic subscription can carry exactly one type, and
      // picking arbitrarily would silently drop the other publisher's messages.
      report("topic advertises " + std::to_string(types.size()) + " types", false);
      continue;
    }

    const std::string& type = types.front();
    const RawSkip verdict = filter_.evaluate(topic, type);
    if (verdict != RawSkip::Accepted)
    {
      report(to_string(verdict), false);
      continue;
    }

    std::shared_ptr<RawMessageConverter> converter;
    try
    {
      converter = std::make_shared<RawMessageConverter>(type);
    }
    catch (const std::exception& e)
    {
      // The message package isn't on this Bridge's AMENT_PREFIX_PATH. Not fatal — the
      // rest of the graph is still collectable — but the operator has to know.
      report(e.what(), true);
      continue;
    }

    const std::string tag = raw_tag(config_.tag_prefix, topic);
    std::shared_ptr<rclcpp::GenericSubscription> subscription;
    try
    {
      subscription =
          node_->create_generic_subscription(topic, type, raw_qos_for_topic(*node_, topic, config_.qos_depth),
                                             [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> message) {
                                               this->on_raw_message(topic, std::move(message));
                                             });
    }
    catch (const std::exception& e)
    {
      report(std::string("creating the generic subscription failed: ") + e.what(), true);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      subscriptions_.emplace(topic, Entry{ subscription, converter, tag });
      if (stats_ != nullptr)
      {
        stats_->subscribed_topics.store(subscriptions_.size());
      }
    }
    RCLCPP_INFO(node_->get_logger(), "raw: subscribed to %s [%s] under Tag '%s'", topic.c_str(), type.c_str(),
                tag.c_str());
  }
}

std::size_t RawSubscriptionManager::subscription_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return subscriptions_.size();
}

void RawSubscriptionManager::on_raw_message(const std::string& topic,
                                            std::shared_ptr<const rclcpp::SerializedMessage> message)
{
  std::shared_ptr<RawMessageConverter> converter;
  std::string tag;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subscriptions_.find(topic);
    if (it == subscriptions_.end())
    {
      return;
    }
    converter = it->second.converter;
    tag = it->second.tag;

    // Both volume gates run before any deserialization work: the cheapest possible drop
    // is the point (#227's "a raw-subscribe-everything mode can outrun the Shipper").
    if (config_.max_message_size_bytes > 0 && message->size() > config_.max_message_size_bytes)
    {
      if (stats_ != nullptr)
      {
        stats_->dropped_oversize.fetch_add(1);
      }
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), RAW_WARN_THROTTLE_MS,
                           "raw: dropping messages on %s: %zu bytes exceeds raw.max_message_size_bytes (%lu)",
                           topic.c_str(), message->size(), static_cast<unsigned long>(config_.max_message_size_bytes));
      return;
    }
    if (!limiter_.allow(topic, RawRateLimiter::Clock::now()))
    {
      if (stats_ != nullptr)
      {
        stats_->dropped_rate.fetch_add(1);
      }
      return;
    }
  }

  json payload;
  try
  {
    payload = converter->to_json(*message);
  }
  catch (const std::exception& e)
  {
    if (stats_ != nullptr)
    {
      stats_->dropped_undecodable.fetch_add(1);
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), RAW_WARN_THROTTLE_MS,
                         "raw: dropping a message on %s: %s", topic.c_str(), e.what());
    return;
  }

  RawStamp stamp;
  if (const auto header_stamp = stamp_from_payload(payload))
  {
    stamp = *header_stamp;
  }
  else
  {
    // No header: stamp with the node's own clock (sim time included, like every other
    // DC timestamp).
    const std::int64_t since_epoch = std::max<std::int64_t>(0, node_->now().nanoseconds());
    stamp.secs = static_cast<std::uint64_t>(since_epoch / 1000000000LL);
    stamp.nanos = static_cast<std::uint32_t>(since_epoch % 1000000000LL);
  }

  const bool forwarded = sink_(tag, stamp, std::move(payload));
  if (stats_ == nullptr)
  {
    return;
  }
  if (forwarded)
  {
    stats_->forwarded.fetch_add(1);
  }
  else
  {
    stats_->dropped_shipper.fetch_add(1);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), RAW_WARN_THROTTLE_MS,
                         "raw: the Shipper refused a Record on %s; raw Records are dropped rather than queued "
                         "(see doc/src/dc/raw_topics.md)",
                         topic.c_str());
  }
}

}  // namespace dc_bridge
