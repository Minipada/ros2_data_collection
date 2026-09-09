// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MEASUREMENT_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_HPP_

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

#include "dc_common/file_scratch_ring.hpp"
#include "dc_common/record_file_walk.hpp"
#include "dc_core/condition.hpp"
#include "dc_core/condition_set.hpp"
#include "dc_core/measurement.hpp"
#include "dc_interfaces/msg/condition.hpp"
#include "dc_interfaces/msg/flush_event.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/incident_releaser.hpp"
#include "dc_measurements/publish_gate.hpp"
#include "dc_measurements/record_enricher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;  // NOLINT
using json = nlohmann::json;
using nlohmann::json_schema::json_validator;

namespace dc_measurements
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

// A json_validator schema_loader resolving a "$ref" naming a sibling file (e.g.
// "mission_base.json") against `schema_dir` -- the directory the root schema itself was loaded
// from. Every schema in this codebase but the Mission Measurement family keeps to same-file
// "#/$defs/..." refs, which nlohmann_json_schema_validator resolves on its own; this loader is
// only ever invoked for a schema that references another file, letting a family of adapters
// (#305/ADR-0010's shared mission_start/mission_end contract) share one base schema instead of
// duplicating its properties per adapter.
inline nlohmann::json_schema::schema_loader makeSchemaFileLoader(const std::string& schema_dir)
{
  return [schema_dir](const nlohmann::json_uri& id, json& value) {
    std::string filename = id.path();
    if (!filename.empty() && filename.front() == '/')
    {
      filename.erase(0, 1);
    }
    std::ifstream f(schema_dir + "/" + filename);
    if (!f)
    {
      throw std::runtime_error{ "could not load schema '" + filename + "' referenced from " + schema_dir };
    }
    value = json::parse(f);
  };
}

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server and basic factory.
 */
// template <typename ActionT>
class Measurement : public dc_core::Measurement
{
public:
  //   using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A Measurement constructor
   */
  Measurement()
  {
  }

  ~Measurement() override = default;

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on cleanup
  // if they chose
  virtual void onCleanup()
  {
  }

  // an opportunity for derived classes to do something on failed validation
  // if they chose
  virtual void onFailedValidation(json data_json)
  {
    (void)data_json;  // Ignore error of variable being unused
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> getNode()
  {
    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{ "Failed to lock node" };
    }
    return node;
  }

  std::string getSavePath(const std::string& path_parameter)
  {
    auto rclcpp_time = rclcpp::Clock().now();
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t{ std::chrono::nanoseconds(
        rclcpp_time.nanoseconds()) };
    std::time_t newt = std::chrono::system_clock::to_time_t(t);
    std::stringstream trans_time;

    auto node = getNode();
    trans_time << std::put_time(localtime(&newt),
                                node->get_parameter(measurement_name_ + "." + path_parameter).as_string().c_str());
    std::string fmt_time = trans_time.str();

    return fmt_time;
  }

  std::string getSavePath(const std::string& path_parameter, const rclcpp::Time& now)
  {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t{ std::chrono::nanoseconds(
        now.nanoseconds()) };
    std::time_t newt = std::chrono::system_clock::to_time_t(t);
    std::stringstream trans_time;

    auto node = getNode();
    trans_time << std::put_time(localtime(&newt),
                                node->get_parameter(measurement_name_ + "." + path_parameter).as_string().c_str());
    std::string fmt_time = trans_time.str();

    return fmt_time;
  }

  void validateSchema(const std::string& package_name, const std::string& json_filename)
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
    std::string schema_dir = package_share_directory + "/plugins/measurements/json";
    std::string path = schema_dir + "/" + json_filename;
    std::ifstream f(path.c_str());
    try
    {
      schema_ = json::parse(f);

      RCLCPP_INFO_STREAM(logger_, "Looking for schema at " << path);

      RCLCPP_INFO_STREAM(logger_, "schema: " << schema_);
      try
      {
        validator_ = json_validator(makeSchemaFileLoader(schema_dir));
        validator_.set_root_schema(schema_);
      }
      catch (const std::exception& e)
      {
        std::string err = std::string("Validation of schema failed: ") + e.what();
        RCLCPP_ERROR(logger_, "%s", err.c_str());
        throw std::runtime_error{ err.c_str() };
      }
    }
    catch (json::parse_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON from file path: " << schema_);
    }
  }

  void validateSchema(const std::string& json_schema_path)
  {
    std::ifstream f(json_schema_path.c_str());
    try
    {
      schema_ = json::parse(f);

      RCLCPP_INFO_STREAM(logger_, "schema: " << schema_);
      try
      {
        const std::string schema_dir = std::filesystem::path(json_schema_path).parent_path().string();
        validator_ = json_validator(makeSchemaFileLoader(schema_dir));
        validator_.set_root_schema(schema_);
      }
      catch (const std::exception& e)
      {
        std::string err = std::string("Validation of schema failed: ") + e.what();
        RCLCPP_ERROR(logger_, "%s", err.c_str());
        throw std::runtime_error{ err.c_str() };
      }
    }
    catch (json::parse_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON file json_schema_path: " << json_schema_path);
    }
  }

  // CamelCase to snake_case, keeping an acronym run together with the word it precedes:
  // TCPHealth -> tcp_health, Ros2ControlStatus -> ros2_control_status, OS -> os.
  static std::string snakeCase(const std::string& camel)
  {
    std::string out;
    for (size_t i = 0; i < camel.size(); i++)
    {
      const unsigned char c = static_cast<unsigned char>(camel[i]);
      const unsigned char prev = i > 0 ? static_cast<unsigned char>(camel[i - 1]) : 0;
      const unsigned char next = i + 1 < camel.size() ? static_cast<unsigned char>(camel[i + 1]) : 0;
      const bool boundary = i > 0 && std::isupper(c) &&
                            ((std::islower(prev) || std::isdigit(prev)) || (std::isupper(prev) && std::islower(next)));
      if (boundary)
      {
        out += '_';
      }
      out += static_cast<char>(std::tolower(c));
    }
    return out;
  }

  // The schema file a plugin type validates against by default, relative to the registering
  // package's plugins/measurements/json directory. Only the type after the '/' is converted:
  // the package part is not a CamelCase word.
  static std::string defaultSchemaFile(const std::string& plugin_lookup_name)
  {
    const size_t sep = plugin_lookup_name.find('/');
    const std::string& plugin_type = sep == std::string::npos ? plugin_lookup_name : plugin_lookup_name.substr(sep + 1);
    return snakeCase(plugin_type) + ".json";
  }

  // The schema a Measurement validates against when json_schema_path is unset: one file per
  // plugin type, named after it, shipped by the package registering the plugin
  // (dc_measurements/Camera -> dc_measurements/plugins/measurements/json/camera.json).
  // Derived from the plugin type rather than the measurement name so an instance id like
  // right_camera still finds the Camera schema.
  void validateDefaultSchema()
  {
    const size_t sep = measurement_plugin_.find('/');
    const std::string package =
        sep == std::string::npos ? std::string("dc_measurements") : measurement_plugin_.substr(0, sep);
    validateSchema(package, defaultSchemaFile(measurement_plugin_));
  }

  // Current state of one of the configured Conditions, as dc_core::ConditionSet asks for it.
  // A name that is not a configured Condition reads as false rather than dereferencing a null
  // plugin, which is what the previous conditions_[name]->getState(msg) did on a typo'd name.
  bool getConditionState(const std::string& condition_name, const dc_interfaces::msg::StringStamped& msg)
  {
    auto condition_it = conditions_.find(condition_name);
    if (condition_it == conditions_.end() || !condition_it->second)
    {
      RCLCPP_ERROR_STREAM(logger_, "Measurement " << measurement_name_ << ": '" << condition_name
                                                  << "' is not a configured condition; treating it as false.");
      return false;
    }
    return condition_it->second->getState(msg);
  }

  bool isConditionOn(const dc_interfaces::msg::StringStamped& msg)
  {
    // The if_all/if_any/if_none composition itself lives in dc_core so the Trigger plugins can
    // compose Conditions the same way (#284); only the state lookup is Measurement's own.
    const auto outcome = condition_set_.evaluate(dc_core::ConditionStateLookup{
        [&](const std::string& condition_name) { return getConditionState(condition_name, msg); } });

    RCLCPP_DEBUG(logger_, "all_conditions_res=%d, any_conditions_res=%d, none_conditions_res=%d",
                 outcome.if_all_satisfied, outcome.if_any_satisfied, outcome.if_none_satisfied);

    return outcome.satisfied();
  }

  bool isAnyConditionSet()
  {
    return !condition_set_.empty();
  }

  // Whether every collection should currently be held back by the gate condition.
  //
  // Semantics (distinct from if_all/if_any/if_none, which are re-evaluated on every collection):
  // gate_condition_ names a Condition that must become true once; from then on the gate latches
  // open permanently for the lifetime of this Measurement instance and the condition is never
  // consulted again, even if it later becomes false again.
  bool isGateArmed(const dc_interfaces::msg::StringStamped& msg)
  {
    // An open gate -- no gate_condition configured, or already latched -- is never re-consulted.
    if (publish_gate_.gateOpen())
    {
      return true;
    }

    auto condition_it = conditions_.find(gate_condition_);
    if (condition_it == conditions_.end())
    {
      RCLCPP_ERROR_STREAM(logger_, "Measurement " << measurement_name_ << ": gate_condition '" << gate_condition_
                                                  << "' is not a configured condition; holding all collection back.");
      return false;
    }

    const bool open = publish_gate_.openGate(condition_it->second->getState(msg));
    if (open)
    {
      RCLCPP_INFO_STREAM(logger_, "Measurement " << measurement_name_ << ": gate_condition '" << gate_condition_
                                                 << "' became true, collection will proceed normally from now on.");
    }
    return open;
  }

  // The validation step of the enrichment pipeline: a Record the schema rejects is logged and
  // handed to the plugin hook, but still published, exactly as this chain always did.
  void validateJSON(const json& data_json)
  {
    try
    {
      validator_.validate(data_json);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Validation failed: " << e.what() << "data=" << data_json.dump());
      onFailedValidation(data_json);
    }
  }

  // Applies the same enrichment publish() applies before publishing (validation, nesting/
  // flattening, run_id, tags, measurement name/plugin, custom keys) without publishing --
  // shared by publish() and bufferSample() so a Record released later from the ring buffer
  // looks identical to one published live, modulo the incident_id onFlushEvent() adds (#287).
  // One parse, one dump: the steps themselves live in RecordEnricher (#482).
  void enrichMsg(dc_interfaces::msg::StringStamped& msg)
  {
    msg.data = record_enricher_.enrich(msg.data);
  }

  void publish(dc_interfaces::msg::StringStamped msg)
  {
    auto msg_copy = msg;
    if (!msg.data.empty() && msg.data != "null")
    {
      enrichMsg(msg);
    }
    if (!msg.data.empty() && msg.data != "null")
    {
      if (!isGateArmed(msg_copy))
      {
        RCLCPP_DEBUG_STREAM(logger_, "Measurement " << measurement_name_ << ": gate_condition '" << gate_condition_
                                                    << "' not yet true, dropping collection.");
        return;
      }

      if (publish_gate_.offer(isAnyConditionSet() && isConditionOn(msg_copy)))
      {
        data_pub_->publish(msg);
      }

      if (publish_gate_.collectionFinished())
      {
        collect_timer_.reset();
      }
    }
    else
    {
      // Not necessarily a fault: a Measurement that found nothing to report
      // returns an empty message on purpose -- Camera does exactly that on every
      // frame with no barcode in view, which is most frames of the QR-code demo.
      // Throttled and worded accordingly, because at the polling rate this used
      // to read like a broken pipeline and sent #279 chasing one that was fine.
      RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                  "No data collected from measurement "
                                      << measurement_name_ << " (nothing to report, or it is not receiving input)");
    }
  }

  void publishFromMsg(const dc_interfaces::msg::StringStamped& msg)
  {
    if (enabled_)
    {
      publish(msg);
    }
  }

  void collectAndPublish()
  {
    dc_interfaces::msg::StringStamped msg = collect();
    if (enabled_)
    {
      if (releaser_)
      {
        offerSample(msg);
      }
      else
      {
        publish(msg);
      }
    }
  }

  // With incident capture configured, collectAndPublish() calls this instead of publish(): the
  // sample is enriched exactly as a live Record would be, then handed to the IncidentReleaser,
  // which buffers it or (during post-roll) publishes it live through publishIncidentRecord() (#288).
  void offerSample(dc_interfaces::msg::StringStamped msg)
  {
    const auto now = std::chrono::system_clock::now();
    if (msg.data.empty() || msg.data == "null")
    {
      // Nothing to offer, but the phase deadlines and a rate-limited release still need driving.
      const std::lock_guard<std::mutex> lock(releaser_mutex_);
      releaser_->tick(now);
      return;
    }
    enrichMsg(msg);
    const std::lock_guard<std::mutex> lock(releaser_mutex_);
    // Drive the phase transitions before deciding what to do with this sample's Files: during
    // post-roll the Record is published live and its Files are left exactly where the Measurement
    // wrote them, which is what the Bridge has always picked up.
    releaser_->tick(now);
    if (releaser_->state() != IncidentState::PostRoll)
    {
      stageSampleFiles(msg, now);
    }
    releaser_->offer(msg.data, now);
  }

  // A buffered Record's Files, staged (#290). Called for every sample the releaser is about to
  // buffer rather than publish: each File the Record references is moved into the scratch ring and
  // the Record rewritten to point at the staged copy, then the ring is aged like the Record ring
  // buffer it shadows, so an armed Measurement's Files stay bounded by buffer_duration_sec instead
  // of piling up in the save path with no Record to ever carry them to the Bridge.
  //
  // Caller holds releaser_mutex_.
  void stageSampleFiles(dc_interfaces::msg::StringStamped& msg, const std::chrono::system_clock::time_point& now)
  {
    json data_json;
    try
    {
      data_json = json::parse(msg.data);
    }
    catch (json::parse_error& e)
    {
      // enrichMsg() already logged whatever made this unparsable; buffer it as-is.
      return;
    }
    if (stageRecordFiles(data_json, now))
    {
      msg.data = data_json.dump(-1, ' ', true);
    }
    // Rolling deletion of the aged-out staged Files -- the disk-backed half of the same eviction
    // the ring buffer does to their Records, driven with the same `now` and the same window, so
    // the two stay in step. Safe to run mid-release: onFlushEvent() released the window being
    // drained out of the ring's ownership before the drain started.
    if (file_scratch_ring_)
    {
      file_scratch_ring_->evict(now);
    }
  }

  // Stage every File `value` references, rewriting each local_paths entry to its staged copy.
  // Returns whether anything was staged. The Record walk itself is owned by dc_common's
  // record_file_walk (#479) -- the same one dc_bridge's parse_file_group() goes through, so
  // exactly the paths the Bridge would have uploaded are the ones that move, in either written
  // form (nested local_paths objects, or a flattened Record's JSON-pointer keys).
  bool stageRecordFiles(json& value, const std::chrono::system_clock::time_point& stamp)
  {
    return dc_common::record_file_walk::rewrite_local_paths(value, [&](std::string& path) {
      return stageOneFile(path, stamp);
    });
  }

  // Move one File into the scratch ring and rewrite `path` to the staged copy. The walk hands
  // over non-empty local-path strings only.
  bool stageOneFile(std::string& path, const std::chrono::system_clock::time_point& stamp)
  {
    const std::string original = path;

    try
    {
      const auto staged_path = scratchRing()->stage(original, stamp);
      // Staging *moves* the File: the Record referencing the original is buffered, not published,
      // so nothing will ever pick that original up -- neither the Bridge (which never sees the
      // Record) nor its retention sweep. Leaving it behind would grow the save path without bound
      // for as long as the Measurement stays armed, which is exactly what the bounded ring exists
      // to prevent. The copy the ring made is what the released Record points at, and it uploads
      // to the unchanged remote_paths key the Measurement computed at collection time.
      std::error_code ec;
      std::filesystem::remove(original, ec);
      path = staged_path.string();
      return true;
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                   "Measurement " << measurement_name_ << ": could not stage File " << original
                                                  << " into the incident scratch ring (" << e.what()
                                                  << "); buffering the Record with the File left in place.");
      return false;
    }
  }

  // The scratch ring, created on the first Record that actually references a File so the many
  // Measurements that produce none never grow a scratch directory. Caller holds releaser_mutex_.
  const std::shared_ptr<dc_common::FileScratchRing>& scratchRing()
  {
    if (!file_scratch_ring_)
    {
      file_scratch_ring_ =
          std::make_shared<dc_common::FileScratchRing>(scratchDir(), durationFromSeconds(buffer_duration_sec_));
    }
    return file_scratch_ring_;
  }

  // Where staged Files live: one stable directory per Measurement, next to the Files themselves
  // (same filesystem, so staging is a cheap local move) but outside the dated save tree. The
  // literal prefix of save_local_base_path is used -- everything up to its first strftime token --
  // so the scratch directory doesn't roll over every hour the way the save path does.
  std::filesystem::path scratchDir() const
  {
    std::string base = save_local_base_path_expanded_;
    const auto token = base.find('%');
    if (token != std::string::npos)
    {
      const auto slash = base.rfind('/', token);
      base = (slash == std::string::npos) ? std::string() : base.substr(0, slash);
    }
    if (base.empty())
    {
      base = std::filesystem::temp_directory_path().string();
    }
    return std::filesystem::path(base) / ".dc_incident_scratch" / measurement_name_;
  }

  // Emits one Record belonging to an incident -- either released from the pre-roll buffer (stamped
  // with when it was collected, not when it was released) or collected live during post-roll.
  // Publishes straight through data_pub_, bypassing publish()'s gate/counter logic: a Record
  // captured for an incident was already unconditionally collected and isn't re-filtered on the
  // way out.
  void publishIncidentRecord(const std::string& record_json, const std::chrono::system_clock::time_point& stamp,
                             const std::string& incident_id)
  {
    dc_interfaces::msg::StringStamped out;
    out.group_key = group_key_;
    out.header.stamp =
        rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count());
    try
    {
      json data_json = json::parse(record_json);
      data_json["incident_id"] = incident_id;
      out.data = data_json.dump(-1, ' ', true);
    }
    catch (json::parse_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_,
                          "Error parsing Record JSON while releasing incident " << incident_id << ": " << record_json);
      return;
    }
    data_pub_->publish(out);
  }

  // A Trigger fired somewhere in the system: hand the event to the state machine, which releases
  // the buffered window under this incident_id when armed and ignores the event otherwise (#288).
  void onFlushEvent(const dc_interfaces::msg::FlushEvent& event)
  {
    if (!releaser_)
    {
      return;
    }
    const auto now = std::chrono::system_clock::now();
    const std::lock_guard<std::mutex> lock(releaser_mutex_);
    releaser_->tick(now);
    // Whether this event starts a cycle has to be read before onFlush() acts on it, since acting
    // on it is exactly what makes isArmed() false.
    const bool starts_cycle = releaser_->isArmed();
    if (file_scratch_ring_ && starts_cycle)
    {
      // Staged Files that have aged out are not part of the window this incident releases (their
      // Records were evicted from the ring buffer too): delete them first, so what survives to
      // release() below is exactly what the released Records reference.
      file_scratch_ring_->evict(now);
    }
    releaser_->onFlush(event.incident_id, now);
    if (file_scratch_ring_ && starts_cycle)
    {
      // The released Records are on their way to the Bridge, which owns their Files from here
      // (intent queue, upload, retention sweep, delete_when_sent). Stop tracking them so no later
      // evict() can delete a File out from under an upload in flight -- #290's whole point being
      // that the Bridge needs no change to ingest them, however old their timestamps are.
      file_scratch_ring_->release();
    }
  }

  virtual dc_interfaces::msg::StringStamped collect() = 0;

  // configure the server on lifecycle setup
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                 const std::map<std::string, std::shared_ptr<dc_core::Condition>>& conditions,
                 std::shared_ptr<tf2_ros::Buffer> tf, const dc_core::MeasurementConfig& config) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    measurement_plugin_ = config.measurement_plugin;
    conditions_ = conditions;
    tf_ = tf;
    measurement_name_ = name;
    topic_output_ = config.topic_output;
    polling_interval_ = config.polling_interval;
    debug_ = config.debug;
    enable_validator_ = config.enable_validator;
    json_schema_path_ = config.json_schema_path;
    group_key_ = config.group_key;
    init_collect_ = config.init_collect;
    remote_keys_ = config.remote_keys;
    remote_prefixes_ = config.remote_prefixes;
    all_base_path_ = config.all_base_path;
    all_base_path_expanded_ = config.all_base_path_expanded;
    save_local_base_path_ = config.save_local_base_path;
    save_local_base_path_expanded_ = config.save_local_base_path_expanded;

    condition_set_ =
        dc_core::ConditionSet(config.if_all_conditions, config.if_any_conditions, config.if_none_conditions);

    gate_condition_ = config.gate_condition;

    // The publish decision -- gate latch, init quota, condition cap -- is state the ROS layer
    // only feeds and reads back (#482).
    PublishGate::Config gate_config;
    gate_config.init_max = config.init_max_measurements;
    gate_config.condition_max = config.condition_max_measurements;
    gate_config.has_conditions = !condition_set_.empty();
    gate_config.gate_enabled = !gate_condition_.empty();
    publish_gate_ = PublishGate(gate_config);

    RecordEnricher::Config enricher_config;
    enricher_config.measurement_name = measurement_name_;
    enricher_config.measurement_plugin = measurement_plugin_;
    enricher_config.nested = config.nested;
    enricher_config.flatten = config.flatten;
    enricher_config.run_id_enabled = config.run_id_enabled;
    enricher_config.run_id = config.run_id;
    enricher_config.include_measurement_name = config.include_measurement_name;
    enricher_config.include_measurement_plugin = config.include_measurement_plugin;
    enricher_config.tags = config.tags;
    enricher_config.custom_keys = config.custom_keys;
    enricher_config.enable_validator = config.enable_validator;
    record_enricher_ = RecordEnricher(
        enricher_config, [this](const json& data_json) { validateJSON(data_json); },
        [this](const std::string& data) {
          RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON while enriching: " << data);
        });

    if (topic_output_.empty())
    {
      topic_output_ = std::string("/dc/measurement/") + measurement_name_;
    }

    data_pub_ = node->create_publisher<dc_interfaces::msg::StringStamped>(topic_output_, 1);
    client_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    collect_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(polling_interval_), [this] { collectAndPublish(); }, client_cb_group_);

    buffer_duration_sec_ = config.buffer_duration_sec;
    post_roll_duration_sec_ = config.post_roll_duration_sec;
    cooldown_sec_ = config.cooldown_sec;
    max_flush_rate_hz_ = config.max_flush_rate_hz;
    flush_topic_ = config.flush_topic.empty() ? std::string("/dc/flush") : config.flush_topic;
    if (buffer_duration_sec_ > 0.0)
    {
      releaser_ = std::make_shared<IncidentReleaser>(
          durationFromSeconds(buffer_duration_sec_), durationFromSeconds(post_roll_duration_sec_),
          durationFromSeconds(cooldown_sec_), max_flush_rate_hz_,
          [this](const std::string& record_json, const std::chrono::system_clock::time_point& stamp,
                 const std::string& incident_id) { publishIncidentRecord(record_json, stamp, incident_id); });
      flush_sub_ = node->create_subscription<dc_interfaces::msg::FlushEvent>(
          flush_topic_, 10, std::bind(&Measurement::onFlushEvent, this, std::placeholders::_1));
      if (max_flush_rate_hz_ > 0.0)
      {
        // A rate-limited release is drained by tick(), and the polling timer alone would drain it
        // at the polling rate -- in bursts, and never faster than collection however high
        // max_flush_rate_hz is. This timer paces the release properly (#289).
        release_timer_ = node->create_wall_timer(
            durationFromSeconds(1.0 / max_flush_rate_hz_),
            [this] {
              const std::lock_guard<std::mutex> lock(releaser_mutex_);
              if (releaser_)
              {
                releaser_->tick(std::chrono::system_clock::now());
              }
            },
            client_cb_group_);
      }
    }

    RCLCPP_INFO(logger_, "Done configuring %s", measurement_name_.c_str());

    if (enable_validator_)
    {
      if (json_schema_path_.empty())
      {
        validateDefaultSchema();
      }
      else
      {
        validateSchema(json_schema_path_);
      }
    }
    // If error during measurement initialization, stop it from publishing
    try
    {
      onConfigure();
    }
    catch (std::runtime_error& error)
    {
      enabled_ = false;
      collect_timer_.reset();
    }

    if (enable_validator_ && schema_.empty())
    {
      throw std::runtime_error{ "Enabled validation but didn't configure schema!" };
    }
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    data_pub_.reset();
    flush_sub_.reset();
    // Before the releaser it ticks, so the timer can't fire on a null one.
    release_timer_.reset();
    {
      const std::lock_guard<std::mutex> lock(releaser_mutex_);
      releaser_.reset();
      if (file_scratch_ring_)
      {
        // The buffered Records go with the releaser, so the Files they referenced would be
        // orphaned in the scratch directory: nothing is left to publish them. Released ones are
        // untracked by now (onFlushEvent()) and are the Bridge's, so purge() can't touch them.
        file_scratch_ring_->purge();
        file_scratch_ring_.reset();
      }
    }
    onCleanup();
  }

  // Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating measurement %s", measurement_name_.c_str());

    data_pub_->on_activate();

    if (enabled_ && init_collect_)
    {
      collectAndPublish();
    }
  }

  // Deactivate server on lifecycle transition
  void deactivate() override
  {
    data_pub_->on_deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string measurement_name_;
  std::string measurement_plugin_;
  bool enabled_{ true };
  bool debug_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // Publish data
  int polling_interval_;
  rclcpp_lifecycle::LifecyclePublisher<dc_interfaces::msg::StringStamped>::SharedPtr data_pub_;
  rclcpp::TimerBase::SharedPtr collect_timer_;
  std::string topic_output_;
  std::string group_key_;

  // Parameters
  bool init_collect_;
  std::vector<std::string> remote_keys_;
  std::vector<std::string> remote_prefixes_;
  std::string all_base_path_;
  std::string all_base_path_expanded_;
  std::string save_local_base_path_;
  std::string save_local_base_path_expanded_;

  // Conditions
  std::map<std::string, std::shared_ptr<dc_core::Condition>> conditions_;
  dc_core::ConditionSet condition_set_;
  std::string gate_condition_;

  // The publish decision (gate latch, init quota, condition cap) and the Record shaping,
  // both ROS-free and directly tested (#482).
  PublishGate publish_gate_;
  RecordEnricher record_enricher_{ RecordEnricher::Config{}, {}, {} };

  // Logger
  rclcpp::Logger logger_{ rclcpp::get_logger("dc_measurements") };

  // Validation
  bool enable_validator_;
  std::string json_schema_path_;
  json_validator validator_;
  json schema_;

  // Incident capture: pre-event circular-buffer capture (#287) driven by the IncidentReleaser
  // state machine (#288). releaser_ is only created when buffer_duration_sec_ > 0; while it
  // exists, collected samples go to it instead of publish(), and a FlushEvent on flush_topic_
  // starts one buffer-release / post-roll / cooldown / re-arm cycle.
  double buffer_duration_sec_{ 0.0 };
  double post_roll_duration_sec_{ 0.0 };
  double cooldown_sec_{ 0.0 };
  double max_flush_rate_hz_{ 0.0 };
  std::string flush_topic_;
  std::shared_ptr<IncidentReleaser> releaser_;
  rclcpp::Subscription<dc_interfaces::msg::FlushEvent>::SharedPtr flush_sub_;
  // Paces a rate-limited release; only created when max_flush_rate_hz_ > 0 (#289).
  rclcpp::TimerBase::SharedPtr release_timer_;
  // The Files half of incident capture (#290): while the releaser buffers a Record instead of
  // publishing it, the Files that Record references are moved in here and the Record rewritten to
  // point at the staged copies, which age out on the same window as the Records themselves.
  // Created lazily by scratchRing(), on the first Record that references a File at all.
  std::shared_ptr<dc_common::FileScratchRing> file_scratch_ring_;
  // The releaser is reached from three concurrent paths -- the polling timer, the FlushEvent
  // subscription and release_timer_ -- so every entry into it, and into the scratch ring that
  // shadows it, is serialized here.
  std::mutex releaser_mutex_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MEASUREMENT_HPP_
