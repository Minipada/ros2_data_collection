// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MEASUREMENT_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_HPP_

#include <yaml-cpp/yaml.h>

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
#include "dc_core/condition.hpp"
#include "dc_core/condition_set.hpp"
#include "dc_core/measurement.hpp"
#include "dc_interfaces/msg/condition.hpp"
#include "dc_interfaces/msg/flush_event.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/incident_releaser.hpp"
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

  // an opportunity for derived classes to set the validation schema
  // if they chose
  virtual void setValidationSchema() = 0;

  void setValidationSchemaFromPath(const std::string& json_schema_path)
  {
    if (enable_validator_)
    {
      validateSchema(json_schema_path);
    }
  }

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
    if (gate_condition_.empty() || gate_armed_)
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

    gate_armed_ = condition_it->second->getState(msg);
    if (gate_armed_)
    {
      RCLCPP_INFO_STREAM(logger_, "Measurement " << measurement_name_ << ": gate_condition '" << gate_condition_
                                                 << "' became true, collection will proceed normally from now on.");
    }
    return gate_armed_;
  }

  void addTags(dc_interfaces::msg::StringStamped& msg)
  {
    // Only put the tags in if the conditions are ok. This way, we still publish the data
    // and can check in conditions but with no tags, this is not received by the Bridge
    if (!tags_.empty())
    {
      try
      {
        json data_json = json::parse(msg.data);
        data_json["tags"] = tags_;
        msg.data = data_json.dump(-1, ' ', true);
      }
      catch (json::parse_error& e)
      {
        RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when adding tags: " << msg.data);
      }
    }
  }

  void addRunId(dc_interfaces::msg::StringStamped& msg)
  {
    if (run_id_enabled_)
    {
      try
      {
        json data_json = json::parse(msg.data);
        data_json["run_id"] = run_id_;
        msg.data = data_json.dump(-1, ' ', true);
      }
      catch (json::parse_error& e)
      {
        RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when adding tags: " << msg.data);
      }
    }
  }

  void addMeasurementName(dc_interfaces::msg::StringStamped& msg)
  {
    // Only put the tags in if the conditions are ok. This way, we still publish the data
    // and can check in conditions but with no tags, this is not received by the Bridge
    if (include_measurement_name_)
    {
      try
      {
        json data_json = json::parse(msg.data);
        data_json["name"] = measurement_name_;
        msg.data = data_json.dump(-1, ' ', true);
      }
      catch (json::parse_error& e)
      {
        RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when adding measurement name " << measurement_name_
                                                                                        << " to data " << msg.data);
      }
    }
  }

  void addMeasurementPluginName(dc_interfaces::msg::StringStamped& msg)
  {
    // Only put the tags in if the conditions are ok. This way, we still publish the data
    // and can check in conditions but with no tags, this is not received by the Bridge
    if (include_measurement_plugin_)
    {
      try
      {
        json data_json = json::parse(msg.data);
        data_json["plugin"] = measurement_plugin_;
        msg.data = data_json.dump(-1, ' ', true);
      }
      catch (json::parse_error& e)
      {
        RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when adding measurement plugin " << measurement_plugin_
                                                                                          << "to data " << msg.data);
      }
    }
  }

  void nestedSample(dc_interfaces::msg::StringStamped& msg)
  {
    try
    {
      json nested_json = json::parse(msg.data);
      json data_json;

      if (nested_)
      {
        data_json[measurement_name_] = nested_json;
        msg.data = data_json.dump(-1, ' ', true);
      }
    }
    catch (json::parse_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when making it nested: " << msg.data);
    }
  }

  void flattenSample(dc_interfaces::msg::StringStamped& msg)
  {
    try
    {
      json data_json = json::parse(msg.data);
      json new_json;
      if (flatten_)
      {
        new_json = data_json.flatten();
        new_json["flattened"] = true;
      }
      else
      {
        new_json = data_json;
        new_json["flattened"] = false;
      }

      if (nested_)
      {
        new_json["nested"] = true;
      }
      else
      {
        new_json["nested"] = false;
      }
      msg.data = new_json.dump(-1, ' ', true);
    }
    catch (json::parse_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when flattening it: " << msg.data);
    }
  }

  void validateJSON(dc_interfaces::msg::StringStamped& msg)
  {
    if (enable_validator_)
    {
      try
      {
        auto json_data = json::parse(msg.data);
        try
        {
          validator_.validate(json_data);
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR_STREAM(logger_, "Validation failed: " << e.what() << "data=" << json_data.dump());
          onFailedValidation(json_data);
        }
      }
      catch (json::parse_error& e)
      {
        RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when publishing: " << msg.data);
      }
    }
  }

  // Applies the same enrichment publish() applies before publishing (validation, nesting/
  // flattening, run_id, tags, measurement name/plugin, custom keys) without publishing --
  // shared by publish() and bufferSample() so a Record released later from the ring buffer
  // looks identical to one published live, modulo the incident_id onFlushEvent() adds (#287).
  void enrichMsg(dc_interfaces::msg::StringStamped& msg)
  {
    // TODO pass the json, not the msg
    validateJSON(msg);
    nestedSample(msg);
    flattenSample(msg);
    addRunId(msg);
    addTags(msg);
    addMeasurementName(msg);
    addMeasurementPluginName(msg);
    addCustomKeys(msg);
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

      // Init publish
      if (init_max_measurements_ != -1 &&
          (init_max_measurements_ == 0 || init_counter_published_ < init_max_measurements_))
      {
        data_pub_->publish(msg);
        init_counter_published_++;
      }
      // Infinite measurements on condition
      else if (isAnyConditionSet() && isConditionOn(msg_copy) && condition_max_measurements_ == 0)
      {
        data_pub_->publish(msg);
      }
      // Trigger publish with maximum
      else if (isAnyConditionSet() && isConditionOn(msg_copy) &&
               condition_counter_published_ < condition_max_measurements_)
      {
        RCLCPP_DEBUG_STREAM(logger_, "condition_counter_published_=" << condition_counter_published_
                                                                     << ", condition_max_measurements_="
                                                                     << condition_max_measurements_);
        data_pub_->publish(msg);
        condition_counter_published_++;
      }
      // Not publish, reset Condition counter
      else if (isAnyConditionSet() && !isConditionOn(msg_copy))
      {
        condition_counter_published_ = 0;
      }

      if (init_max_measurements_ != 0 && init_max_measurements_ != -1 && condition_max_measurements_ < 0 &&
          init_max_measurements_ == init_counter_published_)
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

  void addCustomKeys(dc_interfaces::msg::StringStamped& msg)
  {
    if (!custom_keys_.empty() && (!msg.data.empty() && msg.data != "null"))
    {
      json data;
      try
      {
        data = json::parse(msg.data);
      }
      catch (json::parse_error& e)
      {
        RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON when adding custom keys: " << data.dump());
      }
      for (auto& param : custom_keys_)
      {
        auto key = param["key"].get<std::string>();
        auto value = param["value"].get<std::string>();
        auto override_value = param["override"].get<bool>();
        if (!data.contains(key) || (data.contains(key) && override_value))
        {
          json data_custom = { { key, value } };
          data.update(data_custom);
        }
      }
      msg.data = data.dump(-1, ' ', true);
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
  // Returns whether anything was staged. Walks the Record the same way dc_bridge's
  // parse_file_group() does, so exactly the paths the Bridge would have uploaded are the ones
  // that move: local_paths at any depth (a `nested` Measurement puts it under its own name),
  // never base64 (inline content, not a path).
  bool stageRecordFiles(json& value, const std::chrono::system_clock::time_point& stamp)
  {
    if (!value.is_object())
    {
      return false;
    }

    bool staged = false;
    auto local_paths_it = value.find("local_paths");
    if (local_paths_it != value.end() && local_paths_it->is_object())
    {
      for (auto it = local_paths_it->begin(); it != local_paths_it->end(); ++it)
      {
        staged = stageOneFile(*it, stamp) || staged;
      }
    }

    for (auto it = value.begin(); it != value.end(); ++it)
    {
      const std::string& key = it.key();
      if (key == "local_paths" || key == "remote_paths" || key == "base64")
      {
        continue;
      }
      // A `flatten`ed Record has no nested objects left: its keys are JSON pointers, so the File
      // paths show up as "/local_paths/raw" (or "/<name>/local_paths/raw" when also nested).
      if (it->is_string() && key.find("/local_paths/") != std::string::npos)
      {
        staged = stageOneFile(*it, stamp) || staged;
        continue;
      }
      staged = stageRecordFiles(*it, stamp) || staged;
    }
    return staged;
  }

  // Move one File into the scratch ring and rewrite `path_value` to the staged copy.
  bool stageOneFile(json& path_value, const std::chrono::system_clock::time_point& stamp)
  {
    if (!path_value.is_string())
    {
      return false;
    }
    const std::string original = path_value.get<std::string>();
    if (original.empty())
    {
      return false;
    }

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
      path_value = staged_path.string();
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
                 std::shared_ptr<tf2_ros::Buffer> tf, const std::string& measurement_plugin,
                 const std::string& group_key, const std::string& topic_output, const int& polling_interval,
                 const bool& debug, const bool& enable_validator, const std::string& json_schema_path,
                 const std::vector<std::string>& tags, const bool& init_collect, const int& init_max_measurements,
                 const bool& include_measurement_name, const bool& include_measurement_plugin,
                 const int& condition_max_measurements, const std::vector<std::string>& if_all_conditions,
                 const std::vector<std::string>& if_any_conditions, const std::vector<std::string>& if_none_conditions,
                 const std::string& gate_condition, const std::vector<std::string>& remote_keys,
                 const std::vector<std::string>& remote_prefixes, const bool& nested, const bool& flatten,
                 const std::string& save_local_base_path, const std::string& all_base_path,
                 const std::string& all_base_path_expanded, const std::string& save_local_base_path_expanded,
                 const std::string& run_id, const bool& run_id_enabled, const std::vector<json>& custom_keys,
                 const double& buffer_duration_sec, const double& post_roll_duration_sec, const double& cooldown_sec,
                 const double& max_flush_rate_hz, const std::string& flush_topic) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    measurement_plugin_ = measurement_plugin;
    conditions_ = conditions;
    tf_ = tf;
    measurement_name_ = name;
    topic_output_ = topic_output;
    polling_interval_ = polling_interval;
    debug_ = debug;
    enable_validator_ = enable_validator;
    json_schema_path_ = json_schema_path;
    group_key_ = group_key;
    tags_ = tags;
    init_collect_ = init_collect;
    init_max_measurements_ = init_max_measurements;
    include_measurement_name_ = include_measurement_name;
    include_measurement_plugin_ = include_measurement_plugin;
    remote_keys_ = remote_keys;
    remote_prefixes_ = remote_prefixes;
    nested_ = nested;
    flatten_ = flatten;
    all_base_path_ = all_base_path;
    all_base_path_expanded_ = all_base_path_expanded;
    save_local_base_path_ = save_local_base_path;
    save_local_base_path_expanded_ = save_local_base_path_expanded;
    run_id_ = run_id;
    run_id_enabled_ = run_id_enabled;
    custom_keys_ = custom_keys;

    condition_max_measurements_ = condition_max_measurements;
    condition_set_ = dc_core::ConditionSet(if_all_conditions, if_any_conditions, if_none_conditions);

    gate_condition_ = gate_condition;
    // No gate configured means collection is never held back.
    gate_armed_ = gate_condition_.empty();

    if (topic_output_.empty())
    {
      topic_output_ = std::string("/dc/measurement/") + measurement_name_;
    }

    data_pub_ = node->create_publisher<dc_interfaces::msg::StringStamped>(topic_output_, 1);
    client_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    collect_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(polling_interval_), [this] { collectAndPublish(); }, client_cb_group_);

    buffer_duration_sec_ = buffer_duration_sec;
    post_roll_duration_sec_ = post_roll_duration_sec;
    cooldown_sec_ = cooldown_sec;
    max_flush_rate_hz_ = max_flush_rate_hz;
    flush_topic_ = flush_topic.empty() ? std::string("/dc/flush") : flush_topic;
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

    if (json_schema_path_.empty())
    {
      setValidationSchema();
    }
    else
    {
      setValidationSchemaFromPath(json_schema_path_);
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

  // Run Id
  bool run_id_enabled_;
  std::string run_id_;

  // Custom keys
  std::vector<json> custom_keys_;

  // Parameters
  bool init_collect_;
  bool include_measurement_name_;
  bool include_measurement_plugin_;
  std::vector<std::string> remote_keys_;
  std::vector<std::string> remote_prefixes_;
  bool nested_;
  bool flatten_;
  std::string all_base_path_;
  std::string all_base_path_expanded_;
  std::string save_local_base_path_;
  std::string save_local_base_path_expanded_;

  // Counters
  int init_counter_published_ = 0;
  int init_max_measurements_;

  // Conditions
  std::map<std::string, std::shared_ptr<dc_core::Condition>> conditions_;
  dc_core::ConditionSet condition_set_;
  int condition_max_measurements_;
  int condition_counter_published_ = 0;

  // Gate: one-shot arming latch, distinct from if_all/if_any/if_none above.
  std::string gate_condition_;
  bool gate_armed_{ true };

  // Logger
  rclcpp::Logger logger_{ rclcpp::get_logger("dc_measurements") };

  // Validation
  bool enable_validator_;
  std::string json_schema_path_;
  json_validator validator_;
  json schema_;

  // Tags
  std::vector<std::string> tags_;

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
