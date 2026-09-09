// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MEASUREMENT_CORE_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_CORE_HPP_

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "dc_common/file_scratch_ring.hpp"
#include "dc_core/condition_set.hpp"
#include "dc_measurements/incident_releaser.hpp"
#include "dc_measurements/publish_gate.hpp"
#include "dc_measurements/record_enricher.hpp"

using json = nlohmann::json;                  // NOLINT
using nlohmann::json_schema::json_validator;  // NOLINT

namespace dc_measurements
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

/// Severity of a message a ROS-free module reports through its log callback.
enum class LogLevel
{
  Debug,
  Info,
  Warn,
  Error,
};

/// One Record on its way out of the pipeline: the finished payload, the Group merge key, the
/// collection timestamp in nanoseconds since the epoch, and the Incident it belongs to (empty
/// outside one).
struct RecordOut
{
  std::string data;
  std::string group_key;
  std::string incident_id;
  std::int64_t stamp_ns{ 0 };
};

// A json_validator schema_loader resolving a "$ref" naming a sibling file (e.g.
// "mission_base.json") against `schema_dir` -- the directory the root schema itself was loaded
// from. Every schema in this codebase but the Mission Measurement family keeps to same-file
// "#/$defs/..." refs, which nlohmann_json_schema_validator resolves on its own; this loader is
// only ever invoked for a schema that references another file, letting a family of adapters
// (#305/ADR-0010's shared mission_start/mission_end contract) share one base schema instead of
// duplicating its properties per adapter.
nlohmann::json_schema::schema_loader makeSchemaFileLoader(const std::string& schema_dir);

/**
 * @class dc_measurements::MeasurementCore
 * @brief The Measurement pipeline without ROS (#499): the gate latch, the publish counters, Record
 * enrichment with schema validation, incident buffering, File staging and the scratch ring.
 *
 * This used to live in the header-only dc_measurements::Measurement, where every plugin .so
 * compiled a private copy and nothing was instantiable without a LifecycleNode. The core owns the
 * decisions; the ROS driver (measurement.hpp) owns the publisher, timers, subscription and
 * lifecycle wiring that feed it.
 *
 * Every boundary is injected, so the whole pipeline is testable with no rclcpp type anywhere:
 * - time enters through the `now` argument of each entry point;
 * - Conditions enter as a dc_core::ConditionStateLookup -- the caller resolves names against its
 *   live Condition plugins;
 * - Records leave through PublishFn, diagnostics through LogFn, and a schema-rejected Record
 *   reaches the plugin's hook through ValidationFailureFn.
 */
class MeasurementCore
{
public:
  using TimePoint = std::chrono::system_clock::time_point;
  using ConditionStateLookup = dc_core::ConditionStateLookup;

  /// Emits one Record, live or released from the incident buffer.
  using PublishFn = std::function<void(const RecordOut&)>;
  /// Reports a schema-rejected Record; owns the plugin hook, and does not stop publication.
  using ValidationFailureFn = std::function<void(const json&)>;
  /// Reports a pipeline diagnostic.
  using LogFn = std::function<void(LogLevel, const std::string&)>;

  /// The pure pipeline settings; the ROS wiring around them stays in the driver.
  struct Config
  {
    std::string measurement_name;
    std::string measurement_plugin;  ///< Schema file and default package are derived from the type
    std::string group_key;           ///< Key stamped on Records released from the incident buffer
    bool enable_validator{ true };
    std::string json_schema_path;               ///< Explicit schema file; empty means the default one
    std::string save_local_base_path_expanded;  ///< Scratch ring lives next to this, outside the
                                                ///< dated save tree

    // Conditions and gating
    std::vector<std::string> if_all_conditions;
    std::vector<std::string> if_any_conditions;
    std::vector<std::string> if_none_conditions;
    std::string gate_condition;
    int init_max_measurements{ 0 };
    int condition_max_measurements{ 0 };

    // Record shaping, forwarded to the RecordEnricher (#482)
    bool nested{ false };
    bool flatten{ false };
    bool run_id_enabled{ true };
    std::string run_id;
    bool include_measurement_name{ true };
    bool include_measurement_plugin{ false };
    std::vector<std::string> tags;
    std::vector<json> custom_keys;

    // Incident capture (#287/#288/#289)
    double buffer_duration_sec{ 0.0 };
    double post_roll_duration_sec{ 0.0 };
    double cooldown_sec{ 0.0 };
    double max_flush_rate_hz{ 0.0 };
  };

  MeasurementCore();
  ~MeasurementCore();

  MeasurementCore(const MeasurementCore&) = delete;
  MeasurementCore& operator=(const MeasurementCore&) = delete;

  /**
   * @brief Build the pipeline from `config`; Records go out through `publish`, a schema-rejected
   * Record reaches `on_validation_failure`, diagnostics through `log`.
   */
  void configure(const Config& config, PublishFn publish, ValidationFailureFn on_validation_failure, LogFn log);

  /**
   * @brief The live publish path: enrich, gate, count, emit.
   *
   * Empty and "null" payloads are dropped with a throttled warning -- a Measurement that found
   * nothing to report (Camera on a frame with no barcode) is not a fault. `gate_state` is
   * consulted only until the gate latches open; `conditions` only while the set is non-empty, so
   * Conditions with side effects are polled exactly when the old chain polled them.
   */
  void publish(const std::string& data, const std::string& group_key, std::int64_t stamp_ns,
               const ConditionStateLookup& gate_state, const ConditionStateLookup& conditions, const TimePoint& now);

  /**
   * @brief Hand one collected sample to the incident state machine instead of publish().
   *
   * The sample is enriched like a live Record, its Files staged (#290), then buffered or (during
   * post-roll) released live. Empty and "null" payloads still tick the phase deadlines along.
   */
  void offerSample(const std::string& data, const TimePoint& now);

  /// A Trigger fired somewhere in the system: releases the buffered window under `incident_id`
  /// when armed, ignored otherwise (#288).
  void onFlushEvent(const std::string& incident_id, const TimePoint& now);

  /// Advance a rate-limited release to `now`; safe and cheap to call at any rate (#289).
  void tickRelease(const TimePoint& now);

  /// Whether incident capture is configured (the driver subscribes to the flush topic when it is).
  bool hasReleaser() const;

  /// Whether the init quota is spent with no condition publishing configured; the driver stops
  /// the polling timer when this turns true.
  bool collectionFinished() const;

  /**
   * @brief The publish-enabling state machine, owned explicitly (#499).
   *
   * enabled_ is the lifecycle gate: the driver clears it on deactivate() and restores it on
   * activate(), so a deactivate -> activate cycle resumes collection where the old flag -- set to
   * false and never reset -- silenced the Measurement permanently. configure_failed_ is
   * permanent: an onConfigure() hook that threw keeps the Measurement silent for its lifetime,
   * however many activate() cycles follow.
   */
  bool collectible() const;
  void setEnabled(bool enabled);
  void markConfigureFailed();

  /// Load the schema from `package_name`'s plugins/measurements/json directory.
  void validateSchema(const std::string& package_name, const std::string& json_filename);
  /// Load the schema from an explicit path; sibling "$ref" files resolve next to it.
  void validateSchema(const std::string& json_schema_path);
  /// The schema a Measurement validates against when json_schema_path is unset: one file per
  /// plugin type, named after it, shipped by the package registering the plugin
  /// (dc_measurements/Camera -> dc_measurements/plugins/measurements/json/camera.json).
  void validateDefaultSchema();
  /// Whether no schema is loaded; with the validator enabled that is a configuration error.
  bool schemaEmpty() const;

  // CamelCase to snake_case, keeping an acronym run together with the word it precedes:
  // TCPHealth -> tcp_health, Ros2ControlStatus -> ros2_control_status, OS -> os.
  static std::string snakeCase(const std::string& camel);

  // The schema file a plugin type validates against by default, relative to the registering
  // package's plugins/measurements/json directory. Only the type after the '/' is converted:
  // the package part is not a CamelCase word.
  static std::string defaultSchemaFile(const std::string& plugin_lookup_name);

  /// Frees the incident state: the buffered Records go with the releaser, so the Files they
  /// referenced are purged from the scratch ring rather than orphaned there.
  void teardown();

private:
  // The enrichment step of the pipeline (#482): one parse, one dump, shared by publish() and
  // offerSample() so a released Record looks identical to a live one, modulo the incident_id.
  void enrich(std::string& data);

  // The validation step: a Record the schema rejects is logged and handed to the plugin hook,
  // but still published, exactly as this chain always did.
  void validateJSON(const json& data_json);

  // Whether every collection should currently be held back by the gate condition. Distinct from
  // if_all/if_any/if_none, which are re-evaluated on every collection: gate_condition_ names a
  // Condition that must become true once; from then on the gate latches open permanently for the
  // lifetime of this instance and the condition is never consulted again.
  bool gateOpen(const ConditionStateLookup& gate_state);

  // The condition-composition half of the publish decision; false when no Condition is
  // configured, matching the short-circuit the old isAnyConditionSet() && isConditionOn() had.
  bool conditionSetOn(const ConditionStateLookup& conditions);

  // A buffered Record's Files, staged (#290): each File the Record references is moved into the
  // scratch ring, the Record rewritten to point at the staged copy, and the ring aged like the
  // Record ring buffer it shadows. Caller holds releaser_mutex_.
  void stageSampleFiles(std::string& data, const TimePoint& now);
  bool stageRecordFiles(json& value, const TimePoint& stamp);
  bool stageOneFile(std::string& path, const TimePoint& stamp);

  // The scratch ring, created on the first Record that actually references a File so the many
  // Measurements that produce none never grow a scratch directory. Caller holds releaser_mutex_.
  const std::shared_ptr<dc_common::FileScratchRing>& scratchRing();

  // Where staged Files live: one stable directory per Measurement, next to the Files themselves
  // (same filesystem, so staging is a cheap local move) but outside the dated save tree.
  std::filesystem::path scratchDir() const;

  // Emits one Record belonging to an incident -- either released from the pre-roll buffer (stamped
  // with when it was collected, not when it was released) or collected live during post-roll.
  // Goes straight out through publish_, bypassing publish()'s gate/counter logic: a Record
  // captured for an incident was already unconditionally collected and isn't re-filtered.
  void publishIncidentRecord(const std::string& record_json, const TimePoint& stamp, const std::string& incident_id);

  // Logs `message` through log_ unless `last` is less than one throttle period old, then stamps
  // it. Stands in for the RCLCPP_*_THROTTLE calls this module cannot use.
  void logThrottled(LogLevel level, TimePoint& last, const TimePoint& now, const std::string& message);

  Config config_;
  PublishFn publish_;
  ValidationFailureFn on_validation_failure_;
  LogFn log_;

  dc_core::ConditionSet condition_set_;
  // The publish decision (gate latch, init quota, condition cap) and the Record shaping, both
  // ROS-free and directly tested (#482).
  PublishGate publish_gate_;
  RecordEnricher record_enricher_{ RecordEnricher::Config{}, {}, {} };

  // Validation
  bool enable_validator_{ true };
  json_validator validator_;
  json schema_;

  // Publish-enabling state, see collectible() (#499)
  bool enabled_{ true };
  bool configure_failed_{ false };

  // Incident capture: pre-event circular-buffer capture (#287) driven by the IncidentReleaser
  // state machine (#288). releaser_ is only created when buffer_duration_sec_ > 0; while it
  // exists, collected samples go to it instead of publish(), and a flush event starts one
  // buffer-release / post-roll / cooldown / re-arm cycle.
  std::shared_ptr<IncidentReleaser> releaser_;
  // The Files half of incident capture (#290): while the releaser buffers a Record instead of
  // publishing it, the Files that Record references are moved in here and the Record rewritten to
  // point at the staged copies, which age out on the same window as the Records themselves.
  // Created lazily by scratchRing(), on the first Record that references a File at all.
  std::shared_ptr<dc_common::FileScratchRing> file_scratch_ring_;
  // The releaser is reached from three concurrent paths -- the polling timer, the flush
  // subscription and the release timer -- so every entry into it, and into the scratch ring that
  // shadows it, is serialized here.
  std::mutex releaser_mutex_;

  // Throttle bookkeeping for the two log sites that had one, 10 s like the RCLCPP_*_THROTTLE
  // calls this module cannot use.
  TimePoint last_empty_warn_;
  TimePoint last_stage_error_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MEASUREMENT_CORE_HPP_
