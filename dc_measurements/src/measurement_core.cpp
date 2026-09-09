// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/measurement_core.hpp"

#include <cctype>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_common/record_file_walk.hpp"

namespace dc_measurements
{
namespace
{
// The two throttled log sites the RCLCPP_*_THROTTLE calls used to own.
constexpr std::chrono::milliseconds kThrottlePeriod{ 10000 };
}  // namespace

MeasurementCore::MeasurementCore() = default;

MeasurementCore::~MeasurementCore() = default;

nlohmann::json_schema::schema_loader makeSchemaFileLoader(const std::string& schema_dir)
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

void MeasurementCore::configure(const Config& config, PublishFn publish, ValidationFailureFn on_validation_failure,
                                LogFn log)
{
  config_ = config;
  publish_ = std::move(publish);
  on_validation_failure_ = std::move(on_validation_failure);
  log_ = std::move(log);

  condition_set_ = dc_core::ConditionSet(config.if_all_conditions, config.if_any_conditions, config.if_none_conditions);

  // The publish decision -- gate latch, init quota, condition cap -- is state the ROS layer only
  // feeds and reads back (#482).
  PublishGate::Config gate_config;
  gate_config.init_max = config.init_max_measurements;
  gate_config.condition_max = config.condition_max_measurements;
  gate_config.has_conditions = !condition_set_.empty();
  gate_config.gate_enabled = !config.gate_condition.empty();
  publish_gate_ = PublishGate(gate_config);

  RecordEnricher::Config enricher_config;
  enricher_config.measurement_name = config.measurement_name;
  enricher_config.measurement_plugin = config.measurement_plugin;
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
      [this](const std::string& data) { log_(LogLevel::Error, "Error parsing JSON while enriching: " + data); });

  if (config.buffer_duration_sec > 0.0)
  {
    releaser_ = std::make_shared<IncidentReleaser>(
        durationFromSeconds(config.buffer_duration_sec), durationFromSeconds(config.post_roll_duration_sec),
        durationFromSeconds(config.cooldown_sec), config.max_flush_rate_hz,
        [this](const std::string& record_json, const TimePoint& stamp, const std::string& incident_id) {
          publishIncidentRecord(record_json, stamp, incident_id);
        });
  }
}

void MeasurementCore::publish(const std::string& data, const std::string& group_key, std::int64_t stamp_ns,
                              const ConditionStateLookup& gate_state, const ConditionStateLookup& conditions,
                              const TimePoint& now)
{
  if (data.empty() || data == "null")
  {
    // Not necessarily a fault: a Measurement that found nothing to report returns an empty
    // message on purpose -- Camera does exactly that on every frame with no barcode in view,
    // which is most frames of the QR-code demo. Throttled and worded accordingly, because at the
    // polling rate this used to read like a broken pipeline and sent #279 chasing one that was
    // fine.
    logThrottled(LogLevel::Warn, last_empty_warn_, now,
                 "No data collected from measurement " + config_.measurement_name +
                     " (nothing to report, or it is not receiving input)");
    return;
  }

  std::string enriched = data;
  enrich(enriched);

  if (!gateOpen(gate_state))
  {
    log_(LogLevel::Debug, "Measurement " + config_.measurement_name + ": gate_condition '" + config_.gate_condition +
                              "' not yet true, dropping collection.");
    return;
  }

  if (publish_gate_.offer(conditionSetOn(conditions)))
  {
    publish_(RecordOut{ enriched, group_key, {}, stamp_ns });
  }
}

void MeasurementCore::offerSample(const std::string& data, const TimePoint& now)
{
  if (!releaser_)
  {
    return;
  }
  if (data.empty() || data == "null")
  {
    // Nothing to offer, but the phase deadlines and a rate-limited release still need driving.
    const std::lock_guard<std::mutex> lock(releaser_mutex_);
    releaser_->tick(now);
    return;
  }
  std::string enriched = data;
  enrich(enriched);
  const std::lock_guard<std::mutex> lock(releaser_mutex_);
  // Drive the phase transitions before deciding what to do with this sample's Files: during
  // post-roll the Record is published live and its Files are left exactly where the Measurement
  // wrote them, which is what the Bridge has always picked up.
  releaser_->tick(now);
  if (releaser_->state() != IncidentState::PostRoll)
  {
    stageSampleFiles(enriched, now);
  }
  releaser_->offer(enriched, now);
}

void MeasurementCore::onFlushEvent(const std::string& incident_id, const TimePoint& now)
{
  if (!releaser_)
  {
    return;
  }
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
  releaser_->onFlush(incident_id, now);
  if (file_scratch_ring_ && starts_cycle)
  {
    // The released Records are on their way to the Bridge, which owns their Files from here
    // (intent queue, upload, retention sweep, delete_when_sent). Stop tracking them so no later
    // evict() can delete a File out from under an upload in flight -- #290's whole point being
    // that the Bridge needs no change to ingest them, however old their timestamps are.
    file_scratch_ring_->release();
  }
}

void MeasurementCore::tickRelease(const TimePoint& now)
{
  const std::lock_guard<std::mutex> lock(releaser_mutex_);
  if (releaser_)
  {
    releaser_->tick(now);
  }
}

void MeasurementCore::teardown()
{
  const std::lock_guard<std::mutex> lock(releaser_mutex_);
  releaser_.reset();
  if (file_scratch_ring_)
  {
    // The buffered Records go with the releaser, so the Files they referenced would be orphaned
    // in the scratch directory: nothing is left to publish them. Released ones are untracked by
    // now (onFlushEvent()) and are the Bridge's, so purge() can't touch them.
    file_scratch_ring_->purge();
    file_scratch_ring_.reset();
  }
}

bool MeasurementCore::hasReleaser() const
{
  return releaser_ != nullptr;
}

bool MeasurementCore::collectionFinished() const
{
  return publish_gate_.collectionFinished();
}

bool MeasurementCore::collectible() const
{
  return enabled_ && !configure_failed_;
}

void MeasurementCore::setEnabled(bool enabled)
{
  enabled_ = enabled;
}

void MeasurementCore::markConfigureFailed()
{
  configure_failed_ = true;
}

void MeasurementCore::validateSchema(const std::string& package_name, const std::string& json_filename)
{
  const std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
  const std::string schema_dir = package_share_directory + "/plugins/measurements/json";
  validateSchema(schema_dir + "/" + json_filename);
}

void MeasurementCore::validateSchema(const std::string& json_schema_path)
{
  std::ifstream f(json_schema_path);
  try
  {
    schema_ = json::parse(f);

    log_(LogLevel::Info, "Looking for schema at " + json_schema_path);
    log_(LogLevel::Info, "schema: " + schema_.dump());
    try
    {
      const std::string schema_dir = std::filesystem::path(json_schema_path).parent_path().string();
      validator_ = json_validator(makeSchemaFileLoader(schema_dir));
      validator_.set_root_schema(schema_);
    }
    catch (const std::exception& e)
    {
      const std::string err = std::string("Validation of schema failed: ") + e.what();
      log_(LogLevel::Error, err);
      throw std::runtime_error{ err };
    }
  }
  catch (json::parse_error& e)
  {
    log_(LogLevel::Error, "Error parsing JSON file json_schema_path: " + json_schema_path);
  }
}

void MeasurementCore::validateDefaultSchema()
{
  const size_t sep = config_.measurement_plugin.find('/');
  const std::string package =
      sep == std::string::npos ? std::string("dc_measurements") : config_.measurement_plugin.substr(0, sep);
  validateSchema(package, defaultSchemaFile(config_.measurement_plugin));
}

bool MeasurementCore::schemaEmpty() const
{
  return schema_.empty();
}

std::string MeasurementCore::snakeCase(const std::string& camel)
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

std::string MeasurementCore::defaultSchemaFile(const std::string& plugin_lookup_name)
{
  const size_t sep = plugin_lookup_name.find('/');
  const std::string& plugin_type = sep == std::string::npos ? plugin_lookup_name : plugin_lookup_name.substr(sep + 1);
  return snakeCase(plugin_type) + ".json";
}

void MeasurementCore::enrich(std::string& data)
{
  data = record_enricher_.enrich(data);
}

void MeasurementCore::validateJSON(const json& data_json)
{
  try
  {
    validator_.validate(data_json);
  }
  catch (const std::exception& e)
  {
    log_(LogLevel::Error, "Validation failed: " + std::string(e.what()) + "data=" + data_json.dump());
    if (on_validation_failure_)
    {
      on_validation_failure_(data_json);
    }
  }
}

bool MeasurementCore::gateOpen(const ConditionStateLookup& gate_state)
{
  // An open gate -- no gate_condition configured, or already latched -- is never re-consulted.
  if (publish_gate_.gateOpen())
  {
    return true;
  }

  const bool open = publish_gate_.openGate(gate_state(config_.gate_condition));
  if (open)
  {
    log_(LogLevel::Info, "Measurement " + config_.measurement_name + ": gate_condition '" + config_.gate_condition +
                             "' became true, collection will proceed normally from now on.");
  }
  return open;
}

bool MeasurementCore::conditionSetOn(const ConditionStateLookup& conditions)
{
  if (condition_set_.empty())
  {
    return false;
  }

  // The if_all/if_any/if_none composition itself lives in dc_core so the Trigger plugins can
  // compose Conditions the same way (#284); the state lookup stays with the ROS driver.
  const auto outcome = condition_set_.evaluate(conditions);

  log_(LogLevel::Debug, "all_conditions_res=" + std::to_string(outcome.if_all_satisfied) +
                            ", any_conditions_res=" + std::to_string(outcome.if_any_satisfied) +
                            ", none_conditions_res=" + std::to_string(outcome.if_none_satisfied));

  return outcome.satisfied();
}

void MeasurementCore::stageSampleFiles(std::string& data, const TimePoint& now)
{
  json data_json;
  try
  {
    data_json = json::parse(data);
  }
  catch (json::parse_error& e)
  {
    // enrich() already logged whatever made this unparsable; buffer it as-is.
    return;
  }
  if (stageRecordFiles(data_json, now))
  {
    data = data_json.dump(-1, ' ', true);
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

bool MeasurementCore::stageRecordFiles(json& value, const TimePoint& stamp)
{
  // The Record walk itself is owned by dc_common's record_file_walk (#479) -- the same one
  // dc_bridge's parse_file_group() goes through, so exactly the paths the Bridge would have
  // uploaded are the ones that move, in either written form (nested local_paths objects, or a
  // flattened Record's JSON-pointer keys).
  return dc_common::record_file_walk::rewrite_local_paths(value,
                                                          [&](std::string& path) { return stageOneFile(path, stamp); });
}

bool MeasurementCore::stageOneFile(std::string& path, const TimePoint& stamp)
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
    logThrottled(LogLevel::Error, last_stage_error_, stamp,
                 "Measurement " + config_.measurement_name + ": could not stage File " + original +
                     " into the incident scratch ring (" + e.what() +
                     "); buffering the Record with the File left "
                     "in place.");
    return false;
  }
}

const std::shared_ptr<dc_common::FileScratchRing>& MeasurementCore::scratchRing()
{
  if (!file_scratch_ring_)
  {
    file_scratch_ring_ =
        std::make_shared<dc_common::FileScratchRing>(scratchDir(), durationFromSeconds(config_.buffer_duration_sec));
  }
  return file_scratch_ring_;
}

std::filesystem::path MeasurementCore::scratchDir() const
{
  // The literal prefix of save_local_base_path_expanded is used -- everything up to its first
  // strftime token -- so the scratch directory doesn't roll over every hour the way the save
  // path does.
  std::string base = config_.save_local_base_path_expanded;
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
  return std::filesystem::path(base) / ".dc_incident_scratch" / config_.measurement_name;
}

void MeasurementCore::publishIncidentRecord(const std::string& record_json, const TimePoint& stamp,
                                            const std::string& incident_id)
{
  // The Incident rides the typed envelope field, not a key injected into the payload (#506): no
  // serialize / re-parse / re-dump round trip, and the payload stays pure member data.
  RecordOut out;
  out.data = record_json;
  out.group_key = config_.group_key;
  out.incident_id = incident_id;
  out.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
  publish_(out);
}

void MeasurementCore::logThrottled(LogLevel level, TimePoint& last, const TimePoint& now, const std::string& message)
{
  if (now < last + kThrottlePeriod)
  {
    return;
  }
  last = now;
  log_(level, message);
}

}  // namespace dc_measurements
