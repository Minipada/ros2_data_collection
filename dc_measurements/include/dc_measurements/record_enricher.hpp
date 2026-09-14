// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__RECORD_ENRICHER_HPP_
#define DC_MEASUREMENTS__RECORD_ENRICHER_HPP_

#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

namespace dc_measurements
{

/**
 * @class dc_measurements::RecordEnricher
 * @brief The shaping Measurement applies to every Record before it goes out: one collected json
 * in, one finished Record json out, with no ROS dependency (#482, #500).
 *
 * The steps, in publish()'s order: schema validation, nesting under the measurement name,
 * flattening, run_id, tags, measurement name, measurement plugin, custom keys. The Record stays
 * json end to end -- the only serialization left in the pipeline is the publisher edge's.
 *
 * The module has no logger: a rejected Record is reported through the injected callback, which
 * handles it (log, plugin hook) without stopping publication, exactly as validateJSON() did.
 */
class RecordEnricher
{
public:
  struct Config
  {
    std::string measurement_name;              ///< Value of "name" and the key nested Records sit under
    std::string measurement_plugin;            ///< Value of "plugin"
    bool nested{ false };                      ///< Nest the collected JSON under the measurement name
    bool flatten{ false };                     ///< Flatten the collected JSON (after nesting)
    bool run_id_enabled{ false };              ///< Add "run_id"
    std::string run_id;                        ///< Value of "run_id"
    bool include_measurement_name{ false };    ///< Add "name"
    bool include_measurement_plugin{ false };  ///< Add "plugin"
    std::vector<std::string> tags;             ///< Added as "tags" when not empty
    std::vector<nlohmann::json> custom_keys;   ///< {key, value, override} entries, added as plain keys
    bool enable_validator{ false };            ///< Hand the collected Record to `validate` before shaping
  };

  /// Reports a Record the schema rejected; owns its logging and hook, and does not stop publication.
  using ValidateFn = std::function<void(const nlohmann::json&)>;

  RecordEnricher(const Config& config, ValidateFn validate) : config_(config), validate_(std::move(validate))
  {
  }

  /// Applies every configured step to `record` and returns the finished Record, enriched and
  /// unserialized -- Conditions are polled with this same json (#500).
  nlohmann::json enrich(nlohmann::json record)
  {
    if (config_.enable_validator && validate_)
    {
      validate_(record);
    }

    if (config_.nested)
    {
      nlohmann::json nested;
      nested[config_.measurement_name] = std::move(record);
      record = std::move(nested);
    }

    if (config_.flatten)
    {
      record = record.flatten();
      record["flattened"] = true;
    }
    else
    {
      record["flattened"] = false;
    }
    record["nested"] = config_.nested;

    if (config_.run_id_enabled)
    {
      record["run_id"] = config_.run_id;
    }
    if (!config_.tags.empty())
    {
      record["tags"] = config_.tags;
    }
    if (config_.include_measurement_name)
    {
      record["name"] = config_.measurement_name;
    }
    if (config_.include_measurement_plugin)
    {
      record["plugin"] = config_.measurement_plugin;
    }
    if (!config_.custom_keys.empty())
    {
      addCustomKeys(record);
    }

    return record;
  }

private:
  // Sets the configured custom keys -- an existing key keeps its value unless its entry says
  // override -- and names them under "custom_keys" so the Bridge can carry the labelling onto
  // File metadata Records too (#419).
  void addCustomKeys(nlohmann::json& record) const
  {
    nlohmann::json declared = nlohmann::json::array();
    for (const auto& param : config_.custom_keys)
    {
      const auto key = param["key"].get<std::string>();
      const auto value = param["value"].get<std::string>();
      const auto override_value = param["override"].get<bool>();
      if (!record.contains(key) || (record.contains(key) && override_value))
      {
        record[key] = value;
      }
      declared.push_back(key);
    }
    record["custom_keys"] = std::move(declared);
  }

  Config config_;
  ValidateFn validate_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__RECORD_ENRICHER_HPP_
