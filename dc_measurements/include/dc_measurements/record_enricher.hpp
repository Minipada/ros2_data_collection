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
 * @brief The shaping Measurement applies to every Record before it goes out: one JSON parse in,
 * one finished Record out, with no ROS dependency (#482).
 *
 * The steps, in publish()'s order: schema validation, nesting under the measurement name,
 * flattening, run_id, tags, measurement name, measurement plugin, custom keys. The chain this
 * replaces re-parsed and re-serialised the whole Record once per step -- 7+ parse/dump
 * round-trips on every Record the stack publishes; this does one of each, so the finished Record
 * is byte-for-byte what the chain produced, down to the dump flags (compact, ensure_ascii) and
 * the always-present "flattened"/"nested" markers.
 *
 * The module has no logger: validation and parse failures are reported through the injected
 * callbacks. `validate` sees the parsed Record before any shaping and handles a rejected Record
 * itself (log, plugin hook) without stopping publication, exactly as validateJSON() did.
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
    bool enable_validator{ false };            ///< Hand the parsed Record to `validate` before shaping
  };

  /// Reports a Record the schema rejected; owns its logging and hook, and does not stop publication.
  using ValidateFn = std::function<void(const nlohmann::json&)>;
  /// Reports a Record that is not JSON at all; enrich() then leaves the data as it came in,
  /// modulo the custom-keys rebuild below.
  using ParseErrorFn = std::function<void(const std::string&)>;

  RecordEnricher(const Config& config, ValidateFn validate, ParseErrorFn on_parse_error)
    : config_(config), validate_(std::move(validate)), on_parse_error_(std::move(on_parse_error))
  {
  }

  /// Parses `data`, applies every configured step and returns the finished Record. Data that does
  /// not parse comes back unchanged (with one parse-error report) -- the single-parse form of what
  /// the per-step chain did, see unparsableFallback().
  std::string enrich(const std::string& data)
  {
    nlohmann::json record;
    try
    {
      record = nlohmann::json::parse(data);
    }
    catch (const nlohmann::json::parse_error& e)
    {
      if (on_parse_error_)
      {
        on_parse_error_(data);
      }
      return unparsableFallback(data);
    }

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

    return record.dump(-1, ' ', true);
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

  // What the per-step chain produced from data it could not parse: every step logged and left the
  // data alone, except a non-empty custom_keys config, whose step ran last and rebuilt the Record
  // from an empty object -- replacing it with the custom keys alone. Reproduced as-is, so the one
  // parse changes no output.
  std::string unparsableFallback(const std::string& data) const
  {
    if (config_.custom_keys.empty())
    {
      return data;
    }
    nlohmann::json record;
    addCustomKeys(record);
    return record.dump(-1, ' ', true);
  }

  Config config_;
  ValidateFn validate_;
  ParseErrorFn on_parse_error_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__RECORD_ENRICHER_HPP_
