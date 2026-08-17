#include "dc_bridge/render.hpp"

#include <cstdio>
#include <map>
#include <set>
#include <sstream>
#include <toml++/toml.hpp>

#include "dc_bridge/topic_config.hpp"

namespace dc_bridge
{

namespace
{

const std::set<std::string>& reserved_component_ids()
{
  static const std::set<std::string> ids = { SOURCE_ID, NORMALIZE_TRANSFORM_ID, ROUTE_TRANSFORM_ID };
  return ids;
}

// A required stringly field: empty/absent → MissingField.
std::string required_field(const std::string& name, const std::optional<std::string>& value, const std::string& field)
{
  if (value && !value->empty())
  {
    return *value;
  }
  throw RenderError(RenderErrorKind::MissingField, "destination '" + name + "': missing required field `" + field + "`",
                    name, field);
}

// An optional stringly field: empty → nullopt.
std::optional<std::string> optional_field(const std::optional<std::string>& value)
{
  if (value && !value->empty())
  {
    return *value;
  }
  return std::nullopt;
}

// Debug-quotes a string for the tag/route VRL literals: double-quoted, with `"` and `\`
// escaped — the quoting the checked-in gold fixtures expect. Tags are derived from topic
// names (alphanumeric/underscore/dot), so no other escaping arises in practice; handled
// minimally for safety.
std::string debug_quote(const std::string& s)
{
  std::string out = "\"";
  for (char c : s)
  {
    if (c == '"' || c == '\\')
    {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

// The `route` transform branch key a Tag namespace is exposed under: the prefix with its
// trailing separator dot(s) dropped, so `dc.raw.` → branch `dc.raw` → public route
// `dc.dc.raw` (route_output_for_tag_prefix), reading exactly like an exact-Tag branch.
std::string route_branch_for_tag_prefix(const std::string& prefix)
{
  std::string branch = prefix;
  while (!branch.empty() && branch.back() == '.')
  {
    branch.pop_back();
  }
  return branch;
}

// The VRL predicate matching every Tag in a namespace.
//
// `to_string(.tag) ?? ""` rather than a bare `.tag`: VRL types an event field as `any`,
// and `starts_with` demands an exact string — Vector rejects the whole config at
// `validate` time otherwise ("this expression resolves to any", E110). The `??` fallback
// keeps the predicate infallible, so a Record that somehow arrives with a non-string tag
// simply doesn't match instead of aborting the transform. `includes()` (the exact-Tag
// half) has no such requirement, which is why only this side needs the coercion.
std::string tag_prefix_predicate(const std::string& prefix)
{
  return "starts_with(to_string(.tag) ?? \"\", " + debug_quote(prefix) + ")";
}

bool is_valid_destination_name(const std::string& name)
{
  if (name.empty())
  {
    return false;
  }
  auto is_alpha = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); };
  auto is_alnum_us = [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
  };
  if (!is_alpha(name.front()))
  {
    return false;
  }
  for (std::size_t i = 1; i < name.size(); ++i)
  {
    if (!is_alnum_us(name[i]))
    {
      return false;
    }
  }
  return true;
}

std::string trim(const std::string& s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos)
  {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

// The distinct shipper ingest protocol Tags a Destination's inputs topics derive to,
// plus its Bridge-internal extra_tags — sorted+unique (std::set) so rendered arrays/VRL
// are deterministic.
std::set<std::string> destination_tags(const Destination& dest)
{
  std::set<std::string> tags;
  for (const auto& topic : dest.inputs)
  {
    tags.insert(TopicConfig::derive_tag(topic));
  }
  for (const auto& tag : dest.extra_tags)
  {
    tags.insert(tag);
  }
  return tags;
}

// The Tag namespaces routed to a Destination with `starts_with` rather than equality
// (raw mode, #227) — sorted+unique for the same determinism reason as destination_tags.
std::set<std::string> destination_tag_prefixes(const Destination& dest)
{
  return std::set<std::string>(dest.tag_prefixes.begin(), dest.tag_prefixes.end());
}

// The VRL condition selecting a Destination's events: exact Tags via `includes(...)`,
// plus one `starts_with` per routed Tag namespace. Either half may be empty (a
// raw-only Destination has no exact Tags at all), never both — validate() rejects that.
std::string tag_condition(const std::set<std::string>& tags, const std::set<std::string>& prefixes)
{
  std::vector<std::string> terms;
  if (!tags.empty())
  {
    std::string joined;
    bool first = true;
    for (const auto& t : tags)
    {
      if (!first)
      {
        joined += ", ";
      }
      joined += debug_quote(t);
      first = false;
    }
    terms.push_back("includes([" + joined + "], .tag)");
  }
  for (const auto& prefix : prefixes)
  {
    terms.push_back(tag_prefix_predicate(prefix));
  }
  std::string out;
  for (std::size_t i = 0; i < terms.size(); ++i)
  {
    if (i > 0)
    {
      out += " || ";
    }
    out += terms[i];
  }
  return out;
}

// Normalizes the Record envelope's `incident_id` (#291) for a Destination whose sink maps
// top-level keys onto table columns.
//
// Vector's `postgres` sink has no column-mapping options of its own: it hands the event to
// `jsonb_populate_record`, so a top-level key lands in the same-named column and anything
// else is dropped. There is therefore no `columns = [...]` to write — this block is where the
// generated config names `incident_id` as a field DC maps onto a column, which is what makes
// the column part of the rendered contract (and what the render tests assert) rather than an
// undocumented consequence of the key happening to be top-level.
//
// `to_string(...) ?? null` for the same reason the route predicates coerce `.tag`: VRL types
// an event field as `any`. A Measurement whose own payload already carries a non-string
// `incident_id` (an object, say) would otherwise reach the sink as a value the text column
// cannot take and fail the whole insert batch; coerced, that Record still lands, with a NULL
// in the column. Guarded by `exists` because `to_string(null)` is `""` — a Record that is not
// part of an incident must leave the column NULL, not empty-string.
std::string render_incident_id_normalization()
{
  return "  if exists(.incident_id) {\n"
         "    .incident_id = to_string(.incident_id) ?? null\n"
         "  }\n";
}

std::string render_normalize_source(const RenderConfig& config)
{
  std::string out;
  for (const auto& dest : config.destinations)
  {
    out += "if " + tag_condition(destination_tags(dest), destination_tag_prefixes(dest)) + " {\n";
    if (dest.time_format == TimeFormat::EpochNanos)
    {
      // Exact: an i64 of nanoseconds since the epoch, no float rounding anywhere. Good
      // until the year 2262, and the only format here that can round-trip the full
      // resolution the Forwarder now sends (#308).
      out += "  ." + dest.time_key + " = to_unix_timestamp!(.timestamp, unit: \"nanoseconds\")\n";
    }
    else if (dest.time_format == TimeFormat::Double)
    {
      out +=
          "  ." + dest.time_key + " = to_float(to_unix_timestamp!(.timestamp, unit: \"nanoseconds\")) / 1000000000.0\n";
    }
    else
    {
      out += "  ." + dest.time_key + " = format_timestamp!(.timestamp, format: \"%Y-%m-%dT%H:%M:%S%.9f\")\n";
    }
    if (std::holds_alternative<PostgresParams>(dest.kind))
    {
      out += render_incident_id_normalization();
    }
    out += "}\n";
  }
  return out;
}

// Percent-encodes a postgres URI userinfo component byte-by-byte so arbitrary credential
// characters can't corrupt the postgres://user:pass@host:port/db endpoint.
std::string percent_encode_userinfo(const std::string& value)
{
  std::string out;
  for (unsigned char byte : value)
  {
    char c = static_cast<char>(byte);
    bool unreserved = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' ||
                      c == '.' || c == '_' || c == '~';
    if (unreserved)
    {
      out.push_back(c);
    }
    else
    {
      char hex[4];
      std::snprintf(hex, sizeof(hex), "%%%02X", byte);
      out += hex;
    }
  }
  return out;
}

toml::array sink_inputs(const Destination& dest)
{
  toml::array inputs;
  for (const auto& tag : destination_tags(dest))
  {
    inputs.push_back(route_output_for_tag(tag));
  }
  for (const auto& prefix : destination_tag_prefixes(dest))
  {
    inputs.push_back(route_output_for_tag_prefix(prefix));
  }
  return inputs;
}

toml::table disk_buffer(const RenderConfig& config)
{
  toml::table buffer;
  buffer.insert("type", "disk");
  buffer.insert("max_size", static_cast<std::int64_t>(config.buffer_max_bytes));
  return buffer;
}

toml::table json_encoding()
{
  toml::table encoding;
  encoding.insert("codec", "json");
  return encoding;
}

toml::table render_sink(const RenderConfig& config, const Destination& dest)
{
  toml::table sink;
  sink.insert("inputs", sink_inputs(dest));

  if (const auto* pg = std::get_if<PostgresParams>(&dest.kind))
  {
    sink.insert("type", "postgres");
    sink.insert("endpoint", "postgres://" + percent_encode_userinfo(pg->user) + ":" +
                                percent_encode_userinfo(pg->password) + "@" + pg->host + ":" +
                                std::to_string(pg->port) + "/" + pg->database);
    sink.insert("table", pg->table);
    sink.insert("buffer", disk_buffer(config));
  }
  else if (const auto* s3 = std::get_if<S3Params>(&dest.kind))
  {
    sink.insert("type", "aws_s3");
    sink.insert("bucket", s3->bucket);
    if (s3->region)
    {
      sink.insert("region", *s3->region);
    }
    if (s3->endpoint)
    {
      sink.insert("endpoint", *s3->endpoint);
    }
    if (s3->key_prefix)
    {
      sink.insert("key_prefix", *s3->key_prefix);
    }
    if (s3->force_path_style)
    {
      sink.insert("force_path_style", *s3->force_path_style);
    }
    if (s3->auth)
    {
      toml::table auth;
      auth.insert("access_key_id", s3->auth->access_key_id);
      auth.insert("secret_access_key", s3->auth->secret_access_key);
      sink.insert("auth", std::move(auth));
    }
    if (s3->batch_timeout_secs)
    {
      toml::table batch;
      batch.insert("timeout_secs", static_cast<std::int64_t>(*s3->batch_timeout_secs));
      sink.insert("batch", std::move(batch));
    }
    sink.insert("encoding", json_encoding());
    sink.insert("buffer", disk_buffer(config));
  }
  else if (const auto* file = std::get_if<FileParams>(&dest.kind))
  {
    sink.insert("type", "file");
    sink.insert("path", file->path);
    sink.insert("encoding", json_encoding());
    sink.insert("buffer", disk_buffer(config));
  }
  else
  {  // Console
    sink.insert("type", "console");
    sink.insert("target", "stdout");
    sink.insert("encoding", json_encoding());
    // No disk buffer: console is a debugging sink.
  }
  return sink;
}

void validate(const RenderConfig& config)
{
  if (trim(config.data_dir).empty())
  {
    throw RenderError(RenderErrorKind::EmptyDataDir, "shipper.data_dir must not be empty");
  }
  if (config.destinations.empty())
  {
    throw RenderError(RenderErrorKind::NoDestinations,
                      "no destinations configured; add at least one entry to the `destinations` parameter");
  }
  if (config.buffer_max_bytes < MIN_DISK_BUFFER_BYTES)
  {
    throw RenderError(RenderErrorKind::BufferTooSmall,
                      "shipper.buffer_max_bytes (" + std::to_string(config.buffer_max_bytes) +
                          ") is below Vector's disk-buffer minimum",
                      "", "", static_cast<std::int64_t>(config.buffer_max_bytes));
  }

  std::set<std::string> seen;
  for (const auto& dest : config.destinations)
  {
    if (!is_valid_destination_name(dest.name))
    {
      throw RenderError(RenderErrorKind::InvalidDestinationName,
                        "destination '" + dest.name +
                            "': name must start with a letter and contain only letters, digits, or underscores",
                        dest.name);
    }
    if (reserved_component_ids().count(dest.name))
    {
      throw RenderError(RenderErrorKind::ReservedDestinationName,
                        "destination '" + dest.name +
                            "': name is reserved by the DC-generated Vector config; pick another name",
                        dest.name);
    }
    if (!seen.insert(dest.name).second)
    {
      throw RenderError(RenderErrorKind::DuplicateDestination, "duplicate destination name '" + dest.name + "'",
                        dest.name);
    }
    if (dest.inputs.empty() && dest.extra_tags.empty() && dest.tag_prefixes.empty())
    {
      throw RenderError(RenderErrorKind::EmptyInputs, "destination '" + dest.name + "': `inputs` must not be empty",
                        dest.name);
    }
    if (dest.receives != Receives::Records)
    {
      throw RenderError(RenderErrorKind::FilesDestinationInShipperConfig,
                        "internal error: `receives: files` destination '" + dest.name +
                            "' reached the shipper config renderer",
                        dest.name);
    }
  }

  // A prefix branch and an exact-Tag branch are both keys in the same `route` table, so
  // a Tag equal to a prefix's branch id (Tag `dc.raw` against prefix `dc.raw.`) would
  // have one silently overwrite the other and send that Destination's data to the wrong
  // sink. Vanishingly unlikely — it takes a topic literally named `/dc/raw` — but a
  // silent mis-route is exactly the failure that must not be possible, so it's a loud
  // config error instead.
  std::set<std::string> prefix_branches;
  std::set<std::string> exact_tags;
  for (const auto& dest : config.destinations)
  {
    for (const auto& prefix : dest.tag_prefixes)
    {
      prefix_branches.insert(route_branch_for_tag_prefix(prefix));
    }
    for (const auto& tag : destination_tags(dest))
    {
      exact_tags.insert(tag);
    }
  }
  for (const auto& tag : exact_tags)
  {
    if (prefix_branches.count(tag))
    {
      throw RenderError(RenderErrorKind::TagPrefixCollidesWithTag,
                        "Tag '" + tag + "' collides with the routed Tag namespace '" + tag +
                            ".'; rename the topic or the raw tag_prefix",
                        tag);
    }
  }
}

std::set<std::string> rendered_component_ids(const RenderConfig& config)
{
  std::set<std::string> ids = reserved_component_ids();
  for (const auto& dest : config.destinations)
  {
    ids.insert(dest.name);
  }
  return ids;
}

}  // namespace

std::string route_output_for_tag(const std::string& tag)
{
  return std::string(ROUTE_TRANSFORM_ID) + "." + tag;
}

std::string route_output_for_tag_prefix(const std::string& prefix)
{
  return std::string(ROUTE_TRANSFORM_ID) + "." + route_branch_for_tag_prefix(prefix);
}

PostgresParams postgres_from_raw(const std::string& name, const RawDestinationParams& raw)
{
  PostgresParams pg;
  pg.user = required_field(name, raw.user, "user");
  pg.password = required_field(name, raw.password, "password");
  pg.database = required_field(name, raw.database, "database");
  pg.table = required_field(name, raw.table, "table");
  pg.host = optional_field(raw.host).value_or("127.0.0.1");

  std::int64_t port_raw = raw.port.value_or(5432);
  if (port_raw <= 0 || port_raw > 65535)
  {
    throw RenderError(RenderErrorKind::InvalidPort,
                      "destination '" + name + "': port " + std::to_string(port_raw) +
                          " is out of range (expected 1-65535)",
                      name, "", port_raw);
  }
  pg.port = static_cast<std::uint16_t>(port_raw);
  return pg;
}

S3Params s3_from_raw(const std::string& name, const RawDestinationParams& raw)
{
  S3Params s3;
  s3.bucket = required_field(name, raw.bucket, "bucket");

  auto access = optional_field(raw.access_key_id);
  auto secret = optional_field(raw.secret_access_key);
  if (access && secret)
  {
    s3.auth = S3Auth{ *access, *secret };
  }
  else if (access || secret)
  {
    throw RenderError(
        RenderErrorKind::IncompleteS3Auth,
        "destination '" + name + "': access_key_id and secret_access_key must be set together (got only one)", name);
  }

  if (raw.batch_timeout_secs)
  {
    std::int64_t secs = *raw.batch_timeout_secs;
    if (secs <= 0)
    {
      throw RenderError(RenderErrorKind::InvalidBatchTimeout,
                        "destination '" + name + "': batch_timeout_secs " + std::to_string(secs) +
                            " is invalid (expected a positive integer)",
                        name, "", secs);
    }
    s3.batch_timeout_secs = static_cast<std::uint64_t>(secs);
  }

  s3.region = optional_field(raw.region);
  s3.endpoint = optional_field(raw.endpoint);
  s3.key_prefix = optional_field(raw.key_prefix);
  s3.force_path_style = raw.force_path_style;
  return s3;
}

FileParams file_from_raw(const std::string& name, const RawDestinationParams& raw)
{
  return FileParams{ required_field(name, raw.path, "path") };
}

Destination destination_from_raw(const std::string& name, const std::string& type_str, const std::string& receives_str,
                                 std::vector<std::string> inputs, const RawDestinationParams& raw)
{
  Receives receives;
  if (receives_str == "records")
  {
    receives = Receives::Records;
  }
  else if (receives_str == "files")
  {
    receives = Receives::Files;
  }
  else
  {
    throw RenderError(RenderErrorKind::InvalidReceives,
                      "destination '" + name + "': receives '" + receives_str +
                          "' is invalid (expected 'records' or 'files')",
                      name, receives_str);
  }

  if (receives == Receives::Files && type_str != "s3")
  {
    throw RenderError(RenderErrorKind::FilesRequireObjectStorage,
                      "destination '" + name + "': `receives: files` requires `type: s3` (object storage); got type '" +
                          type_str + "'",
                      name, type_str);
  }

  std::string time_key = optional_field(raw.time_key).value_or("date");
  std::string tf = raw.time_format && !raw.time_format->empty() ? *raw.time_format : "epoch_nanos";
  TimeFormat time_format;
  if (tf == "epoch_nanos")
  {
    time_format = TimeFormat::EpochNanos;
  }
  else if (tf == "double")
  {
    time_format = TimeFormat::Double;
  }
  else if (tf == "iso8601")
  {
    time_format = TimeFormat::Iso8601;
  }
  else
  {
    throw RenderError(RenderErrorKind::InvalidTimeFormat,
                      "destination '" + name + "': time_format '" + tf +
                          "' is invalid (expected 'epoch_nanos', 'double' or 'iso8601')",
                      name, tf);
  }

  DestinationKind kind;
  if (type_str == "postgres")
  {
    kind = postgres_from_raw(name, raw);
  }
  else if (type_str == "s3")
  {
    kind = s3_from_raw(name, raw);
  }
  else if (type_str == "file")
  {
    kind = file_from_raw(name, raw);
  }
  else if (type_str == "console")
  {
    kind = ConsoleParams{};
  }
  else
  {
    throw RenderError(RenderErrorKind::UnsupportedType,
                      "destination '" + name + "': type '" + type_str +
                          "' is not a blessed destination type (supported: postgres, s3, file, console)",
                      name, type_str);
  }

  Destination dest;
  dest.name = name;
  dest.receives = receives;
  dest.inputs = std::move(inputs);
  dest.time_key = std::move(time_key);
  dest.time_format = time_format;
  dest.kind = std::move(kind);
  return dest;
}

std::string expand_env(const std::string& input,
                       const std::function<std::optional<std::string>(const std::string&)>& lookup)
{
  std::string out;
  std::size_t i = 0;
  const std::size_t n = input.size();
  auto is_alpha_us = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; };
  auto is_alnum_us = [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
  };
  while (i < n)
  {
    if (input[i] == '$' && i + 1 < n && input[i + 1] == '{')
    {
      auto close = input.find('}', i + 2);
      if (close != std::string::npos)
      {
        std::string name = input.substr(i + 2, close - (i + 2));
        auto value = lookup(name);
        if (!value)
        {
          throw ExpandError(name, input);
        }
        out += *value;
        i = close + 1;
        continue;
      }
    }
    else if (input[i] == '$' && i + 1 < n && is_alpha_us(input[i + 1]))
    {
      std::size_t start = i + 1;
      std::size_t end = start;
      while (end < n && is_alnum_us(input[end]))
      {
        ++end;
      }
      std::string name = input.substr(start, end - start);
      auto value = lookup(name);
      if (!value)
      {
        throw ExpandError(name, input);
      }
      out += *value;
      i = end;
      continue;
    }
    out.push_back(input[i]);
    ++i;
  }
  return out;
}

void validate_custom_config_files(const RenderConfig& config, const std::vector<CustomConfigFile>& files)
{
  // id -> owner: nullopt means "owned by the rendered config"; a path means that snippet.
  std::map<std::string, std::optional<std::string>> owners;
  for (const auto& id : rendered_component_ids(config))
  {
    owners.emplace(id, std::nullopt);
  }

  for (const auto& file : files)
  {
    toml::table parsed;
    try
    {
      parsed = toml::parse(file.content);
    }
    catch (const toml::parse_error& e)
    {
      throw RenderError(RenderErrorKind::CustomConfigParse,
                        "custom config file '" + file.path + "' is not valid TOML: " + std::string(e.description()),
                        file.path);
    }

    std::vector<std::string> component_ids;
    for (const char* section : { "sources", "transforms", "sinks", "enrichment_tables" })
    {
      if (auto node = parsed[section].as_table())
      {
        for (const auto& [key, _] : *node)
        {
          component_ids.push_back(std::string(key.str()));
        }
      }
    }
    if (component_ids.empty())
    {
      throw RenderError(RenderErrorKind::CustomConfigEmpty,
                        "custom config file '" + file.path +
                            "' defines no sources, transforms, sinks, or enrichment tables",
                        file.path);
    }

    for (const auto& id : component_ids)
    {
      auto it = owners.find(id);
      if (it != owners.end())
      {
        if (!it->second.has_value())
        {
          throw RenderError(RenderErrorKind::CustomConfigReservedCollision,
                            "custom config file '" + file.path + "' defines component '" + id +
                                "', which collides with a component of the DC-generated Vector config",
                            file.path, id);
        }
        throw RenderError(RenderErrorKind::CustomConfigDuplicate,
                          "custom config files '" + *it->second + "' and '" + file.path + "' both define component '" +
                              id + "'",
                          *it->second, file.path);  // arg0=first path, arg1=second path
      }
      owners.emplace(id, file.path);
    }
  }
}

std::string render(const RenderConfig& config)
{
  validate(config);

  toml::table root;
  root.insert("data_dir", config.data_dir);

  // Global acknowledgements (#266): the Bridge's Forwarder tags every frame with a
  // `chunk` id and expects an `ack` in return, so the fluent source's chunk/ack support
  // must actually be switched on. Enabled globally rather than per-source: Vector 0.57
  // deprecates enabling acknowledgements on the source itself in favor of this form
  // (verified against the pinned binary — the source-level knob logs a deprecation
  // warning where this one doesn't, with identical ack behavior either way).
  toml::table acknowledgements;
  acknowledgements.insert("enabled", true);
  root.insert("acknowledgements", std::move(acknowledgements));

  // sources
  toml::table source;
  source.insert("type", "fluent");
  source.insert("address", config.forward_host + ":" + std::to_string(config.forward_port));
  toml::table sources;
  sources.insert(SOURCE_ID, std::move(source));
  root.insert("sources", std::move(sources));

  // transforms: normalize (remap) + route
  toml::table normalize;
  normalize.insert("type", "remap");
  normalize.insert("inputs", toml::array{ SOURCE_ID });
  normalize.insert("source", render_normalize_source(config));

  std::set<std::string> all_tags;
  std::set<std::string> all_tag_prefixes;
  for (const auto& dest : config.destinations)
  {
    for (const auto& tag : destination_tags(dest))
    {
      all_tags.insert(tag);
    }
    for (const auto& prefix : destination_tag_prefixes(dest))
    {
      all_tag_prefixes.insert(prefix);
    }
  }
  toml::table route_branches;
  for (const auto& tag : all_tags)
  {
    toml::table branch;
    branch.insert("type", "vrl");
    branch.insert("source", ".tag == " + debug_quote(tag));
    route_branches.insert(tag, std::move(branch));
  }
  for (const auto& prefix : all_tag_prefixes)
  {
    toml::table branch;
    branch.insert("type", "vrl");
    branch.insert("source", tag_prefix_predicate(prefix));
    route_branches.insert(route_branch_for_tag_prefix(prefix), std::move(branch));
  }
  toml::table route;
  route.insert("type", "route");
  route.insert("inputs", toml::array{ NORMALIZE_TRANSFORM_ID });
  route.insert("reroute_unmatched", false);
  route.insert("route", std::move(route_branches));

  toml::table transforms;
  transforms.insert(NORMALIZE_TRANSFORM_ID, std::move(normalize));
  transforms.insert(ROUTE_TRANSFORM_ID, std::move(route));
  root.insert("transforms", std::move(transforms));

  // sinks
  toml::table sinks;
  for (const auto& dest : config.destinations)
  {
    sinks.insert(dest.name, render_sink(config, dest));
  }
  root.insert("sinks", std::move(sinks));

  std::stringstream ss;
  ss << root;
  return ss.str();
}

}  // namespace dc_bridge
