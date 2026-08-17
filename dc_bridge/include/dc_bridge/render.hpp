// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Config renderer (ADR-0003): a pure, I/O-free mapping from Bridge/Destination
// parameters to a complete Vector configuration. `render()` produces Vector TOML with:
// the shipper ingest protocol source (global acknowledgements enabled — #266, delivery
// is confirmed end-to-end), one VRL transform normalizing each blessed destination's
// timestamp (and, for a `postgres` destination, the envelope's `incident_id` — #291, so
// the incident an armed Measurement released a Record under is a queryable column rather
// than an opaque payload key), one `route` transform exposing the public per-Tag
// `dc.<tag>` routes (the
// passthrough contract — see route_output_for_tag), disk-buffer settings, and the
// blessed sinks (postgres, s3, file, console).
//
// Two layers, both pure: destination_from_raw (+ the per-type from_raw builders) turns
// the flat, stringly-typed values ROS parameters give into validated typed config (this
// is where invalid-parameter rejection lives); render() turns already-validated typed
// config into Vector TOML text.
#ifndef DC_BRIDGE__RENDER_HPP_
#define DC_BRIDGE__RENDER_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace dc_bridge
{

/// Fixed component ids / constants the rendered config owns (ADR-0003).
inline constexpr const char* SOURCE_ID = "dc_bridge_in";
inline constexpr const char* NORMALIZE_TRANSFORM_ID = "dc_bridge_normalize";
inline constexpr const char* ROUTE_TRANSFORM_ID = "dc";
/// Vector rejects a disk buffer's max_size below this (~256 MiB).
inline constexpr std::uint64_t MIN_DISK_BUFFER_BYTES = 268435488ULL;

/// The public Shipper route a Tag's Records are exposed under. Tag
/// `dc.measurement.uptime` → route `dc.dc.measurement.uptime` (leading `dc.` = the
/// route transform id, rest = the Tag verbatim).
std::string route_output_for_tag(const std::string& tag);

/// The public Shipper route a whole Tag *namespace* is exposed under (raw mode, #227):
/// prefix `dc.raw.` → route `dc.dc.raw`, whose branch matches on `starts_with(.tag, …)`
/// instead of an exact Tag. A rendered config is fixed at Bridge startup, but raw mode
/// discovers topics — and therefore mints Tags — while Vector is already running; a
/// prefix branch is what lets those later Tags reach a Destination without re-rendering
/// and restarting the Shipper.
std::string route_output_for_tag_prefix(const std::string& prefix);

/// How a Destination's normalized time field is written.
///
/// `EpochNanos` is the default: an exact integer count of nanoseconds since the epoch.
/// `Double` divides that by 1e9 into a float64, which has ~15-16 significant digits and
/// spends 10 of them on the seconds — so it cannot represent better than roughly
/// microseconds, and rounds. It stays available for consumers that want a fractional
/// seconds column, but it is lossy by construction, not by implementation (#308).
enum class TimeFormat
{
  EpochNanos,
  Double,
  Iso8601,
};

enum class Receives
{
  Records,
  Files,
};

struct PostgresParams
{
  std::string host;
  std::uint16_t port;
  std::string user;
  std::string password;
  std::string database;
  std::string table;
};

struct S3Auth
{
  std::string access_key_id;
  std::string secret_access_key;
};

/// S3-compatible object storage. With only bucket (+ usually region), credentials come
/// from the ambient AWS environment; a self-hosted store takes endpoint, explicit auth,
/// and (typically) force_path_style: true.
struct S3Params
{
  std::string bucket;
  std::optional<std::string> region;
  std::optional<std::string> endpoint;
  std::optional<std::string> key_prefix;
  std::optional<S3Auth> auth;
  std::optional<bool> force_path_style;
  std::optional<std::uint64_t> batch_timeout_secs;
};

struct FileParams
{
  std::string path;  ///< Vector template syntax passes through untouched.
};

struct ConsoleParams
{
};

using DestinationKind = std::variant<PostgresParams, S3Params, FileParams, ConsoleParams>;

struct Destination
{
  std::string name;
  Receives receives;
  /// ROS topic names feeding this Destination; the Tag each is routed under is derived
  /// the same way TopicConfig derives it for subscriptions.
  std::vector<std::string> inputs;
  /// Extra Tags routed here that don't come from a topic (Bridge-internal producers —
  /// today just the Uploader's dc.files Tag).
  std::vector<std::string> extra_tags;
  /// Tag *namespaces* routed here, each matched with `starts_with` rather than equality
  /// (raw mode's `dc.raw.`, #227) — the one way a Destination can receive Tags that did
  /// not exist when the config was rendered.
  std::vector<std::string> tag_prefixes;
  std::string time_key;
  TimeFormat time_format;
  DestinationKind kind;
};

struct RenderConfig
{
  std::string forward_host;
  std::uint16_t forward_port;
  std::string data_dir;
  std::uint64_t buffer_max_bytes;
  std::vector<Destination> destinations;
};

/// Error categories a render() can fail with; kind() lets tests assert the specific
/// failure. Formatted message in what().
enum class RenderErrorKind
{
  EmptyDataDir,
  NoDestinations,
  InvalidDestinationName,
  ReservedDestinationName,
  DuplicateDestination,
  EmptyInputs,
  TagPrefixCollidesWithTag,
  UnsupportedType,
  InvalidReceives,
  FilesRequireObjectStorage,
  FilesDestinationInShipperConfig,
  InvalidTimeFormat,
  MissingField,
  InvalidPort,
  IncompleteS3Auth,
  InvalidBatchTimeout,
  BufferTooSmall,
  CustomConfigParse,
  CustomConfigEmpty,
  CustomConfigReservedCollision,
  CustomConfigDuplicate,
};

class RenderError : public std::runtime_error
{
public:
  RenderError(RenderErrorKind kind, const std::string& message, std::string arg0 = "", std::string arg1 = "",
              std::int64_t number = 0)
    : std::runtime_error(message), kind_(kind), arg0_(std::move(arg0)), arg1_(std::move(arg1)), number_(number)
  {
  }

  RenderErrorKind kind() const noexcept
  {
    return kind_;
  }
  /// Primary subject of the error — usually the destination name, or (for custom-config
  /// errors) the file path.
  const std::string& arg0() const noexcept
  {
    return arg0_;
  }
  /// Secondary detail — the offending type/format/receives string, a field name, or a
  /// second file path / component id.
  const std::string& arg1() const noexcept
  {
    return arg1_;
  }
  std::int64_t number() const noexcept
  {
    return number_;
  }

private:
  RenderErrorKind kind_;
  std::string arg0_;
  std::string arg1_;
  std::int64_t number_;
};

/// Raw, flat Destination parameters as ROS declares them (`<name>.host`, `<name>.bucket`,
/// …), spanning every blessed type. nullopt == "not declared"; empty strings are treated
/// the same as nullopt so a blank YAML value still gets a clear "missing field" error.
struct RawDestinationParams
{
  std::optional<std::string> time_key;
  std::optional<std::string> time_format;
  // postgres
  std::optional<std::string> host;
  std::optional<std::int64_t> port;
  std::optional<std::string> user;
  std::optional<std::string> password;
  std::optional<std::string> database;
  std::optional<std::string> table;
  // s3
  std::optional<std::string> bucket;
  std::optional<std::string> region;
  std::optional<std::string> endpoint;
  std::optional<std::string> key_prefix;
  std::optional<std::string> access_key_id;
  std::optional<std::string> secret_access_key;
  std::optional<bool> force_path_style;
  std::optional<std::int64_t> batch_timeout_secs;
  // file
  std::optional<std::string> path;
};

PostgresParams postgres_from_raw(const std::string& name, const RawDestinationParams& raw);
S3Params s3_from_raw(const std::string& name, const RawDestinationParams& raw);
FileParams file_from_raw(const std::string& name, const RawDestinationParams& raw);

/// Validates and builds one Destination from the flat parameters ROS declares for it.
Destination destination_from_raw(const std::string& name, const std::string& type_str, const std::string& receives_str,
                                 std::vector<std::string> inputs, const RawDestinationParams& raw);

/// Thrown by expand_env for an undefined variable reference.
class ExpandError : public std::runtime_error
{
public:
  ExpandError(const std::string& var, const std::string& input)
    : std::runtime_error("undefined environment variable '" + var + "' referenced in '" + input + "'"), var_(var)
  {
  }
  const std::string& var() const noexcept
  {
    return var_;
  }

private:
  std::string var_;
};

/// Expands `$NAME` and `${NAME}` references using `lookup` (injected rather than reading
/// the real environment, so this stays pure/testable). Throws ExpandError on an
/// undefined variable.
std::string expand_env(const std::string& input,
                       const std::function<std::optional<std::string>(const std::string&)>& lookup);

/// One raw Vector config snippet from custom_config_files: path is only for error
/// messages, content is the file's TOML text.
struct CustomConfigFile
{
  std::string path;
  std::string content;
};

/// Validates passthrough snippets (ADR-0003) against the rendered config: each must be
/// valid TOML, define ≥1 component, and claim no component id owned by the rendered
/// config or another snippet. Throws RenderError naming the offending file.
void validate_custom_config_files(const RenderConfig& config, const std::vector<CustomConfigFile>& files);

/// Renders `config` into a complete Vector TOML configuration. Pure. Throws RenderError
/// on invalid config.
std::string render(const RenderConfig& config);

}  // namespace dc_bridge

#endif  // DC_BRIDGE__RENDER_HPP_
