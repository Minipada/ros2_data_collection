// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Config renderer (ADR-0003): a pure, I/O-free mapping from Bridge/Destination
// parameters to a complete Vector configuration. `render()` produces Vector TOML with:
// the shipper ingest protocol source (global acknowledgements enabled — #266, delivery
// is confirmed end-to-end), one VRL transform normalizing each blessed destination's
// timestamp, one `route` transform exposing the public per-Tag `dc.<tag>` routes (the
// passthrough contract — see route_output_for_tag), disk-buffer settings, and the
// blessed `receives: records` sinks (file, vector). Everything else (postgres, s3,
// console, …) is configured via the `custom_config_files` passthrough (ADR-0003, #472).
//
// `s3` stays a blessed *type* for `receives: files` Destinations only — object storage
// there is served by dc_uploader's own S3 client (ADR-0005), never by a Vector sink, so
// S3Params/s3_from_raw remain even though render_sink() no longer templates an `aws_s3`
// Vector sink for it.
//
// Two layers, both pure: destination_from_raw (+ the per-type from_raw builders) turns
// the flat, stringly-typed values ROS parameters give into validated typed config (this
// is where invalid-parameter rejection lives); render() turns already-validated typed
// config into Vector TOML text.
//
// The launch path renders through this module too (#495): the ROS-free `dc_render` CLI
// (src/render_main.cpp) exposes route_output_for_topic, socket_sink_toml and stage_action
// to `ros2 launch`, which wires processes only and re-derives no config fact.
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

/// The public Shipper route the Records of one ROS topic are exposed under: the Tag the
/// Bridge itself derives for that topic (TopicConfig::derive_tag) under the route
/// transform's `dc.` prefix. One call for the whole ADR-0003 topic→route contract, so
/// code that emits passthrough sinks for topics (the launch-time `dc_render` CLI) hands
/// the render module the topic and re-derives neither half of it.
std::string route_output_for_topic(const std::string& topic);

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

/// Forwards to another Shipper (typically an edge aggregator) over Vector's own native
/// inter-instance protocol — the `vector` sink/source pair. `host`/`port` name that
/// Shipper's `vector` source; there is no sensible default for either (#443).
struct VectorParams
{
  std::string host;
  std::uint16_t port;
};

using DestinationKind = std::variant<S3Params, FileParams, VectorParams>;

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
  TooManyFilesDestinations,
  UnexpectedDestinationKind,
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
  // vector
  std::optional<std::string> host;
  std::optional<std::int64_t> port;
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

S3Params s3_from_raw(const std::string& name, const RawDestinationParams& raw);
FileParams file_from_raw(const std::string& name, const RawDestinationParams& raw);
VectorParams vector_from_raw(const std::string& name, const RawDestinationParams& raw);

/// Validates and builds one Destination from the flat parameters ROS declares for it.
Destination destination_from_raw(const std::string& name, const std::string& type_str, const std::string& receives_str,
                                 std::vector<std::string> inputs, const RawDestinationParams& raw);

/// At most one `receives: files` Destination per deployment (#505): the upload intent
/// queue is one store behind one dc_uploader process, and that process is configured with
/// exactly one object-storage endpoint (DC_UPLOADER_STORAGE_NAME). Enforced here, beside
/// destination_from_raw's per-Destination validation, rather than only in dc_bringup's
/// launch translation. Throws RenderError naming every offending Destination.
void validate_files_destinations(const std::vector<Destination>& files_destinations);

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

/// One passthrough *socket* sink (ADR-0009's `dc_mcap_writer` shape): a Vector `socket`
/// sink streaming newline-delimited JSON to an external consumer listening on host:port,
/// fed by the public `dc.<tag>` routes of `input_topics`. The one Vector sink DC renders
/// for a non-blessed Destination — everything else about such a Destination arrives as
/// hand-written passthrough TOML.
struct SocketSinkParams
{
  std::string sink_id;                    ///< Vector component id, e.g. `dc_mcap_writer`.
  std::vector<std::string> input_topics;  ///< ROS topics; each contributes its `dc.<tag>` route.
  std::string host;
  std::uint16_t port;
  std::string origin;  ///< provenance comment emitted above the table ("" = none).
};

/// Renders `params` as one self-contained passthrough snippet (a `[sinks.<sink_id>]`
/// table) for `custom_config_files`, with the same disk buffer every blessed sink gets —
/// Vector's floor, MIN_DISK_BUFFER_BYTES: a recorded stream must survive a slow or absent
/// listener the way a delivered one does. Routes are sorted+unique like every rendered
/// array. Pure. Throws RenderError on an unusable sink id, topic list or port.
std::string socket_sink_toml(const SocketSinkParams& params);

/// What the launch-time staging of one `custom_config_files` entry should do.
enum class StageAction
{
  Copy,  ///< stage the recipe over whatever the path holds — copy-forward
  Skip,  ///< stage nothing: leave the path alone, even when it holds nothing
};

/// The staging rule for passthrough recipes (ADR-0003): copy-forward. A corrected recipe
/// must win over an already-staged copy, or the first launch freezes whatever the recipe
/// said that day and a later fix never reaches an existing deployment (#495 — staging
/// used to copy only if the path was empty, so a stale hand-edited sink beat the recipe
/// forever). No recipe → Skip, whatever the path holds: an unresolvable entry is the
/// Bridge's own loud "failed to read custom config file" error, and a container bind
/// mount is left alone.
StageAction stage_action(bool recipe_exists, bool staged_exists);

/// Merges validated `custom_config_files` snippets into `rendered` (the already-rendered
/// config text) so the whole pipeline lives in one self-contained file. Vector natively
/// merges multiple `--config` files, but in unmanaged/split-deployment mode (#440/#444)
/// the Shipper is a separate process/container that only ever reads `shipper.config_path`
/// — a snippet's own filesystem path is never wired into that container, so a passthrough
/// sink would silently never run there unless it is folded into the one file everyone
/// reads. Call only after validate_custom_config_files() has confirmed no id collisions.
std::string merge_custom_config_files(const std::string& rendered, const std::vector<CustomConfigFile>& files);

/// Renders `config` into a complete Vector TOML configuration. Pure. Throws RenderError
/// on invalid config.
std::string render(const RenderConfig& config);

}  // namespace dc_bridge

#endif  // DC_BRIDGE__RENDER_HPP_
