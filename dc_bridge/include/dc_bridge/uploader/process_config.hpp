// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// The Uploader process's configuration (#446, docs/adr/0014-uploader-runs-as-its-own-
// process.md): everything BridgeNode used to read from ROS parameters when it ran the
// Uploader in-process, now read from DC_UPLOADER_* environment variables instead, so the
// standalone process has no ROS dependency. See process_config.cpp for the full list of
// variables, their defaults, and which are required.
#ifndef DC_BRIDGE__UPLOADER__PROCESS_CONFIG_HPP_
#define DC_BRIDGE__UPLOADER__PROCESS_CONFIG_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

#include "dc_bridge/render.hpp"  // S3Params
#include "dc_bridge/uploader/retention.hpp"
#include "dc_bridge/uploader/uploader.hpp"  // UploaderConfig

namespace dc_bridge::uploader
{

/// Looks up an environment variable by name; nullopt if unset. A plain std::function
/// rather than calling ::getenv directly so tests can inject an in-memory map instead of
/// the real process environment.
using EnvLookup = std::function<std::optional<std::string>(const std::string&)>;

/// Thrown by load_uploader_process_config when a required variable is missing, or a
/// present one can't be parsed as the type it names.
class ProcessConfigError : public std::runtime_error
{
public:
  explicit ProcessConfigError(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

/// Everything the Uploader process needs to run, parsed from environment variables.
/// Building one does no I/O — no filesystem access, no network, no AWS SDK calls;
/// load_uploader_process_config only parses and validates strings.
struct UploaderProcessConfig
{
  /// The `receives: files` Destination name this process uploads for — must match the
  /// name the Bridge's own params file gave that Destination, since it's the key
  /// FileRef::remote_paths (and therefore the status Records' `storage_type`) is built
  /// under (see dc_bridge/uploader/group.hpp).
  std::string storage_name;
  S3Params storage_params;

  /// The Uploader core's own config: delete_when_sent, multipart sizing, thumbnails.
  /// state_dir is the multipart-resume / thumbnail-scratch directory
  /// (DC_UPLOADER_STATE_DIR). Default-constructed here (UploaderConfig has no default
  /// constructor of its own) and always overwritten by load_uploader_process_config.
  UploaderConfig uploader_config{ std::string(), false };

  /// The durable upload intent queue directory (DC_UPLOADER_QUEUE_DIR) — the Bridge
  /// writes intents here; this process reads, acks, and replays them.
  std::string queue_dir;

  /// Sanity-checked for existence at process startup only (DC_UPLOADER_FILES_DIR):
  /// catches a missing/unmounted shared volume with a clear startup error rather than a
  /// confusing failure on the first upload. Every File is otherwise referenced by the
  /// absolute `local_path` its Record already carries — nothing here is joined onto it.
  std::optional<std::string> files_dir;

  /// The shipper ingest protocol endpoint this process forwards File status/metadata
  /// Records to (DC_UPLOADER_SHIPPER_HOST/DC_UPLOADER_SHIPPER_PORT) — the same Shipper
  /// the Bridge forwards Records to.
  std::string shipper_host;
  std::uint16_t shipper_port;

  RetentionConfig retention;

  std::string ffprobe_binary;
  std::string ffmpeg_binary;
};

/// Parses and validates the Uploader process's environment-variable configuration.
/// Throws ProcessConfigError on a missing required variable or an unparsable value.
UploaderProcessConfig load_uploader_process_config(const EnvLookup& env);

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__PROCESS_CONFIG_HPP_
