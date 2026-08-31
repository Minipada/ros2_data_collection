// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Environment variables read by the Uploader process (#446). Mirrors the `files.*` /
// `uploader.*` ROS parameters BridgeNode declared when it ran the Uploader in-process —
// see docs/adr/0014-uploader-runs-as-its-own-process.md for why the split happened and
// bridge_node.cpp for the Bridge-side half that remains (Files subscription, intent
// writing, metadata_destination Tag routing).
//
//   DC_UPLOADER_QUEUE_DIR            (required) durable upload intent queue directory.
//   DC_UPLOADER_STATE_DIR            (required) multipart-resume / thumbnail-scratch dir.
//   DC_UPLOADER_FILES_DIR            (optional) sanity-checked for existence only.
//   DC_UPLOADER_SHIPPER_HOST         (default "127.0.0.1")
//   DC_UPLOADER_SHIPPER_PORT         (default "24224")
//   DC_UPLOADER_STORAGE_NAME         (required) must match the Bridge's Destination name.
//   DC_UPLOADER_S3_BUCKET            (required)
//   DC_UPLOADER_S3_REGION            (optional)
//   DC_UPLOADER_S3_ENDPOINT          (optional)
//   DC_UPLOADER_S3_KEY_PREFIX        (optional)
//   DC_UPLOADER_S3_ACCESS_KEY_ID     (optional; must be set together with ...SECRET)
//   DC_UPLOADER_S3_SECRET_ACCESS_KEY (optional; must be set together with ...ACCESS_KEY_ID)
//   DC_UPLOADER_S3_FORCE_PATH_STYLE  (optional bool, default false)
//   DC_UPLOADER_DELETE_WHEN_SENT     (optional bool, default false)
//   DC_UPLOADER_MULTIPART_THRESHOLD_BYTES  (optional)
//   DC_UPLOADER_MULTIPART_PART_SIZE_BYTES  (optional)
//   DC_UPLOADER_FFPROBE_BINARY       (default "ffprobe")
//   DC_UPLOADER_THUMBNAILS_ENABLED   (optional bool, default false)
//   DC_UPLOADER_FFMPEG_BINARY        (default "ffmpeg")
//   DC_UPLOADER_THUMBNAIL_MAX_DIMENSION    (optional)
//   DC_UPLOADER_RETENTION_MAX_BYTES        (optional)
//   DC_UPLOADER_RETENTION_MAX_AGE_DAYS     (optional)
#include "dc_bridge/uploader/process_config.hpp"

#include <algorithm>
#include <cctype>

namespace dc_bridge::uploader
{

namespace
{

std::optional<std::string> lookup(const EnvLookup& env, const std::string& name)
{
  auto v = env(name);
  if (v && v->empty())
  {
    return std::nullopt;  // an explicitly-empty var is treated the same as unset.
  }
  return v;
}

std::string required_string(const EnvLookup& env, const std::string& name)
{
  auto v = lookup(env, name);
  if (!v)
  {
    throw ProcessConfigError("required environment variable " + name + " is not set");
  }
  return *v;
}

std::string string_or(const EnvLookup& env, const std::string& name, const std::string& fallback)
{
  return lookup(env, name).value_or(fallback);
}

std::int64_t parse_int64(const std::string& name, const std::string& raw)
{
  try
  {
    return std::stoll(raw);
  }
  catch (const std::exception&)
  {
    throw ProcessConfigError("environment variable " + name + " is not a valid integer: '" + raw + "'");
  }
}

std::optional<std::uint64_t> optional_uint64(const EnvLookup& env, const std::string& name)
{
  auto v = lookup(env, name);
  if (!v)
  {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(std::max<std::int64_t>(1, parse_int64(name, *v)));
}

std::optional<std::uint32_t> optional_uint32(const EnvLookup& env, const std::string& name)
{
  auto v = lookup(env, name);
  if (!v)
  {
    return std::nullopt;
  }
  return static_cast<std::uint32_t>(std::max<std::int64_t>(1, parse_int64(name, *v)));
}

bool parse_bool(const std::string& name, const std::string& raw)
{
  std::string lowered = raw;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on")
  {
    return true;
  }
  if (lowered == "0" || lowered == "false" || lowered == "no" || lowered == "off")
  {
    return false;
  }
  throw ProcessConfigError("environment variable " + name + " is not a valid boolean: '" + raw + "'");
}

bool bool_or(const EnvLookup& env, const std::string& name, bool fallback)
{
  auto v = lookup(env, name);
  if (!v)
  {
    return fallback;
  }
  return parse_bool(name, *v);
}

std::uint16_t parse_port(const std::string& name, const std::string& raw)
{
  const std::int64_t port = parse_int64(name, raw);
  if (port <= 0 || port > 65535)
  {
    throw ProcessConfigError("environment variable " + name + " is out of range (expected 1-65535): '" + raw + "'");
  }
  return static_cast<std::uint16_t>(port);
}

}  // namespace

UploaderProcessConfig load_uploader_process_config(const EnvLookup& env)
{
  UploaderProcessConfig cfg;

  cfg.storage_name = required_string(env, "DC_UPLOADER_STORAGE_NAME");
  cfg.queue_dir = required_string(env, "DC_UPLOADER_QUEUE_DIR");
  const std::string state_dir = required_string(env, "DC_UPLOADER_STATE_DIR");
  cfg.files_dir = lookup(env, "DC_UPLOADER_FILES_DIR");

  cfg.shipper_host = string_or(env, "DC_UPLOADER_SHIPPER_HOST", "127.0.0.1");
  cfg.shipper_port = parse_port("DC_UPLOADER_SHIPPER_PORT", string_or(env, "DC_UPLOADER_SHIPPER_PORT", "24224"));

  cfg.storage_params.bucket = required_string(env, "DC_UPLOADER_S3_BUCKET");
  cfg.storage_params.region = lookup(env, "DC_UPLOADER_S3_REGION");
  cfg.storage_params.endpoint = lookup(env, "DC_UPLOADER_S3_ENDPOINT");
  cfg.storage_params.key_prefix = lookup(env, "DC_UPLOADER_S3_KEY_PREFIX");
  cfg.storage_params.force_path_style = bool_or(env, "DC_UPLOADER_S3_FORCE_PATH_STYLE", false);

  auto access = lookup(env, "DC_UPLOADER_S3_ACCESS_KEY_ID");
  auto secret = lookup(env, "DC_UPLOADER_S3_SECRET_ACCESS_KEY");
  if (access && secret)
  {
    cfg.storage_params.auth = S3Auth{ *access, *secret };
  }
  else if (access || secret)
  {
    throw ProcessConfigError(
        "DC_UPLOADER_S3_ACCESS_KEY_ID and DC_UPLOADER_S3_SECRET_ACCESS_KEY must be set together (got only one)");
  }

  cfg.uploader_config = UploaderConfig(state_dir, bool_or(env, "DC_UPLOADER_DELETE_WHEN_SENT", false));
  if (auto v = optional_uint64(env, "DC_UPLOADER_MULTIPART_THRESHOLD_BYTES"))
  {
    cfg.uploader_config.multipart_threshold_bytes = *v;
  }
  if (auto v = optional_uint64(env, "DC_UPLOADER_MULTIPART_PART_SIZE_BYTES"))
  {
    cfg.uploader_config.multipart_part_size_bytes = *v;
  }
  cfg.uploader_config.thumbnails.enabled = bool_or(env, "DC_UPLOADER_THUMBNAILS_ENABLED", false);
  if (auto v = optional_uint32(env, "DC_UPLOADER_THUMBNAIL_MAX_DIMENSION"))
  {
    cfg.uploader_config.thumbnails.max_dimension = *v;
  }

  cfg.ffprobe_binary = string_or(env, "DC_UPLOADER_FFPROBE_BINARY", "ffprobe");
  cfg.ffmpeg_binary = string_or(env, "DC_UPLOADER_FFMPEG_BINARY", "ffmpeg");

  if (auto v = optional_uint64(env, "DC_UPLOADER_RETENTION_MAX_BYTES"))
  {
    cfg.retention.max_bytes = *v;
  }
  if (auto v = optional_uint32(env, "DC_UPLOADER_RETENTION_MAX_AGE_DAYS"))
  {
    cfg.retention.max_age_days = *v;
  }

  return cfg;
}

}  // namespace dc_bridge::uploader
