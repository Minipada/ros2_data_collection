// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Exercises the Uploader process's environment-variable configuration (#446) against an
// in-memory map — no real process environment, no AWS SDK, no network.
#include "dc_bridge/uploader/process_config.hpp"

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>

using namespace dc_bridge::uploader;

namespace
{

EnvLookup from_map(const std::map<std::string, std::string>& vars)
{
  return [vars](const std::string& name) -> std::optional<std::string> {
    auto it = vars.find(name);
    if (it == vars.end())
    {
      return std::nullopt;
    }
    return it->second;
  };
}

std::map<std::string, std::string> minimal_valid_env()
{
  return {
    { "DC_UPLOADER_STORAGE_NAME", "rustfs" },
    { "DC_UPLOADER_QUEUE_DIR", "/data/queue/upload" },
    { "DC_UPLOADER_STATE_DIR", "/data/uploader" },
    { "DC_UPLOADER_S3_BUCKET", "dc-e2e" },
  };
}

}  // namespace

TEST(ProcessConfig, MinimalEnvironmentParsesWithDefaults)
{
  auto cfg = load_uploader_process_config(from_map(minimal_valid_env()));

  EXPECT_EQ(cfg.storage_name, "rustfs");
  EXPECT_EQ(cfg.queue_dir, "/data/queue/upload");
  EXPECT_EQ(cfg.uploader_config.state_dir, "/data/uploader");
  EXPECT_FALSE(cfg.files_dir.has_value());
  EXPECT_EQ(cfg.shipper_host, "127.0.0.1");
  EXPECT_EQ(cfg.shipper_port, 24224);
  EXPECT_EQ(cfg.storage_params.bucket, "dc-e2e");
  EXPECT_FALSE(cfg.storage_params.auth.has_value());
  EXPECT_FALSE(cfg.uploader_config.delete_when_sent);
  EXPECT_FALSE(cfg.uploader_config.thumbnails.enabled);
  EXPECT_EQ(cfg.ffprobe_binary, "ffprobe");
  EXPECT_EQ(cfg.ffmpeg_binary, "ffmpeg");
  EXPECT_FALSE(cfg.retention.enabled());
}

TEST(ProcessConfig, MissingRequiredVariableThrowsWithItsNameInTheMessage)
{
  auto vars = minimal_valid_env();
  vars.erase("DC_UPLOADER_S3_BUCKET");
  try
  {
    load_uploader_process_config(from_map(vars));
    FAIL() << "expected ProcessConfigError";
  }
  catch (const ProcessConfigError& e)
  {
    EXPECT_NE(std::string(e.what()).find("DC_UPLOADER_S3_BUCKET"), std::string::npos);
  }
}

TEST(ProcessConfig, FullEnvironmentOverridesEveryDefault)
{
  auto vars = minimal_valid_env();
  vars["DC_UPLOADER_FILES_DIR"] = "/data/files";
  vars["DC_UPLOADER_SHIPPER_HOST"] = "shipper.local";
  vars["DC_UPLOADER_SHIPPER_PORT"] = "9999";
  vars["DC_UPLOADER_S3_REGION"] = "us-east-1";
  vars["DC_UPLOADER_S3_ENDPOINT"] = "http://127.0.0.1:9000";
  vars["DC_UPLOADER_S3_KEY_PREFIX"] = "robot-01/";
  vars["DC_UPLOADER_S3_ACCESS_KEY_ID"] = "id";
  vars["DC_UPLOADER_S3_SECRET_ACCESS_KEY"] = "secret";
  vars["DC_UPLOADER_S3_FORCE_PATH_STYLE"] = "true";
  vars["DC_UPLOADER_DELETE_WHEN_SENT"] = "yes";
  vars["DC_UPLOADER_MULTIPART_THRESHOLD_BYTES"] = "1048576";
  vars["DC_UPLOADER_MULTIPART_PART_SIZE_BYTES"] = "524288";
  vars["DC_UPLOADER_FFPROBE_BINARY"] = "/usr/bin/ffprobe";
  vars["DC_UPLOADER_THUMBNAILS_ENABLED"] = "1";
  vars["DC_UPLOADER_FFMPEG_BINARY"] = "/usr/bin/ffmpeg";
  vars["DC_UPLOADER_THUMBNAIL_MAX_DIMENSION"] = "640";
  vars["DC_UPLOADER_RETENTION_MAX_BYTES"] = "100000";
  vars["DC_UPLOADER_RETENTION_MAX_AGE_DAYS"] = "7";

  auto cfg = load_uploader_process_config(from_map(vars));

  ASSERT_TRUE(cfg.files_dir.has_value());
  EXPECT_EQ(*cfg.files_dir, "/data/files");
  EXPECT_EQ(cfg.shipper_host, "shipper.local");
  EXPECT_EQ(cfg.shipper_port, 9999);
  ASSERT_TRUE(cfg.storage_params.region.has_value());
  EXPECT_EQ(*cfg.storage_params.region, "us-east-1");
  ASSERT_TRUE(cfg.storage_params.endpoint.has_value());
  EXPECT_EQ(*cfg.storage_params.endpoint, "http://127.0.0.1:9000");
  ASSERT_TRUE(cfg.storage_params.key_prefix.has_value());
  EXPECT_EQ(*cfg.storage_params.key_prefix, "robot-01/");
  ASSERT_TRUE(cfg.storage_params.auth.has_value());
  EXPECT_EQ(cfg.storage_params.auth->access_key_id, "id");
  EXPECT_EQ(cfg.storage_params.auth->secret_access_key, "secret");
  ASSERT_TRUE(cfg.storage_params.force_path_style.has_value());
  EXPECT_TRUE(*cfg.storage_params.force_path_style);
  EXPECT_TRUE(cfg.uploader_config.delete_when_sent);
  EXPECT_EQ(cfg.uploader_config.multipart_threshold_bytes, 1048576u);
  EXPECT_EQ(cfg.uploader_config.multipart_part_size_bytes, 524288u);
  EXPECT_EQ(cfg.ffprobe_binary, "/usr/bin/ffprobe");
  EXPECT_TRUE(cfg.uploader_config.thumbnails.enabled);
  EXPECT_EQ(cfg.ffmpeg_binary, "/usr/bin/ffmpeg");
  EXPECT_EQ(cfg.uploader_config.thumbnails.max_dimension, 640u);
  EXPECT_TRUE(cfg.retention.enabled());
  EXPECT_EQ(cfg.retention.max_bytes, 100000u);
  ASSERT_TRUE(cfg.retention.max_age_days.has_value());
  EXPECT_EQ(*cfg.retention.max_age_days, 7u);
}

TEST(ProcessConfig, AccessKeyWithoutSecretThrows)
{
  auto vars = minimal_valid_env();
  vars["DC_UPLOADER_S3_ACCESS_KEY_ID"] = "id-only";
  EXPECT_THROW(load_uploader_process_config(from_map(vars)), ProcessConfigError);
}

TEST(ProcessConfig, InvalidBooleanValueThrows)
{
  auto vars = minimal_valid_env();
  vars["DC_UPLOADER_DELETE_WHEN_SENT"] = "maybe";
  EXPECT_THROW(load_uploader_process_config(from_map(vars)), ProcessConfigError);
}

TEST(ProcessConfig, PortOutOfRangeThrows)
{
  auto vars = minimal_valid_env();
  vars["DC_UPLOADER_SHIPPER_PORT"] = "70000";
  EXPECT_THROW(load_uploader_process_config(from_map(vars)), ProcessConfigError);
}
