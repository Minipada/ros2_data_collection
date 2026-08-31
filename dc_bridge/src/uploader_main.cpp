// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// dc_uploader process entry point (#446, docs/adr/0014-uploader-runs-as-its-own-process.md):
// reads the durable intent queue a dc_bridge process writes, uploads Files to object
// storage, and emits each File's metadata Record itself over the shipper ingest protocol
// — the same loop bridge_node.cpp used to run on its own worker thread, now a standalone
// process configured entirely by DC_UPLOADER_* environment variables
// (dc_bridge/uploader/process_config.hpp) with no ROS dependency. SIGINT/SIGTERM set an
// atomic flag the loop checks every poll, same shutdown contract rclcpp gives dc_bridge.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "dc_bridge/forwarder.hpp"
#include "dc_bridge/uploader/intent_queue.hpp"
#include "dc_bridge/uploader/object_store.hpp"
#include "dc_bridge/uploader/process_config.hpp"
#include "dc_bridge/uploader/retention.hpp"
#include "dc_bridge/uploader/uploader.hpp"

namespace
{

std::atomic<bool> g_stop{ false };

void on_signal(int)
{
  g_stop.store(true);
}

std::optional<std::string> real_getenv(const std::string& name)
{
  const char* v = ::getenv(name.c_str());
  if (v == nullptr)
  {
    return std::nullopt;
  }
  return std::string(v);
}

// Stamps a Bridge/Uploader-generated Record with the current time, split into the
// seconds/nanoseconds pair the ingest protocol's EventTime carries — mirrors
// bridge_node.cpp's stamp_now(), duplicated rather than shared since the two processes no
// longer share a translation unit.
void stamp_now(dc_bridge::Record& record)
{
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  record.timestamp_secs = static_cast<std::uint64_t>(secs.count());
  record.timestamp_nanos =
      static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs).count());
}

}  // namespace

int main()
{
  using namespace dc_bridge;            // NOLINT
  using namespace dc_bridge::uploader;  // NOLINT

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  try
  {
    const auto cfg = load_uploader_process_config(real_getenv);

    if (cfg.files_dir && !std::filesystem::is_directory(*cfg.files_dir))
    {
      throw std::runtime_error("DC_UPLOADER_FILES_DIR '" + *cfg.files_dir + "' does not exist or is not a directory");
    }

    init_aws_api();
    std::vector<Storage> storages{ make_s3_storage(cfg.storage_name, cfg.storage_params) };

    Uploader uploader(cfg.uploader_config, storages, ffprobe_duration_prober(cfg.ffprobe_binary),
                      ffmpeg_thumbnail_generator(cfg.ffmpeg_binary));
    IntentQueue intent_queue(cfg.queue_dir);

    ForwarderConfig fcfg;
    fcfg.host = cfg.shipper_host;
    fcfg.port = cfg.shipper_port;
    fcfg.on_warning = [](const std::string& msg) { std::cerr << "dc_uploader: " << msg << std::endl; };
    Forwarder forwarder(fcfg);

    auto emit = [&forwarder](const nlohmann::json& row) {
      Record record;
      record.tag = FILE_STATUS_TAG;
      stamp_now(record);
      record.payload = row;
      // Vector may be briefly down (restart, backpressure); keep trying for a while
      // before handing the whole Record back for an idempotent retry.
      std::string last_err;
      for (int i = 0; i < 120; ++i)
      {
        try
        {
          forwarder.send(record);
          return;
        }
        catch (const std::exception& e)
        {
          last_err = e.what();
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
      throw std::runtime_error(last_err);
    };
    // Retention's own audit rows are strictly best-effort: a single send, not the
    // 60s-of-retries loop `emit` above gets — the shed itself must never be held up by
    // "is the Shipper reachable right now".
    auto retention_emit = [&forwarder](const nlohmann::json& row) {
      Record record;
      record.tag = FILE_STATUS_TAG;
      stamp_now(record);
      record.payload = row;
      forwarder.send(record);
    };
    auto is_verified_everywhere = [&uploader](const FileGroup& group) { return uploader.is_verified_everywhere(group); };
    auto retention_warn = [](const std::string& msg) { std::cerr << "dc_uploader: " << msg << std::endl; };

    constexpr std::chrono::seconds RETENTION_SWEEP_INTERVAL{ 30 };
    auto last_retention_sweep = std::chrono::steady_clock::now() - RETENTION_SWEEP_INTERVAL;

    std::cout << "dc_uploader up: storage '" << cfg.storage_name << "', queue '" << cfg.queue_dir << "', shipper "
              << cfg.shipper_host << ":" << cfg.shipper_port << std::endl;

    while (!g_stop.load())
    {
      // Picks up any intent the Bridge process enqueued since the last poll (#446) — this
      // process's IntentQueue instance only learns about on-disk entries it didn't write
      // itself by rescanning.
      intent_queue.rescan();

      forwarder.poll();

      if (cfg.retention.enabled())
      {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_retention_sweep >= RETENTION_SWEEP_INTERVAL)
        {
          last_retention_sweep = now;
          sweep(intent_queue, storages, cfg.retention, is_verified_everywhere, retention_emit, retention_warn);
        }
      }

      auto item = intent_queue.next_ready();
      if (!item)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }

      try
      {
        const auto summary = uploader.process_record(item->payload, item->tag, emit);
        intent_queue.ack(item->id);
        if (summary.files > 0)
        {
          std::cout << "dc_uploader: group '" << item->tag << "': " << summary.files << " file(s), " << summary.verified
                    << " verified, " << summary.missing << " missing, " << summary.deleted
                    << " deleted, complete=" << summary.group_complete << std::endl;
        }
        if (!summary.dropped_custom_keys.empty())
        {
          std::string joined;
          for (const auto& key : summary.dropped_custom_keys)
          {
            joined += (joined.empty() ? "" : ", ") + key;
          }
          std::cerr << "dc_uploader: group '" << item->tag << "': custom key(s) " << joined
                    << " name fields the File metadata Records already carry — the Uploader's own values are kept"
                    << std::endl;
        }
        if (summary.thumbnails_failed > 0)
        {
          std::cerr << "dc_uploader: group '" << item->tag << "': " << summary.thumbnails << " thumbnail(s) generated, "
                    << summary.thumbnails_failed << " could not be generated (is '" << cfg.ffmpeg_binary
                    << "' on PATH?) — the Files themselves uploaded normally" << std::endl;
        }
      }
      catch (const std::exception& e)
      {
        // Retryable: the intent stays on disk (ack() never ran), so even an Uploader
        // crash/restart right after this replays it — processing is idempotent (#248).
        intent_queue.record_failure(item->id);
        std::cerr << "dc_uploader: " << e.what() << "; will retry intent " << item->id << " with backoff" << std::endl;
      }
    }

    std::cout << "dc_uploader shutting down" << std::endl;
    shutdown_aws_api();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "dc_uploader: fatal: " << e.what() << std::endl;
    return 1;
  }
}
