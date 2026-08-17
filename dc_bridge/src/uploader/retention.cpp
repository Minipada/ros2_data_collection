// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/uploader/retention.hpp"

#include <algorithm>
#include <filesystem>
#include <set>
#include <system_error>

#include "dc_bridge/uploader/status.hpp"

namespace dc_bridge::uploader
{

namespace
{

std::uint64_t file_size_or_zero(const std::string& path)
{
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  return ec ? 0 : static_cast<std::uint64_t>(size);
}

struct Candidate
{
  std::string id;
  FileGroup group;
  std::uint64_t bytes;
  std::chrono::system_clock::time_point enqueued_at;
};

}  // namespace

SweepResult sweep(IntentQueue& queue, const std::vector<Storage>& storages, const RetentionConfig& config,
                  const VerifiedEverywhereFn& is_verified_everywhere, const RetentionEmitFn& emit,
                  const std::function<void(const std::string&)>& warn, std::chrono::system_clock::time_point now)
{
  SweepResult result;
  if (!config.enabled())
  {
    return result;
  }

  std::set<std::string> storage_names;
  for (const auto& s : storages)
  {
    storage_names.insert(s.name);
  }

  // Oldest-first, mirroring the queue's own on-disk order. Files-less Records (nothing
  // for retention to act on) are skipped entirely — they hold no bytes and never count
  // toward either limit.
  std::vector<Candidate> candidates;
  std::uint64_t pool_bytes = 0;
  for (const auto& intent : queue.pending())
  {
    FileGroup group = parse_file_group(intent.payload, intent.tag, storage_names);
    if (group.files.empty())
    {
      continue;
    }
    std::uint64_t bytes = 0;
    for (const auto& f : group.files)
    {
      bytes += file_size_or_zero(f.local_path);
    }
    pool_bytes += bytes;
    candidates.push_back(Candidate{ intent.id, std::move(group), bytes, intent.enqueued_at });
  }

  const auto max_age = config.max_age_days ?
                           std::optional<std::chrono::hours>(std::chrono::hours(24) * *config.max_age_days) :
                           std::nullopt;

  auto over_limits = [&](std::size_t idx) {
    if (idx >= candidates.size())
    {
      return false;
    }
    if (config.max_bytes > 0 && pool_bytes > config.max_bytes)
    {
      return true;
    }
    if (max_age && (now - candidates[idx].enqueued_at) > *max_age)
    {
      return true;
    }
    return false;
  };

  std::size_t i = 0;
  while (over_limits(i))
  {
    const Candidate& c = candidates[i];
    if (is_verified_everywhere(c.group))
    {
      // Fully verified but still pending — its own deletion is what's still
      // failing/retrying (delete_when_sent). Not retention's problem to solve; leave it
      // pending and evaluate the next-oldest entry instead.
      ++i;
      continue;
    }

    for (const auto& file : c.group.files)
    {
      std::error_code ec;
      std::filesystem::remove(file.local_path, ec);  // idempotent; already-gone is fine (replay tolerance).
      for (const auto& [storage_name, remote_path] : file.remote_paths)
      {
        auto st =
            std::find_if(storages.begin(), storages.end(), [&](const Storage& s) { return s.name == storage_name; });
        if (st == storages.end())
        {
          continue;
        }
        try
        {
          emit(status::shed_row(c.group, file, *st, remote_path));
        }
        catch (const std::exception&)
        {
          // Best-effort: the shed itself must not be held up by "the Shipper is
          // reachable" — the operator-visible warn below still fires regardless.
        }
      }
    }
    // The intent leaves the queue only after its Files are gone (never one without the
    // other) — matches #265's own File+intent lifecycle contract.
    queue.ack(c.id);

    result.shed_intents += 1;
    result.bytes_freed += c.bytes;
    pool_bytes -= c.bytes;
    candidates.erase(candidates.begin() + static_cast<std::ptrdiff_t>(i));
    // Don't advance i: the next-oldest candidate has shifted into this index.
  }

  if (result.shed_intents > 0)
  {
    warn("retention: shed " + std::to_string(result.shed_intents) + " intent(s), " + std::to_string(result.bytes_freed) +
         " byte(s) freed, to stay under the configured files.retention limit(s) — data was abandoned without "
         "uploading");
  }

  return result;
}

}  // namespace dc_bridge::uploader
