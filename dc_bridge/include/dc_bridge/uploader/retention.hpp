// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Files retention policy — bounded local storage for un-uploaded Files (#267, ADR-0005
// follow-up). #265's durable intent queue never abandons an intent by design (an intent
// and its Files live and die together), so a robot offline for weeks — or a store that's
// unreachable for weeks — accumulates un-uploaded Files on disk without bound, same as
// Humble did. This is the one, explicit, opt-in mechanism allowed to abandon data under
// disk pressure: default off (files.retention absent/zero = unlimited, Humble parity);
// when configured, it sheds the oldest un-uploaded intent+Files, together, until back
// under the configured limit(s).
#ifndef DC_BRIDGE__UPLOADER__RETENTION_HPP_
#define DC_BRIDGE__UPLOADER__RETENTION_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "dc_bridge/uploader/group.hpp"
#include "dc_bridge/uploader/intent_queue.hpp"
#include "dc_bridge/uploader/object_store.hpp"

namespace dc_bridge::uploader
{

struct RetentionConfig
{
  std::uint64_t max_bytes = 0;                ///< cap on the un-uploaded File pool. 0 = unlimited (default).
  std::optional<std::uint32_t> max_age_days;  ///< optional; whichever limit hits first. absent = unlimited.

  bool enabled() const
  {
    return max_bytes > 0 || max_age_days.has_value();
  }
};

/// True if every File `group` references is already fully verified (uploaded,
/// size-confirmed) on every storage it targets. A pending intent can only be in this
/// state if its own deletion is what's still failing/retrying (delete_when_sent) — that's
/// not retention's problem to solve, and retention must never shed it.
using VerifiedEverywhereFn = std::function<bool(const FileGroup& group)>;

/// Emits one shed_row audit row. Called at most once per (File, storage) a shed intent
/// referenced; a thrown exception is swallowed by sweep() (best-effort — the disk
/// pressure being relieved matters more than the audit Record reaching the Shipper this
/// instant) and never blocks or reverts the shed.
using RetentionEmitFn = std::function<void(const nlohmann::json& row)>;

struct SweepResult
{
  std::size_t shed_intents = 0;
  std::uint64_t bytes_freed = 0;
};

/// One retention sweep: stats the on-disk pool of Files referenced by `queue`'s pending
/// intents (every intent still in `<uploader.data_dir>/queue/upload/` — the Uploader hasn't
/// finished with it, so this excludes Records whose Files already uploaded, verified, and
/// left the queue via ack()) and, while either configured limit is exceeded, sheds the
/// oldest eligible intent's Files — File and intent removed together, never one without
/// the other — until back under both limits. An intent that `is_verified_everywhere`
/// reports as fully verified is skipped (left pending; delete_when_sent's own retry path
/// owns it) rather than shed, and evaluation continues with the next-oldest entry. A
/// no-op — returns {0, 0} immediately, touches nothing — when `config` isn't enabled
/// (files.retention absent, the default). `now` defaults to the real clock; tests inject
/// a synthetic value to exercise max_age_days without waiting real time.
SweepResult sweep(IntentQueue& queue, const std::vector<Storage>& storages, const RetentionConfig& config,
                  const VerifiedEverywhereFn& is_verified_everywhere, const RetentionEmitFn& emit,
                  const std::function<void(const std::string&)>& warn,
                  std::chrono::system_clock::time_point now = std::chrono::system_clock::now());

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__RETENTION_HPP_
