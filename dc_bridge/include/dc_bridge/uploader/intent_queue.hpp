// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// A disk-backed durable intent queue for the Uploader (ADR-0005 follow-up, #265):
// Humble's embedded Fluent Bit made ingestion and durable buffering atomic (`in_ros2`
// wrote straight into FLB's own filesystem chunks; chunk retry was the durable upload
// state). The Bridge/Shipper split broke that for the File pipeline — an in-memory
// queue forgets every pending upload on a Bridge restart, silently orphaning Files that
// will never upload, report, or clean up. This restores that durability: every Record a
// files-Destination's subscription receives is written to disk as one intent file before
// it's ever handed to the Uploader, and it leaves the queue only via ack() after
// processing actually succeeds — no cap, no drop-oldest, no dead-letter. An upload
// intent and its Files live and die together.
//
// This directory is the crash-recovery state, not a notification channel: on startup
// every unacked intent left over from a previous run is replayed, oldest-first, alongside
// live traffic. Since #446 split the Uploader into its own process, there is no
// in-process wake-up signal at all — a Bridge process enqueues and a separate dc_uploader
// process discovers new intents purely by polling rescan() (see that method below).
#ifndef DC_BRIDGE__UPLOADER__INTENT_QUEUE_HPP_
#define DC_BRIDGE__UPLOADER__INTENT_QUEUE_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace dc_bridge::uploader
{

/// One pending upload intent, whether freshly enqueued or replayed from a previous run.
struct Intent
{
  std::string id;  ///< the intent's filename — the key ack()/record_failure() take.
  std::string tag;
  nlohmann::json payload;
  /// Wall-clock time the intent was enqueued — survives a restart (parsed back from the
  /// filename's own timestamp prefix on replay). Retention (#267) uses this for its
  /// `max_age_days` limit.
  std::chrono::system_clock::time_point enqueued_at;
};

/// FLB's own scheduler defaults (`destination_server.cpp`'s pre-#265 Fluent Bit config):
/// 5s base, 2000s cap.
inline constexpr std::chrono::milliseconds DEFAULT_BASE_BACKOFF{ 5000 };
inline constexpr std::chrono::milliseconds DEFAULT_MAX_BACKOFF{ 2000000 };

/// A crash-atomic, disk-backed FIFO of upload intents with oldest-first scheduling and
/// per-entry exponential backoff, so one permanently-failing intent can't starve the
/// rest of the backlog behind it. Thread-safe: enqueue() is expected to run on the
/// subscription callback's thread while next_ready()/ack()/record_failure() run on the
/// uploader worker thread.
class IntentQueue
{
public:
  /// `dir` is created if missing. Every `*.json` file already there — a previous run's
  /// unacked intents — is loaded immediately, oldest-first by filename, and made
  /// eligible for next_ready() right away. A `.tmp` file (a crash mid tmp+rename) is
  /// left untouched and simply not loaded as a pending intent.
  explicit IntentQueue(std::string dir, std::chrono::milliseconds base_backoff = DEFAULT_BASE_BACKOFF,
                       std::chrono::milliseconds max_backoff = DEFAULT_MAX_BACKOFF);

  /// Writes one intent — `{version: 1, tag, timestamp, payload}` — to disk via tmp+write
  /// +rename (crash-atomic; no fsync, matching FLB's `storage.sync normal`) and makes it
  /// immediately eligible for next_ready(). Returns its id.
  std::string enqueue(const std::string& tag, const nlohmann::json& payload);

  /// Unlinks the intent's file and drops it from scheduling. Idempotent: acking an id
  /// that's already gone (or was never known) is a no-op, never a thrown error.
  void ack(const std::string& id);

  /// Records a failed processing attempt for `id`: its backoff doubles (starting at
  /// base_backoff, capped at max_backoff) and its next-eligible time moves out from
  /// `now`. A no-op if `id` isn't currently pending (e.g. it was already acked).
  void record_failure(const std::string& id,
                      std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

  /// The oldest pending intent whose backoff has elapsed as of `now`, or nullopt if the
  /// queue is empty or every entry is still backing off. Does not remove it — ack() (on
  /// success) or record_failure() (on failure) does that once the caller knows which.
  std::optional<Intent> next_ready(std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) const;

  /// Every currently pending intent, oldest-first, regardless of backoff state. Unlike
  /// next_ready() (which only ever surfaces one entry not currently backing off, for the
  /// normal upload-retry path), retention (#267) cares about disk pressure, not retry
  /// timing — a backed-off intent's File still occupies space and still counts.
  std::vector<Intent> pending() const;

  /// Number of intents currently pending on disk (ready or backing off).
  std::size_t size() const;
  bool empty() const;

  /// Picks up any `*.json` file that exists on disk but isn't yet known to this
  /// instance — the multi-process split (#446): a Bridge process enqueues intents while a
  /// separate Uploader process holds the read side (next_ready()/ack()/record_failure()),
  /// each with its own IntentQueue instance over the same directory. enqueue() only
  /// updates its own caller's in-memory view, so the Uploader's instance never otherwise
  /// learns about an intent a different process wrote; this is what closes that gap.
  /// Already-known entries are left untouched (so in-flight backoff state survives);
  /// returns the number of newly discovered intents. Safe to call from the same loop that
  /// already polls for next_ready(), same cost as the constructor's initial scan.
  std::size_t rescan();

private:
  struct Entry
  {
    std::string tag;
    nlohmann::json payload;
    std::chrono::system_clock::time_point enqueued_at;
    // steady_clock::time_point::min() = ready immediately (never failed yet).
    std::chrono::steady_clock::time_point next_attempt{ std::chrono::steady_clock::time_point::min() };
    std::chrono::milliseconds backoff{ 0 };
  };

  std::string path_for(const std::string& id) const;
  /// Every "*.json" filename currently on disk under `dir`, sorted oldest-first (ids sort
  /// lexicographically in enqueue order — see intent_queue.cpp's make_id()).
  static std::vector<std::string> list_json_names(const std::string& dir);
  /// Parses one intent file's body into an Entry, or nullopt if it can't be read/parsed (a
  /// "final" .json file should always be complete, since rename is atomic).
  static std::optional<Entry> load_entry(const std::string& dir, const std::string& name);

  mutable std::mutex mutex_;
  std::string dir_;
  std::chrono::milliseconds base_backoff_;
  std::chrono::milliseconds max_backoff_;
  std::vector<std::string> order_;        // ids, oldest-first (mirrors on-disk sort order).
  std::map<std::string, Entry> entries_;  // id -> in-memory scheduling state + payload.
  std::uint64_t seq_{ 0 };
};

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__INTENT_QUEUE_HPP_
