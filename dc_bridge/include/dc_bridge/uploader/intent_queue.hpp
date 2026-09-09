// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// The durable upload intent queue (ADR-0005/#265, ADR-0014): one intent file on disk per
// Record a files-Destination receives, removed only by ack() after processing succeeds —
// no cap, no drop-oldest, no dead-letter. Two classes over one store: IntentQueueWriter
// is the Bridge's write half, IntentQueue is dc_uploader's read half — the queue *is* the
// interface, and each process now holds only its own half of it.
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

/// `<uploader.data_dir>/queue/upload` — the one C++ derivation of the queue path
/// (ADR-0014: both processes derive it from the same `uploader.data_dir`; dc_bringup hands
/// the result to dc_uploader as DC_UPLOADER_QUEUE_DIR).
std::string intent_queue_dir(const std::string& uploader_data_dir);

/// The queue's write half — what the Bridge sees. Append an intent, count the backlog;
/// nothing else. replay/backoff/ack/rescan are the reader's half, so they aren't on this
/// type and writer-side code cannot call them.
class IntentWriter
{
public:
  virtual ~IntentWriter() = default;

  /// Writes one intent — `{version: 1, tag, timestamp, payload}` — to disk via tmp+write
  /// +rename (crash-atomic; no fsync, matching FLB's `storage.sync normal`) and returns
  /// its id. Durable before this returns, and visible to a reader's rescan() — there is
  /// no in-process wake-up across the process boundary (#446).
  virtual std::string enqueue(const std::string& tag, const nlohmann::json& payload) = 0;

  /// Number of intents currently in the queue.
  virtual std::size_t size() const = 0;
};

/// A writer over the store holding none of the reader's in-memory state: the constructor
/// scans nothing and keeps nothing but `dir` (the Bridge used to construct a full queue
/// and load every pending intent's payload it would never read back). `size()` therefore
/// counts the `*.json` files on disk — the only depth a writer-side caller can observe
/// truthfully anyway, since acks happen in the reader's address space.
class IntentQueueWriter final : public IntentWriter
{
public:
  /// `dir` is created if missing; anything already in it is left for the reader.
  explicit IntentQueueWriter(std::string dir);

  std::string enqueue(const std::string& tag, const nlohmann::json& payload) override;

  std::size_t size() const override;

private:
  std::string dir_;
  std::mutex mutex_;  ///< guards seq_ (enqueue may run on several subscription threads).
  std::uint64_t seq_{ 0 };
};

/// The queue's read half (dc_uploader's), plus the full queue for a single-process
/// deployment: oldest-first scheduling with per-entry exponential backoff, so one
/// permanently-failing intent can't starve the backlog behind it, and replay of every
/// unacked intent a previous run left behind. Thread-safe: enqueue() is expected to run
/// on the subscription callback's thread while next_ready()/ack()/record_failure() run on
/// the uploader worker thread.
class IntentQueue final : public IntentWriter
{
public:
  /// `dir` is created if missing. Every `*.json` file already there — a previous run's
  /// unacked intents — is loaded immediately, oldest-first by filename, and made
  /// eligible for next_ready() right away. A `.tmp` file (a crash mid tmp+rename) is
  /// left untouched and simply not loaded as a pending intent.
  explicit IntentQueue(std::string dir, std::chrono::milliseconds base_backoff = DEFAULT_BASE_BACKOFF,
                       std::chrono::milliseconds max_backoff = DEFAULT_MAX_BACKOFF);

  std::string enqueue(const std::string& tag, const nlohmann::json& payload) override;

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
  std::size_t size() const override;
  bool empty() const;

  /// Picks up any `*.json` file that exists on disk but isn't yet known to this
  /// instance — the multi-process split (#446): a Bridge process enqueues intents while a
  /// separate Uploader process holds the read side (next_ready()/ack()/record_failure()),
  /// each with its own instance over the same directory. enqueue() only updates its own
  /// caller's in-memory view, so the Uploader's instance never otherwise learns about an
  /// intent a different process wrote; this is what closes that gap. Already-known
  /// entries are left untouched (so in-flight backoff state survives); returns the number
  /// of newly discovered intents. Safe to call from the same loop that already polls for
  /// next_ready(), same cost as the constructor's initial scan.
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
