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
// The in-process wake-up channel (a condition_variable in BridgeNode) is still how the
// worker learns "there's new work" cheaply; this directory is the crash-recovery state,
// not the communication path — on startup every unacked intent left over from a
// previous run is replayed, oldest-first, alongside live traffic.
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

  /// Number of intents currently pending on disk (ready or backing off).
  std::size_t size() const;
  bool empty() const;

private:
  struct Entry
  {
    std::string tag;
    nlohmann::json payload;
    // steady_clock::time_point::min() = ready immediately (never failed yet).
    std::chrono::steady_clock::time_point next_attempt{ std::chrono::steady_clock::time_point::min() };
    std::chrono::milliseconds backoff{ 0 };
  };

  std::string path_for(const std::string& id) const;

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
