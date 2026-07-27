#include "dc_bridge/uploader/intent_queue.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>

namespace dc_bridge::uploader
{

namespace
{

// Wall-clock nanoseconds since epoch, zero-padded to 20 digits, plus a zero-padded
// per-process sequence counter: sorts lexicographically in enqueue order both within one
// run and after a restart (a fresh process starts with a later wall-clock time than
// anything a previous run left behind; equal-timestamp collisions across restarts are
// tolerable — the spec only requires *an* order, not a strict one, between those).
std::string make_id(std::uint64_t now_ns, std::uint64_t seq)
{
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%020llu-%06llu.json", static_cast<unsigned long long>(now_ns),
                static_cast<unsigned long long>(seq));
  return std::string(buf);
}

std::string iso8601_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto secs = std::chrono::system_clock::to_time_t(now);
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % std::chrono::seconds(1);
  std::tm tm{};
  ::gmtime_r(&secs, &tm);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.' << std::setfill('0') << std::setw(6) << us.count() << 'Z';
  return ss.str();
}

// The intent id's own leading field is a 20-digit zero-padded wall-clock-nanoseconds
// timestamp (make_id() above) — parsing it back on replay avoids a second, redundant
// timestamp representation to keep in sync with the ISO8601 string in the JSON body.
// Falls back to the epoch (treated as "infinitely old" by retention's max_age_days,
// which is the safer default for a name that doesn't match our own generated format).
std::chrono::system_clock::time_point enqueued_at_from_id(const std::string& id)
{
  if (id.size() < 20)
  {
    return std::chrono::system_clock::time_point{};
  }
  try
  {
    const std::uint64_t ns = std::stoull(id.substr(0, 20));
    return std::chrono::system_clock::time_point(std::chrono::nanoseconds(static_cast<std::int64_t>(ns)));
  }
  catch (const std::exception&)
  {
    return std::chrono::system_clock::time_point{};
  }
}

}  // namespace

IntentQueue::IntentQueue(std::string dir, std::chrono::milliseconds base_backoff, std::chrono::milliseconds max_backoff)
  : dir_(std::move(dir)), base_backoff_(base_backoff), max_backoff_(max_backoff)
{
  std::filesystem::create_directories(dir_);

  std::vector<std::string> names;
  for (const auto& e : std::filesystem::directory_iterator(dir_))
  {
    if (!e.is_regular_file())
    {
      continue;
    }
    // ".json.tmp" leftovers from a crash mid tmp+rename are simply not ".json" and are
    // left alone rather than loaded as pending intents.
    if (e.path().extension() != ".json")
    {
      continue;
    }
    names.push_back(e.path().filename().string());
  }
  std::sort(names.begin(), names.end());

  for (auto& name : names)
  {
    std::ifstream in(dir_ + "/" + name, std::ios::binary);
    if (!in.good())
    {
      continue;
    }
    nlohmann::json doc;
    try
    {
      in >> doc;
    }
    catch (const nlohmann::json::exception&)
    {
      continue;  // a "final" .json file should always be complete (rename is atomic).
    }
    Entry entry;
    entry.tag = doc.value("tag", "");
    entry.payload = doc.value("payload", nlohmann::json::object());
    entry.enqueued_at = enqueued_at_from_id(name);
    entries_.emplace(name, std::move(entry));
    order_.push_back(name);
  }
}

std::string IntentQueue::path_for(const std::string& id) const
{
  return dir_ + "/" + id;
}

std::string IntentQueue::enqueue(const std::string& tag, const nlohmann::json& payload)
{
  std::uint64_t now_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
  std::uint64_t seq;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    seq = seq_++;
  }
  const std::string id = make_id(now_ns, seq);

  nlohmann::json doc{ { "version", 1 }, { "tag", tag }, { "timestamp", iso8601_now() }, { "payload", payload } };
  const std::string final_path = path_for(id);
  const std::string tmp_path = final_path + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
    out << doc.dump();
  }
  // No fsync (FLB `storage.sync normal` parity) — the rename itself is what makes the
  // write crash-atomic; losing the last few milliseconds of writes on a hard power loss
  // is an accepted, pre-existing tradeoff this queue doesn't change.
  if (std::rename(tmp_path.c_str(), final_path.c_str()) != 0)
  {
    throw std::runtime_error("failed to rename intent queue entry into place: " + final_path);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  Entry entry;
  entry.tag = tag;
  entry.payload = payload;
  entry.enqueued_at =
      std::chrono::system_clock::time_point(std::chrono::nanoseconds(static_cast<std::int64_t>(now_ns)));
  entries_.emplace(id, std::move(entry));
  order_.push_back(id);
  return id;
}

void IntentQueue::ack(const std::string& id)
{
  std::error_code ec;
  std::filesystem::remove(path_for(id), ec);  // idempotent; an already-gone file is fine.

  std::lock_guard<std::mutex> lock(mutex_);
  entries_.erase(id);
  auto it = std::find(order_.begin(), order_.end(), id);
  if (it != order_.end())
  {
    order_.erase(it);
  }
}

void IntentQueue::record_failure(const std::string& id, std::chrono::steady_clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = entries_.find(id);
  if (it == entries_.end())
  {
    return;
  }
  Entry& e = it->second;
  e.backoff = (e.backoff.count() == 0) ? base_backoff_ : std::min(e.backoff * 2, max_backoff_);
  e.next_attempt = now + e.backoff;
}

std::optional<Intent> IntentQueue::next_ready(std::chrono::steady_clock::time_point now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Oldest-first sweep, skipping any entry still backing off: a single
  // permanently-failing intent enters backoff and is passed over so the rest of the
  // backlog keeps making progress, rather than blocking behind the head of the queue.
  for (const auto& id : order_)
  {
    const Entry& e = entries_.at(id);
    if (e.next_attempt <= now)
    {
      return Intent{ id, e.tag, e.payload, e.enqueued_at };
    }
  }
  return std::nullopt;
}

std::vector<Intent> IntentQueue::pending() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Intent> out;
  out.reserve(order_.size());
  for (const auto& id : order_)
  {
    const Entry& e = entries_.at(id);
    out.push_back(Intent{ id, e.tag, e.payload, e.enqueued_at });
  }
  return out;
}

std::size_t IntentQueue::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return order_.size();
}

bool IntentQueue::empty() const
{
  return size() == 0;
}

}  // namespace dc_bridge::uploader
