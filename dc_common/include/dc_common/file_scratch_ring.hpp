// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_COMMON_FILE_SCRATCH_RING_HPP_
#define DC_COMMON_FILE_SCRATCH_RING_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace dc_common
{

/**
 * @brief One staged File held by a FileScratchRing: its path inside the scratch directory plus
 * the timestamp it was staged with.
 */
struct ScratchedFile
{
  std::filesystem::path path;
  std::chrono::system_clock::time_point stamp;
};

/**
 * @class dc_common::FileScratchRing
 * @brief Disk-backed bounded time-window buffer of Files -- the FileScratchRing analogue of
 * RecordRingBuffer, with no ROS dependency.
 *
 * stage() copies a produced File into a private scratch directory (created on construction if
 * missing) under a collision-free name and records it with a timestamp; evict() deletes staged
 * copies older than the configured window, from disk and from window(). window() reports the
 * current contents as an ordered list of (path, timestamp), oldest first. Same push/evict/read
 * shape as RecordRingBuffer, and the same "evict is caller-driven, not push-driven" contract: the
 * window is relative to a caller-supplied "now", not to the last stage() call. Two ways out
 * besides evict(): release() hands the window on without deleting anything (the staged copies now
 * belong to whoever the caller passed them to), purge() deletes the lot regardless of age.
 *
 * Part of pre-event circular-buffer capture (#282) -- exercised purely against a tmp directory in
 * tests, same as RecordRingBuffer against an injected clock.
 */
class FileScratchRing
{
public:
  FileScratchRing(std::filesystem::path scratch_dir, std::chrono::system_clock::duration window)
    : scratch_dir_(std::move(scratch_dir)), window_(window)
  {
    std::filesystem::create_directories(scratch_dir_);
  }

  /**
   * @brief Copy `source_path` into the scratch directory under a unique staged name and record it
   * with `stamp`. Returns the staged path.
   *
   * Throws std::filesystem::filesystem_error if the copy fails (source missing, disk full, ...).
   */
  std::filesystem::path stage(const std::filesystem::path& source_path, std::chrono::system_clock::time_point stamp)
  {
    const auto epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
    const auto staged_name =
        std::to_string(epoch_ns) + "_" + std::to_string(next_id_++) + "_" + source_path.filename().string();
    const auto staged_path = scratch_dir_ / staged_name;

    std::filesystem::copy_file(source_path, staged_path, std::filesystem::copy_options::overwrite_existing);

    const auto pos = std::upper_bound(entries_.begin(), entries_.end(), stamp,
                                      [](const std::chrono::system_clock::time_point& s, const ScratchedFile& e) {
                                        return s < e.stamp;
                                      });
    entries_.insert(pos, ScratchedFile{ staged_path, stamp });
    return staged_path;
  }

  /**
   * @brief Delete every staged File older than the configured window relative to `now` (inclusive
   * boundary, same as RecordRingBuffer::evict()), from disk and from window().
   *
   * A staged File already missing from disk (e.g. removed out of band) is still dropped from the
   * window without throwing -- replay-tolerant, mirroring dc_bridge's retention sweep.
   */
  void evict(std::chrono::system_clock::time_point now)
  {
    while (!entries_.empty() && (now - entries_.front().stamp) > window_)
    {
      std::error_code ec;
      std::filesystem::remove(entries_.front().path, ec);
      entries_.pop_front();
    }
  }

  /// Current contents, oldest first. Does not evict -- call evict() first if that's wanted.
  std::vector<ScratchedFile> window() const
  {
    return std::vector<ScratchedFile>(entries_.begin(), entries_.end());
  }

  /**
   * @brief Hand the current window on: returns it oldest-first and stops tracking it, deleting
   * nothing from disk. The RecordRingBuffer::clear() analogue -- what the ring "forgetting" an
   * entry means for a File is "someone else owns it now", not "delete it".
   *
   * Called when an incident releases the buffered window (#290): from then on the staged copies
   * belong to whoever the released Records were handed to (the Bridge's Files pipeline -- upload,
   * retention sweep, delete_when_sent), so a later evict() must not delete a File out from under
   * an upload in flight.
   */
  std::vector<ScratchedFile> release()
  {
    std::vector<ScratchedFile> released(entries_.begin(), entries_.end());
    entries_.clear();
    return released;
  }

  /**
   * @brief Delete every staged File still tracked, from disk and from window(), whatever its age.
   *
   * The counterpart of release(): used when the Records referencing the staged copies are being
   * dropped rather than published (a lifecycle cleanup), so nothing will ever pick them up and
   * leaving them behind would just orphan them on disk. Tolerates a File already gone, same as
   * evict().
   */
  void purge()
  {
    for (const auto& entry : entries_)
    {
      std::error_code ec;
      std::filesystem::remove(entry.path, ec);
    }
    entries_.clear();
  }

  std::size_t size() const
  {
    return entries_.size();
  }

  bool empty() const
  {
    return entries_.empty();
  }

private:
  std::filesystem::path scratch_dir_;
  std::chrono::system_clock::duration window_;
  std::deque<ScratchedFile> entries_;
  std::uint64_t next_id_{ 0 };
};

}  // namespace dc_common

#endif  // DC_COMMON_FILE_SCRATCH_RING_HPP_
