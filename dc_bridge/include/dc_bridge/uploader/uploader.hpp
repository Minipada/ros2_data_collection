// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// The Uploader (ADR-0005): per File × storage, head+size verify-or-upload (an
// already-verified object short-circuits — idempotent retries), verified-only status
// Records, a group-completion marker only when every File in the group is verified on
// all its storages, then delete_when_sent deletion strictly after verification
// everywhere with `deleted: true` rows. Every emission is deduped by key, so
// retried/redelivered Records never duplicate rows. Synchronous (aws-sdk-cpp is
// blocking) — no async runtime.
#ifndef DC_BRIDGE__UPLOADER__UPLOADER_HPP_
#define DC_BRIDGE__UPLOADER__UPLOADER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "dc_bridge/uploader/file_status_tag.hpp"
#include "dc_bridge/uploader/group.hpp"
#include "dc_bridge/uploader/object_store.hpp"
#include "dc_bridge/uploader/status.hpp"
#include "dc_bridge/uploader/thumbnail.hpp"

namespace dc_bridge::uploader
{

struct UploaderConfig
{
  bool delete_when_sent = false;
  std::string state_dir;
  std::uint64_t multipart_threshold_bytes = 16ULL * 1024 * 1024;
  std::uint64_t multipart_part_size_bytes = 8ULL * 1024 * 1024;
  std::uint32_t max_attempts = 3;
  std::chrono::milliseconds retry_backoff{ 500 };
  /// Optional derived previews (#256); disabled by default.
  ThumbnailConfig thumbnails;

  UploaderConfig(std::string state_dir_, bool delete_when_sent_)
    : delete_when_sent(delete_when_sent_), state_dir(std::move(state_dir_))
  {
  }
};

struct ProcessSummary
{
  std::size_t files = 0;
  std::size_t verified = 0;
  std::size_t missing = 0;
  std::size_t deleted = 0;
  bool group_complete = false;
  /// Previews uploaded and previews that couldn't be produced (#256). Counted rather
  /// than thrown: a failed preview is not an upload failure, but an operator who turned
  /// the feature on and gets no previews still needs to see that from the Bridge's own
  /// log line rather than by diffing the object store.
  std::size_t thumbnails = 0;
  std::size_t thumbnails_failed = 0;
  /// Custom keys (#419) the rows could not carry because they name a field the Uploader
  /// emits itself. Reported so the collision is visible in the Bridge's log instead of
  /// being the silent drop this whole feature exists to remove.
  std::vector<std::string> dropped_custom_keys;
};

/// Thrown by process_record when a Record couldn't be fully processed. `Incomplete` is
/// retryable (the node re-feeds the same Record — safe, since processing is idempotent);
/// `Emit` means a status Record couldn't be forwarded.
class UploadError : public std::runtime_error
{
public:
  enum class Kind
  {
    Incomplete,
    Emit,
  };
  UploadError(Kind kind, const std::string& msg) : std::runtime_error(msg), kind_(kind)
  {
  }
  Kind kind() const noexcept
  {
    return kind_;
  }

private:
  Kind kind_;
};

/// Probes a media file's duration in seconds; nullopt if it can't be determined.
using DurationProber = std::function<std::optional<double>(const std::string& path)>;

/// A DurationProber that shells out to ffprobe (argument-vector, not `sh -c`).
DurationProber ffprobe_duration_prober(const std::string& ffprobe_binary);

/// Emits one status Record. Throws on failure (the implementation may retry internally).
using EmitFn = std::function<void(const nlohmann::json& row)>;

class Uploader
{
public:
  /// `thumbnail_generator` may be empty when `config.thumbnails.enabled` is false (the
  /// default) — nothing will ask it for a preview.
  Uploader(UploaderConfig config, std::vector<Storage> storages, DurationProber duration_prober,
           ThumbnailGenerator thumbnail_generator = {});

  /// Processes one Record payload: uploads+verifies every File it references on every
  /// configured storage, emits status/metadata Records via `emit`, and (if
  /// delete_when_sent) deletes verified local Files. Throws UploadError on incomplete
  /// processing so the caller can retry idempotently.
  ProcessSummary process_record(const nlohmann::json& payload, const std::string& fallback_group, const EmitFn& emit);

  /// True if every File `group` references is already uploaded and size-verified on
  /// every storage it targets — a plain HEAD/size check per storage, no upload attempt,
  /// no status Records emitted. Used by retention (#267) to recognize a pending intent
  /// whose only remaining step is its own deletion (delete_when_sent's job), which
  /// retention must never shed.
  bool is_verified_everywhere(const FileGroup& group) const;

private:
  enum class EnsureOutcome
  {
    Verified,
    VerifiedRemoteOnly,
    MissingLocal,
  };
  struct FileOutcome
  {
    bool verified_everywhere;
    bool missing;
    std::size_t thumbnails = 0;
    std::size_t thumbnails_failed = 0;
  };

  FileOutcome upload_and_verify_file(const FileGroup& group, const FileRef& file, const EmitFn& emit,
                                     std::vector<std::string>& failures);
  /// Derives, uploads and verifies `file`'s preview on `storage`, returning its remote
  /// path (before key_prefix) on success. Called only once the File itself is verified
  /// there. Never throws and never reports failure to the caller as anything but
  /// nullopt — a preview must not be able to fail the upload it decorates (#256).
  std::optional<std::string> ensure_thumbnail(const FileRef& file, const FileMeta& meta, const Storage& storage,
                                              const std::string& remote_path);
  bool delete_verified_file(const FileGroup& group, const FileRef& file, const EmitFn& emit,
                            std::vector<std::string>& failures);
  void emit_group_complete(const FileGroup& group, const EmitFn& emit);
  const Storage& storage(const std::string& name) const;
  std::optional<FileMeta> file_meta(const std::string& local_path) const;
  // Uploads if needed and verifies via a head+size check; throws on hard failure.
  EnsureOutcome ensure_uploaded(const Storage& storage, const FileRef& file, const std::optional<FileMeta>& meta,
                                const std::string& remote_path);
  void emit_once(const EmitFn& emit, const std::string& dedup_key, const nlohmann::json& row);

  UploaderConfig config_;
  std::vector<Storage> storages_;
  std::set<std::string> storage_names_;
  DurationProber duration_prober_;
  ThumbnailGenerator thumbnail_generator_;
  std::mutex emitted_mutex_;
  std::set<std::string> emitted_;
};

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__UPLOADER_HPP_
