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

#include "dc_bridge/uploader/group.hpp"
#include "dc_bridge/uploader/object_store.hpp"
#include "dc_bridge/uploader/status.hpp"

namespace dc_bridge::uploader
{

/// The Tag the Uploader emits status/metadata Records under.
inline constexpr const char* FILE_STATUS_TAG = "dc.files";

struct UploaderConfig
{
  bool delete_when_sent = false;
  std::string state_dir;
  std::uint64_t multipart_threshold_bytes = 16ULL * 1024 * 1024;
  std::uint64_t multipart_part_size_bytes = 8ULL * 1024 * 1024;
  std::uint32_t max_attempts = 3;
  std::chrono::milliseconds retry_backoff{ 500 };

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
  Uploader(UploaderConfig config, std::vector<Storage> storages, DurationProber duration_prober);

  /// Processes one Record payload: uploads+verifies every File it references on every
  /// configured storage, emits status/metadata Records via `emit`, and (if
  /// delete_when_sent) deletes verified local Files. Throws UploadError on incomplete
  /// processing so the caller can retry idempotently.
  ProcessSummary process_record(const nlohmann::json& payload, const std::string& fallback_group, const EmitFn& emit);

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
  };

  FileOutcome upload_and_verify_file(const FileGroup& group, const FileRef& file, const EmitFn& emit,
                                     std::vector<std::string>& failures);
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
  std::mutex emitted_mutex_;
  std::set<std::string> emitted_;
};

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__UPLOADER_HPP_
