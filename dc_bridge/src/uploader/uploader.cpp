#include "dc_bridge/uploader/uploader.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

#include "dc_bridge/uploader/content_type.hpp"
#include "dc_bridge/uploader/multipart.hpp"

namespace dc_bridge::uploader
{

namespace
{
// Runs ffprobe (argument vector, not `sh -c`), returning trimmed stdout or "" on failure.
std::string run_ffprobe(const std::string& bin, const std::string& path)
{
  int pipefd[2];
  if (::pipe(pipefd) != 0)
  {
    return "";
  }
  pid_t pid = ::fork();
  if (pid < 0)
  {
    ::close(pipefd[0]);
    ::close(pipefd[1]);
    return "";
  }
  if (pid == 0)
  {
    ::dup2(pipefd[1], STDOUT_FILENO);
    ::close(pipefd[0]);
    ::close(pipefd[1]);
    ::execlp(bin.c_str(), bin.c_str(), "-i", path.c_str(), "-show_entries", "format=duration", "-v", "quiet", "-of",
             "csv=p=0", static_cast<char*>(nullptr));
    _exit(127);
  }
  ::close(pipefd[1]);
  std::string out;
  char buf[256];
  ssize_t n;
  while ((n = ::read(pipefd[0], buf, sizeof(buf))) > 0)
  {
    out.append(buf, static_cast<std::size_t>(n));
  }
  ::close(pipefd[0]);
  int status = 0;
  ::waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
  {
    return "";
  }
  // trim
  auto b = out.find_first_not_of(" \t\r\n");
  auto e = out.find_last_not_of(" \t\r\n");
  return b == std::string::npos ? "" : out.substr(b, e - b + 1);
}
}  // namespace

DurationProber ffprobe_duration_prober(const std::string& ffprobe_binary)
{
  return [ffprobe_binary](const std::string& path) -> std::optional<double> {
    const std::string s = run_ffprobe(ffprobe_binary, path);
    if (s.empty())
    {
      return std::nullopt;
    }
    try
    {
      return std::stod(s);
    }
    catch (...)
    {
      return std::nullopt;
    }
  };
}

Uploader::Uploader(UploaderConfig config, std::vector<Storage> storages, DurationProber duration_prober)
  : config_(std::move(config)), storages_(std::move(storages)), duration_prober_(std::move(duration_prober))
{
  std::filesystem::create_directories(config_.state_dir);
  for (const auto& s : storages_)
  {
    storage_names_.insert(s.name);
  }
}

const Storage& Uploader::storage(const std::string& name) const
{
  for (const auto& s : storages_)
  {
    if (s.name == name)
    {
      return s;
    }
  }
  throw std::logic_error("parse_file_group only keeps configured storages");
}

std::optional<FileMeta> Uploader::file_meta(const std::string& local_path) const
{
  std::error_code ec;
  auto size = std::filesystem::file_size(local_path, ec);
  if (ec)
  {
    return std::nullopt;
  }
  std::string head(512, '\0');
  std::ifstream f(local_path, std::ios::binary);
  std::streamsize n = 0;
  if (f.good())
  {
    f.read(head.data(), 512);
    n = f.gcount();
  }
  head.resize(static_cast<std::size_t>(n));

  FileMeta meta;
  meta.size = size;
  meta.content_type = content_type::sniff(head);
  if (content_type::is_video(meta.content_type))
  {
    meta.duration = duration_prober_(local_path);
  }
  return meta;
}

void Uploader::emit_once(const EmitFn& emit, const std::string& dedup_key, const nlohmann::json& row)
{
  {
    std::lock_guard<std::mutex> lock(emitted_mutex_);
    if (emitted_.count(dedup_key))
    {
      return;
    }
  }
  try
  {
    emit(row);
  }
  catch (const std::exception& e)
  {
    throw UploadError(UploadError::Kind::Emit, e.what());
  }
  std::lock_guard<std::mutex> lock(emitted_mutex_);
  emitted_.insert(dedup_key);
}

Uploader::EnsureOutcome Uploader::ensure_uploaded(const Storage& storage, const FileRef& file,
                                                  const std::optional<FileMeta>& meta, const std::string& remote_path)
{
  const std::string key = storage.object_key(remote_path);
  std::string last_err;

  for (std::uint32_t attempt = 0; attempt < config_.max_attempts; ++attempt)
  {
    if (attempt > 0)
    {
      std::this_thread::sleep_for(config_.retry_backoff);
    }

    std::optional<std::uint64_t> remote_size;
    try
    {
      remote_size = storage.store->head(key);
    }
    catch (const std::exception& e)
    {
      last_err = std::string("head failed: ") + e.what();
      continue;
    }

    if (remote_size && meta && *remote_size == meta->size)
    {
      return EnsureOutcome::Verified;
    }
    if (remote_size && !meta)
    {
      return EnsureOutcome::VerifiedRemoteOnly;
    }
    if (!meta)
    {
      return EnsureOutcome::MissingLocal;
    }

    // Upload (multipart for large Files, plain put otherwise), then head-verify.
    try
    {
      if (meta->size >= config_.multipart_threshold_bytes)
      {
        const std::string state_path = multipart::resume_state_path(config_.state_dir, storage.name, key);
        multipart::upload_resumable(*storage.store, key, file.local_path, meta->size, config_.multipart_part_size_bytes,
                                    state_path);
      }
      else
      {
        std::ifstream in(file.local_path, std::ios::binary);
        if (!in.good())
        {
          last_err = "failed to read local file";
          continue;
        }
        std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        storage.store->put(key, bytes);
      }
    }
    catch (const std::exception& e)
    {
      last_err = e.what();
      continue;
    }

    try
    {
      auto verified = storage.store->head(key);
      if (verified && *verified == meta->size)
      {
        return EnsureOutcome::Verified;
      }
      last_err = verified ? "verification failed: object has " + std::to_string(*verified) + " bytes, local file has " +
                                std::to_string(meta->size) :
                            "verification failed: object not found after upload";
    }
    catch (const std::exception& e)
    {
      last_err = std::string("verification failed: ") + e.what();
    }
  }
  throw std::runtime_error(last_err);
}

Uploader::FileOutcome Uploader::upload_and_verify_file(const FileGroup& group, const FileRef& file, const EmitFn& emit,
                                                       std::vector<std::string>& failures)
{
  const std::optional<FileMeta> meta = file_meta(file.local_path);
  FileOutcome outcome{ true, false };

  for (const auto& [storage_name, remote_path] : file.remote_paths)
  {
    const Storage& st = storage(storage_name);
    try
    {
      switch (ensure_uploaded(st, file, meta, remote_path))
      {
        case EnsureOutcome::Verified:
          emit_once(emit, "uploaded|" + file.local_path + "|" + st.name,
                    status::uploaded_row(group, file, st, remote_path, *meta));
          break;
        case EnsureOutcome::VerifiedRemoteOnly:
          break;
        case EnsureOutcome::MissingLocal:
          outcome.verified_everywhere = false;
          outcome.missing = true;
          emit_once(emit, "missing|" + file.local_path + "|" + st.name,
                    status::missing_row(group, file, st, remote_path));
          break;
      }
    }
    catch (const UploadError&)
    {
      throw;  // emit failures propagate straight up
    }
    catch (const std::exception& e)
    {
      outcome.verified_everywhere = false;
      failures.push_back("'" + file.local_path + "' -> " + st.name + ": " + e.what());
    }
  }
  return outcome;
}

void Uploader::emit_group_complete(const FileGroup& group, const EmitFn& emit)
{
  std::string files_key;
  for (const auto& f : group.files)
  {
    if (!files_key.empty())
    {
      files_key += "|";
    }
    files_key += f.local_path;
  }
  emit_once(emit, "group|" + group.group_name + "|" + files_key, status::group_complete_row(group));
}

bool Uploader::delete_verified_file(const FileGroup& group, const FileRef& file, const EmitFn& emit,
                                    std::vector<std::string>& failures)
{
  std::error_code ec;
  if (!std::filesystem::exists(file.local_path, ec))
  {
    return false;
  }
  if (!std::filesystem::remove(file.local_path, ec) || ec)
  {
    failures.push_back("failed to delete '" + file.local_path + "'");
    return false;
  }
  for (const auto& [storage_name, remote_path] : file.remote_paths)
  {
    const Storage& st = storage(storage_name);
    emit_once(emit, "deleted|" + file.local_path + "|" + st.name, status::deleted_row(group, file, st, remote_path));
  }
  return true;
}

ProcessSummary Uploader::process_record(const nlohmann::json& payload, const std::string& fallback_group,
                                        const EmitFn& emit)
{
  const FileGroup group = parse_file_group(payload, fallback_group, storage_names_);
  ProcessSummary summary;
  summary.files = group.files.size();
  if (group.files.empty())
  {
    return summary;
  }

  std::vector<std::string> failures;
  std::vector<const FileRef*> verified_files;

  for (const auto& file : group.files)
  {
    const FileOutcome outcome = upload_and_verify_file(group, file, emit, failures);
    if (outcome.missing)
    {
      summary.missing += 1;
    }
    if (outcome.verified_everywhere)
    {
      summary.verified += 1;
      verified_files.push_back(&file);
    }
  }

  if (summary.verified == summary.files)
  {
    summary.group_complete = true;
    emit_group_complete(group, emit);
  }

  if (config_.delete_when_sent)
  {
    for (const FileRef* file : verified_files)
    {
      if (delete_verified_file(group, *file, emit, failures))
      {
        summary.deleted += 1;
      }
    }
  }

  if (!failures.empty())
  {
    std::string joined;
    for (const auto& f : failures)
    {
      if (!joined.empty())
      {
        joined += "; ";
      }
      joined += f;
    }
    throw UploadError(UploadError::Kind::Incomplete, joined);
  }
  return summary;
}

}  // namespace dc_bridge::uploader
