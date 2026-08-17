// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Multipart, resumable uploads for large Files (ADR-0005): an interrupted transfer must
// resume, not restart. Progress (the multipart upload id + each completed part's ETag)
// is checkpointed to a JSON sidecar after every part, so a retry or a fresh process
// re-reads the sidecar, skips parts that already landed, and finishes the same upload.
// The sidecar is removed on completion.
#ifndef DC_BRIDGE__UPLOADER__MULTIPART_HPP_
#define DC_BRIDGE__UPLOADER__MULTIPART_HPP_

#include <cstdint>
#include <string>

#include "dc_bridge/uploader/object_store.hpp"

namespace dc_bridge::uploader::multipart
{

/// Where the resume sidecar for one (storage, object key) upload lives. Stable across
/// runs (FNV-1a, not a per-process-randomized hash).
std::string resume_state_path(const std::string& state_dir, const std::string& storage_name,
                              const std::string& object_key);

/// Uploads `local_path` to `key` on `store` in `part_size`-byte parts, checkpointing to
/// `state_path` so an interrupted upload resumes from the last completed part.
/// `file_size` must be the file's current size. Throws on failure.
void upload_resumable(ObjectStore& store, const std::string& key, const std::string& local_path,
                      std::uint64_t file_size, std::uint64_t part_size, const std::string& state_path);

}  // namespace dc_bridge::uploader::multipart

#endif  // DC_BRIDGE__UPLOADER__MULTIPART_HPP_
