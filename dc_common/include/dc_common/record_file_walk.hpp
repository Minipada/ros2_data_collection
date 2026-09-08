// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#pragma once
// The one owner of "where Files live in a Record". Measurements embed Files as nested
// `local_paths`/`remote_paths` objects (camera, map), and `flatten: true` rewrites those
// into JSON-pointer keys. The Measurement's staging side (dc_measurements) and the
// Bridge/Uploader's parse_file_group side (dc_bridge) both walk Records through this
// module — consumers must not re-derive the rules, or the two sides drift (#479).

#include <cstring>
#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

namespace dc_common::record_file_walk
{

/// One File location in a Record: its key within local_paths, its local path, and where
/// it must land per Destination (storage name -> remote path). Destinations the caller
/// is not configured for are filtered by the caller, not the walk.
struct FileSite
{
  std::string key;
  std::string local_path;
  std::map<std::string, std::string> remote_paths;
};

namespace detail
{

constexpr char kLocalPaths[] = "local_paths";
constexpr char kRemotePaths[] = "remote_paths";
constexpr char kBase64[] = "base64";

// Nested form: remote_paths = {storage: {key: remote}} beside the local_paths object.
inline void collect_remotes_nested(const nlohmann::json& remote_paths, const std::string& key, FileSite& site)
{
  if (!remote_paths.is_object())
  {
    return;
  }
  for (auto storage = remote_paths.begin(); storage != remote_paths.end(); ++storage)
  {
    if (!storage->is_object())
    {
      continue;
    }
    auto remote = storage->find(key);
    if (remote != storage->end() && remote->is_string() && !remote->get<std::string>().empty())
    {
      site.remote_paths.emplace(storage.key(), remote->get<std::string>());
    }
  }
}

// Flattened form: parent holds pointer keys; local sits at "<prefix>/local_paths/<key>"
// and remotes at "<prefix>/remote_paths/<storage>/<key>" (prefix is "" or "/<name>" when
// the Record is also nested).
inline void collect_remotes_flattened(const nlohmann::json& parent, const std::string& prefix, const std::string& key,
                                      FileSite& site)
{
  const std::string remote_marker = prefix + "/remote_paths/";
  for (auto it = parent.begin(); it != parent.end(); ++it)
  {
    if (!it->is_string() || it.key().rfind(remote_marker, 0) != 0)
    {
      continue;
    }
    const std::size_t key_start = remote_marker.size();
    const std::size_t key_end = it.key().rfind('/');
    if (key_end == std::string::npos || key_end < key_start ||
        it.key().compare(key_end + 1, std::string::npos, key) != 0)
    {
      continue;
    }
    const std::string storage = it.key().substr(key_start, key_end - key_start);
    if (!storage.empty() && !it->get<std::string>().empty())
    {
      site.remote_paths.emplace(storage, it->get<std::string>());
    }
  }
}

inline void walk(const nlohmann::json& value, const std::function<void(const FileSite&)>& visit)
{
  if (!value.is_object())
  {
    return;
  }

  auto local_paths = value.find(kLocalPaths);
  if (local_paths != value.end() && local_paths->is_object())
  {
    auto remote_paths = value.find(kRemotePaths);
    for (auto it = local_paths->begin(); it != local_paths->end(); ++it)
    {
      if (!it->is_string() || it->get<std::string>().empty())
      {
        continue;
      }
      FileSite site;
      site.key = it.key();
      site.local_path = it->get<std::string>();
      if (remote_paths != value.end())
      {
        collect_remotes_nested(*remote_paths, site.key, site);
      }
      visit(site);
    }
  }

  for (auto it = value.begin(); it != value.end(); ++it)
  {
    const std::string& key = it.key();
    if (key == kLocalPaths || key == kRemotePaths || key == kBase64)
    {
      continue;
    }
    // A flattened Record has no nested objects left: its keys are JSON pointers, so the
    // local paths show up as "/local_paths/raw" (or "/<name>/local_paths/raw" when also
    // nested). base64 is inline content, never a File.
    if (it->is_string() && key.find("/local_paths/") != std::string::npos)
    {
      if (it->get<std::string>().empty())
      {
        continue;
      }
      const std::size_t marker = key.find("/local_paths/");
      FileSite site;
      site.key = key.substr(marker + std::strlen("/local_paths/"));
      site.local_path = it->get<std::string>();
      collect_remotes_flattened(value, key.substr(0, marker), site.key, site);
      visit(site);
      continue;
    }
    walk(*it, visit);
  }
}

inline bool rewrite_in(nlohmann::json& value, const std::function<bool(std::string&)>& rewrite)
{
  if (!value.is_object())
  {
    return false;
  }

  bool any = false;
  auto local_paths = value.find(kLocalPaths);
  if (local_paths != value.end() && local_paths->is_object())
  {
    for (auto it = local_paths->begin(); it != local_paths->end(); ++it)
    {
      if (!it->is_string() || it->get_ref<std::string&>().empty())
      {
        continue;
      }
      any = rewrite(it->get_ref<std::string&>()) || any;
    }
  }

  for (auto it = value.begin(); it != value.end(); ++it)
  {
    const std::string& key = it.key();
    if (key == kLocalPaths || key == kRemotePaths || key == kBase64)
    {
      continue;
    }
    if (it->is_string() && key.find("/local_paths/") != std::string::npos && !it->get_ref<std::string&>().empty())
    {
      any = rewrite(it->get_ref<std::string&>()) || any;
      continue;
    }
    any = rewrite_in(*it, rewrite) || any;
  }
  return any;
}

}  // namespace detail

/// Visit every File site in a Record, in either written form: nested local_paths objects
/// at any depth, or flattened JSON-pointer keys. Empty local paths and base64 content are
/// skipped. remote_paths entries with empty values are dropped.
inline void for_each_file_site(const nlohmann::json& record, const std::function<void(const FileSite&)>& visit)
{
  detail::walk(record, visit);
}

/// Rewrite every local-path string in the Record in place — nested values and flattened
/// pointer-key values alike — by calling `rewrite(path)`, which may replace the string and
/// returns true when it did. Returns whether any path was rewritten. Remote paths and
/// base64 content are never handed to `rewrite`.
inline bool rewrite_local_paths(nlohmann::json& record, const std::function<bool(std::string&)>& rewrite)
{
  return detail::rewrite_in(record, rewrite);
}

}  // namespace dc_common::record_file_walk
