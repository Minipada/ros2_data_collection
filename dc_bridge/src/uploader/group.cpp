// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/uploader/group.hpp"

namespace dc_bridge::uploader
{

namespace
{

// A non-empty string value, else nullopt.
std::optional<std::string> str_field(const nlohmann::json& obj, const char* key)
{
  auto it = obj.find(key);
  if (it != obj.end() && it->is_string())
  {
    std::string s = it->get<std::string>();
    if (!s.empty())
    {
      return s;
    }
  }
  return std::nullopt;
}

void collect_files(const nlohmann::json& value, const std::set<std::string>& storages,
                   std::map<std::string, FileRef>& files)
{
  if (!value.is_object())
  {
    return;
  }

  auto local_paths_it = value.find("local_paths");
  if (local_paths_it != value.end() && local_paths_it->is_object())
  {
    auto remote_paths_it = value.find("remote_paths");
    for (auto it = local_paths_it->begin(); it != local_paths_it->end(); ++it)
    {
      const std::string& key = it.key();
      if (!it->is_string())
      {
        continue;
      }
      const std::string local_path = it->get<std::string>();
      if (local_path.empty())
      {
        continue;
      }
      std::map<std::string, std::string> remotes;
      if (remote_paths_it != value.end() && remote_paths_it->is_object())
      {
        for (const auto& storage : storages)
        {
          auto s_it = remote_paths_it->find(storage);
          if (s_it == remote_paths_it->end() || !s_it->is_object())
          {
            continue;
          }
          auto k_it = s_it->find(key);
          if (k_it != s_it->end() && k_it->is_string())
          {
            std::string remote = k_it->get<std::string>();
            if (!remote.empty())
            {
              remotes.emplace(storage, remote);
            }
          }
        }
      }
      if (remotes.empty())
      {
        continue;
      }
      // The same local path can appear in several sub-objects; merge their Destination
      // sets rather than uploading twice.
      auto existing = files.find(local_path);
      if (existing != files.end())
      {
        for (auto& [k, v] : remotes)
        {
          existing->second.remote_paths[k] = v;
        }
      }
      else
      {
        files.emplace(local_path, FileRef{ key, local_path, std::move(remotes) });
      }
    }
  }

  for (auto it = value.begin(); it != value.end(); ++it)
  {
    const std::string& key = it.key();
    // base64 holds inline file content (potentially huge) and never Files.
    if (key == "local_paths" || key == "remote_paths" || key == "base64")
    {
      continue;
    }
    collect_files(*it, storages, files);
  }
}

}  // namespace

FileGroup parse_file_group(const nlohmann::json& payload, const std::string& fallback_group,
                           const std::set<std::string>& storages)
{
  std::map<std::string, FileRef> files;
  collect_files(payload, storages, files);

  FileGroup group;
  group.group_name = payload.is_object() ? str_field(payload, "name").value_or(fallback_group) : fallback_group;
  if (payload.is_object())
  {
    group.robot_name = str_field(payload, "robot_name");
    auto id_it = payload.find("id");
    if (id_it != payload.end() && !id_it->is_null())
    {
      group.robot_id = *id_it;
    }
  }
  for (auto& [local_path, file] : files)
  {
    group.files.push_back(std::move(file));
  }
  return group;
}

}  // namespace dc_bridge::uploader
