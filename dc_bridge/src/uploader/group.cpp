// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/uploader/group.hpp"

#include "dc_common/record_file_walk.hpp"

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

void collect_files(const nlohmann::json& record, const std::set<std::string>& storages,
                   std::map<std::string, FileRef>& files)
{
  // The walk — nested local_paths objects and a flattened Record's JSON-pointer keys alike —
  // is owned by dc_common's record_file_walk (#479), the same one the Measurement's staging
  // side goes through, so exactly the Files staged upstream are the ones collected here.
  dc_common::record_file_walk::for_each_file_site(record, [&](const dc_common::record_file_walk::FileSite& site) {
    std::map<std::string, std::string> remotes;
    for (const auto& storage : storages)
    {
      auto it = site.remote_paths.find(storage);
      if (it != site.remote_paths.end())
      {
        remotes.emplace(storage, it->second);
      }
    }
    if (remotes.empty())
    {
      return;
    }
    // The same local path can appear in several sub-objects; merge their Destination
    // sets rather than uploading twice.
    auto existing = files.find(site.local_path);
    if (existing != files.end())
    {
      for (auto& [k, v] : remotes)
      {
        existing->second.remote_paths[k] = v;
      }
    }
    else
    {
      files.emplace(site.local_path, FileRef{ site.key, site.local_path, std::move(remotes) });
    }
  });
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
    // Top level only: a Group-merged Record namespaces its members' fields under their
    // `group_key`, so those keys are no longer the Measurement's own labelling.
    auto declared_it = payload.find("custom_keys");
    if (declared_it != payload.end() && declared_it->is_array())
    {
      for (const auto& name : *declared_it)
      {
        if (!name.is_string())
        {
          continue;
        }
        const std::string key = name.get<std::string>();
        auto value_it = payload.find(key);
        if (key.empty() || value_it == payload.end())
        {
          continue;
        }
        group.custom_keys.emplace(key, *value_it);
      }
    }
  }
  for (auto& [local_path, file] : files)
  {
    group.files.push_back(std::move(file));
  }
  return group;
}

}  // namespace dc_bridge::uploader
