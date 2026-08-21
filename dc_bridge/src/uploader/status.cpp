// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/uploader/status.hpp"

#include <chrono>
#include <set>

namespace dc_bridge::uploader::status
{

namespace
{

// Fields the rows compute themselves. A custom key naming one of these means the
// Measurement and the Uploader disagree about what the name stands for.
const std::set<std::string>& computed_fields()
{
  static const std::set<std::string> fields{
    "kind",       "group_name", "robot_id", "local_path",    "remote_path",    "storage_type",
    "updated_at", "uploaded",   "deleted",  "on_filesystem", "duration",       "content_type",
    "size",       "complete",   "files",    "file_count",    "thumbnail_path",
  };
  return fields;
}

// Record keys the rows already carry, verbatim or under a Humble-era name of their own
// (`name` → group_name, `id` → robot_id). Also dropped, but not a collision: the value is
// in the row already, so re-emitting it would only put it in a second column.
const std::set<std::string>& payload_derived_fields()
{
  static const std::set<std::string> keys{ "name", "id", "robot_name" };
  return keys;
}

// Appends the emitting Measurement's custom keys (#419). Called last, so a key that names
// a field the row already carries is dropped instead of overwriting it.
void apply_custom_keys(nlohmann::json& row, const FileGroup& group)
{
  for (const auto& [key, value] : group.custom_keys)
  {
    if (!computed_fields().count(key) && !payload_derived_fields().count(key))
    {
      row[key] = value;
    }
  }
}

double unix_now()
{
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

// The shared file_status fields (Humble out_files_metrics column set).
nlohmann::json base_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                        const std::string& remote_path)
{
  nlohmann::json row;
  row["kind"] = "file_status";
  row["group_name"] = group.group_name;
  if (group.robot_name)
  {
    row["robot_name"] = *group.robot_name;
  }
  if (group.robot_id)
  {
    row["robot_id"] = *group.robot_id;
  }
  row["local_path"] = file.local_path;
  row["remote_path"] = storage.url_prefix + storage.object_key(remote_path);
  row["storage_type"] = storage.name;
  row["updated_at"] = unix_now();
  return row;
}

}  // namespace

nlohmann::json uploaded_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                            const std::string& remote_path, const FileMeta& meta,
                            const std::optional<std::string>& thumbnail_path)
{
  nlohmann::json row = base_row(group, file, storage, remote_path);
  row["uploaded"] = true;
  row["on_filesystem"] = true;
  row["deleted"] = false;
  row["content_type"] = meta.content_type;
  row["size"] = meta.size;
  if (meta.duration)
  {
    row["duration"] = *meta.duration;
  }
  if (thumbnail_path)
  {
    // Same shape as `remote_path` above (url_prefix + key), so both columns are read the
    // same way by a consumer.
    row["thumbnail_path"] = storage.url_prefix + storage.object_key(*thumbnail_path);
  }
  apply_custom_keys(row, group);
  return row;
}

nlohmann::json missing_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                           const std::string& remote_path)
{
  nlohmann::json row = base_row(group, file, storage, remote_path);
  row["uploaded"] = false;
  row["on_filesystem"] = false;
  row["deleted"] = false;
  apply_custom_keys(row, group);
  return row;
}

nlohmann::json deleted_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                           const std::string& remote_path)
{
  nlohmann::json row = base_row(group, file, storage, remote_path);
  row["uploaded"] = true;
  row["on_filesystem"] = false;
  row["deleted"] = true;
  apply_custom_keys(row, group);
  return row;
}

nlohmann::json shed_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                        const std::string& remote_path)
{
  nlohmann::json row = base_row(group, file, storage, remote_path);
  row["uploaded"] = false;
  row["on_filesystem"] = false;
  row["deleted"] = true;
  apply_custom_keys(row, group);
  return row;
}

nlohmann::json group_complete_row(const FileGroup& group)
{
  nlohmann::json row;
  row["kind"] = "group_complete";
  row["group_name"] = group.group_name;
  if (group.robot_name)
  {
    row["robot_name"] = *group.robot_name;
  }
  if (group.robot_id)
  {
    row["robot_id"] = *group.robot_id;
  }
  row["complete"] = true;
  row["file_count"] = group.files.size();
  nlohmann::json files = nlohmann::json::array();
  for (const auto& f : group.files)
  {
    files.push_back({ { "key", f.key }, { "local_path", f.local_path } });
  }
  row["files"] = std::move(files);
  row["updated_at"] = unix_now();
  apply_custom_keys(row, group);
  return row;
}

bool is_reserved_field(const std::string& name)
{
  return computed_fields().count(name) > 0;
}

}  // namespace dc_bridge::uploader::status
