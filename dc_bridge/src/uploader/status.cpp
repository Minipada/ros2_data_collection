#include "dc_bridge/uploader/status.hpp"

#include <chrono>

namespace dc_bridge::uploader::status
{

namespace
{

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
                            const std::string& remote_path, const FileMeta& meta)
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
  return row;
}

nlohmann::json missing_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                           const std::string& remote_path)
{
  nlohmann::json row = base_row(group, file, storage, remote_path);
  row["uploaded"] = false;
  row["on_filesystem"] = false;
  row["deleted"] = false;
  return row;
}

nlohmann::json deleted_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                           const std::string& remote_path)
{
  nlohmann::json row = base_row(group, file, storage, remote_path);
  row["uploaded"] = true;
  row["on_filesystem"] = false;
  row["deleted"] = true;
  return row;
}

nlohmann::json shed_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                        const std::string& remote_path)
{
  nlohmann::json row = base_row(group, file, storage, remote_path);
  row["uploaded"] = false;
  row["on_filesystem"] = false;
  row["deleted"] = true;
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
  return row;
}

}  // namespace dc_bridge::uploader::status
