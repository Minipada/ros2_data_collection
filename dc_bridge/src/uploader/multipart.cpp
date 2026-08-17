// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/uploader/multipart.hpp"

#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

namespace dc_bridge::uploader::multipart
{

namespace
{

std::size_t part_count(std::uint64_t file_size, std::uint64_t part_size)
{
  std::uint64_t n = (file_size + part_size - 1) / part_size;  // ceil
  return static_cast<std::size_t>(n < 1 ? 1 : n);
}

struct ResumeState
{
  std::string upload_id;
  std::uint64_t part_size;
  std::uint64_t file_size;
  std::vector<std::optional<std::string>> parts;  // ETag per part index; nullopt = pending.
};

// A sidecar from a different policy or a changed file can't be resumed safely.
std::optional<ResumeState> load_state(const std::string& state_path, std::uint64_t part_size, std::uint64_t file_size)
{
  std::ifstream in(state_path);
  if (!in.good())
  {
    return std::nullopt;
  }
  nlohmann::json j;
  try
  {
    in >> j;
  }
  catch (const nlohmann::json::exception&)
  {
    return std::nullopt;
  }
  ResumeState s;
  try
  {
    s.upload_id = j.at("upload_id").get<std::string>();
    s.part_size = j.at("part_size").get<std::uint64_t>();
    s.file_size = j.at("file_size").get<std::uint64_t>();
    for (const auto& p : j.at("parts"))
    {
      s.parts.push_back(p.is_null() ? std::nullopt : std::optional<std::string>(p.get<std::string>()));
    }
  }
  catch (const nlohmann::json::exception&)
  {
    return std::nullopt;
  }
  if (s.part_size != part_size || s.file_size != file_size || s.parts.size() != part_count(file_size, part_size))
  {
    return std::nullopt;
  }
  return s;
}

void save_state(const std::string& state_path, const ResumeState& s)
{
  nlohmann::json j;
  j["upload_id"] = s.upload_id;
  j["part_size"] = s.part_size;
  j["file_size"] = s.file_size;
  nlohmann::json parts = nlohmann::json::array();
  for (const auto& p : s.parts)
  {
    parts.push_back(p ? nlohmann::json(*p) : nlohmann::json(nullptr));
  }
  j["parts"] = std::move(parts);

  const std::string tmp = state_path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    out << j.dump();
    if (!out.good())
    {
      throw ObjectStoreError("failed to write multipart resume sidecar " + tmp);
    }
  }
  if (std::rename(tmp.c_str(), state_path.c_str()) != 0)
  {
    throw ObjectStoreError("failed to rename multipart resume sidecar into place");
  }
}

}  // namespace

std::string resume_state_path(const std::string& state_dir, const std::string& storage_name,
                              const std::string& object_key)
{
  // FNV-1a over storage_name, a NUL, then object_key.
  std::uint64_t hash = 0xcbf29ce484222325ULL;
  auto mix = [&hash](unsigned char byte) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 0x100000001b3ULL;
  };
  for (unsigned char c : storage_name)
  {
    mix(c);
  }
  mix(0);
  for (unsigned char c : object_key)
  {
    mix(c);
  }
  char name[32];
  std::snprintf(name, sizeof(name), "multipart-%016lx.json", static_cast<unsigned long>(hash));
  std::string sep = (!state_dir.empty() && state_dir.back() == '/') ? "" : "/";
  return state_dir + sep + name;
}

void upload_resumable(ObjectStore& store, const std::string& key, const std::string& local_path,
                      std::uint64_t file_size, std::uint64_t part_size, const std::string& state_path)
{
  const std::size_t n_parts = part_count(file_size, part_size);

  std::optional<ResumeState> loaded = load_state(state_path, part_size, file_size);
  ResumeState state;
  if (loaded)
  {
    state = std::move(*loaded);
  }
  else
  {
    state.upload_id = store.create_multipart(key);
    state.part_size = part_size;
    state.file_size = file_size;
    state.parts.assign(n_parts, std::nullopt);
    save_state(state_path, state);
  }

  std::ifstream file(local_path, std::ios::binary);
  if (!file.good())
  {
    throw ObjectStoreError("failed to open local file for multipart upload: " + local_path);
  }
  for (std::size_t idx = 0; idx < n_parts; ++idx)
  {
    if (state.parts[idx])
    {
      continue;
    }
    const std::uint64_t offset = static_cast<std::uint64_t>(idx) * part_size;
    const std::size_t len = static_cast<std::size_t>(std::min(part_size, file_size - offset));
    std::string buf(len, '\0');
    file.seekg(static_cast<std::streamoff>(offset));
    file.read(buf.data(), static_cast<std::streamsize>(len));
    if (static_cast<std::size_t>(file.gcount()) != len)
    {
      throw ObjectStoreError("short read while assembling multipart part " + std::to_string(idx));
    }
    // ObjectStore part numbers are 1-based (S3 convention).
    std::string etag = store.put_part(key, state.upload_id, static_cast<int>(idx) + 1, buf);
    state.parts[idx] = etag;
    save_state(state_path, state);
  }

  std::vector<std::string> etags;
  etags.reserve(state.parts.size());
  for (const auto& p : state.parts)
  {
    etags.push_back(*p);  // every part index was just filled in
  }
  store.complete_multipart(key, state.upload_id, etags);
  std::remove(state_path.c_str());
}

}  // namespace dc_bridge::uploader::multipart
