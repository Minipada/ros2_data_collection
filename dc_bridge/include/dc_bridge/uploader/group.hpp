// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Parsing the Files a Record references out of its JSON payload — the
// local_paths/remote_paths structure Measurements embed (camera/map), replacing the
// Humble-era parallel src_fields/upload_fields arrays. remote_paths is keyed by
// Destination name; the payload is walked recursively so a bare Measurement Record and a
// Group-merged Record parse identically; base64 subtrees are skipped. All Files found in
// one Record form one group (the unit ADR-0005's completion marker covers).
#ifndef DC_BRIDGE__UPLOADER__GROUP_HPP_
#define DC_BRIDGE__UPLOADER__GROUP_HPP_

#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace dc_bridge::uploader
{

/// One File referenced by a Record.
struct FileRef
{
  std::string key;         ///< the local_paths key naming this File ("pgm", "raw", …).
  std::string local_path;  ///< the on-robot path.
  /// Destination name → object key, from remote_paths.<destination>.<key>. Only
  /// configured Destinations are retained; never empty.
  std::map<std::string, std::string> remote_paths;
};

/// Every File one Record references — the unit the group completion marker is emitted for.
struct FileGroup
{
  std::string group_name;
  std::optional<std::string> robot_name;
  std::optional<nlohmann::json> robot_id;
  std::vector<FileRef> files;  ///< deterministic (local-path-sorted) order.
};

/// Extracts the FileGroup a Record payload references. `fallback_group` names the group
/// when the payload carries no `name` field; `storages` is the set of configured
/// `receives: files` Destination names. The group is empty if the Record references no
/// Files any configured Destination receives.
FileGroup parse_file_group(const nlohmann::json& payload, const std::string& fallback_group,
                           const std::set<std::string>& storages);

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__GROUP_HPP_
