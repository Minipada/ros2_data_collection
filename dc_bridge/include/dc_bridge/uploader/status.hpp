// Status-Record row shapes the Uploader emits (ADR-0005), preserving the Humble
// out_files_metrics column set.
#ifndef DC_BRIDGE__UPLOADER__STATUS_HPP_
#define DC_BRIDGE__UPLOADER__STATUS_HPP_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "dc_bridge/uploader/group.hpp"
#include "dc_bridge/uploader/object_store.hpp"

namespace dc_bridge::uploader
{

/// Sniffed/stat'd metadata for one local File.
struct FileMeta
{
  std::uint64_t size;
  std::string content_type;
  std::optional<double> duration;
};

namespace status
{

/// The three per-(group, File, Destination) file_status rows, preserving the Humble
/// column set. `remote_path` is the File's object path on `storage` (before key_prefix).
nlohmann::json uploaded_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                            const std::string& remote_path, const FileMeta& meta);
nlohmann::json missing_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                           const std::string& remote_path);
nlohmann::json deleted_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                           const std::string& remote_path);

/// The retention policy's audit row (#267): a File shed under disk pressure without ever
/// having been uploaded — distinct from deleted_row's `uploaded: true` (delete_when_sent
/// only ever deletes a File after it's fully verified). `deleted: true, uploaded: false`
/// is the queryable "shed without upload" signature.
nlohmann::json shed_row(const FileGroup& group, const FileRef& file, const Storage& storage,
                        const std::string& remote_path);

/// ADR-0005's group completion marker.
nlohmann::json group_complete_row(const FileGroup& group);

}  // namespace status
}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__STATUS_HPP_
