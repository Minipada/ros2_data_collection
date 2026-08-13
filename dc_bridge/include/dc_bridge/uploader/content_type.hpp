// Content-type sniffing for uploaded Files (ADR-0005), replacing the Humble Go plugin's
// http.DetectContentType. Magic-byte based for the formats DC Measurements produce
// (camera JPEGs, map PGM/PNG/YAML, videos), with a UTF-8 text fallback. Pure, no I/O.
#ifndef DC_BRIDGE__UPLOADER__CONTENT_TYPE_HPP_
#define DC_BRIDGE__UPLOADER__CONTENT_TYPE_HPP_

#include <string>

namespace dc_bridge::content_type
{

/// Sniffs a content type from the first bytes of a File (512 bytes is plenty).
std::string sniff(const std::string& buf);

/// Whether a sniffed content type is a video — the gate for ffprobe duration probing.
bool is_video(const std::string& content_type);

/// Whether a sniffed content type is an image. Together with is_video, the gate for
/// optional thumbnail generation (#256).
bool is_image(const std::string& content_type);

}  // namespace dc_bridge::content_type

#endif  // DC_BRIDGE__UPLOADER__CONTENT_TYPE_HPP_
