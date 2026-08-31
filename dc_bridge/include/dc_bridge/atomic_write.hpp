// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Crash/partial-write-safe file writes (#444): a reader polling a path — the Shipper
// watching its rendered config on a shared volume, in unmanaged-shipper mode — must never
// observe a truncated or half-written file.
#ifndef DC_BRIDGE__ATOMIC_WRITE_HPP_
#define DC_BRIDGE__ATOMIC_WRITE_HPP_

#include <string>

namespace dc_bridge
{

/// Writes `content` to `path` atomically: a full write to `<path>.tmp` followed by
/// `rename()` over `path`, so `path` itself is either the previous complete file or the
/// new complete one, never a partial write (same tmp+rename convention as the Uploader's
/// intent queue, #265). Throws std::runtime_error if either step fails.
void write_file_atomically(const std::string& path, const std::string& content);

}  // namespace dc_bridge

#endif  // DC_BRIDGE__ATOMIC_WRITE_HPP_
