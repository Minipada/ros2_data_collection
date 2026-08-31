// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Split out of uploader.hpp (#446) so a process that only needs to know the Uploader's
// Tag — the Bridge, wiring Vector's routing for it — doesn't have to pull in the full
// Uploader/ObjectStore/Retention class surface just for one constant.
#ifndef DC_BRIDGE__UPLOADER__FILE_STATUS_TAG_HPP_
#define DC_BRIDGE__UPLOADER__FILE_STATUS_TAG_HPP_

namespace dc_bridge::uploader
{

/// The Tag the Uploader emits status/metadata Records under.
inline constexpr const char* FILE_STATUS_TAG = "dc.files";

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__FILE_STATUS_TAG_HPP_
