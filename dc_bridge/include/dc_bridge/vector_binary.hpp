// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Locates the vendored Vector binary installed by the vector_vendor package.
#ifndef DC_BRIDGE__VECTOR_BINARY_HPP_
#define DC_BRIDGE__VECTOR_BINARY_HPP_

#include <optional>
#include <string>

namespace dc_bridge
{

/// Locates the vendored Vector binary by scanning a colon-separated AMENT_PREFIX_PATH
/// (as colcon sets it) for `lib/vector_vendor/vector`, the install path
/// vector_vendor's CMakeLists.txt installs to. Returns the first match, or nullopt.
std::optional<std::string> find_vector_binary(const std::string& ament_prefix_path);

}  // namespace dc_bridge

#endif  // DC_BRIDGE__VECTOR_BINARY_HPP_
