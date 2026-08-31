// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_BRIDGE__NET_RESOLVE_HPP_
#define DC_BRIDGE__NET_RESOLVE_HPP_

#include <netinet/in.h>

#include <cstdint>
#include <string>

namespace dc_bridge
{

/// Resolves `host` (IPv4 literal or hostname) to a `sockaddr_in` for `port` via
/// getaddrinfo(). Returns false if `host` has no usable IPv4 address.
bool resolve_ipv4(const std::string& host, std::uint16_t port, sockaddr_in& out);

}  // namespace dc_bridge

#endif  // DC_BRIDGE__NET_RESOLVE_HPP_
