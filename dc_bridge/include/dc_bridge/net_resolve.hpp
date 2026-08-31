// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_BRIDGE__NET_RESOLVE_HPP_
#define DC_BRIDGE__NET_RESOLVE_HPP_

#include <netinet/in.h>

#include <cstdint>
#include <string>

namespace dc_bridge
{

/// Resolves `host` — an IPv4 literal or a hostname — to a `sockaddr_in` for `port`, via
/// `getaddrinfo()` restricted to AF_INET/SOCK_STREAM. A literal address is returned as-is
/// (no network round trip); a hostname is resolved through the system resolver, which is
/// what lets `vector_forward_host`/Destination hosts name a container-network peer (e.g.
/// a Compose service) rather than only ever a fixed IP. Returns false, leaving `out`
/// unspecified, if `host` resolves to no usable IPv4 address at all.
bool resolve_ipv4(const std::string& host, std::uint16_t port, sockaddr_in& out);

}  // namespace dc_bridge

#endif  // DC_BRIDGE__NET_RESOLVE_HPP_
