// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/net_resolve.hpp"

#include <arpa/inet.h>
#include <netdb.h>

namespace dc_bridge
{

bool resolve_ipv4(const std::string& host, std::uint16_t port, sockaddr_in& out)
{
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* result = nullptr;
  // Port applied below, once we know which address came back.
  const int rc = ::getaddrinfo(host.c_str(), nullptr, &hints, &result);
  if (rc != 0 || result == nullptr)
  {
    return false;
  }

  out = *reinterpret_cast<sockaddr_in*>(result->ai_addr);
  out.sin_port = htons(port);
  ::freeaddrinfo(result);
  return true;
}

}  // namespace dc_bridge
