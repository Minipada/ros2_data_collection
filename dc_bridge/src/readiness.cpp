// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/readiness.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>

namespace dc_bridge
{

bool probe(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout)
{
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
  {
    return false;
  }

  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(port);
  if (::inet_pton(AF_INET, host.c_str(), &sa.sin_addr) != 1)
  {
    ::close(fd);
    return false;
  }

  int flags = ::fcntl(fd, F_GETFL, 0);
  ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  bool ok = false;
  int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
  if (rc == 0)
  {
    ok = true;
  }
  else if (errno == EINPROGRESS)
  {
    pollfd pfd{ fd, POLLOUT, 0 };
    if (::poll(&pfd, 1, static_cast<int>(timeout.count())) > 0)
    {
      int soerr = 0;
      socklen_t len = sizeof(soerr);
      ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len);
      ok = (soerr == 0);
    }
  }

  ::close(fd);
  return ok;
}

}  // namespace dc_bridge
