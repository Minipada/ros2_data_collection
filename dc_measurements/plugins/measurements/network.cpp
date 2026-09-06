// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/network.hpp"

namespace dc_measurements
{

Network::Network() : dc_measurements::Measurement()
{
}

void Network::onConfigure()
{
  auto node = getNode();
  ping_address_ = dc_util::get_str_type_param(node, measurement_name_, "ping_address", "8.8.8.8");
  ping_timeout_ms_ = dc_util::get_int_type_param(node, measurement_name_, "ping_timeout", 200);

  // Validate parameters
  // https://stackoverflow.com/questions/5284147/validating-ipv4-addresses-with-regexp
  boost::system::error_code ec;
  boost::asio::ip::address::from_string(ping_address_, ec);
  if (ec)
  {
    RCLCPP_ERROR(logger_, "Must be a valid ip address");
    throw std::runtime_error("Must be a valid ip address");
  }
  if (ping_timeout_ms_ < 1)
  {
    RCLCPP_ERROR(logger_, "Timeout must be an integer superior to 0");
    throw std::runtime_error("Timeout must be an integer superior to 0");
  }

  bzero((char*)&to_, sizeof(struct sockaddr_in));
  to_.sin_family = AF_INET;
  to_.sin_addr.s_addr = inet_addr(ping_address_.c_str());
  hostname_ = ping_address_;
  hp_ = gethostbyname(ping_address_.c_str());
  if (hp_)
  {
    to_.sin_family = hp_->h_addrtype;
    bcopy(hp_->h_addr, (caddr_t)&to_.sin_addr, hp_->h_length);
    hostname_ = hp_->h_name;
  }
  else {}
  id_ = getpid() & 0xFFFF;

  // SOCK_DGRAM + IPPROTO_ICMP is the unprivileged "ping socket" (Linux >=3.0): the kernel
  // handles the IP layer itself, gated only by net.ipv4.ping_group_range rather than
  // CAP_NET_RAW/root. SOCK_RAW needed real root -- which rootless container runtimes
  // (Podman, Docker) can never grant regardless of --cap-add/--privileged, since the
  // process is still an unprivileged UID from the kernel's point of view.
  if ((skt_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_ICMP)) < 0)
  {
    RCLCPP_ERROR(logger_, "ping: [ICMP] permission denied -- check net.ipv4.ping_group_range");
  }
  setsockopt(skt_, IPPROTO_IP, IP_TTL, (char*)&ttl_, sizeof(ttl_));
}

void Network::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "network.json");
  }
}

Network::~Network() = default;

bool Network::ping()
{
  int i, cc = datalen_ + 8;

  unsigned char outpack[MAXPACKETLEN];
  struct icmp* icmp_rout = (struct icmp*)outpack;
  struct timeval* tp = (struct timeval*)&outpack[8];

  icmp_rout->icmp_type = ICMP_ECHO;
  icmp_rout->icmp_code = 0;
  icmp_rout->icmp_cksum = 0;
  icmp_rout->icmp_seq = ntransmitted_++;
  icmp_rout->icmp_id = id_;

  gettimeofday(tp, &tz_);
  icmp_rout->icmp_cksum = inCksum((unsigned short*)icmp_rout, cc);

  i = sendto(skt_, (char*)outpack, cc, 0, (struct sockaddr*)&to_, (socklen_t)sizeof(struct sockaddr_in));
  if (i < 0 || i != cc)
  {
    if (i < 0)
    {
      RCLCPP_ERROR(logger_, "ping: Error occurred in sendto call");
      return false;
    }
    RCLCPP_ERROR(logger_, "ping: Sent to_ %s %d characters and received %d", hostname_.c_str(), cc, i);
  }
  return true;
}

json Network::getNetworkInterfaces()
{
  json interfaces = json::array();
  struct if_nameindex *if_nidxs, *intf;

  if_nidxs = if_nameindex();
  if (if_nidxs != NULL)
  {
    for (intf = if_nidxs; intf->if_index != 0 || intf->if_name != NULL; intf++)
    {
      interfaces.push_back(intf->if_name);
    }

    if_freenameindex(if_nidxs);
  }
  return interfaces;
}

uint16_t Network::inCksum(unsigned short* addr, unsigned int len)
{
  uint16_t answer = 0;
  uint32_t sum = 0;
  unsigned short* buf = (unsigned short*)addr;

  for (sum = 0; len > 1; len -= 2)
    sum += *buf++;
  if (len == 1)
    sum += *(unsigned char*)buf;

  sum = (sum >> 16) + (sum & 0xffff);
  sum += (sum >> 16);
  answer = ~sum;
  return answer;
}

int Network::unpack()
{
  int cc, fromlen, triptime;
  struct timeval timeout_str, *ep;
  fd_set rfds;

  FD_ZERO(&rfds);
  FD_SET(skt_, &rfds);
  timeout_str.tv_sec = 0;
  timeout_str.tv_usec = ping_timeout_ms_ * 1000;
  for (;;)
  {
    cc = select(32, &rfds, NULL, NULL, &timeout_str);
    if (cc == -1)
    {
      return -1;
    }
    else if (cc)
    {
      struct icmp* icp;
      struct timeval tv;
      fromlen = sizeof(sockaddr_in);
      if ((cc = recvfrom(skt_, packet_, sizeof(packet_), 0, (struct sockaddr*)&from_, (socklen_t*)&fromlen)) < 0)
      {
        RCLCPP_ERROR(logger_, "ping: Error occurred in recvfrom call");
        return -1;
      }
      gettimeofday(&tv, &tz_);

      // A SOCK_DGRAM ping socket delivers just the ICMP message -- no IP header in front
      // of it to skip, unlike SOCK_RAW.
      if (cc < ICMP_MINLEN)
      {
        RCLCPP_ERROR(logger_, "ping: Packet too short (%d bytes) from_ %s", cc, hostname_.c_str());
        return -1;
      }

      icp = (struct icmp*)packet_;
      if (icp->icmp_type != ICMP_ECHOREPLY)
      {
        RCLCPP_DEBUG(logger_, "%d bytes from_ %s, icmp_type=%d, icmp_code=%d", cc, inet_ntoa(from_.sin_addr),
                     icp->icmp_type, icp->icmp_code);
        return -1;
      }
      // No icmp_id cross-check here: a ping socket is already demultiplexed by the kernel
      // to only deliver replies matching what this socket itself sent, and the kernel
      // rewrites icmp_id on the wire to its own per-socket value on both send and receive
      // -- verified directly (sent id_=1956, delivered reply's icmp_id=2561) -- so
      // comparing against our own id_ here would reject every real reply.

      ep = (struct timeval*)&icp->icmp_data[0];
      if ((tv.tv_usec -= ep->tv_usec) < 0)
      {
        tv.tv_sec--;
        tv.tv_usec += 1000000;
      }
      tv.tv_sec -= ep->tv_sec;
      triptime = tv.tv_sec * 1000 + (tv.tv_usec / 1000);
      tsum_ += triptime;
      if (triptime < tmin_)
        tmin_ = triptime;
      if (triptime > tmax_)
        tmax_ = triptime;

      RCLCPP_DEBUG(logger_, "%d bytes from_ %s, icmp_type=%d, icmp_code=%d, ttl_=%d, triptime=%d ms", cc,
                   inet_ntoa(from_.sin_addr), icp->icmp_type, icp->icmp_code, ttl_, triptime);
      return triptime;
    }
    else
    {
      RCLCPP_DEBUG(logger_, "%d bytes from_ %s (%s). Time exceeded: Hop limit.", cc, hostname_.c_str(),
                   inet_ntoa(from_.sin_addr));
      return -1;
    }
  }
  return -1;
}

dc_interfaces::msg::StringStamped Network::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  if (ping())
  {
    int ping_value = unpack();
    data_json["ping"] = ping_value;
    // Not (bool)ping_value: a 0ms reply (bool false) would wrongly read as offline, and
    // unpack()'s -1 failure sentinel (bool true) would wrongly read as online.
    data_json["online"] = (ping_value >= 0);
  }
  else
  {
    data_json["ping"] = -1;
    data_json["online"] = false;
  }
  data_json["interfaces"] = getNetworkInterfaces();
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Network, dc_core::Measurement)
