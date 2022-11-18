#include "network.hpp"

#include <chrono>
#include <memory>

namespace dc_measurements
{

Network::Network() : dc_measurements::Measurement()
{
}

void Network::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".ping_address",
                                               rclcpp::ParameterValue("8.8.8.8"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".ping_timeout", rclcpp::ParameterValue(200));
  node->get_parameter(measurement_name_ + ".ping_address", ping_address_);
  node->get_parameter(measurement_name_ + ".ping_timeout", ping_timeout_ms_);

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

  if ((skt = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP)) < 0)
  {
    RCLCPP_ERROR(logger_, "ping: [ICMP] unknown protocol or Permission denied, try again with sudo");
  }
  setsockopt(skt, IPPROTO_IP, IP_TTL, (char*)&ttl, sizeof(ttl));
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

  i = sendto(skt, (char*)outpack, cc, 0, (struct sockaddr*)&to_, (socklen_t)sizeof(struct sockaddr_in));
  if (i < 0 || i != cc)
  {
    if (i < 0)
    {
      RCLCPP_ERROR(logger_, "ping: Error occured in sendto call");
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
  if (if_nidxs != nullptr)
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
  int cc, fromlen, hlen, triptime;
  struct ip* ip;
  struct timeval timeout_str, *ep;
  fd_set rfds;

  FD_ZERO(&rfds);
  FD_SET(skt, &rfds);
  timeout_str.tv_sec = 0;
  timeout_str.tv_usec = ping_timeout_ms_ * 1000;
  for (;;)
  {
    cc = select(32, &rfds, 0, 0, &timeout_str);
    if (cc == -1)
    {
      return -1;
    }
    else if (cc)
    {
      struct icmp* icp;
      struct timeval tv;
      fromlen = sizeof(sockaddr_in);
      if ((cc = recvfrom(skt, packet_, sizeof(packet_), 0, (struct sockaddr*)&from_, (socklen_t*)&fromlen)) < 0)
      {
        RCLCPP_ERROR(logger_, "ping: Error occured in recvfrom call");
        return -1;
      }
      gettimeofday(&tv, &tz_);

      ip = (struct ip*)((char*)packet_);
      hlen = ip->ip_hl << 2;
      if (cc < (hlen + ICMP_MINLEN))
      {
        RCLCPP_ERROR(logger_, "ping: Packet too short (%d bytes) from_ %s", cc, hostname_.c_str());
        return -1;
      }

      cc -= hlen;
      icp = (struct icmp*)(packet_ + hlen);
      if (icp->icmp_type != ICMP_ECHOREPLY)
      {
        RCLCPP_DEBUG(logger_, "%d bytes from_ %s, icmp_type=%d, icmp_code=%d", cc, inet_ntoa(from_.sin_addr),
                     icp->icmp_type, icp->icmp_code);
        return -1;
      }
      if (icp->icmp_id != id_)
        return -1;

      ep = (struct timeval*)&icp->icmp_data[0];
      if ((tv.tv_usec -= ep->tv_usec) < 0)
      {
        tv.tv_sec--;
        tv.tv_usec += 1000000;
      }
      tv.tv_sec -= ep->tv_sec;
      triptime = tv.tv_sec * 1000 + (tv.tv_usec / 1000);
      tsum += triptime;
      if (triptime < tmin)
        tmin = triptime;
      if (triptime > tmax)
        tmax = triptime;

      RCLCPP_DEBUG(logger_, "%d bytes from_ %s, icmp_type=%d, icmp_code=%d, ttl=%d, triptime=%d ms", cc,
                   inet_ntoa(from_.sin_addr), icp->icmp_type, icp->icmp_code, ttl, triptime);
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
    data_json["online"] = (bool)ping_value;
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
