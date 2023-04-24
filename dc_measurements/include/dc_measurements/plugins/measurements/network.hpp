
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__NETWORK_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__NETWORK_HPP_

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <limits>
#include <thread>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

#define MAXPACKETLEN 4096
#define TIMEOUT 10
#define TTL 54

namespace dc_measurements
{

class Network : public dc_measurements::Measurement
{
public:
  Network();
  ~Network() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  std::string ping_address_;
  int ping_timeout_ms_;
  bool ping();
  int unpack();
  json getNetworkInterfaces();
  uint16_t inCksum(unsigned short* addr, unsigned int len);

  int skt_, ttl_ = TTL;
  struct sockaddr_in to_, from_;
  unsigned char packet_[MAXPACKETLEN];
  std::string hostname_;
  struct hostent* hp_;
  struct timezone tz_;
  int id_, datalen_ = 56;
  int ntransmitted_ = 0;
  int tmin_ = std::numeric_limits<int>::max(), tmax_ = 0, tsum_ = 0;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__NETWORK_HPP_
