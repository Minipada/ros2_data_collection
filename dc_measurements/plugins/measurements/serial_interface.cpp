#include "dc_measurements/plugins/measurements/serial_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace dc_measurements
{

SerialInterface::SerialInterface() : dc_measurements::Measurement()
{
}

SerialInterface::~SerialInterface()
{
  closePort();
}

int SerialInterface::baudToSpeed(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200:
      return B1200;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      RCLCPP_ERROR_STREAM(logger_, "Unsupported serial baud_rate " << baud_rate << ", defaulting to 9600");
      return B9600;
  }
}

void SerialInterface::onConfigure()
{
  auto node = getNode();
  port_ = dc_util::get_str_type_param(node, measurement_name_, "port", "");
  baud_rate_ = dc_util::get_int_type_param(node, measurement_name_, "baud_rate", 9600);
  framing_ = dc_util::get_str_type_param(node, measurement_name_, "framing", "line");
  parsing_type_ = dc_util::get_str_type_param(node, measurement_name_, "parsing_type", "delimiter");
  delimiter_ = dc_util::get_str_type_param(node, measurement_name_, "delimiter", ",");
  regex_pattern_ = dc_util::get_str_type_param(node, measurement_name_, "regex", "");
  fields_ = dc_util::get_str_array_type_param(node, measurement_name_, "fields", std::vector<std::string>{});

  if (framing_ != "line")
  {
    RCLCPP_ERROR_STREAM(logger_,
                        "Unsupported serial framing '" << framing_ << "', only 'line' is implemented; using 'line'");
    framing_ = "line";
  }

  if (parsing_type_ == "regex")
  {
    try
    {
      compiled_regex_ = std::regex(regex_pattern_);
      regex_valid_ = true;
    }
    catch (const std::regex_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Invalid serial 'regex' pattern '" << regex_pattern_ << "': " << e.what());
      regex_valid_ = false;
    }
  }
  else if (parsing_type_ != "delimiter")
  {
    RCLCPP_ERROR_STREAM(logger_, "Unknown serial parsing_type '" << parsing_type_ << "', defaulting to 'delimiter'");
    parsing_type_ = "delimiter";
  }

  if (port_.empty())
  {
    RCLCPP_WARN_STREAM(logger_,
                       "Serial measurement '" << measurement_name_ << "' has no 'port' configured; will not read data");
  }

  // Deliberately don't open the port (or throw) here: an unplugged/not-yet-connected device
  // must not fail activation. collect() lazily opens (and re-opens) the port on every poll
  // instead, so activation always succeeds and reconnection happens automatically.
}

void SerialInterface::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "serial_interface.json");
  }
}

void SerialInterface::onCleanup()
{
  closePort();
}

bool SerialInterface::openPort()
{
  if (port_.empty())
  {
    return false;
  }

  int fd = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Could not open serial port '" << port_ << "': " << std::strerror(errno));
    return false;
  }

  termios tty{};
  if (::tcgetattr(fd, &tty) != 0)
  {
    RCLCPP_ERROR_STREAM(logger_, "tcgetattr failed on '" << port_ << "': " << std::strerror(errno));
    ::close(fd);
    return false;
  }

  ::cfmakeraw(&tty);
  speed_t speed = static_cast<speed_t>(baudToSpeed(baud_rate_));
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR_STREAM(logger_, "tcsetattr failed on '" << port_ << "': " << std::strerror(errno));
    ::close(fd);
    return false;
  }

  fd_ = fd;
  read_buffer_.clear();
  RCLCPP_INFO_STREAM(logger_, "Opened serial port '" << port_ << "' at " << baud_rate_ << " baud");
  return true;
}

void SerialInterface::closePort()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

dc_interfaces::msg::StringStamped SerialInterface::parseLine(const std::string& line)
{
  json fields_json = json::object();

  if (parsing_type_ == "regex")
  {
    if (regex_valid_)
    {
      std::smatch match;
      if (std::regex_search(line, match, compiled_regex_))
      {
        // std::regex has no native named-capture-group syntax; groups are paired with
        // the configured 'fields' list by capture order instead (group 1 -> fields[0], ...).
        for (size_t i = 1; i < match.size() && (i - 1) < fields_.size(); ++i)
        {
          fields_json[fields_[i - 1]] = match[i].str();
        }
        if (match.size() - 1 != fields_.size())
        {
          RCLCPP_WARN_STREAM(logger_, "Serial regex captured " << (match.size() - 1) << " group(s) but "
                                                               << fields_.size()
                                                               << " field name(s) are configured: " << line);
        }
      }
      else
      {
        RCLCPP_WARN_STREAM(logger_, "Serial line did not match configured regex: " << line);
      }
    }
  }
  else
  {
    std::vector<std::string> tokens;
    if (delimiter_.empty())
    {
      tokens.push_back(line);
    }
    else
    {
      size_t start = 0;
      size_t pos;
      while ((pos = line.find(delimiter_, start)) != std::string::npos)
      {
        tokens.push_back(line.substr(start, pos - start));
        start = pos + delimiter_.size();
      }
      tokens.push_back(line.substr(start));
    }

    for (size_t i = 0; i < tokens.size() && i < fields_.size(); ++i)
    {
      fields_json[fields_[i]] = tokens[i];
    }
    if (tokens.size() != fields_.size())
    {
      RCLCPP_WARN_STREAM(logger_, "Serial line split into " << tokens.size() << " token(s) but " << fields_.size()
                                                            << " field name(s) are configured: " << line);
    }
  }

  json data_json;
  data_json["raw"] = line;
  data_json["fields"] = fields_json;

  dc_interfaces::msg::StringStamped msg;
  msg.group_key = group_key_;
  msg.data = data_json.dump(-1, ' ', true);
  auto node = getNode();
  msg.header.stamp = node->get_clock()->now();
  return msg;
}

dc_interfaces::msg::StringStamped SerialInterface::collect()
{
  if (fd_ < 0 && !openPort())
  {
    // Port not (yet) available: nothing to report this cycle, will retry next poll.
    return dc_interfaces::msg::StringStamped();
  }

  char chunk[256];
  bool disconnected = false;
  while (true)
  {
    ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n > 0)
    {
      read_buffer_.append(chunk, static_cast<size_t>(n));
      continue;
    }
    if (n == 0)
    {
      // With VMIN=0/VTIME=0 raw-mode termios (set in openPort()), a serial/tty read()
      // returns 0 -- not -1/EAGAIN -- when no data is currently available; this is the
      // normal "nothing to read this cycle" case, not EOF/disconnect.
      break;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      // No more data available right now.
      break;
    }
    // A real I/O error (e.g. EIO/ENXIO when the device is unplugged).
    disconnected = true;
    break;
  }

  // read() alone can't distinguish "no data yet" from "the peer hung up" under VMIN=0/
  // VTIME=0 -- both return 0 on this kernel/tty implementation. poll()'s POLLHUP/POLLERR is
  // the reliable, distinct signal for an actual hangup (e.g. the other end of a pty closing,
  // or certain USB-serial removal paths), independent of what read() itself reported.
  if (!disconnected)
  {
    pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;
    if (::poll(&pfd, 1, 0) > 0 && (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)))
    {
      disconnected = true;
    }
  }

  if (disconnected)
  {
    RCLCPP_WARN_STREAM(logger_, "Serial port '" << port_ << "' disconnected; will attempt to reconnect");
    closePort();
    return dc_interfaces::msg::StringStamped();
  }

  // Line-delimited framing: keep only the most recently completed line this cycle (mirrors
  // the lossy-between-polls behavior other subscription-driven Measurements already have),
  // buffering any trailing partial line for the next poll.
  std::string line;
  bool has_line = false;
  size_t pos;
  while ((pos = read_buffer_.find('\n')) != std::string::npos)
  {
    line = read_buffer_.substr(0, pos);
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    read_buffer_.erase(0, pos + 1);
    has_line = true;
  }

  if (!has_line)
  {
    return dc_interfaces::msg::StringStamped();
  }

  return parseLine(line);
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::SerialInterface, dc_core::Measurement)
