#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SERIAL_INTERFACE_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SERIAL_INTERFACE_HPP_

#include <regex>
#include <string>
#include <vector>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/node_utils.hpp"

namespace dc_measurements
{

class SerialInterface : public dc_measurements::Measurement
{
public:
  SerialInterface();
  ~SerialInterface() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  bool openPort();
  void closePort();
  int baudToSpeed(int baud_rate);
  dc_interfaces::msg::StringStamped parseLine(const std::string& line);

  int fd_{ -1 };
  std::string port_;
  int baud_rate_{ 9600 };
  std::string framing_;
  std::string parsing_type_;
  std::string delimiter_;
  std::string regex_pattern_;
  std::vector<std::string> fields_;
  std::regex compiled_regex_;
  bool regex_valid_{ false };
  std::string read_buffer_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SERIAL_INTERFACE_HPP_
