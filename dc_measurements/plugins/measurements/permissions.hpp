
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__PERMISSIONS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__PERMISSIONS_HPP_

#include <fcntl.h>
#include <grp.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/string_utils.hpp"

namespace dc_measurements
{

enum class Formatpermissions
{
  RWX = 0,
  INT = 1,
};

class Permissions : public dc_measurements::Measurement
{
public:
  Permissions();
  ~Permissions();
  dc_interfaces::msg::StringStamped collect() override;

private:
  struct stat getOwner(const std::string& path);
  std::string formatPermissions(const mode_t& perm, Formatpermissions format);
  std::string path_;
  std::string full_path_;
  Formatpermissions permission_format_;
  int permission_format_setting_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__PERMISSIONS_HPP_
