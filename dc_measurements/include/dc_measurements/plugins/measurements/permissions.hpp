
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

#include <filesystem>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace dc_measurements
{

class Permissions : public dc_measurements::Measurement
{
public:
  Permissions();
  ~Permissions();
  dc_interfaces::msg::StringStamped collect() override;

private:
  struct stat getOwner(const std::string& path);
  std::string formatPermissions(const mode_t& perm, std::string format);
  std::string path_;
  std::string full_path_;
  std::string permission_format_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__PERMISSIONS_HPP_
