
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MEMORY_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MEMORY_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "system/linux_parser.hpp"
#include "system/system.hpp"

namespace dc_measurements
{
// using MemoryMsg = dc_interfaces::msg::Memory;

class Memory : public dc_measurements::Measurement
{
public:
  Memory();
  ~Memory();
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CPU_HPP_
