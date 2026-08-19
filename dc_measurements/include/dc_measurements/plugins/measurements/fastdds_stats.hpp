// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__FASTDDS_STATS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__FASTDDS_STATS_HPP_

#include <chrono>
#include <cstdint>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "fastdds_statistics_backend/StatisticsBackend.hpp"
#include "fastdds_statistics_backend/types/EntityId.hpp"

namespace dc_measurements
{

// Reads eProsima Fast DDS's own Statistics Module through Fast-DDS-statistics-backend and emits
// it as a periodic-sample Record, the same shape every other Measurement uses (#392). Only
// meaningful when the deployment runs Fast DDS as its RMW -- this plugin only builds when
// `fastdds_statistics_backend` was found at CMake configure time; see
// dc_measurements/CMakeLists.txt and doc/src/dc/measurements/fastdds_stats.md.
class FastddsStats : public dc_measurements::Measurement
{
public:
  FastddsStats();
  ~FastddsStats() override;
  dc_interfaces::msg::StringStamped collect() override;

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;

private:
  uint32_t domain_id_{ 0 };
  eprosima::statistics_backend::EntityId monitor_id_{ eprosima::statistics_backend::EntityId::invalid() };
  // Statistics data is queried for the window since the previous poll, so two consecutive
  // Records never double-count the same sample.
  std::chrono::system_clock::time_point last_poll_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__FASTDDS_STATS_HPP_
