// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__INTERVENTION_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__INTERVENTION_HPP_

#include <string>

#include "dc_common/state_transition_detector.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/driving_mode_source.hpp"
#include "dc_measurements/measurement.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::Intervention
 * @brief Human takeovers, as a projection of driving-mode transitions rather than a second
 * detector: the same DrivingModeSource `driving_type` reads, fed to a StateTransitionDetector, so
 * the two can never disagree about when the robot was autonomous (#362). A Record is produced
 * only for the two boundaries `dc_kpi_intervention_events` (#369) already know how to read: a
 * transition from `autonomous` into `manual`/`teleop` starts a takeover, and the reverse ends one.
 */
class Intervention : public dc_measurements::Measurement
{
public:
  Intervention();
  ~Intervention() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  DrivingModeSource mode_source_;
  dc_common::StateTransitionDetector<std::string> detector_;

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__INTERVENTION_HPP_
