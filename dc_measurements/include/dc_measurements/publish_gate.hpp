// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PUBLISH_GATE_HPP_
#define DC_MEASUREMENTS__PUBLISH_GATE_HPP_

namespace dc_measurements
{

/**
 * @class dc_measurements::PublishGate
 * @brief Whether one collected Record goes out: the init quota, the condition cap and the one-shot
 * gate latch of Measurement::publish(), with no ROS dependency (#482).
 *
 * Three mechanisms, applied in the order publish() uses them:
 *
 * - **Gate** (`gate_condition`): until the gate condition has read true once, every collection is
 *   held back. The latch never un-arms: a later false reading changes nothing, and the caller
 *   stops consulting the condition once open. The caller resolves the condition name and hands
 *   the reading in, so the gate knows nothing about Condition plugins.
 * - **Init quota** (`init_max_measurements`): the first N collections go out unconditionally
 *   (-1 disables the quota, 0 makes every collection unconditional). While the quota lasts the
 *   conditions are not consulted at all.
 * - **Condition cap** (`condition_max_measurements`): once the quota is spent, a collection goes
 *   out while the condition reads true -- unbounded when the cap is 0, up to N per
 *   continuously-true stretch when positive (-1 never); a false reading resets the count, so a
 *   fresh true stretch gets a fresh N.
 *
 * Nothing here reads a clock or touches ROS: the caller evaluates the conditions and offers the
 * outcomes, which is what makes the whole decision testable without a MeasurementServer (#482).
 */
class PublishGate
{
public:
  struct Config
  {
    int init_max{ 0 };             ///< First N collections go out unconditionally (-1 never, 0 always)
    int condition_max{ 0 };        ///< Cap while the condition reads true (-1 never, 0 unbounded)
    bool has_conditions{ false };  ///< Whether any if_all/if_any/if_none Condition is configured
    bool gate_enabled{ false };    ///< Whether gate_condition names a Condition at all
  };

  PublishGate() : PublishGate(Config{})
  {
  }

  explicit PublishGate(const Config& config) : config_(config)
  {
  }

  /// A gate-condition reading arrived: once one reads true the gate latches open for good.
  /// While open the reading is not even looked at, matching a caller that stops consulting
  /// the condition once the gate is open.
  bool openGate(const bool& gate_condition_state)
  {
    if (gateOpen())
    {
      return true;
    }
    gate_open_ = gate_condition_state;
    return gate_open_;
  }

  /// Whether collection may proceed at all right now; no gate configured counts as open.
  bool gateOpen() const
  {
    return !config_.gate_enabled || gate_open_;
  }

  /**
   * @brief Offer one collection; true means publish it.
   *
   * A closed gate rejects the collection without touching either counter -- the same effect the
   * early return in publish() always had -- so the gate and the counters stay consistent whoever
   * checks the gate first.
   */
  bool offer(const bool& condition_on)
  {
    if (!gateOpen())
    {
      return false;
    }

    // Init publish: unconditional, and the only mechanism consulted while the quota lasts.
    if (config_.init_max != -1 && (config_.init_max == 0 || init_counter_published_ < config_.init_max))
    {
      init_counter_published_++;
      return true;
    }

    if (!config_.has_conditions)
    {
      return false;
    }
    if (condition_on)
    {
      // Unbounded while the condition holds.
      if (config_.condition_max == 0)
      {
        return true;
      }
      // Capped: up to condition_max per continuously-true stretch.
      if (condition_counter_published_ < config_.condition_max)
      {
        condition_counter_published_++;
        return true;
      }
      return false;
    }
    // Condition dropped: a fresh true stretch starts from a full cap again.
    condition_counter_published_ = 0;
    return false;
  }

  /// Whether the init quota is spent with no condition publishing configured; publish() stops the
  /// collect timer when this turns true, ending collection for this Measurement.
  bool collectionFinished() const
  {
    return config_.init_max != 0 && config_.init_max != -1 && config_.condition_max < 0 &&
           init_counter_published_ == config_.init_max;
  }

  int initCounterPublished() const
  {
    return init_counter_published_;
  }

  int conditionCounterPublished() const
  {
    return condition_counter_published_;
  }

private:
  Config config_;
  int init_counter_published_{ 0 };
  int condition_counter_published_{ 0 };
  bool gate_open_{ false };
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PUBLISH_GATE_HPP_
