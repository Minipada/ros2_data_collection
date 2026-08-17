#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "dc_measurements/incident_releaser.hpp"

using dc_measurements::IncidentReleaser;
using dc_measurements::IncidentState;
using dc_measurements::toString;

// One captured emission, standing in for a ROS publish: what went out, stamped with what, under
// which incident.
struct Published
{
  std::string json;
  IncidentReleaser::TimePoint stamp;
  std::string incident_id;
};

class IncidentReleaserTest : public ::testing::Test
{
protected:
  // Fake clock: every entry point takes the time from the test, so nothing here reads a real one.
  // Starting well past the epoch keeps `base_ - something` valid.
  IncidentReleaser::TimePoint base_{ std::chrono::system_clock::time_point() + std::chrono::hours(24) };

  IncidentReleaser::TimePoint at(const double& seconds) const
  {
    return base_ + dc_measurements::durationFromSeconds(seconds);
  }

  std::vector<Published> published_;

  IncidentReleaser::PublishFn capture()
  {
    return [this](const std::string& json, const IncidentReleaser::TimePoint& stamp, const std::string& incident_id) {
      published_.push_back(Published{ json, stamp, incident_id });
    };
  }

  // buffer 10s, post-roll/cooldown per test.
  IncidentReleaser makeReleaser(const double& post_roll_sec, const double& cooldown_sec)
  {
    return IncidentReleaser(std::chrono::seconds(10), dc_measurements::durationFromSeconds(post_roll_sec),
                            dc_measurements::durationFromSeconds(cooldown_sec), capture());
  }

  std::vector<std::string> publishedJson() const
  {
    std::vector<std::string> out;
    for (const auto& entry : published_)
    {
      out.push_back(entry.json);
    }
    return out;
  }
};

TEST_F(IncidentReleaserTest, StartsArmedAndBufferingPublishesNothing)
{
  auto releaser = makeReleaser(0.0, 0.0);

  releaser.offer("{\"a\":1}", at(0.0));
  releaser.offer("{\"a\":2}", at(1.0));
  releaser.tick(at(2.0));

  EXPECT_TRUE(releaser.isArmed());
  EXPECT_EQ(releaser.state(), IncidentState::Buffering);
  EXPECT_EQ(releaser.bufferedCount(), 2u);
  EXPECT_TRUE(published_.empty());
  EXPECT_TRUE(releaser.incidentId().empty());
}

TEST_F(IncidentReleaserTest, FlushReleasesBufferedWindowOldestFirstWithOriginalStamps)
{
  auto releaser = makeReleaser(0.0, 0.0);

  releaser.offer("{\"a\":1}", at(0.0));
  releaser.offer("{\"a\":2}", at(1.0));
  releaser.offer("{\"a\":3}", at(2.0));

  releaser.onFlush("incident-1", at(3.0));

  ASSERT_EQ(published_.size(), 3u);
  EXPECT_EQ(publishedJson(), (std::vector<std::string>{ "{\"a\":1}", "{\"a\":2}", "{\"a\":3}" }));
  EXPECT_EQ(published_[0].stamp, at(0.0));
  EXPECT_EQ(published_[1].stamp, at(1.0));
  EXPECT_EQ(published_[2].stamp, at(2.0));
  for (const auto& entry : published_)
  {
    EXPECT_EQ(entry.incident_id, "incident-1");
  }
  // The released window is consumed, not replayed on the next incident.
  EXPECT_EQ(releaser.bufferedCount(), 0u);
}

TEST_F(IncidentReleaserTest, SamplesOlderThanBufferWindowAreNotReleased)
{
  auto releaser = makeReleaser(0.0, 0.0);

  releaser.offer("{\"a\":\"stale\"}", at(0.0));
  releaser.offer("{\"a\":\"fresh\"}", at(20.0));

  releaser.onFlush("incident-1", at(21.0));

  ASSERT_EQ(published_.size(), 1u);
  EXPECT_EQ(published_[0].json, "{\"a\":\"fresh\"}");
}

// Acceptance criterion: post-roll transition and duration.
TEST_F(IncidentReleaserTest, PostRollPublishesLiveUnderTheSameIncidentIdForItsDuration)
{
  auto releaser = makeReleaser(5.0, 0.0);

  releaser.offer("{\"a\":\"pre\"}", at(0.0));
  releaser.onFlush("incident-1", at(1.0));

  ASSERT_EQ(published_.size(), 1u);
  EXPECT_EQ(releaser.state(), IncidentState::PostRoll) << "state is " << toString(releaser.state());
  EXPECT_FALSE(releaser.isArmed());
  EXPECT_EQ(releaser.incidentId(), "incident-1");

  // Within post-roll: published live, right away, still tagged with the incident.
  releaser.offer("{\"a\":\"post-1\"}", at(2.0));
  releaser.offer("{\"a\":\"post-2\"}", at(5.9));
  ASSERT_EQ(published_.size(), 3u);
  EXPECT_EQ(published_[1].json, "{\"a\":\"post-1\"}");
  EXPECT_EQ(published_[1].stamp, at(2.0));
  EXPECT_EQ(published_[1].incident_id, "incident-1");
  EXPECT_EQ(published_[2].incident_id, "incident-1");
  EXPECT_EQ(releaser.bufferedCount(), 0u);

  // Post-roll ends 5s after the flush (at 6.0): this sample is buffered, not published, and with
  // no cooldown configured the machine is armed again.
  releaser.offer("{\"a\":\"after\"}", at(6.1));
  EXPECT_EQ(published_.size(), 3u);
  EXPECT_EQ(releaser.state(), IncidentState::Buffering) << "state is " << toString(releaser.state());
  EXPECT_TRUE(releaser.isArmed());
  EXPECT_EQ(releaser.bufferedCount(), 1u);
  EXPECT_TRUE(releaser.incidentId().empty());
}

// Acceptance criterion: cooldown suppresses a second flush during the window.
TEST_F(IncidentReleaserTest, CooldownIgnoresASecondFlushWithinItsWindow)
{
  auto releaser = makeReleaser(2.0, 5.0);

  releaser.offer("{\"a\":\"pre\"}", at(0.0));
  releaser.onFlush("incident-1", at(1.0));
  ASSERT_EQ(published_.size(), 1u);

  // Still in post-roll: a second FlushEvent belongs to the incident already being captured.
  releaser.onFlush("incident-2", at(2.0));
  EXPECT_EQ(published_.size(), 1u);
  EXPECT_EQ(releaser.state(), IncidentState::PostRoll);
  EXPECT_EQ(releaser.incidentId(), "incident-1");

  // Post-roll ended at 3.0, cooldown runs to 8.0: samples buffer again but flushes stay ignored.
  releaser.offer("{\"a\":\"during-cooldown\"}", at(4.0));
  EXPECT_EQ(releaser.state(), IncidentState::Cooldown);
  EXPECT_FALSE(releaser.isArmed());
  EXPECT_EQ(releaser.bufferedCount(), 1u);

  releaser.onFlush("incident-3", at(7.9));
  EXPECT_EQ(published_.size(), 1u);
  EXPECT_EQ(releaser.state(), IncidentState::Cooldown);
}

// Acceptance criterion: auto re-arm after cooldown.
TEST_F(IncidentReleaserTest, ReArmsItselfAfterCooldownAndCapturesTheNextIncident)
{
  auto releaser = makeReleaser(2.0, 5.0);

  releaser.onFlush("incident-1", at(1.0));
  // Post-roll ends at 3.0, cooldown at 8.0.
  releaser.offer("{\"a\":\"during-cooldown\"}", at(4.0));
  ASSERT_FALSE(releaser.isArmed());

  releaser.tick(at(8.0));
  EXPECT_TRUE(releaser.isArmed()) << "still in " << toString(releaser.state()) << " after cooldown";
  EXPECT_EQ(releaser.state(), IncidentState::Buffering);
  EXPECT_TRUE(releaser.incidentId().empty());

  // The window buffered during cooldown is the next incident's pre-roll -- no hole at its start.
  releaser.offer("{\"a\":\"after-rearm\"}", at(9.0));
  EXPECT_EQ(releaser.bufferedCount(), 2u);

  releaser.onFlush("incident-2", at(10.0));
  ASSERT_EQ(published_.size(), 2u);
  EXPECT_EQ(published_[0].json, "{\"a\":\"during-cooldown\"}");
  EXPECT_EQ(published_[0].incident_id, "incident-2");
  EXPECT_EQ(published_[1].json, "{\"a\":\"after-rearm\"}");
  EXPECT_EQ(published_[1].incident_id, "incident-2");
}

// A FlushEvent arriving exactly when cooldown expires is honored: the window is inclusive of its
// own end, so an armed machine never silently drops the event that re-armed it.
TEST_F(IncidentReleaserTest, FlushAtTheCooldownDeadlineIsHonored)
{
  auto releaser = makeReleaser(2.0, 5.0);

  releaser.onFlush("incident-1", at(0.0));
  releaser.offer("{\"a\":\"buffered\"}", at(4.0));
  // Post-roll ended at 2.0, cooldown ends exactly at 7.0.
  releaser.onFlush("incident-2", at(7.0));

  ASSERT_EQ(published_.size(), 1u);
  EXPECT_EQ(published_[0].incident_id, "incident-2");
}

// Default configuration (both durations 0): pre-roll only, and back to buffering right away rather
// than publishing live from then on.
TEST_F(IncidentReleaserTest, ZeroPostRollAndCooldownReArmImmediatelyAfterRelease)
{
  auto releaser = makeReleaser(0.0, 0.0);

  releaser.offer("{\"a\":\"pre\"}", at(0.0));
  releaser.onFlush("incident-1", at(1.0));

  ASSERT_EQ(published_.size(), 1u);
  EXPECT_EQ(releaser.state(), IncidentState::Buffering);
  EXPECT_TRUE(releaser.isArmed());

  // Samples after the release go back into the buffer instead of being published live.
  releaser.offer("{\"a\":\"next\"}", at(2.0));
  EXPECT_EQ(published_.size(), 1u);
  EXPECT_EQ(releaser.bufferedCount(), 1u);

  // ... and a second incident is captured immediately, with its own id.
  releaser.onFlush("incident-2", at(3.0));
  ASSERT_EQ(published_.size(), 2u);
  EXPECT_EQ(published_[1].json, "{\"a\":\"next\"}");
  EXPECT_EQ(published_[1].incident_id, "incident-2");
}

// Cooldown with no post-roll still suppresses, measured from the flush itself.
TEST_F(IncidentReleaserTest, ZeroPostRollStillEntersCooldownWhenOneIsConfigured)
{
  auto releaser = makeReleaser(0.0, 5.0);

  releaser.onFlush("incident-1", at(0.0));
  EXPECT_EQ(releaser.state(), IncidentState::Cooldown);
  EXPECT_FALSE(releaser.isArmed());

  releaser.tick(at(4.9));
  EXPECT_FALSE(releaser.isArmed());
  releaser.tick(at(5.0));
  EXPECT_TRUE(releaser.isArmed());
}

// Nothing buffered yet is not an error: the incident's post-roll is captured all the same.
TEST_F(IncidentReleaserTest, FlushWithAnEmptyBufferStillRunsPostRoll)
{
  auto releaser = makeReleaser(3.0, 0.0);

  releaser.onFlush("incident-1", at(0.0));
  EXPECT_TRUE(published_.empty());
  EXPECT_EQ(releaser.state(), IncidentState::PostRoll);

  releaser.offer("{\"a\":\"post\"}", at(1.0));
  ASSERT_EQ(published_.size(), 1u);
  EXPECT_EQ(published_[0].incident_id, "incident-1");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
