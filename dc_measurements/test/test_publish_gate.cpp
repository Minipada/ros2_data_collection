// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include "dc_measurements/publish_gate.hpp"

using dc_measurements::PublishGate;

// The parameter defaults a Measurement reads (init_max 0, condition_max 0, no conditions, no gate)
// publish every collection.
TEST(PublishGateTest, DefaultConfigPublishesEveryCollection)
{
  PublishGate gate;

  EXPECT_TRUE(gate.gateOpen());
  EXPECT_TRUE(gate.offer(false));
  EXPECT_TRUE(gate.offer(true));
  EXPECT_EQ(gate.initCounterPublished(), 2);
  EXPECT_FALSE(gate.collectionFinished());
}

TEST(PublishGateTest, InitQuotaPublishesTheFirstNUnconditionally)
{
  PublishGate::Config config;
  config.init_max = 3;
  config.condition_max = 0;
  config.has_conditions = true;
  PublishGate gate(config);

  // While the quota lasts, the condition is not consulted at all.
  EXPECT_TRUE(gate.offer(false));
  EXPECT_TRUE(gate.offer(true));
  EXPECT_TRUE(gate.offer(false));
  EXPECT_EQ(gate.initCounterPublished(), 3);
  EXPECT_EQ(gate.conditionCounterPublished(), 0);
}

TEST(PublishGateTest, AfterTheQuotaTheConditionDecides)
{
  PublishGate::Config config;
  config.init_max = 1;
  config.condition_max = 0;
  config.has_conditions = true;
  PublishGate gate(config);

  EXPECT_TRUE(gate.offer(false));
  EXPECT_FALSE(gate.offer(false));
  EXPECT_TRUE(gate.offer(true));
  EXPECT_EQ(gate.initCounterPublished(), 1);
}

TEST(PublishGateTest, DisabledInitQuotaNeverCounts)
{
  PublishGate::Config config;
  config.init_max = -1;
  config.condition_max = 0;
  config.has_conditions = false;
  PublishGate gate(config);

  EXPECT_FALSE(gate.offer(true));
  EXPECT_EQ(gate.initCounterPublished(), 0);
}

TEST(PublishGateTest, NoConditionConfiguredNeverPublishesAfterTheQuota)
{
  PublishGate::Config config;
  config.init_max = 1;
  config.condition_max = 0;
  config.has_conditions = false;
  PublishGate gate(config);

  EXPECT_TRUE(gate.offer(true));
  EXPECT_FALSE(gate.offer(true));
}

// condition_max 0 = unbounded for as long as the condition reads true.
TEST(PublishGateTest, UnboundedConditionPublishesForAsLongAsItHolds)
{
  PublishGate::Config config;
  config.init_max = -1;
  config.condition_max = 0;
  config.has_conditions = true;
  PublishGate gate(config);

  for (int i = 0; i < 5; ++i)
  {
    EXPECT_TRUE(gate.offer(true));
  }
  EXPECT_FALSE(gate.offer(false));
  EXPECT_EQ(gate.conditionCounterPublished(), 0);
}

// condition_max -1 = never publish on the condition.
TEST(PublishGateTest, DisabledConditionNeverPublishes)
{
  PublishGate::Config config;
  config.init_max = -1;
  config.condition_max = -1;
  config.has_conditions = true;
  PublishGate gate(config);

  EXPECT_FALSE(gate.offer(true));
  EXPECT_FALSE(gate.offer(true));
}

TEST(PublishGateTest, ConditionCapAndItsReset)
{
  PublishGate::Config config;
  config.init_max = -1;
  config.condition_max = 2;
  config.has_conditions = true;
  PublishGate gate(config);

  EXPECT_TRUE(gate.offer(true));
  EXPECT_TRUE(gate.offer(true));
  // Cap reached: the same true stretch publishes nothing more.
  EXPECT_FALSE(gate.offer(true));
  EXPECT_EQ(gate.conditionCounterPublished(), 2);

  // Condition dropped: nothing published and the count starts over.
  EXPECT_FALSE(gate.offer(false));
  EXPECT_EQ(gate.conditionCounterPublished(), 0);

  EXPECT_TRUE(gate.offer(true));
  EXPECT_TRUE(gate.offer(true));
  EXPECT_FALSE(gate.offer(true));
}

TEST(PublishGateTest, QuotaSpentWithNoConditionPublishingEndsCollection)
{
  PublishGate::Config config;
  config.init_max = 3;
  config.condition_max = -1;
  config.has_conditions = true;
  PublishGate gate(config);

  EXPECT_FALSE(gate.collectionFinished());
  EXPECT_TRUE(gate.offer(false));
  EXPECT_TRUE(gate.offer(false));
  EXPECT_FALSE(gate.collectionFinished());
  EXPECT_TRUE(gate.offer(false));
  EXPECT_TRUE(gate.collectionFinished());

  // It stays finished: this is what stops the collect timer.
  EXPECT_FALSE(gate.offer(false));
  EXPECT_TRUE(gate.collectionFinished());
}

TEST(PublishGateTest, UnboundedConditionOrUnspentQuotaNeverEndsCollection)
{
  PublishGate::Config unbounded_condition;
  unbounded_condition.init_max = 3;
  unbounded_condition.condition_max = 0;
  unbounded_condition.has_conditions = true;
  PublishGate with_condition(unbounded_condition);
  for (int i = 0; i < 5; ++i)
  {
    with_condition.offer(true);
  }
  EXPECT_FALSE(with_condition.collectionFinished());

  PublishGate::Config infinite_init;
  PublishGate no_quota(infinite_init);
  for (int i = 0; i < 5; ++i)
  {
    no_quota.offer(true);
  }
  EXPECT_FALSE(no_quota.collectionFinished());
}

TEST(PublishGateTest, ClosedGateDropsCollectionsWithoutSpendingCounters)
{
  PublishGate::Config config;
  config.init_max = 3;
  config.condition_max = -1;
  config.gate_enabled = true;
  PublishGate gate(config);

  EXPECT_FALSE(gate.gateOpen());
  EXPECT_FALSE(gate.offer(true));
  EXPECT_FALSE(gate.offer(false));
  EXPECT_EQ(gate.initCounterPublished(), 0);
  EXPECT_FALSE(gate.collectionFinished());
}

TEST(PublishGateTest, GateOpensOnceAndNeverRecloses)
{
  PublishGate::Config config;
  config.gate_enabled = true;
  PublishGate gate(config);

  EXPECT_FALSE(gate.openGate(false));
  EXPECT_FALSE(gate.gateOpen());

  EXPECT_TRUE(gate.openGate(true));
  EXPECT_TRUE(gate.gateOpen());

  // A later false reading changes nothing: the gate never re-closes.
  EXPECT_TRUE(gate.openGate(false));
  EXPECT_TRUE(gate.openGate(true));
}

TEST(PublishGateTest, NoGateConfiguredIsAlwaysOpen)
{
  PublishGate gate;

  EXPECT_TRUE(gate.gateOpen());
  EXPECT_TRUE(gate.openGate(false));
}

TEST(PublishGateTest, OpenedGatePublishesUnderTheQuotaLikeNoGateAtAll)
{
  PublishGate::Config config;
  config.init_max = 2;
  config.condition_max = -1;
  config.gate_enabled = true;
  PublishGate gate(config);

  EXPECT_FALSE(gate.offer(true));
  gate.openGate(true);
  EXPECT_TRUE(gate.offer(true));
  EXPECT_TRUE(gate.offer(true));
  EXPECT_FALSE(gate.offer(true));
  EXPECT_TRUE(gate.collectionFinished());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
