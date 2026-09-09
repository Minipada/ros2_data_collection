// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <nlohmann/json-schema.hpp>

#include "measurement_test_bench.hpp"

class MeasurementFastddsStatsTest : public MeasurementBench
{
protected:
  MeasurementFastddsStatsTest() : MeasurementBench("fastdds_stats")
  {
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("fastdds_stats.plugin", std::string("dc_measurements/FastddsStats"));
    ms_node_->declare_parameter("fastdds_stats.group_key", std::string("fastdds_stats"));
    ms_node_->declare_parameter("fastdds_stats.topic_output", std::string("/dc/measurement/fastdds_stats"));
    // A domain unlikely to collide with any other DDS traffic on the test host/CI runner.
    ms_node_->declare_parameter("fastdds_stats.domain_id", 221);
    ms_node_->declare_parameter("fastdds_stats.polling_interval", 100);
    ms_node_->declare_parameter("fastdds_stats.init_collect", false);
  }

  // The schema the plugin itself validates against, applied here directly so a Record that only
  // half fills it fails the test rather than only logging.
  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string path = ament_index_cpp::get_package_share_directory("dc_measurements") +
                             "/plugins/measurements/json/fastdds_stats.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }
};

TEST_F(MeasurementFastddsStatsTest, ReportsASampleOnThePollingIntervalEvenWithNoOtherParticipants)
{
  declareCommonParameters();
  startLifecycleNode();

  // Nothing else is running on domain 221 for this test, so every count is expected at zero --
  // the point is that a sample is still emitted and still validates, the same way `uptime`
  // always reports something on every poll regardless of what else is going on.
  ASSERT_TRUE(spinUntil([this] { return !records_.empty(); }, 5000)) << "no Record ever arrived";
  const auto record = records_.front();

  EXPECT_EQ(record["event"], "sample");
  EXPECT_EQ(record["domain_id"].get<int>(), 221);
  EXPECT_GE(record["participant_count"].get<int>(), 0);
  EXPECT_GE(record["datawriter_count"].get<int>(), 0);
  EXPECT_GE(record["datareader_count"].get<int>(), 0);
  expectValidatesAgainstSchema(record);
}

DC_MEASUREMENT_TEST_MAIN()
