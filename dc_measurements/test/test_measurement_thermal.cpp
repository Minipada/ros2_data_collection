// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json-schema.hpp>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "measurement_test_bench.hpp"

// Runs a Record through the Measurement's own installed schema, the same file and validator
// `Measurement::validateJSON()` loads when `enable_validator` is on.
class ThermalSchema
{
public:
  ThermalSchema()
  {
    std::ifstream schema_file(ament_index_cpp::get_package_share_directory("dc_measurements") +
                              "/plugins/measurements/json/thermal.json");
    validator_.set_root_schema(nlohmann::json::parse(schema_file));
  }

  bool accepts(const nlohmann::json& record)
  {
    try
    {
      validator_.validate(record);
      return true;
    }
    catch (const std::exception&)
    {
      return false;
    }
  }

private:
  nlohmann::json_schema::json_validator validator_;
};

// Builds a fake /sys/class/thermal-shaped directory tree under /tmp so the plugin can be
// exercised for real (auto-discovery, zone `type` as Record key, ARM-style non-numeric-suffix
// naming) without depending on whatever thermal zones (if any) the test host/container exposes.
class FakeThermalTree
{
public:
  FakeThermalTree()
  {
    root_ = std::filesystem::temp_directory_path() / ("dc_test_thermal_" + std::to_string(::getpid()));
    std::filesystem::create_directories(root_);
  }

  ~FakeThermalTree()
  {
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
  }

  void addZone(const std::string& zone_dir_name, const std::string& type, long temp_millidegrees)
  {
    std::filesystem::path zone_path = root_ / zone_dir_name;
    std::filesystem::create_directories(zone_path);

    std::ofstream type_file(zone_path / "type");
    type_file << type << "\n";

    std::ofstream temp_file(zone_path / "temp");
    temp_file << temp_millidegrees << "\n";
  }

  std::string path() const
  {
    return root_.string();
  }

private:
  std::filesystem::path root_;
};

class MeasurementThermalTest : public MeasurementBench
{
protected:
  MeasurementThermalTest() : MeasurementBench("thermal")
  {
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("thermal.plugin", std::string("dc_measurements/Thermal"));
    ms_node_->declare_parameter("thermal.group_key", std::string("thermal"));
    ms_node_->declare_parameter("thermal.topic_output", std::string("/dc/measurement/thermal"));
  }

  void spinUntilCallback()
  {
    ASSERT_TRUE(spinUntil([this] { return callback_active_; }, 10000)) << "No Record received within the timeout";
  }
};

TEST_F(MeasurementThermalTest, AutoDiscoversZonesKeyedByType)
{
  FakeThermalTree tree;
  // Non-sequential, non-x86-style zone directory names (ARM platforms often skip indices or
  // name them differently) -- acceptance criterion: don't hardcode `thermal_zone0`.
  tree.addZone("thermal_zone0", "cpu-thermal", 45123);
  tree.addZone("thermal_zone2", "gpu-thermal", 52000);

  declareCommonParameters();
  ms_node_->declare_parameter("thermal.base_path", tree.path());

  startLifecycleNode();
  spinUntilCallback();

  EXPECT_DOUBLE_EQ(data_json_["cpu-thermal"].get<double>(), 45.123);
  EXPECT_DOUBLE_EQ(data_json_["gpu-thermal"].get<double>(), 52.0);
}

TEST_F(MeasurementThermalTest, ExplicitZoneListOverridesAutoDiscovery)
{
  FakeThermalTree tree;
  tree.addZone("thermal_zone0", "cpu-thermal", 40000);
  tree.addZone("thermal_zone1", "board-thermal", 30000);

  declareCommonParameters();
  ms_node_->declare_parameter("thermal.base_path", tree.path());
  ms_node_->declare_parameter("thermal.zones", std::vector<std::string>{ "thermal_zone1" });

  startLifecycleNode();
  spinUntilCallback();

  EXPECT_FALSE(data_json_.contains("cpu-thermal"));
  EXPECT_DOUBLE_EQ(data_json_["board-thermal"].get<double>(), 30.0);
}

TEST_F(MeasurementThermalTest, ActivatesSuccessfullyWithMissingBasePath)
{
  declareCommonParameters();
  ms_node_->declare_parameter("thermal.base_path", std::string("/tmp/dc_test_thermal_does_not_exist"));

  // Must not throw: a missing/unreadable /sys/class/thermal should not fail activation. No
  // zone can be read, so (like SerialInterface with no port) nothing is published either --
  // this just proves the node keeps spinning without crashing.
  ASSERT_NO_THROW(startLifecycleNode());

  spinFor(100);

  EXPECT_FALSE(callback_active_);
  SUCCEED();
}

TEST(ThermalSchemaTest, AcceptsARepresentativeRecord)
{
  ThermalSchema schema;
  EXPECT_TRUE(schema.accepts(nlohmann::json{ { "x86_pkg_temp", 52.0 }, { "gpu-thermal", 61.5 } }));
}

TEST(ThermalSchemaTest, RejectsARecordWithNoZoneEntry)
{
  // Zone type strings *are* the field names, so the entry itself is what's required: a Record
  // with none carries no reading at all. The Measurement never emits one (it publishes nothing
  // that cycle instead), so an empty Record reaching a Destination means something went wrong.
  ThermalSchema schema;
  EXPECT_FALSE(schema.accepts(nlohmann::json::object()));
}

TEST(ThermalSchemaTest, RejectsMalformedZoneEntries)
{
  ThermalSchema schema;
  EXPECT_FALSE(schema.accepts(nlohmann::json{ { "cpu-thermal", "45.1" } }));
  EXPECT_FALSE(schema.accepts(nlohmann::json{ { "cpu-thermal", -400.0 } }));
  EXPECT_FALSE(schema.accepts(nlohmann::json{ { "", 45.1 } }));
}

TEST_F(MeasurementThermalTest, PublishedRecordValidatesAgainstTheSchema)
{
  FakeThermalTree tree;
  tree.addZone("thermal_zone0", "x86_pkg_temp", 52000);

  declareCommonParameters();
  ms_node_->declare_parameter("thermal.base_path", tree.path());

  startLifecycleNode();
  spinUntilCallback();

  // publish() enriches the Record *after* validateJSON() has run, so strip what the framework
  // added to get back the Record the validator actually saw.
  nlohmann::json record = data_json_;
  for (const char* enrichment_key : { "name", "plugin", "nested", "flattened", "run_id", "tags" })
  {
    record.erase(enrichment_key);
  }

  EXPECT_TRUE(ThermalSchema().accepts(record));
}

TEST_F(MeasurementThermalTest, PublishesTheSameRecordWithTheValidatorOff)
{
  FakeThermalTree tree;
  tree.addZone("thermal_zone0", "cpu-thermal", 45123);

  declareCommonParameters();
  ms_node_->declare_parameter("thermal.base_path", tree.path());
  ms_node_->declare_parameter("thermal.enable_validator", false);

  startLifecycleNode();
  spinUntilCallback();

  EXPECT_DOUBLE_EQ(data_json_["cpu-thermal"].get<double>(), 45.123);
}

DC_MEASUREMENT_TEST_MAIN()
