// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "dc_measurements/record_enricher.hpp"
#include "measurement_test_bench.hpp"

using dc_measurements::RecordEnricher;
using json = nlohmann::json;

namespace
{

RecordEnricher::Config config()
{
  RecordEnricher::Config config;
  config.measurement_name = "camera";
  config.measurement_plugin = "dc_measurements/Camera";
  return config;
}

RecordEnricher makeEnricher(RecordEnricher::Config config = {}, std::vector<json>* validated = nullptr)
{
  RecordEnricher::ValidateFn validate;
  if (validated != nullptr)
  {
    validate = [validated](const json& record) { validated->push_back(record); };
  }
  return RecordEnricher(config, validate);
}

json record(const char* literal)
{
  return json::parse(literal);
}

}  // namespace

TEST(RecordEnricherTest, WithNoShapingConfiguredOnlyTheMarkersAreAdded)
{
  EXPECT_EQ(makeEnricher(config()).enrich(record(R"({"zone":"cpu","temp":45.1})")),
            (json{ { "zone", "cpu" }, { "temp", 45.1 }, { "flattened", false }, { "nested", false } }));
}

TEST(RecordEnricherTest, NestingPutsTheRecordUnderTheMeasurementName)
{
  auto cfg = config();
  cfg.nested = true;
  // flattenSample() marks every Record with both flags, whatever the settings.
  EXPECT_EQ(makeEnricher(cfg).enrich(record(R"({"a":1})")),
            (json{ { "camera", { { "a", 1 } } }, { "flattened", false }, { "nested", true } }));
}

TEST(RecordEnricherTest, FlatteningRewritesKeysAsPointersAndMarksBothFlags)
{
  auto cfg = config();
  cfg.flatten = true;
  EXPECT_EQ(makeEnricher(cfg).enrich(record(R"({"a":1,"b":{"c":2}})")),
            (json{ { "/a", 1 }, { "/b/c", 2 }, { "flattened", true }, { "nested", false } }));
}

// Nesting runs first, so a nested+flattened Record's pointers carry the measurement name.
TEST(RecordEnricherTest, NestedAndFlattenedPointersCarryTheMeasurementName)
{
  auto cfg = config();
  cfg.nested = true;
  cfg.flatten = true;
  EXPECT_EQ(makeEnricher(cfg).enrich(record(R"({"a":1})")),
            (json{ { "/camera/a", 1 }, { "flattened", true }, { "nested", true } }));
}

TEST(RecordEnricherTest, RunIdTagsNameAndPluginLandOnTheTopLevel)
{
  auto cfg = config();
  cfg.nested = true;
  cfg.run_id_enabled = true;
  cfg.run_id = "run-1";
  cfg.tags = { "inspection", "line-3" };
  cfg.include_measurement_name = true;
  cfg.include_measurement_plugin = true;
  EXPECT_EQ(makeEnricher(cfg).enrich(record(R"({"a":1})")),
            (json{ { "camera", { { "a", 1 } } },
                   { "flattened", false },
                   { "name", "camera" },
                   { "nested", true },
                   { "plugin", "dc_measurements/Camera" },
                   { "run_id", "run-1" },
                   { "tags", json::array({ "inspection", "line-3" }) } }));
}

TEST(RecordEnricherTest, NoRunIdNoTagsWhenDisabledOrEmpty)
{
  auto cfg = config();
  cfg.include_measurement_name = true;
  const json shaped = makeEnricher(cfg).enrich(record(R"({"a":1})"));
  EXPECT_EQ(shaped.find("run_id"), shaped.end());
  EXPECT_EQ(shaped.find("tags"), shaped.end());
  EXPECT_EQ(shaped.find("plugin"), shaped.end());
  EXPECT_EQ(shaped["name"], "camera");
}

TEST(RecordEnricherTest, CustomKeysFillMissingKeysAndDeclareThemselves)
{
  auto cfg = config();
  cfg.custom_keys = { json{ { "key", "site" }, { "value", "north" }, { "override", false } } };
  const json shaped = makeEnricher(cfg).enrich(record(R"({"a":1})"));
  EXPECT_EQ(shaped["site"], "north");
  EXPECT_EQ(shaped["custom_keys"], json::array({ "site" }));
}

TEST(RecordEnricherTest, ExistingKeyKeepsItsValueUnlessItsEntrySaysOverride)
{
  auto cfg = config();
  cfg.custom_keys = { json{ { "key", "a" }, { "value", "forced" }, { "override", false } },
                      json{ { "key", "b" }, { "value", "replaced" }, { "override", true } } };
  const json shaped = makeEnricher(cfg).enrich(record(R"({"a":"original","b":"original"})"));
  EXPECT_EQ(shaped["a"], "original");
  EXPECT_EQ(shaped["b"], "replaced");
  EXPECT_EQ(shaped["custom_keys"], json::array({ "a", "b" }));
}

TEST(RecordEnricherTest, CustomKeysRunLastOnTheShapedRecord)
{
  auto cfg = config();
  cfg.flatten = true;
  cfg.custom_keys = { json{ { "key", "site" }, { "value", "north" }, { "override", false } } };
  // The step runs after flattening, so unlike the collected fields a custom key is a plain
  // top-level key, not a JSON pointer.
  EXPECT_EQ(makeEnricher(cfg).enrich(record(R"({"a":1})")), (json{ { "/a", 1 },
                                                                   { "site", "north" },
                                                                   { "custom_keys", json::array({ "site" }) },
                                                                   { "flattened", true },
                                                                   { "nested", false } }));
}

TEST(RecordEnricherTest, ValidationSeesTheCollectedRecordBeforeAnyShaping)
{
  auto cfg = config();
  cfg.nested = true;
  cfg.include_measurement_name = true;
  cfg.enable_validator = true;
  std::vector<json> validated;
  const json shaped = makeEnricher(cfg, &validated).enrich(record(R"({"a":1})"));
  ASSERT_EQ(validated.size(), 1u);
  // Exactly what was collected: no "name", no markers -- the shaping has not happened yet.
  EXPECT_EQ(validated.front(), (json{ { "a", 1 } }));
  EXPECT_EQ(shaped["name"], "camera");
}

TEST(RecordEnricherTest, ARecordTheValidatorRejectedIsStillShapedAndPublished)
{
  auto cfg = config();
  cfg.include_measurement_name = true;
  cfg.enable_validator = true;
  // The injected callback owns the failure: it reports and returns, like the Measurement's
  // validateJSON() logs and calls the plugin hook without stopping publication.
  bool rejected = false;
  RecordEnricher::ValidateFn report_rejection = [&rejected](const json&) { rejected = true; };
  RecordEnricher enricher(cfg, report_rejection);
  const json shaped = enricher.enrich(record(R"({"a":1})"));
  EXPECT_TRUE(rejected);
  EXPECT_EQ(shaped["name"], "camera");
}

DC_MEASUREMENT_TEST_MAIN()
