// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "dc_measurements/record_enricher.hpp"

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

RecordEnricher makeEnricher(RecordEnricher::Config config = {}, std::vector<std::string>* parse_errors = nullptr,
                            std::vector<json>* validated = nullptr)
{
  RecordEnricher::ValidateFn validate;
  if (validated != nullptr)
  {
    validate = [validated](const json& record) { validated->push_back(record); };
  }
  return RecordEnricher(config, validate, [parse_errors](const std::string& data) {
    if (parse_errors != nullptr)
    {
      parse_errors->push_back(data);
    }
  });
}

json parse(const std::string& data)
{
  return json::parse(data);
}

}  // namespace

TEST(RecordEnricherTest, WithNoShapingConfiguredOnlyTheMarkersAreAdded)
{
  const json record = parse(makeEnricher(config()).enrich("{\"zone\":\"cpu\",\"temp\":45.1}"));
  EXPECT_EQ(record, (json{ { "zone", "cpu" }, { "temp", 45.1 }, { "flattened", false }, { "nested", false } }));
}

TEST(RecordEnricherTest, NestingPutsTheRecordUnderTheMeasurementName)
{
  auto cfg = config();
  cfg.nested = true;
  // flattenSample() marks every Record with both flags, whatever the settings.
  EXPECT_EQ(parse(makeEnricher(cfg).enrich("{\"a\":1}")),
            (json{ { "camera", { { "a", 1 } } }, { "flattened", false }, { "nested", true } }));
}

TEST(RecordEnricherTest, FlatteningRewritesKeysAsPointersAndMarksBothFlags)
{
  auto cfg = config();
  cfg.flatten = true;
  EXPECT_EQ(parse(makeEnricher(cfg).enrich("{\"a\":1,\"b\":{\"c\":2}}")),
            (json{ { "/a", 1 }, { "/b/c", 2 }, { "flattened", true }, { "nested", false } }));
}

// Nesting runs first, so a nested+flattened Record's pointers carry the measurement name.
TEST(RecordEnricherTest, NestedAndFlattenedPointersCarryTheMeasurementName)
{
  auto cfg = config();
  cfg.nested = true;
  cfg.flatten = true;
  EXPECT_EQ(parse(makeEnricher(cfg).enrich("{\"a\":1}")),
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
  EXPECT_EQ(parse(makeEnricher(cfg).enrich("{\"a\":1}")),
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
  const json record = parse(makeEnricher(cfg).enrich("{\"a\":1}"));
  EXPECT_EQ(record.find("run_id"), record.end());
  EXPECT_EQ(record.find("tags"), record.end());
  EXPECT_EQ(record.find("plugin"), record.end());
  EXPECT_EQ(record["name"], "camera");
}

TEST(RecordEnricherTest, CustomKeysFillMissingKeysAndDeclareThemselves)
{
  auto cfg = config();
  cfg.custom_keys = { json{ { "key", "site" }, { "value", "north" }, { "override", false } } };
  const json record = parse(makeEnricher(cfg).enrich("{\"a\":1}"));
  EXPECT_EQ(record["site"], "north");
  EXPECT_EQ(record["custom_keys"], json::array({ "site" }));
}

TEST(RecordEnricherTest, ExistingKeyKeepsItsValueUnlessItsEntrySaysOverride)
{
  auto cfg = config();
  cfg.custom_keys = { json{ { "key", "a" }, { "value", "forced" }, { "override", false } },
                      json{ { "key", "b" }, { "value", "replaced" }, { "override", true } } };
  const json record = parse(makeEnricher(cfg).enrich("{\"a\":\"original\",\"b\":\"original\"}"));
  EXPECT_EQ(record["a"], "original");
  EXPECT_EQ(record["b"], "replaced");
  EXPECT_EQ(record["custom_keys"], json::array({ "a", "b" }));
}

TEST(RecordEnricherTest, CustomKeysRunLastOnTheShapedRecord)
{
  auto cfg = config();
  cfg.flatten = true;
  cfg.custom_keys = { json{ { "key", "site" }, { "value", "north" }, { "override", false } } };
  // The step runs after flattening, so unlike the collected fields a custom key is a plain
  // top-level key, not a JSON pointer.
  EXPECT_EQ(parse(makeEnricher(cfg).enrich("{\"a\":1}")), (json{ { "/a", 1 },
                                                                 { "site", "north" },
                                                                 { "custom_keys", json::array({ "site" }) },
                                                                 { "flattened", true },
                                                                 { "nested", false } }));
}

TEST(RecordEnricherTest, ValidationSeesTheParsedRecordBeforeAnyShaping)
{
  auto cfg = config();
  cfg.nested = true;
  cfg.include_measurement_name = true;
  cfg.enable_validator = true;
  std::vector<json> validated;
  const json record = parse(makeEnricher(cfg, nullptr, &validated).enrich("{\"a\":1}"));
  ASSERT_EQ(validated.size(), 1u);
  // Exactly what was collected: no "name", no markers -- the shaping has not happened yet.
  EXPECT_EQ(validated.front(), (json{ { "a", 1 } }));
  EXPECT_EQ(record["name"], "camera");
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
  std::vector<std::string> parse_errors;
  RecordEnricher enricher(cfg, report_rejection,
                          [&parse_errors](const std::string& data) { parse_errors.push_back(data); });
  const json record = parse(enricher.enrich("{\"a\":1}"));
  EXPECT_TRUE(rejected);
  EXPECT_EQ(record["name"], "camera");
  EXPECT_TRUE(parse_errors.empty());
}

TEST(RecordEnricherTest, DataThatIsNotJsonComesBackUnchangedWithOneParseError)
{
  std::vector<std::string> parse_errors;
  const std::string data = "not json at all";
  EXPECT_EQ(makeEnricher(config(), &parse_errors).enrich(data), data);
  ASSERT_EQ(parse_errors.size(), 1u);
  EXPECT_EQ(parse_errors.front(), data);
}

TEST(RecordEnricherTest, UnparsableDataWithCustomKeysIsRebuiltAsTheOldChainRebuiltIt)
{
  auto cfg = config();
  cfg.custom_keys = { json{ { "key", "site" }, { "value", "north" }, { "override", false } } };
  std::vector<std::string> parse_errors;
  // The per-step chain left the data alone and let the last step (custom keys) rebuild it from
  // an empty object: the keys themselves plus their declaration, nothing else. The one-parse
  // pipeline keeps that output byte-for-byte.
  EXPECT_EQ(parse(makeEnricher(cfg, &parse_errors).enrich("not json")),
            (json{ { "site", "north" }, { "custom_keys", json::array({ "site" }) } }));
  EXPECT_EQ(parse_errors.size(), 1u);
}

TEST(RecordEnricherTest, DumpsCompactAndEnsureAsciiLikeTheStackAlwaysHas)
{
  const std::string out = makeEnricher(config()).enrich("{\"city\":\"é\"}");
  EXPECT_EQ(out, "{\"city\":\"\\u00e9\",\"flattened\":false,\"nested\":false}");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
