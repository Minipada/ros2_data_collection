// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Gold-file comparisons parse both the rendered output and the checked-in fixture and
// compare toml++ tables (order/format insensitive) rather than raw text.
#include "dc_bridge/render.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <toml++/toml.hpp>

using namespace dc_bridge;

namespace
{

std::string fixtures_dir()
{
  // Set by CMake (add_definitions) to the test/fixtures/render dir.
  return std::string(DC_BRIDGE_FIXTURES_DIR);
}

std::string read_fixture(const std::string& name)
{
  std::ifstream in(fixtures_dir() + "/" + name);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

Destination make_destination(const std::string& name, std::vector<std::string> inputs, TimeFormat tf,
                             DestinationKind kind)
{
  Destination d;
  d.name = name;
  d.receives = Receives::Records;
  d.inputs = std::move(inputs);
  d.time_key = "date";
  d.time_format = tf;
  d.kind = std::move(kind);
  return d;
}

FileParams file_kind()
{
  return FileParams{ "/var/log/dc/records.log" };
}

VectorParams vector_kind()
{
  return VectorParams{ "dc-e2e-limits-agg", 6000 };
}

Destination files_destination(const std::string& name)
{
  Destination d = make_destination(name, {}, TimeFormat::EpochNanos, S3Params{});
  d.receives = Receives::Files;
  return d;
}

SocketSinkParams mcap_like_sink()
{
  SocketSinkParams params;
  params.sink_id = "dc_mcap_writer";
  params.input_topics = { "/dc/measurement/uptime", "/dc/group/robot" };
  params.host = "127.0.0.1";
  params.port = 9191;
  return params;
}

RenderConfig config_with(std::vector<Destination> dests)
{
  RenderConfig c;
  c.forward_host = "127.0.0.1";
  c.forward_port = 24224;
  c.data_dir = "/home/dc/.dc/buffer";
  c.buffer_max_bytes = MIN_DISK_BUFFER_BYTES;
  c.destinations = std::move(dests);
  return c;
}

RenderConfig basic_config(TimeFormat tf)
{
  return config_with({ make_destination("records_log", { "/dc/group/robot" }, tf, file_kind()) });
}

void assert_matches_fixture(const std::string& rendered, const std::string& fixture_name)
{
  toml::table actual = toml::parse(rendered);
  toml::table expected = toml::parse(read_fixture(fixture_name));
  EXPECT_TRUE(actual == expected) << "rendered config did not match fixture " << fixture_name << "\n--- rendered ---\n"
                                  << rendered << "\n--- expected ---\n"
                                  << read_fixture(fixture_name);
}

}  // namespace

TEST(Render, SingleFileDestinationDouble)
{
  assert_matches_fixture(render(basic_config(TimeFormat::Double)), "basic_double.toml");
}

TEST(Render, Iso8601FormatTimestamp)
{
  assert_matches_fixture(render(basic_config(TimeFormat::Iso8601)), "iso8601.toml");
}

TEST(Render, MultipleDestinationsSharePerTagRoutes)
{
  auto config = config_with({
      make_destination("records_log", { "/dc/group/robot" }, TimeFormat::Double, file_kind()),
      make_destination("to_aggregator", { "/dc/measurement/uptime" }, TimeFormat::Iso8601, vector_kind()),
  });
  assert_matches_fixture(render(config), "multiple_destinations.toml");
}

// #472: postgres/s3(records)/console retired their blessed Vector-sink templating —
// `file` and `vector` are the only `receives: records` sinks render_sink() still knows
// how to build, and coexist fine in one config.
TEST(Render, FileAndVectorDestinationsInOneConfig)
{
  auto config = config_with({
      make_destination("local_log", { "/dc/group/robot" }, TimeFormat::Double,
                       FileParams{ "/var/log/dc/records-%Y-%m-%d.log" }),
      make_destination("to_aggregator", { "/dc/group/robot" }, TimeFormat::Double, vector_kind()),
  });
  assert_matches_fixture(render(config), "file_and_vector.toml");
}

// #443: `vector` forwards to another Shipper (typically an edge aggregator) over
// Vector's own native inter-instance protocol — `address` is `host:port`, and it gets
// the same disk buffer as every other blessed sink.
TEST(Render, VectorDestinationForwardsToAggregator)
{
  auto config =
      config_with({ make_destination("to_aggregator", { "/dc/group/robot" }, TimeFormat::Double, vector_kind()) });
  assert_matches_fixture(render(config), "vector_destination.toml");
}

// #472: `S3Params` stays a valid DestinationKind (object storage for `receives: files`,
// ADR-0005), but render_sink() no longer templates a Vector `aws_s3` sink for it — that
// was the blessed-s3-for-records feature this issue removes. A `receives: files`
// destination itself never reaches render() (validate() rejects it, see
// RejectsFilesDestinationReachingShipperConfig below); this covers the other way an
// S3Params destination could reach render_sink() — a `receives: records` one, which
// destination_from_raw itself can no longer produce (`type: s3` + `receives: records` is
// UnsupportedType), but render()'s pure API still accepts directly.
TEST(Render, RecordsDestinationWithS3KindIsRejectedByRenderSink)
{
  S3Params s3;
  s3.bucket = "dc-records";
  auto config = config_with({ make_destination("rustfs", { "/dc/group/robot" }, TimeFormat::Double, s3) });
  try
  {
    render(config);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::UnexpectedDestinationKind);
    EXPECT_EQ(e.arg0(), "rustfs");
  }
}

// #472: incident_id normalization was postgres-specific column-mapping support; with the
// blessed postgres sink gone, no destination kind gets it any more — the field rides
// along in the JSON payload as it always has, for every kind.
TEST(Render, NormalizeNeverEmitsIncidentIdHandling)
{
  auto config = config_with({
      make_destination("local_log", { "/dc/group/robot" }, TimeFormat::EpochNanos, file_kind()),
      make_destination("to_aggregator", { "/dc/measurement/uptime" }, TimeFormat::EpochNanos, vector_kind()),
  });
  toml::table parsed = toml::parse(render(config));
  const std::string source(parsed["transforms"][NORMALIZE_TRANSFORM_ID]["source"].value_or(""));
  EXPECT_EQ(source.find("incident_id"), std::string::npos) << source;
}

// #266: the Forwarder tags every frame with a chunk id and depends on Vector actually
// acking it, so acknowledgements must be switched on in the rendered config — global,
// not per-source, since the source-level knob is deprecated in the pinned Vector version
// (verified against the real binary; see the destinations.md delivery-guarantees note).
TEST(Render, EnablesGlobalAcknowledgements)
{
  auto config = basic_config(TimeFormat::Double);
  toml::table parsed = toml::parse(render(config));
  auto* acks = parsed["acknowledgements"].as_table();
  ASSERT_NE(acks, nullptr);
  EXPECT_EQ(acks->at("enabled").as_boolean()->get(), true);
}

TEST(Render, RoutesArePerTagAndSinksConsumeDcDotTag)
{
  auto config = config_with({
      make_destination("records_log", { "/dc/group/robot", "/dc/measurement/uptime" }, TimeFormat::Double, file_kind()),
      make_destination("to_aggregator", { "/dc/measurement/uptime" }, TimeFormat::Double, vector_kind()),
  });
  toml::table parsed = toml::parse(render(config));

  auto* route = parsed["transforms"]["dc"]["route"].as_table();
  ASSERT_NE(route, nullptr);
  EXPECT_TRUE(route->contains("dc.group.robot"));
  EXPECT_TRUE(route->contains("dc.measurement.uptime"));
  EXPECT_EQ(route->size(), 2u);

  auto records_log_inputs = parsed["sinks"]["records_log"]["inputs"].as_array();
  ASSERT_NE(records_log_inputs, nullptr);
  EXPECT_EQ((*records_log_inputs)[0].value<std::string>(), "dc.dc.group.robot");
  EXPECT_EQ((*records_log_inputs)[1].value<std::string>(), "dc.dc.measurement.uptime");
}

TEST(Render, RouteOutputForTagIsPublicName)
{
  EXPECT_EQ(route_output_for_tag("dc.measurement.uptime"), "dc.dc.measurement.uptime");
}

// #495: launch code used to re-derive this in Python (`_topic_to_dc_tag_route`) — the
// copy matched for ordinary topics but dropped derive_tag's `/`-alone → "dc" rule, so an
// empty topic silently left the ADR-0003 public route on the launch path. One call owns
// the whole derivation now; these are the same cases topic_config's own tests pin.
TEST(Render, RouteOutputForTopicMatchesTheBridgeTagDerivation)
{
  EXPECT_EQ(route_output_for_topic("/dc/measurement/cpu"), "dc.dc.measurement.cpu");
  EXPECT_EQ(route_output_for_topic("dc/measurement/cpu"), "dc.dc.measurement.cpu");
  EXPECT_EQ(route_output_for_topic("/dc/group/robot"), "dc.dc.group.robot");
}

TEST(Render, RouteOutputForTopicEmptyTopicRoutesToDc)
{
  // The half the Python copy lost: `/` alone (or nothing at all) derives the Tag `dc`,
  // so its route is the route transform's own id followed by that Tag.
  EXPECT_EQ(route_output_for_topic(""), "dc.dc");
  EXPECT_EQ(route_output_for_topic("/"), "dc.dc");
}

// #495/#210: the passthrough socket sink ADR-0009 records — the block launch used to
// hand-write in Python, disk-buffer floor literal included.
TEST(Render, SocketSinkRendersRoutesAndTheMinDiskBuffer)
{
  auto params = mcap_like_sink();
  params.origin = "Generated by dc_bringup.launch.py (ADR-0009, #210)";
  const std::string toml = socket_sink_toml(params);

  EXPECT_NE(toml.find("# Generated by dc_bringup.launch.py (ADR-0009, #210)"), std::string::npos) << toml;

  toml::table parsed = toml::parse(toml);
  auto* sink = parsed["sinks"]["dc_mcap_writer"].as_table();
  ASSERT_NE(sink, nullptr) << toml;
  EXPECT_EQ(sink->at("type").value<std::string>(), "socket");
  EXPECT_EQ(sink->at("mode").value<std::string>(), "tcp");
  EXPECT_EQ(sink->at("address").value<std::string>(), "127.0.0.1:9191");
  EXPECT_EQ(parsed["sinks"]["dc_mcap_writer"]["encoding"]["codec"].value<std::string>(), "json");
  EXPECT_EQ(parsed["sinks"]["dc_mcap_writer"]["framing"]["method"].value<std::string>(), "newline_delimited");

  auto inputs = sink->at("inputs").as_array();
  ASSERT_NE(inputs, nullptr) << toml;
  // Sorted, and each topic through the same route_output_for_topic the Bridge derives.
  ASSERT_EQ(inputs->size(), 2u);
  EXPECT_EQ((*inputs)[0].value<std::string>(), "dc.dc.group.robot");
  EXPECT_EQ((*inputs)[1].value<std::string>(), "dc.dc.measurement.uptime");

  // The literal launch re-typed as 268435488 is Vector's disk-buffer floor, owned here.
  auto* buffer = sink->at("buffer").as_table();
  ASSERT_NE(buffer, nullptr) << toml;
  EXPECT_EQ(buffer->at("type").value<std::string>(), "disk");
  EXPECT_EQ(buffer->at("max_size").value<std::int64_t>(), static_cast<std::int64_t>(MIN_DISK_BUFFER_BYTES));
}

TEST(Render, SocketSinkDeduplicatesRepeatedTopics)
{
  auto params = mcap_like_sink();
  params.input_topics.push_back("/dc/group/robot");
  toml::table parsed = toml::parse(socket_sink_toml(params));
  auto* inputs = parsed["sinks"]["dc_mcap_writer"]["inputs"].as_array();
  ASSERT_NE(inputs, nullptr);
  EXPECT_EQ(inputs->size(), 2u);
}

TEST(Render, SocketSinkRejectsNoTopics)
{
  auto params = mcap_like_sink();
  params.input_topics.clear();
  try
  {
    socket_sink_toml(params);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::EmptyInputs);
    EXPECT_EQ(e.arg0(), "dc_mcap_writer");
  }
}

TEST(Render, SocketSinkRejectsMissingHostAndSinkId)
{
  auto params = mcap_like_sink();
  params.host.clear();
  try
  {
    socket_sink_toml(params);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::MissingField);
    EXPECT_EQ(e.arg1(), "host");
  }

  auto no_id = mcap_like_sink();
  no_id.sink_id.clear();
  try
  {
    socket_sink_toml(no_id);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::MissingField);
    EXPECT_EQ(e.arg1(), "sink_id");
  }
}

TEST(Render, SocketSinkRejectsPortZero)
{
  auto params = mcap_like_sink();
  params.port = 0;
  try
  {
    socket_sink_toml(params);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::InvalidPort);
    EXPECT_EQ(e.number(), 0);
  }
}

// #495: staging is copy-forward — the recipe wins even over an already-staged copy, so a
// corrected recipe reaches an existing deployment instead of the first launch's snapshot
// of it. The old copy-only-if-absent rule was exactly the third row below.
TEST(Render, StagingRuleIsCopyForward)
{
  EXPECT_EQ(stage_action(false, false), StageAction::Skip);
  EXPECT_EQ(stage_action(false, true), StageAction::Skip);
  EXPECT_EQ(stage_action(true, false), StageAction::Copy);
  EXPECT_EQ(stage_action(true, true), StageAction::Copy);
}

TEST(Render, RejectsEmptyDestinations)
{
  try
  {
    render(config_with({}));
    FAIL() << "expected NoDestinations";
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::NoDestinations);
  }
}

TEST(Render, RejectsEmptyDataDir)
{
  auto config = basic_config(TimeFormat::Double);
  config.data_dir = "";
  try
  {
    render(config);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::EmptyDataDir);
  }
}

TEST(Render, RejectsTooSmallDiskBuffer)
{
  auto config = basic_config(TimeFormat::Double);
  config.buffer_max_bytes = 1024;
  try
  {
    render(config);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::BufferTooSmall);
    EXPECT_EQ(e.number(), 1024);
  }
}

TEST(Render, RejectsEmptyInputs)
{
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].inputs.clear();
  try
  {
    render(config);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::EmptyInputs);
    EXPECT_EQ(e.arg0(), "records_log");
  }
}

TEST(Render, RejectsDuplicateDestinationNames)
{
  auto config = basic_config(TimeFormat::Double);
  config.destinations.push_back(config.destinations[0]);
  try
  {
    render(config);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::DuplicateDestination);
    EXPECT_EQ(e.arg0(), "records_log");
  }
}

TEST(Render, RejectsInvalidDestinationNames)
{
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].name = "pg-sql!";
  try
  {
    render(config);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::InvalidDestinationName);
  }
}

TEST(Render, RejectsReservedDestinationNames)
{
  for (const std::string reserved : { "dc", "dc_bridge_in", "dc_bridge_normalize" })
  {
    auto config = basic_config(TimeFormat::Double);
    config.destinations[0].name = reserved;
    try
    {
      render(config);
      FAIL() << "name '" << reserved << "' must be rejected";
    }
    catch (const RenderError& e)
    {
      EXPECT_EQ(e.kind(), RenderErrorKind::ReservedDestinationName);
      EXPECT_EQ(e.arg0(), reserved);
    }
  }
}

TEST(Render, DestinationFromRawRejectsUnsupportedType)
{
  try
  {
    destination_from_raw("archive", "mongodb", "records", { "/dc/group/robot" }, {});
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::UnsupportedType);
    EXPECT_EQ(e.arg0(), "archive");
    EXPECT_EQ(e.arg1(), "mongodb");
  }
}

// #472: postgres/s3(records)/console are gone as blessed `receives: records` types —
// `mongodb`-style rejection above covers those the same as any other unrecognized
// string now. `s3` stays valid, but only for `receives: files` (ADR-0005 object
// storage) — see DestinationFromRawAcceptsFilesS3.
TEST(Render, DestinationFromRawBuildsFileAndVector)
{
  RawDestinationParams fileraw;
  fileraw.path = "/var/log/dc/records.log";
  auto file = destination_from_raw("local_log", "file", "records", { "/dc/group/robot" }, fileraw);
  EXPECT_TRUE(std::holds_alternative<FileParams>(file.kind));

  RawDestinationParams vecraw;
  vecraw.host = "dc-e2e-limits-agg";
  vecraw.port = 6000;
  auto vec = destination_from_raw("to_aggregator", "vector", "records", { "/dc/group/robot" }, vecraw);
  EXPECT_TRUE(std::holds_alternative<VectorParams>(vec.kind));
}

TEST(Render, DestinationFromRawRejectsFilesOnNonObjectStorage)
{
  for (const std::string bad : { "file", "vector" })
  {
    try
    {
      destination_from_raw("archive", bad, "files", { "/dc/measurement/map" }, {});
      FAIL() << "type '" << bad << "' must be rejected for receives: files";
    }
    catch (const RenderError& e)
    {
      EXPECT_EQ(e.kind(), RenderErrorKind::FilesRequireObjectStorage);
      EXPECT_EQ(e.arg1(), bad);
    }
  }
}

TEST(Render, DestinationFromRawAcceptsFilesS3)
{
  RawDestinationParams raw;
  raw.bucket = "dc-files";
  raw.endpoint = "http://127.0.0.1:9000";
  raw.access_key_id = "rustfsadmin";
  raw.secret_access_key = "rustfsadmin";
  auto dest = destination_from_raw("minio", "s3", "files", { "/dc/measurement/camera" }, raw);
  EXPECT_EQ(dest.receives, Receives::Files);
  EXPECT_TRUE(std::holds_alternative<S3Params>(dest.kind));
}

TEST(Render, RejectsFilesDestinationReachingShipperConfig)
{
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].receives = Receives::Files;
  try
  {
    render(config);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::FilesDestinationInShipperConfig);
  }
}

// #505: the intent queue is one store behind one dc_uploader process — the second
// `receives: files` Destination is rejected where Destinations are validated, not only by
// dc_bringup's launch translation.
TEST(Render, RejectsTwoFilesDestinations)
{
  const std::vector<Destination> files{ files_destination("minio_a"), files_destination("minio_b") };
  try
  {
    validate_files_destinations(files);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::TooManyFilesDestinations);
    EXPECT_EQ(e.arg0(), "minio_a, minio_b");
  }
}

TEST(Render, AcceptsZeroOrOneFilesDestination)
{
  EXPECT_NO_THROW(validate_files_destinations({}));
  EXPECT_NO_THROW(validate_files_destinations({ files_destination("minio") }));
}

TEST(Render, ExtraTagsAreRoutedNormalizedAndConsumed)
{
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].extra_tags.push_back("dc.files");
  toml::table parsed = toml::parse(render(config));

  auto* route = parsed["transforms"]["dc"]["route"].as_table();
  EXPECT_TRUE(route->contains("dc.files"));
  EXPECT_TRUE(route->contains("dc.group.robot"));

  auto inputs = parsed["sinks"]["records_log"]["inputs"].as_array();
  EXPECT_EQ((*inputs)[0].value<std::string>(), "dc.dc.files");
  EXPECT_EQ((*inputs)[1].value<std::string>(), "dc.dc.group.robot");

  auto normalize = parsed["transforms"]["dc_bridge_normalize"]["source"].value<std::string>();
  EXPECT_NE(normalize->find("includes([\"dc.files\", \"dc.group.robot\"], .tag)"), std::string::npos);
}

TEST(Render, TagPrefixesRouteAWholeNamespaceWithStartsWith)
{
  // Raw mode (#227): the Tags don't exist yet when this config is rendered — topics are
  // discovered while Vector is already running — so the branch matches the namespace.
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].tag_prefixes.push_back("dc.raw.");
  toml::table parsed = toml::parse(render(config));

  auto* route = parsed["transforms"]["dc"]["route"].as_table();
  ASSERT_TRUE(route->contains("dc.raw"));
  EXPECT_EQ((*route)["dc.raw"]["source"].value<std::string>(), "starts_with(to_string(.tag) ?? \"\", \"dc.raw.\")");
  // The exact-Tag branches are untouched.
  EXPECT_TRUE(route->contains("dc.group.robot"));

  auto inputs = parsed["sinks"]["records_log"]["inputs"].as_array();
  ASSERT_EQ(inputs->size(), 2u);
  EXPECT_EQ((*inputs)[0].value<std::string>(), "dc.dc.group.robot");
  EXPECT_EQ((*inputs)[1].value<std::string>(), "dc.dc.raw");

  // The timestamp normalization has to cover the namespace too, or raw Records would
  // reach the sink with no `date` column.
  auto normalize = parsed["transforms"]["dc_bridge_normalize"]["source"].value<std::string>();
  EXPECT_NE(normalize->find("includes([\"dc.group.robot\"], .tag) || "
                            "starts_with(to_string(.tag) ?? \"\", \"dc.raw.\")"),
            std::string::npos);
}

TEST(Render, DestinationWithOnlyATagPrefixNeedsNoInputs)
{
  // The raw-only Destination a `dc_raw.launch.py` bringup produces: no Measurement
  // topics at all, everything arrives under the namespace.
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].inputs.clear();
  config.destinations[0].tag_prefixes.push_back("dc.raw.");
  ASSERT_NO_THROW(render(config));

  toml::table parsed = toml::parse(render(config));
  auto normalize = parsed["transforms"]["dc_bridge_normalize"]["source"].value<std::string>();
  EXPECT_NE(normalize->find("if starts_with(to_string(.tag) ?? \"\", \"dc.raw.\") {"), std::string::npos);
  EXPECT_EQ(normalize->find("includes("), std::string::npos);
}

TEST(Render, RouteOutputForTagPrefixDropsTheTrailingSeparator)
{
  EXPECT_EQ(route_output_for_tag_prefix("dc.raw."), "dc.dc.raw");
  EXPECT_EQ(route_output_for_tag_prefix("dc.raw"), "dc.dc.raw");
}

TEST(Render, RejectsATagThatCollidesWithARoutedNamespace)
{
  // Topic `/dc/raw` derives Tag `dc.raw`, which is also the branch id the `dc.raw.`
  // namespace claims — one would silently overwrite the other in the route table.
  auto config = config_with({ make_destination("records_log", { "/dc/raw" }, TimeFormat::Double, file_kind()) });
  config.destinations[0].tag_prefixes.push_back("dc.raw.");
  try
  {
    render(config);
    FAIL() << "expected a RenderError";
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::TagPrefixCollidesWithTag);
    EXPECT_EQ(e.arg0(), "dc.raw");
  }
}

TEST(Render, DestinationWithOnlyExtraTagsNeedsNoInputs)
{
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].inputs.clear();
  config.destinations[0].extra_tags.push_back("dc.files");
  EXPECT_NO_THROW(render(config));
}

TEST(Render, DestinationFromRawRejectsInvalidReceives)
{
  try
  {
    destination_from_raw("archive", "file", "bogus", { "/dc/measurement/map" }, {});
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::InvalidReceives);
    EXPECT_EQ(e.arg1(), "bogus");
  }
}

TEST(Render, DestinationFromRawRejectsInvalidTimeFormat)
{
  RawDestinationParams raw;
  raw.time_format = "rfc2822";
  try
  {
    destination_from_raw("misconfigured", "vector", "records", { "/dc/measurement/map" }, raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::InvalidTimeFormat);
    EXPECT_EQ(e.arg1(), "rfc2822");
  }
}

TEST(Render, DestinationFromRawAcceptsEveryTimeFormat)
{
  const std::vector<std::pair<std::string, TimeFormat>> cases = { { "epoch_nanos", TimeFormat::EpochNanos },
                                                                  { "double", TimeFormat::Double },
                                                                  { "iso8601", TimeFormat::Iso8601 } };
  for (const auto& c : cases)
  {
    RawDestinationParams raw;
    raw.time_format = c.first;
    raw.path = "/var/log/dc/records.log";
    auto dest = destination_from_raw("local_log", "file", "records", { "/dc/measurement/map" }, raw);
    EXPECT_EQ(dest.time_format, c.second) << "time_format '" << c.first << "'";
  }
}

// The normalize transform must emit the timestamp with no float arithmetic in it —
// `to_float(...) / 1000000000.0` is precisely what loses resolution below ~1 us.
TEST(Render, EpochNanosNormalizesWithoutFloatRounding)
{
  auto config =
      config_with({ make_destination("records_log", { "/dc/group/robot" }, TimeFormat::EpochNanos, file_kind()) });
  const std::string rendered = render(config);
  EXPECT_NE(rendered.find(".date = to_unix_timestamp!(.timestamp, unit: \"nanoseconds\")"), std::string::npos)
      << rendered;
  EXPECT_EQ(rendered.find("to_float("), std::string::npos)
      << "epoch_nanos must not round through a float: " << rendered;
}

TEST(Render, VectorFromRawRejectsMissingHost)
{
  RawDestinationParams raw;
  raw.port = 6000;
  try
  {
    vector_from_raw("to_aggregator", raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::MissingField);
    EXPECT_EQ(e.arg1(), "host");
  }
}

TEST(Render, VectorFromRawRejectsMissingPort)
{
  RawDestinationParams raw;
  raw.host = "dc-e2e-limits-agg";
  try
  {
    vector_from_raw("to_aggregator", raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::MissingField);
    EXPECT_EQ(e.arg1(), "port");
  }
}

TEST(Render, VectorFromRawRejectsOutOfRangePort)
{
  RawDestinationParams raw;
  raw.host = "dc-e2e-limits-agg";
  raw.port = 70000;
  try
  {
    vector_from_raw("to_aggregator", raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::InvalidPort);
    EXPECT_EQ(e.number(), 70000);
  }
}

TEST(Render, VectorFromRawBuildsAddress)
{
  RawDestinationParams raw;
  raw.host = "dc-e2e-limits-agg";
  raw.port = 6000;
  auto vec = vector_from_raw("to_aggregator", raw);
  EXPECT_EQ(vec.host, "dc-e2e-limits-agg");
  EXPECT_EQ(vec.port, 6000);
}

// #308: the default is the exact integer format, not the lossy float one. A Destination
// that says nothing about time must not silently round its timestamps.
TEST(Render, DestinationFromRawAppliesTimeDefaults)
{
  RawDestinationParams raw;
  raw.path = "/var/log/dc/records.log";
  auto dest = destination_from_raw("local_log", "file", "records", { "/dc/group/robot" }, raw);
  EXPECT_EQ(dest.time_key, "date");
  EXPECT_EQ(dest.time_format, TimeFormat::EpochNanos);
}

TEST(Render, S3FromRawRejectsMissingBucket)
{
  try
  {
    s3_from_raw("rustfs", {});
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::MissingField);
    EXPECT_EQ(e.arg1(), "bucket");
  }
}

TEST(Render, S3FromRawRejectsHalfCredentialPair)
{
  RawDestinationParams raw;
  raw.bucket = "dc-records";
  raw.access_key_id = "rustfsadmin";
  try
  {
    s3_from_raw("rustfs", raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::IncompleteS3Auth);
  }
}

TEST(Render, S3FromRawRejectsNonPositiveBatchTimeout)
{
  RawDestinationParams raw;
  raw.bucket = "dc-records";
  raw.batch_timeout_secs = 0;
  try
  {
    s3_from_raw("rustfs", raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::InvalidBatchTimeout);
  }
}

TEST(Render, FileFromRawRejectsMissingPath)
{
  try
  {
    file_from_raw("local_log", {});
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::MissingField);
    EXPECT_EQ(e.arg1(), "path");
  }
}

TEST(Render, CustomConfigFilesPassWithFreshComponents)
{
  auto config = basic_config(TimeFormat::Double);
  std::vector<CustomConfigFile> files = { { "/etc/dc/http.toml",
                                            "[sinks.my_http]\ntype = \"http\"\ninputs = [\"dc.dc.group.robot\"]\nuri = "
                                            "\"http://127.0.0.1:8080/ingest\"\nencoding.codec = \"json\"\n" } };
  EXPECT_NO_THROW(validate_custom_config_files(config, files));
}

TEST(Render, CustomConfigFilesRejectInvalidToml)
{
  auto config = basic_config(TimeFormat::Double);
  std::vector<CustomConfigFile> files = { { "/etc/dc/broken.toml", "[sinks.broken\ntype =" } };
  try
  {
    validate_custom_config_files(config, files);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::CustomConfigParse);
    EXPECT_EQ(e.arg0(), "/etc/dc/broken.toml");
  }
}

TEST(Render, CustomConfigFilesRejectEmptySnippet)
{
  auto config = basic_config(TimeFormat::Double);
  std::vector<CustomConfigFile> files = { { "/etc/dc/empty.toml", "# nothing here\n" } };
  try
  {
    validate_custom_config_files(config, files);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::CustomConfigEmpty);
  }
}

TEST(Render, CustomConfigFilesRejectCollisionsWithRendered)
{
  auto config = basic_config(TimeFormat::Double);
  for (const std::string id : { "records_log", "dc", "dc_bridge_in", "dc_bridge_normalize" })
  {
    std::vector<CustomConfigFile> files = {
      { "/etc/dc/collide.toml",
        "[sinks." + id + "]\ntype = \"console\"\ninputs = [\"dc.dc.group.robot\"]\nencoding.codec = \"json\"\n" }
    };
    try
    {
      validate_custom_config_files(config, files);
      FAIL() << "component id '" << id << "' must be rejected";
    }
    catch (const RenderError& e)
    {
      EXPECT_EQ(e.kind(), RenderErrorKind::CustomConfigReservedCollision);
      EXPECT_EQ(e.arg1(), id);
    }
  }
}

TEST(Render, CustomConfigFilesRejectSameComponentInTwoSnippets)
{
  auto config = basic_config(TimeFormat::Double);
  const std::string snippet = "[sinks.my_http]\ntype = \"http\"\ninputs = [\"dc.dc.group.robot\"]\nuri = "
                              "\"http://127.0.0.1:8080/ingest\"\nencoding.codec = \"json\"\n";
  std::vector<CustomConfigFile> files = { { "/a.toml", snippet }, { "/b.toml", snippet } };
  try
  {
    validate_custom_config_files(config, files);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::CustomConfigDuplicate);
    EXPECT_EQ(e.arg0(), "/a.toml");
    EXPECT_EQ(e.arg1(), "/b.toml");
  }
}

// #472: the fix for the unmanaged/split-deployment gap (#471's commit message) — a
// passthrough snippet's own filesystem path is never wired into a separately-run
// Shipper container, so it has to be folded into the one file that container reads.
TEST(Render, MergeCustomConfigFilesAppendsSnippetsAfterTheRenderedConfig)
{
  auto config = basic_config(TimeFormat::Double);
  const std::string rendered = render(config);
  std::vector<CustomConfigFile> files = { { "/etc/dc/http.toml",
                                            "[sinks.my_http]\ntype = \"http\"\ninputs = [\"dc.dc.group.robot\"]\nuri = "
                                            "\"http://127.0.0.1:8080/ingest\"\nencoding.codec = \"json\"\n" } };
  const std::string merged = merge_custom_config_files(rendered, files);

  toml::table parsed = toml::parse(merged);
  EXPECT_NE(parsed["sinks"]["records_log"].as_table(), nullptr) << merged;
  auto* my_http = parsed["sinks"]["my_http"].as_table();
  ASSERT_NE(my_http, nullptr) << merged;
  EXPECT_EQ(my_http->at("type").value<std::string>(), "http");
}

TEST(Render, MergeCustomConfigFilesWithNoFilesReturnsRenderedUnchanged)
{
  auto config = basic_config(TimeFormat::Double);
  const std::string rendered = render(config);
  EXPECT_EQ(merge_custom_config_files(rendered, {}), rendered);
}

TEST(Render, ExpandEnvSubstitutesBothForms)
{
  auto lookup = [](const std::string& k) -> std::optional<std::string> {
    if (k == "HOME")
      return "/home/dc";
    if (k == "DC_S3_SECRET")
      return "s3cr3t";
    return std::nullopt;
  };
  EXPECT_EQ(expand_env("$HOME/.dc/buffer", lookup), "/home/dc/.dc/buffer");
  EXPECT_EQ(expand_env("${DC_S3_SECRET}", lookup), "s3cr3t");
}

TEST(Render, ExpandEnvRejectsUndefinedVariable)
{
  try
  {
    expand_env("$MISSING", [](const std::string&) { return std::nullopt; });
    FAIL();
  }
  catch (const ExpandError& e)
  {
    EXPECT_EQ(e.var(), "MISSING");
  }
}
