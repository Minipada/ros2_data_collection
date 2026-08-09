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

PostgresParams postgres_kind()
{
  return PostgresParams{ "127.0.0.1", 5432, "dc", "hunter2", "dc", "dc" };
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
  return config_with({ make_destination("pgsql", { "/dc/group/robot" }, tf, postgres_kind()) });
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

TEST(Render, SinglePostgresDestinationDouble)
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
      make_destination("pgsql", { "/dc/group/robot" }, TimeFormat::Double, postgres_kind()),
      make_destination("pgsql_archive", { "/dc/measurement/uptime" }, TimeFormat::Iso8601, postgres_kind()),
  });
  assert_matches_fixture(render(config), "multiple_destinations.toml");
}

TEST(Render, S3SelfHostedCustomEndpoint)
{
  S3Params s3;
  s3.bucket = "dc-records";
  s3.region = "us-east-1";
  s3.endpoint = "http://127.0.0.1:9000";
  s3.key_prefix = "robot1/";
  s3.auth = S3Auth{ "rustfsadmin", "rustfsadmin" };
  s3.force_path_style = true;
  s3.batch_timeout_secs = 2;
  auto config = config_with(
      { make_destination("rustfs", { "/dc/group/robot", "/dc/measurement/uptime" }, TimeFormat::Double, s3) });
  assert_matches_fixture(render(config), "s3_custom_endpoint.toml");
}

TEST(Render, S3AwsAmbientCredentials)
{
  S3Params s3;
  s3.bucket = "dc-records";
  s3.region = "eu-west-1";
  auto config = config_with({ make_destination("s3_archive", { "/dc/group/robot" }, TimeFormat::Iso8601, s3) });
  assert_matches_fixture(render(config), "s3_aws.toml");
}

TEST(Render, FileAndConsoleDestinations)
{
  auto config = config_with({
      make_destination("local_log", { "/dc/group/robot" }, TimeFormat::Double,
                       FileParams{ "/var/log/dc/records-%Y-%m-%d.log" }),
      make_destination("debug_console", { "/dc/group/robot" }, TimeFormat::Double, ConsoleParams{}),
  });
  assert_matches_fixture(render(config), "file_and_console.toml");
}

TEST(Render, ConsoleSinkHasNoDiskBuffer)
{
  auto config =
      config_with({ make_destination("debug_console", { "/dc/group/robot" }, TimeFormat::Double, ConsoleParams{}) });
  toml::table parsed = toml::parse(render(config));
  EXPECT_FALSE(parsed["sinks"]["debug_console"].as_table()->contains("buffer"));
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
      make_destination("pgsql", { "/dc/group/robot", "/dc/measurement/uptime" }, TimeFormat::Double, postgres_kind()),
      make_destination("debug_console", { "/dc/measurement/uptime" }, TimeFormat::Double, ConsoleParams{}),
  });
  toml::table parsed = toml::parse(render(config));

  auto* route = parsed["transforms"]["dc"]["route"].as_table();
  ASSERT_NE(route, nullptr);
  EXPECT_TRUE(route->contains("dc.group.robot"));
  EXPECT_TRUE(route->contains("dc.measurement.uptime"));
  EXPECT_EQ(route->size(), 2u);

  auto pgsql_inputs = parsed["sinks"]["pgsql"]["inputs"].as_array();
  ASSERT_NE(pgsql_inputs, nullptr);
  EXPECT_EQ((*pgsql_inputs)[0].value<std::string>(), "dc.dc.group.robot");
  EXPECT_EQ((*pgsql_inputs)[1].value<std::string>(), "dc.dc.measurement.uptime");
}

TEST(Render, RouteOutputForTagIsPublicName)
{
  EXPECT_EQ(route_output_for_tag("dc.measurement.uptime"), "dc.dc.measurement.uptime");
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
    EXPECT_EQ(e.arg0(), "pgsql");
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
    EXPECT_EQ(e.arg0(), "pgsql");
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

TEST(Render, DestinationFromRawBuildsS3FileConsole)
{
  RawDestinationParams s3raw;
  s3raw.bucket = "dc-records";
  s3raw.endpoint = "http://127.0.0.1:9000";
  s3raw.access_key_id = "rustfsadmin";
  s3raw.secret_access_key = "rustfsadmin";
  s3raw.force_path_style = true;
  auto s3 = destination_from_raw("rustfs", "s3", "records", { "/dc/group/robot" }, s3raw);
  EXPECT_TRUE(std::holds_alternative<S3Params>(s3.kind));

  RawDestinationParams fileraw;
  fileraw.path = "/var/log/dc/records.log";
  auto file = destination_from_raw("local_log", "file", "records", { "/dc/group/robot" }, fileraw);
  EXPECT_TRUE(std::holds_alternative<FileParams>(file.kind));

  auto console = destination_from_raw("debug_console", "console", "records", { "/dc/group/robot" }, {});
  EXPECT_TRUE(std::holds_alternative<ConsoleParams>(console.kind));
}

TEST(Render, DestinationFromRawRejectsFilesOnNonObjectStorage)
{
  for (const std::string bad : { "postgres", "file", "console" })
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

TEST(Render, ExtraTagsAreRoutedNormalizedAndConsumed)
{
  auto config = basic_config(TimeFormat::Double);
  config.destinations[0].extra_tags.push_back("dc.files");
  toml::table parsed = toml::parse(render(config));

  auto* route = parsed["transforms"]["dc"]["route"].as_table();
  EXPECT_TRUE(route->contains("dc.files"));
  EXPECT_TRUE(route->contains("dc.group.robot"));

  auto inputs = parsed["sinks"]["pgsql"]["inputs"].as_array();
  EXPECT_EQ((*inputs)[0].value<std::string>(), "dc.dc.files");
  EXPECT_EQ((*inputs)[1].value<std::string>(), "dc.dc.group.robot");

  auto normalize = parsed["transforms"]["dc_bridge_normalize"]["source"].value<std::string>();
  EXPECT_NE(normalize->find("includes([\"dc.files\", \"dc.group.robot\"], .tag)"), std::string::npos);
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
    destination_from_raw("archive", "postgres", "bogus", { "/dc/measurement/map" }, {});
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
    destination_from_raw("debug_console", "console", "records", { "/dc/measurement/map" }, raw);
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
    auto dest = destination_from_raw("debug_console", "console", "records", { "/dc/measurement/map" }, raw);
    EXPECT_EQ(dest.time_format, c.second) << "time_format '" << c.first << "'";
  }
}

// The normalize transform must emit the timestamp with no float arithmetic in it —
// `to_float(...) / 1000000000.0` is precisely what loses resolution below ~1 us.
TEST(Render, EpochNanosNormalizesWithoutFloatRounding)
{
  auto config =
      config_with({ make_destination("pgsql", { "/dc/group/robot" }, TimeFormat::EpochNanos, postgres_kind()) });
  const std::string rendered = render(config);
  EXPECT_NE(rendered.find(".date = to_unix_timestamp!(.timestamp, unit: \"nanoseconds\")"), std::string::npos)
      << rendered;
  EXPECT_EQ(rendered.find("to_float("), std::string::npos)
      << "epoch_nanos must not round through a float: " << rendered;
}

TEST(Render, PostgresFromRawRejectsMissingRequiredFields)
{
  RawDestinationParams raw;
  raw.user = "dc";
  try
  {
    postgres_from_raw("pgsql", raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::MissingField);
    EXPECT_EQ(e.arg1(), "password");
  }
}

TEST(Render, PostgresFromRawRejectsOutOfRangePort)
{
  RawDestinationParams raw;
  raw.user = "dc";
  raw.password = "secret";
  raw.database = "dc";
  raw.table = "dc";
  raw.port = 70000;
  try
  {
    postgres_from_raw("pgsql", raw);
    FAIL();
  }
  catch (const RenderError& e)
  {
    EXPECT_EQ(e.kind(), RenderErrorKind::InvalidPort);
    EXPECT_EQ(e.number(), 70000);
  }
}

TEST(Render, PostgresFromRawAppliesDefaults)
{
  RawDestinationParams raw;
  raw.user = "dc";
  raw.password = "secret";
  raw.database = "dc";
  raw.table = "dc";
  auto pg = postgres_from_raw("pgsql", raw);
  EXPECT_EQ(pg.host, "127.0.0.1");
  EXPECT_EQ(pg.port, 5432);
}

// #308: the default is the exact integer format, not the lossy float one. A Destination
// that says nothing about time must not silently round its timestamps.
TEST(Render, DestinationFromRawAppliesTimeDefaults)
{
  auto dest = destination_from_raw("debug_console", "console", "records", { "/dc/group/robot" }, {});
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
  for (const std::string id : { "pgsql", "dc", "dc_bridge_in", "dc_bridge_normalize" })
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

TEST(Render, ExpandEnvSubstitutesBothForms)
{
  auto lookup = [](const std::string& k) -> std::optional<std::string> {
    if (k == "HOME")
      return "/home/dc";
    if (k == "DC_PG_PASSWORD")
      return "s3cr3t";
    return std::nullopt;
  };
  EXPECT_EQ(expand_env("$HOME/.dc/buffer", lookup), "/home/dc/.dc/buffer");
  EXPECT_EQ(expand_env("${DC_PG_PASSWORD}", lookup), "s3cr3t");
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

TEST(Render, PercentEncodesSpecialCharactersInCredentials)
{
  auto config = basic_config(TimeFormat::Double);
  std::get<PostgresParams>(config.destinations[0].kind).password = "p@ss/w:rd%";
  toml::table parsed = toml::parse(render(config));
  EXPECT_EQ(parsed["sinks"]["pgsql"]["endpoint"].value<std::string>(),
            "postgres://dc:p%40ss%2Fw%3Ard%25@127.0.0.1:5432/dc");
}
