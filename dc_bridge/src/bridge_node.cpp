// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/bridge_node.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "dc_bridge/atomic_write.hpp"
#include "dc_bridge/topic_config.hpp"
#include "dc_bridge/vector_binary.hpp"

namespace dc_bridge
{

namespace
{

std::optional<std::string> env_lookup(const std::string& name)
{
  const char* v = ::getenv(name.c_str());
  if (v == nullptr)
  {
    return std::nullopt;
  }
  return std::string(v);
}

std::string expand_with_env(const std::string& input)
{
  return expand_env(input, env_lookup);
}

// Declares `name` with dynamic typing and a PARAMETER_NOT_SET default, so a value the
// user didn't provide reads back as nullopt (rather than forcing a sentinel/default).
std::optional<std::string> declare_optional_string(rclcpp::Node* node, const std::string& name)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.dynamic_typing = true;
  node->declare_parameter(name, rclcpp::ParameterValue(), d);
  rclcpp::Parameter p = node->get_parameter(name);
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    return std::nullopt;
  }
  return p.as_string();
}

std::optional<std::int64_t> declare_optional_int(rclcpp::Node* node, const std::string& name)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.dynamic_typing = true;
  node->declare_parameter(name, rclcpp::ParameterValue(), d);
  rclcpp::Parameter p = node->get_parameter(name);
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return std::nullopt;
  }
  return p.as_int();
}

std::optional<bool> declare_optional_bool(rclcpp::Node* node, const std::string& name)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.dynamic_typing = true;
  node->declare_parameter(name, rclcpp::ParameterValue(), d);
  rclcpp::Parameter p = node->get_parameter(name);
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
  {
    return std::nullopt;
  }
  return p.as_bool();
}

// Declares every <name>.* parameter a Destination might need and validates them into a
// typed Destination via destination_from_raw. Credentials support $VAR env expansion.
Destination declare_destination(rclcpp::Node* node, const std::string& name)
{
  const std::string type_str = node->declare_parameter<std::string>(name + ".type", "");
  const std::string receives_str = node->declare_parameter<std::string>(name + ".receives", "records");
  // Empty `inputs` is legitimate (a `receives: files` metadata destination takes no topic
  // inputs — the Uploader routes to it internally). It must be *omitted* from the params
  // file, not written as `inputs: []`: rclcpp can't load an empty YAML array (it has no
  // inferable element type) and fatals before this code runs. Omitted -> this default {}.
  const std::vector<std::string> inputs =
      node->declare_parameter<std::vector<std::string>>(name + ".inputs", std::vector<std::string>{});

  RawDestinationParams raw;
  raw.time_key = declare_optional_string(node, name + ".time_key");
  raw.time_format = declare_optional_string(node, name + ".time_format");
  raw.host = declare_optional_string(node, name + ".host");
  raw.port = declare_optional_int(node, name + ".port");
  raw.user = declare_optional_string(node, name + ".user");
  raw.database = declare_optional_string(node, name + ".database");
  raw.table = declare_optional_string(node, name + ".table");
  raw.bucket = declare_optional_string(node, name + ".bucket");
  raw.region = declare_optional_string(node, name + ".region");
  raw.endpoint = declare_optional_string(node, name + ".endpoint");
  raw.key_prefix = declare_optional_string(node, name + ".key_prefix");
  raw.access_key_id = declare_optional_string(node, name + ".access_key_id");
  raw.force_path_style = declare_optional_bool(node, name + ".force_path_style");
  raw.batch_timeout_secs = declare_optional_int(node, name + ".batch_timeout_secs");
  raw.path = declare_optional_string(node, name + ".path");

  // Credentials support $DC_PG_PASSWORD-style env references (ADR-0003 contract).
  if (auto pw = declare_optional_string(node, name + ".password"))
  {
    raw.password = expand_with_env(*pw);
  }
  if (auto sk = declare_optional_string(node, name + ".secret_access_key"))
  {
    raw.secret_access_key = expand_with_env(*sk);
  }

  return destination_from_raw(name, type_str, receives_str, inputs, raw);
}

// Runs a program with args, returning {exit_code, combined stdout+stderr}. -1 exit_code
// means the program couldn't be started.
std::pair<int, std::string> run_capture(const std::string& program, const std::vector<std::string>& args)
{
  int pipefd[2];
  if (::pipe(pipefd) != 0)
  {
    return { -1, "pipe() failed" };
  }
  pid_t pid = ::fork();
  if (pid < 0)
  {
    ::close(pipefd[0]);
    ::close(pipefd[1]);
    return { -1, "fork() failed" };
  }
  if (pid == 0)
  {
    ::dup2(pipefd[1], STDOUT_FILENO);
    ::dup2(pipefd[1], STDERR_FILENO);
    ::close(pipefd[0]);
    ::close(pipefd[1]);
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(program.c_str()));
    for (const auto& a : args)
    {
      argv.push_back(const_cast<char*>(a.c_str()));
    }
    argv.push_back(nullptr);
    ::execvp(program.c_str(), argv.data());
    _exit(127);
  }
  ::close(pipefd[1]);
  std::string out;
  char buf[4096];
  ssize_t n;
  while ((n = ::read(pipefd[0], buf, sizeof(buf))) > 0)
  {
    out.append(buf, static_cast<std::size_t>(n));
  }
  ::close(pipefd[0]);
  int status = 0;
  ::waitpid(pid, &status, 0);
  int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return { code, out };
}

}  // namespace

BridgeNode::BridgeNode(const rclcpp::NodeOptions& options) : rclcpp::Node("dc_bridge", options)
{
  const std::string vector_host = this->declare_parameter<std::string>("vector_forward_host", "127.0.0.1");
  const std::int64_t vector_port = this->declare_parameter<std::int64_t>("vector_forward_port", 24224);
  const std::string vector_binary_override = this->declare_parameter<std::string>("vector_binary", "");
  const auto forward_port = static_cast<std::uint16_t>(vector_port);

  // --- shipper + destinations parameters (ADR-0003 config contract) ---
  const std::string shipper_data_dir =
      expand_with_env(this->declare_parameter<std::string>("shipper.data_dir", "$HOME/.dc/buffer"));
  // The Uploader's own directory (#441): the durable upload intent queue and multipart-resume
  // state don't need to share a directory with the Shipper's disk buffer — only pulling from
  // one parameter made them. Defaults to shipper.data_dir so a deployment that only ever set
  // that keeps working unchanged; the two may still point at the same directory.
  const std::string uploader_data_dir =
      expand_with_env(this->declare_parameter<std::string>("uploader.data_dir", shipper_data_dir));
  const std::int64_t buffer_max_bytes = this->declare_parameter<std::int64_t>(
      "shipper.buffer_max_bytes", static_cast<std::int64_t>(MIN_DISK_BUFFER_BYTES));
  // Managed (default, ADR-0001/0006/0007): the Bridge locates the vendored Vector
  // binary, spawns it, and supervises it. Unmanaged (#440/#444, the split-deployment
  // topology): an orchestrator owns the Shipper's lifecycle — the Bridge only renders
  // the config and connects, exactly as in managed mode.
  const bool shipper_managed = this->declare_parameter<bool>("shipper.managed", true);
  // Empty (default) keeps the historic temp-file path so managed-mode behaviour is
  // unchanged. Configurable so unmanaged mode can place the file on a volume shared with
  // the Shipper container/pod rather than a path private to the Bridge's own filesystem.
  const std::string shipper_config_path_param = this->declare_parameter<std::string>("shipper.config_path", "");
  const std::vector<std::string> destination_names =
      this->declare_parameter<std::vector<std::string>>("destinations", std::vector<std::string>{});

  std::vector<Destination> records_destinations;
  std::vector<Destination> files_destinations;
  for (const auto& name : destination_names)
  {
    Destination d = declare_destination(this, name);
    if (d.receives == Receives::Files)
    {
      files_destinations.push_back(std::move(d));
    }
    else
    {
      records_destinations.push_back(std::move(d));
    }
  }

  // --- raw / generic-subscription mode (#227) ---
  // Declared before render() below, because enabling it adds a routed Tag namespace to
  // one of the Destinations the Vector config is rendered from.
  raw_config_.enabled = this->declare_parameter<bool>("raw.enabled", false);
  raw_config_.destination = this->declare_parameter<std::string>("raw.destination", "");
  raw_config_.include =
      this->declare_parameter<std::vector<std::string>>("raw.include", std::vector<std::string>{ "^/" });
  raw_config_.exclude = this->declare_parameter<std::vector<std::string>>("raw.exclude", default_raw_exclude());
  raw_config_.exclude_types =
      this->declare_parameter<std::vector<std::string>>("raw.exclude_types", default_raw_exclude_types());
  raw_config_.rescan_interval_secs = this->declare_parameter<double>("raw.rescan_interval_secs", 5.0);
  raw_config_.qos_depth =
      static_cast<std::size_t>(std::max<std::int64_t>(1, this->declare_parameter<std::int64_t>("raw.qos_depth", 10)));
  raw_config_.max_rate_hz = this->declare_parameter<double>("raw.max_rate_hz", 10.0);
  raw_config_.max_message_size_bytes = static_cast<std::uint64_t>(
      std::max<std::int64_t>(0, this->declare_parameter<std::int64_t>("raw.max_message_size_bytes", 1048576)));
  raw_config_.tag_prefix = this->declare_parameter<std::string>("raw.tag_prefix", RAW_TAG_PREFIX);

  if (raw_config_.enabled)
  {
    if (raw_config_.tag_prefix.empty())
    {
      // An empty prefix renders as `starts_with(.tag, "")`, which matches every Record
      // in the pipeline — every Measurement Record would be duplicated into the raw
      // Destination. Refuse rather than route the whole pipeline by accident.
      throw std::runtime_error("`raw.tag_prefix` must not be empty");
    }
    auto target = std::find_if(records_destinations.begin(), records_destinations.end(),
                               [this](const Destination& d) { return d.name == raw_config_.destination; });
    if (target == records_destinations.end())
    {
      throw std::runtime_error("raw.destination '" + raw_config_.destination +
                               "' does not name a configured `receives: records` destination");
    }
    // One routed Tag *namespace* rather than one route per topic: raw mode discovers
    // topics while Vector is already running, and a rendered config can't grow new
    // routes without a restart (see route_output_for_tag_prefix).
    target->tag_prefixes.push_back(raw_config_.tag_prefix);
  }

  // files.metadata_destination (ADR-0005): the only files.* parameter the Bridge itself
  // still needs. Everything else the Uploader used to read from files.*/uploader.data_dir
  // (delete_when_sent, ffprobe/ffmpeg binaries, multipart sizing, thumbnails, retention)
  // moved with it into dc_uploader's own DC_UPLOADER_* environment variables (#446,
  // docs/adr/0014-uploader-runs-as-its-own-process.md) — the Bridge no longer uploads, so
  // it has nothing to configure there.
  const std::string metadata_destination = this->declare_parameter<std::string>("files.metadata_destination", "");

  if (!files_destinations.empty())
  {
    if (metadata_destination.empty())
    {
      throw std::runtime_error("`files.metadata_destination` is required when a `receives: files` destination is "
                               "configured");
    }
    auto target = std::find_if(records_destinations.begin(), records_destinations.end(),
                               [&](const Destination& d) { return d.name == metadata_destination; });
    if (target == records_destinations.end())
    {
      throw std::runtime_error("files.metadata_destination '" + metadata_destination +
                               "' does not name a configured `receives: records` destination");
    }
    // The Uploader's status Records land at the configured metadata destination: append
    // its Tag to that Destination's routed set (rendered/normalized/consumed like any
    // topic-derived Tag). The Uploader itself runs as dc_uploader, a separate process
    // (#446); the Bridge's job here is only to make sure Vector routes dc.files there.
    target->extra_tags.push_back(uploader::FILE_STATUS_TAG);
    for (const auto& dest : files_destinations)
    {
      if (std::get_if<S3Params>(&dest.kind) == nullptr)
      {
        // destination_from_raw enforces `type: s3` for `receives: files`.
        throw std::runtime_error("internal error: files destination '" + dest.name + "' is not object storage");
      }
    }

    // The durable intent queue (#265/#446): the Bridge writes an intent for every Record
    // a files-Destination's subscription receives and never reads it back — a separate
    // dc_uploader process owns replay, backoff, and acking. Its own directory
    // (uploader.data_dir's multipart-resume state) is dc_uploader's concern now, not the
    // Bridge's.
    intent_queue_ = std::make_unique<uploader::IntentQueue>(uploader_data_dir + "/queue/upload");
  }

  RenderConfig render_config;
  render_config.forward_host = vector_host;
  render_config.forward_port = forward_port;
  render_config.data_dir = shipper_data_dir;
  render_config.buffer_max_bytes = static_cast<std::uint64_t>(std::max<std::int64_t>(0, buffer_max_bytes));
  render_config.destinations = records_destinations;

  const std::string rendered = render(render_config);

  // --- custom_config_files passthrough (ADR-0003) ---
  const std::vector<std::string> custom_paths =
      this->declare_parameter<std::vector<std::string>>("custom_config_files", std::vector<std::string>{});
  std::vector<CustomConfigFile> custom_files;
  for (const auto& raw_path : custom_paths)
  {
    const std::string path = expand_with_env(raw_path);
    std::ifstream in(path);
    if (!in.good())
    {
      throw std::runtime_error("failed to read custom config file '" + path + "'");
    }
    std::stringstream ss;
    ss << in.rdbuf();
    custom_files.push_back({ path, ss.str() });
  }
  validate_custom_config_files(render_config, custom_files);

  // --- write the rendered config atomically (write then rename, #444), build the
  // --config set --- so a Shipper reading the file (its own process in managed mode, an
  // orchestrator-supervised one in unmanaged mode) can never observe a partial write.
  const std::string config_path = shipper_config_path_param.empty() ?
                                      (std::filesystem::temp_directory_path() / "dc_bridge_vector.toml").string() :
                                      expand_with_env(shipper_config_path_param);
  write_file_atomically(config_path, rendered);

  std::vector<std::string> config_paths{ config_path };
  for (const auto& f : custom_files)
  {
    config_paths.push_back(f.path);
  }

  // --- locate, validate and supervise the vendored Vector binary — managed mode only
  // (#444). In unmanaged mode the Shipper's lifecycle belongs to an orchestrator: the
  // Bridge locates no binary, runs no validator against it, spawns no child, and installs
  // no parent-death signal — it only rendered the config above and connects below.
  std::string vector_binary;
  if (shipper_managed)
  {
    if (!vector_binary_override.empty())
    {
      vector_binary = vector_binary_override;
    }
    else
    {
      const std::string amentp = env_lookup("AMENT_PREFIX_PATH").value_or("");
      auto found = find_vector_binary(amentp);
      if (!found)
      {
        throw std::runtime_error("could not locate vendored Vector binary; set the vector_binary parameter");
      }
      vector_binary = *found;
    }

    supervisor_ = std::make_shared<Supervisor>(SupervisorConfig::vector(vector_binary, config_paths));

    // Backstop for anything the pure checks can't see (dangling dc.<tag> inputs, sink
    // options Vector itself rejects): run Vector's own validator before starting, so an
    // invalid config is a loud startup failure, not a supervised-Vector crash loop.
    {
      std::vector<std::string> validate_args{ "validate", "--no-environment" };
      validate_args.insert(validate_args.end(), config_paths.begin(), config_paths.end());
      auto [code, output] = run_capture(vector_binary, validate_args);
      if (code != 0)
      {
        throw std::runtime_error("vector rejected the merged configuration:\n" + output);
      }
    }

    {
      std::lock_guard<std::mutex> lock(supervisor_mutex_);
      supervisor_->start();
    }
  }

  ForwarderConfig fcfg;
  fcfg.host = vector_host;
  fcfg.port = forward_port;
  fcfg.on_warning = [this](const std::string& msg) { RCLCPP_WARN(this->get_logger(), "%s", msg.c_str()); };
  forwarder_ = std::make_shared<Forwarder>(fcfg);

  // Background prober: respawns Vector if it dies and keeps the readiness flag current.
  prober_thread_ = std::thread(&BridgeNode::run_prober, this, vector_host, forward_port);

  // Topics feeding Records destinations (forwarded to Vector) vs. topics feeding files
  // destinations (scanned for File references, whose intents the Bridge writes for the
  // separate dc_uploader process to read, #446). A topic can be in both.
  std::set<std::string> records_topics;
  for (const auto& d : render_config.destinations)
  {
    for (const auto& t : d.inputs)
    {
      records_topics.insert(t);
    }
  }
  std::set<std::string> files_topics;
  for (const auto& d : files_destinations)
  {
    for (const auto& t : d.inputs)
    {
      files_topics.insert(t);
    }
  }

  // Subscribe to the union, once each.
  std::set<std::string> subscribed = records_topics;
  subscribed.insert(files_topics.begin(), files_topics.end());

  for (const auto& topic : subscribed)
  {
    const std::string tag = TopicConfig::derive_tag(topic);
    const bool forward_to_vector = records_topics.count(topic) > 0;
    const bool feeds_uploader = files_topics.count(topic) > 0;
    auto sub = this->create_subscription<dc_interfaces::msg::StringStamped>(
        topic, rclcpp::QoS(10),
        [this, tag, forward_to_vector, feeds_uploader](const dc_interfaces::msg::StringStamped& msg) {
          nlohmann::json payload;
          try
          {
            payload = nlohmann::json::parse(msg.data);
          }
          catch (const nlohmann::json::exception&)
          {
            payload = nlohmann::json(msg.data);
          }

          if (feeds_uploader)
          {
            // Durable enqueue (#265): the intent lands on disk before this callback
            // returns, so it survives a Bridge crash/restart (or the separate dc_uploader
            // process, #446, never having been up at all). dc_uploader's own poll loop
            // picks it up by rescanning the queue directory; there is no in-process
            // wake-up signal to a different OS process.
            intent_queue_->enqueue(tag, payload);
          }

          if (forward_to_vector)
          {
            Record record;
            record.tag = tag;
            // Both halves of the ROS stamp. The nanoseconds used to be dropped here,
            // which rounded every Record to the second and left a Measurement polling
            // faster than 1 Hz with Records indistinguishable in time (#308).
            record.timestamp_secs = static_cast<std::uint64_t>(std::max<std::int32_t>(0, msg.header.stamp.sec));
            record.timestamp_nanos = msg.header.stamp.nanosec;
            record.payload = std::move(payload);
            try
            {
              std::lock_guard<std::mutex> lock(forwarder_mutex_);
              forwarder_->send(record);
            }
            catch (const ForwarderError& e)
            {
              RCLCPP_WARN(this->get_logger(), "failed to forward record on tag '%s': %s", tag.c_str(), e.what());
            }
          }
        });
    subscriptions_.push_back(sub);
  }

  // Raw mode's own subscriptions are created last: they discover topics from the live
  // graph rather than from parameters, and the Forwarder they feed has to exist first.
  if (raw_config_.enabled)
  {
    raw_manager_ = std::make_unique<RawSubscriptionManager>(
        this, raw_config_,
        [this](const std::string& tag, const RawStamp& stamp, nlohmann::json payload) {
          Record record;
          record.tag = tag;
          record.timestamp_secs = stamp.secs;
          record.timestamp_nanos = stamp.nanos;
          record.payload = std::move(payload);
          try
          {
            std::lock_guard<std::mutex> lock(forwarder_mutex_);
            forwarder_->send(record);
            return true;
          }
          catch (const ForwarderError&)
          {
            // Deliberately dropped, not retried or queued: raw mode can produce Records
            // faster than the Shipper accepts them, and the only bounded answer is to
            // shed at the source. The manager counts the drop and warns (throttled).
            return false;
          }
        },
        &raw_stats_);
    raw_manager_->start();
  }

  readiness_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/ready", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = readiness_.is_ready();
        response->message = response->success ? "vector is accepting connections" : "vector is not ready";
        if (intent_queue_)
        {
          // Cheap observability hook (#265): lets an operator see the upload backlog
          // without a separate metrics path.
          response->message += " | upload queue depth: " + std::to_string(intent_queue_->size());
        }
        if (raw_manager_)
        {
          // The drop counters are the observable half of raw mode's documented
          // backpressure behaviour — an operator asking "is raw mode shedding?" gets the
          // answer from the same probe the readiness gate already uses.
          response->message += " | " + raw_stats_.summary();
        }
      });

  if (shipper_managed)
  {
    RCLCPP_INFO(this->get_logger(), "dc_bridge up: %zu subscribed topic(s), supervising %s", subscriptions_.size(),
                vector_binary.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),
                "dc_bridge up: %zu subscribed topic(s), unmanaged shipper mode (config written to %s)",
                subscriptions_.size(), config_path.c_str());
  }
  if (raw_manager_)
  {
    RCLCPP_INFO(this->get_logger(),
                "raw mode on: %zu topic(s) discovered so far, Tag namespace '%s' → destination '%s'",
                raw_manager_->subscription_count(), raw_config_.tag_prefix.c_str(), raw_config_.destination.c_str());
  }
}

void BridgeNode::run_prober(std::string forward_host, std::uint16_t forward_port)
{
  while (!prober_stop_.load())
  {
    {
      // supervisor_ is null in unmanaged mode (#444) — nothing to restart, the readiness
      // probe below is what tracks the Shipper either way.
      std::lock_guard<std::mutex> lock(supervisor_mutex_);
      if (supervisor_)
      {
        supervisor_->poll_restart();
      }
    }
    const bool ready = probe(forward_host, forward_port, std::chrono::milliseconds(200));
    readiness_.set_ready(ready);
    {
      // Keeps the unacked window (#266) converging — draining fresh acks, resending
      // anything past ack_timeout, and redelivering after a reconnect — even while no
      // new Record is being published to trigger it via send().
      std::lock_guard<std::mutex> lock(forwarder_mutex_);
      forwarder_->poll();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}

void BridgeNode::stop()
{
  if (stopped_.exchange(true))
  {
    return;
  }
  prober_stop_.store(true);
  if (prober_thread_.joinable())
  {
    prober_thread_.join();
  }
  std::lock_guard<std::mutex> lock(supervisor_mutex_);
  if (supervisor_)
  {
    supervisor_->stop();
  }
}

BridgeNode::~BridgeNode()
{
  stop();
}

}  // namespace dc_bridge
