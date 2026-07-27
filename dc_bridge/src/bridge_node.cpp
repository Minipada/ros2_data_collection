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
  const std::string data_dir =
      expand_with_env(this->declare_parameter<std::string>("shipper.data_dir", "$HOME/.dc/buffer"));
  const std::int64_t buffer_max_bytes = this->declare_parameter<std::int64_t>(
      "shipper.buffer_max_bytes", static_cast<std::int64_t>(MIN_DISK_BUFFER_BYTES));
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

  // files.* parameters (ADR-0005).
  const bool files_delete_when_sent = this->declare_parameter<bool>("files.delete_when_sent", false);
  const std::string files_ffprobe = this->declare_parameter<std::string>("files.ffprobe_binary", "ffprobe");
  const auto files_multipart_threshold = declare_optional_int(this, "files.multipart_threshold_bytes");
  const auto files_multipart_part_size = declare_optional_int(this, "files.multipart_part_size_bytes");
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
    // topic-derived Tag).
    target->extra_tags.push_back(uploader::FILE_STATUS_TAG);

    // Build the Uploader (ADR-0005): one S3 storage per `receives: files` Destination.
    std::vector<Storage> storages;
    for (const auto& dest : files_destinations)
    {
      const auto* s3 = std::get_if<S3Params>(&dest.kind);
      if (s3 == nullptr)
      {
        // destination_from_raw enforces `type: s3` for `receives: files`.
        throw std::runtime_error("internal error: files destination '" + dest.name + "' is not object storage");
      }
      storages.push_back(make_s3_storage(dest.name, *s3));
    }
    uploader::UploaderConfig ucfg(data_dir + "/uploader", files_delete_when_sent);
    if (files_multipart_threshold)
    {
      ucfg.multipart_threshold_bytes =
          static_cast<std::uint64_t>(std::max<std::int64_t>(1, *files_multipart_threshold));
    }
    if (files_multipart_part_size)
    {
      ucfg.multipart_part_size_bytes =
          static_cast<std::uint64_t>(std::max<std::int64_t>(1, *files_multipart_part_size));
    }
    uploader_ = std::make_unique<uploader::Uploader>(std::move(ucfg), std::move(storages),
                                                     uploader::ffprobe_duration_prober(files_ffprobe));
    // The durable intent queue (#265): sibling to the Uploader's own
    // <data_dir>/uploader multipart-resume state, replayed on every startup so a Bridge
    // restart never forgets a pending upload.
    intent_queue_ = std::make_unique<uploader::IntentQueue>(data_dir + "/queue/upload");
  }

  RenderConfig render_config;
  render_config.forward_host = vector_host;
  render_config.forward_port = forward_port;
  render_config.data_dir = data_dir;
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

  // --- locate the vendored Vector binary ---
  std::string vector_binary;
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

  // --- write the rendered config, build the --config set ---
  const std::string config_path = (std::filesystem::temp_directory_path() / "dc_bridge_vector.toml").string();
  std::ofstream(config_path) << rendered;

  std::vector<std::string> config_paths{ config_path };
  for (const auto& f : custom_files)
  {
    config_paths.push_back(f.path);
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

  ForwarderConfig fcfg;
  fcfg.host = vector_host;
  fcfg.port = forward_port;
  fcfg.on_warning = [this](const std::string& msg) { RCLCPP_WARN(this->get_logger(), "%s", msg.c_str()); };
  forwarder_ = std::make_shared<Forwarder>(fcfg);

  // Background prober: respawns Vector if it dies and keeps the readiness flag current.
  prober_thread_ = std::thread(&BridgeNode::run_prober, this, vector_host, forward_port);

  // The Uploader worker (ADR-0005), if any `receives: files` destination is configured.
  if (uploader_)
  {
    uploader_thread_ = std::thread(&BridgeNode::run_uploader_worker, this, vector_host, forward_port);
  }

  // Topics feeding Records destinations (forwarded to Vector) vs. topics feeding files
  // destinations (scanned for File references by the Uploader). A topic can be in both.
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
            // returns, so it survives a Bridge crash/restart even if the worker never
            // gets to it. The condition variable is only a cheap in-process wake-up
            // signal, not the source of truth.
            intent_queue_->enqueue(tag, payload);
            uploader_wake_cv_.notify_one();
          }

          if (forward_to_vector)
          {
            Record record;
            record.tag = tag;
            record.timestamp_secs = static_cast<std::uint64_t>(std::max<std::int32_t>(0, msg.header.stamp.sec));
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
      });

  RCLCPP_INFO(this->get_logger(), "dc_bridge up: %zu subscribed topic(s), supervising %s", subscriptions_.size(),
              vector_binary.c_str());
}

void BridgeNode::run_uploader_worker(std::string forward_host, std::uint16_t forward_port)
{
  // The Uploader's own Forwarder connection, so long uploads never hold the subscription
  // callbacks' Forwarder lock.
  ForwarderConfig fcfg;
  fcfg.host = forward_host;
  fcfg.port = forward_port;
  fcfg.on_warning = [this](const std::string& msg) { RCLCPP_WARN(this->get_logger(), "%s", msg.c_str()); };
  Forwarder forwarder(fcfg);

  auto emit = [&forwarder](const nlohmann::json& row) {
    Record record;
    record.tag = uploader::FILE_STATUS_TAG;
    record.timestamp_secs = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    record.payload = row;
    // Vector may be briefly down (restart, backpressure); keep trying for a while before
    // handing the whole Record back for an idempotent retry.
    std::string last_err;
    for (int i = 0; i < 120; ++i)
    {
      try
      {
        forwarder.send(record);
        return;
      }
      catch (const std::exception& e)
      {
        last_err = e.what();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    throw std::runtime_error(last_err);
  };

  for (;;)
  {
    {
      std::unique_lock<std::mutex> lock(uploader_wake_mutex_);
      if (upload_stop_)
      {
        return;
      }
      // The condition_variable is only a cheap wake-up signal for a fresh enqueue; the
      // bounded wait also re-checks periodically so a backed-off intent's retry time
      // elapsing gets noticed even without a new Record arriving (#265 — the queue
      // itself has no timer, only next_ready()'s point-in-time check).
      uploader_wake_cv_.wait_for(lock, std::chrono::milliseconds(500));
      if (upload_stop_)
      {
        return;
      }
    }

    // Keeps this Forwarder's own unacked window (#266) converging between status-Record
    // emissions, same reason the main prober thread polls the subscription Forwarder.
    forwarder.poll();

    auto item = intent_queue_->next_ready();
    if (!item)
    {
      continue;  // empty, or every pending intent is still backing off.
    }

    try
    {
      const auto summary = uploader_->process_record(item->payload, item->tag, emit);
      intent_queue_->ack(item->id);
      if (summary.files > 0)
      {
        RCLCPP_INFO(this->get_logger(),
                    "uploader: group '%s': %zu file(s), %zu verified, %zu missing, %zu deleted, complete=%d",
                    item->tag.c_str(), summary.files, summary.verified, summary.missing, summary.deleted,
                    static_cast<int>(summary.group_complete));
      }
    }
    catch (const std::exception& e)
    {
      // Retryable: the intent stays on disk (ack() never ran), so even a Bridge
      // crash/restart right after this replays it — processing is idempotent (#248).
      intent_queue_->record_failure(item->id);
      RCLCPP_WARN(this->get_logger(), "uploader: %s; will retry intent %s with backoff", e.what(), item->id.c_str());
    }
  }
}

void BridgeNode::run_prober(std::string forward_host, std::uint16_t forward_port)
{
  while (!prober_stop_.load())
  {
    {
      std::lock_guard<std::mutex> lock(supervisor_mutex_);
      supervisor_->poll_restart();
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
  {
    std::lock_guard<std::mutex> lock(uploader_wake_mutex_);
    upload_stop_ = true;
  }
  uploader_wake_cv_.notify_all();
  if (uploader_thread_.joinable())
  {
    uploader_thread_.join();
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
