// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// dc_render (#495): the ROS-free CLI over the render module. `ros2 launch` runs it at
// plan time to emit and stage the Shipper config snippets launch used to hand-write in
// Python, so the `dc.<tag>` topic→route rule (ADR-0003), the passthrough socket-sink
// block (ADR-0009) and the recipe staging rule each have one home, in render.cpp, pinned
// by render_test. Launch wires processes; it re-derives no config fact.
//
//   dc_render socket-sink --sink-id dc_mcap_writer --host 127.0.0.1 --port 9191
//                         [--topic /dc/measurement/cpu]… --output <path> [--origin <text>]
//   dc_render stage --recipe-dir <dir> --path <custom_config_files entry>…

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dc_bridge/atomic_write.hpp"
#include "dc_bridge/render.hpp"

namespace
{

[[noreturn]] void fail(const std::string& message)
{
  std::cerr << "dc_render: " << message << "\n";
  std::exit(2);
}

std::optional<std::string> env_lookup(const std::string& name)
{
  const char* value = ::getenv(name.c_str());
  return value == nullptr ? std::nullopt : std::optional<std::string>(value);
}

// The expansion the Bridge itself applies to a custom_config_files entry (expand_env,
// with a leading `~` on top), so the path this stages and the path the Bridge reads
// cannot disagree about what $HOME/.dc meant.
std::string expand_entry(const std::string& raw)
{
  std::string expanded = dc_bridge::expand_env(raw, env_lookup);
  if (expanded == "~" || expanded.rfind("~/", 0) == 0)
  {
    if (const char* home = ::getenv("HOME"); home != nullptr)
    {
      expanded.replace(0, 1, home);
    }
  }
  return expanded;
}

std::string read_file(const std::string& path)
{
  std::ifstream in(path);
  if (!in.good())
  {
    fail("cannot read '" + path + "'");
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void write_file(const std::string& path, const std::string& content)
{
  const std::filesystem::path parent = std::filesystem::path(path).parent_path();
  std::error_code ec;
  if (!parent.empty())
  {
    std::filesystem::create_directories(parent, ec);
    if (ec)
    {
      fail("cannot create directory '" + parent.string() + "': " + ec.message());
    }
  }
  // Atomic, like the Bridge's own config write (#444): a Bridge (re)starting while a
  // later launch re-renders this file reads it whole or not at all.
  try
  {
    dc_bridge::write_file_atomically(path, content);
  }
  catch (const std::exception& e)
  {
    fail(e.what());
  }
}

bool flag_present(const std::vector<std::string>& args, std::size_t& i, const std::string& flag, std::string& into)
{
  if (args[i] != flag)
  {
    return false;
  }
  if (i + 1 >= args.size())
  {
    fail(flag + " needs a value");
  }
  into = args[++i];
  return true;
}

unsigned short parse_port(const std::string& text)
{
  std::size_t consumed = 0;
  unsigned long port = 0;
  try
  {
    port = std::stoul(text, &consumed);
  }
  catch (const std::exception&)
  {
    fail("--port '" + text + "' is not a port number");
  }
  if (consumed != text.size() || port == 0 || port > 65535)
  {
    fail("--port '" + text + "' is out of range (expected 1-65535)");
  }
  return static_cast<unsigned short>(port);
}

int socket_sink(const std::vector<std::string>& args)
{
  std::string sink_id;
  std::string host;
  std::string port_text;
  std::string output;
  std::string origin;
  std::vector<std::string> topics;
  for (std::size_t i = 0; i < args.size(); ++i)
  {
    if (flag_present(args, i, "--sink-id", sink_id) || flag_present(args, i, "--host", host) ||
        flag_present(args, i, "--port", port_text) || flag_present(args, i, "--output", output) ||
        flag_present(args, i, "--origin", origin))
    {
      continue;
    }
    if (args[i] == "--topic")
    {
      if (i + 1 >= args.size())
      {
        fail("--topic needs a value");
      }
      topics.push_back(args[++i]);
      continue;
    }
    fail("socket-sink: unknown argument '" + args[i] + "'");
  }
  if (output.empty())
  {
    fail("socket-sink: --output is required");
  }

  dc_bridge::SocketSinkParams params;
  params.sink_id = sink_id;
  params.input_topics = topics;
  params.host = host;
  params.origin = origin;
  params.port = port_text.empty() ? 0 : parse_port(port_text);

  try
  {
    write_file(output, dc_bridge::socket_sink_toml(params));
  }
  catch (const dc_bridge::RenderError& e)
  {
    fail(e.what());
  }
  std::cout << "wrote " << output << "\n";
  return 0;
}

int stage(const std::vector<std::string>& args)
{
  std::string recipe_dir;
  std::vector<std::string> entries;
  for (std::size_t i = 0; i < args.size(); ++i)
  {
    if (flag_present(args, i, "--recipe-dir", recipe_dir))
    {
      continue;
    }
    if (args[i] == "--path")
    {
      if (i + 1 >= args.size())
      {
        fail("--path needs a value");
      }
      entries.push_back(args[++i]);
      continue;
    }
    fail("stage: unknown argument '" + args[i] + "'");
  }
  if (recipe_dir.empty())
  {
    fail("stage: --recipe-dir is required");
  }
  if (entries.empty())
  {
    fail("stage: at least one --path is required");
  }

  const std::filesystem::path recipes(recipe_dir);
  for (const auto& raw_entry : entries)
  {
    const std::string path = expand_entry(raw_entry);
    const std::filesystem::path recipe = recipes / std::filesystem::path(path).filename();
    std::error_code ec;
    const bool recipe_exists = std::filesystem::is_regular_file(recipe, ec);
    const bool staged_exists = std::filesystem::exists(std::filesystem::path(path), ec);
    switch (dc_bridge::stage_action(recipe_exists, staged_exists))
    {
      case dc_bridge::StageAction::Skip:
        // No recipe for this entry: a path that ends up missing is the Bridge's own loud
        // "failed to read custom config file" error, and a bind mount is left alone.
        continue;
      case dc_bridge::StageAction::Copy:
        write_file(path, read_file(recipe.string()));
        std::cout << "staged " << recipe.string() << " -> " << path << "\n";
        break;
    }
  }
  return 0;
}

void print_usage(std::ostream& out)
{
  out << "usage:\n"
         "  dc_render socket-sink --sink-id <id> --host <host> --port <port> --output <path>\n"
         "                        [--topic <ros topic>]… [--origin <comment text>]\n"
         "  dc_render stage --recipe-dir <dir> --path <custom_config_files entry>…\n";
}

}  // namespace

int main(int argc, char** argv)
{
  const std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args.front() == "-h" || args.front() == "--help")
  {
    print_usage(args.empty() ? std::cerr : std::cout);
    return args.empty() ? 2 : 0;
  }
  if (args.front() == "socket-sink")
  {
    return socket_sink({ args.begin() + 1, args.end() });
  }
  if (args.front() == "stage")
  {
    return stage({ args.begin() + 1, args.end() });
  }
  fail("unknown command '" + args.front() + "'");
}
