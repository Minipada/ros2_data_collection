// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/uploader/thumbnail.hpp"

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <string>
#include <vector>

#include "dc_bridge/uploader/content_type.hpp"

namespace dc_bridge::uploader
{

namespace
{

/// Runs ffmpeg with the given argument vector (no shell — nothing here is ever
/// interpreted as a command line, so a File path containing shell metacharacters is
/// just a path). stdout/stderr go to /dev/null: ffmpeg's diagnostics are not actionable
/// for a best-effort preview, and letting them inherit the Bridge's stderr would spam
/// the node log once per undecodable File. True iff ffmpeg exited 0.
bool run_ffmpeg(const std::vector<std::string>& argv)
{
  pid_t pid = ::fork();
  if (pid < 0)
  {
    return false;
  }
  if (pid == 0)
  {
    int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull >= 0)
    {
      ::dup2(devnull, STDOUT_FILENO);
      ::dup2(devnull, STDERR_FILENO);
      ::close(devnull);
    }
    std::vector<char*> raw;
    raw.reserve(argv.size() + 1);
    for (const auto& a : argv)
    {
      raw.push_back(const_cast<char*>(a.c_str()));
    }
    raw.push_back(nullptr);
    ::execvp(raw[0], raw.data());
    _exit(127);  // binary not on PATH — reported as a plain generation failure.
  }
  int status = 0;
  ::waitpid(pid, &status, 0);
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

}  // namespace

ThumbnailGenerator ffmpeg_thumbnail_generator(const std::string& ffmpeg_binary)
{
  return [ffmpeg_binary](const std::string& source_path, const std::string& dest_path,
                         std::uint32_t max_dimension) -> bool {
    const std::string bound = std::to_string(max_dimension == 0 ? 1u : max_dimension);
    // `force_original_aspect_ratio=decrease` fits the frame inside the w×h box while
    // preserving its aspect ratio; sizing that box as min(source, bound) rather than the
    // bound itself is what stops a File already smaller than `max_dimension` from being
    // *up*scaled into a larger "thumbnail" than the original. The `\,` escapes are
    // ffmpeg's own filter-expression escaping (a bare comma separates filters in the
    // graph) — not shell escaping; nothing here goes through a shell.
    const std::string scale =
        "scale=w=min(iw\\," + bound + "):h=min(ih\\," + bound + "):force_original_aspect_ratio=decrease";

    // -nostdin: never block waiting on the Bridge's own stdin.
    // -y: overwrite a scratch file left behind by a killed previous attempt.
    // -frames:v 1: one frame — the still itself, or a video's first frame, which is
    //   what lets a single invocation cover both halves of #256.
    // -an / -f image2 / -c:v mjpeg: baseline JPEG out, whatever went in.
    const std::vector<std::string> argv{ ffmpeg_binary, "-nostdin", "-loglevel", "error",     "-y",     "-i",
                                         source_path,   "-vf",      scale,       "-frames:v", "1",      "-an",
                                         "-f",          "image2",   "-c:v",      "mjpeg",     dest_path };
    return run_ffmpeg(argv);
  };
}

bool thumbnailable(const std::string& content_type)
{
  return content_type::is_image(content_type) || content_type::is_video(content_type);
}

std::string thumbnail_remote_path(const std::string& remote_path)
{
  return remote_path + THUMBNAIL_SUFFIX;
}

std::string thumbnail_scratch_path(const std::string& state_dir, const std::string& storage_name,
                                   const std::string& object_key)
{
  // FNV-1a over storage_name, a NUL, then object_key — the same construction
  // multipart::resume_state_path uses, for the same reason: an object key is not a legal
  // filename, and two different keys must never map to one scratch path.
  std::uint64_t hash = 0xcbf29ce484222325ULL;
  auto mix = [&hash](unsigned char byte) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 0x100000001b3ULL;
  };
  for (unsigned char c : storage_name)
  {
    mix(c);
  }
  mix(0);
  for (unsigned char c : object_key)
  {
    mix(c);
  }
  char name[32];
  std::snprintf(name, sizeof(name), "thumb-%016lx.jpg", static_cast<unsigned long>(hash));
  const std::string sep = (!state_dir.empty() && state_dir.back() == '/') ? "" : "/";
  return state_dir + sep + name;
}

}  // namespace dc_bridge::uploader
