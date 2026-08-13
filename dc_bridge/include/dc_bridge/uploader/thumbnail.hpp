// Optional derived previews for uploaded image/video Files (#256, ADR-0005 follow-up).
// A dashboard rendering a gallery of inspection photos pulls full-size camera JPEGs (and
// whole video files) just to draw a grid of tiles; a small JPEG preview uploaded next to
// each original — `<remote_path>.thumb.jpg` — is what makes those views load at all over
// a robot-fleet link.
//
// A thumbnail is a *derived* artefact, and derived artefacts in DC follow one rule: they
// are strictly secondary to the data they derive from. So this is opt-in
// (`files.thumbnails.enabled`, off by default), it runs only *after* the original is
// verified on that Destination, and every failure path — no ffmpeg on PATH, an
// undecodable File, a store that rejects the put — is swallowed. A thumbnail can never
// fail, block, retry, or delay the primary upload; at worst it is absent.
//
// Retention (#267) and durability (#265) both fall out of that ordering rather than
// needing their own rules: the local preview is a scratch file under
// `<state_dir>/thumbs/` that is unlinked before process_record returns, so a thumbnail
// never joins the un-uploaded pool retention measures and can never outlive the original
// it came from — a shed intent's Files never uploaded, so no thumbnail was ever derived
// from them. On the remote side the object is idempotent by key: a replayed intent
// after a Bridge restart finds the thumbnail already there (a plain head) and skips
// regeneration entirely.
//
// # The decode dependency
//
// Generation shells out to the **ffmpeg CLI** (argument vector, never `sh -c`), the same
// way `ffprobe_duration_prober` already probes video durations. This is the first place
// the Bridge needs to *decode* pixels rather than read metadata, so the choice is
// deliberate:
//
//   - It is not a new dependency *class*. The Bridge already requires the ffmpeg
//     toolchain at runtime for `files.ffprobe_binary`; `ffmpeg` and `ffprobe` ship in
//     the same package, so enabling thumbnails adds no install that a DC deployment
//     doing video duration probing doesn't already have. (`ffmpeg` is now declared
//     explicitly in package.xml, which the ffprobe use had left implicit.)
//   - It covers both halves of the requirement with one tool: stills for `image/*`
//     (including the Netpbm maps DC produces) and first-frame extraction for `video/*`.
//     A C++ image library (OpenCV, libvips, ImageMagick) would decode stills only and
//     still leave video needing ffmpeg.
//   - It stays out of the link line. `dc_bridge_core` is deliberately dependency-light
//     and unit-tested with plain gtest against in-memory fakes; linking a decoder would
//     put a heavyweight (and CVE-prone) library into every Bridge build, including
//     deployments that never enable this. A subprocess also contains the blast radius —
//     a malformed camera file that crashes a decoder kills a child process, not the
//     Bridge.
//
// The seam is a `std::function`, mirroring `DurationProber`, so the tests drive the
// Uploader's thumbnail path (success, failure, replay) with no ffmpeg present at all.
#ifndef DC_BRIDGE__UPLOADER__THUMBNAIL_HPP_
#define DC_BRIDGE__UPLOADER__THUMBNAIL_HPP_

#include <cstdint>
#include <functional>
#include <string>

namespace dc_bridge::uploader
{

/// Appended to a File's remote path to name its preview, per #256: `<path>.thumb.jpg`.
/// Deterministic on purpose — a consumer that has a File's `remote_path` can construct
/// its preview's key without a second query (the demo dashboard's gallery does exactly
/// that), and the Uploader can recognize an already-derived preview by key alone.
inline constexpr const char* THUMBNAIL_SUFFIX = ".thumb.jpg";

/// Previews are always baseline JPEG, whatever the original was.
inline constexpr const char* THUMBNAIL_CONTENT_TYPE = "image/jpeg";

struct ThumbnailConfig
{
  /// Off by default (#256: "opt-in via the `files` policy config").
  bool enabled = false;
  /// Bound on the preview's longest side, in pixels. Aspect ratio is preserved and a
  /// File already smaller than this is never upscaled.
  std::uint32_t max_dimension = 320;
};

/// Writes a JPEG preview of `source_path` to `dest_path`, bounded to `max_dimension` on
/// its longest side. Returns false if no preview could be produced — for any reason,
/// including the generator binary being absent. Never throws: a preview is best-effort
/// by construction, and the Uploader's error paths are reserved for the primary upload.
using ThumbnailGenerator =
    std::function<bool(const std::string& source_path, const std::string& dest_path, std::uint32_t max_dimension)>;

/// A ThumbnailGenerator shelling out to the ffmpeg CLI (argument vector, not `sh -c`) —
/// see this header's "decode dependency" note. For `video/*` inputs ffmpeg's own
/// decoding yields the first frame, which is what makes one code path cover both stills
/// and video.
ThumbnailGenerator ffmpeg_thumbnail_generator(const std::string& ffmpeg_binary);

/// Whether a preview can be derived from a File with this sniffed content type — images
/// and videos, the two the issue names. Everything else (map YAML, logs, PDFs) is left
/// alone rather than handed to a decoder that would just fail.
bool thumbnailable(const std::string& content_type);

/// Where a File's preview lives on the store, relative to the Destination's key_prefix:
/// `<remote_path>.thumb.jpg`.
std::string thumbnail_remote_path(const std::string& remote_path);

/// Path of the scratch file a preview is generated into before upload, under
/// `<state_dir>/thumbs/`. Named by a hash of (storage, object key) exactly like
/// `multipart::resume_state_path`, so concurrent work on different Files (or the same
/// File on different Destinations) can never collide on one scratch path.
std::string thumbnail_scratch_path(const std::string& state_dir, const std::string& storage_name,
                                   const std::string& object_key);

}  // namespace dc_bridge::uploader

#endif  // DC_BRIDGE__UPLOADER__THUMBNAIL_HPP_
