# The Uploader runs as its own process

**Narrows:** ADR-0005 ("File uploads live in the Bridge, not the shipper"), which is
still true at the *pipeline* level — uploads are a DC/Bridge concern, not something a log
shipper does — but no longer at the *process* level: the Uploader was a module and a
worker thread inside `dc_bridge`; it is now `dc_uploader`, a separate executable.

## Why

Epic #440 (split deployment for fleets) needs the Shipper and the Uploader to run as
independently-restartable units so an orchestrator can give each its own lifecycle,
resource limits, and credentials. Of the two, the Uploader is the one #440 itself singles
out: *"I want the upload daemon to crash and restart freely, so that a failing upload
never takes down data collection"* (user story 18). Inside the Bridge process, that was
never true — an unhandled Uploader-thread failure takes the whole Bridge process down
with it, Record collection included, because both run in the same address space.

It is also the one piece of this split that does not need to wait for containers. The
Bridge's `receives: files` subscription, the durable intent queue (#265), and the
Uploader's upload/verify/delete logic (ADR-0005) were already aws-sdk-free at the
library level (`dc_bridge_core` builds without AWS SDK); only the S3 `ObjectStore`
implementation and the worker thread's wiring held the Uploader inside the Bridge's
address space. Pulling that into its own OS process is a complete, independently useful
step before #447's container work: unmanaged-shipper mode (#444) needed a *second*
process to exist on the robot before split deployment made sense; this is that second
process's first independent capability.

## Decision

- **`dc_uploader`** is a new executable in the `dc_bridge` colcon package (not a new ROS
  package — nothing about it needs `ament_cmake`'s ROS-specific machinery beyond reusing
  the existing build). It links `dc_bridge_core` (the same ROS-free library `dc_bridge`
  itself links) plus the AWS SDK S3 `ObjectStore` implementation, and nothing from
  `rclcpp`/`rclpy`. `dc_bridge` no longer links the AWS SDK at all.
- **Configured entirely by `DC_UPLOADER_*` environment variables**
  (`dc_bridge/uploader/process_config.hpp`), parsed by a pure function
  (`load_uploader_process_config`) that is unit-tested against an in-memory map — no ROS
  parameters, no getenv calls to test around. Queue/state/files directories,
  object-storage endpoint and credentials, the shipper ingest protocol target, and
  delete-when-sent/multipart/thumbnail/retention knobs all move here from the `files.*`/
  `uploader.*` ROS parameters the Bridge used to read for the Uploader's sake. The Bridge
  keeps only `files.metadata_destination` (it still renders Vector's config and has to
  know where the Uploader's status Records route) and `uploader.data_dir` (both processes
  derive the same queue path from it independently, so a deployment does not need to
  write the same path out twice).
- **The Bridge keeps the Files subscription and intent-writing side.** A Record on a
  `receives: files` Destination's topic is parsed, durably enqueued (ADR-0005/#265), and
  forgotten — `dc_uploader` is the only reader. This split exactly where ADR-0005's
  durable queue already drew its own internal seam; no new coupling was invented; the
  queue *is* the interface.
- **`IntentQueue` gains `rescan()`.** The queue's on-disk format and single-writer
  crash-atomicity (#265) were already safe for two processes; what was not is that each
  process's in-memory scheduling state (oldest-first order, per-entry backoff) was
  populated once, at construction, from whatever was on disk *then*. A Bridge process's
  `enqueue()` only updates the Bridge's own in-memory view — a separate `dc_uploader`
  process holding its own `IntentQueue` instance over the same directory never otherwise
  learns a new intent exists. `rescan()` closes that gap: it picks up any `*.json` file
  on disk the instance doesn't already know about, without touching already-known
  entries' backoff state, and `dc_uploader`'s poll loop calls it every cycle (the same
  ~500ms cadence the worker thread used to poll `next_ready()` on).
- **Still one process tree, one machine.** `dc_bringup.launch.py` starts `dc_uploader` as
  a supervised `ExecuteProcess` (the same pattern `dc_mcap_writer` already uses, and for
  the same reason: `ros2 run` does not forward signals to its child), translating the
  `dc_bridge:` params block's single `receives: files` Destination and
  `uploader.data_dir`/`vector_forward_host`/`vector_forward_port`/`files.*` into that
  process's environment. A deployment's params file is unchanged; only the process
  boundary moved. Running `dc_uploader` as a separate container, with its own volumes and
  credentials, is #447's work, not this change's.

## Consequences

- Killing `dc_uploader` no longer touches Record collection at all — there is no shared
  address space left for an Uploader failure to take down. Killing it mid-upload loses
  nothing: the intent that was in flight is still on disk (only `ack()` removes it, and
  that never ran), so the next `dc_uploader` start replays it from `IntentQueue`'s
  existing crash-replay guarantee, unchanged by this ADR.
- The ROS container's own credential surface shrinks: object-storage keys live only in
  `dc_uploader`'s environment, never in the Bridge's. This is epic #440's user story 5
  ("each container receives only the credentials it needs") arriving one process early.
- A deployment with no `receives: files` Destination configured starts no `dc_uploader`
  process at all — `build_uploader_action` returns no action when the params file names
  none, matching the Bridge's own "Uploader only exists when Files are configured"
  behaviour before this change.
- Only one `receives: files` Destination per deployment is supported by the environment-
  variable surface and by `dc_bringup.launch.py`'s translation of it (every params file
  in this repo already configures at most one). Multiple object-storage endpoints behind
  one `dc_uploader` process is out of scope here; it was equally possible and equally
  untested before this change.
