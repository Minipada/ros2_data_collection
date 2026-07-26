# Data Pipeline

## DC 2.0 (Jazzy): deterministic startup ordering

On the Jazzy line, `dc_bringup.launch.py` brings the pipeline up in a fixed order
(ADR-0006), so no Record can be emitted before the pipeline is able to accept it:

1. **Bridge first.** `dc_bridge` starts as a plain node (outside the lifecycle
   manager) and spawns the Vector shipper as a supervised child process. The
   measurement server also starts here, but stays unconfigured and inactive — its
   publishers cannot emit anything yet.
2. **Readiness gate.** A `bridge_ready_gate` process blocks polling the Bridge's
   `~/ready` service (`std_srvs/Trigger`), which answers `success=True` only once
   Vector is accepting Fluent Forward connections. The gate's `service`,
   `timeout_s` (default 120 s), and `poll_interval_s` parameters are configurable
   from the params file under `bridge_ready_gate:`.
3. **Activation.** Only when the gate exits successfully does the launch start
   `lifecycle_manager_dc`, which configures and then activates the collection
   nodes. If the Bridge never becomes ready before the gate's deadline, the whole
   launch shuts down loudly instead of leaving a half-started pipeline running.

Supervision after startup: the launch file respawns `dc_bridge` unconditionally
(independent of `use_respawn`), and the Bridge supervises its Vector child —
including a Linux parent-death signal, so Vector can never outlive the Bridge
even across a SIGKILL/crash. Records published while the Bridge is down are
dropped (topics are fire-and-forget); delivery resumes as soon as the respawned
Bridge is ready.
