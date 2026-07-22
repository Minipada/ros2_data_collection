# The Bridge is a plain node, outside the lifecycle manager

Every other DC node is a nav2-style lifecycle node orchestrated by `dc_lifecycle_manager` (bond heartbeats, autostart). The Bridge deliberately is not: rclrs has no lifecycle-node or bond support, and the Bridge has no meaningful deactivated state — it is infrastructure, like the shipper itself. Startup determinism comes from launch ordering instead: Vector → Bridge → readiness gate → lifecycle manager activates collection nodes. Supervision is launch `respawn` (Bridge) and the Bridge supervising the Vector child process.

## Consequences

- Lifecycle introspection does not cover the Bridge; monitoring it means "process up + ready service answers".
- If rclrs gains lifecycle support, the Bridge can adopt it without changing this boundary.
