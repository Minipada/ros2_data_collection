# The Bridge is written in Rust (rclrs) as a contained pilot; the rest stays C++

We want first-party Rust in the project, but a full core rewrite was rejected: DC's value is its dynamically-loaded plugin ecosystem, and Rust has no stable ABI for dynamic plugins, while rclrs still lacks lifecycle nodes, message_filters, and pluginlib equivalents — a rewrite would be a 6–12 month redesign that discards 30+ tested C++ plugins. Instead, the **new Bridge node is Rust**: new code, no pluginlib dependency, minimal ROS surface (subscribe, socket forward, config render, child-process supervision), squarely within rclrs's capability. Measurements, conditions, groups, and lifecycle management remain C++.

## Consequences

- `cargo`/rustup joins the source-build toolchain (while Go and the C fork leave — net fewer toolchains than Humble).
- If the pilot proves out and rclrs matures, Rust can expand component-by-component from a working system; if not, the loss is one small package.
