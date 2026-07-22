//! Core Bridge logic (ADRs 0001/0004/0006), kept free of any ROS/rclrs dependency so it
//! can be unit-tested with plain `cargo test` (see README.md). `main.rs` is the thin ROS
//! node that wires these modules to topic subscriptions and a readiness service.

pub mod config;
pub mod forwarder;
pub mod readiness;
pub mod render;
pub mod supervisor;
pub mod vector_config;

pub use config::{BridgeConfig, TopicConfig};
pub use forwarder::{Forwarder, ForwarderConfig, ForwarderError, Record};
pub use readiness::Readiness;
pub use render::{
    destination_from_raw, expand_env, render as render_vector_config, Destination, DestinationKind,
    PostgresParams, RawPostgresParams, Receives, RenderConfig, RenderError, TimeFormat,
    MIN_DISK_BUFFER_BYTES, ROUTE_TRANSFORM_ID,
};
pub use supervisor::{Supervisor, SupervisorConfig};
