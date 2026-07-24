//! Core Bridge logic (ADRs 0001/0004/0006), kept free of any ROS/rclrs dependency so it
//! can be unit-tested with plain `cargo test` (see README.md). `main.rs` is the thin ROS
//! node that wires these modules to topic subscriptions and a readiness service.

pub mod config;
pub mod forwarder;
pub mod readiness;
pub mod render;
pub mod supervisor;
pub mod uploader;
pub mod vector_config;

pub use config::{BridgeConfig, TopicConfig};
pub use forwarder::{Forwarder, ForwarderConfig, ForwarderError, Record};
pub use readiness::Readiness;
pub use render::{
    destination_from_raw, expand_env, render as render_vector_config, route_output_for_tag,
    validate_custom_config_files, CustomConfigFile, Destination, DestinationKind, FileParams,
    PostgresParams, RawDestinationParams, Receives, RenderConfig, RenderError, S3Auth, S3Params,
    TimeFormat, MIN_DISK_BUFFER_BYTES, ROUTE_TRANSFORM_ID,
};
pub use supervisor::{Supervisor, SupervisorConfig};
pub use uploader::store::{Storage, UploadStore};
pub use uploader::{
    ffprobe_duration_prober, ProcessSummary, UploadError, Uploader, UploaderConfig, FILE_STATUS_TAG,
};
