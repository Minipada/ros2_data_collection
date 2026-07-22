//! The `dc_bridge` ROS node: wires the ROS-independent core modules in `dc_bridge_core`
//! (unit-tested standalone, see dc_bridge_core/README via its Cargo.toml doc comment) to
//! topic subscriptions and a readiness service. Only buildable inside a colcon workspace
//! with the ros2_rust (`rclrs`) toolchain — see dc_bridge/README.md.
//!
//! Vector's own configuration (the Fluent Forward source, the timestamp/Tag-normalizing
//! VRL transform, the public per-Tag `dc.<tag>` routes, and the blessed sinks —
//! `postgres`, `s3`, `file`, `console`) is produced by `dc_bridge_core::render`
//! (ADR-0003) from the `shipper` and `destinations` parameters below — this node's job
//! is just to declare those parameters, expand the `$HOME`/`$DC_PG_PASSWORD`-style env
//! references the config contract uses, and hand the result to the Supervisor. The set
//! of topics subscribed is derived from every configured Destination's `inputs`, not a
//! separate parameter. Raw Vector snippets listed in `custom_config_files` (ADR-0003's
//! passthrough) are collision-checked, `vector validate`d together with the rendered
//! config so a bad snippet fails loudly at startup, and passed to Vector as additional
//! `--config` files.
//!
//! Built, run, and `colcon test`-covered (see tests/end_to_end.rs) in a real ROS 2
//! Jazzy + ros2_rust Docker environment: the node locates and supervises the vendored
//! Vector binary, the `~/ready` service answers correctly, a `StringStamped` Record
//! published on a subscribed topic reaches Vector's console sink, and the supervised
//! Vector process stops when dc_bridge receives SIGINT/SIGTERM. See dc_bridge/README.md
//! for the verification recipe and the three real bugs it caught.

use dc_bridge_core::render::{
    destination_from_raw, expand_env, render as render_vector_config, validate_custom_config_files,
    CustomConfigFile, RawDestinationParams, RenderConfig, MIN_DISK_BUFFER_BYTES,
};
use dc_bridge_core::{readiness, vector_config};
use dc_bridge_core::{
    Forwarder, ForwarderConfig, Readiness, Record, Supervisor, SupervisorConfig, TopicConfig,
};
use dc_interfaces::msg::StringStamped;
use rclrs::{CreateBasicExecutor, RclrsErrorFilter};
use std::collections::BTreeSet;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use std_srvs::srv::{Trigger_Request, Trigger_Response};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("dc_bridge")?;

    // rclrs's ParameterVariant is only implemented for Arc<str>/Arc<[Arc<str>]> (not
    // String/Vec<String>), so parameters are declared in those types and converted.
    let vector_host: Arc<str> = node
        .declare_parameter("vector_forward_host")
        .default(Arc::from("127.0.0.1"))
        .mandatory()?
        .get();
    let vector_port: i64 = node
        .declare_parameter("vector_forward_port")
        .default(24224_i64)
        .mandatory()?
        .get();
    let vector_binary_override: Arc<str> = node
        .declare_parameter("vector_binary")
        .default(Arc::from(""))
        .mandatory()?
        .get();
    let forward_addr: SocketAddr = format!("{}:{}", vector_host, vector_port).parse()?;

    let render_config = build_render_config(&node, forward_addr)?;
    let rendered_vector_config = render_vector_config(&render_config)?;

    // ADR-0003 passthrough: raw Vector config snippets merged natively by Vector via
    // additional `--config` arguments. Collision-checked against the rendered config
    // here so a bad snippet is a clear startup error naming the file, not a
    // supervised-Vector crash loop.
    let custom_config_paths: Arc<[Arc<str>]> = node
        .declare_parameter("custom_config_files")
        .default(Arc::from(Vec::<Arc<str>>::new()))
        .mandatory()?
        .get();
    let mut custom_config_files = Vec::with_capacity(custom_config_paths.len());
    for path_raw in custom_config_paths.iter() {
        let path = expand_env(path_raw, |name| std::env::var(name).ok())?;
        let content = std::fs::read_to_string(&path)
            .map_err(|e| format!("failed to read custom config file '{path}': {e}"))?;
        custom_config_files.push(CustomConfigFile { path, content });
    }
    validate_custom_config_files(&render_config, &custom_config_files)?;

    let vector_binary = if !vector_binary_override.is_empty() {
        PathBuf::from(vector_binary_override.to_string())
    } else {
        let ament_prefix_path = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
        vector_config::find_vector_binary(&ament_prefix_path)
            .ok_or("could not locate vendored Vector binary; set the vector_binary parameter")?
    };

    let vector_config_path = std::env::temp_dir().join("dc_bridge_vector.toml");
    std::fs::write(&vector_config_path, rendered_vector_config)?;

    let vector_config_paths: Vec<PathBuf> = std::iter::once(vector_config_path)
        .chain(custom_config_files.iter().map(|f| PathBuf::from(&f.path)))
        .collect();

    let supervisor = Arc::new(Mutex::new(Supervisor::new(SupervisorConfig::vector(
        vector_binary.clone(),
        vector_config_paths.clone(),
    ))));

    // rclrs has no signal handling yet (`Context::ok()` unconditionally returns true, so
    // `executor.spin()` below never returns on its own): without this, dc_bridge exiting
    // via Ctrl-C, `ros2 launch` shutdown, or a plain SIGTERM would leave the supervised
    // Vector process running forever, orphaned. SIGKILL still can't be caught (a Unix
    // limit, not specific to this), but that's now the only path that leaks it.
    // Installed *before* the Supervisor ever starts Vector: `Supervisor::start` is a
    // no-op after `stop`, so a signal landing during startup can't race the handler
    // into leaving an unsupervised Vector behind.
    {
        let supervisor = supervisor.clone();
        ctrlc::set_handler(move || {
            supervisor.lock().unwrap().stop();
            std::process::exit(0);
        })?;
    }

    // Backstop for anything the pure checks above can't see (e.g. a snippet consuming a
    // `dc.<tag>` route no configured Destination subscribes, or sink options Vector
    // itself rejects): run Vector's own validator on the merged config set before
    // starting the Supervisor, so an invalid config is a loud startup failure instead
    // of a crash loop.
    validate_with_vector(&vector_binary, &vector_config_paths)?;

    supervisor.lock().unwrap().start()?;

    let readiness = Readiness::new();
    {
        let readiness = readiness.clone();
        let supervisor = supervisor.clone();
        std::thread::spawn(move || loop {
            let _ = supervisor.lock().unwrap().poll_restart();
            let ready = readiness::probe(forward_addr, Duration::from_millis(200));
            readiness.set_ready(ready);
            std::thread::sleep(Duration::from_millis(250));
        });
    }

    let forwarder = Arc::new(Mutex::new(Forwarder::new(ForwarderConfig::new(
        forward_addr,
    ))));

    // Every Destination's `inputs` topic is subscribed to exactly once, even if more
    // than one Destination consumes it.
    let mut subscribed_topics = BTreeSet::new();
    for destination in &render_config.destinations {
        for topic in &destination.inputs {
            subscribed_topics.insert(topic.clone());
        }
    }

    // Held so subscriptions aren't dropped once out of scope; the node keeps them alive.
    let mut _subscriptions = Vec::new();
    for topic in subscribed_topics {
        let topic_config = TopicConfig::new(topic, None);
        let forwarder = forwarder.clone();
        let tag = topic_config.tag.clone();
        let subscription =
            node.create_subscription(topic_config.topic.as_str(), move |msg: StringStamped| {
                let timestamp = header_stamp_to_system_time(&msg.header);
                let payload = serde_json::from_str(&msg.data)
                    .unwrap_or_else(|_| serde_json::Value::String(msg.data.clone()));
                let record = Record::new(tag.clone(), timestamp, payload);
                if let Err(e) = forwarder.lock().unwrap().send(&record) {
                    eprintln!(
                        "dc_bridge: failed to forward record on tag '{}': {}",
                        tag, e
                    );
                }
            })?;
        _subscriptions.push(subscription);
    }

    let _readiness_service = node.create_service::<std_srvs::srv::Trigger, _>(
        "~/ready",
        move |_request: Trigger_Request, _info: rclrs::ServiceInfo| Trigger_Response {
            success: readiness.is_ready(),
            message: if readiness.is_ready() {
                "vector is accepting connections".to_string()
            } else {
                "vector is not ready".to_string()
            },
        },
    )?;

    executor.spin(rclrs::SpinOptions::default()).first_error()?;
    Ok(())
}

/// Runs `vector validate --no-environment` on the full `--config` set (rendered config
/// plus passthrough snippets) and turns a failure into a startup error carrying
/// Vector's own diagnostics. `--no-environment` skips healthchecks and
/// environment-dependent checks (ports, data_dir, remote endpoints) — this is a config
/// correctness gate, not a connectivity probe.
fn validate_with_vector(
    vector_binary: &std::path::Path,
    config_paths: &[PathBuf],
) -> Result<(), Box<dyn std::error::Error>> {
    let output = std::process::Command::new(vector_binary)
        .args(["validate", "--no-environment"])
        .args(config_paths.iter().map(|p| p.as_os_str()))
        .output()
        .map_err(|e| format!("failed to run '{}' validate: {e}", vector_binary.display()))?;
    if !output.status.success() {
        return Err(format!(
            "vector rejected the merged configuration:\n{}{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        )
        .into());
    }
    Ok(())
}

/// Declares the `shipper`/`destinations` parameters (ADR-0003's config contract) and
/// builds the [`RenderConfig`] the config renderer needs. `$NAME`/`${NAME}` references in
/// `shipper.data_dir` and each Postgres destination's `password` are expanded against the
/// real process environment (`std::env::var`) here — `dc_bridge_core::render` itself
/// never touches the environment, so it stays pure and testable without ROS or Vector.
fn build_render_config(
    node: &rclrs::Node,
    forward_addr: SocketAddr,
) -> Result<RenderConfig, Box<dyn std::error::Error>> {
    let data_dir_raw: Arc<str> = node
        .declare_parameter("shipper.data_dir")
        .default(Arc::from("$HOME/.dc/buffer"))
        .mandatory()?
        .get();
    let data_dir = expand_env(&data_dir_raw, |name| std::env::var(name).ok())?;

    let buffer_max_bytes: i64 = node
        .declare_parameter("shipper.buffer_max_bytes")
        .default(MIN_DISK_BUFFER_BYTES as i64)
        .mandatory()?
        .get();

    let destination_names: Arc<[Arc<str>]> = node
        .declare_parameter("destinations")
        .default(Arc::from(Vec::<Arc<str>>::new()))
        .mandatory()?
        .get();

    let mut destinations = Vec::with_capacity(destination_names.len());
    for name in destination_names.iter() {
        destinations.push(declare_destination(node, name)?);
    }

    Ok(RenderConfig {
        forward_addr,
        data_dir,
        buffer_max_bytes: buffer_max_bytes.max(0) as u64,
        destinations,
    })
}

/// Declares every `<name>.*` parameter one Destination might need — the flat union
/// across all blessed types, mirroring `RawDestinationParams` — and validates them into
/// a typed `Destination` via `dc_bridge_core::render::destination_from_raw` — this is
/// where an invalid `type`/`receives`/`time_format`, or a missing required field, turns
/// into the clear startup error the acceptance criteria call for, rather than a
/// confusing failure deep inside Vector.
fn declare_destination(
    node: &rclrs::Node,
    name: &str,
) -> Result<dc_bridge_core::render::Destination, Box<dyn std::error::Error>> {
    let declare_str = |param: &str| -> Result<Option<Arc<str>>, Box<dyn std::error::Error>> {
        let value: Option<Arc<str>> = node
            .declare_parameter(format!("{name}.{param}"))
            .optional()?
            .get();
        Ok(value)
    };

    let type_str: Arc<str> = node
        .declare_parameter(format!("{name}.type"))
        .default(Arc::from(""))
        .mandatory()?
        .get();
    let receives_str: Arc<str> = node
        .declare_parameter(format!("{name}.receives"))
        .default(Arc::from("records"))
        .mandatory()?
        .get();
    let inputs: Arc<[Arc<str>]> = node
        .declare_parameter(format!("{name}.inputs"))
        .default(Arc::from(Vec::<Arc<str>>::new()))
        .mandatory()?
        .get();

    let time_key = declare_str("time_key")?;
    let time_format = declare_str("time_format")?;
    // postgres
    let host = declare_str("host")?;
    let port: Option<i64> = node
        .declare_parameter(format!("{name}.port"))
        .optional()?
        .get();
    let user = declare_str("user")?;
    let password_raw = declare_str("password")?;
    let database = declare_str("database")?;
    let table = declare_str("table")?;
    // s3
    let bucket = declare_str("bucket")?;
    let region = declare_str("region")?;
    let endpoint = declare_str("endpoint")?;
    let key_prefix = declare_str("key_prefix")?;
    let access_key_id = declare_str("access_key_id")?;
    let secret_access_key_raw = declare_str("secret_access_key")?;
    let force_path_style: Option<bool> = node
        .declare_parameter(format!("{name}.force_path_style"))
        .optional()?
        .get();
    let batch_timeout_secs: Option<i64> = node
        .declare_parameter(format!("{name}.batch_timeout_secs"))
        .optional()?
        .get();
    // file
    let path = declare_str("path")?;

    // Credentials support `$DC_PG_PASSWORD`-style env references (ADR-0003 contract).
    let password = password_raw
        .map(|p| expand_env(&p, |var| std::env::var(var).ok()))
        .transpose()?;
    let secret_access_key = secret_access_key_raw
        .map(|s| expand_env(&s, |var| std::env::var(var).ok()))
        .transpose()?;

    let raw = RawDestinationParams {
        time_key: time_key.as_deref(),
        time_format: time_format.as_deref(),
        host: host.as_deref(),
        port,
        user: user.as_deref(),
        password: password.as_deref(),
        database: database.as_deref(),
        table: table.as_deref(),
        bucket: bucket.as_deref(),
        region: region.as_deref(),
        endpoint: endpoint.as_deref(),
        key_prefix: key_prefix.as_deref(),
        access_key_id: access_key_id.as_deref(),
        secret_access_key: secret_access_key.as_deref(),
        force_path_style,
        batch_timeout_secs,
        path: path.as_deref(),
    };

    let inputs: Vec<String> = inputs.iter().map(|s| s.to_string()).collect();
    Ok(destination_from_raw(
        name,
        &type_str,
        &receives_str,
        inputs,
        raw,
    )?)
}

fn header_stamp_to_system_time(header: &std_msgs::msg::Header) -> SystemTime {
    let secs = header.stamp.sec.max(0) as u64;
    let nanos = header.stamp.nanosec;
    UNIX_EPOCH + Duration::new(secs, nanos)
}
