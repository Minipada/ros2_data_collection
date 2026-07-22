//! The `dc_bridge` ROS node: wires the ROS-independent core modules in `dc_bridge_core`
//! (unit-tested standalone, see dc_bridge_core/README via its Cargo.toml doc comment) to
//! topic subscriptions and a readiness service. Only buildable inside a colcon workspace
//! with the ros2_rust (`rclrs`) toolchain — see dc_bridge/README.md.
//!
//! Vector's own configuration (the Fluent Forward source, the timestamp/Tag-normalizing
//! VRL transform, the `dc.<name>` route per Destination, and the blessed sinks
//! themselves) is produced by `dc_bridge_core::render` (ADR-0003) from the `shipper` and
//! `destinations` parameters below — this node's job is just to declare those
//! parameters, expand the `$HOME`/`$DC_PG_PASSWORD`-style env references the config
//! contract uses, and hand the result to the Supervisor. The set of topics subscribed
//! is derived from every configured Destination's `inputs`, not a separate parameter.
//!
//! Built, run, and `colcon test`-covered (see tests/end_to_end.rs) in a real ROS 2
//! Jazzy + ros2_rust Docker environment: the node locates and supervises the vendored
//! Vector binary, the `~/ready` service answers correctly, a `StringStamped` Record
//! published on a subscribed topic reaches Vector's console sink, and the supervised
//! Vector process stops when dc_bridge receives SIGINT/SIGTERM. See dc_bridge/README.md
//! for the verification recipe and the three real bugs it caught.

use dc_bridge_core::render::{
    destination_from_raw, expand_env, render as render_vector_config, RawPostgresParams,
    RenderConfig, MIN_DISK_BUFFER_BYTES,
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

    let vector_binary = if !vector_binary_override.is_empty() {
        PathBuf::from(vector_binary_override.to_string())
    } else {
        let ament_prefix_path = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
        vector_config::find_vector_binary(&ament_prefix_path)
            .ok_or("could not locate vendored Vector binary; set the vector_binary parameter")?
    };

    let vector_config_path = std::env::temp_dir().join("dc_bridge_vector.toml");
    std::fs::write(&vector_config_path, rendered_vector_config)?;

    let supervisor = Arc::new(Mutex::new(Supervisor::new(SupervisorConfig::vector(
        vector_binary,
        vector_config_path,
    ))));
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

    // rclrs has no signal handling yet (`Context::ok()` unconditionally returns true, so
    // `executor.spin()` below never returns on its own): without this, dc_bridge exiting
    // via Ctrl-C, `ros2 launch` shutdown, or a plain SIGTERM would leave the supervised
    // Vector process running forever, orphaned. SIGKILL still can't be caught (a Unix
    // limit, not specific to this), but that's now the only path that leaks it.
    {
        let supervisor = supervisor.clone();
        ctrlc::set_handler(move || {
            supervisor.lock().unwrap().stop();
            std::process::exit(0);
        })?;
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

/// Declares every `<name>.*` parameter one Destination needs and validates them into a
/// typed `Destination` via `dc_bridge_core::render::destination_from_raw` — this is
/// where an invalid `type`/`receives`/`time_format`, or a missing required Postgres
/// field, turns into the clear startup error the acceptance criteria call for, rather
/// than a confusing failure deep inside Vector.
fn declare_destination(
    node: &rclrs::Node,
    name: &str,
) -> Result<dc_bridge_core::render::Destination, Box<dyn std::error::Error>> {
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

    let host: Option<Arc<str>> = node
        .declare_parameter(format!("{name}.host"))
        .optional()?
        .get();
    let port: Option<i64> = node
        .declare_parameter(format!("{name}.port"))
        .optional()?
        .get();
    let user: Option<Arc<str>> = node
        .declare_parameter(format!("{name}.user"))
        .optional()?
        .get();
    let password_raw: Option<Arc<str>> = node
        .declare_parameter(format!("{name}.password"))
        .optional()?
        .get();
    let database: Option<Arc<str>> = node
        .declare_parameter(format!("{name}.database"))
        .optional()?
        .get();
    let table: Option<Arc<str>> = node
        .declare_parameter(format!("{name}.table"))
        .optional()?
        .get();
    let time_key: Option<Arc<str>> = node
        .declare_parameter(format!("{name}.time_key"))
        .optional()?
        .get();
    let time_format: Option<Arc<str>> = node
        .declare_parameter(format!("{name}.time_format"))
        .optional()?
        .get();

    let password = password_raw
        .map(|p| expand_env(&p, |var| std::env::var(var).ok()))
        .transpose()?;

    let postgres = RawPostgresParams {
        host: host.as_deref(),
        port,
        user: user.as_deref(),
        password: password.as_deref(),
        database: database.as_deref(),
        table: table.as_deref(),
        time_key: time_key.as_deref(),
        time_format: time_format.as_deref(),
    };

    let inputs: Vec<String> = inputs.iter().map(|s| s.to_string()).collect();
    Ok(destination_from_raw(
        name,
        &type_str,
        &receives_str,
        inputs,
        postgres,
    )?)
}

fn header_stamp_to_system_time(header: &std_msgs::msg::Header) -> SystemTime {
    let secs = header.stamp.sec.max(0) as u64;
    let nanos = header.stamp.nanosec;
    UNIX_EPOCH + Duration::new(secs, nanos)
}
