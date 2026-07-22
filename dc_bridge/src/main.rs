//! The `dc_bridge` ROS node: wires the ROS-independent core modules in `dc_bridge_core`
//! (unit-tested standalone, see dc_bridge_core/README via its Cargo.toml doc comment) to
//! topic subscriptions and a readiness service. Only buildable inside a colcon workspace
//! with the ros2_rust (`rclrs`) toolchain — see dc_bridge/README.md.
//!
//! Built, run, and `colcon test`-covered (see tests/end_to_end.rs) in a real ROS 2
//! Jazzy + ros2_rust Docker environment: the node locates and supervises the vendored
//! Vector binary, the `~/ready` service answers correctly, a `StringStamped` Record
//! published on a subscribed topic reaches Vector's console sink, and the supervised
//! Vector process stops when dc_bridge receives SIGINT/SIGTERM. See dc_bridge/README.md
//! for the verification recipe and the three real bugs it caught.

use dc_bridge_core::{readiness, vector_config};
use dc_bridge_core::{
    BridgeConfig, Forwarder, ForwarderConfig, Readiness, Record, Supervisor, SupervisorConfig,
    TopicConfig,
};
use dc_interfaces::msg::StringStamped;
use rclrs::{CreateBasicExecutor, RclrsErrorFilter};
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
    let topics: Arc<[Arc<str>]> = node
        .declare_parameter("topics")
        .default(Arc::from(Vec::<Arc<str>>::new()))
        .mandatory()?
        .get();
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

    let config = BridgeConfig {
        topics: topics
            .iter()
            .map(|t| TopicConfig::new(t.to_string(), None))
            .collect(),
        vector_forward_addr: format!("{}:{}", vector_host, vector_port),
    };
    let forward_addr: SocketAddr = config.vector_forward_addr.parse()?;

    let vector_binary = if !vector_binary_override.is_empty() {
        PathBuf::from(vector_binary_override.to_string())
    } else {
        let ament_prefix_path = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
        vector_config::find_vector_binary(&ament_prefix_path)
            .ok_or("could not locate vendored Vector binary; set the vector_binary parameter")?
    };

    let vector_config_path = std::env::temp_dir().join("dc_bridge_vector.toml");
    std::fs::write(&vector_config_path, vector_config::render(forward_addr))?;

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

    // Held so subscriptions aren't dropped once out of scope; the node keeps them alive.
    let mut _subscriptions = Vec::new();
    for topic_config in config.topics {
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

fn header_stamp_to_system_time(header: &std_msgs::msg::Header) -> SystemTime {
    let secs = header.stamp.sec.max(0) as u64;
    let nanos = header.stamp.nanosec;
    UNIX_EPOCH + Duration::new(secs, nanos)
}
