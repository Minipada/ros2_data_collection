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
//! `receives: files` Destinations (ADR-0005) are not rendered into the Vector config at
//! all: Records on their `inputs` topics go to the Bridge's own Uploader
//! (`dc_bridge_core::uploader`, worker thread spawned by [`spawn_uploader_worker`]),
//! which uploads each referenced File to every such Destination via `object_store`,
//! verifies it landed, and emits per-File status plus group-completion metadata Records
//! back through the Forwarder under the `dc.files` Tag — routed by the rendered config
//! to the `files.metadata_destination` Destination like any other Records.
//!
//! Built, run, and `colcon test`-covered (see tests/end_to_end.rs) in a real ROS 2
//! Jazzy + ros2_rust Docker environment: the node locates and supervises the vendored
//! Vector binary, the `~/ready` service answers correctly, a `StringStamped` Record
//! published on a subscribed topic reaches Vector's console sink, and the supervised
//! Vector process stops when dc_bridge receives SIGINT/SIGTERM. See dc_bridge/README.md
//! for the verification recipe and the three real bugs it caught.

use dc_bridge_core::render::{
    destination_from_raw, expand_env, render as render_vector_config, validate_custom_config_files,
    CustomConfigFile, Destination, DestinationKind, RawDestinationParams, Receives, RenderConfig,
    MIN_DISK_BUFFER_BYTES,
};
use dc_bridge_core::{ffprobe_duration_prober, readiness, vector_config};
use dc_bridge_core::{
    Forwarder, ForwarderConfig, Readiness, Record, Storage, Supervisor, SupervisorConfig,
    TopicConfig, Uploader, UploaderConfig, FILE_STATUS_TAG,
};
use dc_interfaces::msg::StringStamped;
use rclrs::{CreateBasicExecutor, RclrsErrorFilter};
use std::collections::BTreeSet;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{mpsc, Arc, Mutex};
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

    let bridge_config = build_bridge_config(&node, forward_addr)?;
    let render_config = &bridge_config.render_config;
    let rendered_vector_config = render_vector_config(render_config)?;

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
    validate_custom_config_files(render_config, &custom_config_files)?;

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

    // The Uploader (ADR-0005): `receives: files` Destinations are not Vector sinks —
    // the Bridge itself uploads the Files those Records reference, verifies them, and
    // emits status/metadata Records through the Forwarder under FILE_STATUS_TAG. A
    // dedicated worker thread keeps slow uploads off the subscription callbacks.
    // Mutex-wrapped because `mpsc::Sender` is Send but not Sync, while subscription
    // callbacks (like the Forwarder's) may be shared across executor threads.
    let uploader_tx = if bridge_config.files_destinations.is_empty() {
        None
    } else {
        Some(Arc::new(Mutex::new(spawn_uploader_worker(
            &bridge_config,
            forward_addr,
        )?)))
    };

    // Tags forwarded to Vector (Records-destination inputs) vs. tags scanned for File
    // references (files-destination inputs). A topic can be in both sets.
    let records_tags: BTreeSet<String> = render_config
        .destinations
        .iter()
        .flat_map(|d| d.inputs.iter())
        .map(|topic| TopicConfig::new(topic.clone(), None).tag)
        .collect();
    let files_tags: BTreeSet<String> = bridge_config
        .files_destinations
        .iter()
        .flat_map(|d| d.inputs.iter())
        .map(|topic| TopicConfig::new(topic.clone(), None).tag)
        .collect();

    // Every Destination's `inputs` topic is subscribed to exactly once, even if more
    // than one Destination consumes it.
    let mut subscribed_topics = BTreeSet::new();
    for destination in render_config
        .destinations
        .iter()
        .chain(&bridge_config.files_destinations)
    {
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
        let forward_to_vector = records_tags.contains(&tag);
        let uploader_tx = files_tags
            .contains(&tag)
            .then(|| uploader_tx.clone())
            .flatten();
        let subscription =
            node.create_subscription(topic_config.topic.as_str(), move |msg: StringStamped| {
                let timestamp = header_stamp_to_system_time(&msg.header);
                let payload = serde_json::from_str(&msg.data)
                    .unwrap_or_else(|_| serde_json::Value::String(msg.data.clone()));
                if let Some(tx) = &uploader_tx {
                    let _ = tx.lock().unwrap().send((tag.clone(), payload.clone()));
                }
                if forward_to_vector {
                    let record = Record::new(tag.clone(), timestamp, payload);
                    if let Err(e) = forwarder.lock().unwrap().send(&record) {
                        eprintln!(
                            "dc_bridge: failed to forward record on tag '{}': {}",
                            tag, e
                        );
                    }
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

/// Everything the parameter contract configures: the Shipper-side [`RenderConfig`]
/// (Records destinations only), the `receives: files` Destinations the Uploader
/// handles (ADR-0005), and the `files.*` Uploader options.
struct BridgeConfig {
    render_config: RenderConfig,
    files_destinations: Vec<Destination>,
    files: FilesParams,
}

/// The `files.*` parameter block from the issue-#248 config contract.
struct FilesParams {
    delete_when_sent: bool,
    ffprobe_binary: String,
    multipart_threshold_bytes: Option<i64>,
    multipart_part_size_bytes: Option<i64>,
}

/// Declares the `shipper`/`destinations`/`files` parameters (ADR-0003's config
/// contract plus ADR-0005's Uploader) and builds the [`BridgeConfig`]. `$NAME`/
/// `${NAME}` references in `shipper.data_dir` and destination credentials are expanded
/// against the real process environment (`std::env::var`) here — `dc_bridge_core`
/// itself never touches the environment, so it stays pure and testable without ROS or
/// Vector.
fn build_bridge_config(
    node: &rclrs::Node,
    forward_addr: SocketAddr,
) -> Result<BridgeConfig, Box<dyn std::error::Error>> {
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

    // `receives: files` Destinations are the Uploader's (ADR-0005), never Vector sinks.
    let (files_destinations, mut records_destinations): (Vec<_>, Vec<_>) = destinations
        .into_iter()
        .partition(|d| d.receives == Receives::Files);

    let delete_when_sent: bool = node
        .declare_parameter("files.delete_when_sent")
        .default(false)
        .mandatory()?
        .get();
    let ffprobe_binary: Arc<str> = node
        .declare_parameter("files.ffprobe_binary")
        .default(Arc::from("ffprobe"))
        .mandatory()?
        .get();
    let multipart_threshold_bytes: Option<i64> = node
        .declare_parameter("files.multipart_threshold_bytes")
        .optional()?
        .get();
    let multipart_part_size_bytes: Option<i64> = node
        .declare_parameter("files.multipart_part_size_bytes")
        .optional()?
        .get();
    let files = FilesParams {
        delete_when_sent,
        ffprobe_binary: ffprobe_binary.to_string(),
        multipart_threshold_bytes,
        multipart_part_size_bytes,
    };
    let metadata_destination: Arc<str> = node
        .declare_parameter("files.metadata_destination")
        .default(Arc::from(""))
        .mandatory()?
        .get();

    if !files_destinations.is_empty() {
        // The Uploader's status Records must land somewhere: the configured
        // metadata destination gets the Uploader's Tag appended to its routed set.
        if metadata_destination.is_empty() {
            return Err(
                "`files.metadata_destination` is required when a `receives: files` \
                 destination is configured"
                    .into(),
            );
        }
        let target = records_destinations
            .iter_mut()
            .find(|d| d.name.as_str() == metadata_destination.as_ref())
            .ok_or_else(|| {
                format!(
                    "files.metadata_destination '{metadata_destination}' does not name a \
                     configured `receives: records` destination"
                )
            })?;
        target.extra_tags.push(FILE_STATUS_TAG.to_string());
    }

    Ok(BridgeConfig {
        render_config: RenderConfig {
            forward_addr,
            data_dir,
            buffer_max_bytes: buffer_max_bytes.max(0) as u64,
            destinations: records_destinations,
        },
        files_destinations,
        files,
    })
}

/// Builds the Uploader from the `receives: files` Destinations and starts its worker
/// thread. The returned channel receives `(tag, payload)` pairs from the topic
/// subscriptions; the worker retries each Record (idempotently — verified objects are
/// never re-uploaded, status rows never duplicated) with capped backoff until its
/// Files are all verified, and emits status Records through its own Forwarder
/// connection under [`FILE_STATUS_TAG`].
fn spawn_uploader_worker(
    bridge_config: &BridgeConfig,
    forward_addr: SocketAddr,
) -> Result<mpsc::Sender<(String, serde_json::Value)>, Box<dyn std::error::Error>> {
    let mut storages = Vec::with_capacity(bridge_config.files_destinations.len());
    for dest in &bridge_config.files_destinations {
        let DestinationKind::S3(params) = &dest.kind else {
            // destination_from_raw enforces `type: s3` for `receives: files`.
            return Err(format!(
                "internal error: files destination '{}' is not object storage",
                dest.name
            )
            .into());
        };
        storages.push(Storage::s3(&dest.name, params)?);
    }

    let mut config = UploaderConfig::new(
        PathBuf::from(&bridge_config.render_config.data_dir).join("uploader"),
        bridge_config.files.delete_when_sent,
    );
    if let Some(threshold) = bridge_config.files.multipart_threshold_bytes {
        config.multipart_threshold_bytes = threshold.max(1) as u64;
    }
    if let Some(part_size) = bridge_config.files.multipart_part_size_bytes {
        config.multipart_part_size_bytes = part_size.max(1) as u64;
    }
    let prober = ffprobe_duration_prober(PathBuf::from(&bridge_config.files.ffprobe_binary));
    let uploader = Uploader::new(config, storages, prober)?;

    let (tx, rx) = mpsc::channel::<(String, serde_json::Value)>();
    std::thread::spawn(move || {
        // The Uploader's own Forwarder connection, so long uploads never hold the
        // subscription callbacks' Forwarder lock.
        let mut forwarder = Forwarder::new(ForwarderConfig::new(forward_addr));
        let mut emit = move |row: serde_json::Value| -> Result<(), String> {
            let record = Record::new(FILE_STATUS_TAG, SystemTime::now(), row);
            let mut last_err = String::new();
            // Vector may be briefly down (restart, backpressure); keep trying for a
            // while before handing the whole Record back for an idempotent retry.
            for _ in 0..120 {
                match forwarder.send(&record) {
                    Ok(()) => return Ok(()),
                    Err(e) => {
                        last_err = e.to_string();
                        std::thread::sleep(Duration::from_millis(500));
                    }
                }
            }
            Err(last_err)
        };

        while let Ok((tag, payload)) = rx.recv() {
            let mut backoff = Duration::from_secs(1);
            loop {
                match uploader.process_record(&payload, &tag, &mut emit) {
                    Ok(summary) => {
                        if summary.files > 0 {
                            eprintln!(
                                "dc_bridge uploader: group '{}': {} file(s), {} verified, \
                                 {} missing, {} deleted, complete={}",
                                tag,
                                summary.files,
                                summary.verified,
                                summary.missing,
                                summary.deleted,
                                summary.group_complete
                            );
                        }
                        break;
                    }
                    Err(e) => {
                        eprintln!("dc_bridge uploader: {e}; retrying in {backoff:?}");
                        std::thread::sleep(backoff);
                        backoff = (backoff * 2).min(Duration::from_secs(60));
                    }
                }
            }
        }
    });
    Ok(tx)
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
