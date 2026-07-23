//! End-to-end workflow tests for the actual ROS node: spawns the real `dc_bridge` binary
//! (which locates and supervises the real vendored Vector binary), waits for the
//! readiness service to report true, publishes a Record on a subscribed topic, and
//! asserts the pipeline behaves as expected — automating the manual verification done
//! for #244/#245 (see dc_bridge/README.md) as a real `colcon test` / `cargo test`.
//!
//! Requires a sourced ROS 2 + rclrs environment (this is what `colcon test
//! --packages-select dc_bridge` runs under the hood, via colcon-ros-cargo's
//! `AmentCargoTestTask`, which is just `cargo test`); it cannot run in a plain sandbox.
//!
//! Since the config renderer (ADR-0003) only produces blessed-Destination sinks — no
//! more bare tracer-bullet console sink — most tests here configure dc_bridge with one
//! `postgres`-type destination. Two tests only need Vector to have started successfully
//! (readiness, shutdown behavior) and work against a Postgres endpoint that need not
//! actually be reachable; the Postgres- and S3-store-backed (RustFS) tests verify a
//! Record actually lands in the real downstream store, and **hard-fail immediately
//! (with setup instructions) when nothing is listening at the configured address** —
//! they deliberately never skip. They used to, but libtest swallows passing tests'
//! output, so a "6 passed" run whose store assertions never executed was
//! indistinguishable from a real pass — the suite could stay green while the pipeline
//! was never exercised. Running this suite therefore requires the Postgres and RustFS
//! containers (see each test's panic message for the one-liners to start them). The
//! passthrough test (`custom_config_files`, ADR-0003) is fully self-contained: the test
//! itself plays the un-blessed HTTP sink's server.

use dc_interfaces::msg::StringStamped;
use postgres::{Client, NoTls};
use rclrs::{CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use std::net::{SocketAddr, TcpStream};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};

/// All three tests in this file identify the Vector process they spawned by matching
/// its install path system-wide or as a direct child (see `vector_child_of`), since it's
/// a grandchild of the test, not directly reachable via `Child`. That match can't
/// distinguish between different tests' own Vector instances, so they must not run
/// concurrently (`cargo test` parallelizes tests within a binary by default).
static PROCESS_LOCK: Mutex<()> = Mutex::new(());

/// The Postgres address every test's rendered config points at. Matches
/// `tools/infrastructure/docker/docker-compose.postgresql.yaml` and
/// `dc_bringup/params/dc_params.yaml`'s `pgsql` destination.
const PG_HOST: &str = "127.0.0.1";
const PG_PORT: u16 = 5432;
const PG_USER: &str = "dc";
const PG_PASSWORD: &str = "password";
const PG_DATABASE: &str = "dc";
const PG_TABLE: &str = "dc";

/// A fresh, unique directory per spawned `dc_bridge`, so the disk buffer Vector writes
/// to (`shipper.data_dir`) never survives across test invocations. Vector's `Child::kill`
/// (see `Supervisor::stop`) sends SIGKILL, so a supervised Vector never gets to flush its
/// buffer gracefully; without a unique `data_dir` per spawn, a later test run's Vector
/// process finds the previous run's unflushed, disk-buffered records still on disk and
/// redelivers them on startup — a stale row lands in Postgres carrying an *older* run's
/// timestamp but the *same* hardcoded marker value, which was observed to intermittently
/// fail `published_record_lands_in_postgres`'s plausibility assertion
/// before this was isolated per spawn.
fn unique_data_dir() -> String {
    static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    std::env::temp_dir()
        .join(format!("dc_bridge_e2e_test_{}_{n}", std::process::id()))
        .to_string_lossy()
        .into_owned()
}

/// `--ros-args -p ...` flags declaring one `pgsql` postgres destination subscribed to
/// `/dc/measurement/uptime`, matching `dc_bringup/params/dc_params.yaml`'s config
/// contract (ADR-0003), plus a fresh `shipper.data_dir` (see `unique_data_dir`). Valid
/// whether or not a real Postgres is listening at `PG_HOST:PG_PORT` — Vector's Fluent
/// source (and therefore dc_bridge's readiness probe) starts independently of downstream
/// sink connectivity.
fn postgres_destination_args() -> Vec<String> {
    [
        "--ros-args",
        "-p",
        &format!("shipper.data_dir:={}", unique_data_dir()),
        "-p",
        "destinations:=[\"pgsql\"]",
        "-p",
        "pgsql.type:=postgres",
        "-p",
        "pgsql.inputs:=[\"/dc/measurement/uptime\"]",
        "-p",
        &format!("pgsql.host:={PG_HOST}"),
        "-p",
        &format!("pgsql.port:={PG_PORT}"),
        "-p",
        &format!("pgsql.user:={PG_USER}"),
        "-p",
        &format!("pgsql.password:={PG_PASSWORD}"),
        "-p",
        &format!("pgsql.database:={PG_DATABASE}"),
        "-p",
        &format!("pgsql.table:={PG_TABLE}"),
    ]
    .into_iter()
    .map(str::to_string)
    .collect()
}

fn spawn_dc_bridge(extra_args: &[String], stdout: Stdio, stderr: Stdio) -> std::process::Child {
    let mut args = postgres_destination_args();
    args.extend(extra_args.iter().cloned());
    spawn_dc_bridge_with_args(&args, stdout, stderr)
}

/// Spawns the binary under test with exactly `args` — for tests whose destination
/// isn't the shared `pgsql` one `postgres_destination_args` declares.
fn spawn_dc_bridge_with_args(args: &[String], stdout: Stdio, stderr: Stdio) -> std::process::Child {
    let bin = env!("CARGO_BIN_EXE_dc_bridge");
    Command::new(bin)
        .args(args)
        .stdout(stdout)
        .stderr(stderr)
        .spawn()
        .expect("failed to spawn the dc_bridge binary under test")
}

/// Polls dc_bridge's `~/ready` service (a TCP-accept probe against Vector's Fluent
/// Forward port — independent of any downstream sink's connectivity) until it reports
/// `true` or `deadline` passes.
fn wait_for_ready(deadline: Instant) -> bool {
    let context = rclrs::Context::default_from_env().expect("failed to create an rclrs context");
    let mut executor = context.create_basic_executor();
    // ROS node names may only contain alphanumerics/underscores; `ThreadId`'s `Debug`
    // output (e.g. `ThreadId(2)`) violates that, so a nanosecond timestamp is used for
    // uniqueness instead.
    let node = executor
        .create_node(&format!(
            "dc_bridge_end_to_end_test_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ))
        .expect("failed to create the test node");
    let ready_client = node
        .create_client::<Trigger>("/dc_bridge/ready")
        .expect("failed to create a client for /dc_bridge/ready");

    let ready = Arc::new(Mutex::new(false));
    while !*ready.lock().unwrap() && Instant::now() < deadline {
        let ready = ready.clone();
        let ready_client = ready_client.clone();
        let promise = executor.commands().run(async move {
            if ready_client.notify_on_service_ready().await.is_err() {
                return;
            }
            let response: Result<Trigger_Response, _> = ready_client
                .call(&Trigger_Request::default())
                .unwrap()
                .await;
            if let Ok(response) = response {
                *ready.lock().unwrap() = response.success;
            }
        });
        let _ = executor
            .spin(
                SpinOptions::new()
                    .until_promise_resolved(promise)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error();
    }
    let is_ready = *ready.lock().unwrap();
    is_ready
}

/// Since the config renderer only produces blessed-Destination sinks (ADR-0003) — no
/// bare `console` sink to grep dc_bridge's stdout for anymore — this proves the
/// rendered `postgres` sink config is accepted by the real Vector binary and Vector
/// actually starts serving the Fluent source, without requiring a reachable Postgres:
/// `~/ready` only probes the Fluent Forward port, which Vector accepts on regardless of
/// downstream sink connectivity.
#[test]
fn dc_bridge_becomes_ready_with_a_configured_postgres_destination() {
    let _guard = PROCESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut child = spawn_dc_bridge(&[], Stdio::null(), Stdio::null());

    let ready = wait_for_ready(Instant::now() + Duration::from_secs(30));
    terminate(&mut child);

    assert!(
        ready,
        "dc_bridge's readiness service never reported ready within 30s with a \
         postgres-type destination configured"
    );
}

/// rclrs has no signal handling yet, so `executor.spin()` never returns on its own;
/// dc_bridge installs its own SIGINT/SIGTERM handler specifically to stop the Vector
/// process it supervises before exiting (see main.rs). Without it, Vector would be
/// orphaned and keep running after every normal shutdown (Ctrl-C, `ros2 launch`,
/// `kill <pid>`), not just a SIGKILL (which no userspace handler can catch anyway).
#[test]
fn stops_the_supervised_vector_process_on_sigterm() {
    let _guard = PROCESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut child = spawn_dc_bridge(&[], Stdio::null(), Stdio::null());
    let dc_bridge_pid = child.id();

    let deadline = Instant::now() + Duration::from_secs(10);
    let mut vector_pid = vector_child_of(dc_bridge_pid);
    while vector_pid.is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
        vector_pid = vector_child_of(dc_bridge_pid);
    }
    let vector_pid = vector_pid
        .expect("expected dc_bridge to have spawned a Vector child process before shutdown");

    terminate(&mut child);

    let deadline = Instant::now() + Duration::from_secs(5);
    while is_running(vector_pid) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        !is_running(vector_pid),
        "expected dc_bridge's supervised Vector process (pid {vector_pid}) to be stopped \
         after it received SIGTERM, but it's still running"
    );
}

/// A quick TCP-connect probe (not a real Postgres handshake) for whether something is
/// listening at `PG_HOST:PG_PORT` — used to fail the Postgres-backed test below
/// immediately with setup instructions, instead of a slow, confusing timeout deep in
/// the pipeline assertions.
fn postgres_reachable() -> bool {
    let addr: SocketAddr = format!("{PG_HOST}:{PG_PORT}")
        .parse()
        .expect("PG_HOST:PG_PORT must parse as a socket address");
    TcpStream::connect_timeout(&addr, Duration::from_millis(500)).is_ok()
}

/// Exercises the full path end-to-end against a real Postgres: spawns dc_bridge with a
/// `postgres`-type `pgsql` destination, publishes a `StringStamped` Record on its
/// `inputs` topic, and asserts the row actually lands in the target table with `date`
/// populated as a plausible Unix-epoch-seconds float and `uptime_s` equal to the
/// published value — the same demo documented in dc_bridge/README.md, automated.
///
/// A live Postgres is REQUIRED: without one this test fails immediately (with setup
/// instructions) rather than skipping. It used to skip — but libtest swallows a
/// passing test's output, so a run whose store assertions never executed was
/// indistinguishable from a real pass, and a change could look verified when the
/// pipeline was in fact never exercised.
#[test]
fn published_record_lands_in_postgres() {
    assert!(
        postgres_reachable(),
        "no Postgres reachable at {PG_HOST}:{PG_PORT} — this end-to-end test requires \
         one and deliberately fails (not skips) without it. Start \
         tools/infrastructure/docker/docker-compose.postgresql.yaml (or an equivalent \
         postgres:13 container with user/password 'dc'/'password' and a \
         'dc (date double precision, uptime_s integer)' table in database 'dc')"
    );

    let _guard = PROCESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let mut db = Client::connect(
        &format!(
            "host={PG_HOST} port={PG_PORT} user={PG_USER} password={PG_PASSWORD} \
             dbname={PG_DATABASE}"
        ),
        NoTls,
    )
    .expect("failed to connect to the test Postgres instance");
    db.execute(&format!("DELETE FROM {PG_TABLE}"), &[])
        .expect("failed to clear the target table before the test");

    // A distinctive marker so this assertion can't pass on a stale row left over from a
    // previous run or a concurrent demo hitting the same table.
    const MARKER_UPTIME_S: i32 = 918_273;

    let before = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();

    let mut child = spawn_dc_bridge(&[], Stdio::null(), Stdio::null());

    let context = rclrs::Context::default_from_env().expect("failed to create an rclrs context");
    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("dc_bridge_end_to_end_pg_test")
        .expect("failed to create the test node");
    let ready_client = node
        .create_client::<Trigger>("/dc_bridge/ready")
        .expect("failed to create a client for /dc_bridge/ready");
    let publisher = node
        .create_publisher::<StringStamped>("/dc/measurement/uptime")
        .expect("failed to create a publisher on /dc/measurement/uptime");

    let ready = Arc::new(Mutex::new(false));
    let ready_deadline = Instant::now() + Duration::from_secs(30);
    while !*ready.lock().unwrap() && Instant::now() < ready_deadline {
        let ready = ready.clone();
        let ready_client = ready_client.clone();
        let promise = executor.commands().run(async move {
            if ready_client.notify_on_service_ready().await.is_err() {
                return;
            }
            let response: Result<Trigger_Response, _> = ready_client
                .call(&Trigger_Request::default())
                .unwrap()
                .await;
            if let Ok(response) = response {
                *ready.lock().unwrap() = response.success;
            }
        });
        let _ = executor
            .spin(
                SpinOptions::new()
                    .until_promise_resolved(promise)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error();
    }
    assert!(
        *ready.lock().unwrap(),
        "dc_bridge's readiness service never reported ready within 30s"
    );

    // A default (zeroed) `header.stamp`, as a bare `StringStamped::default()` would
    // have, forwards as the Unix epoch — `date` would then always be `0`, useless for
    // the plausibility assertion below. Set a real stamp, as any actual Measurement
    // publisher would.
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap();
    let mut message = StringStamped::default();
    message.header.stamp.sec = now.as_secs() as i32;
    message.header.stamp.nanosec = now.subsec_nanos();
    message.data = format!(r#"{{"uptime_s": {MARKER_UPTIME_S}}}"#);

    let mut row: Option<(f64, i32)> = None;
    let publish_deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < publish_deadline {
        publisher
            .publish(&message)
            .expect("failed to publish the test Record");
        std::thread::sleep(Duration::from_millis(300));
        let rows = db
            .query(
                &format!("SELECT date, uptime_s FROM {PG_TABLE} WHERE uptime_s = $1"),
                &[&MARKER_UPTIME_S],
            )
            .expect("failed to query the target table");
        if let Some(r) = rows.into_iter().next() {
            row = Some((r.get(0), r.get(1)));
            break;
        }
    }

    let after = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();

    terminate(&mut child);

    let (date, uptime_s) = row.unwrap_or_else(|| {
        panic!(
            "expected a row with uptime_s = {MARKER_UPTIME_S} to land in Postgres \
             within 20s of publishing, but none appeared"
        )
    });
    assert_eq!(uptime_s, MARKER_UPTIME_S);
    assert!(
        date >= before - 1.0 && date <= after + 1.0,
        "expected `date` ({date}) to be a plausible Unix-epoch-seconds timestamp \
         between {before} and {after}"
    );
}

/// ADR-0003's passthrough contract, end to end: a raw Vector snippet listed in
/// `custom_config_files` ships Records to an un-blessed sink type (`http`) by consuming
/// the public per-Tag route `dc.dc.measurement.uptime` (`dc.<tag>` for Tag
/// `dc.measurement.uptime`). The test itself plays the HTTP server on an ephemeral
/// port, so this runs everywhere — no external container needed.
#[test]
fn custom_snippet_ships_records_to_an_http_sink() {
    let _guard = PROCESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let listener = std::net::TcpListener::bind("127.0.0.1:0")
        .expect("failed to bind the test HTTP server socket");
    let port = listener.local_addr().unwrap().port();
    let received = Arc::new(Mutex::new(String::new()));
    {
        // Answers 200 to everything (including Vector's sink healthcheck) and collects
        // every request into `received`; detached, dies with the test binary.
        let received = received.clone();
        std::thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { continue };
                let _ = stream.set_read_timeout(Some(Duration::from_millis(500)));
                let mut request = Vec::new();
                let mut chunk = [0u8; 4096];
                loop {
                    match std::io::Read::read(&mut stream, &mut chunk) {
                        Ok(0) | Err(_) => break,
                        Ok(n) => {
                            request.extend_from_slice(&chunk[..n]);
                            // Stop once the whole body announced by Content-Length has
                            // arrived (the header separator alone isn't enough — the
                            // body may land in a later read).
                            let text = String::from_utf8_lossy(&request);
                            if let Some((head, body)) = text.split_once("\r\n\r\n") {
                                let content_length: usize = head
                                    .lines()
                                    .find_map(|l| {
                                        let (k, v) = l.split_once(':')?;
                                        k.eq_ignore_ascii_case("content-length")
                                            .then(|| v.trim().parse().ok())?
                                    })
                                    .unwrap_or(0);
                                if body.len() >= content_length {
                                    break;
                                }
                            }
                        }
                    }
                }
                received
                    .lock()
                    .unwrap()
                    .push_str(&String::from_utf8_lossy(&request));
                let _ = std::io::Write::write_all(
                    &mut stream,
                    b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
                );
            }
        });
    }

    let snippet_path = std::env::temp_dir().join(format!(
        "dc_bridge_e2e_http_snippet_{}.toml",
        std::process::id()
    ));
    std::fs::write(
        &snippet_path,
        format!(
            r#"[sinks.e2e_http]
type = "http"
inputs = ["dc.dc.measurement.uptime"]
uri = "http://127.0.0.1:{port}/ingest"
encoding.codec = "json"
batch.timeout_secs = 1
"#
        ),
    )
    .expect("failed to write the custom config snippet");

    let mut child = spawn_dc_bridge(
        &[
            "-p".to_string(),
            format!("custom_config_files:=[\"{}\"]", snippet_path.display()),
        ],
        Stdio::null(),
        Stdio::null(),
    );
    assert!(
        wait_for_ready(Instant::now() + Duration::from_secs(30)),
        "dc_bridge's readiness service never reported ready within 30s with a \
         custom_config_files snippet configured"
    );

    const MARKER_UPTIME_S: i32 = 653_421;
    let context = rclrs::Context::default_from_env().expect("failed to create an rclrs context");
    let executor = context.create_basic_executor();
    let node = executor
        .create_node("dc_bridge_end_to_end_http_test")
        .expect("failed to create the test node");
    let publisher = node
        .create_publisher::<StringStamped>("/dc/measurement/uptime")
        .expect("failed to create a publisher on /dc/measurement/uptime");

    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap();
    let mut message = StringStamped::default();
    message.header.stamp.sec = now.as_secs() as i32;
    message.header.stamp.nanosec = now.subsec_nanos();
    message.data = format!(r#"{{"uptime_s": {MARKER_UPTIME_S}}}"#);

    let marker_number = MARKER_UPTIME_S.to_string();
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut delivered = false;
    while !delivered && Instant::now() < deadline {
        publisher
            .publish(&message)
            .expect("failed to publish the test Record");
        std::thread::sleep(Duration::from_millis(300));
        delivered = received.lock().unwrap().contains(&marker_number);
    }

    terminate(&mut child);
    std::fs::remove_file(&snippet_path).ok();

    assert!(
        delivered,
        "expected the custom http-sink snippet to deliver the published Record to the \
         test HTTP server within 30s, but the marker never arrived; received so far:\n{}",
        received.lock().unwrap()
    );
}

/// "Invalid or colliding custom snippet names fail loudly at startup, not silently":
/// a snippet claiming a component id the rendered config owns (the `pgsql` sink) must
/// make dc_bridge itself exit non-zero with a clear error naming the file — not start
/// up anyway, and not crash-loop the supervised Vector.
#[test]
fn colliding_custom_snippet_fails_startup_loudly() {
    let _guard = PROCESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let snippet_path = std::env::temp_dir().join(format!(
        "dc_bridge_e2e_colliding_snippet_{}.toml",
        std::process::id()
    ));
    std::fs::write(
        &snippet_path,
        "[sinks.pgsql]\ntype = \"console\"\ninputs = [\"dc.dc.measurement.uptime\"]\nencoding.codec = \"json\"\n",
    )
    .expect("failed to write the colliding snippet");

    let mut child = spawn_dc_bridge(
        &[
            "-p".to_string(),
            format!("custom_config_files:=[\"{}\"]", snippet_path.display()),
        ],
        Stdio::null(),
        Stdio::piped(),
    );

    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        match child.try_wait().expect("failed to poll dc_bridge") {
            Some(status) => break Some(status),
            None if Instant::now() >= deadline => break None,
            None => std::thread::sleep(Duration::from_millis(100)),
        }
    };
    let status = status.unwrap_or_else(|| {
        terminate(&mut child);
        std::fs::remove_file(&snippet_path).ok();
        panic!(
            "expected dc_bridge to exit at startup when a custom snippet collides with \
             a rendered component, but it was still running after 20s"
        );
    });

    let mut stderr = String::new();
    if let Some(mut pipe) = child.stderr.take() {
        let _ = std::io::Read::read_to_string(&mut pipe, &mut stderr);
    }
    std::fs::remove_file(&snippet_path).ok();

    assert!(
        !status.success(),
        "expected a non-zero exit for a colliding custom snippet, got: {status}"
    );
    assert!(
        stderr.contains("pgsql"),
        "expected dc_bridge's startup error to name the colliding component; stderr:\n{stderr}"
    );
}

/// Whether an S3-compatible server answers at 127.0.0.1:9000 (the standard S3 API port
/// of RustFS/MinIO-style stores) — used to fail the store-backed test below
/// immediately with setup instructions, instead of a slow, confusing timeout in the
/// bucket-polling loop.
fn s3_store_reachable() -> bool {
    let addr: SocketAddr = "127.0.0.1:9000"
        .parse()
        .expect("the S3 store address must parse");
    TcpStream::connect_timeout(&addr, Duration::from_millis(500)).is_ok()
}

/// Exercises the `s3` blessed destination end-to-end against a real self-hosted
/// S3-compatible store: dc_bridge is configured purely via ROS params (custom
/// `endpoint`, explicit credentials, `force_path_style`, a short
/// `batch_timeout_secs`), a Record is published, and the test asserts an object
/// containing the marker value lands in the bucket.
///
/// REQUIRES an S3-compatible store at 127.0.0.1:9000 with credentials
/// `rustfsadmin`/`rustfsadmin` and an existing bucket `dc-records` whose anonymous
/// access policy allows public read/list (`mc anonymous set public`), so the assertion
/// side can use plain unauthenticated HTTP (via `curl`) instead of a full SigV4
/// client; objects are gzip-decoded with `zcat` (Vector's `aws_s3` default
/// compression). RustFS (`rustfs/rustfs`, whose defaults these are) is the recommended
/// store — MinIO's community edition was archived upstream in 2026 — and is what this
/// test was verified against, but the `s3` type (and this test) is store-agnostic.
/// Without a store this test fails immediately (with setup instructions) rather than
/// skipping — see `published_record_lands_in_postgres` for why skipping was removed.
#[test]
fn published_record_reaches_s3_bucket() {
    assert!(
        s3_store_reachable(),
        "nothing listening at 127.0.0.1:9000 — this end-to-end test requires an \
         S3-compatible store and deliberately fails (not skips) without one. Start \
         e.g. a RustFS container (credentials rustfsadmin/rustfsadmin) and create a \
         public-read bucket: mc alias set local http://127.0.0.1:9000 rustfsadmin \
         rustfsadmin && mc mb -p local/dc-records && mc anonymous set public \
         local/dc-records"
    );

    let _guard = PROCESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    const MARKER_UPTIME_S: i32 = 786_534;
    // A unique prefix per run, so the assertion below can't match an object left over
    // from an earlier run against the same long-lived MinIO container.
    let prefix = format!(
        "e2e/{}_{}/",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    );

    let mut child = spawn_dc_bridge_with_args(
        &[
            "--ros-args".to_string(),
            "-p".to_string(),
            format!("shipper.data_dir:={}", unique_data_dir()),
            "-p".to_string(),
            "destinations:=[\"rustfs\"]".to_string(),
            "-p".to_string(),
            "rustfs.type:=s3".to_string(),
            "-p".to_string(),
            "rustfs.inputs:=[\"/dc/measurement/uptime\"]".to_string(),
            "-p".to_string(),
            "rustfs.bucket:=dc-records".to_string(),
            "-p".to_string(),
            "rustfs.endpoint:=http://127.0.0.1:9000".to_string(),
            "-p".to_string(),
            "rustfs.region:=us-east-1".to_string(),
            "-p".to_string(),
            "rustfs.access_key_id:=rustfsadmin".to_string(),
            "-p".to_string(),
            "rustfs.secret_access_key:=rustfsadmin".to_string(),
            "-p".to_string(),
            "rustfs.force_path_style:=true".to_string(),
            "-p".to_string(),
            format!("rustfs.key_prefix:={prefix}"),
            "-p".to_string(),
            "rustfs.batch_timeout_secs:=2".to_string(),
        ],
        Stdio::null(),
        Stdio::null(),
    );
    assert!(
        wait_for_ready(Instant::now() + Duration::from_secs(30)),
        "dc_bridge's readiness service never reported ready within 30s with an s3-type \
         destination configured"
    );

    let context = rclrs::Context::default_from_env().expect("failed to create an rclrs context");
    let executor = context.create_basic_executor();
    let node = executor
        .create_node("dc_bridge_end_to_end_s3_test")
        .expect("failed to create the test node");
    let publisher = node
        .create_publisher::<StringStamped>("/dc/measurement/uptime")
        .expect("failed to create a publisher on /dc/measurement/uptime");

    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap();
    let mut message = StringStamped::default();
    message.header.stamp.sec = now.as_secs() as i32;
    message.header.stamp.nanosec = now.subsec_nanos();
    message.data = format!(r#"{{"uptime_s": {MARKER_UPTIME_S}}}"#);

    let deadline = Instant::now() + Duration::from_secs(45);
    let mut found = false;
    while !found && Instant::now() < deadline {
        publisher
            .publish(&message)
            .expect("failed to publish the test Record");
        std::thread::sleep(Duration::from_millis(500));
        found = bucket_prefix_contains_marker(&prefix, &MARKER_UPTIME_S.to_string());
    }

    terminate(&mut child);

    assert!(
        found,
        "expected an object containing uptime_s = {MARKER_UPTIME_S} under prefix \
         '{prefix}' of bucket 'dc-records' within 45s of publishing"
    );
}

/// Lists `dc-records` objects under `prefix` via anonymous HTTP (`curl`) and returns
/// whether any of them, gzip-decoded (`zcat`), contains `marker`.
fn bucket_prefix_contains_marker(prefix: &str, marker: &str) -> bool {
    let listing = Command::new("curl")
        .args([
            "-s",
            &format!("http://127.0.0.1:9000/dc-records?list-type=2&prefix={prefix}"),
        ])
        .output()
        .expect("failed to run curl against MinIO");
    let listing = String::from_utf8_lossy(&listing.stdout).into_owned();
    for key in listing
        .split("<Key>")
        .skip(1)
        .filter_map(|part| part.split("</Key>").next())
    {
        let object = Command::new("sh")
            .args([
                "-c",
                &format!("curl -s 'http://127.0.0.1:9000/dc-records/{key}' | zcat 2>/dev/null"),
            ])
            .output()
            .expect("failed to fetch an object from MinIO");
        if String::from_utf8_lossy(&object.stdout).contains(marker) {
            return true;
        }
    }
    false
}

/// Sends SIGTERM (dc_bridge's install-hooked signal, not SIGKILL) and waits for exit.
/// `kill`'s own stderr is suppressed: if dc_bridge has already exited by the time this
/// runs (e.g. it reacted fast enough that the signal is redundant), that's not this
/// test's concern — only whether the *effect* (dc_bridge, and the Vector it supervises,
/// both gone) holds, which the callers check separately.
fn terminate(child: &mut std::process::Child) {
    let _ = Command::new("kill")
        .arg(child.id().to_string())
        .stderr(Stdio::null())
        .status();
    let _ = child.wait();
}

/// Finds a direct child of `parent_pid`. dc_bridge spawns Vector directly (no shell
/// wrapper), so this identifies exactly *this* test's Vector process — unlike matching
/// on the Vector binary's path system-wide, it can't be confused with an unrelated
/// Vector instance that happens to be running elsewhere on the same machine.
fn vector_child_of(parent_pid: u32) -> Option<u32> {
    let output = Command::new("pgrep")
        .args(["-P", &parent_pid.to_string()])
        .output()
        .ok()?;
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .next()?
        .trim()
        .parse()
        .ok()
}

/// Checks whether `pid` is still alive via `kill -0` (tests for existence/permission
/// without sending a real signal), since a `pgrep`-based liveness re-check would need
/// the same disambiguation `vector_child_of` already provides once the process could
/// have exited (and its PID potentially been reused).
fn is_running(pid: u32) -> bool {
    Command::new("kill")
        .args(["-0", &pid.to_string()])
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}
