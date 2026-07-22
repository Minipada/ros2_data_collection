//! End-to-end workflow test for the actual ROS node: spawns the real `dc_bridge` binary
//! (which locates and supervises the real vendored Vector binary), waits for the
//! readiness service to report true, publishes a Record on a subscribed topic, and
//! asserts it reaches Vector's console sink — automating the manual verification done
//! for #244 (see dc_bridge/README.md) as a real `colcon test` / `cargo test`.
//!
//! Requires a sourced ROS 2 + rclrs environment (this is what `colcon test
//! --packages-select dc_bridge` runs under the hood, via colcon-ros-cargo's
//! `AmentCargoTestTask`, which is just `cargo test`); it cannot run in a plain sandbox.

use dc_interfaces::msg::StringStamped;
use rclrs::{CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};

/// Both tests in this file identify the Vector process they spawned by matching its
/// install path system-wide (see `vector_pid`), since it's a grandchild of the test, not
/// directly reachable via `Child`. That match can't distinguish between the two tests'
/// own Vector instances, so they must not run concurrently (`cargo test` parallelizes
/// tests within a binary by default).
static PROCESS_LOCK: Mutex<()> = Mutex::new(());

#[test]
fn published_record_reaches_vectors_console_sink() {
    let _guard = PROCESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let bin = env!("CARGO_BIN_EXE_dc_bridge");
    let mut child = Command::new(bin)
        .args(["--ros-args", "-p", "topics:=[\"/dc/measurement/uptime\"]"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn the dc_bridge binary under test");

    let captured = Arc::new(Mutex::new(String::new()));
    {
        let stdout = child.stdout.take().expect("child stdout was not piped");
        let captured = captured.clone();
        std::thread::spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                let mut captured = captured.lock().unwrap();
                captured.push_str(&line);
                captured.push('\n');
            }
        });
    }
    {
        // Drained (not asserted on) so the child never blocks on a full stderr pipe.
        let stderr = child.stderr.take().expect("child stderr was not piped");
        std::thread::spawn(
            move || {
                for _line in BufReader::new(stderr).lines().map_while(Result::ok) {}
            },
        );
    }

    let context = rclrs::Context::default_from_env().expect("failed to create an rclrs context");
    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("dc_bridge_end_to_end_test")
        .expect("failed to create the test node");
    let ready_client = node
        .create_client::<Trigger>("/dc_bridge/ready")
        .expect("failed to create a client for /dc_bridge/ready");
    let publisher = node
        .create_publisher::<StringStamped>("/dc/measurement/uptime")
        .expect("failed to create a publisher on /dc/measurement/uptime");

    let ready = Arc::new(Mutex::new(false));
    let deadline = Instant::now() + Duration::from_secs(30);
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
    assert!(
        *ready.lock().unwrap(),
        "dc_bridge's readiness service never reported ready within 30s"
    );

    let mut message = StringStamped::default();
    message.data = r#"{"uptime_s": 42}"#.to_string();
    let publish_deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < publish_deadline {
        publisher
            .publish(&message)
            .expect("failed to publish the test Record");
        if captured.lock().unwrap().contains("uptime_s") {
            break;
        }
        std::thread::sleep(Duration::from_millis(200));
    }

    let output = captured.lock().unwrap().clone();
    terminate(&mut child);

    assert!(
        output.contains("\"tag\":\"dc.measurement.uptime\""),
        "expected the Record forwarded with its topic-derived Fluent Forward tag on \
         Vector's console sink; captured dc_bridge output:\n{output}"
    );
    assert!(
        output.contains("\"uptime_s\":42"),
        "expected the Record's JSON payload to reach Vector's console sink; captured \
         dc_bridge output:\n{output}"
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
    let bin = env!("CARGO_BIN_EXE_dc_bridge");
    let mut child = Command::new(bin)
        .args(["--ros-args", "-p", "topics:=[\"/dc/measurement/uptime\"]"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("failed to spawn the dc_bridge binary under test");
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
