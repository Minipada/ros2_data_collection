//! Supervisor acceptance test: a killed/exited process is restarted automatically.
//! Deliberately supervises a plain shell command rather than the real Vector binary so
//! this runs without the vendored binary being built.

use dc_bridge_core::{Supervisor, SupervisorConfig};
use std::time::Duration;

#[test]
fn restarts_a_process_that_exits_on_its_own() {
    let mut supervisor = Supervisor::new(SupervisorConfig {
        program: "sh".into(),
        args: vec!["-c".to_string(), "sleep 0.05".to_string()],
        restart_backoff: Duration::from_millis(0),
    });

    supervisor.start().unwrap();
    assert!(supervisor.is_running());

    std::thread::sleep(Duration::from_millis(300));
    assert!(
        !supervisor.is_running(),
        "the shell command should have exited by now"
    );

    let restarted = supervisor.poll_restart().unwrap();
    assert!(restarted);
    assert!(
        supervisor.is_running(),
        "supervisor should have respawned the process"
    );
}

#[test]
fn respects_restart_backoff_between_respawn_attempts() {
    let mut supervisor = Supervisor::new(SupervisorConfig {
        program: "sh".into(),
        args: vec!["-c".to_string(), "exit 1".to_string()],
        restart_backoff: Duration::from_secs(60),
    });

    supervisor.start().unwrap();
    std::thread::sleep(Duration::from_millis(200));
    assert!(!supervisor.is_running());

    // First poll after exit restarts immediately (no prior exit recorded yet).
    assert!(supervisor.poll_restart().unwrap());
    std::thread::sleep(Duration::from_millis(200));
    assert!(!supervisor.is_running());

    // Second exit is within the (60s) backoff window, so no respawn should happen yet.
    assert!(!supervisor.poll_restart().unwrap());
}

/// Regression test for a real race hit in a live Docker verification pass (see the
/// `stopped` field doc on `Supervisor`): `main.rs`'s SIGTERM handler calls `stop()`
/// then exits, while a background thread independently calls `poll_restart()` in a
/// loop. If `stop()` didn't permanently disable future respawns, a `poll_restart` tick
/// landing right after `stop()` reads the just-cleared `child` as "the process died"
/// and spawns a brand-new, now-permanently-orphaned replacement.
#[test]
fn poll_restart_never_respawns_after_stop() {
    let mut supervisor = Supervisor::new(SupervisorConfig {
        program: "sh".into(),
        args: vec!["-c".to_string(), "sleep 5".to_string()],
        restart_backoff: Duration::from_millis(0),
    });

    supervisor.start().unwrap();
    assert!(supervisor.is_running());

    supervisor.stop();
    assert!(!supervisor.is_running());

    // Simulate the background restart thread's next tick(s) racing right behind
    // `stop()`, exactly as they do in `main.rs`.
    for _ in 0..5 {
        assert!(
            !supervisor.poll_restart().unwrap(),
            "poll_restart must never respawn once the supervisor has been stopped"
        );
    }
    assert!(!supervisor.is_running());
}

/// Companion to the test above, for the startup side of the same class of race:
/// `main.rs` installs its SIGTERM handler *before* the first `start()`, so a signal
/// landing during startup calls `stop()` on a supervisor that hasn't spawned anything
/// yet — a subsequent `start()` must then be a no-op rather than spawning a process
/// nobody will ever stop (hit for real as an intermittent `colcon test` failure of
/// `stops_the_supervised_vector_process_on_sigterm` while the startup sequence also
/// grew a `vector validate` step).
#[test]
fn start_never_spawns_after_stop() {
    let mut supervisor = Supervisor::new(SupervisorConfig {
        program: "sh".into(),
        args: vec!["-c".to_string(), "sleep 5".to_string()],
        restart_backoff: Duration::from_millis(0),
    });

    supervisor.stop();
    supervisor.start().unwrap();
    assert!(
        !supervisor.is_running(),
        "start() must not spawn anything once the supervisor has been stopped"
    );
}
