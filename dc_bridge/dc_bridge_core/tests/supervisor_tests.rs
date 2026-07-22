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
