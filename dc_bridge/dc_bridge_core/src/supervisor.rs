//! Supervisor: spawns the vendored Vector binary with a generated config and restarts
//! it if it dies. Deliberately agnostic to what program it supervises (see tests) so
//! the restart logic is exercised without needing a real Vector binary.

use std::io;
use std::path::PathBuf;
use std::process::{Child, Command};
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct SupervisorConfig {
    pub program: PathBuf,
    pub args: Vec<String>,
    /// Minimum time to wait after an exit before respawning, to avoid a restart storm
    /// if the supervised process keeps failing immediately.
    pub restart_backoff: Duration,
}

impl SupervisorConfig {
    pub fn vector(binary: PathBuf, config_path: PathBuf) -> Self {
        Self {
            program: binary,
            args: vec![
                "--config".to_string(),
                config_path.to_string_lossy().into_owned(),
            ],
            restart_backoff: Duration::from_millis(500),
        }
    }
}

pub struct Supervisor {
    config: SupervisorConfig,
    child: Option<Child>,
    last_exit: Option<Instant>,
    /// Set by [`Supervisor::stop`]; once true, [`Supervisor::poll_restart`] refuses to
    /// respawn. Without this, a caller that stops the supervised process (e.g.
    /// `main.rs`'s SIGTERM handler, which explicitly kills Vector before exiting) races
    /// the background thread that also calls `poll_restart` in a loop: `stop` sets
    /// `child` to `None`, and if the next `poll_restart` tick lands before the process
    /// actually exits, it reads that same `None` as "the process died, respawn it" and
    /// spawns a brand-new, now-permanently-orphaned Vector — reproduced by hand (a
    /// direct `kill -TERM` on a real, running `dc_bridge` left a freshly-spawned Vector
    /// child reparented to init once dc_bridge itself had exited).
    stopped: bool,
}

impl Supervisor {
    pub fn new(config: SupervisorConfig) -> Self {
        Self {
            config,
            child: None,
            last_exit: None,
            stopped: false,
        }
    }

    pub fn start(&mut self) -> io::Result<()> {
        let child = Command::new(&self.config.program)
            .args(&self.config.args)
            .spawn()?;
        self.child = Some(child);
        Ok(())
    }

    pub fn is_running(&mut self) -> bool {
        match &mut self.child {
            Some(child) => matches!(child.try_wait(), Ok(None)),
            None => false,
        }
    }

    /// Checks whether the supervised process has exited and, if so, restarts it
    /// (subject to `restart_backoff`). Returns `Ok(true)` if a restart happened. A
    /// no-op, returning `Ok(false)`, once [`Supervisor::stop`] has been called — see
    /// the `stopped` field doc for the respawn-after-stop race this prevents.
    pub fn poll_restart(&mut self) -> io::Result<bool> {
        if self.stopped {
            return Ok(false);
        }
        let exited = match &mut self.child {
            Some(child) => child.try_wait()?.is_some(),
            None => true,
        };
        if !exited {
            return Ok(false);
        }
        if let Some(last_exit) = self.last_exit {
            if last_exit.elapsed() < self.config.restart_backoff {
                return Ok(false);
            }
        }

        self.child = None;
        self.last_exit = Some(Instant::now());
        self.start()?;
        Ok(true)
    }

    /// Stops the supervised process and permanently disables future respawns from
    /// `poll_restart` (there is no corresponding "un-stop": a `Supervisor` is only ever
    /// stopped once, right before the owning process exits).
    pub fn stop(&mut self) {
        self.stopped = true;
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

impl Drop for Supervisor {
    fn drop(&mut self) {
        self.stop();
    }
}
