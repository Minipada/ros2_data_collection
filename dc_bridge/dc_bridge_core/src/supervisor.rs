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
}

impl Supervisor {
    pub fn new(config: SupervisorConfig) -> Self {
        Self {
            config,
            child: None,
            last_exit: None,
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
    /// (subject to `restart_backoff`). Returns `Ok(true)` if a restart happened.
    pub fn poll_restart(&mut self) -> io::Result<bool> {
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

    pub fn stop(&mut self) {
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
