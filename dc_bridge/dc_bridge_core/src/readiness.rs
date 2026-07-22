//! Readiness: tracks whether Vector is currently accepting Fluent Forward connections,
//! shared between a background prober and the ROS readiness service (ADR-0006 — the
//! Bridge has no lifecycle state beyond "process up + ready service answers").

use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

#[derive(Clone)]
pub struct Readiness {
    ready: Arc<AtomicBool>,
}

impl Readiness {
    pub fn new() -> Self {
        Self {
            ready: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn set_ready(&self, ready: bool) {
        self.ready.store(ready, Ordering::SeqCst);
    }

    pub fn is_ready(&self) -> bool {
        self.ready.load(Ordering::SeqCst)
    }
}

impl Default for Readiness {
    fn default() -> Self {
        Self::new()
    }
}

/// A plain TCP connect probe against Vector's Fluent Forward listen address: ready
/// means "accepting connections", not-ready means it doesn't (yet, or anymore).
pub fn probe(addr: SocketAddr, timeout: Duration) -> bool {
    TcpStream::connect_timeout(&addr, timeout).is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_not_ready_and_reflects_set_ready() {
        let readiness = Readiness::new();
        assert!(!readiness.is_ready());
        readiness.set_ready(true);
        assert!(readiness.is_ready());
        readiness.set_ready(false);
        assert!(!readiness.is_ready());
    }

    #[test]
    fn clones_share_the_same_underlying_state() {
        let readiness = Readiness::new();
        let clone = readiness.clone();
        readiness.set_ready(true);
        assert!(clone.is_ready());
    }
}
