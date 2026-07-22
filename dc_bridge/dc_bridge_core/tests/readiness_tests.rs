//! Readiness probe acceptance test: ready only while something is accepting
//! connections on the Fluent Forward address, not-ready once it stops.

use dc_bridge_core::readiness::probe;
use std::net::TcpListener;
use std::time::Duration;

#[test]
fn ready_while_the_peer_accepts_connections_not_ready_once_it_stops() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    assert!(probe(addr, Duration::from_millis(200)));

    drop(listener);

    assert!(!probe(addr, Duration::from_millis(200)));
}
