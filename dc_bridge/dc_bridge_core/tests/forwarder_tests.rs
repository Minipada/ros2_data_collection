//! Forwarder acceptance tests against a mock Fluent Forward server: frame correctness,
//! tag handling, reconnection after socket drop, and backpressure when the peer stalls.

use dc_bridge_core::{Forwarder, ForwarderConfig, ForwarderError, Record};
use socket2::Socket;
use std::io::BufReader;
use std::net::TcpListener;
use std::net::TcpStream;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

fn record(tag: &str, json: &str) -> Record {
    Record::new(
        tag,
        UNIX_EPOCH + Duration::from_secs(1_700_000_000),
        serde_json::from_str(json).unwrap(),
    )
}

#[test]
fn sends_a_well_formed_fluent_forward_frame_with_the_correct_tag() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let server = std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut reader = BufReader::new(stream);
        rmpv::decode::read_value(&mut reader).unwrap()
    });

    let mut forwarder = Forwarder::new(ForwarderConfig::new(addr));
    forwarder
        .send(&record("dc.measurement.uptime", r#"{"uptime_s": 42}"#))
        .unwrap();

    let frame = server.join().unwrap();
    let top = frame.as_array().expect("frame must be an array");
    assert_eq!(top.len(), 2, "Forward Mode frame is [tag, entries]");
    assert_eq!(top[0].as_str(), Some("dc.measurement.uptime"));

    let entries = top[1].as_array().expect("entries must be an array");
    assert_eq!(entries.len(), 1);
    let entry = entries[0].as_array().expect("entry must be [time, record]");
    assert_eq!(entry[0].as_u64(), Some(1_700_000_000));

    let record_map = entry[1].as_map().expect("record must be a map");
    let uptime = record_map
        .iter()
        .find(|(k, _)| k.as_str() == Some("uptime_s"))
        .and_then(|(_, v)| v.as_u64());
    assert_eq!(uptime, Some(42));
}

#[test]
fn non_object_payloads_are_wrapped_in_a_message_field() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let server = std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut reader = BufReader::new(stream);
        rmpv::decode::read_value(&mut reader).unwrap()
    });

    let mut forwarder = Forwarder::new(ForwarderConfig::new(addr));
    forwarder
        .send(&record("dc.test", r#""plain string data""#))
        .unwrap();

    let frame = server.join().unwrap();
    let entries = frame.as_array().unwrap()[1].as_array().unwrap();
    let record_map = entries[0].as_array().unwrap()[1].as_map().unwrap();
    let message = record_map
        .iter()
        .find(|(k, _)| k.as_str() == Some("message"))
        .and_then(|(_, v)| v.as_str().map(str::to_string));
    assert_eq!(message.as_deref(), Some("plain string data"));
}

#[test]
fn different_records_carry_their_own_tag_on_a_shared_connection() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let server = std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut reader = BufReader::new(stream);
        let first = rmpv::decode::read_value(&mut reader).unwrap();
        let second = rmpv::decode::read_value(&mut reader).unwrap();
        (first, second)
    });

    let mut forwarder = Forwarder::new(ForwarderConfig::new(addr));
    forwarder
        .send(&record("dc.measurement.cpu", r#"{"pct": 1}"#))
        .unwrap();
    forwarder
        .send(&record("dc.measurement.uptime", r#"{"uptime_s": 2}"#))
        .unwrap();

    let (first, second) = server.join().unwrap();
    assert_eq!(
        first.as_array().unwrap()[0].as_str(),
        Some("dc.measurement.cpu")
    );
    assert_eq!(
        second.as_array().unwrap()[0].as_str(),
        Some("dc.measurement.uptime")
    );
}

#[test]
fn reconnects_after_the_peer_drops_the_connection() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let server = std::thread::spawn(move || {
        // First connection: read one frame, then abortively close (SO_LINGER 0 forces
        // an RST rather than a graceful FIN) to simulate the peer disappearing.
        let (stream, _) = listener.accept().unwrap();
        let sock = Socket::from(stream);
        sock.set_linger(Some(Duration::from_secs(0))).unwrap();
        let mut reader = BufReader::new(TcpStream::from(sock));
        let first = rmpv::decode::read_value(&mut reader).unwrap();
        drop(reader);

        // Second connection: proves the Forwarder reconnected on its own.
        let (stream2, _) = listener.accept().unwrap();
        let mut reader2 = BufReader::new(stream2);
        let second = rmpv::decode::read_value(&mut reader2).unwrap();
        (first, second)
    });

    let mut forwarder = Forwarder::new(ForwarderConfig::new(addr));
    forwarder.send(&record("dc.test", r#"{"n": 1}"#)).unwrap();

    // Give the abortive close time to reach the client's TCP stack.
    std::thread::sleep(Duration::from_millis(100));

    let mut reconnected = false;
    for _ in 0..100 {
        if forwarder.send(&record("dc.test", r#"{"n": 2}"#)).is_ok() {
            reconnected = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        reconnected,
        "forwarder should reconnect after the peer resets the connection"
    );

    let (first, second) = server.join().unwrap();
    let n_of = |frame: &rmpv::Value| -> Option<u64> {
        let entries = frame.as_array()?[1].as_array()?;
        let record_map = entries[0].as_array()?[1].as_map()?;
        record_map
            .iter()
            .find(|(k, _)| k.as_str() == Some("n"))
            .and_then(|(_, v)| v.as_u64())
    };
    assert_eq!(n_of(&first), Some(1));
    assert_eq!(n_of(&second), Some(2));
}

#[test]
fn returns_backpressure_when_the_peer_stalls() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let sock = Socket::from(stream);
        let _ = sock.set_recv_buffer_size(1024);
        // Never read: let the peer's writes back up until they block/time out.
        std::thread::sleep(Duration::from_secs(2));
    });

    let mut config = ForwarderConfig::new(addr);
    config.write_timeout = Duration::from_millis(50);
    let mut forwarder = Forwarder::new(config);

    let big_payload = serde_json::json!({ "blob": "x".repeat(1_000_000) });
    let big_record = Record::new("dc.test", SystemTime::now(), big_payload);

    let mut saw_backpressure = false;
    for _ in 0..20 {
        if let Err(ForwarderError::Backpressure) = forwarder.send(&big_record) {
            saw_backpressure = true;
            break;
        }
    }
    assert!(
        saw_backpressure,
        "expected a stalled peer to eventually trigger backpressure"
    );
}
