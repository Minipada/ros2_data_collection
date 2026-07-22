//! Forwarder: `send(record)` hides Fluent Forward msgpack framing, socket lifecycle,
//! reconnection, and backpressure behind one call. The wire protocol is the interface;
//! nothing else about Vector or the transport leaks past this module.

use rmpv::Value;
use std::io::{self, Write};
use std::net::{SocketAddr, TcpStream};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use thiserror::Error;

/// A DC Record ready to be forwarded: a Fluent Forward tag, a timestamp, and the
/// Record's JSON payload (`StringStamped.data`, already parsed).
#[derive(Debug, Clone)]
pub struct Record {
    pub tag: String,
    pub timestamp: SystemTime,
    pub payload: serde_json::Value,
}

impl Record {
    pub fn new(tag: impl Into<String>, timestamp: SystemTime, payload: serde_json::Value) -> Self {
        Self {
            tag: tag.into(),
            timestamp,
            payload,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ForwarderConfig {
    pub addr: SocketAddr,
    pub connect_timeout: Duration,
    pub write_timeout: Duration,
}

impl ForwarderConfig {
    pub fn new(addr: SocketAddr) -> Self {
        Self {
            addr,
            connect_timeout: Duration::from_secs(2),
            write_timeout: Duration::from_millis(200),
        }
    }
}

#[derive(Debug, Error)]
pub enum ForwarderError {
    #[error("failed to connect to Fluent Forward peer at {0}: {1}")]
    Connect(SocketAddr, io::Error),
    #[error("peer stalled: write did not complete within the configured timeout")]
    Backpressure,
    #[error("io error while forwarding record: {0}")]
    Io(io::Error),
}

/// Sends Records to a Fluent Forward peer (Vector's `fluent` source) over TCP.
///
/// Reconnects lazily: a dead connection is dropped on write failure and re-established
/// on the next `send()`. A write that blocks past `write_timeout` is reported as
/// `Backpressure` without dropping the connection, since the peer may just be slow to
/// drain, not gone.
pub struct Forwarder {
    config: ForwarderConfig,
    stream: Option<TcpStream>,
}

impl Forwarder {
    pub fn new(config: ForwarderConfig) -> Self {
        Self {
            config,
            stream: None,
        }
    }

    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }

    pub fn send(&mut self, record: &Record) -> Result<(), ForwarderError> {
        self.ensure_connected()?;
        let frame = Self::frame(record);

        let stream = self
            .stream
            .as_mut()
            .expect("ensure_connected just populated this");
        match stream.write_all(&frame) {
            Ok(()) => Ok(()),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                Err(ForwarderError::Backpressure)
            }
            Err(e) => {
                self.stream = None;
                Err(ForwarderError::Io(e))
            }
        }
    }

    fn ensure_connected(&mut self) -> Result<(), ForwarderError> {
        if self.stream.is_some() {
            return Ok(());
        }
        let stream = TcpStream::connect_timeout(&self.config.addr, self.config.connect_timeout)
            .map_err(|e| ForwarderError::Connect(self.config.addr, e))?;
        stream
            .set_write_timeout(Some(self.config.write_timeout))
            .map_err(|e| ForwarderError::Connect(self.config.addr, e))?;
        let _ = stream.set_nodelay(true);
        self.stream = Some(stream);
        Ok(())
    }

    /// Fluent Forward "Forward Mode" frame: `[tag, [[time, record]]]`.
    fn frame(record: &Record) -> Vec<u8> {
        let time = record
            .timestamp
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        let entry = Value::Array(vec![
            Value::from(time),
            record_to_msgpack_map(&record.payload),
        ]);
        let message = Value::Array(vec![
            Value::from(record.tag.as_str()),
            Value::Array(vec![entry]),
        ]);

        let mut buf = Vec::new();
        rmpv::encode::write_value(&mut buf, &message).expect("encoding into a Vec cannot fail");
        buf
    }
}

/// The Fluent Forward wire record must be a map. A Record's JSON payload usually is one;
/// anything else (bare string/number/array) is wrapped rather than dropped.
fn record_to_msgpack_map(payload: &serde_json::Value) -> Value {
    match payload {
        serde_json::Value::Object(_) => json_to_value(payload),
        other => Value::Map(vec![(Value::from("message"), json_to_value(other))]),
    }
}

fn json_to_value(value: &serde_json::Value) -> Value {
    match value {
        serde_json::Value::Null => Value::Nil,
        serde_json::Value::Bool(b) => Value::Boolean(*b),
        serde_json::Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                Value::from(i)
            } else if let Some(u) = n.as_u64() {
                Value::from(u)
            } else {
                Value::from(n.as_f64().unwrap_or(0.0))
            }
        }
        serde_json::Value::String(s) => Value::from(s.as_str()),
        serde_json::Value::Array(arr) => Value::Array(arr.iter().map(json_to_value).collect()),
        serde_json::Value::Object(map) => Value::Map(
            map.iter()
                .map(|(k, v)| (Value::from(k.as_str()), json_to_value(v)))
                .collect(),
        ),
    }
}
