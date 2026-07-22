//! Config renderer (ADR-0003): a pure, I/O-free mapping from Bridge/Destination
//! parameters to a complete Vector configuration. Given a fully-resolved [`RenderConfig`],
//! [`render`] produces Vector TOML with: the Fluent Forward source, one VRL transform
//! that normalizes each blessed destination's timestamp field (replacing the Humble-era
//! chained Lua filters — see `dc_destinations/include/dc_destinations/flb_destination.hpp`
//! on the `humble` branch), one `route` transform exposing a stable `dc.<tag>` output per
//! Destination (the passthrough contract from ADR-0003), disk-buffer settings, and the
//! blessed sinks themselves (only `postgres` is implemented; `s3`/`file`/`console` are
//! reserved names that reject clearly until their own issues land).
//!
//! Two layers, both pure and independently testable:
//! - [`destination_from_raw`] / [`PostgresParams::from_raw`] turn the flat, stringly-typed
//!   values ROS parameters naturally give us into validated, typed config — this is where
//!   "invalid-parameter rejection with clear error messages" lives.
//! - [`render`] turns already-validated, typed [`RenderConfig`] into Vector TOML text.
//!
//! [`expand_env`] handles the `$HOME`/`$DC_PG_PASSWORD`-style expansion the config
//! contract uses for `shipper.data_dir` and destination passwords; it takes an injected
//! lookup function rather than reading the real environment, so it stays pure too.

use std::collections::HashSet;
use std::fmt::Write as _;
use std::net::SocketAddr;
use toml::value::Table;
use toml::Value;

use crate::config::TopicConfig;

const SOURCE_ID: &str = "dc_bridge_in";
const NORMALIZE_TRANSFORM_ID: &str = "dc_bridge_normalize";
/// The stable public route-transform id ADR-0003 fixes: each Destination's output is
/// addressable as `dc.<name>`, the name passthrough `custom_sinks` snippets consume.
pub const ROUTE_TRANSFORM_ID: &str = "dc";
/// Vector rejects a disk buffer's `max_size` below this (~256 MiB); see the `postgres`
/// sink reference docs.
pub const MIN_DISK_BUFFER_BYTES: u64 = 268_435_488;

#[derive(Debug, Clone, PartialEq)]
pub struct RenderConfig {
    pub forward_addr: SocketAddr,
    pub data_dir: String,
    pub buffer_max_bytes: u64,
    pub destinations: Vec<Destination>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Destination {
    pub name: String,
    pub receives: Receives,
    /// ROS topic names feeding this Destination; the Fluent Forward tag each one is
    /// routed under is derived the same way `TopicConfig` derives it for subscriptions.
    pub inputs: Vec<String>,
    pub kind: DestinationKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Receives {
    Records,
    Files,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DestinationKind {
    Postgres(PostgresParams),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeFormat {
    Double,
    Iso8601,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PostgresParams {
    pub host: String,
    pub port: u16,
    pub user: String,
    pub password: String,
    pub database: String,
    pub table: String,
    pub time_key: String,
    pub time_format: TimeFormat,
}

#[derive(Debug, Clone, thiserror::Error, PartialEq, Eq)]
pub enum RenderError {
    #[error("shipper.data_dir must not be empty")]
    EmptyDataDir,
    #[error("no destinations configured; add at least one entry to the `destinations` parameter")]
    NoDestinations,
    #[error(
        "destination '{0}': name must start with a letter and contain only letters, digits, or underscores"
    )]
    InvalidDestinationName(String),
    #[error("duplicate destination name '{0}'")]
    DuplicateDestination(String),
    #[error("destination '{0}': `inputs` must not be empty")]
    EmptyInputs(String),
    #[error(
        "destination '{0}': type '{1}' is not a blessed destination type (supported: postgres; s3, file, console are reserved names not yet implemented)"
    )]
    UnsupportedType(String, String),
    #[error("destination '{0}': receives '{1}' is invalid (expected 'records' or 'files')")]
    InvalidReceives(String, String),
    #[error("destination '{0}': `receives: files` is not yet supported (only `records`)")]
    FilesNotSupported(String),
    #[error("destination '{0}': time_format '{1}' is invalid (expected 'double' or 'iso8601')")]
    InvalidTimeFormat(String, String),
    #[error("destination '{0}': missing required field `{1}`")]
    MissingField(String, String),
    #[error("destination '{0}': port {1} is out of range (expected 1-65535)")]
    InvalidPort(String, i64),
    #[error(
        "shipper.buffer_max_bytes ({0}) is below Vector's disk-buffer minimum of {MIN_DISK_BUFFER_BYTES} bytes"
    )]
    BufferTooSmall(u64),
    #[error("failed to serialize the rendered config: {0}")]
    Serialize(String),
}

/// Raw, flat Postgres parameters as ROS would declare them (`<name>.host`, `<name>.port`,
/// …). `None` means "not declared"; empty strings are treated the same as `None` so an
/// accidentally-blank YAML value still gets a clear "missing field" error rather than a
/// silently broken connection string.
#[derive(Debug, Clone, Copy, Default)]
pub struct RawPostgresParams<'a> {
    pub host: Option<&'a str>,
    pub port: Option<i64>,
    pub user: Option<&'a str>,
    pub password: Option<&'a str>,
    pub database: Option<&'a str>,
    pub table: Option<&'a str>,
    pub time_key: Option<&'a str>,
    pub time_format: Option<&'a str>,
}

impl PostgresParams {
    pub fn from_raw(name: &str, raw: RawPostgresParams) -> Result<Self, RenderError> {
        let required = |value: Option<&str>, field: &str| {
            value
                .filter(|s| !s.is_empty())
                .map(str::to_string)
                .ok_or_else(|| RenderError::MissingField(name.to_string(), field.to_string()))
        };

        let user = required(raw.user, "user")?;
        let password = required(raw.password, "password")?;
        let database = required(raw.database, "database")?;
        let table = required(raw.table, "table")?;

        let host = raw
            .host
            .filter(|s| !s.is_empty())
            .unwrap_or("127.0.0.1")
            .to_string();
        let port_raw = raw.port.unwrap_or(5432);
        let port = u16::try_from(port_raw)
            .ok()
            .filter(|p| *p != 0)
            .ok_or_else(|| RenderError::InvalidPort(name.to_string(), port_raw))?;
        let time_key = raw
            .time_key
            .filter(|s| !s.is_empty())
            .unwrap_or("date")
            .to_string();
        let time_format = match raw
            .time_format
            .filter(|s| !s.is_empty())
            .unwrap_or("double")
        {
            "double" => TimeFormat::Double,
            "iso8601" => TimeFormat::Iso8601,
            other => {
                return Err(RenderError::InvalidTimeFormat(
                    name.to_string(),
                    other.to_string(),
                ))
            }
        };

        Ok(PostgresParams {
            host,
            port,
            user,
            password,
            database,
            table,
            time_key,
            time_format,
        })
    }
}

/// Validates and builds one [`Destination`] from the flat parameters ROS would declare
/// for it. `type_str`/`receives_str` come straight from the `<name>.type` /
/// `<name>.receives` parameters.
pub fn destination_from_raw(
    name: &str,
    type_str: &str,
    receives_str: &str,
    inputs: Vec<String>,
    postgres: RawPostgresParams,
) -> Result<Destination, RenderError> {
    let receives = match receives_str {
        "records" => Receives::Records,
        "files" => Receives::Files,
        other => {
            return Err(RenderError::InvalidReceives(
                name.to_string(),
                other.to_string(),
            ))
        }
    };
    if receives != Receives::Records {
        return Err(RenderError::FilesNotSupported(name.to_string()));
    }

    let kind = match type_str {
        "postgres" => DestinationKind::Postgres(PostgresParams::from_raw(name, postgres)?),
        other => {
            return Err(RenderError::UnsupportedType(
                name.to_string(),
                other.to_string(),
            ))
        }
    };

    Ok(Destination {
        name: name.to_string(),
        receives,
        inputs,
        kind,
    })
}

#[derive(Debug, Clone, thiserror::Error, PartialEq, Eq)]
#[error("undefined environment variable '{0}' referenced in '{1}'")]
pub struct ExpandError(String, String);

/// Expands `$NAME` and `${NAME}` references in `input` using `lookup`, matching the DC
/// config contract's `$HOME`/`$DC_PG_PASSWORD`-style expansion. Takes an injected lookup
/// function (rather than reading the real environment) so it stays pure and deterministic
/// under test; callers pass `std::env::var` (or similar) in production.
pub fn expand_env(
    input: &str,
    lookup: impl Fn(&str) -> Option<String>,
) -> Result<String, ExpandError> {
    let chars: Vec<char> = input.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i] == '$' && i + 1 < chars.len() && chars[i + 1] == '{' {
            if let Some(rel_end) = chars[i + 2..].iter().position(|&c| c == '}') {
                let end = i + 2 + rel_end;
                let name: String = chars[i + 2..end].iter().collect();
                let value =
                    lookup(&name).ok_or_else(|| ExpandError(name.clone(), input.to_string()))?;
                out.push_str(&value);
                i = end + 1;
                continue;
            }
        } else if chars[i] == '$'
            && i + 1 < chars.len()
            && (chars[i + 1].is_ascii_alphabetic() || chars[i + 1] == '_')
        {
            let start = i + 1;
            let mut end = start;
            while end < chars.len() && (chars[end].is_ascii_alphanumeric() || chars[end] == '_') {
                end += 1;
            }
            let name: String = chars[start..end].iter().collect();
            let value =
                lookup(&name).ok_or_else(|| ExpandError(name.clone(), input.to_string()))?;
            out.push_str(&value);
            i = end;
            continue;
        }
        out.push(chars[i]);
        i += 1;
    }
    Ok(out)
}

fn is_valid_destination_name(name: &str) -> bool {
    let mut chars = name.chars();
    matches!(chars.next(), Some(c) if c.is_ascii_alphabetic())
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn validate(config: &RenderConfig) -> Result<(), RenderError> {
    if config.data_dir.trim().is_empty() {
        return Err(RenderError::EmptyDataDir);
    }
    if config.destinations.is_empty() {
        return Err(RenderError::NoDestinations);
    }
    if config.buffer_max_bytes < MIN_DISK_BUFFER_BYTES {
        return Err(RenderError::BufferTooSmall(config.buffer_max_bytes));
    }

    let mut seen = HashSet::new();
    for dest in &config.destinations {
        if !is_valid_destination_name(&dest.name) {
            return Err(RenderError::InvalidDestinationName(dest.name.clone()));
        }
        if !seen.insert(dest.name.as_str()) {
            return Err(RenderError::DuplicateDestination(dest.name.clone()));
        }
        if dest.inputs.is_empty() {
            return Err(RenderError::EmptyInputs(dest.name.clone()));
        }
        if dest.receives != Receives::Records {
            return Err(RenderError::FilesNotSupported(dest.name.clone()));
        }
    }
    Ok(())
}

/// The Fluent Forward tags a Destination's `inputs` topics are subscribed/forwarded
/// under — the same derivation `TopicConfig` uses, kept in lock-step so the rendered
/// `route`/`remap` conditions actually match what the Bridge forwards.
fn derived_tags(inputs: &[String]) -> Vec<String> {
    inputs
        .iter()
        .map(|topic| TopicConfig::new(topic.clone(), None).tag)
        .collect()
}

/// A VRL `includes([...], .tag)` condition matching any of `tags`.
fn tag_condition(tags: &[String]) -> String {
    let quoted: Vec<String> = tags.iter().map(|t| format!("{:?}", t)).collect();
    format!("includes([{}], .tag)", quoted.join(", "))
}

fn render_normalize_source(config: &RenderConfig) -> String {
    let mut out = String::new();
    for dest in &config.destinations {
        let DestinationKind::Postgres(pg) = &dest.kind;
        let cond = tag_condition(&derived_tags(&dest.inputs));
        writeln!(out, "if {cond} {{").unwrap();
        match pg.time_format {
            TimeFormat::Double => writeln!(
                out,
                "  .{key} = to_float(to_unix_timestamp!(.timestamp, unit: \"nanoseconds\")) / 1000000000.0",
                key = pg.time_key
            )
            .unwrap(),
            TimeFormat::Iso8601 => writeln!(
                out,
                "  .{key} = format_timestamp!(.timestamp, format: \"%Y-%m-%dT%H:%M:%S%.9f\")",
                key = pg.time_key
            )
            .unwrap(),
        }
        out.push_str("}\n");
    }
    out
}

/// Percent-encodes a postgres URI userinfo component (user/password) byte-by-byte, so
/// arbitrary credential characters (`@`, `:`, `/`, `%`, non-ASCII…) can't corrupt the
/// `postgres://user:password@host:port/db` endpoint string Vector's `postgres` sink parses.
fn percent_encode_userinfo(value: &str) -> String {
    let mut out = String::new();
    for byte in value.bytes() {
        let c = byte as char;
        if c.is_ascii_alphanumeric() || matches!(c, '-' | '.' | '_' | '~') {
            out.push(c);
        } else {
            write!(out, "%{byte:02X}").unwrap();
        }
    }
    out
}

fn render_sink(config: &RenderConfig, dest: &Destination) -> Table {
    let DestinationKind::Postgres(pg) = &dest.kind;
    let mut sink = Table::new();
    sink.insert("type".into(), Value::String("postgres".into()));
    sink.insert(
        "inputs".into(),
        Value::Array(vec![Value::String(format!(
            "{ROUTE_TRANSFORM_ID}.{}",
            dest.name
        ))]),
    );
    sink.insert(
        "endpoint".into(),
        Value::String(format!(
            "postgres://{}:{}@{}:{}/{}",
            percent_encode_userinfo(&pg.user),
            percent_encode_userinfo(&pg.password),
            pg.host,
            pg.port,
            pg.database,
        )),
    );
    sink.insert("table".into(), Value::String(pg.table.clone()));

    let mut buffer = Table::new();
    buffer.insert("type".into(), Value::String("disk".into()));
    buffer.insert(
        "max_size".into(),
        Value::Integer(config.buffer_max_bytes as i64),
    );
    sink.insert("buffer".into(), Value::Table(buffer));

    sink
}

/// Renders `config` into a complete Vector TOML configuration. Pure: no file, network,
/// ROS, or Vector-process I/O — every input is already resolved (env expansion, if any,
/// must happen before this is called; see [`expand_env`]).
pub fn render(config: &RenderConfig) -> Result<String, RenderError> {
    validate(config)?;

    let mut root = Table::new();
    root.insert("data_dir".into(), Value::String(config.data_dir.clone()));

    let mut source = Table::new();
    source.insert("type".into(), Value::String("fluent".into()));
    source.insert(
        "address".into(),
        Value::String(config.forward_addr.to_string()),
    );
    let mut sources = Table::new();
    sources.insert(SOURCE_ID.into(), Value::Table(source));
    root.insert("sources".into(), Value::Table(sources));

    let mut normalize = Table::new();
    normalize.insert("type".into(), Value::String("remap".into()));
    normalize.insert(
        "inputs".into(),
        Value::Array(vec![Value::String(SOURCE_ID.into())]),
    );
    normalize.insert(
        "source".into(),
        Value::String(render_normalize_source(config)),
    );

    let mut route_branches = Table::new();
    for dest in &config.destinations {
        let mut branch = Table::new();
        branch.insert("type".into(), Value::String("vrl".into()));
        branch.insert(
            "source".into(),
            Value::String(tag_condition(&derived_tags(&dest.inputs))),
        );
        route_branches.insert(dest.name.clone(), Value::Table(branch));
    }
    let mut route = Table::new();
    route.insert("type".into(), Value::String("route".into()));
    route.insert(
        "inputs".into(),
        Value::Array(vec![Value::String(NORMALIZE_TRANSFORM_ID.into())]),
    );
    // Every Record the Bridge forwards belongs to some Destination's `inputs`, so an
    // unmatched branch should never fire in practice; discard rather than expose an
    // `_unmatched` output nothing is meant to consume.
    route.insert("reroute_unmatched".into(), Value::Boolean(false));
    route.insert("route".into(), Value::Table(route_branches));

    let mut transforms = Table::new();
    transforms.insert(NORMALIZE_TRANSFORM_ID.into(), Value::Table(normalize));
    transforms.insert(ROUTE_TRANSFORM_ID.into(), Value::Table(route));
    root.insert("transforms".into(), Value::Table(transforms));

    let mut sinks = Table::new();
    for dest in &config.destinations {
        sinks.insert(dest.name.clone(), Value::Table(render_sink(config, dest)));
    }
    root.insert("sinks".into(), Value::Table(sinks));

    toml::to_string_pretty(&Value::Table(root)).map_err(|e| RenderError::Serialize(e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn postgres_destination(name: &str, inputs: &[&str], time_format: TimeFormat) -> Destination {
        Destination {
            name: name.to_string(),
            receives: Receives::Records,
            inputs: inputs.iter().map(|s| s.to_string()).collect(),
            kind: DestinationKind::Postgres(PostgresParams {
                host: "127.0.0.1".to_string(),
                port: 5432,
                user: "dc".to_string(),
                password: "hunter2".to_string(),
                database: "dc".to_string(),
                table: "dc".to_string(),
                time_key: "date".to_string(),
                time_format,
            }),
        }
    }

    fn basic_config(time_format: TimeFormat) -> RenderConfig {
        RenderConfig {
            forward_addr: "127.0.0.1:24224".parse().unwrap(),
            data_dir: "/home/dc/.dc/buffer".to_string(),
            buffer_max_bytes: MIN_DISK_BUFFER_BYTES,
            destinations: vec![postgres_destination(
                "pgsql",
                &["/dc/group/robot"],
                time_format,
            )],
        }
    }

    fn parsed(output: &str) -> toml::Table {
        output.parse().expect("rendered config must be valid TOML")
    }

    #[test]
    fn renders_source_transform_route_and_sink_for_a_single_postgres_destination() {
        let rendered = render(&basic_config(TimeFormat::Double)).unwrap();
        let parsed_matches = parsed(&rendered);
        insta_free_assert(
            &parsed_matches,
            include_str!("../tests/fixtures/render/basic_double.toml"),
        );
    }

    #[test]
    fn iso8601_time_format_renders_a_format_timestamp_call() {
        let rendered = render(&basic_config(TimeFormat::Iso8601)).unwrap();
        let parsed_matches = parsed(&rendered);
        insta_free_assert(
            &parsed_matches,
            include_str!("../tests/fixtures/render/iso8601.toml"),
        );
    }

    #[test]
    fn multiple_destinations_each_get_their_own_dc_dot_tag_route_and_sink() {
        let config = RenderConfig {
            forward_addr: "127.0.0.1:24224".parse().unwrap(),
            data_dir: "/home/dc/.dc/buffer".to_string(),
            buffer_max_bytes: MIN_DISK_BUFFER_BYTES,
            destinations: vec![
                postgres_destination("pgsql", &["/dc/group/robot"], TimeFormat::Double),
                postgres_destination(
                    "pgsql_archive",
                    &["/dc/measurement/uptime"],
                    TimeFormat::Iso8601,
                ),
            ],
        };
        let rendered = render(&config).unwrap();
        let parsed_matches = parsed(&rendered);
        insta_free_assert(
            &parsed_matches,
            include_str!("../tests/fixtures/render/multiple_destinations.toml"),
        );
    }

    /// Gold-file comparison via parsed `toml::Table` equality rather than raw string
    /// equality, so incidental key-ordering/formatting differences in `toml`'s
    /// pretty-printer don't make these tests brittle.
    fn insta_free_assert(actual: &toml::Table, expected_fixture: &str) {
        let expected: toml::Table = expected_fixture
            .parse()
            .expect("fixture file must be valid TOML");
        assert_eq!(actual, &expected);
    }

    #[test]
    fn tag_routes_are_rendered_under_dc_dot_name_exactly() {
        let rendered = render(&basic_config(TimeFormat::Double)).unwrap();
        let parsed = parsed(&rendered);
        let sink = parsed["sinks"]["pgsql"].as_table().unwrap();
        assert_eq!(
            sink["inputs"].as_array().unwrap(),
            &[Value::String("dc.pgsql".to_string())]
        );
        let route = parsed["transforms"]["dc"].as_table().unwrap();
        assert!(route["route"].as_table().unwrap().contains_key("pgsql"));
    }

    #[test]
    fn rejects_empty_destinations() {
        let config = RenderConfig {
            forward_addr: "127.0.0.1:24224".parse().unwrap(),
            data_dir: "/home/dc/.dc/buffer".to_string(),
            buffer_max_bytes: MIN_DISK_BUFFER_BYTES,
            destinations: vec![],
        };
        assert_eq!(render(&config), Err(RenderError::NoDestinations));
    }

    #[test]
    fn rejects_empty_data_dir() {
        let mut config = basic_config(TimeFormat::Double);
        config.data_dir = "".to_string();
        assert_eq!(render(&config), Err(RenderError::EmptyDataDir));
    }

    #[test]
    fn rejects_a_too_small_disk_buffer() {
        let mut config = basic_config(TimeFormat::Double);
        config.buffer_max_bytes = 1024;
        assert_eq!(render(&config), Err(RenderError::BufferTooSmall(1024)));
    }

    #[test]
    fn rejects_empty_inputs() {
        let mut config = basic_config(TimeFormat::Double);
        config.destinations[0].inputs = vec![];
        assert_eq!(
            render(&config),
            Err(RenderError::EmptyInputs("pgsql".to_string()))
        );
    }

    #[test]
    fn rejects_duplicate_destination_names() {
        let mut config = basic_config(TimeFormat::Double);
        let dup = config.destinations[0].clone();
        config.destinations.push(dup);
        assert_eq!(
            render(&config),
            Err(RenderError::DuplicateDestination("pgsql".to_string()))
        );
    }

    #[test]
    fn rejects_invalid_destination_names() {
        let mut config = basic_config(TimeFormat::Double);
        config.destinations[0].name = "pg-sql!".to_string();
        assert_eq!(
            render(&config),
            Err(RenderError::InvalidDestinationName("pg-sql!".to_string()))
        );
    }

    #[test]
    fn destination_from_raw_rejects_an_unsupported_blessed_type() {
        let err = destination_from_raw(
            "archive",
            "mongodb",
            "records",
            vec!["/dc/group/robot".to_string()],
            RawPostgresParams::default(),
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::UnsupportedType("archive".to_string(), "mongodb".to_string())
        );
    }

    #[test]
    fn destination_from_raw_rejects_receives_files_clearly_for_now() {
        let err = destination_from_raw(
            "archive",
            "postgres",
            "files",
            vec!["/dc/measurement/map".to_string()],
            RawPostgresParams::default(),
        )
        .unwrap_err();
        assert_eq!(err, RenderError::FilesNotSupported("archive".to_string()));
    }

    #[test]
    fn destination_from_raw_rejects_an_invalid_receives_value() {
        let err = destination_from_raw(
            "archive",
            "postgres",
            "bogus",
            vec!["/dc/measurement/map".to_string()],
            RawPostgresParams::default(),
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::InvalidReceives("archive".to_string(), "bogus".to_string())
        );
    }

    #[test]
    fn postgres_from_raw_rejects_missing_required_fields() {
        let err = PostgresParams::from_raw(
            "pgsql",
            RawPostgresParams {
                user: Some("dc"),
                ..Default::default()
            },
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::MissingField("pgsql".to_string(), "password".to_string())
        );
    }

    #[test]
    fn postgres_from_raw_rejects_an_invalid_time_format() {
        let err = PostgresParams::from_raw(
            "pgsql",
            RawPostgresParams {
                user: Some("dc"),
                password: Some("secret"),
                database: Some("dc"),
                table: Some("dc"),
                time_format: Some("rfc2822"),
                ..Default::default()
            },
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::InvalidTimeFormat("pgsql".to_string(), "rfc2822".to_string())
        );
    }

    #[test]
    fn postgres_from_raw_rejects_an_out_of_range_port() {
        let err = PostgresParams::from_raw(
            "pgsql",
            RawPostgresParams {
                user: Some("dc"),
                password: Some("secret"),
                database: Some("dc"),
                table: Some("dc"),
                port: Some(70000),
                ..Default::default()
            },
        )
        .unwrap_err();
        assert_eq!(err, RenderError::InvalidPort("pgsql".to_string(), 70000));
    }

    #[test]
    fn postgres_from_raw_applies_documented_defaults() {
        let params = PostgresParams::from_raw(
            "pgsql",
            RawPostgresParams {
                user: Some("dc"),
                password: Some("secret"),
                database: Some("dc"),
                table: Some("dc"),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(params.host, "127.0.0.1");
        assert_eq!(params.port, 5432);
        assert_eq!(params.time_key, "date");
        assert_eq!(params.time_format, TimeFormat::Double);
    }

    #[test]
    fn expand_env_substitutes_dollar_name_and_braced_form() {
        let mut env = HashMap::new();
        env.insert("HOME".to_string(), "/home/dc".to_string());
        env.insert("DC_PG_PASSWORD".to_string(), "s3cr3t".to_string());

        assert_eq!(
            expand_env("$HOME/.dc/buffer", |k| env.get(k).cloned()),
            Ok("/home/dc/.dc/buffer".to_string())
        );
        assert_eq!(
            expand_env("${DC_PG_PASSWORD}", |k| env.get(k).cloned()),
            Ok("s3cr3t".to_string())
        );
    }

    #[test]
    fn expand_env_rejects_an_undefined_variable() {
        let result = expand_env("$MISSING", |_| None);
        assert_eq!(
            result,
            Err(ExpandError("MISSING".to_string(), "$MISSING".to_string()))
        );
    }

    #[test]
    fn percent_encodes_special_characters_in_credentials() {
        let mut config = basic_config(TimeFormat::Double);
        let DestinationKind::Postgres(pg) = &mut config.destinations[0].kind;
        pg.password = "p@ss/w:rd%".to_string();
        let rendered = render(&config).unwrap();
        let parsed = parsed(&rendered);
        let endpoint = parsed["sinks"]["pgsql"]["endpoint"].as_str().unwrap();
        assert_eq!(
            endpoint,
            "postgres://dc:p%40ss%2Fw%3Ard%25@127.0.0.1:5432/dc"
        );
    }
}
