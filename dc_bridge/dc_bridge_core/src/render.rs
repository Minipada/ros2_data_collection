//! Config renderer (ADR-0003): a pure, I/O-free mapping from Bridge/Destination
//! parameters to a complete Vector configuration. Given a fully-resolved [`RenderConfig`],
//! [`render`] produces Vector TOML with: the Fluent Forward source, one VRL transform
//! that normalizes each blessed destination's timestamp field (replacing the Humble-era
//! chained Lua filters — see `dc_destinations/include/dc_destinations/flb_destination.hpp`
//! on the `humble` branch), one `route` transform exposing the public per-Tag routes
//! (the passthrough contract from ADR-0003 — see [`route_output_for_tag`]), disk-buffer
//! settings, and the blessed sinks themselves (`postgres`, `s3`, `file`, `console`).
//!
//! Two layers, both pure and independently testable:
//! - [`destination_from_raw`] (with the per-type `*Params::from_raw` constructors) turns
//!   the flat, stringly-typed values ROS parameters naturally give us into validated,
//!   typed config — this is where "invalid-parameter rejection with clear error
//!   messages" lives.
//! - [`render`] turns already-validated, typed [`RenderConfig`] into Vector TOML text.
//!
//! [`validate_custom_config_files`] checks the passthrough side of ADR-0003: raw Vector
//! config snippets (the `custom_config_files` parameter) must parse and must not claim
//! component ids the rendered config already owns — so a bad snippet fails loudly at
//! Bridge startup instead of crash-looping the supervised Vector.
//!
//! [`expand_env`] handles the `$HOME`/`$DC_PG_PASSWORD`-style expansion the config
//! contract uses for `shipper.data_dir` and destination credentials; it takes an
//! injected lookup function rather than reading the real environment, so it stays pure
//! too.

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::fmt::Write as _;
use std::net::SocketAddr;
use toml::value::Table;
use toml::Value;

use crate::config::TopicConfig;

const SOURCE_ID: &str = "dc_bridge_in";
const NORMALIZE_TRANSFORM_ID: &str = "dc_bridge_normalize";
/// The stable public route-transform id ADR-0003 fixes: combined with Vector's
/// `<transform_id>.<route_id>` addressing convention it makes every Tag's Records
/// addressable as `dc.<tag>` — see [`route_output_for_tag`].
pub const ROUTE_TRANSFORM_ID: &str = "dc";
/// Component ids the rendered config owns; Destination names and passthrough snippets
/// must not claim them.
const RESERVED_COMPONENT_IDS: [&str; 3] = [SOURCE_ID, NORMALIZE_TRANSFORM_ID, ROUTE_TRANSFORM_ID];
/// Vector rejects a disk buffer's `max_size` below this (~256 MiB); see the `postgres`
/// sink reference docs.
pub const MIN_DISK_BUFFER_BYTES: u64 = 268_435_488;

/// The public Shipper route name a Tag's Records are exposed under — ADR-0003's
/// passthrough contract, documented as API in `doc/src/dc/destinations.md`. The Tag for
/// topic `/dc/measurement/uptime` is `dc.measurement.uptime`, so its route is
/// `dc.dc.measurement.uptime`: the leading `dc.` names the route transform, the rest is
/// the Tag verbatim.
pub fn route_output_for_tag(tag: &str) -> String {
    format!("{ROUTE_TRANSFORM_ID}.{tag}")
}

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
    /// Event field the normalized timestamp is written to before routing.
    pub time_key: String,
    pub time_format: TimeFormat,
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
    S3(S3Params),
    File(FileParams),
    Console,
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
}

/// S3-compatible object storage (`type: s3`). With only `bucket` (and usually `region`)
/// set, credentials come from the ambient AWS environment/instance profile; a
/// MinIO-style server takes `endpoint`, explicit `access_key_id`/`secret_access_key`,
/// and (typically) `force_path_style: true`.
#[derive(Debug, Clone, PartialEq)]
pub struct S3Params {
    pub bucket: String,
    pub region: Option<String>,
    /// Custom S3-compatible endpoint URL (MinIO, Ceph RGW, …); omit for AWS.
    pub endpoint: Option<String>,
    pub key_prefix: Option<String>,
    pub auth: Option<S3Auth>,
    pub force_path_style: Option<bool>,
    /// Maximum seconds a batch buffers before an object is written; omitted, Vector's
    /// own default (300 s) applies.
    pub batch_timeout_secs: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct S3Auth {
    pub access_key_id: String,
    pub secret_access_key: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FileParams {
    /// Output path; Vector template syntax (e.g. `/var/log/dc/records-%Y-%m-%d.log`)
    /// is passed through untouched.
    pub path: String,
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
    #[error(
        "destination '{0}': name is reserved by the DC-generated Vector config; pick another name"
    )]
    ReservedDestinationName(String),
    #[error("duplicate destination name '{0}'")]
    DuplicateDestination(String),
    #[error("destination '{0}': `inputs` must not be empty")]
    EmptyInputs(String),
    #[error(
        "destination '{0}': type '{1}' is not a blessed destination type (supported: postgres, s3, file, console; other Vector sinks are available via `custom_config_files`)"
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
        "destination '{0}': access_key_id and secret_access_key must be set together (got only one)"
    )]
    IncompleteS3Auth(String),
    #[error("destination '{0}': batch_timeout_secs {1} is invalid (expected a positive integer)")]
    InvalidBatchTimeout(String, i64),
    #[error(
        "shipper.buffer_max_bytes ({0}) is below Vector's disk-buffer minimum of {MIN_DISK_BUFFER_BYTES} bytes"
    )]
    BufferTooSmall(u64),
    #[error("custom config file '{0}' is not valid TOML: {1}")]
    CustomConfigParse(String, String),
    #[error(
        "custom config file '{0}' defines no sources, transforms, sinks, or enrichment tables"
    )]
    CustomConfigEmpty(String),
    #[error(
        "custom config file '{0}' defines component '{1}', which collides with a component of the DC-generated Vector config"
    )]
    CustomConfigReservedCollision(String, String),
    #[error("custom config files '{0}' and '{1}' both define component '{2}'")]
    CustomConfigDuplicate(String, String, String),
    #[error("failed to serialize the rendered config: {0}")]
    Serialize(String),
}

/// Raw, flat Destination parameters as ROS would declare them (`<name>.host`,
/// `<name>.bucket`, …), spanning every blessed type; each type's `from_raw` picks the
/// fields it needs. `None` means "not declared"; empty strings are treated the same as
/// `None` so an accidentally-blank YAML value still gets a clear "missing field" error
/// rather than a silently broken config.
#[derive(Debug, Clone, Copy, Default)]
pub struct RawDestinationParams<'a> {
    // Shared
    pub time_key: Option<&'a str>,
    pub time_format: Option<&'a str>,
    // postgres
    pub host: Option<&'a str>,
    pub port: Option<i64>,
    pub user: Option<&'a str>,
    pub password: Option<&'a str>,
    pub database: Option<&'a str>,
    pub table: Option<&'a str>,
    // s3
    pub bucket: Option<&'a str>,
    pub region: Option<&'a str>,
    pub endpoint: Option<&'a str>,
    pub key_prefix: Option<&'a str>,
    pub access_key_id: Option<&'a str>,
    pub secret_access_key: Option<&'a str>,
    pub force_path_style: Option<bool>,
    pub batch_timeout_secs: Option<i64>,
    // file
    pub path: Option<&'a str>,
}

fn required(name: &str, value: Option<&str>, field: &str) -> Result<String, RenderError> {
    value
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .ok_or_else(|| RenderError::MissingField(name.to_string(), field.to_string()))
}

fn optional(value: Option<&str>) -> Option<String> {
    value.filter(|s| !s.is_empty()).map(str::to_string)
}

impl PostgresParams {
    pub fn from_raw(name: &str, raw: &RawDestinationParams) -> Result<Self, RenderError> {
        let user = required(name, raw.user, "user")?;
        let password = required(name, raw.password, "password")?;
        let database = required(name, raw.database, "database")?;
        let table = required(name, raw.table, "table")?;

        let host = optional(raw.host).unwrap_or_else(|| "127.0.0.1".to_string());
        let port_raw = raw.port.unwrap_or(5432);
        let port = u16::try_from(port_raw)
            .ok()
            .filter(|p| *p != 0)
            .ok_or_else(|| RenderError::InvalidPort(name.to_string(), port_raw))?;

        Ok(PostgresParams {
            host,
            port,
            user,
            password,
            database,
            table,
        })
    }
}

impl S3Params {
    pub fn from_raw(name: &str, raw: &RawDestinationParams) -> Result<Self, RenderError> {
        let bucket = required(name, raw.bucket, "bucket")?;
        let auth = match (optional(raw.access_key_id), optional(raw.secret_access_key)) {
            (Some(access_key_id), Some(secret_access_key)) => Some(S3Auth {
                access_key_id,
                secret_access_key,
            }),
            (None, None) => None,
            _ => return Err(RenderError::IncompleteS3Auth(name.to_string())),
        };
        let batch_timeout_secs = raw
            .batch_timeout_secs
            .map(|secs| {
                u64::try_from(secs)
                    .ok()
                    .filter(|s| *s > 0)
                    .ok_or(RenderError::InvalidBatchTimeout(name.to_string(), secs))
            })
            .transpose()?;

        Ok(S3Params {
            bucket,
            region: optional(raw.region),
            endpoint: optional(raw.endpoint),
            key_prefix: optional(raw.key_prefix),
            auth,
            force_path_style: raw.force_path_style,
            batch_timeout_secs,
        })
    }
}

impl FileParams {
    pub fn from_raw(name: &str, raw: &RawDestinationParams) -> Result<Self, RenderError> {
        Ok(FileParams {
            path: required(name, raw.path, "path")?,
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
    raw: RawDestinationParams,
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

    let time_key = optional(raw.time_key).unwrap_or_else(|| "date".to_string());
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

    let kind = match type_str {
        "postgres" => DestinationKind::Postgres(PostgresParams::from_raw(name, &raw)?),
        "s3" => DestinationKind::S3(S3Params::from_raw(name, &raw)?),
        "file" => DestinationKind::File(FileParams::from_raw(name, &raw)?),
        "console" => DestinationKind::Console,
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
        time_key,
        time_format,
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
        if RESERVED_COMPONENT_IDS.contains(&dest.name.as_str()) {
            return Err(RenderError::ReservedDestinationName(dest.name.clone()));
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

/// The component ids the rendered config claims for itself: the fixed
/// source/transform ids plus one sink per Destination.
fn rendered_component_ids(config: &RenderConfig) -> BTreeSet<String> {
    RESERVED_COMPONENT_IDS
        .iter()
        .map(|id| id.to_string())
        .chain(config.destinations.iter().map(|d| d.name.clone()))
        .collect()
}

/// One raw Vector config snippet from the `custom_config_files` parameter: `path` is
/// only used in error messages, `content` is the file's TOML text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CustomConfigFile {
    pub path: String,
    pub content: String,
}

/// Validates passthrough snippets (ADR-0003) against the rendered config: each must be
/// valid TOML, define at least one component, and no component id may collide with the
/// DC-generated config or another snippet — Vector would reject the merged config
/// anyway, but this turns it into a clear Bridge startup error naming the offending
/// file instead of a supervised-Vector crash loop.
pub fn validate_custom_config_files(
    config: &RenderConfig,
    files: &[CustomConfigFile],
) -> Result<(), RenderError> {
    let mut owners: BTreeMap<String, Option<String>> = rendered_component_ids(config)
        .into_iter()
        .map(|id| (id, None))
        .collect();

    for file in files {
        let parsed: Table = file.content.parse().map_err(|e: toml::de::Error| {
            RenderError::CustomConfigParse(file.path.clone(), e.to_string())
        })?;

        let mut component_ids = Vec::new();
        for section in ["sources", "transforms", "sinks", "enrichment_tables"] {
            if let Some(Value::Table(components)) = parsed.get(section) {
                component_ids.extend(components.keys().cloned());
            }
        }
        if component_ids.is_empty() {
            return Err(RenderError::CustomConfigEmpty(file.path.clone()));
        }

        for id in component_ids {
            match owners.get(&id) {
                Some(None) => {
                    return Err(RenderError::CustomConfigReservedCollision(
                        file.path.clone(),
                        id,
                    ))
                }
                Some(Some(other_path)) => {
                    return Err(RenderError::CustomConfigDuplicate(
                        other_path.clone(),
                        file.path.clone(),
                        id,
                    ))
                }
                None => {
                    owners.insert(id, Some(file.path.clone()));
                }
            }
        }
    }
    Ok(())
}

/// The Fluent Forward tags a Destination's `inputs` topics are subscribed/forwarded
/// under — the same derivation `TopicConfig` uses, kept in lock-step so the rendered
/// `route`/`remap` conditions actually match what the Bridge forwards.
fn derived_tags(inputs: &[String]) -> BTreeSet<String> {
    inputs
        .iter()
        .map(|topic| TopicConfig::new(topic.clone(), None).tag)
        .collect()
}

/// A VRL `includes([...], .tag)` condition matching any of `tags`.
fn tag_condition(tags: &BTreeSet<String>) -> String {
    let quoted: Vec<String> = tags.iter().map(|t| format!("{:?}", t)).collect();
    format!("includes([{}], .tag)", quoted.join(", "))
}

fn render_normalize_source(config: &RenderConfig) -> String {
    let mut out = String::new();
    for dest in &config.destinations {
        let cond = tag_condition(&derived_tags(&dest.inputs));
        writeln!(out, "if {cond} {{").unwrap();
        match dest.time_format {
            TimeFormat::Double => writeln!(
                out,
                "  .{key} = to_float(to_unix_timestamp!(.timestamp, unit: \"nanoseconds\")) / 1000000000.0",
                key = dest.time_key
            )
            .unwrap(),
            TimeFormat::Iso8601 => writeln!(
                out,
                "  .{key} = format_timestamp!(.timestamp, format: \"%Y-%m-%dT%H:%M:%S%.9f\")",
                key = dest.time_key
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

/// The `dc.<tag>` route outputs feeding a Destination's sink — one per distinct Tag of
/// its `inputs` topics.
fn sink_inputs(dest: &Destination) -> Value {
    Value::Array(
        derived_tags(&dest.inputs)
            .iter()
            .map(|tag| Value::String(route_output_for_tag(tag)))
            .collect(),
    )
}

fn disk_buffer(config: &RenderConfig) -> Value {
    let mut buffer = Table::new();
    buffer.insert("type".into(), Value::String("disk".into()));
    buffer.insert(
        "max_size".into(),
        Value::Integer(config.buffer_max_bytes as i64),
    );
    Value::Table(buffer)
}

fn json_encoding() -> Value {
    let mut encoding = Table::new();
    encoding.insert("codec".into(), Value::String("json".into()));
    Value::Table(encoding)
}

fn render_sink(config: &RenderConfig, dest: &Destination) -> Table {
    let mut sink = Table::new();
    sink.insert("inputs".into(), sink_inputs(dest));
    match &dest.kind {
        DestinationKind::Postgres(pg) => {
            sink.insert("type".into(), Value::String("postgres".into()));
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
            sink.insert("buffer".into(), disk_buffer(config));
        }
        DestinationKind::S3(s3) => {
            sink.insert("type".into(), Value::String("aws_s3".into()));
            sink.insert("bucket".into(), Value::String(s3.bucket.clone()));
            if let Some(region) = &s3.region {
                sink.insert("region".into(), Value::String(region.clone()));
            }
            if let Some(endpoint) = &s3.endpoint {
                sink.insert("endpoint".into(), Value::String(endpoint.clone()));
            }
            if let Some(key_prefix) = &s3.key_prefix {
                sink.insert("key_prefix".into(), Value::String(key_prefix.clone()));
            }
            if let Some(force_path_style) = s3.force_path_style {
                sink.insert("force_path_style".into(), Value::Boolean(force_path_style));
            }
            if let Some(auth) = &s3.auth {
                let mut auth_table = Table::new();
                auth_table.insert(
                    "access_key_id".into(),
                    Value::String(auth.access_key_id.clone()),
                );
                auth_table.insert(
                    "secret_access_key".into(),
                    Value::String(auth.secret_access_key.clone()),
                );
                sink.insert("auth".into(), Value::Table(auth_table));
            }
            if let Some(timeout) = s3.batch_timeout_secs {
                let mut batch = Table::new();
                batch.insert("timeout_secs".into(), Value::Integer(timeout as i64));
                sink.insert("batch".into(), Value::Table(batch));
            }
            sink.insert("encoding".into(), json_encoding());
            sink.insert("buffer".into(), disk_buffer(config));
        }
        DestinationKind::File(file) => {
            sink.insert("type".into(), Value::String("file".into()));
            sink.insert("path".into(), Value::String(file.path.clone()));
            sink.insert("encoding".into(), json_encoding());
            sink.insert("buffer".into(), disk_buffer(config));
        }
        DestinationKind::Console => {
            sink.insert("type".into(), Value::String("console".into()));
            sink.insert("target".into(), Value::String("stdout".into()));
            sink.insert("encoding".into(), json_encoding());
            // No disk buffer: console is a debugging sink; the in-memory default is
            // the right behavior (and a disk buffer would replay old output on start).
        }
    }
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

    // One route branch per distinct Tag across every Destination's `inputs` — the
    // public `dc.<tag>` outputs of ADR-0003's passthrough contract. Blessed sinks and
    // custom snippets alike consume them.
    let all_tags: BTreeSet<String> = config
        .destinations
        .iter()
        .flat_map(|dest| derived_tags(&dest.inputs))
        .collect();
    let mut route_branches = Table::new();
    for tag in &all_tags {
        let mut branch = Table::new();
        branch.insert("type".into(), Value::String("vrl".into()));
        branch.insert("source".into(), Value::String(format!(".tag == {tag:?}")));
        route_branches.insert(tag.clone(), Value::Table(branch));
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

    fn destination(
        name: &str,
        inputs: &[&str],
        time_format: TimeFormat,
        kind: DestinationKind,
    ) -> Destination {
        Destination {
            name: name.to_string(),
            receives: Receives::Records,
            inputs: inputs.iter().map(|s| s.to_string()).collect(),
            time_key: "date".to_string(),
            time_format,
            kind,
        }
    }

    fn postgres_kind() -> DestinationKind {
        DestinationKind::Postgres(PostgresParams {
            host: "127.0.0.1".to_string(),
            port: 5432,
            user: "dc".to_string(),
            password: "hunter2".to_string(),
            database: "dc".to_string(),
            table: "dc".to_string(),
        })
    }

    fn config_with(destinations: Vec<Destination>) -> RenderConfig {
        RenderConfig {
            forward_addr: "127.0.0.1:24224".parse().unwrap(),
            data_dir: "/home/dc/.dc/buffer".to_string(),
            buffer_max_bytes: MIN_DISK_BUFFER_BYTES,
            destinations,
        }
    }

    fn basic_config(time_format: TimeFormat) -> RenderConfig {
        config_with(vec![destination(
            "pgsql",
            &["/dc/group/robot"],
            time_format,
            postgres_kind(),
        )])
    }

    fn parsed(output: &str) -> toml::Table {
        output.parse().expect("rendered config must be valid TOML")
    }

    /// Gold-file comparison via parsed `toml::Table` equality rather than raw string
    /// equality, so incidental key-ordering/formatting differences in `toml`'s
    /// pretty-printer don't make these tests brittle.
    fn assert_matches_fixture(actual: &toml::Table, expected_fixture: &str) {
        let expected: toml::Table = expected_fixture
            .parse()
            .expect("fixture file must be valid TOML");
        assert_eq!(actual, &expected);
    }

    #[test]
    fn renders_source_transform_route_and_sink_for_a_single_postgres_destination() {
        let rendered = render(&basic_config(TimeFormat::Double)).unwrap();
        assert_matches_fixture(
            &parsed(&rendered),
            include_str!("../tests/fixtures/render/basic_double.toml"),
        );
    }

    #[test]
    fn iso8601_time_format_renders_a_format_timestamp_call() {
        let rendered = render(&basic_config(TimeFormat::Iso8601)).unwrap();
        assert_matches_fixture(
            &parsed(&rendered),
            include_str!("../tests/fixtures/render/iso8601.toml"),
        );
    }

    #[test]
    fn multiple_destinations_share_per_tag_routes_and_get_their_own_sinks() {
        let config = config_with(vec![
            destination(
                "pgsql",
                &["/dc/group/robot"],
                TimeFormat::Double,
                postgres_kind(),
            ),
            destination(
                "pgsql_archive",
                &["/dc/measurement/uptime"],
                TimeFormat::Iso8601,
                postgres_kind(),
            ),
        ]);
        let rendered = render(&config).unwrap();
        assert_matches_fixture(
            &parsed(&rendered),
            include_str!("../tests/fixtures/render/multiple_destinations.toml"),
        );
    }

    #[test]
    fn renders_an_s3_destination_with_a_minio_style_custom_endpoint() {
        let config = config_with(vec![destination(
            "minio",
            &["/dc/group/robot", "/dc/measurement/uptime"],
            TimeFormat::Double,
            DestinationKind::S3(S3Params {
                bucket: "dc-records".to_string(),
                region: Some("us-east-1".to_string()),
                endpoint: Some("http://127.0.0.1:9000".to_string()),
                key_prefix: Some("robot1/".to_string()),
                auth: Some(S3Auth {
                    access_key_id: "minioadmin".to_string(),
                    secret_access_key: "minioadmin".to_string(),
                }),
                force_path_style: Some(true),
                batch_timeout_secs: Some(2),
            }),
        )]);
        let rendered = render(&config).unwrap();
        assert_matches_fixture(
            &parsed(&rendered),
            include_str!("../tests/fixtures/render/s3_minio.toml"),
        );
    }

    #[test]
    fn renders_an_aws_s3_destination_with_ambient_credentials() {
        let config = config_with(vec![destination(
            "s3_archive",
            &["/dc/group/robot"],
            TimeFormat::Iso8601,
            DestinationKind::S3(S3Params {
                bucket: "dc-records".to_string(),
                region: Some("eu-west-1".to_string()),
                endpoint: None,
                key_prefix: None,
                auth: None,
                force_path_style: None,
                batch_timeout_secs: None,
            }),
        )]);
        let rendered = render(&config).unwrap();
        assert_matches_fixture(
            &parsed(&rendered),
            include_str!("../tests/fixtures/render/s3_aws.toml"),
        );
    }

    #[test]
    fn renders_file_and_console_destinations() {
        let config = config_with(vec![
            destination(
                "local_log",
                &["/dc/group/robot"],
                TimeFormat::Double,
                DestinationKind::File(FileParams {
                    path: "/var/log/dc/records-%Y-%m-%d.log".to_string(),
                }),
            ),
            destination(
                "debug_console",
                &["/dc/group/robot"],
                TimeFormat::Double,
                DestinationKind::Console,
            ),
        ]);
        let rendered = render(&config).unwrap();
        assert_matches_fixture(
            &parsed(&rendered),
            include_str!("../tests/fixtures/render/file_and_console.toml"),
        );
    }

    #[test]
    fn console_sink_has_no_disk_buffer() {
        let config = config_with(vec![destination(
            "debug_console",
            &["/dc/group/robot"],
            TimeFormat::Double,
            DestinationKind::Console,
        )]);
        let rendered = render(&config).unwrap();
        let parsed = parsed(&rendered);
        let sink = parsed["sinks"]["debug_console"].as_table().unwrap();
        assert!(!sink.contains_key("buffer"));
    }

    #[test]
    fn routes_are_per_tag_and_sinks_consume_dc_dot_tag_outputs() {
        let config = config_with(vec![
            destination(
                "pgsql",
                &["/dc/group/robot", "/dc/measurement/uptime"],
                TimeFormat::Double,
                postgres_kind(),
            ),
            destination(
                "debug_console",
                &["/dc/measurement/uptime"],
                TimeFormat::Double,
                DestinationKind::Console,
            ),
        ]);
        let rendered = render(&config).unwrap();
        let parsed = parsed(&rendered);

        let route = parsed["transforms"]["dc"]["route"].as_table().unwrap();
        assert_eq!(
            route.keys().collect::<Vec<_>>(),
            vec!["dc.group.robot", "dc.measurement.uptime"]
        );

        let pgsql_inputs = parsed["sinks"]["pgsql"]["inputs"].as_array().unwrap();
        assert_eq!(
            pgsql_inputs,
            &[
                Value::String("dc.dc.group.robot".to_string()),
                Value::String("dc.dc.measurement.uptime".to_string()),
            ]
        );
        let console_inputs = parsed["sinks"]["debug_console"]["inputs"]
            .as_array()
            .unwrap();
        assert_eq!(
            console_inputs,
            &[Value::String("dc.dc.measurement.uptime".to_string())]
        );
    }

    #[test]
    fn route_output_for_tag_is_the_documented_public_name() {
        assert_eq!(
            route_output_for_tag("dc.measurement.uptime"),
            "dc.dc.measurement.uptime"
        );
    }

    #[test]
    fn rejects_empty_destinations() {
        let config = config_with(vec![]);
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
    fn rejects_destination_names_reserved_by_the_rendered_config() {
        for reserved in ["dc", "dc_bridge_in", "dc_bridge_normalize"] {
            let mut config = basic_config(TimeFormat::Double);
            config.destinations[0].name = reserved.to_string();
            assert_eq!(
                render(&config),
                Err(RenderError::ReservedDestinationName(reserved.to_string())),
                "name '{reserved}' must be rejected"
            );
        }
    }

    #[test]
    fn destination_from_raw_rejects_an_unsupported_blessed_type() {
        let err = destination_from_raw(
            "archive",
            "mongodb",
            "records",
            vec!["/dc/group/robot".to_string()],
            RawDestinationParams::default(),
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::UnsupportedType("archive".to_string(), "mongodb".to_string())
        );
    }

    #[test]
    fn destination_from_raw_builds_s3_file_and_console_kinds() {
        let s3 = destination_from_raw(
            "minio",
            "s3",
            "records",
            vec!["/dc/group/robot".to_string()],
            RawDestinationParams {
                bucket: Some("dc-records"),
                endpoint: Some("http://127.0.0.1:9000"),
                access_key_id: Some("minioadmin"),
                secret_access_key: Some("minioadmin"),
                force_path_style: Some(true),
                ..Default::default()
            },
        )
        .unwrap();
        assert!(matches!(s3.kind, DestinationKind::S3(_)));

        let file = destination_from_raw(
            "local_log",
            "file",
            "records",
            vec!["/dc/group/robot".to_string()],
            RawDestinationParams {
                path: Some("/var/log/dc/records.log"),
                ..Default::default()
            },
        )
        .unwrap();
        assert!(matches!(file.kind, DestinationKind::File(_)));

        let console = destination_from_raw(
            "debug_console",
            "console",
            "records",
            vec!["/dc/group/robot".to_string()],
            RawDestinationParams::default(),
        )
        .unwrap();
        assert_eq!(console.kind, DestinationKind::Console);
    }

    #[test]
    fn destination_from_raw_rejects_receives_files_clearly_for_now() {
        let err = destination_from_raw(
            "archive",
            "postgres",
            "files",
            vec!["/dc/measurement/map".to_string()],
            RawDestinationParams::default(),
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
            RawDestinationParams::default(),
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::InvalidReceives("archive".to_string(), "bogus".to_string())
        );
    }

    #[test]
    fn destination_from_raw_rejects_an_invalid_time_format() {
        let err = destination_from_raw(
            "debug_console",
            "console",
            "records",
            vec!["/dc/measurement/map".to_string()],
            RawDestinationParams {
                time_format: Some("rfc2822"),
                ..Default::default()
            },
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::InvalidTimeFormat("debug_console".to_string(), "rfc2822".to_string())
        );
    }

    #[test]
    fn postgres_from_raw_rejects_missing_required_fields() {
        let err = PostgresParams::from_raw(
            "pgsql",
            &RawDestinationParams {
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
    fn postgres_from_raw_rejects_an_out_of_range_port() {
        let err = PostgresParams::from_raw(
            "pgsql",
            &RawDestinationParams {
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
            &RawDestinationParams {
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
    }

    #[test]
    fn destination_from_raw_applies_time_defaults() {
        let dest = destination_from_raw(
            "debug_console",
            "console",
            "records",
            vec!["/dc/group/robot".to_string()],
            RawDestinationParams::default(),
        )
        .unwrap();
        assert_eq!(dest.time_key, "date");
        assert_eq!(dest.time_format, TimeFormat::Double);
    }

    #[test]
    fn s3_from_raw_rejects_a_missing_bucket() {
        let err = S3Params::from_raw("minio", &RawDestinationParams::default()).unwrap_err();
        assert_eq!(
            err,
            RenderError::MissingField("minio".to_string(), "bucket".to_string())
        );
    }

    #[test]
    fn s3_from_raw_rejects_half_a_credential_pair() {
        let err = S3Params::from_raw(
            "minio",
            &RawDestinationParams {
                bucket: Some("dc-records"),
                access_key_id: Some("minioadmin"),
                ..Default::default()
            },
        )
        .unwrap_err();
        assert_eq!(err, RenderError::IncompleteS3Auth("minio".to_string()));
    }

    #[test]
    fn s3_from_raw_rejects_a_non_positive_batch_timeout() {
        let err = S3Params::from_raw(
            "minio",
            &RawDestinationParams {
                bucket: Some("dc-records"),
                batch_timeout_secs: Some(0),
                ..Default::default()
            },
        )
        .unwrap_err();
        assert_eq!(
            err,
            RenderError::InvalidBatchTimeout("minio".to_string(), 0)
        );
    }

    #[test]
    fn file_from_raw_rejects_a_missing_path() {
        let err = FileParams::from_raw("local_log", &RawDestinationParams::default()).unwrap_err();
        assert_eq!(
            err,
            RenderError::MissingField("local_log".to_string(), "path".to_string())
        );
    }

    #[test]
    fn custom_config_files_pass_when_they_define_fresh_components() {
        let config = basic_config(TimeFormat::Double);
        let files = vec![CustomConfigFile {
            path: "/etc/dc/http.toml".to_string(),
            content: r#"
[sinks.my_http]
type = "http"
inputs = ["dc.dc.group.robot"]
uri = "http://127.0.0.1:8080/ingest"
encoding.codec = "json"
"#
            .to_string(),
        }];
        assert_eq!(validate_custom_config_files(&config, &files), Ok(()));
    }

    #[test]
    fn custom_config_files_reject_invalid_toml() {
        let config = basic_config(TimeFormat::Double);
        let files = vec![CustomConfigFile {
            path: "/etc/dc/broken.toml".to_string(),
            content: "[sinks.broken\ntype =".to_string(),
        }];
        assert!(matches!(
            validate_custom_config_files(&config, &files),
            Err(RenderError::CustomConfigParse(path, _)) if path == "/etc/dc/broken.toml"
        ));
    }

    #[test]
    fn custom_config_files_reject_an_empty_snippet() {
        let config = basic_config(TimeFormat::Double);
        let files = vec![CustomConfigFile {
            path: "/etc/dc/empty.toml".to_string(),
            content: "# nothing here\n".to_string(),
        }];
        assert_eq!(
            validate_custom_config_files(&config, &files),
            Err(RenderError::CustomConfigEmpty(
                "/etc/dc/empty.toml".to_string()
            ))
        );
    }

    #[test]
    fn custom_config_files_reject_collisions_with_rendered_components() {
        let config = basic_config(TimeFormat::Double);
        for id in ["pgsql", "dc", "dc_bridge_in", "dc_bridge_normalize"] {
            let files = vec![CustomConfigFile {
                path: "/etc/dc/collide.toml".to_string(),
                content: format!(
                    "[sinks.{id}]\ntype = \"console\"\ninputs = [\"dc.dc.group.robot\"]\nencoding.codec = \"json\"\n"
                ),
            }];
            assert_eq!(
                validate_custom_config_files(&config, &files),
                Err(RenderError::CustomConfigReservedCollision(
                    "/etc/dc/collide.toml".to_string(),
                    id.to_string()
                )),
                "component id '{id}' must be rejected"
            );
        }
    }

    #[test]
    fn custom_config_files_reject_the_same_component_in_two_snippets() {
        let config = basic_config(TimeFormat::Double);
        let snippet = |path: &str| CustomConfigFile {
            path: path.to_string(),
            content: r#"
[sinks.my_http]
type = "http"
inputs = ["dc.dc.group.robot"]
uri = "http://127.0.0.1:8080/ingest"
encoding.codec = "json"
"#
            .to_string(),
        };
        assert_eq!(
            validate_custom_config_files(&config, &[snippet("/a.toml"), snippet("/b.toml")]),
            Err(RenderError::CustomConfigDuplicate(
                "/a.toml".to_string(),
                "/b.toml".to_string(),
                "my_http".to_string()
            ))
        );
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
        let DestinationKind::Postgres(pg) = &mut config.destinations[0].kind else {
            panic!("basic_config must hold a postgres destination");
        };
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
