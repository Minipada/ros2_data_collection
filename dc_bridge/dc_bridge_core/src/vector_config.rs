//! Renders the minimal Vector config the Supervisor hands to the vendored binary: a
//! Fluent Forward source feeding a console sink. Just enough to prove Records reach
//! Vector end-to-end (the tracer bullet); real deployments will add Destinations here
//! per ADR-0003 without touching the Bridge.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};

const SOURCE_ID: &str = "dc_bridge_in";
const SINK_ID: &str = "dc_console";

/// Locates the vendored Vector binary by scanning a colon-separated `AMENT_PREFIX_PATH`
/// (as colcon sets it) for `lib/vector_vendor/vector`, the install path `vector_vendor`'s
/// CMakeLists.txt installs to.
pub fn find_vector_binary(ament_prefix_path: &str) -> Option<PathBuf> {
    ament_prefix_path
        .split(':')
        .filter(|prefix| !prefix.is_empty())
        .map(|prefix| Path::new(prefix).join("lib/vector_vendor/vector"))
        .find(|candidate| candidate.is_file())
}

pub fn render(forward_addr: SocketAddr) -> String {
    format!(
        "[sources.{source}]\n\
         type = \"fluent\"\n\
         address = \"{addr}\"\n\
         \n\
         [sinks.{sink}]\n\
         type = \"console\"\n\
         inputs = [\"{source}\"]\n\
         target = \"stdout\"\n\
         encoding.codec = \"json\"\n",
        source = SOURCE_ID,
        sink = SINK_ID,
        addr = forward_addr,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renders_a_fluent_source_and_console_sink_bound_to_the_given_address() {
        let config = render("127.0.0.1:24224".parse().unwrap());
        let parsed: toml::Table = config.parse().expect("rendered config must be valid TOML");

        let source = parsed["sources"][SOURCE_ID].as_table().unwrap();
        assert_eq!(source["type"].as_str(), Some("fluent"));
        assert_eq!(source["address"].as_str(), Some("127.0.0.1:24224"));

        let sink = parsed["sinks"][SINK_ID].as_table().unwrap();
        assert_eq!(sink["type"].as_str(), Some("console"));
        assert_eq!(
            sink["inputs"].as_array().unwrap(),
            &[toml::Value::String(SOURCE_ID.to_string())]
        );
    }

    #[test]
    fn finds_vector_binary_in_the_first_prefix_that_has_it() {
        let base = std::env::temp_dir().join(format!(
            "dc_bridge_test_{}_{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let empty_prefix = base.join("empty");
        let real_prefix = base.join("real");
        let vector_dir = real_prefix.join("lib/vector_vendor");
        std::fs::create_dir_all(&empty_prefix).unwrap();
        std::fs::create_dir_all(&vector_dir).unwrap();
        std::fs::write(vector_dir.join("vector"), b"#!/bin/sh\n").unwrap();

        let ament_prefix_path = format!("{}:{}", empty_prefix.display(), real_prefix.display());
        let found = find_vector_binary(&ament_prefix_path);

        std::fs::remove_dir_all(&base).ok();

        assert_eq!(found, Some(vector_dir.join("vector")));
    }

    #[test]
    fn returns_none_when_no_prefix_has_the_binary() {
        assert_eq!(find_vector_binary("/nonexistent/prefix"), None);
    }
}
