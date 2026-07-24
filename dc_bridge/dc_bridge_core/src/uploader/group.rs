//! Parsing the Files a Record references out of its JSON payload — the
//! `local_paths`/`remote_paths` structure Measurements embed (see
//! `dc_measurements/plugins/measurements/{camera,map}.cpp`), replacing the Humble-era
//! parallel `src_fields`/`upload_fields` config arrays.
//!
//! The shape, per File-bearing (sub-)object:
//!
//! ```json
//! {
//!   "name": "map",
//!   "local_paths":  { "yaml": "/tmp/map.yaml", "pgm": "/tmp/map.pgm" },
//!   "remote_paths": { "minio": { "yaml": "robot/map.yaml", "pgm": "robot/map.pgm" } }
//! }
//! ```
//!
//! `remote_paths` is keyed by Destination name: a File is uploaded to every configured
//! `receives: files` Destination whose name appears in its `remote_paths`. The whole
//! payload is walked recursively, so both a bare Measurement Record and a Group-merged
//! Record (Files nested under per-Measurement keys) parse the same way; all Files found
//! in one Record form one group (ADR-0005's group completion marker covers exactly this
//! set).

use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

/// One File referenced by a Record: its `local_paths` key, the on-robot path, and the
/// object key it should land under per Destination.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FileRef {
    /// The `local_paths` key naming this File within its Measurement ("pgm", "raw", …).
    pub key: String,
    pub local_path: String,
    /// Destination name → object key, from `remote_paths.<destination>.<key>`. Only
    /// configured Destinations are retained; never empty (a File no configured
    /// Destination references is not part of the upload group).
    pub remote_paths: BTreeMap<String, String>,
}

/// Every File one Record references — the unit ADR-0005's group completion marker is
/// emitted for.
#[derive(Debug, Clone, PartialEq)]
pub struct FileGroup {
    pub group_name: String,
    pub robot_name: Option<String>,
    pub robot_id: Option<Value>,
    pub files: Vec<FileRef>,
}

/// Extracts the [`FileGroup`] a Record payload references. `fallback_group` names the
/// group when the payload carries no `name` field (the Record's Tag is the natural
/// caller choice); `storages` is the set of configured `receives: files` Destination
/// names. Returns files in deterministic (local-path-sorted) order; the group is empty
/// if the Record references no Files any configured Destination receives.
pub fn parse_file_group(
    payload: &Value,
    fallback_group: &str,
    storages: &BTreeSet<String>,
) -> FileGroup {
    let mut files: BTreeMap<String, FileRef> = BTreeMap::new();
    collect_files(payload, storages, &mut files);

    let group_name = payload
        .get("name")
        .and_then(Value::as_str)
        .unwrap_or(fallback_group)
        .to_string();
    let robot_name = payload
        .get("robot_name")
        .and_then(Value::as_str)
        .map(str::to_string);
    let robot_id = payload.get("id").cloned().filter(|v| !v.is_null());

    FileGroup {
        group_name,
        robot_name,
        robot_id,
        files: files.into_values().collect(),
    }
}

fn collect_files(
    value: &Value,
    storages: &BTreeSet<String>,
    files: &mut BTreeMap<String, FileRef>,
) {
    let Value::Object(obj) = value else {
        return;
    };

    if let Some(Value::Object(local_paths)) = obj.get("local_paths") {
        let remote_paths = obj.get("remote_paths");
        for (key, local) in local_paths {
            let Some(local_path) = local.as_str().filter(|p| !p.is_empty()) else {
                continue;
            };
            let mut remotes = BTreeMap::new();
            for storage in storages {
                if let Some(remote) = remote_paths
                    .and_then(|r| r.get(storage))
                    .and_then(|s| s.get(key))
                    .and_then(Value::as_str)
                    .filter(|p| !p.is_empty())
                {
                    remotes.insert(storage.clone(), remote.to_string());
                }
            }
            if remotes.is_empty() {
                continue;
            }
            // The same local path can appear in several sub-objects; merge their
            // Destination sets rather than uploading twice.
            files
                .entry(local_path.to_string())
                .and_modify(|f| f.remote_paths.extend(remotes.clone()))
                .or_insert_with(|| FileRef {
                    key: key.clone(),
                    local_path: local_path.to_string(),
                    remote_paths: remotes,
                });
        }
    }

    for (key, child) in obj {
        // `base64` holds inline file content (potentially huge) and never Files.
        if key == "local_paths" || key == "remote_paths" || key == "base64" {
            continue;
        }
        collect_files(child, storages, files);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn storages(names: &[&str]) -> BTreeSet<String> {
        names.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn parses_a_flat_measurement_record_like_the_map_measurement_produces() {
        let payload = json!({
            "name": "map",
            "robot_name": "robot1",
            "id": "r1",
            "resolution": 0.05,
            "local_paths":  { "yaml": "/tmp/map.yaml", "pgm": "/tmp/map.pgm" },
            "remote_paths": { "minio": { "yaml": "maps/map.yaml", "pgm": "maps/map.pgm" } }
        });
        let group = parse_file_group(&payload, "dc.measurement.map", &storages(&["minio"]));
        assert_eq!(group.group_name, "map");
        assert_eq!(group.robot_name.as_deref(), Some("robot1"));
        assert_eq!(group.robot_id, Some(json!("r1")));
        assert_eq!(group.files.len(), 2);
        assert_eq!(group.files[0].local_path, "/tmp/map.pgm");
        assert_eq!(group.files[0].remote_paths["minio"], "maps/map.pgm");
    }

    #[test]
    fn walks_group_merged_records_with_files_nested_under_measurement_keys() {
        let payload = json!({
            "camera": {
                "local_paths":  { "raw": "/tmp/cam.jpg" },
                "remote_paths": { "minio": { "raw": "cam/cam.jpg" } }
            },
            "map": {
                "local_paths":  { "pgm": "/tmp/map.pgm" },
                "remote_paths": { "minio": { "pgm": "maps/map.pgm" } }
            }
        });
        let group = parse_file_group(&payload, "dc.group.robot", &storages(&["minio"]));
        assert_eq!(group.group_name, "dc.group.robot");
        assert_eq!(group.files.len(), 2);
    }

    #[test]
    fn keeps_only_configured_destinations_and_drops_unreferenced_files() {
        let payload = json!({
            "local_paths":  { "raw": "/tmp/cam.jpg", "png": "/tmp/preview.png" },
            "remote_paths": {
                "minio":     { "raw": "cam/cam.jpg" },
                "elsewhere": { "raw": "other/cam.jpg", "png": "other/preview.png" }
            }
        });
        let group = parse_file_group(&payload, "dc.measurement.camera", &storages(&["minio"]));
        // `png` has no remote path on any configured Destination → not in the group;
        // `elsewhere` is not a configured Destination → dropped from `raw`'s set.
        assert_eq!(group.files.len(), 1);
        assert_eq!(group.files[0].local_path, "/tmp/cam.jpg");
        assert_eq!(
            group.files[0].remote_paths.keys().collect::<Vec<_>>(),
            vec!["minio"]
        );
    }

    #[test]
    fn a_record_without_files_yields_an_empty_group() {
        let payload = json!({ "uptime_s": 42 });
        let group = parse_file_group(&payload, "dc.measurement.uptime", &storages(&["minio"]));
        assert!(group.files.is_empty());
        assert_eq!(group.group_name, "dc.measurement.uptime");
    }

    #[test]
    fn base64_subtrees_are_not_walked() {
        let payload = json!({
            "base64": {
                "local_paths":  { "yaml": "/tmp/decoy.yaml" },
                "remote_paths": { "minio": { "yaml": "decoy.yaml" } }
            }
        });
        let group = parse_file_group(&payload, "dc.measurement.map", &storages(&["minio"]));
        assert!(group.files.is_empty());
    }
}
