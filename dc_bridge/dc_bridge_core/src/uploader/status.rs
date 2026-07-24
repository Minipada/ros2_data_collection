//! Status-Record row shapes the Uploader emits (ADR-0005), preserving the Humble
//! `out_files_metrics` column set. [`StatusContext`] bundles the (group, File,
//! Destination) a row is about — the one tuple every row shares — so each row
//! constructor only states what makes that row distinct.

use serde_json::{json, Value};
use std::time::{SystemTime, UNIX_EPOCH};

use super::group::{FileGroup, FileRef};
use super::store::Storage;
use super::FileMeta;

fn unix_now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

/// The (group, File, Destination) a status row is about.
pub(super) struct StatusContext<'a> {
    group: &'a FileGroup,
    file: &'a FileRef,
    storage: &'a Storage,
    remote_path: &'a str,
}

impl<'a> StatusContext<'a> {
    pub(super) fn new(
        group: &'a FileGroup,
        file: &'a FileRef,
        storage: &'a Storage,
        remote_path: &'a str,
    ) -> Self {
        Self {
            group,
            file,
            storage,
            remote_path,
        }
    }

    /// The shared status-row fields, preserving the Humble `out_files_metrics` column set.
    fn base_row(&self) -> serde_json::Map<String, Value> {
        let mut row = serde_json::Map::new();
        row.insert("kind".into(), json!("file_status"));
        row.insert("group_name".into(), json!(self.group.group_name));
        if let Some(robot_name) = &self.group.robot_name {
            row.insert("robot_name".into(), json!(robot_name));
        }
        if let Some(robot_id) = &self.group.robot_id {
            row.insert("robot_id".into(), robot_id.clone());
        }
        row.insert("local_path".into(), json!(self.file.local_path));
        row.insert(
            "remote_path".into(),
            json!(format!(
                "{}{}",
                self.storage.url_prefix,
                self.storage.object_key(self.remote_path)
            )),
        );
        row.insert("storage_type".into(), json!(self.storage.name));
        row.insert("updated_at".into(), json!(unix_now()));
        row
    }

    pub(super) fn uploaded(&self, meta: &FileMeta) -> Value {
        let mut row = self.base_row();
        row.insert("uploaded".into(), json!(true));
        row.insert("on_filesystem".into(), json!(true));
        row.insert("deleted".into(), json!(false));
        row.insert("content_type".into(), json!(meta.content_type));
        row.insert("size".into(), json!(meta.size));
        if let Some(duration) = meta.duration {
            row.insert("duration".into(), json!(duration));
        }
        Value::Object(row)
    }

    pub(super) fn missing(&self) -> Value {
        let mut row = self.base_row();
        row.insert("uploaded".into(), json!(false));
        row.insert("on_filesystem".into(), json!(false));
        row.insert("deleted".into(), json!(false));
        Value::Object(row)
    }

    pub(super) fn deleted(&self) -> Value {
        let mut row = self.base_row();
        row.insert("uploaded".into(), json!(true));
        row.insert("on_filesystem".into(), json!(false));
        row.insert("deleted".into(), json!(true));
        Value::Object(row)
    }
}

/// ADR-0005's group completion marker — the "manifest as completion checkpoint".
pub(super) fn group_complete_row(group: &FileGroup) -> Value {
    let mut row = serde_json::Map::new();
    row.insert("kind".into(), json!("group_complete"));
    row.insert("group_name".into(), json!(group.group_name));
    if let Some(robot_name) = &group.robot_name {
        row.insert("robot_name".into(), json!(robot_name));
    }
    if let Some(robot_id) = &group.robot_id {
        row.insert("robot_id".into(), robot_id.clone());
    }
    row.insert("complete".into(), json!(true));
    row.insert("file_count".into(), json!(group.files.len()));
    row.insert(
        "files".into(),
        Value::Array(
            group
                .files
                .iter()
                .map(|f| json!({ "key": f.key, "local_path": f.local_path }))
                .collect(),
        ),
    );
    row.insert("updated_at".into(), json!(unix_now()));
    Value::Object(row)
}
