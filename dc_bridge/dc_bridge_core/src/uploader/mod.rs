//! The Uploader (ADR-0005): verified File uploads with metadata Records, replacing the
//! Humble-era Go plugins (`out_minio`, `out_files_metrics`).
//!
//! For each Record referencing Files (see [`group`]), per File: upload to every
//! `receives: files` Destination via the `object_store` abstraction ([`store`], with
//! multipart + resume for large Files — [`multipart`]), **verify** the object landed
//! (`head` + size comparison — an upload only counts once the store says the bytes are
//! there), extract metadata (content type, size, video duration), and emit a status
//! Record per File × Destination through an injected emit function (`main.rs` wires it
//! to the Forwarder under [`FILE_STATUS_TAG`], so status rows reach PostgreSQL through
//! the Shipper's parameterized sink — no SQL strings anywhere in DC). The upload-status
//! semantics (uploaded / on-filesystem / deleted per File and storage) are preserved
//! from Humble, but append-only: deletion emits a new status Record rather than
//! updating a row, since the pipeline is insert-only by design.
//!
//! Group completion (ADR-0005): once — and only once — every File the Record references
//! is verified on every Destination that receives it, a `group_complete` marker Record
//! is emitted, so consumers never guess whether a multi-File group (map = pgm+yaml,
//! camera batches) has fully arrived. A consumer querying mid-upload sees per-File
//! status rows but no marker.
//!
//! Local deletion (`files.delete_when_sent`) happens per File, strictly after that
//! File is verified on **all** its Destinations. Retries are idempotent: an
//! already-verified object short-circuits (no re-upload) and every status Record is
//! emitted at most once per Uploader instance (keyed dedup), so a retried Record
//! produces no duplicate rows. A Record redelivered after its Files were deleted
//! locally is recognized (object verified remotely, nothing on disk) and produces no
//! spurious "missing" rows.

pub mod content_type;
pub mod group;
pub mod multipart;
pub mod store;

use serde_json::{json, Value};
use std::collections::{BTreeSet, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use group::{FileGroup, FileRef};
use object_store::path::Path as ObjectPath;
use store::Storage;

/// The Tag the Uploader's status/metadata Records are emitted under. The Destination
/// named by `files.metadata_destination` gets this Tag appended to its inputs, so the
/// rendered Shipper config routes these Records to it like any others (public route
/// `dc.dc.files` under ADR-0003's addressing).
pub const FILE_STATUS_TAG: &str = "dc.files";

#[derive(Debug, Clone)]
pub struct UploaderConfig {
    /// Delete a local File once it is verified uploaded on all its Destinations.
    pub delete_when_sent: bool,
    /// Directory for multipart resume sidecars (survives Bridge restarts).
    pub state_dir: PathBuf,
    /// Files at least this large upload multipart (and therefore resumable).
    pub multipart_threshold_bytes: u64,
    /// Part size for multipart uploads. S3 requires ≥ 5 MiB for all but the last part.
    pub multipart_part_size_bytes: u64,
    /// Upload/verify attempts per (File, Destination) within one `process_record` call
    /// before the Record is reported as incomplete (the caller re-queues it).
    pub max_attempts: u32,
    /// Sleep between those attempts.
    pub retry_backoff: Duration,
}

impl UploaderConfig {
    pub fn new(state_dir: PathBuf, delete_when_sent: bool) -> Self {
        Self {
            delete_when_sent,
            state_dir,
            multipart_threshold_bytes: 16 * 1024 * 1024,
            multipart_part_size_bytes: 8 * 1024 * 1024,
            max_attempts: 3,
            retry_backoff: Duration::from_millis(500),
        }
    }
}

#[derive(Debug, Clone, thiserror::Error, PartialEq, Eq)]
pub enum UploadError {
    /// At least one (File, Destination) upload could not be verified within
    /// `max_attempts`; the Record should be re-processed later (idempotently).
    #[error("upload incomplete, will retry: {0}")]
    Incomplete(String),
    /// The injected emit function failed permanently (status Records must not be
    /// silently dropped, so this too re-queues the Record).
    #[error("failed to emit a status Record: {0}")]
    Emit(String),
}

/// What one `process_record` call did — the assertion surface for tests and logging.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ProcessSummary {
    /// Files referenced by the Record (with at least one configured Destination).
    pub files: usize,
    /// Files verified on every Destination that receives them (this call or earlier).
    pub verified: usize,
    /// Files not on disk and not verified remotely.
    pub missing: usize,
    /// Local Files deleted by this call.
    pub deleted: usize,
    /// Whether the group completion marker has been emitted (this call or earlier).
    pub group_complete: bool,
}

/// Probes a video's duration in seconds; `None` if it can't be determined.
pub type DurationProber = Box<dyn Fn(&Path) -> Option<f64> + Send + Sync>;

/// A [`DurationProber`] shelling out to ffprobe (the Humble behavior, minus the shell:
/// the path is passed as an argument vector, never interpolated into a shell string).
pub fn ffprobe_duration_prober(ffprobe_binary: PathBuf) -> DurationProber {
    Box::new(move |path: &Path| {
        let output = std::process::Command::new(&ffprobe_binary)
            .arg("-i")
            .arg(path)
            .args([
                "-show_entries",
                "format=duration",
                "-v",
                "quiet",
                "-of",
                "csv=p=0",
            ])
            .output()
            .ok()?;
        if !output.status.success() {
            return None;
        }
        String::from_utf8_lossy(&output.stdout).trim().parse().ok()
    })
}

struct FileMeta {
    size: u64,
    content_type: &'static str,
    duration: Option<f64>,
}

enum EnsureOutcome {
    /// Object verified on the store (uploaded now or already there).
    Verified,
    /// Nothing on disk, but the object is already verified remotely — the
    /// redelivered-after-delete case; counts as verified, emits nothing.
    VerifiedRemoteOnly,
    /// Nothing on disk and nothing (matching) in the store: can never succeed.
    MissingLocal,
}

pub struct Uploader {
    config: UploaderConfig,
    storages: Vec<Storage>,
    storage_names: BTreeSet<String>,
    duration_prober: DurationProber,
    runtime: tokio::runtime::Runtime,
    /// Dedup keys of every status Record already emitted, making retried Records
    /// idempotent (no duplicate rows).
    emitted: Mutex<HashSet<String>>,
}

impl Uploader {
    pub fn new(
        config: UploaderConfig,
        storages: Vec<Storage>,
        duration_prober: DurationProber,
    ) -> std::io::Result<Self> {
        std::fs::create_dir_all(&config.state_dir)?;
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()?;
        let storage_names = storages.iter().map(|s| s.name.clone()).collect();
        Ok(Self {
            config,
            storages,
            storage_names,
            duration_prober,
            runtime,
            emitted: Mutex::new(HashSet::new()),
        })
    }

    /// Processes one Record's Files end to end: upload + verify everywhere, status
    /// Records, group completion marker, delete-when-sent. `emit` receives each status
    /// Record's JSON payload (already deduplicated); returning `Err` re-queues the
    /// Record without risking duplicate rows.
    pub fn process_record(
        &self,
        payload: &Value,
        fallback_group: &str,
        emit: &mut dyn FnMut(Value) -> Result<(), String>,
    ) -> Result<ProcessSummary, UploadError> {
        let group = group::parse_file_group(payload, fallback_group, &self.storage_names);
        let mut summary = ProcessSummary {
            files: group.files.len(),
            ..ProcessSummary::default()
        };
        if group.files.is_empty() {
            return Ok(summary);
        }

        let mut failures: Vec<String> = Vec::new();
        let mut verified_files: Vec<&FileRef> = Vec::new();

        for file in &group.files {
            let meta = self.file_meta(&file.local_path);
            let mut verified_everywhere = true;

            for (storage_name, remote_path) in &file.remote_paths {
                let storage = self
                    .storages
                    .iter()
                    .find(|s| &s.name == storage_name)
                    .expect("parse_file_group only keeps configured storages");

                match self.ensure_uploaded(storage, file, meta.as_ref(), remote_path) {
                    Ok(EnsureOutcome::Verified) => {
                        let meta = meta.as_ref().expect("Verified implies a local file");
                        self.emit_once(
                            emit,
                            &format!("uploaded|{}|{}", file.local_path, storage.name),
                            uploaded_row(&group, file, storage, remote_path, meta),
                        )?;
                    }
                    Ok(EnsureOutcome::VerifiedRemoteOnly) => {}
                    Ok(EnsureOutcome::MissingLocal) => {
                        verified_everywhere = false;
                        summary.missing += 1;
                        self.emit_once(
                            emit,
                            &format!("missing|{}|{}", file.local_path, storage.name),
                            missing_row(&group, file, storage, remote_path),
                        )?;
                    }
                    Err(e) => {
                        verified_everywhere = false;
                        failures.push(format!("'{}' -> {}: {e}", file.local_path, storage.name));
                    }
                }
            }

            if verified_everywhere {
                summary.verified += 1;
                verified_files.push(file);
            }
        }

        // Group completion marker: only when every File in the group is verified on
        // every Destination that receives it (ADR-0005). Missing Files keep the group
        // incomplete forever — loud in the status table, never guessed complete.
        if summary.verified == summary.files {
            summary.group_complete = true;
            let files_key: Vec<&str> = group.files.iter().map(|f| f.local_path.as_str()).collect();
            self.emit_once(
                emit,
                &format!("group|{}|{}", group.group_name, files_key.join("|")),
                group_complete_row(&group),
            )?;
        }

        // Deletion strictly after verified upload on all storages — per File, so one
        // permanently failing File doesn't strand its group-mates on a full disk.
        if self.config.delete_when_sent {
            for file in &verified_files {
                let local = Path::new(&file.local_path);
                if !local.exists() {
                    continue;
                }
                if let Err(e) = std::fs::remove_file(local) {
                    failures.push(format!("failed to delete '{}': {e}", file.local_path));
                    continue;
                }
                summary.deleted += 1;
                for (storage_name, remote_path) in &file.remote_paths {
                    let storage = self
                        .storages
                        .iter()
                        .find(|s| &s.name == storage_name)
                        .expect("parse_file_group only keeps configured storages");
                    self.emit_once(
                        emit,
                        &format!("deleted|{}|{}", file.local_path, storage.name),
                        deleted_row(&group, file, storage, remote_path),
                    )?;
                }
            }
        }

        if !failures.is_empty() {
            return Err(UploadError::Incomplete(failures.join("; ")));
        }
        Ok(summary)
    }

    fn file_meta(&self, local_path: &str) -> Option<FileMeta> {
        let path = Path::new(local_path);
        let size = std::fs::metadata(path).ok()?.len();
        let mut head = [0u8; 512];
        let n = std::fs::File::open(path)
            .and_then(|mut f| std::io::Read::read(&mut f, &mut head))
            .unwrap_or(0);
        let content_type = content_type::sniff(&head[..n]);
        let duration = content_type::is_video(content_type)
            .then(|| (self.duration_prober)(path))
            .flatten();
        Some(FileMeta {
            size,
            content_type,
            duration,
        })
    }

    /// Upload-and-verify one (File, Destination) with bounded retries. Verification is
    /// a `head` on the object plus a size comparison; an already-matching object
    /// short-circuits the upload entirely (idempotent retries).
    fn ensure_uploaded(
        &self,
        storage: &Storage,
        file: &FileRef,
        meta: Option<&FileMeta>,
        remote_path: &str,
    ) -> Result<EnsureOutcome, String> {
        let key = storage.object_key(remote_path);
        let location = ObjectPath::from(key.as_str());
        let mut last_err = String::new();

        for attempt in 0..self.config.max_attempts {
            if attempt > 0 {
                std::thread::sleep(self.config.retry_backoff);
            }

            let head = self.runtime.block_on(storage.store.head(&location));
            match (&head, meta) {
                (Ok(obj), Some(meta)) if obj.size == meta.size => {
                    return Ok(EnsureOutcome::Verified)
                }
                // No local file: any existing object is taken as the verified upload
                // of a Record redelivered after delete-when-sent (size unknowable).
                (Ok(_), None) => return Ok(EnsureOutcome::VerifiedRemoteOnly),
                _ => {}
            }
            let Some(meta) = meta else {
                return Ok(EnsureOutcome::MissingLocal);
            };

            let uploaded = if meta.size >= self.config.multipart_threshold_bytes {
                let state_path =
                    multipart::resume_state_path(&self.config.state_dir, &storage.name, &key);
                self.runtime.block_on(multipart::upload_resumable(
                    storage.store.as_ref(),
                    &location,
                    Path::new(&file.local_path),
                    meta.size,
                    self.config.multipart_part_size_bytes,
                    &state_path,
                ))
            } else {
                match std::fs::read(&file.local_path) {
                    Ok(bytes) => self
                        .runtime
                        .block_on(
                            storage
                                .store
                                .put(&location, object_store::PutPayload::from(bytes)),
                        )
                        .map(|_| ()),
                    Err(e) => {
                        last_err = format!("failed to read local file: {e}");
                        continue;
                    }
                }
            };
            if let Err(e) = uploaded {
                last_err = e.to_string();
                continue;
            }

            // The upload call returning is not proof the object landed — verify.
            match self.runtime.block_on(storage.store.head(&location)) {
                Ok(obj) if obj.size == meta.size => return Ok(EnsureOutcome::Verified),
                Ok(obj) => {
                    last_err = format!(
                        "verification failed: object has {} bytes, local file has {}",
                        obj.size, meta.size
                    );
                }
                Err(e) => last_err = format!("verification failed: {e}"),
            }
        }
        Err(last_err)
    }

    fn emit_once(
        &self,
        emit: &mut dyn FnMut(Value) -> Result<(), String>,
        dedup_key: &str,
        row: Value,
    ) -> Result<(), UploadError> {
        if self.emitted.lock().unwrap().contains(dedup_key) {
            return Ok(());
        }
        emit(row).map_err(UploadError::Emit)?;
        self.emitted.lock().unwrap().insert(dedup_key.to_string());
        Ok(())
    }
}

fn unix_now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

/// The shared status-row fields, preserving the Humble `out_files_metrics` column set.
fn base_row(
    group: &FileGroup,
    file: &FileRef,
    storage: &Storage,
    remote_path: &str,
) -> serde_json::Map<String, Value> {
    let mut row = serde_json::Map::new();
    row.insert("kind".into(), json!("file_status"));
    row.insert("group_name".into(), json!(group.group_name));
    if let Some(robot_name) = &group.robot_name {
        row.insert("robot_name".into(), json!(robot_name));
    }
    if let Some(robot_id) = &group.robot_id {
        row.insert("robot_id".into(), robot_id.clone());
    }
    row.insert("local_path".into(), json!(file.local_path));
    row.insert(
        "remote_path".into(),
        json!(format!(
            "{}{}",
            storage.url_prefix,
            storage.object_key(remote_path)
        )),
    );
    row.insert("storage_type".into(), json!(storage.name));
    row.insert("updated_at".into(), json!(unix_now()));
    row
}

fn uploaded_row(
    group: &FileGroup,
    file: &FileRef,
    storage: &Storage,
    remote_path: &str,
    meta: &FileMeta,
) -> Value {
    let mut row = base_row(group, file, storage, remote_path);
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

fn missing_row(group: &FileGroup, file: &FileRef, storage: &Storage, remote_path: &str) -> Value {
    let mut row = base_row(group, file, storage, remote_path);
    row.insert("uploaded".into(), json!(false));
    row.insert("on_filesystem".into(), json!(false));
    row.insert("deleted".into(), json!(false));
    Value::Object(row)
}

fn deleted_row(group: &FileGroup, file: &FileRef, storage: &Storage, remote_path: &str) -> Value {
    let mut row = base_row(group, file, storage, remote_path);
    row.insert("uploaded".into(), json!(true));
    row.insert("on_filesystem".into(), json!(false));
    row.insert("deleted".into(), json!(true));
    Value::Object(row)
}

/// ADR-0005's group completion marker — the "manifest as completion checkpoint".
fn group_complete_row(group: &FileGroup) -> Value {
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
