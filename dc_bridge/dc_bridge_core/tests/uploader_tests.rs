//! Uploader acceptance-criteria tests (issue #248, ADR-0005) against `object_store`'s
//! in-memory backend and real temp files: happy path, verify-fails-then-retry, delete
//! only after verified upload on all configured storages, metadata Record shape,
//! file-missing-on-disk, group completion markers, idempotent retries (no duplicate
//! status rows), video duration extraction, and multipart resume after interruption.

use async_trait::async_trait;
use dc_bridge_core::uploader::store::UploadStore;
use dc_bridge_core::{ProcessSummary, Storage, UploadError, Uploader, UploaderConfig};
use futures::stream::BoxStream;
use object_store::memory::InMemory;
use object_store::multipart::{MultipartStore, PartId};
use object_store::path::Path as ObjectPath;
use object_store::{
    GetOptions, GetResult, ListResult, MultipartId, MultipartUpload, ObjectMeta, ObjectStore,
    PutMultipartOptions, PutOptions, PutPayload, PutResult, Result as StoreResult,
};
use serde_json::{json, Value};
use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::atomic::{AtomicI64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tempfile::TempDir;

/// An `InMemory` store instrumented with call counters and failure injection — the
/// "in-memory `object_store` backend" the acceptance criteria name, plus the knobs the
/// retry/interruption tests need.
#[derive(Debug, Default)]
struct Instrumented {
    inner: InMemory,
    puts: AtomicUsize,
    /// Successful `put_part` indices, in call order.
    part_puts: Mutex<Vec<usize>>,
    multipart_creates: AtomicUsize,
    /// Fail the next N `head` calls.
    fail_next_heads: AtomicI64,
    /// Fail every `put` for these object keys.
    fail_put_keys: Mutex<HashSet<String>>,
    /// Remaining `put_part` calls allowed before they start failing (i64::MAX = all).
    allow_parts: AtomicI64,
}

impl Instrumented {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            allow_parts: AtomicI64::new(i64::MAX),
            ..Self::default()
        })
    }

    fn injected(what: &str) -> object_store::Error {
        object_store::Error::Generic {
            store: "instrumented-test-store",
            source: format!("injected {what} failure").into(),
        }
    }

    fn object_bytes(&self, key: &str) -> Vec<u8> {
        futures::executor::block_on(async {
            self.inner
                .get(&ObjectPath::from(key))
                .await
                .unwrap()
                .bytes()
                .await
                .unwrap()
                .to_vec()
        })
    }

    fn has_object(&self, key: &str) -> bool {
        futures::executor::block_on(self.inner.head(&ObjectPath::from(key))).is_ok()
    }
}

impl std::fmt::Display for Instrumented {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Instrumented({})", self.inner)
    }
}

#[async_trait]
impl ObjectStore for Instrumented {
    async fn put_opts(
        &self,
        location: &ObjectPath,
        payload: PutPayload,
        opts: PutOptions,
    ) -> StoreResult<PutResult> {
        if self
            .fail_put_keys
            .lock()
            .unwrap()
            .contains(location.as_ref())
        {
            return Err(Self::injected("put"));
        }
        self.puts.fetch_add(1, Ordering::SeqCst);
        self.inner.put_opts(location, payload, opts).await
    }

    async fn put_multipart_opts(
        &self,
        location: &ObjectPath,
        opts: PutMultipartOptions,
    ) -> StoreResult<Box<dyn MultipartUpload>> {
        self.inner.put_multipart_opts(location, opts).await
    }

    async fn get_opts(&self, location: &ObjectPath, options: GetOptions) -> StoreResult<GetResult> {
        self.inner.get_opts(location, options).await
    }

    async fn head(&self, location: &ObjectPath) -> StoreResult<ObjectMeta> {
        if self.fail_next_heads.fetch_sub(1, Ordering::SeqCst) > 0 {
            return Err(Self::injected("head"));
        }
        self.inner.head(location).await
    }

    async fn delete(&self, location: &ObjectPath) -> StoreResult<()> {
        self.inner.delete(location).await
    }

    fn list(&self, prefix: Option<&ObjectPath>) -> BoxStream<'static, StoreResult<ObjectMeta>> {
        self.inner.list(prefix)
    }

    async fn list_with_delimiter(&self, prefix: Option<&ObjectPath>) -> StoreResult<ListResult> {
        self.inner.list_with_delimiter(prefix).await
    }

    async fn copy(&self, from: &ObjectPath, to: &ObjectPath) -> StoreResult<()> {
        self.inner.copy(from, to).await
    }

    async fn copy_if_not_exists(&self, from: &ObjectPath, to: &ObjectPath) -> StoreResult<()> {
        self.inner.copy_if_not_exists(from, to).await
    }
}

#[async_trait]
impl MultipartStore for Instrumented {
    async fn create_multipart(&self, path: &ObjectPath) -> StoreResult<MultipartId> {
        self.multipart_creates.fetch_add(1, Ordering::SeqCst);
        self.inner.create_multipart(path).await
    }

    async fn put_part(
        &self,
        path: &ObjectPath,
        id: &MultipartId,
        part_idx: usize,
        data: PutPayload,
    ) -> StoreResult<PartId> {
        if self.allow_parts.fetch_sub(1, Ordering::SeqCst) <= 0 {
            return Err(Self::injected("put_part"));
        }
        let part = self.inner.put_part(path, id, part_idx, data).await?;
        self.part_puts.lock().unwrap().push(part_idx);
        Ok(part)
    }

    async fn complete_multipart(
        &self,
        path: &ObjectPath,
        id: &MultipartId,
        parts: Vec<PartId>,
    ) -> StoreResult<PutResult> {
        self.inner.complete_multipart(path, id, parts).await
    }

    async fn abort_multipart(&self, path: &ObjectPath, id: &MultipartId) -> StoreResult<()> {
        self.inner.abort_multipart(path, id).await
    }
}

struct Fixture {
    tmp: TempDir,
    stores: Vec<Arc<Instrumented>>,
    storages: Vec<Storage>,
}

impl Fixture {
    fn new(storage_names: &[&str]) -> Self {
        let tmp = TempDir::new().unwrap();
        let stores: Vec<Arc<Instrumented>> =
            storage_names.iter().map(|_| Instrumented::new()).collect();
        let storages = storage_names
            .iter()
            .zip(&stores)
            .map(|(name, store)| {
                Storage::custom(
                    name,
                    &format!("s3://{name}-bucket/"),
                    store.clone() as Arc<dyn UploadStore>,
                )
            })
            .collect();
        Self {
            tmp,
            stores,
            storages,
        }
    }

    fn config(&self, delete_when_sent: bool) -> UploaderConfig {
        let mut config = UploaderConfig::new(self.tmp.path().join("state"), delete_when_sent);
        config.retry_backoff = Duration::from_millis(0);
        config
    }

    fn uploader(&self, delete_when_sent: bool) -> Uploader {
        self.uploader_with(self.config(delete_when_sent), |_| None)
    }

    fn uploader_with(
        &self,
        config: UploaderConfig,
        prober: impl Fn(&std::path::Path) -> Option<f64> + Send + Sync + 'static,
    ) -> Uploader {
        Uploader::new(config, self.storages.clone(), Box::new(prober)).unwrap()
    }

    fn write_file(&self, name: &str, content: &[u8]) -> String {
        let path = self.tmp.path().join(name);
        std::fs::write(&path, content).unwrap();
        path.to_string_lossy().into_owned()
    }
}

/// A camera-style Record payload referencing one File per configured storage.
fn camera_payload(local_path: &str, storages: &[&str]) -> Value {
    let mut remote = serde_json::Map::new();
    for storage in storages {
        remote.insert(storage.to_string(), json!({ "raw": "cam/2026/img.jpg" }));
    }
    json!({
        "name": "camera",
        "robot_name": "robot1",
        "id": "r1",
        "local_paths": { "raw": local_path },
        "remote_paths": remote,
    })
}

fn collect_rows(rows: &Arc<Mutex<Vec<Value>>>) -> impl FnMut(Value) -> Result<(), String> {
    let rows = rows.clone();
    move |row| {
        rows.lock().unwrap().push(row);
        Ok(())
    }
}

fn rows_of_kind<'a>(rows: &'a [Value], kind: &str) -> Vec<&'a Value> {
    rows.iter().filter(|r| r["kind"] == json!(kind)).collect()
}

const JPEG_BYTES: &[u8] = b"\xff\xd8\xff\xe0fake-jpeg-body";

#[test]
fn happy_path_uploads_to_every_storage_verifies_and_emits_status_rows() {
    let fx = Fixture::new(&["minio", "s3_archive"]);
    let local = fx.write_file("img.jpg", JPEG_BYTES);
    let uploader = fx.uploader(false);
    let rows = Arc::new(Mutex::new(Vec::new()));

    let summary = uploader
        .process_record(
            &camera_payload(&local, &["minio", "s3_archive"]),
            "dc.measurement.camera",
            &mut collect_rows(&rows),
        )
        .unwrap();

    assert_eq!(
        summary,
        ProcessSummary {
            files: 1,
            verified: 1,
            missing: 0,
            deleted: 0,
            group_complete: true,
        }
    );
    for store in &fx.stores {
        assert_eq!(store.object_bytes("cam/2026/img.jpg"), JPEG_BYTES);
    }
    let rows = rows.lock().unwrap();
    let uploaded = rows_of_kind(&rows, "file_status");
    assert_eq!(uploaded.len(), 2, "one status row per File x storage");
    assert_eq!(rows_of_kind(&rows, "group_complete").len(), 1);
    // The local File must still be on disk: delete_when_sent is off.
    assert!(std::path::Path::new(&local).exists());
}

/// The metadata Record shape: the Humble `out_files_metrics` column set, preserved.
#[test]
fn metadata_record_has_the_humble_status_row_shape() {
    let fx = Fixture::new(&["minio"]);
    let local = fx.write_file("img.jpg", JPEG_BYTES);
    let uploader = fx.uploader(false);
    let rows = Arc::new(Mutex::new(Vec::new()));

    uploader
        .process_record(
            &camera_payload(&local, &["minio"]),
            "dc.measurement.camera",
            &mut collect_rows(&rows),
        )
        .unwrap();

    let rows = rows.lock().unwrap();
    let row = rows_of_kind(&rows, "file_status")[0];
    assert_eq!(row["group_name"], json!("camera"));
    assert_eq!(row["robot_name"], json!("robot1"));
    assert_eq!(row["robot_id"], json!("r1"));
    assert_eq!(row["local_path"], json!(local));
    assert_eq!(
        row["remote_path"],
        json!("s3://minio-bucket/cam/2026/img.jpg")
    );
    assert_eq!(row["storage_type"], json!("minio"));
    assert_eq!(row["uploaded"], json!(true));
    assert_eq!(row["on_filesystem"], json!(true));
    assert_eq!(row["deleted"], json!(false));
    assert_eq!(row["content_type"], json!("image/jpeg"));
    assert_eq!(row["size"], json!(JPEG_BYTES.len()));
    assert!(row["updated_at"].as_f64().unwrap() > 0.0);
    assert!(
        row.get("duration").is_none(),
        "no duration for non-video content"
    );

    let marker = rows_of_kind(&rows, "group_complete")[0];
    assert_eq!(marker["complete"], json!(true));
    assert_eq!(marker["file_count"], json!(1));
    assert_eq!(marker["files"][0]["local_path"], json!(local));
}

#[test]
fn verify_failure_is_retried_without_a_second_upload() {
    let fx = Fixture::new(&["minio"]);
    let local = fx.write_file("img.jpg", JPEG_BYTES);
    let uploader = fx.uploader(false);
    let rows = Arc::new(Mutex::new(Vec::new()));

    // Attempt 1: pre-check head fails (injected), put succeeds, verification head
    // fails (injected). Attempt 2: pre-check head sees the object → verified, no
    // second put.
    fx.stores[0].fail_next_heads.store(2, Ordering::SeqCst);

    let summary = uploader
        .process_record(
            &camera_payload(&local, &["minio"]),
            "dc.measurement.camera",
            &mut collect_rows(&rows),
        )
        .unwrap();

    assert_eq!(summary.verified, 1);
    assert_eq!(fx.stores[0].puts.load(Ordering::SeqCst), 1);
    let rows = rows.lock().unwrap();
    assert_eq!(rows_of_kind(&rows, "file_status").len(), 1);
}

#[test]
fn local_file_is_deleted_only_after_verified_upload_on_all_storages() {
    let fx = Fixture::new(&["minio", "s3_archive"]);
    let local = fx.write_file("img.jpg", JPEG_BYTES);
    let uploader = fx.uploader(true);
    let rows = Arc::new(Mutex::new(Vec::new()));
    let payload = camera_payload(&local, &["minio", "s3_archive"]);

    // The second storage rejects every put: the Record must come back as incomplete,
    // and the local File must survive.
    fx.stores[1]
        .fail_put_keys
        .lock()
        .unwrap()
        .insert("cam/2026/img.jpg".to_string());

    let err = uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap_err();
    assert!(matches!(err, UploadError::Incomplete(_)));
    assert!(
        std::path::Path::new(&local).exists(),
        "File must not be deleted before verified upload everywhere"
    );
    {
        let rows = rows.lock().unwrap();
        assert!(rows_of_kind(&rows, "group_complete").is_empty());
        assert!(rows.iter().all(|r| r["deleted"] == json!(false)));
    }

    // Storage heals; the retried Record completes: upload verified everywhere, group
    // marker emitted, File deleted, deletion rows appended — and the first storage's
    // already-verified upload is not re-uploaded and not re-reported.
    fx.stores[1].fail_put_keys.lock().unwrap().clear();
    let summary = uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap();
    assert_eq!(summary.verified, 1);
    assert_eq!(summary.deleted, 1);
    assert!(summary.group_complete);
    assert!(!std::path::Path::new(&local).exists());
    assert_eq!(fx.stores[0].puts.load(Ordering::SeqCst), 1);
    {
        let rows = rows.lock().unwrap();
        let uploaded: Vec<_> = rows_of_kind(&rows, "file_status")
            .into_iter()
            .filter(|r| r["uploaded"] == json!(true) && r["deleted"] == json!(false))
            .collect();
        assert_eq!(uploaded.len(), 2, "exactly one uploaded row per storage");
        let deleted: Vec<_> = rows_of_kind(&rows, "file_status")
            .into_iter()
            .filter(|r| r["deleted"] == json!(true))
            .collect();
        assert_eq!(deleted.len(), 2, "one deletion row per storage");
        for row in deleted {
            assert_eq!(row["on_filesystem"], json!(false));
            assert_eq!(row["uploaded"], json!(true));
        }
    }

    // A Record redelivered after delete-when-sent (e.g. the Bridge restarted) is
    // recognized by the verified remote objects: no spurious rows, no failure.
    let before = rows.lock().unwrap().len();
    let summary = uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap();
    assert_eq!(summary.verified, 1);
    assert!(summary.group_complete);
    assert_eq!(summary.missing, 0);
    assert_eq!(rows.lock().unwrap().len(), before, "no duplicate rows");
}

#[test]
fn retried_records_do_not_duplicate_status_rows() {
    let fx = Fixture::new(&["minio"]);
    let local = fx.write_file("img.jpg", JPEG_BYTES);
    let uploader = fx.uploader(false);
    let rows = Arc::new(Mutex::new(Vec::new()));
    let payload = camera_payload(&local, &["minio"]);

    uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap();
    let after_first = rows.lock().unwrap().len();
    let summary = uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap();

    assert!(summary.group_complete);
    assert_eq!(fx.stores[0].puts.load(Ordering::SeqCst), 1, "no re-upload");
    assert_eq!(rows.lock().unwrap().len(), after_first, "no duplicate rows");
}

#[test]
fn a_file_missing_on_disk_is_reported_and_never_completes_its_group() {
    let fx = Fixture::new(&["minio"]);
    let local = fx.tmp.path().join("never-written.jpg");
    let uploader = fx.uploader(false);
    let rows = Arc::new(Mutex::new(Vec::new()));

    let summary = uploader
        .process_record(
            &camera_payload(&local.to_string_lossy(), &["minio"]),
            "dc.measurement.camera",
            &mut collect_rows(&rows),
        )
        .unwrap();

    assert_eq!(summary.missing, 1);
    assert_eq!(summary.verified, 0);
    assert!(!summary.group_complete);
    let rows = rows.lock().unwrap();
    let row = rows_of_kind(&rows, "file_status")[0];
    assert_eq!(row["uploaded"], json!(false));
    assert_eq!(row["on_filesystem"], json!(false));
    assert!(rows_of_kind(&rows, "group_complete").is_empty());
    assert!(!fx.stores[0].has_object("cam/2026/img.jpg"));
}

/// A map-style multi-File group: the completion marker appears only once every File is
/// verified, and a consumer querying mid-upload can tell partial from complete.
#[test]
fn group_completion_marker_is_emitted_only_after_every_file_is_verified() {
    let fx = Fixture::new(&["minio"]);
    let yaml = fx.write_file("map.yaml", b"image: map.pgm\nresolution: 0.05\n");
    let pgm = fx.write_file("map.pgm", b"P5\n2 2\n255\n\x00\x01\x02\x03");
    let payload = json!({
        "name": "map",
        "local_paths":  { "yaml": yaml, "pgm": pgm },
        "remote_paths": { "minio": { "yaml": "maps/map.yaml", "pgm": "maps/map.pgm" } },
    });
    let uploader = fx.uploader(false);
    let rows = Arc::new(Mutex::new(Vec::new()));

    // First pass: the pgm upload fails → partial group. Mid-upload state: one File's
    // status row present, NO group_complete marker.
    fx.stores[0]
        .fail_put_keys
        .lock()
        .unwrap()
        .insert("maps/map.pgm".to_string());
    let err = uploader
        .process_record(&payload, "dc.measurement.map", &mut collect_rows(&rows))
        .unwrap_err();
    assert!(matches!(err, UploadError::Incomplete(_)));
    {
        let rows = rows.lock().unwrap();
        assert_eq!(rows_of_kind(&rows, "file_status").len(), 1);
        assert!(
            rows_of_kind(&rows, "group_complete").is_empty(),
            "a partial group must not carry a completion marker"
        );
    }

    // Second pass: the pgm lands → the marker is emitted, exactly once, listing both.
    fx.stores[0].fail_put_keys.lock().unwrap().clear();
    let summary = uploader
        .process_record(&payload, "dc.measurement.map", &mut collect_rows(&rows))
        .unwrap();
    assert!(summary.group_complete);
    let rows = rows.lock().unwrap();
    assert_eq!(rows_of_kind(&rows, "file_status").len(), 2);
    let markers = rows_of_kind(&rows, "group_complete");
    assert_eq!(markers.len(), 1);
    assert_eq!(markers[0]["file_count"], json!(2));
}

#[test]
fn video_duration_is_extracted_for_video_content_types_only() {
    let fx = Fixture::new(&["minio"]);
    let mut mp4 = b"\x00\x00\x00\x20ftypisom".to_vec();
    mp4.extend_from_slice(&[0u8; 64]);
    let video = fx.write_file("clip.mp4", &mp4);
    let image = fx.write_file("img.jpg", JPEG_BYTES);

    let probed = Arc::new(Mutex::new(Vec::<PathBuf>::new()));
    let probed_clone = probed.clone();
    let uploader = fx.uploader_with(fx.config(false), move |path| {
        probed_clone.lock().unwrap().push(path.to_path_buf());
        Some(12.34)
    });
    let payload = json!({
        "name": "inspection",
        "local_paths":  { "video": video, "still": image },
        "remote_paths": { "minio": { "video": "insp/clip.mp4", "still": "insp/img.jpg" } },
    });
    let rows = Arc::new(Mutex::new(Vec::new()));
    uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap();

    let rows = rows.lock().unwrap();
    let by_path = |suffix: &str| {
        rows_of_kind(&rows, "file_status")
            .into_iter()
            .find(|r| r["local_path"].as_str().unwrap().ends_with(suffix))
            .unwrap()
            .clone()
    };
    assert_eq!(by_path("clip.mp4")["duration"], json!(12.34));
    assert_eq!(by_path("clip.mp4")["content_type"], json!("video/mp4"));
    assert!(by_path("img.jpg").get("duration").is_none());
    let probed = probed.lock().unwrap();
    assert_eq!(probed.len(), 1, "the prober runs only for video files");
    assert!(probed[0].to_string_lossy().ends_with("clip.mp4"));
}

/// The real ffprobe code path, exercised against a stub binary: the duration must come
/// from running the configured executable, not from parsing magic.
#[test]
fn ffprobe_prober_parses_the_binary_output() {
    let tmp = TempDir::new().unwrap();
    let stub = tmp.path().join("ffprobe");
    std::fs::write(&stub, "#!/bin/sh\necho 3.25\n").unwrap();
    let mut perms = std::fs::metadata(&stub).unwrap().permissions();
    std::os::unix::fs::PermissionsExt::set_mode(&mut perms, 0o755);
    std::fs::set_permissions(&stub, perms).unwrap();

    let prober = dc_bridge_core::ffprobe_duration_prober(stub);
    assert_eq!(prober(tmp.path()), Some(3.25));

    let missing = dc_bridge_core::ffprobe_duration_prober(tmp.path().join("nonexistent"));
    assert_eq!(missing(tmp.path()), None);
}

/// ADR-0005's multipart resume: an interrupted large-File transfer picks up from the
/// last checkpointed part — never re-uploading completed parts, never restarting.
#[test]
fn interrupted_multipart_upload_resumes_instead_of_restarting() {
    let fx = Fixture::new(&["minio"]);
    // 4 parts of 1 KiB (config below): distinct content per part to catch misordering.
    let content: Vec<u8> = (0..4096u32).map(|i| (i % 251) as u8).collect();
    let local = fx.write_file("big.bin", &content);
    let payload = json!({
        "name": "video_batch",
        "local_paths":  { "bin": local },
        "remote_paths": { "minio": { "bin": "big/big.bin" } },
    });

    let mut config = fx.config(false);
    config.multipart_threshold_bytes = 1024;
    config.multipart_part_size_bytes = 1024;
    config.max_attempts = 1;

    // First run: the transfer dies after 2 parts.
    fx.stores[0].allow_parts.store(2, Ordering::SeqCst);
    let uploader = fx.uploader_with(config.clone(), |_| None);
    let rows = Arc::new(Mutex::new(Vec::new()));
    let err = uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap_err();
    assert!(matches!(err, UploadError::Incomplete(_)));
    assert_eq!(*fx.stores[0].part_puts.lock().unwrap(), vec![0, 1]);
    assert!(!fx.stores[0].has_object("big/big.bin"));

    // Second run — a fresh Uploader over the same state dir, as after a Bridge
    // restart: only the two missing parts are uploaded, the same multipart upload is
    // completed, and the object verifies byte-for-byte.
    fx.stores[0].allow_parts.store(i64::MAX, Ordering::SeqCst);
    let uploader = fx.uploader_with(config, |_| None);
    let summary = uploader
        .process_record(&payload, "dc.measurement.camera", &mut collect_rows(&rows))
        .unwrap();
    assert_eq!(summary.verified, 1);
    assert!(summary.group_complete);
    assert_eq!(
        *fx.stores[0].part_puts.lock().unwrap(),
        vec![0, 1, 2, 3],
        "resume must upload only the parts that had not landed"
    );
    assert_eq!(
        fx.stores[0].multipart_creates.load(Ordering::SeqCst),
        1,
        "resume must reuse the interrupted multipart upload, not create a new one"
    );
    assert_eq!(fx.stores[0].object_bytes("big/big.bin"), content);
    // The resume sidecar is cleaned up after completion.
    let state_dir = fx.tmp.path().join("state");
    let leftovers: Vec<_> = std::fs::read_dir(&state_dir)
        .map(|entries| entries.filter_map(|e| e.ok()).collect())
        .unwrap_or_default();
    assert!(
        leftovers.is_empty(),
        "resume state must be removed once the upload completes: {leftovers:?}"
    );
}

/// Records that reference no Files pass through as no-ops (the Uploader shares topics
/// with Records-destinations; not every Record carries Files).
#[test]
fn records_without_files_are_noops() {
    let fx = Fixture::new(&["minio"]);
    let uploader = fx.uploader(true);
    let rows = Arc::new(Mutex::new(Vec::new()));
    let summary = uploader
        .process_record(
            &json!({ "uptime_s": 42 }),
            "dc.measurement.uptime",
            &mut collect_rows(&rows),
        )
        .unwrap();
    assert_eq!(summary, ProcessSummary::default());
    assert!(rows.lock().unwrap().is_empty());
}
