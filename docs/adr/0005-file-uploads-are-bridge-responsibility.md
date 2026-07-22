# File uploads live in the Bridge, not the shipper

Log shippers move event streams, not arbitrary local files as named objects — which is why the Humble design needed custom Go plugins (`out_minio`, `out_files_metrics`). In the Jazzy design the Bridge hosts an uploader module (Rust, `object_store` crate: S3/MinIO/GCS/Azure) that uploads Files, verifies they landed, extracts metadata (content-type, size, video duration), and emits the resulting metadata Record through Vector to PostgreSQL like any other Record. Delete-after-confirmed-upload and the upload-status table are preserved as features; local deletion stays robot-side.

## Consequences

- PostgreSQL writes happen only via Vector's parameterized sink — this retires the string-formatted SQL in the Go plugin and gives file metadata the same buffering/retry guarantees as all Records.
- Consumers never guess completeness: multi-File groups (map = pgm+yaml, camera batches) get an explicit **group completion marker** written by the uploader only after every File in the group is verified — the "manifest as completion checkpoint" pattern, borrowed from dataset-platform upload design.
- Large Files upload **multipart and resume** after interruption rather than restarting, since robot networks are flaky and videos/maps are large; `object_store` provides this natively.
