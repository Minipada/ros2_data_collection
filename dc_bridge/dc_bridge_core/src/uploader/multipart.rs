//! Multipart, resumable uploads for large Files (ADR-0005): robot networks are flaky
//! and videos/maps are large, so an interrupted transfer must resume, not restart.
//!
//! Built on `object_store`'s low-level [`MultipartStore`] API rather than the
//! high-level `put_multipart` writer, because only the low-level API exposes the
//! persistent [`MultipartId`] and per-part [`PartId`]s that make resumption possible:
//! both are checkpointed to a JSON sidecar file after every part, so a new process (or
//! a retry after a dropped connection) re-reads the sidecar, skips every part that
//! already landed, and finishes the same multipart upload instead of starting over.
//! The sidecar is removed once the upload completes.

use object_store::multipart::PartId;
use object_store::path::Path as ObjectPath;
use object_store::{MultipartId, PutPayload};
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

use super::store::UploadStore;

/// Progress of one interrupted multipart upload, checkpointed after every part.
#[derive(Debug, Serialize, Deserialize)]
struct ResumeState {
    multipart_id: MultipartId,
    part_size: u64,
    file_size: u64,
    /// `content_id` of each completed part, by part index; `None` = not yet uploaded.
    parts: Vec<Option<String>>,
}

/// Where the resume sidecar for one (storage, object key) upload lives. Stable across
/// runs — FNV-1a rather than `DefaultHasher`, whose keys are randomized per process.
pub fn resume_state_path(state_dir: &Path, storage_name: &str, object_key: &str) -> PathBuf {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in storage_name.bytes().chain([0u8]).chain(object_key.bytes()) {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    state_dir.join(format!("multipart-{hash:016x}.json"))
}

fn generic_err(
    source: impl Into<Box<dyn std::error::Error + Send + Sync + 'static>>,
) -> object_store::Error {
    object_store::Error::Generic {
        store: "dc_bridge uploader resume state",
        source: source.into(),
    }
}

fn load_state(state_path: &Path, part_size: u64, file_size: u64) -> Option<ResumeState> {
    let content = fs::read_to_string(state_path).ok()?;
    let state: ResumeState = serde_json::from_str(&content).ok()?;
    // A sidecar from a different policy or a changed file can't be resumed safely.
    (state.part_size == part_size
        && state.file_size == file_size
        && state.parts.len() == part_count(file_size, part_size))
    .then_some(state)
}

fn save_state(state_path: &Path, state: &ResumeState) -> Result<(), object_store::Error> {
    let tmp = state_path.with_extension("json.tmp");
    let content = serde_json::to_string(state).map_err(generic_err)?;
    fs::write(&tmp, content).map_err(generic_err)?;
    fs::rename(&tmp, state_path).map_err(generic_err)?;
    Ok(())
}

fn part_count(file_size: u64, part_size: u64) -> usize {
    (file_size.div_ceil(part_size)).max(1) as usize
}

/// Uploads `local` to `location` on `store` in `part_size`-byte parts, checkpointing
/// progress to `state_path` so an interrupted upload resumes from the last completed
/// part. `file_size` must be `local`'s current size (the caller already stat'ed it).
pub async fn upload_resumable(
    store: &dyn UploadStore,
    location: &ObjectPath,
    local: &Path,
    file_size: u64,
    part_size: u64,
    state_path: &Path,
) -> Result<(), object_store::Error> {
    let n_parts = part_count(file_size, part_size);

    let mut state = match load_state(state_path, part_size, file_size) {
        Some(state) => state,
        None => {
            let multipart_id = store.create_multipart(location).await?;
            let state = ResumeState {
                multipart_id,
                part_size,
                file_size,
                parts: vec![None; n_parts],
            };
            save_state(state_path, &state)?;
            state
        }
    };

    let mut file = fs::File::open(local).map_err(generic_err)?;
    for idx in 0..n_parts {
        if state.parts[idx].is_some() {
            continue;
        }
        let offset = idx as u64 * part_size;
        let len = part_size.min(file_size - offset) as usize;
        let mut buf = vec![0u8; len];
        file.seek(SeekFrom::Start(offset)).map_err(generic_err)?;
        file.read_exact(&mut buf).map_err(generic_err)?;

        let part = store
            .put_part(location, &state.multipart_id, idx, PutPayload::from(buf))
            .await?;
        state.parts[idx] = Some(part.content_id);
        save_state(state_path, &state)?;
    }

    let parts: Vec<PartId> = state
        .parts
        .iter()
        .map(|content_id| PartId {
            content_id: content_id
                .clone()
                .expect("every part index was just filled in"),
        })
        .collect();
    store
        .complete_multipart(location, &state.multipart_id, parts)
        .await?;
    let _ = fs::remove_file(state_path);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resume_state_paths_are_stable_and_distinct() {
        let dir = Path::new("/state");
        let a = resume_state_path(dir, "minio", "maps/map.pgm");
        assert_eq!(a, resume_state_path(dir, "minio", "maps/map.pgm"));
        assert_ne!(a, resume_state_path(dir, "s3", "maps/map.pgm"));
        assert_ne!(a, resume_state_path(dir, "minio", "maps/other.pgm"));
    }

    #[test]
    fn part_count_covers_edge_sizes() {
        assert_eq!(part_count(0, 1024), 1);
        assert_eq!(part_count(1, 1024), 1);
        assert_eq!(part_count(1024, 1024), 1);
        assert_eq!(part_count(1025, 1024), 2);
    }
}
