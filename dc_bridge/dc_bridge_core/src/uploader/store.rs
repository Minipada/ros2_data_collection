//! The object-storage side of the Uploader (ADR-0005): a [`Storage`] wraps one
//! `receives: files` Destination's `object_store` client. The `object_store` crate is
//! the abstraction the ADR names — S3-compatible today (AWS, RustFS, MinIO, Ceph…),
//! GCS/Azure become new constructors here without touching the upload logic.

use crate::render::S3Params;
use object_store::aws::AmazonS3Builder;
use object_store::multipart::MultipartStore;
use object_store::ObjectStore;
use std::sync::Arc;

/// What the Uploader needs from a store: plain puts plus the low-level multipart API
/// ([`MultipartStore`]) whose persistent upload ids make interrupted transfers
/// resumable (ADR-0005's "multipart, resumable uploads" consequence). Implemented by
/// `object_store`'s cloud backends and its `InMemory` test double alike.
pub trait UploadStore: ObjectStore + MultipartStore {}
impl<T: ObjectStore + MultipartStore> UploadStore for T {}

/// One `receives: files` Destination, ready to upload to.
#[derive(Clone)]
pub struct Storage {
    /// The Destination's configured name — the key Files' `remote_paths` maps use, and
    /// the `storage_type` value in emitted status Records.
    pub name: String,
    /// Prefix rendering an object key into the `remote_path` status field
    /// (`s3://<bucket>/`), preserving the Humble status-row shape.
    pub url_prefix: String,
    /// Prepended to every object key (the Destination's `key_prefix`, if any).
    pub key_prefix: String,
    pub store: Arc<dyn UploadStore>,
}

impl std::fmt::Debug for Storage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Storage")
            .field("name", &self.name)
            .field("url_prefix", &self.url_prefix)
            .field("key_prefix", &self.key_prefix)
            .finish_non_exhaustive()
    }
}

impl Storage {
    /// Builds the S3 client for a `type: s3, receives: files` Destination. Credentials
    /// and endpoint semantics match the Records-side `aws_s3` Vector sink: explicit
    /// key pair when configured, ambient AWS environment/instance credentials
    /// otherwise; custom `endpoint` for self-hosted stores (http endpoints allowed —
    /// robot-LAN RustFS/MinIO deployments are rarely TLS-terminated).
    pub fn s3(name: &str, params: &S3Params) -> Result<Self, object_store::Error> {
        // `from_env` picks up ambient AWS credentials/region; explicit config overrides.
        let mut builder = AmazonS3Builder::from_env().with_bucket_name(&params.bucket);
        if let Some(region) = &params.region {
            builder = builder.with_region(region);
        }
        if let Some(endpoint) = &params.endpoint {
            builder = builder
                .with_endpoint(endpoint)
                .with_allow_http(endpoint.starts_with("http://"));
        }
        if let Some(auth) = &params.auth {
            builder = builder
                .with_access_key_id(&auth.access_key_id)
                .with_secret_access_key(&auth.secret_access_key);
        }
        if let Some(force_path_style) = params.force_path_style {
            builder = builder.with_virtual_hosted_style_request(!force_path_style);
        }
        Ok(Self {
            name: name.to_string(),
            url_prefix: format!("s3://{}/", params.bucket),
            key_prefix: params.key_prefix.clone().unwrap_or_default(),
            store: Arc::new(builder.build()?),
        })
    }

    /// A [`Storage`] over any [`UploadStore`] — how tests inject `InMemory` (or an
    /// instrumented wrapper around it).
    pub fn custom(name: &str, url_prefix: &str, store: Arc<dyn UploadStore>) -> Self {
        Self {
            name: name.to_string(),
            url_prefix: url_prefix.to_string(),
            key_prefix: String::new(),
            store,
        }
    }

    /// The full object key for a File's remote path on this Storage.
    pub fn object_key(&self, remote_path: &str) -> String {
        format!("{}{}", self.key_prefix, remote_path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::render::S3Auth;

    fn params() -> S3Params {
        S3Params {
            bucket: "dc-records".to_string(),
            region: Some("us-east-1".to_string()),
            endpoint: Some("http://127.0.0.1:9000".to_string()),
            key_prefix: Some("robot1/".to_string()),
            auth: Some(S3Auth {
                access_key_id: "rustfsadmin".to_string(),
                secret_access_key: "rustfsadmin".to_string(),
            }),
            force_path_style: Some(true),
            batch_timeout_secs: None,
        }
    }

    #[test]
    fn builds_an_s3_storage_with_the_humble_style_url_prefix() {
        let storage = Storage::s3("minio", &params()).unwrap();
        assert_eq!(storage.name, "minio");
        assert_eq!(storage.url_prefix, "s3://dc-records/");
        assert_eq!(storage.object_key("maps/map.pgm"), "robot1/maps/map.pgm");
    }

    #[test]
    fn key_prefix_defaults_to_empty() {
        let mut p = params();
        p.key_prefix = None;
        let storage = Storage::s3("minio", &p).unwrap();
        assert_eq!(storage.object_key("maps/map.pgm"), "maps/map.pgm");
    }
}
