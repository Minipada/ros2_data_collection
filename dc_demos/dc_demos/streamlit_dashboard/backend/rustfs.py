import minio
from config import config

# RustFS is S3-compatible, so the generic `minio` client library talks to it directly.
rustfs_client = minio.Minio(
    config.RUSTFS_URL,
    access_key=config.RUSTFS_ACCESS_KEY,
    secret_key=config.RUSTFS_SECRET_KEY,
    secure=False,
)
