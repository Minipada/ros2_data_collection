from functools import lru_cache

import minio
from config import config

# RustFS is S3-compatible, so the generic `minio` client library talks to it directly.
rustfs_client = minio.Minio(
    config.RUSTFS_URL,
    access_key=config.RUSTFS_ACCESS_KEY,
    secret_key=config.RUSTFS_SECRET_KEY,
    secure=False,
)

# The Uploader's optional thumbnails (#256) land next to their original at
# `<remote_path>.thumb.jpg`. The suffix is a documented, deterministic contract
# (dc_bridge/include/dc_bridge/uploader/thumbnail.hpp), so a gallery can address a
# preview from the original's path alone — no second query, and no schema change here
# when the feature is off.
THUMBNAIL_SUFFIX = ".thumb.jpg"


# Whether a preview was actually uploaded for this File. Thumbnails are opt-in
# (`files.thumbnails.enabled`) and best-effort — an undecodable File, or a Bridge with no
# ffmpeg installed, simply produces none — so a gallery can never assume one is there.
# Cached because a gallery re-renders on every Streamlit interaction while the answer
# doesn't change for an already-uploaded File.
@lru_cache(maxsize=4096)
def _thumbnail_exists(object_path: str) -> bool:
    try:
        rustfs_client.stat_object(config.RUSTFS_BUCKET, object_path + THUMBNAIL_SUFFIX)
        return True
    except Exception:
        return False


# Presigned URL for a File's preview, falling back to the full-size original. Gallery
# tiles are the whole reason previews exist: a grid of full-size camera JPEGs is what
# makes those views slow. Where the full image is what the user actually wants (the
# expanded map view), call `rustfs_client.get_presigned_url` directly instead.
def gallery_url(object_path: str) -> str:
    if _thumbnail_exists(object_path):
        object_path += THUMBNAIL_SUFFIX
    return rustfs_client.get_presigned_url("GET", config.RUSTFS_BUCKET, object_path)
