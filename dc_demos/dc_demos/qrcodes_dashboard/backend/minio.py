import minio
from config import config

# Create a Minio client
minio_client = minio.Minio(
    config.MINIO_URL,
    access_key=config.MINIO_ACCESS_KEY,
    secret_key=config.MINIO_SECRET_KEY,
    secure=False,
)
