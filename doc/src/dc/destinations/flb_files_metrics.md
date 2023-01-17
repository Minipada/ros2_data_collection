# Files metrics - Fluent Bit

## Description

Tracks files (yaml files, jpg pictures etc.) sent to their destinations (s3, minio etc.). We can then delete files locally when they have been sent to all their storage destinations.

It loads a Fluent Bit plugin we wrote ourselves to do this task, located in the fluent_bit_plugins package.

It creates a table on the database (currently PostgreSQL only) to store each file status.

```mermaid
erDiagram
    files_metrics {
        integer id
        timestamp timestamp
        text robot_name
        text robot_id
        text input_name
        double duration
        text local_path
        text remote_path
        bool uploaded
        bool on_filesystem
        bool deleted
        bool ignored
        text storage_type
        text content_type
        integer size
        timestamp created_at
        timestamp updated_at
    }
```

## Parameters

| Parameter                   | Description                            | Type      | Default                                 |
| --------------------------- | -------------------------------------- | --------- | --------------------------------------- |
| **plugin_path**             | Where data will be stored              | str       | <install_path>/lib/out_files_metrics.so |
| **file_storage**            | Where data will be stored              | list[str] | N/A (Mandatory)                         |
| **db_type**                 | Speed threshold used in the hysteresis | str       | "pgsql"                                 |
| **delete_when_sent**        | Counter to know if the robot is moving | bool      | true                                    |
| **minio.endpoint**          | Minio endpoint                         | str       | "127.0.0.1:9000"                        |
| **minio.access_key_id**     | Minio Access key ID                    | str       | N/A (Mandatory)                         |
| **minio.secret_access_key** | Minio Secret access key                | str       | N/A (Mandatory)                         |
| **minio.use_ssl**           | Use SSL for Minio                      | bool      | true                                    |
| **minio.bucket**            | Minio Bucket name                      | str       | "dc_bucket"                             |
| **minio.src_fields**        | Local paths for Minio                  | str       | N/A (Mandatory)                         |
| **minio.upload_fields**     | Remote paths for Minio                 | str       | N/A (Mandatory)                         |
| **s3.endpoint**             | Hysteresis counter                     | str       | N/A (Mandatory)                         |
| **s3.access_key_id**        | Hysteresis counter                     | str       | N/A (Mandatory)                         |
| **s3.secret_access_key**    | Hysteresis counter                     | str       | N/A (Mandatory)                         |
| **s3.bucket**               | Hysteresis counter                     | str       | N/A (Mandatory)                         |
| **s3.src_fields**           | Hysteresis counter                     | str       | N/A (Mandatory)                         |
| **s3.upload_fields**        | Hysteresis counter                     | str       | N/A (Mandatory)                         |
| **pgsql.host**              | Hysteresis counter                     | str       | "127.0.0.1"                             |
| **pgsql.port**              | Hysteresis counter                     | str       | "5432"                                  |
| **pgsql.user**              | Hysteresis counter                     | str       | <system_username>                       |
| **pgsql.password**          | Hysteresis counter                     | str       | ""                                      |
| **pgsql.database**          | Hysteresis counter                     | str       | <system_username>                       |
| **pgsql.table**             | Hysteresis counter                     | str       | "pg_table"                              |
| **pgsql.ssl**               | Hysteresis counter                     | bool      | true                                    |


## Example
```yaml
flb_files_metrics:
    plugin: "dc_destinations/Flbfilesmetrics"
    inputs: ["/dc/group/map"]
    file_storage: ["minio", "s3"]
    db_type: "pgsql"
    debug: true
    delete_when_sent: true
    minio:
        endpoint: 127.0.0.1:9000
        access_key_id: XEYqG4ZcPY5jiq5i
        secret_access_key: ji011KCtI82ZeQS6UwsQAg8x9VR4lSaQ
        use_ssl: false
        bucket: "mybucket"
        src_fields: ["map.local_paths.pgm", "map.local_paths.yaml"]
        upload_fields: ["map.minio_paths.pgm", "map.minio_paths.yaml"]
    s3:
        endpoint: 127.0.0.1:9000
        access_key_id: XEYqG4ZcPY5jiq5i
        secret_access_key: ji011KCtI82ZeQS6UwsQAg8x9VR4lSaQ
        bucket: "mybucket"
        src_fields: ["map.local_paths.yaml"]
        upload_fields: ["map.s3_paths.yaml"]
    pgsql:
        host: "127.0.0.1"
        port: "5432"
        user: fluentbit
        password: password
        database: "fluentbit"
        table: "files_metrics"
        timestamp_key: "date"
        time_format: "double"
        time_key: "date"
        ssl: false
```
