# RustFS

## Requirements

1. Docker installed
2. Docker compose installed

## Description

RustFS is an S3-compatible object storage server, written in Rust and licensed Apache 2.0. It is the `s3` blessed Destination's recommended backing store — MinIO's community edition was archived upstream in 2026 and no longer receives maintenance, so DC 2.0's own examples and E2E harness (`tools/e2e/`) both target RustFS instead; existing MinIO or Ceph RGW deployments work identically since `dc_bridge`'s `s3` Destination just talks plain S3 API. See [Destinations](../destinations.md) for the full Destination contract.

## Start with docker compose
Execute:

```bash
./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=rustfs \
  --install-type=docker
```

This starts a single RustFS container plus a one-shot container that bootstraps the `dc-files` bucket the demo params files upload into (RustFS does not auto-create buckets on startup).

```admonish info
There is no native (non-container) install path for RustFS in this script yet — unlike MinIO, it has no apt/systemd packaging convention to mirror. Use the docker compose path above.
```

## How to use

RustFS listens on port 9000 and speaks the S3 API directly — point any S3-compatible client (`mc`, `aws s3`, or `dc_bridge`'s own `s3` Destination) at `http://localhost:9000`.

## Credentials

| User        | Password    | Port |
| ----------- | ----------- | ---- |
| rustfsadmin | rustfsadmin | 9000 |

These are RustFS's built-in default credentials (no environment variables are set in `docker-compose.rustfs.yaml`) — set your own for anything beyond a local demo.
