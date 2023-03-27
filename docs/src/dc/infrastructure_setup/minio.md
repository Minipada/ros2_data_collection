# MinIO

## Requirements

1. Docker installed
2. Docker compose installed

## Description

MinIO is an object storage solution that provides an Amazon Web Services S3-compatible API and supports all core S3 features. MinIO is built to deploy anywhere - public or private cloud, baremetal infrastructure, orchestrated environments, and edge infrastructure.

## Start with Docker compose
Following the [official repository](https://github.com/minio/minio), you can find the [docker-compose.yaml](https://github.com/minio/minio/blob/master/docs/orchestration/docker-compose/docker-compose.yaml)

Follow instructions from the official documentation available [here](https://github.com/minio/minio/tree/master/docs/orchestration/docker-compose)

You can now access it by browsing to [http://localhost:9001](http://localhost:9001)

Default login and password currently are **minioadmin** / **minioadmin**. Login and create a new access key.

```admonish info
WIth this current deployment, your ROS 2 application will require to be able to reach the container. For this, either start it in the same docker network, or have all containers use network host (at your own risk)
```
