# Common tools to run
## Postgres
```yaml
version: '3.1'
networks:
  windrose-ui:
    external: true

services:
  db:
    image: postgres
    restart: unless-stopped
    environment:
      POSTGRES_USER: fluentbit
      POSTGRES_PASSWORD: password
    networks:
      - windrose-ui
    ports:
      - 5432:5432

  adminer:
    image: adminer
    restart: unless-stopped
    networks:
      - windrose-ui
    ports:
      - 8080:8080
```


## MiniIo
```yaml
version: '3.7'
networks:
  windrose-ui:
    external: true

# Settings and configurations that are common for all containers
x-minio-common: &minio-common
  image: quay.io/minio/minio:RELEASE.2022-11-29T23-40-49Z
  command: server --console-address ":9001" http://minio{1...4}/data{1...2}
  expose:
    - "9000"
    - "9001"
  environment:
    MINIO_ROOT_USER: brisa-minio
    MINIO_ROOT_PASSWORD: OGfwK62ErFCwsJ5OvT0SN93afP6wdv2m
  healthcheck:
    test: ["CMD", "curl", "-f", "http://localhost:9000/minio/health/live"]
    interval: 30s
    timeout: 20s
    retries: 3

# starts 4 docker containers running minio server instances.
# using nginx reverse proxy, load balancing, you can access
# it through port 9000.
services:
  minio1:
    <<: *minio-common
    hostname: minio1
    volumes:
      - data1-1:/data1
      - data1-2:/data2
    networks:
      - windrose-ui

  minio2:
    <<: *minio-common
    hostname: minio2
    volumes:
      - data2-1:/data1
      - data2-2:/data2
    networks:
      - windrose-ui

  minio3:
    <<: *minio-common
    hostname: minio3
    volumes:
      - data3-1:/data1
      - data3-2:/data2
    networks:
      - windrose-ui

  minio4:
    <<: *minio-common
    hostname: minio4
    volumes:
      - data4-1:/data1
      - data4-2:/data2
    networks:
      - windrose-ui

  minio-nginx:
    image: nginx:1.19.2-alpine
    hostname: minio-nginx
    volumes:
      - ./minio-nginx.conf:/etc/nginx/nginx.conf:ro
    ports:
      - "9000:9000"
      - "9001:9001"
    depends_on:
      - minio1
      - minio2
      - minio3
      - minio4
    networks:
      - windrose-ui

## By default this config uses default local driver,
## For custom volumes replace with volume driver configuration.
volumes:
  data1-1:
  data1-2:
  data2-1:
  data2-2:
  data3-1:
  data3-2:
  data4-1:
  data4-2:
```
