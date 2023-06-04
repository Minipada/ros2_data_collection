# Grafana

## Description
[Grafana](https://grafana.com/) is a multi-platform open source analytics and interactive visualization web application. It provides charts, graphs, and alerts for the web when connected to supported data sources.

## Start with docker compose
Execute:

```bash
$ ./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=grafana \
  --install-type=docker
```

## Start natively
```bash
$ ./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=grafana \
  --install-type=native
```

## Credentials

| User  | Password | Port |
| ----- | -------- | ---- |
| admin | admin    | 3000 |

## How to use
Natively, by accessing [http://localhost:80/adminer](http://localhost:80/adminer), and in docker, by accessing [http://localhost:8080](http://localhost:8080), you will be able to see this page:

![Grafana](../../images/adminer.png)
