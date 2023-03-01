# PostgreSQL
## Requirements

1. Docker installed
2. Docker compose installed

## Description
PostgreSQL is a powerful, open source object-relational database system that uses and extends the SQL language combined with many features that safely store and scale the most complicated data workloads. The origins of PostgreSQL date back to 1986 as part of the POSTGRES project at the University of California at Berkeley and has more than 35 years of active development on the core platform.

## Start with docker compose

```yaml
version: '3.1'

services:
  db:
    image: postgres
    restart: unless-stopped
    environment:
      POSTGRES_USER: fluentbit
      POSTGRES_PASSWORD: password
    ports:
      - 5432:5432

  adminer:
    image: adminer
    restart: unless-stopped
    ports:
      - 8080:8080
```

```admonish info
WIth this current deployment, your ROS 2 application will require to be able to reach the container. If it's running in a container, start it in the same docker network, or have all containers use network host (at your own risk)
```
