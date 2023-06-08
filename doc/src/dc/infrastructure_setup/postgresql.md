# PostgreSQL
## Description
PostgreSQL is a powerful, open source object-relational database system that uses and extends the SQL language combined with many features that safely store and scale the most complicated data workloads. The origins of PostgreSQL date back to 1986 as part of the POSTGRES project at the University of California at Berkeley and has more than 35 years of active development on the core platform.

## Start with docker compose
Execute:

```bash
./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=postgresql \
  --install-type=docker
```

## Start natively
Execute:

```bash
./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=postgresql \
  --install-type=native
```

## Credentials

| User | Password | Database | Port |
| ---- | -------- | -------- | ---- |
| dc   | password | dc       | 5432 |
