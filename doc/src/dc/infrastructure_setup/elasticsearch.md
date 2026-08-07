# Elasticsearch

## Description

[Elasticsearch](https://www.elastic.co/elasticsearch) is a distributed, JSON-native search
and analytics engine. It stores Records as documents — no schema to declare up front, no
columns to pre-create — which makes it a natural fit for the [passthrough
Destination](../destinations.md#passthrough-custom_config_files): `dc_bridge` has no
`type: elasticsearch`, and none is needed, because [Vector](https://vector.dev) ships an
`elasticsearch` sink that speaks the bulk API directly.

[Kibana](https://www.elastic.co/kibana) is Elasticsearch's own query and dashboard UI, and
is what the [Elasticsearch tutorial](../demos/elasticsearch.md) uses to look at what
landed.

## Start with docker compose

Execute:

```bash
./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=elasticsearch \
  --install-type=docker
```

or, driving the compose file directly with Podman (the engine the rest of this repo's
container tooling uses):

```bash
podman compose -f tools/infrastructure/docker/docker-compose.elasticsearch.yaml up -d
```

This starts a single-node Elasticsearch plus a Kibana pointed at it. Elasticsearch has a
healthcheck and Kibana waits for it, so the stack is usable once `up -d` returns.

Check it is alive:

```bash
curl -s http://localhost:9200/_cluster/health
```

```json
{"cluster_name":"docker-cluster","status":"green","timed_out":false,"number_of_nodes":1,"number_of_data_nodes":1,...}
```

```admonish info
There is no native (non-container) install path for Elasticsearch in this script — it
needs a JVM and a `vm.max_map_count` of at least 262144 on the host, which is more
system-level setup than the script does for any other tool. Use the compose path above.
```

## Configuration and credentials

| Service       | URL                     | Credentials |
| ------------- | ----------------------- | ----------- |
| Elasticsearch | <http://localhost:9200> | none        |
| Kibana        | <http://localhost:5601> | none        |

The compose file sets `xpack.security.enabled=false`, so there are no credentials to
manage and the tutorial's Vector sink needs no `auth` block.

```admonish warning
Security disabled means anyone who can reach port 9200 can read and delete every index,
and Records travel in cleartext. This is a demo setting. For anything else, re-enable
`xpack.security`, and give the passthrough sink an `auth` block (or an API key) — see the
commented example at the bottom of `dc_demos/config/elasticsearch_sink.toml`.
```

Two other settings are demo-shaped and worth changing before this stack is anything but
local:

- `discovery.type=single-node` — one node, so no replicas are ever allocated. Indices
  report `yellow`, not `green`, because their replica shard has nowhere to go; that is
  expected here and not a symptom.
- `ES_JAVA_OPTS=-Xms512m -Xmx512m` — a small fixed heap so this fits next to a simulator
  on a laptop. Elasticsearch's own sizing guidance applies for a real deployment.

Kibana is only there to look at the data. On a constrained machine, comment its service
out — the tutorial's verification step queries the `_search` API with `curl` and needs
only Elasticsearch.
