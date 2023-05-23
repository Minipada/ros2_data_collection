# PostgreSQL - Fluent Bit

## Description

PostgreSQL is a very popular and versatile open source database management system that supports the SQL language and that is capable of storing both structured and unstructured data, such as JSON objects. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/postgresql) for more information.

## Parameters

| Parameter         | Description                                                    | Type | Default      |
| ----------------- | -------------------------------------------------------------- | ---- | ------------ |
| **host**          | Hostname/IP address of the PostgreSQL instance.                | str  | "127.0.0.1"  |
| **port**          | PostgreSQL port.                                               | str  | "5432"       |
| **user**          | PostgreSQL username.                                           | str  | "{username}" |
| **password**      | Password of PostgreSQL username.                               | str  | N/A          |
| **database**      | Database name to connect to.                                   | str  | "{username}" |
| **table**         | Table name where to store data.                                | str  | N/A          |
| **timestamp_Key** | Key in the JSON object containing the record timestamp.        | str  | "date"       |
| **async**         | Define if we will use async or sync connections.               | bool | false        |
| **min_pool_size** | Minimum number of connection in async mode.                    | str  | "1"          |
| **max_pool_size** | Maximum amount of connections in async mode.                   | str  | "4"          |
| **cockroachdb**   | Set to true if you will connect the plugin with a CockroachDB. | bool | false        |

## Node configuration

```yaml
...
flb_pgsql:
  plugin: "dc_destinations/FlbPgSQL"
  inputs: ["/dc/group/data"]
  host: "127.0.0.1"
  port: 5432
  user: fluentbit
  password: password
  database: "fluentbit"
  table: "dc"
  timestamp_key: "date"
  async: false
  time_format: "double"
  time_key: "date"
...
```
