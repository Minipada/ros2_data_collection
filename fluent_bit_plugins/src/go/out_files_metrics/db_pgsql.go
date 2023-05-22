package main

import (
	"C"
	"database/sql"
	"fmt"
	"unsafe"

	"github.com/fluent/fluent-bit-go/output"
	_ "github.com/lib/pq"
)

type PGSQLConfig struct {
	host     string
	port     string
	user     string
	password string
	database string
	table    string
	time_key string
	ssl      string
}

func PGSQLInit(plugin unsafe.Pointer) error {
	plugin_conf.db.postgres = true
	pgsql_config = PGSQLConfig{
		host:     output.FLBPluginConfigKey(plugin, "pgsql_host"),
		port:     output.FLBPluginConfigKey(plugin, "pgsql_port"),
		user:     output.FLBPluginConfigKey(plugin, "pgsql_user"),
		password: output.FLBPluginConfigKey(plugin, "pgsql_password"),
		database: output.FLBPluginConfigKey(plugin, "pgsql_database"),
		table:    output.FLBPluginConfigKey(plugin, "pgsql_table"),
		time_key: output.FLBPluginConfigKey(plugin, "pgsql_timestamp_key"),
		ssl:      output.FLBPluginConfigKey(plugin, "pgsql_use_ssl"),
	}

	connStr := fmt.Sprintf("postgres://%s:%s@%s:%s?sslmode=%s", pgsql_config.user, pgsql_config.password, pgsql_config.host, pgsql_config.port, pgsql_config.ssl)
	fmt.Printf("[flb-files-metric] PGSQL connstr: '%s'\n", connStr)
	var err error
	db, _ = sql.Open("postgres", connStr)
	err = db.Ping()
	if err != nil {
		fmt.Println("[flb-files-metric] Could not ping Postgres")
		return err
	}
	var exists bool
	fmt.Printf("[flb-files-metric] Trying to find database %s...\n", pgsql_config.database)
	err = db.QueryRow(fmt.Sprintf("SELECT 1 from pg_database WHERE datname='%s'", pgsql_config.database)).Scan(&exists)

	if err != nil && err != sql.ErrNoRows {
		fmt.Println("[flb-files-metric] Error checking if database exists")
	} else {
		if exists {
			fmt.Println("[flb-files-metric] Database exists")
		} else {
			_, err = db.Exec("CREATE DATABASE " + pgsql_config.database)
			if err != nil {
				return err
			}
			fmt.Printf("[flb-files-metric] Created database %s\n", pgsql_config.database)
		}
	}

	_, table_check := db.Query("select * from " + pgsql_config.table + ";")

	if table_check == nil {
		fmt.Println("[flb-files-metric] Table exists")
	} else {
		fmt.Println("[flb-files-metric] Table not found, creating...")
		query := fmt.Sprintf(`CREATE TABLE IF NOT EXISTS %s(
		id SERIAL PRIMARY KEY,
		timestamp TIMESTAMP NOT NULL,
		robot_name TEXT NOT NULL,
		robot_id TEXT NOT NULL,
		group_name TEXT NOT NULL,
		duration FLOAT8,
		local_path TEXT NOT NULL,
		remote_path TEXT NOT NULL,
		uploaded BOOLEAN NOT NULL,
		on_filesystem BOOLEAN NOT NULL,
        deleted BOOLEAN NOT NULL,
		ignored BOOLEAN NOT NULL,
        storage_type TEXT NOT NULL,
        content_type TEXT NOT NULL,
		size INTEGER NOT NULL,
		created_at TIMESTAMP NOT NULL,
		updated_at TIMESTAMP NOT NULL)`, pgsql_config.table)
		_, err := db.ExecContext(ctx, query)
		if err != nil {
			fmt.Printf("[flb-files-metric] Error %s when creating table", err)
			return err
		} else {
			fmt.Printf("[flb-files-metric] Created table %s\n", pgsql_config.table)
		}
	}
	return err
}
