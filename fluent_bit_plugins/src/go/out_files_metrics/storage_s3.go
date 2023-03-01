package main

import (
	"C"
	"fmt"
	// "strconv"
    "strings"
	"unsafe"

	"github.com/fluent/fluent-bit-go/output"
)

type S3Config struct {
    endpoint string
    access_key_id string
    secret_access_key string
    use_ssl bool
    bucket string
    src_fields []string
    upload_fields []string
}

// S3
var s3_config S3Config
var s3_endpoint string
var s3_access_key_id string
var s3_secret_access_key string
var s3_use_ssl string
var s3_create_bucket string
var s3_bucket string
var s3_upload_fields string
var s3_src_fields string
var s3_input_names string
var s3_split_upload_fields = make([]string, 0)
var s3_split_src_fields = make([]string, 0)
var s3_split_input_names = make([]string, 0)

func S3Init() error{
	// Initialize s3 client object.
	//TODO

	return nil
}

func S3InitConf(plugin unsafe.Pointer) S3Config {
    plugin_conf.storage.s3 = true
    fmt.Printf("[flb-files-metric] Initializing S3...\n")

    s3_src_fields = output.FLBPluginConfigKey(plugin, "s3_src_fields")
    s3_split_src_fields = strings.Fields(s3_src_fields)
    s3_upload_fields = output.FLBPluginConfigKey(plugin, "s3_upload_fields")
    s3_split_upload_fields = strings.Fields(s3_upload_fields)
    s3_input_names = output.FLBPluginConfigKey(plugin, "s3_input_names")
    s3_split_input_names = strings.Fields(s3_input_names)

    s3_config = S3Config{
        endpoint: output.FLBPluginConfigKey(plugin, "s3_endpoint"),
        access_key_id: output.FLBPluginConfigKey(plugin, "s3_access_key_id"),
        secret_access_key: output.FLBPluginConfigKey(plugin, "s3_secret_access_key"),
        bucket:  output.FLBPluginConfigKey(plugin, "s3_bucket"),
    }
    fmt.Printf("[flb-files-metric] Initialized S3\n")
    return s3_config
}
