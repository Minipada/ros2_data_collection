package main

import (
	"C"
	"fmt"
	"strconv"
    "strings"
	"unsafe"

	"github.com/fluent/fluent-bit-go/output"
	"github.com/minio/minio-go/v7"
	"github.com/minio/minio-go/v7/pkg/credentials"
)

type MinioConfig struct {
    endpoint string
    access_key_id string
    secret_access_key string
    use_ssl bool
    bucket string
    src_fields []string
    upload_fields []string
}

// Minio
var minio_config MinioConfig
var minio_client *minio.Client
var minio_endpoint string
var minio_access_key_id string
var minio_secret_access_key string
var minio_use_ssl string
var minio_create_bucket string
var minio_bucket string
var minio_upload_fields string
var minio_src_fields string
var minio_input_names string
var minio_split_upload_fields = make([]string, 0)
var minio_split_src_fields = make([]string, 0)
var minio_split_input_names = make([]string, 0)

func MinioInit() error{
	// Initialize minio client object.
	var err error
	minio_client, err = minio.New(minio_config.endpoint, &minio.Options{
		Creds: credentials.NewStaticV4(minio_config.access_key_id, minio_config.secret_access_key, ""),
		Secure: minio_config.use_ssl,
	})

	return err
}

func MinioInitConf(plugin unsafe.Pointer) MinioConfig {
    plugin_conf.storage.minio = true
    fmt.Printf("[flb-files-metric] Initializing Minio...\n")
    minio_use_ssl = output.FLBPluginConfigKey(plugin, "minio_use_ssl")
    use_ssl_bool, err := strconv.ParseBool(minio_use_ssl)
    if err != nil {
        fmt.Printf("[flb-files-metric] Could not parse use_ssl\n")
        return MinioConfig{}
    }

    minio_src_fields = output.FLBPluginConfigKey(plugin, "minio_src_fields")
    minio_split_src_fields = strings.Fields(minio_src_fields)
    minio_upload_fields = output.FLBPluginConfigKey(plugin, "minio_upload_fields")
    minio_split_upload_fields = strings.Fields(minio_upload_fields)
    minio_input_names = output.FLBPluginConfigKey(plugin, "minio_input_names")
    minio_split_input_names = strings.Fields(minio_input_names)

    minio_config = MinioConfig{
        endpoint: output.FLBPluginConfigKey(plugin, "minio_endpoint"),
        access_key_id: output.FLBPluginConfigKey(plugin, "minio_access_key_id"),
        secret_access_key: output.FLBPluginConfigKey(plugin, "minio_secret_access_key"),
        use_ssl: use_ssl_bool,
        bucket:  output.FLBPluginConfigKey(plugin, "minio_bucket"),
    }
    fmt.Printf("[flb-files-metric] Initialized Minio\n")
    return minio_config
}
