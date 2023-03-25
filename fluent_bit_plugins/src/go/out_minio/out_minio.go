package main

import (
	"C"
	"context"
	"errors"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
	"time"
	"unsafe"

	"github.com/fluent/fluent-bit-go/output"
	"github.com/minio/minio-go/v7"
	"github.com/minio/minio-go/v7/pkg/credentials"
)

var verbose string
var verbose_bool bool
var endpoint string
var access_key_id string
var secret_access_key string
var use_ssl string
var create_bucket string
var bucket string
var upload_fields string
var src_fields string
var groups string
var split_upload_fields = make([]string, 0)
var split_src_fields = make([]string, 0)
var split_groups = make([]string, 0)
var minio_client *minio.Client
var ctx = context.Background()

//export FLBPluginRegister
func FLBPluginRegister(def unsafe.Pointer) int {
	return output.FLBPluginRegister(def, "minio", "Minio GO!")
}

//export FLBPluginInit
// (fluentbit will call this)
// plugin (context) pointer to fluentbit context (state/ c code)
func FLBPluginInit(plugin unsafe.Pointer) int {
	verbose = output.FLBPluginConfigKey(plugin, "verbose")
	var err error
	verbose_bool, err = strconv.ParseBool(verbose)
	if err != nil {
		fmt.Printf("[flb-minio] Could not parse verbose into boolean\n")
		return output.FLB_RETRY
	}
	fmt.Printf("[flb-minio] verbose = '%t'\n", verbose_bool)

	endpoint = output.FLBPluginConfigKey(plugin, "endpoint")
	fmt.Printf("[flb-minio] endpoint = '%s'\n", endpoint)
	access_key_id = output.FLBPluginConfigKey(plugin, "access_key_id")
	fmt.Printf("[flb-minio] access_key_id = '%s'\n", access_key_id)
	secret_access_key = output.FLBPluginConfigKey(plugin, "secret_access_key")
	fmt.Printf("[flb-minio] secret_access_key = '%s'\n", secret_access_key)
	use_ssl = output.FLBPluginConfigKey(plugin, "use_ssl")
	fmt.Printf("[flb-minio] use_ssl = '%s'\n", use_ssl)
	create_bucket = output.FLBPluginConfigKey(plugin, "create_bucket")
	fmt.Printf("[flb-minio] create_bucket = '%s'\n", create_bucket)
	bucket = output.FLBPluginConfigKey(plugin, "bucket")
	fmt.Printf("[flb-minio] bucket = '%s'\n", bucket)
	upload_fields = output.FLBPluginConfigKey(plugin, "upload_fields")
	fmt.Printf("[flb-minio] upload_fields = '%s'\n", upload_fields)
	src_fields = output.FLBPluginConfigKey(plugin, "src_fields")
	fmt.Printf("[flb-minio] src_fields = '%s'\n", src_fields)
	groups = output.FLBPluginConfigKey(plugin, "groups")
	fmt.Printf("[flb-minio] groups = '%s'\n", groups)

	split_upload_fields = strings.Fields(upload_fields)
	fmt.Printf("[flb-minio] split_upload_fields = '%v'\n", split_upload_fields)
	split_src_fields = strings.Fields(src_fields)
	fmt.Printf("[flb-minio] split_src_fields = '%v'\n", split_src_fields)
	split_groups = strings.Fields(groups)
	fmt.Printf("[flb-minio] split_groups = '%v'\n", split_groups)

	return output.FLB_OK
}

func MinioInit() error{

	use_ssl_bool, err := strconv.ParseBool(use_ssl)
	if err != nil {
		fmt.Printf("[flb-minio] Could not parse ssl into boolean\n")
		return err
	}
	// Initialize minio client object.
	minio_client, err = minio.New(endpoint, &minio.Options{
		Creds: credentials.NewStaticV4(access_key_id, secret_access_key, ""),
		Secure: use_ssl_bool,
	})

	if err != nil {
		fmt.Printf("[flb-minio] Could not create client while creating bucket\n")
		return err
	}

	// Create the bucket
	create_bucket, err := strconv.ParseBool(create_bucket)
	if create_bucket {
		err = minio_client.MakeBucket(ctx, bucket, minio.MakeBucketOptions{Region: "us-east-1"})
		if err != nil {
			// Check to see if we already own this bucket (which happens if you run this twice)
			exists, errBucketExists := minio_client.BucketExists(ctx, bucket)
			if errBucketExists == nil && exists {
				// log.Printf("We already own %s\n", bucket)
			} else {
				fmt.Printf("[flb-minio] Could not create bucket\n")
				return err
			}
		} else {
			log.Printf("Successfully created %s\n", bucket)
		}
	}
	return nil
}

//export FLBPluginFlush
func FLBPluginFlush(data unsafe.Pointer, length C.int, tag *C.char) int {
	init_err := MinioInit()

	if init_err != nil {
		// fmt.Printf("[flb-minio] Could not connect to minio\n")
		return output.FLB_RETRY
	}
	// fmt.Printf("[flb-minio] Connected to Minio\n")
	var ret int
	var ts interface{}
	var record map[interface{}]interface{}

	// Create Fluent Bit decoder
	dec := output.NewDecoder(data, int(length))
	retry := false

	for {
		// Extract Record
		ret, ts, record = output.GetRecord(dec)
		if ts == nil {
			fmt.Printf("[flb-minio] Could not get records, ts = nil\n")
			break
		}
		if ret != 0{
			// fmt.Printf("[flb-minio] No more records, ret = %d\n", ret)
			break
		}
		if verbose_bool {
			fmt.Printf("[flb-minio] Got records!\n")
			// Print record keys and values
			fmt.Printf("%s: {", C.GoString(tag))
			for k, v := range record {
				fmt.Printf("\"%s\": %v, ", k, v)
			}
			fmt.Printf("}\n")
		}

		var current_group string

		// Indexes of paths to ignore
		index_ignore := []int{}
		src_paths := []string{}
		index := 0
		for split_src_field_i, split_src_field_v := range split_src_fields {
			fmt.Printf("[flb-minio] current_group=%s\n", current_group)
			// Set group if it is not yet done
			if len(current_group) == 0 {
				val, err := NestedMapLookup(record, strings.Split(split_src_field_v, ".")...)
				val_str := fmt.Sprintf("%s", val)
				if err == nil {
					if _, err := os.Stat(val_str); errors.Is(err, os.ErrNotExist) {
						// File does not exist
					} else {
						current_group = split_groups[split_src_field_i]
					}
				}
			// Ignore fields that don't belong to the group. Since they are not in the JSON, we skip them to not create errors
			}
			if current_group == split_groups[split_src_field_i]{
				fmt.Printf("[flb-minio] split_src_field_i match index %d: current_group=%s, split_src_field_v=%s\n",split_src_field_i, current_group, split_src_field_v)
				val, err := NestedMapLookup(record, strings.Split(split_src_field_v, ".")...)
				val_str := fmt.Sprintf("%s", val)
				if err == nil {
					if _, err := os.Stat(val_str); errors.Is(err, os.ErrNotExist) {
						// File does not exist
						index_ignore = append(index_ignore, index)
					} else {
						src_paths = append(src_paths, val_str)
					}
				}
			}
			index++
		}

		index = 0
		upload_paths :=[]string{}
		for split_upload_field_i, split_upload_field_v := range split_upload_fields {
			// Ignore fields that don't belong to the group. Since they are not in the JSON, we skip them to not create errors
			if current_group == split_groups[split_upload_field_i]{
				fmt.Printf("[flb-minio] split_upload_field_i match index %d: current_group=%s\n",split_upload_field_i, current_group)
				// Skip indexes of files that don't exist
				if ! contains(index_ignore, index) {
					val, err := NestedMapLookup(record, strings.Split(split_upload_field_v, ".")...)
					if err == nil {
						val_str := fmt.Sprintf("%s", val)
						upload_paths = append(upload_paths, val_str)
					}
				}
			}
			index++
		}

		for src_paths_i, src_paths_v := range src_paths {
			// Upload the file file with FPutObject
			fmt.Printf("[flb-minio] Uploading %s to %s, format: %s\n", src_paths_v, upload_paths[src_paths_i], get_file_content(src_paths_v))

			// Get the initial modification time
			file, err := os.Open(src_paths_v)
			if err != nil {
				fmt.Println(err)
				retry = true
				break
			}
			defer file.Close()
			initialModTime, err := file.Stat()
			if err != nil {
				fmt.Println(err)
				retry = true
				break
			}
			fmt.Printf("[flb-minio] Initial mode: %s", initialModTime)
			// Keep looping until the file is no longer being written to
			for {
				time.Sleep(20*time.Millisecond)

				// Get the current modification time
				currentModTime, err := file.Stat()
				if err != nil {
					fmt.Println(err)
					retry = true
					break
				}

				// Compare the modification times
				if !currentModTime.ModTime().Equal(initialModTime.ModTime()) {
					fmt.Println("[flb-minio] The file is being written to.")
					initialModTime = currentModTime
				} else {
					fmt.Println("[flb-minio] The file is no longer being written to.")
					_, err = minio_client.FPutObject(ctx, bucket, upload_paths[src_paths_i], src_paths_v, minio.PutObjectOptions{ContentType: get_file_content(src_paths_v)})
					if err != nil {
						fmt.Printf("[flb-minio] Could not upload %s to %s, format: %s\n", src_paths_v, upload_paths[src_paths_i], get_file_content(src_paths_v))
						retry = true
					} else {
						fmt.Printf("[flb-minio] Uploaded %s to %s, format: %s\n", src_paths_v, upload_paths[src_paths_i], get_file_content(src_paths_v))
						break
					}
				}
			}
		}
	}

	if retry == true {
		return output.FLB_RETRY
	}
	return output.FLB_OK
}

//export FLBPluginExit
func FLBPluginExit() int {
	fmt.Printf("[flb-minio] Exited plugin\n")
	return output.FLB_OK
}

func main() {
}
