package main

import (
	"C"
	"errors"
	"fmt"
	"os"
	"strings"
	"time"
	"unsafe"

	"github.com/fluent/fluent-bit-go/output"
	"github.com/minio/minio-go/v7"
)
import "strconv"

var verbose string
var verbose_bool bool
var upload_fields string
var src_fields string
var groups string
var split_upload_fields = make([]string, 0)
var split_src_fields = make([]string, 0)
var split_groups = make([]string, 0)

func getPath(record map[interface{}]interface{}, dot_separated_path string) string {
	var key_lookup string = ""
	var path string = ""

	if record["nested"] == true && record["flattened"] == false {
		key_lookup = "/" + fmt.Sprintf("%s", record["name"])
		path = fmt.Sprintf("%s", record[key_lookup])
	} else if record["nested"] == true && record["flattened"] == true {
		key_lookup = "/" + fmt.Sprintf("%s", record["name"]) + "/" + strings.Replace(dot_separated_path, ".", "/", -1)
		path = fmt.Sprintf("%s", record[key_lookup])
	} else if record["nested"] == true && record["flattened"] == false {
		key_lookup = "/" + fmt.Sprintf("%s", record["name"]) + "/" + strings.Replace(dot_separated_path, ".", "/", -1)
		path = fmt.Sprintf("%s", record[key_lookup])
	} else if record["nested"] == false && record["flattened"] == false {
		local_path_val, err := NestedMapLookup(record, strings.Split(dot_separated_path, ".")...)
		if err == nil {
			path = fmt.Sprintf("%s", local_path_val)
		}
	}

	return path
}

//export FLBPluginFlush
func FLBPluginFlush(data unsafe.Pointer, length C.int, tag *C.char) int {
	init_err := MinioInit()

	if init_err != nil {
		// fmt.Printf("[flb-minio] Could not connect to minio\n")
		return output.FLB_RETRY
	}
	fmt.Printf("[flb-minio] Connected to Minio\n")
	var ret int
	var ts interface{}
	var record map[interface{}]interface{}

	// Create Fluent Bit decoder
	dec := output.NewDecoder(data, int(length))
	retry := false

	for {
		// Extract Record
		ret, ts, record = output.GetRecord(dec)
		if ret != 0 || ts == nil {
			fmt.Printf("[flb-minio] Could not get records, ts = %d, ret = %d\n", ts, ret)
			break
		}
		if ret != 0 {
			// fmt.Printf("[flb-minio] No more records, ret = %d\n", ret)
			break
		}
		if verbose_bool || true {
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

			var local_path string = ""
			if len(current_group) == 0 {
				local_path = getPath(record, split_src_field_v)
				fmt.Printf("[flb-minio] Found %s\n", local_path)

				// Add to group if the file exists
				if len(local_path) != 0 {
					if _, err := os.Stat(local_path); errors.Is(err, os.ErrNotExist) {
						// File does not exist
						fmt.Printf("[flb-minio] File %s does not exist\n", local_path)
					} else {
						current_group = fmt.Sprintf("%s", record["name"])
						fmt.Printf("[flb-minio] File %s exists, setting group to %s\n", local_path, current_group)
					}
				}
			}
			if current_group == fmt.Sprintf("%s", record["name"]) {
				fmt.Printf("[flb-minio] split_src_field_i match index %d: current_group=%s, split_src_field_v=%s\n", split_src_field_i, current_group, split_src_field_v)
				local_path = getPath(record, split_src_field_v)

				// Add to group if the file exists
				if len(local_path) != 0 {
					if _, err := os.Stat(local_path); errors.Is(err, os.ErrNotExist) {
						// File does not exist
						index_ignore = append(index_ignore, index)
					} else {
						src_paths = append(src_paths, fmt.Sprintf("%s", local_path))
					}
				}
			}
			index++
		}

		index = 0
		upload_paths := []string{}
		for split_upload_field_i, split_upload_field_v := range split_upload_fields {
			// Ignore fields that don't belong to the group. Since they are not in the JSON, we skip them to not create errors
			if current_group == fmt.Sprintf("%s", record["name"]) {
				fmt.Printf("[flb-minio] split_upload_field_i match index %d: current_group=%s\n", split_upload_field_i, current_group)
				// Skip indexes of files that don't exist
				if !contains(index_ignore, index) {
					var upload_path = getPath(record, split_upload_field_v)
					fmt.Printf("[flb-minio] Adding path %s in list\n", upload_path)
					if len(upload_path) != 0 {
						upload_paths = append(upload_paths, upload_path)
					}
				}
			}
			index++
		}

		size := len(src_paths)
		fmt.Println("[flb-minio] Files to upload:", size)
		for _, path := range upload_paths {
			fmt.Println(path)
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
				time.Sleep(20 * time.Millisecond)

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
						fmt.Println(err)
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

//export FLBPluginRegister
func FLBPluginRegister(def unsafe.Pointer) int {
	return output.FLBPluginRegister(def, "minio", "Minio GO!")
}

// (fluentbit will call this)
// plugin (context) pointer to fluentbit context (state/ c code)
//
//export FLBPluginInit
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

	split_upload_fields = strings.Fields(upload_fields)
	fmt.Printf("[flb-minio] split_upload_fields = '%v'\n", split_upload_fields)
	split_src_fields = strings.Fields(src_fields)
	fmt.Printf("[flb-minio] split_src_fields = '%v'\n", split_src_fields)

	return output.FLB_OK
}

//export FLBPluginExit
func FLBPluginExit() int {
	fmt.Printf("[flb-minio] Exited plugin\n")
	return output.FLB_OK
}

func main() {
}
