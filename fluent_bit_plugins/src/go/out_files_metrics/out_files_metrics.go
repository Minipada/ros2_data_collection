package main

import (
	"C"
	"context"
	"database/sql"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"time"
	"unsafe"

	_ "github.com/lib/pq"
	"github.com/fluent/fluent-bit-go/output"
	"github.com/minio/minio-go/v7"
	// "github.com/minio/minio-go/v7/pkg/credentials"
)

type StorageConfig struct {
	minio bool
	s3 bool
}

type DBConfig struct {
	postgres bool
}

type PluginConfig struct {
    storage StorageConfig
	db DBConfig
}

// General config
var storage = make([]string, 0)
var plugin_conf PluginConfig

/*
{
	"src": ["p1", "p2"],
	"dest": [{"s3": "rp1", "minio": "foo/rp1"}, {"s3": "rp2"}]
	"uploaded": [true, true]
}
*/

var uploaded = make(map[string]bool)

var db *sql.DB
var db_type string
var ctx = context.Background()
var pgsql_config PGSQLConfig
var delete_when_sent bool

func get_file_metadata(path string) (string, int64) {
	file, err := os.Open(path)

	defer file.Close()

	if err != nil {
	   return "", 0
	}
	// Get the file content
	buf := make([]byte, 512)
	_, err = file.Read(buf)

	if err != nil {
		return "", 0
	}

	defer file.Close()

	fileInfo, err := file.Stat()
	if err != nil {
		fmt.Println(err)
		return "", 0
	}

	contentType := http.DetectContentType(buf)

	return contentType, fileInfo.Size()
}


func getDurationVideo(path string) float64 {
	// Execute the "ffprobe" command to get the size of a file
	cmd := exec.Command("sh", "-c", fmt.Sprintf("/usr/bin/ffprobe -i %s -show_entries format=duration -v quiet -of csv=\"p=0\"", path))

	stdout, err := cmd.Output()

    if err != nil {
        fmt.Println(err.Error())
        return -1.0
    }

    // Print the output
	result := string(stdout)
	result = strings.TrimSuffix(result, "\n")

	f_result, err := strconv.ParseFloat(result, 64)
	if err != nil {
		fmt.Printf("Could not get the duration when parsing")
		result := -1.0
		return result
	}
	return f_result
}

//export FLBPluginRegister
func FLBPluginRegister(ctx unsafe.Pointer) int {
	return output.FLBPluginRegister(ctx, "files_metrics", "Files metrics GO!")
}

func getDateFromPath(path string) string {
	// Split the string by '/' character
	fmt.Printf("path=%s\n", path)
	parts := strings.Split(path, "/")

	// Get the last element in the slice
	lastIndex := len(parts) - 1
	dateTimeString := parts[lastIndex]
	fmt.Printf("dateTimeString=%s\n", dateTimeString)

	// Find the index of the dot character
	dotIndex := strings.Index(dateTimeString, ".")
	fmt.Printf("dotIndex=%d\n", dotIndex)

	// Trim the characters starting from the dot character
	dateTimeString = strings.TrimSuffix(dateTimeString[:dotIndex], ".")
	fmt.Printf("dateTimeString=%s\n", dateTimeString)

	var layout string = "2006-01-02T15-04-05"

	date, err := time.Parse(layout,dateTimeString)
	if err != nil {
		fmt.Println(err)
	}
	return date.UTC().Format(time.RFC3339Nano)
}

//export FLBPluginInit
// (fluentbit will call this)
// plugin (context) pointer to fluentbit context (state/ c code)
func FLBPluginInit(plugin unsafe.Pointer) int {
	fmt.Printf("[flb-files-metric] Init plugin...\n")
	storage = strings.Fields(output.FLBPluginConfigKey(plugin, "file_storage"))
	delete_when_sent_str := output.FLBPluginConfigKey(plugin, "delete_when_sent")
    db_type = output.FLBPluginConfigKey(plugin, "db_type")
	var err error
	delete_when_sent, err = strconv.ParseBool(delete_when_sent_str)
	if err != nil {
		return output.FLB_RETRY
	}


    // Storage
    // Minio
	fmt.Printf("[flb-files-metric] Initializing storages...: %s\n", storage)
    for _, val := range storage {
		if strings.ToLower(val) == "minio" {
			minio_config = MinioInitConf(plugin)
			if minio_config.endpoint == "" {
				return output.FLB_RETRY
			}
        } else if strings.ToLower(val) == "s3" {
			s3_config = S3InitConf(plugin)
			if s3_config.endpoint == "" {
				return output.FLB_RETRY
			}
        }
    }
	fmt.Printf("[flb-files-metric] Initialized storages\n")

    // Database
    // PGSQL
    if strings.ToLower(db_type) == "pgsql" {
		init_err := PGSQLInit(plugin)
		if init_err != nil {
			fmt.Printf("[flb-files-metric] Cannot initialize PGSQL\n")
			return output.FLB_RETRY
        } else {
			fmt.Printf("[flb-files-metric] Initialized PGSQL\n")
		}
    } else {
		fmt.Printf("[flb-files-metric] No database setup. Stopping.\n")
		return output.FLB_RETRY
	}

	fmt.Printf("[flb-files-metric] Initialized plugin\n")

	return output.FLB_OK
}

//export FLBPluginFlush
func FLBPluginFlush(data unsafe.Pointer, length C.int, tag *C.char) int {
	var ret int
	var ts interface{}
	var record map[interface{}]interface{}

	// Create Fluent Bit decoder
	dec := output.NewDecoder(data, int(length))
	retry := false

	// Connect to remote services
	if plugin_conf.storage.minio {
        init_err := MinioInit()
        if init_err != nil {
			fmt.Printf("[flb-files-metric] Cannot initialize Minio\n")
            return output.FLB_RETRY
        }
        fmt.Printf("[flb-files-metric] Initialized Minio\n")
    }
	if plugin_conf.storage.s3 {
        init_err := S3Init()
        if init_err != nil {
			fmt.Printf("[flb-files-metric] Cannot initialize S3\n")
            return output.FLB_RETRY
        }
        fmt.Printf("[flb-files-metric] Initialized S3\n")
    }


	for {
		// Extract Record
		ret, ts, record = output.GetRecord(dec)
		if ret != 0 || ts == nil{
			fmt.Printf("[flb-files-metric] No more records, ret = %d\n", ret)
			break
		}
		fmt.Printf("[flb-files-metric] Got records %s!\n", record)

		var data = make(map[string]map[string]map[string]string) // Map of string to map of string
		var split_upload_fields = make([]string, 0)
		var split_src_fields = make([]string, 0)
		var split_input_names = make([]string, 0)
		// Loop for each storage
		for _, storage_v := range storage {
			if strings.ToLower(storage_v) == "minio" {
				split_src_fields = minio_split_src_fields
				split_upload_fields = minio_split_upload_fields
				split_input_names = minio_split_input_names
			} else if strings.ToLower(storage_v) == "s3" {
				split_src_fields = s3_split_src_fields
				split_upload_fields = s3_split_upload_fields
				split_input_names = s3_split_input_names
			}

			fmt.Printf("data: %s\n", data)
			fmt.Printf("split_upload_fields: %s\n", split_upload_fields)
			fmt.Printf("split_input_names: %s\n", split_input_names)
			for split_src_fields_i, split_src_fields_v := range split_src_fields {
				if data[split_src_fields_v] == nil {
					data[split_src_fields_v] =  map[string]map[string]string{storage_v: {"remote_path": split_upload_fields[split_src_fields_i], "input_name": split_input_names[split_src_fields_i]}}
					} else {
					data[split_src_fields_v][storage_v]["remote_path"] = split_upload_fields[split_src_fields_i]
				}
			}
		}

		jsonString, _ := json.Marshal(data)
		fmt.Printf("[flb-files-metric] data %s\n", jsonString)

		for data_src, _ := range data {
			status_on_filesystem := false
			status_all_uploaded := true
			local_file, err := NestedMapLookup(record, strings.Split(data_src, ".")...)
			if err != nil {
				fmt.Printf("[flb-files-metric] Key %s not found!\n", local_file)
				retry = true
				// Skip record but need to redo it later
				continue
			}
			// input_name, err := NestedMapLookup(record, strings.Split(data_src, ".")...)
			// if err != nil {
			// 	fmt.Printf("[flb-files-metric] Key %s not found!\n", local_file)
			// 	retry = true
			// 	// Skip record but need to redo it later
			// 	continue
			// }

			if _, err := os.Stat(fmt.Sprintf("%s", local_file)); errors.Is(err, os.ErrNotExist) {
				// file does not exist
			} else {
				status_on_filesystem = true
			}

			status_uploaded_per_dest := make(map[string]bool)
			for _, storage_v := range storage {
				if path_key, ok := data[data_src][storage_v]["remote_path"]; ok {
					fmt.Printf("[flb-files-metric] Getting %s\n", path_key)
					remote_path_interface, err_nested_lookup := NestedMapLookup(record, strings.Split(path_key, ".")...)
					var remote_path string
					if err_nested_lookup != nil {
						retry = true
						fmt.Printf("[flb-files-metric] Lookup error %s\n", err_nested_lookup)
						remote_path = ""
					} else {
						remote_path = fmt.Sprintf("%s", remote_path_interface)
					}
					var err_found error = nil
					status_ignored := true
					prefix_path := ""
					if plugin_conf.storage.minio && storage_v == "minio" {
						status_ignored = false
						fmt.Printf("[flb-files-metric] Searching %s in bucket %s...\n", remote_path, minio_config.bucket)
						_, err_found = minio_client.StatObject(ctx, minio_config.bucket, remote_path, minio.StatObjectOptions{})
						prefix_path = "minio://" + minio_config.bucket + "/"
					}
					if ! status_ignored {
						if err_found == nil {
							// Found
							status_all_uploaded = status_all_uploaded && true
							status_uploaded_per_dest[storage_v] = true
							fmt.Printf("[flb-files-metric] Found %s on %s!\n", remote_path, storage_v)
						} else {
							// Not found
							retry = true
							status_all_uploaded = false
							status_uploaded_per_dest[storage_v] = false
							fmt.Printf("[flb-files-metric] Not %s found on %s!\n", remote_path, storage_v)
						}
					} else {
						status_all_uploaded = false
						status_uploaded_per_dest[storage_v] = false
						fmt.Printf("[flb-files-metric] Ignored %s\n", path_key)
					}
					ts = time.Now().UTC().Format(time.RFC3339Nano)
					format_file, size_file := get_file_metadata(fmt.Sprintf("%s", local_file))
					data_ts := getDateFromPath(fmt.Sprintf("%s", local_file))
					optional_fields := ""
					optional_values := ""
					if (format_file == "video/mp4" || format_file == "application/octet-stream"){
						optional_fields += ", duration"
						optional_values += fmt.Sprintf(", '%f'", getDurationVideo(fmt.Sprintf("%s", local_file)))
					}
					sqlStatement := fmt.Sprintf(`INSERT INTO %s(timestamp, robot_id, robot_name, input_name, local_path, remote_path, uploaded, on_filesystem, deleted, ignored, storage_type, content_type, size, created_at, updated_at %s)
					VALUES ('%s', '%s', '%s', '%s', '%s', '%s', %t, %t, false, %t, '%s', '%s', %d, '%s', '%s'%s);`, pgsql_config.table, optional_fields, data_ts, record["id"], record["robot_name"], data[data_src][storage_v]["input_name"], local_file, prefix_path + remote_path, status_uploaded_per_dest[storage_v], status_on_filesystem, status_ignored, storage_v, format_file, size_file, ts, ts, optional_values)
					fmt.Printf("[flb-files-metric] Statement: %s\n", sqlStatement)
					_, err = db.Exec(sqlStatement)
					if err != nil {
						retry = true
						fmt.Printf("[flb-files-metric] Could not insert in DB\n")
					} else {
						fmt.Printf("[flb-files-metric] Inserted in DB\n")
					}
				}
			}
			if status_all_uploaded {
				fmt.Printf("[flb-files-metric] Uploaded everywhere %s\n", local_file)
				if delete_when_sent {
					fmt.Printf("[flb-files-metric] Deleting ...\n")
					e := os.Remove(fmt.Sprintf("%s", local_file))
					if e != nil {
						retry = true
						fmt.Printf("[flb-files-metric] Could not delete %s\n", local_file)
					} else {
						sqlStatement := fmt.Sprintf(`UPDATE %s
						SET deleted = true,
						updated_at = '%s'
						WHERE local_path = '%s';`, pgsql_config.table, time.Now().UTC().Format(time.RFC3339Nano), local_file)
						fmt.Printf("[flb-files-metric] Statement: %s\n", sqlStatement)
						_, err = db.Exec(sqlStatement)
						if err != nil {
							fmt.Printf("[flb-files-metric] Could not update DB\n")
						} else {
							fmt.Printf("[flb-files-metric] Updated DB\n")
						}
					}
				}
			} else {
				retry = true
				fmt.Printf("[flb-files-metric] Not uploaded everywhere %s\n", local_file)
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
	return output.FLB_OK
}

func main() {
}
