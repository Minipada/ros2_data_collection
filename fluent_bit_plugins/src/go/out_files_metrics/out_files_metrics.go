package main

import (
	"C"
	"context"
	"database/sql"
	"fmt"
	"net/http"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"time"
	"unsafe"

	"github.com/fluent/fluent-bit-go/output"
	_ "github.com/lib/pq"
	// "github.com/minio/minio-go/v7/pkg/credentials"
)
import (
	"errors"

	"github.com/minio/minio-go/v7"
)

type StorageConfig struct {
	minio bool
	s3    bool
}

type DBConfig struct {
	postgres bool
}

type PluginConfig struct {
	storage StorageConfig
	db      DBConfig
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

// var uploaded = make(map[string]bool)

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
	fmt.Printf("[flb-files-metrics] path=%s\n", path)
	parts := strings.Split(path, "/")

	// Get the last element in the slice
	lastIndex := len(parts) - 1
	dateTimeString := parts[lastIndex]
	fmt.Printf("[flb-files-metrics] dateTimeString=%s\n", dateTimeString)

	// Find the index of the dot character
	dotIndex := strings.Index(dateTimeString, ".")
	fmt.Printf("[flb-files-metrics] dotIndex=%d\n", dotIndex)

	// Trim the characters starting from the dot character
	dateTimeString = strings.TrimSuffix(dateTimeString[:dotIndex], ".")
	fmt.Printf("[flb-files-metrics] dateTimeString=%s\n", dateTimeString)

	var layout string = "2006-01-02T15-04-05"

	date, err := time.Parse(layout, dateTimeString)
	if err != nil {
		fmt.Println(err)
	}
	return date.UTC().Format(time.RFC3339Nano)
}

// (fluentbit will call this)
// plugin (context) pointer to fluentbit context (state/ c code)
//
//export FLBPluginInit
func FLBPluginInit(plugin unsafe.Pointer) int {
	fmt.Printf("[flb-files-metrics] Init plugin...\n")
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
	fmt.Printf("[flb-files-metrics] Initializing storages...: %s\n", storage)
	for _, val := range storage {
		if strings.ToLower(val) == "minio" {
			minio_config = MinioInitConf(plugin)
			if minio_config.endpoint == "" {
				fmt.Printf("[flb-files-metrics] Failure when initializing Storage MinIO\n")
				return output.FLB_RETRY
			}
		} else if strings.ToLower(val) == "s3" {
			s3_config = S3InitConf(plugin)
			if s3_config.endpoint == "" {
				fmt.Printf("[flb-files-metrics] Failure when initializing Storage S3\n")
				return output.FLB_RETRY
			}
		}
	}
	fmt.Printf("[flb-files-metrics] Initialized storages\n")

	// Database
	// PGSQL
	if strings.ToLower(db_type) == "pgsql" {
		init_err := PGSQLInit(plugin)
		if init_err != nil {
			fmt.Printf("[flb-files-metrics] Cannot initialize PGSQL\n")
			return output.FLB_RETRY
		} else {
			fmt.Printf("[flb-files-metrics] Initialized PGSQL\n")
		}
	} else {
		fmt.Printf("[flb-files-metrics] No database setup. Stopping.\n")
		return output.FLB_RETRY
	}

	fmt.Printf("[flb-files-metrics] Initialized plugin\n")

	return output.FLB_OK
}

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

type FileToAnalyze struct {
	RobotName    string
	GroupName    string
	LocalPath    string
	UploadPath   string
	Uploaded     []bool
	UploadTarget []string
	// UploadCurrent []string
	ContentType string
	Size        int64
}

func isFileUploaded(storage string, remote_path string) bool {
	if plugin_conf.storage.minio && storage == "minio" {
		_, err_found := minio_client.StatObject(ctx, minio_config.bucket, remote_path, minio.StatObjectOptions{})
		if err_found == nil {
			return true
		} else {
			return false
		}
	}

	return true
}

func findFileByLocalPath(fileList []FileToAnalyze, searchPath string) (bool, int) {
	for index, file := range fileList {
		if file.LocalPath == searchPath {
			return true, index
		}
	}
	return false, -1
}

//export FLBPluginFlush
func FLBPluginFlush(data unsafe.Pointer, length C.int, tag *C.char) int {
	var ret int
	var ts interface{}
	var record map[interface{}]interface{}
	var files []FileToAnalyze
	var split_src_fields = make([]string, 0)
	var split_upload_fields = make([]string, 0)

	// Create Fluent Bit decoder
	dec := output.NewDecoder(data, int(length))
	retry := false

	// Connect to remote services
	if plugin_conf.storage.minio {
		init_err := MinioInit()
		if init_err != nil {
			fmt.Printf("[flb-files-metrics] Cannot initialize Minio\n")
			return output.FLB_RETRY
		}
		fmt.Printf("[flb-files-metrics] Initialized Minio\n")
	}
	if plugin_conf.storage.s3 {
		init_err := S3Init()
		if init_err != nil {
			fmt.Printf("[flb-files-metrics] Cannot initialize S3\n")
			return output.FLB_RETRY
		}
		fmt.Printf("[flb-files-metrics] Initialized S3\n")
	}

	for {
		// Extract Record
		ret, ts, record = output.GetRecord(dec)
		if ret != 0 || ts == nil {
			fmt.Printf("[flb-files-metrics] Could not get records, ts = %d, ret = %d\n", ts, ret)
			break
		}
		if ret != 0 {
			break
		}
		fmt.Printf("[flb-files-metrics] Got records %s!\n", record)

		var current_group string

		// Loop through each storage type
		for _, storage_type := range storage {
			// Associate functions to store paths depending on storage type
			if strings.ToLower(storage_type) == "minio" {
				split_src_fields = minio_split_src_fields
				split_upload_fields = minio_split_upload_fields
			} else if strings.ToLower(storage_type) == "s3" {
				split_src_fields = s3_split_src_fields
				split_upload_fields = s3_split_upload_fields
			}

			// Ensure length of src and upload is the same. If not, stop the plugin
			// The configuration is wrong
			if len(split_upload_fields) != len(split_src_fields) {
				// Throw an error
				err := fmt.Errorf("Source and destination fields must be of same length")
				panic(err)
			}

			// Iterate though each src_fields path
			for split_src_field_i, split_src_field_v := range split_src_fields {
				var uploaded bool
				// Each field is associated with a group equal to its measurement name
				// We find all fields for the same measurement name.
				fmt.Printf("[flb-files-metrics] current_group=%s\n", current_group)
				on_filesystem := true
				var local_path string = ""
				// Set group if it is not yet done
				var file_to_analyze FileToAnalyze
				// If first path received, set the group
				if len(current_group) == 0 {
					// Get path from the field name and record received
					local_path = getPath(record, split_src_field_v)
					fmt.Printf("[flb-files-metrics] Found %s, split_src_field_v %s\n", local_path, split_src_field_v)

					// Add to group if the file exists
					if len(local_path) != 0 {
						if _, err := os.Stat(local_path); errors.Is(err, os.ErrNotExist) {
							// File does not exist
							fmt.Printf("[flb-files-metrics] File %s does not exist\n", local_path)
							on_filesystem = false
						} else {
							current_group = fmt.Sprintf("%s", record["name"])
							uploaded = isFileUploaded(storage_type, getPath(record, split_upload_fields[split_src_field_i]))
							var content_type string
							var size int64
							if !uploaded {
								retry = true
							} else {
								content_type, size = get_file_metadata(fmt.Sprintf("%s", local_path))
							}
							file_to_analyze = FileToAnalyze{
								RobotName:    string(record["robot_name"].([]uint8)),
								GroupName:    current_group,
								LocalPath:    local_path,
								UploadPath:   getPath(record, split_upload_fields[split_src_field_i]),
								UploadTarget: []string{storage_type},
								Uploaded:     []bool{uploaded},
								ContentType:  content_type,
								Size:         size,
							}
							files = append(files, file_to_analyze)
							fmt.Printf("[flb-files-metrics] File %s exists, setting group to %s\n", local_path, current_group)
						}
					}
				} else if current_group == fmt.Sprintf("%s", record["name"]) {
					fmt.Printf("[flb-files-metrics] split_src_field_i match index %d: current_group=%s, split_src_field_v=%s\n", split_src_field_i, current_group, split_src_field_v)
					local_path = getPath(record, split_src_field_v)

					// Add to group if the file exists
					if len(local_path) != 0 {
						if _, err := os.Stat(local_path); errors.Is(err, os.ErrNotExist) {
							// File does not exist
							on_filesystem = false
							fmt.Printf("[flb-files-metrics] File %s does not exist\n", local_path)
						} else {
							// File was maybe added before, when adding it for another storage type
							found, index := findFileByLocalPath(files, local_path)
							uploaded = isFileUploaded(storage_type, getPath(record, split_upload_fields[split_src_field_i]))
							var content_type string
							var size int64
							if found {
								files[index].UploadTarget = append(files[index].UploadTarget, storage_type)
								if !uploaded {
									retry = true
								} else {
									content_type, size = get_file_metadata(fmt.Sprintf("%s", local_path))
								}
								files[index].Uploaded = append(files[index].Uploaded, uploaded)
							} else {
								content_type, size = get_file_metadata(fmt.Sprintf("%s", local_path))
								file_to_analyze = FileToAnalyze{
									RobotName:    string(record["robot_name"].([]uint8)),
									GroupName:    current_group,
									LocalPath:    local_path,
									UploadPath:   getPath(record, split_upload_fields[split_src_field_i]),
									UploadTarget: []string{storage_type},
									Uploaded:     []bool{uploaded},
									ContentType:  content_type,
									Size:         size,
								}
								files = append(files, file_to_analyze)
							}
						}
					}
				}
				if len(local_path) != 0 {
					fmt.Printf("[flb-files-metrics] Files %v\n", files)
					_, index := findFileByLocalPath(files, local_path)
					if index != -1 {
						file := files[index]
						optional_fields := ""
						optional_values := ""
						if file.ContentType == "video/mp4" || file.ContentType == "application/octet-stream" {
							optional_fields += ", duration"
							optional_values += fmt.Sprintf(", '%f'", getDurationVideo(fmt.Sprintf("%s", file.LocalPath)))
						}
						timestamp := getDateFromPath(fmt.Sprintf("%s", file.LocalPath))
						updated_at := time.Now().UTC().Format(time.RFC3339Nano)
						robot_id := record["id"]
						prefix_path := ""
						if plugin_conf.storage.minio && storage_type == "minio" {
							prefix_path = "minio://" + minio_config.bucket + "/"
						} else if plugin_conf.storage.minio && storage_type == "s3" {
							prefix_path = "s3://" + s3_config.bucket + "/"
						}
						sqlStatement := fmt.Sprintf(`INSERT INTO %s(timestamp, robot_id, robot_name, group_name, local_path, remote_path, uploaded, on_filesystem, deleted, storage_type, content_type, size, updated_at %s)
			 		VALUES ('%s', '%s', '%s', '%s', '%s', '%s', %t, %t, false, '%s', '%s', %d, '%s'%s);`, pgsql_config.table, optional_fields, timestamp, robot_id, file.RobotName, file.GroupName, file.LocalPath, prefix_path+file.UploadPath, uploaded, on_filesystem, storage_type, file.ContentType, file.Size, updated_at, optional_values)
						fmt.Printf("[flb-files-metrics] Statement: %s\n", sqlStatement)
						_, err := db.Exec(sqlStatement)
						if err != nil {
							retry = true
							fmt.Printf("[flb-files-metrics] Could not insert in DB: %s\n", err)
						} else {
							fmt.Printf("[flb-files-metrics] Inserted in DB\n")
						}

						// If uploaded everywhere
						if !retry {
							fmt.Printf("[flb-files-metrics] Uploaded everywhere %s\n", local_path)
							if delete_when_sent {
								fmt.Printf("[flb-files-metrics] Deleting ...\n")
								os.Remove(fmt.Sprintf("%s", local_path))
								// Ignore errors of os.Remove.
								// We should not retry because it has already been sent. It is possible the file was sent and application stopped and when starting again, want to send again
								// So we ignore it.
								sqlStatement := fmt.Sprintf(`UPDATE %s
						SET deleted = true,
						updated_at = '%s'
						WHERE local_path = '%s';`, pgsql_config.table, time.Now().UTC().Format(time.RFC3339Nano), local_path)
								fmt.Printf("[flb-files-metrics] Statement: %s\n", sqlStatement)
								_, err = db.Exec(sqlStatement)
								if err != nil {
									fmt.Printf("[flb-files-metrics] Could not update DB\n")
								} else {
									fmt.Printf("[flb-files-metrics] Updated DB\n")
								}
							}
						} else {
							retry = true
							fmt.Printf("[flb-files-metrics] Not uploaded everywhere %s\n", local_path)
						}
					}
				}
			}
		}
		fmt.Printf("[flb-files-metrics] Files %v\n", files)

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
