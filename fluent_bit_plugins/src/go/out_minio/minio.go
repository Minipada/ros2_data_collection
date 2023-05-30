package main

import (
	"context"
	"fmt"
	"log"
	"strconv"

	"github.com/minio/minio-go/v7"
	"github.com/minio/minio-go/v7/pkg/credentials"
)

var minio_client *minio.Client
var ctx = context.Background()
var endpoint string
var access_key_id string
var secret_access_key string
var use_ssl string
var create_bucket string
var bucket string

func MinioInit() error {

	use_ssl_bool, err := strconv.ParseBool(use_ssl)
	if err != nil {
		fmt.Printf("[flb-minio] Could not parse ssl into boolean\n")
		return err
	}
	// Initialize minio client object.
	minio_client, err = minio.New(endpoint, &minio.Options{
		Creds:  credentials.NewStaticV4(access_key_id, secret_access_key, ""),
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
