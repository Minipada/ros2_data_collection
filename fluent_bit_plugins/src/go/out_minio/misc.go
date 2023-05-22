package main

import (
	"fmt"
	"net/http"
	"os"
)

func NestedMapLookup(m map[interface{}]interface{}, ks ...string) (rval interface{}, err error) {
	var ok bool

	if len(ks) == 0 { // degenerate input
		return nil, fmt.Errorf("NestedMapLookup needs at least one key")
	}
	if rval, ok = m[ks[0]]; !ok {
		return nil, fmt.Errorf("key not found; remaining keys: %v", ks)
	} else if len(ks) == 1 { // we've reached the final key
		return rval, nil
	} else if m, ok = rval.(map[interface{}]interface{}); !ok {
		return nil, fmt.Errorf("malformed structure at %#v", rval)
	} else { // 1+ more keys
		return NestedMapLookup(m, ks[1:]...)
	}
}

func get_file_content(path string) string {
	file, err := os.Open(path)

	defer file.Close()

	if err != nil {
		panic(err)
	}
	// Get the file content
	buf := make([]byte, 512)
	_, err = file.Read(buf)

	if err != nil {
		panic(err)
	}

	contentType := http.DetectContentType(buf)

	return contentType
}

func contains(elems []int, v int) bool {
	for _, s := range elems {
		if v == s {
			return true
		}
	}
	return false
}
