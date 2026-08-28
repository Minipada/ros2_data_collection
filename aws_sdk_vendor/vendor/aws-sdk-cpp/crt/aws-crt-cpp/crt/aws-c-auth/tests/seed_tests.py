#
# Copyright 2010-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# 
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
# 
#  http://aws.amazon.com/apache2.0
# 
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.
#

import os
import copy
import argparse
import pdb
import json

base_context = {
    "region":"us-east-1",
    "service":"service",
    "timestamp":"2015-08-30T12:36:00Z",
    "expiration_in_seconds":3600,
    "credentials": {
        "access_key_id" : "AKIDEXAMPLE",
        "secret_access_key" : "wJalrXUtnFEMI/K7MDENG+bPxRfiCYEXAMPLEKEY"
    },
    "sign_body": False
}

def parse_arguments():
    parser = argparse.ArgumentParser(description="Sigv4 Test Suite Generation Script")
    parser.add_argument("source_dir", action="store")
    parser.add_argument("dest_dir", action="store")

    args = vars( parser.parse_args() )

    return args

def merge_dicts(source, destination):
    for key, value in source.items():
        if isinstance(value, dict):
            # get node or create one
            node = destination.setdefault(key, {})
            merge_dicts(value, node)
        else:
            destination[key] = value

    return destination

def generate_test_case(source_dir, dest_dir, test_name, context_map):    
    source_request_filename = os.path.join(source_dir, test_name + ".req")
    if not os.path.exists(source_request_filename):
        return

    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
        
    dest_request_filename = os.path.join(dest_dir, "request.txt")
    
    with open(source_request_filename, "r") as source_file:
        with open(dest_request_filename, "w") as dest_file:
            for _, line in enumerate(source_file):
                if not line.startswith("X-Amz-Date"):
                    dest_file.write(line)
            
    test_context = copy.deepcopy(base_context)
    test_context = merge_dicts(context_map, test_context)

    context_contents = json.dumps(test_context, sort_keys=True, indent=4)

    dest_context_filename = os.path.join(dest_dir, "context.json")
    context_file = open(dest_context_filename,"w")
    context_file.write(context_contents)
    context_file.close() 
    
    return

normalized_context = {
    "normalize" : True
}

unnormalized_context = {
    "normalize" : False
}

token_context = {
    "normalize" : True,
    "credentials" : {
        "token" : "6e86291e8372ff2a2260956d9b8aae1d763fbf315fa00fa31553b73ebf194267"
    }    
}

sign_body_context = {
    "normalize": True,
    "sign_body": True
}

def generate_tests(source_dir, dest_dir, suffix, default_context_map):
    for root_dir, dir_names, file_names in os.walk( source_dir ):
        if root_dir == source_dir:
            for dir_name in dir_names:
                test_case_source_dir = os.path.join(root_dir, dir_name)

                context_map = default_context_map
                if dir_name == "get-vanilla-with-session-token":
                    context_map = token_context
                elif dir_name.startswith('post-x-www-form'):
                    context_map = sign_body_context

                v4_test_case_dest_dir = os.path.join(dest_dir, "v4", dir_name + suffix)
                v4a_test_case_dest_dir = os.path.join(dest_dir, "v4a", dir_name + suffix)

                generate_test_case(test_case_source_dir, v4_test_case_dest_dir, dir_name, context_map)
                generate_test_case(test_case_source_dir, v4a_test_case_dest_dir, dir_name, context_map)
            
    return


def main():    
    args = parse_arguments()

    source_dir = args["source_dir"]
    if not os.path.exists(source_dir):
        print("Source directory {0} does not exist".format(source_dir))
        return
    
    dest_dir = args["dest_dir"]
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
        
    generate_tests(source_dir, dest_dir, "", normalized_context)

    post_sts_token_dir = os.path.join(source_dir, "post-sts-token")
    generate_tests(post_sts_token_dir, dest_dir, "", normalized_context)

    normalize_dir = os.path.join(source_dir, "normalize-path")
    generate_tests(normalize_dir, dest_dir, "-normalized", normalized_context)
    generate_tests(normalize_dir, dest_dir, "-unnormalized", unnormalized_context)

main()
