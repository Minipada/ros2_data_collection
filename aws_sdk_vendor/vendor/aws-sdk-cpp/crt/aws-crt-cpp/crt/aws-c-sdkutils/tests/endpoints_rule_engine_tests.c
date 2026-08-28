/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/byte_buf.h>
#include <aws/common/file.h>
#include <aws/common/hash_table.h>
#include <aws/common/json.h>
#include <aws/common/string.h>
#include <aws/sdkutils/endpoints_rule_engine.h>
#include <aws/sdkutils/partitions.h>
#include <aws/sdkutils/private/endpoints_types_impl.h>
#include <aws/testing/aws_test_harness.h>
#include <time.h>

static int read_file_contents(
    struct aws_byte_buf *out_buf,
    struct aws_allocator *alloc,
    const struct aws_byte_cursor filename_cur) {
    AWS_ZERO_STRUCT(*out_buf);
    struct aws_string *mode = aws_string_new_from_c_str(alloc, "r");
    struct aws_string *filename = aws_string_new_from_cursor(alloc, &filename_cur);
    FILE *fp = aws_fopen_safe(filename, mode);
    aws_string_destroy(filename);
    aws_string_destroy(mode);
    ASSERT_NOT_NULL(fp);

    int64_t file_size = 0;
    ASSERT_SUCCESS(aws_file_get_length(fp, &file_size));

    ASSERT_SUCCESS(aws_byte_buf_init(out_buf, alloc, (size_t)file_size));
    size_t read = fread(out_buf->buffer, 1, (size_t)file_size, fp);
    fclose(fp);

    /* TODO: On win size read seems to be smaller than what get length returns,
    but its still a valid json*/
    /* ASSERT_INT_EQUALS(file_size, read); */

    out_buf->len = read;

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(parse_ruleset_from_string, s_test_parse_ruleset_from_string)
static int s_test_parse_ruleset_from_string(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_sdkutils_library_init(allocator);

    struct aws_byte_buf buf;
    ASSERT_SUCCESS(read_file_contents(&buf, allocator, aws_byte_cursor_from_c_str("sample_ruleset.json")));
    struct aws_byte_cursor ruleset_json = aws_byte_cursor_from_buf(&buf);

    clock_t begin = clock();
    struct aws_endpoints_ruleset *ruleset = aws_endpoints_ruleset_new_from_string(allocator, ruleset_json);
    clock_t end = clock();
    double time_taken = (((double)(end - begin)) / CLOCKS_PER_SEC);
    AWS_LOGF_INFO(AWS_LS_SDKUTILS_ENDPOINTS_PARSING, "Parsed in(s): %f", time_taken);

    ASSERT_NOT_NULL(ruleset);

    struct aws_byte_buf partitions_buf;
    ASSERT_SUCCESS(
        read_file_contents(&partitions_buf, allocator, aws_byte_cursor_from_c_str("sample_partitions.json")));
    struct aws_byte_cursor partitions_json = aws_byte_cursor_from_buf(&partitions_buf);
    struct aws_partitions_config *partitions = aws_partitions_config_new_from_string(allocator, partitions_json);

    ASSERT_NOT_NULL(partitions);

    const struct aws_hash_table *parameters = aws_endpoints_ruleset_get_parameters(ruleset);
    struct aws_byte_cursor param_name_cur = aws_byte_cursor_from_c_str("Region");
    struct aws_hash_element *element = NULL;
    aws_hash_table_find(parameters, &param_name_cur, &element);
    ASSERT_NOT_NULL(element);

    struct aws_byte_cursor built_in =
        aws_endpoints_parameter_get_built_in((struct aws_endpoints_parameter *)element->value);
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(built_in, "AWS::Region");

    struct aws_endpoints_rule_engine *engine = aws_endpoints_rule_engine_new(allocator, ruleset, partitions);

    struct aws_endpoints_request_context *context = aws_endpoints_request_context_new(allocator);
    ASSERT_SUCCESS(aws_endpoints_request_context_add_string(
        allocator, context, aws_byte_cursor_from_c_str("Region"), aws_byte_cursor_from_c_str("us-west-2")));

    struct aws_endpoints_resolved_endpoint *resolved_endpoint = NULL;
    clock_t begin_resolve = clock();
    ASSERT_SUCCESS(aws_endpoints_rule_engine_resolve(engine, context, &resolved_endpoint));
    clock_t end_resolve = clock();
    double time_taken_resolve = (((double)(end_resolve - begin_resolve)) / CLOCKS_PER_SEC);
    AWS_LOGF_INFO(AWS_LS_SDKUTILS_ENDPOINTS_PARSING, "Resolved in(s): %f", time_taken_resolve);

    ASSERT_INT_EQUALS(AWS_ENDPOINTS_RESOLVED_ENDPOINT, aws_endpoints_resolved_endpoint_get_type(resolved_endpoint));

    struct aws_byte_cursor url_cur;
    ASSERT_SUCCESS(aws_endpoints_resolved_endpoint_get_url(resolved_endpoint, &url_cur));

    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(url_cur, "https://example.us-west-2.amazonaws.com");

    aws_endpoints_ruleset_release(ruleset);
    aws_partitions_config_release(partitions);
    aws_endpoints_rule_engine_release(engine);
    aws_endpoints_resolved_endpoint_release(resolved_endpoint);
    aws_endpoints_request_context_release(context);
    aws_byte_buf_clean_up(&buf);
    aws_byte_buf_clean_up(&partitions_buf);
    aws_sdkutils_library_clean_up();
    return AWS_OP_SUCCESS;
}

struct iteration_wrapper {
    struct aws_allocator *allocator;
    struct aws_endpoints_request_context *context;
};

enum {
    /* something is really wrong if there is more than 4 elems during the test */
    MAX_STRING_ARRAY_ELEMENTS = 4
};

static int s_on_parameter_key(
    const struct aws_byte_cursor *key,
    const struct aws_json_value *value,
    bool *out_should_continue,
    void *user_data) {
    (void)out_should_continue;

    struct iteration_wrapper *wrapper = user_data;

    if (aws_json_value_is_string(value)) {
        struct aws_byte_cursor cur;
        ASSERT_SUCCESS(aws_json_value_get_string(value, &cur));
        ASSERT_SUCCESS(aws_endpoints_request_context_add_string(wrapper->allocator, wrapper->context, *key, cur));
        return AWS_OP_SUCCESS;
    } else if (aws_json_value_is_boolean(value)) {
        bool b;
        ASSERT_SUCCESS(aws_json_value_get_boolean(value, &b));
        ASSERT_SUCCESS(aws_endpoints_request_context_add_boolean(wrapper->allocator, wrapper->context, *key, b));
        return AWS_OP_SUCCESS;
    } else if (aws_json_value_is_array(value)) {
        struct aws_byte_cursor strings[MAX_STRING_ARRAY_ELEMENTS];
        size_t len = aws_json_get_array_size(value);
        ASSERT_TRUE(len <= MAX_STRING_ARRAY_ELEMENTS);
        for (size_t i = 0; i < len; ++i) {
            struct aws_json_value *str = aws_json_get_array_element(value, i);
            ASSERT_SUCCESS(aws_json_value_get_string(str, &strings[i]));
        }
        ASSERT_SUCCESS(
            aws_endpoints_request_context_add_string_array(wrapper->allocator, wrapper->context, *key, strings, len));
        return AWS_OP_SUCCESS;
    }

    return AWS_OP_ERR;
}

struct headers_wrapper {
    struct aws_allocator *allocator;
    const struct aws_hash_table *headers;
};

static int s_on_header_key(
    const struct aws_byte_cursor *key,
    const struct aws_json_value *value,
    bool *out_should_continue,
    void *user_data) {
    (void)out_should_continue;

    struct headers_wrapper *wrapper = user_data;

    struct aws_string *key_string = aws_string_new_from_cursor(wrapper->allocator, key);
    struct aws_hash_element *element = NULL;

    ASSERT_SUCCESS(aws_hash_table_find(wrapper->headers, key_string, &element));

    ASSERT_NOT_NULL(element);

    struct aws_array_list *header_values = element->value;
    ASSERT_NOT_NULL(header_values);

    ASSERT_INT_EQUALS(aws_json_get_array_size(value), aws_array_list_length(header_values));

    for (size_t i = 0; i < aws_json_get_array_size(value); ++i) {
        struct aws_json_value *val = aws_json_get_array_element(value, i);
        struct aws_byte_cursor cur;
        ASSERT_SUCCESS(aws_json_value_get_string(val, &cur));

        bool found_match = false;
        for (size_t j = 0; j < aws_array_list_length(header_values); ++j) {
            struct aws_string *header_val = NULL;
            ASSERT_SUCCESS(aws_array_list_get_at(header_values, &header_val, j));
            if (aws_string_eq_byte_cursor(header_val, &cur)) {
                found_match = true;
                break;
            }
        }

        ASSERT_TRUE(found_match);
    }

    aws_string_destroy(key_string);

    return AWS_OP_SUCCESS;
}

static int eval_expected(struct aws_allocator *allocator, struct aws_byte_cursor file_name) {
    aws_sdkutils_library_init(allocator);

    struct aws_byte_buf ruleset_file_path;
    ASSERT_SUCCESS(
        aws_byte_buf_init_copy_from_cursor(&ruleset_file_path, allocator, aws_byte_cursor_from_c_str("valid-rules/")));
    ASSERT_SUCCESS(aws_byte_buf_append_dynamic(&ruleset_file_path, &file_name));

    struct aws_byte_buf test_cases_file_path;
    ASSERT_SUCCESS(aws_byte_buf_init_copy_from_cursor(
        &test_cases_file_path, allocator, aws_byte_cursor_from_c_str("test-cases/")));
    ASSERT_SUCCESS(aws_byte_buf_append_dynamic(&test_cases_file_path, &file_name));

    struct aws_byte_buf ruleset_buf;
    ASSERT_SUCCESS(read_file_contents(&ruleset_buf, allocator, aws_byte_cursor_from_buf(&ruleset_file_path)));
    struct aws_byte_cursor ruleset_json = aws_byte_cursor_from_buf(&ruleset_buf);

    clock_t begin = clock();
    struct aws_endpoints_ruleset *ruleset = aws_endpoints_ruleset_new_from_string(allocator, ruleset_json);
    clock_t end = clock();
    double time_taken = (((double)(end - begin)) / CLOCKS_PER_SEC);
    AWS_LOGF_INFO(AWS_LS_SDKUTILS_ENDPOINTS_PARSING, "Parsed in(s): %f", time_taken);

    ASSERT_NOT_NULL(ruleset);

    struct aws_byte_buf partitions_buf;
    ASSERT_SUCCESS(read_file_contents(&partitions_buf, allocator, aws_byte_cursor_from_c_str("partitions.json")));
    struct aws_byte_cursor partitions_json = aws_byte_cursor_from_buf(&partitions_buf);
    struct aws_partitions_config *partitions = aws_partitions_config_new_from_string(allocator, partitions_json);

    struct aws_endpoints_rule_engine *engine = aws_endpoints_rule_engine_new(allocator, ruleset, partitions);

    struct aws_byte_buf test_cases_buf;
    if (read_file_contents(&test_cases_buf, allocator, aws_byte_cursor_from_buf(&test_cases_file_path))) {
        AWS_LOGF_INFO(
            AWS_LS_SDKUTILS_ENDPOINTS_PARSING,
            "Ruleset has no associated test cases: " PRInSTR,
            AWS_BYTE_CURSOR_PRI(file_name));
        goto skip_test_cases;
    }
    struct aws_byte_cursor test_cases_json = aws_byte_cursor_from_buf(&test_cases_buf);

    struct aws_json_value *test_cases = aws_json_value_new_from_string(allocator, test_cases_json);
    struct aws_json_value *tests = aws_json_value_get_from_object(test_cases, aws_byte_cursor_from_c_str("testCases"));

    for (size_t i = 0; i < aws_json_get_array_size(tests); ++i) {
        struct aws_endpoints_request_context *context = aws_endpoints_request_context_new(allocator);
        struct aws_json_value *test = aws_json_get_array_element(tests, i);

        struct aws_byte_cursor documentation;
        struct aws_json_value *doc_json =
            aws_json_value_get_from_object(test, aws_byte_cursor_from_c_str("documentation"));
        ASSERT_SUCCESS(aws_json_value_get_string(doc_json, &documentation));

        AWS_LOGF_INFO(0, "Running test case #%zu: " PRInSTR, i, AWS_BYTE_CURSOR_PRI(documentation));

        struct aws_json_value *params = aws_json_value_get_from_object(test, aws_byte_cursor_from_c_str("params"));
        struct iteration_wrapper wrapper = {.allocator = allocator, .context = context};
        ASSERT_SUCCESS(aws_json_const_iterate_object(params, s_on_parameter_key, &wrapper));

        struct aws_endpoints_resolved_endpoint *resolved_endpoint = NULL;
        clock_t begin_resolve = clock();
        ASSERT_SUCCESS(aws_endpoints_rule_engine_resolve(engine, context, &resolved_endpoint));
        clock_t end_resolve = clock();
        double time_taken_resolve = (((double)(end_resolve - begin_resolve)) / CLOCKS_PER_SEC);
        AWS_LOGF_INFO(0, "Resolved in(s): %f", time_taken_resolve);

        struct aws_json_value *expect = aws_json_value_get_from_object(test, aws_byte_cursor_from_c_str("expect"));
        struct aws_json_value *endpoint =
            aws_json_value_get_from_object(expect, aws_byte_cursor_from_c_str("endpoint"));

        if (endpoint != NULL) {
            ASSERT_INT_EQUALS(
                AWS_ENDPOINTS_RESOLVED_ENDPOINT, aws_endpoints_resolved_endpoint_get_type(resolved_endpoint));
            struct aws_byte_cursor url;
            ASSERT_SUCCESS(aws_endpoints_resolved_endpoint_get_url(resolved_endpoint, &url));
            struct aws_json_value *expected_url_node =
                aws_json_value_get_from_object(endpoint, aws_byte_cursor_from_c_str("url"));
            struct aws_byte_cursor expected_url;
            aws_json_value_get_string(expected_url_node, &expected_url);
            ASSERT_TRUE(aws_byte_cursor_eq(&url, &expected_url));

            struct aws_byte_cursor properties;
            ASSERT_SUCCESS(aws_endpoints_resolved_endpoint_get_properties(resolved_endpoint, &properties));

            struct aws_json_value *properties_json = aws_json_value_new_from_string(allocator, properties);
            struct aws_json_value *expected_properties =
                aws_json_value_get_from_object(endpoint, aws_byte_cursor_from_c_str("properties"));

            ASSERT_TRUE(expected_properties == NULL ? properties.len == 0 : properties.len > 0);

            if (expected_properties != NULL) {
                ASSERT_TRUE(aws_json_value_compare(properties_json, expected_properties, false));
            }

            aws_json_value_destroy(properties_json);

            const struct aws_hash_table *headers;
            ASSERT_SUCCESS(aws_endpoints_resolved_endpoint_get_headers(resolved_endpoint, &headers));
            struct aws_json_value *expected_headers_node =
                aws_json_value_get_from_object(endpoint, aws_byte_cursor_from_c_str("headers"));
            if (expected_headers_node) {
                struct headers_wrapper headers_wrapper = {.allocator = allocator, .headers = headers};
                ASSERT_SUCCESS(aws_json_const_iterate_object(expected_headers_node, s_on_header_key, &headers_wrapper));
            }
        }

        struct aws_json_value *error_node = aws_json_value_get_from_object(expect, aws_byte_cursor_from_c_str("error"));
        if (error_node != NULL) {
            ASSERT_INT_EQUALS(
                AWS_ENDPOINTS_RESOLVED_ERROR, aws_endpoints_resolved_endpoint_get_type(resolved_endpoint));
            struct aws_byte_cursor error;
            ASSERT_SUCCESS(aws_endpoints_resolved_endpoint_get_error(resolved_endpoint, &error));
            struct aws_byte_cursor expected_error;
            ASSERT_SUCCESS(aws_json_value_get_string(error_node, &expected_error));

            ASSERT_TRUE(aws_byte_cursor_eq(&error, &expected_error));
        }

        aws_endpoints_resolved_endpoint_release(resolved_endpoint);
        aws_endpoints_request_context_release(context);
    }

    aws_json_value_destroy(test_cases);
    aws_byte_buf_clean_up(&test_cases_buf);

skip_test_cases:
    aws_endpoints_ruleset_release(ruleset);
    aws_partitions_config_release(partitions);
    aws_endpoints_rule_engine_release(engine);
    aws_byte_buf_clean_up(&ruleset_file_path);
    aws_byte_buf_clean_up(&ruleset_buf);
    aws_byte_buf_clean_up(&partitions_buf);
    aws_byte_buf_clean_up(&test_cases_file_path);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_aws_region, s_test_endpoints_aws_region)
static int s_test_endpoints_aws_region(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("aws-region.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_default_values, s_test_endpoints_default_values)
static int s_test_endpoints_default_values(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("default-values.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_eventbridge, s_test_endpoints_eventbridge)
static int s_test_endpoints_eventbridge(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("eventbridge.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_fns, s_test_endpoints_fns)
static int s_test_endpoints_fns(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("fns.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_get_attr_type_inference, s_test_endpoints_get_attr_type_inference)
static int s_test_endpoints_get_attr_type_inference(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("get-attr-type-inference.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_headers, s_test_endpoints_headers)
static int s_test_endpoints_headers(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("headers.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_is_virtual_hostable_s3_bucket, s_test_endpoints_is_virtual_hostable_s3_bucket)
static int s_test_endpoints_is_virtual_hostable_s3_bucket(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("is-virtual-hostable-s3-bucket.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_region_override, s_test_endpoints_region_override)
static int s_test_endpoints_region_override(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("region-override.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_minimal_ruleset, s_test_endpoints_minimal_ruleset)
static int s_test_endpoints_minimal_ruleset(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("minimal-ruleset.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_parse_arn, s_test_endpoints_parse_arn)
static int s_test_endpoints_parse_arn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("parse-arn.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_parse_url, s_test_endpoints_parse_url)
static int s_test_endpoints_parse_url(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("parse-url.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_partition_fn, s_test_endpoints_partition_fn)
static int s_test_endpoints_partition_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("partition-fn.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_substring, s_test_endpoints_substring)
static int s_test_endpoints_substring(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("substring.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_uri_encode, s_test_endpoints_uri_encode)
static int s_test_endpoints_uri_encode(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("uri-encode.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_valid_hostlabel, s_test_endpoints_valid_hostlabel)
static int s_test_endpoints_valid_hostlabel(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("valid-hostlabel.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_condition_mem_clean_up, s_test_condition_mem_clean_up)
static int s_test_condition_mem_clean_up(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("custom_object_condition.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_custom, s_test_endpoints_custom)
static int s_test_endpoints_custom(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("custom_partition.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_string_array, s_test_endpoints_string_array)
static int s_test_endpoints_string_array(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(eval_expected(allocator, aws_byte_cursor_from_c_str("string_array.json")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_malformed_no_required_default, s_test_endpoints_malformed_no_required_default)
static int s_test_endpoints_malformed_no_required_default(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_sdkutils_library_init(allocator);

    struct aws_byte_buf buf;
    ASSERT_SUCCESS(
        read_file_contents(&buf, allocator, aws_byte_cursor_from_c_str("malformed-rules/no_default_on_required.json")));
    struct aws_byte_cursor ruleset_json = aws_byte_cursor_from_buf(&buf);

    struct aws_endpoints_ruleset *ruleset = aws_endpoints_ruleset_new_from_string(allocator, ruleset_json);
    ASSERT_NOT_NULL(ruleset);

    struct aws_byte_buf partitions_buf;
    ASSERT_SUCCESS(
        read_file_contents(&partitions_buf, allocator, aws_byte_cursor_from_c_str("sample_partitions.json")));
    struct aws_byte_cursor partitions_json = aws_byte_cursor_from_buf(&partitions_buf);
    struct aws_partitions_config *partitions = aws_partitions_config_new_from_string(allocator, partitions_json);
    ASSERT_NOT_NULL(partitions);

    struct aws_endpoints_rule_engine *engine = aws_endpoints_rule_engine_new(allocator, ruleset, partitions);
    ASSERT_NOT_NULL(engine);
    struct aws_endpoints_request_context *context = aws_endpoints_request_context_new(allocator);
    ASSERT_NOT_NULL(context);
    struct aws_endpoints_resolved_endpoint *resolved = NULL;
    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_RESOLVE_INIT_FAILED,
        aws_endpoints_rule_engine_resolve(engine, context, &resolved));

    aws_endpoints_ruleset_release(ruleset);
    aws_partitions_config_release(partitions);
    aws_endpoints_rule_engine_release(engine);
    aws_endpoints_request_context_release(context);
    aws_byte_buf_clean_up(&buf);
    aws_byte_buf_clean_up(&partitions_buf);
    aws_sdkutils_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(test_endpoints_malformed_regex, s_test_endpoints_malformed_regex)
static int s_test_endpoints_malformed_regex(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_sdkutils_library_init(allocator);

    struct aws_byte_buf partitions_buf;
    ASSERT_SUCCESS(
        read_file_contents(&partitions_buf, allocator, aws_byte_cursor_from_c_str("sample_partitions_bad_regex.json")));
    struct aws_byte_cursor partitions_json = aws_byte_cursor_from_buf(&partitions_buf);
    struct aws_partitions_config *partitions = aws_partitions_config_new_from_string(allocator, partitions_json);
    ASSERT_NULL(partitions);
    ASSERT_INT_EQUALS(AWS_ERROR_SDKUTILS_PARTITIONS_PARSE_FAILED, aws_last_error());

    aws_byte_buf_clean_up(&partitions_buf);
    aws_sdkutils_library_clean_up();

    return AWS_OP_SUCCESS;
}
