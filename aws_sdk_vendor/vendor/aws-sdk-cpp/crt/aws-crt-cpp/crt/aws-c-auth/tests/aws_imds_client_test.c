/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/auth/aws_imds_client.h>
#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/credentials_utils.h>
#include <aws/common/clock.h>
#include <aws/common/condition_variable.h>
#include <aws/common/device_random.h>
#include <aws/common/string.h>
#include <aws/common/thread.h>
#include <aws/http/request_response.h>
#include <aws/http/status_code.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/logging.h>
#include <aws/io/socket.h>

#if defined(_MSC_VER)
#    pragma warning(disable : 4244)
#endif /* _MSC_VER */

#define IMDS_CLIENT_MAX_REQUESTS 15
struct aws_mock_imds_client_tester {
    struct aws_byte_buf request_uris[IMDS_CLIENT_MAX_REQUESTS];
    struct aws_array_list response_data_callbacks[IMDS_CLIENT_MAX_REQUESTS];
    struct aws_allocator *allocator;
    int current_request;
    int response_code[IMDS_CLIENT_MAX_REQUESTS];
    int token_request_idx;
    int error_code;
    uint64_t timestamp;
    bool is_connection_acquire_successful;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    struct aws_event_loop_group *el_group;
    struct aws_client_bootstrap *bootstrap;
    bool has_received_resource_callback;
    bool has_received_shutdown_callback;
    bool token_ttl_header_exist[IMDS_CLIENT_MAX_REQUESTS];
    bool token_ttl_header_expected[IMDS_CLIENT_MAX_REQUESTS];
    bool token_header_exist[IMDS_CLIENT_MAX_REQUESTS];
    bool token_header_expected[IMDS_CLIENT_MAX_REQUESTS];
    bool alternate_closed_connections;

    struct aws_byte_buf resource;

    int successful_requests;
};

static struct aws_mock_imds_client_tester s_tester;

static void s_on_shutdown_complete(void *user_data) {
    (void)user_data;

    aws_mutex_lock(&s_tester.lock);
    s_tester.has_received_shutdown_callback = true;
    aws_mutex_unlock(&s_tester.lock);

    aws_condition_variable_notify_one(&s_tester.signal);
}

static bool s_has_tester_received_shutdown_callback(void *user_data) {
    (void)user_data;

    return s_tester.has_received_shutdown_callback;
}

static void s_aws_wait_for_imds_client_shutdown_callback(void) {
    aws_mutex_lock(&s_tester.lock);
    aws_condition_variable_wait_pred(&s_tester.signal, &s_tester.lock, s_has_tester_received_shutdown_callback, NULL);
    aws_mutex_unlock(&s_tester.lock);
}

static struct aws_http_connection_manager *s_aws_http_connection_manager_new_mock(
    struct aws_allocator *allocator,
    const struct aws_http_connection_manager_options *options) {

    (void)allocator;
    (void)options;

    return (struct aws_http_connection_manager *)1;
}

static void s_aws_http_connection_manager_release_mock(struct aws_http_connection_manager *manager) {
    (void)manager;

    s_on_shutdown_complete(NULL);
}

static void s_aws_http_connection_manager_acquire_connection_mock(
    struct aws_http_connection_manager *manager,
    aws_http_connection_manager_on_connection_setup_fn *callback,
    void *user_data) {

    (void)manager;

    if (s_tester.is_connection_acquire_successful) {
        callback((struct aws_http_connection *)1, AWS_OP_SUCCESS, user_data);
    } else {
        aws_raise_error(AWS_ERROR_HTTP_UNKNOWN);
        callback(NULL, AWS_OP_ERR, user_data);
    }
}

static int s_aws_http_connection_manager_release_connection_mock(
    struct aws_http_connection_manager *manager,
    struct aws_http_connection *connection) {

    (void)manager;
    (void)connection;

    return AWS_OP_SUCCESS;
}

static void s_invoke_mock_request_callbacks(
    const struct aws_http_make_request_options *options,
    struct aws_array_list *data_callbacks) {

    size_t data_callback_count = aws_array_list_length(data_callbacks);

    struct aws_http_header headers[1];
    AWS_ZERO_ARRAY(headers);

    headers[0].name = aws_byte_cursor_from_c_str("some-header");
    headers[0].value = aws_byte_cursor_from_c_str("value");

    options->on_response_headers(
        (struct aws_http_stream *)1, AWS_HTTP_HEADER_BLOCK_MAIN, headers, 1, options->user_data);

    if (options->on_response_header_block_done) {
        options->on_response_header_block_done(
            (struct aws_http_stream *)1, data_callback_count > 0, options->user_data);
    }

    for (size_t i = 0; i < data_callback_count; ++i) {
        struct aws_byte_cursor data_callback_cursor;
        if (aws_array_list_get_at(data_callbacks, &data_callback_cursor, i)) {
            continue;
        }

        options->on_response_body((struct aws_http_stream *)1, &data_callback_cursor, options->user_data);
    }

    if (!s_tester.alternate_closed_connections) {
        options->on_complete((struct aws_http_stream *)1, AWS_ERROR_SUCCESS, options->user_data);
    } else {
        options->on_complete(
            (struct aws_http_stream *)1,
            ((uint8_t)s_tester.current_request & 0x01) ? AWS_ERROR_HTTP_CONNECTION_CLOSED : AWS_ERROR_SUCCESS,
            options->user_data);
    }
}

static void s_validate_token_ttl_header(const struct aws_http_message *request);
static void s_validate_token_header(const struct aws_http_message *request);

static struct aws_http_stream *s_aws_http_connection_make_request_mock(
    struct aws_http_connection *client_connection,
    const struct aws_http_make_request_options *options) {

    (void)client_connection;
    (void)options;

    struct aws_byte_cursor path;
    AWS_ZERO_STRUCT(path);
    aws_http_message_get_request_path(options->request, &path);

    if (s_tester.current_request == s_tester.token_request_idx) {
        /* verify token ttl header */
        s_validate_token_ttl_header(options->request);
    } else if (s_tester.current_request > s_tester.token_request_idx) {
        /* verify token header */
        s_validate_token_header(options->request);
    }

    int idx = s_tester.current_request++;
    aws_byte_buf_append_dynamic(&(s_tester.request_uris[idx]), &path);
    s_invoke_mock_request_callbacks(options, &s_tester.response_data_callbacks[idx]);
    return (struct aws_http_stream *)1;
}

static int s_aws_http_stream_activate_mock(struct aws_http_stream *stream) {
    (void)stream;
    return AWS_OP_SUCCESS;
}

static struct aws_http_connection *s_aws_http_stream_get_connection_mock(const struct aws_http_stream *stream) {
    (void)stream;
    return (struct aws_http_connection *)1;
}

static int s_aws_http_stream_get_incoming_response_status_mock(
    const struct aws_http_stream *stream,
    int *out_status_code) {
    (void)stream;
    if (s_tester.response_code[s_tester.current_request - 1] != 0) {
        *out_status_code = s_tester.response_code[s_tester.current_request - 1];
    } else {
        *out_status_code = AWS_HTTP_STATUS_CODE_200_OK;
    }

    return AWS_OP_SUCCESS;
}

static void s_aws_http_stream_release_mock(struct aws_http_stream *stream) {
    (void)stream;
}

static void s_aws_http_connection_close_mock(struct aws_http_connection *connection) {
    (void)connection;
}

static int s_aws_high_res_clock_get_ticks_mock(uint64_t *timestamp) {
    *timestamp = s_tester.timestamp;
    return AWS_OP_SUCCESS;
}

static struct aws_auth_http_system_vtable s_mock_function_table = {
    .aws_http_connection_manager_new = s_aws_http_connection_manager_new_mock,
    .aws_http_connection_manager_release = s_aws_http_connection_manager_release_mock,
    .aws_http_connection_manager_acquire_connection = s_aws_http_connection_manager_acquire_connection_mock,
    .aws_http_connection_manager_release_connection = s_aws_http_connection_manager_release_connection_mock,
    .aws_http_connection_make_request = s_aws_http_connection_make_request_mock,
    .aws_http_stream_activate = s_aws_http_stream_activate_mock,
    .aws_http_stream_get_connection = s_aws_http_stream_get_connection_mock,
    .aws_http_stream_get_incoming_response_status = s_aws_http_stream_get_incoming_response_status_mock,
    .aws_http_stream_release = s_aws_http_stream_release_mock,
    .aws_http_connection_close = s_aws_http_connection_close_mock,
    .aws_high_res_clock_get_ticks = s_aws_high_res_clock_get_ticks_mock,
};

static int s_aws_imds_tester_init(struct aws_allocator *allocator) {

    aws_auth_library_init(allocator);

    AWS_ZERO_STRUCT(s_tester);

    s_tester.allocator = allocator;
    for (size_t i = 0; i < IMDS_CLIENT_MAX_REQUESTS; i++) {
        if (aws_array_list_init_dynamic(
                &s_tester.response_data_callbacks[i], allocator, 10, sizeof(struct aws_byte_cursor))) {
            return AWS_OP_ERR;
        }
        if (aws_byte_buf_init(&s_tester.request_uris[i], allocator, 100)) {
            return AWS_OP_ERR;
        }
    }

    if (aws_mutex_init(&s_tester.lock)) {
        return AWS_OP_ERR;
    }

    if (aws_condition_variable_init(&s_tester.signal)) {
        return AWS_OP_ERR;
    }

    /* default to everything successful */
    s_tester.is_connection_acquire_successful = true;

    s_tester.el_group = aws_event_loop_group_new_default(allocator, 1, NULL);
    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = s_tester.el_group,
        .user_data = NULL,
        .host_resolution_config = NULL,
        .host_resolver = NULL,
        .on_shutdown_complete = NULL,
    };
    s_tester.bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);
    ASSERT_NOT_NULL(s_tester.bootstrap);

    if (aws_byte_buf_init(&s_tester.resource, allocator, 256)) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

static int s_aws_imds_tester_cleanup(void) {
    for (size_t i = 0; i < IMDS_CLIENT_MAX_REQUESTS; i++) {
        aws_array_list_clean_up(&s_tester.response_data_callbacks[i]);
        aws_byte_buf_clean_up(&s_tester.request_uris[i]);
    }

    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);

    aws_client_bootstrap_release(s_tester.bootstrap);
    aws_event_loop_group_release(s_tester.el_group);
    aws_byte_buf_clean_up(&s_tester.resource);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

static bool s_has_tester_received_resource_callback(void *user_data) {
    (void)user_data;

    return s_tester.has_received_resource_callback;
}

static void s_aws_wait_for_resource_result(void) {
    aws_mutex_lock(&s_tester.lock);
    aws_condition_variable_wait_pred(&s_tester.signal, &s_tester.lock, s_has_tester_received_resource_callback, NULL);
    aws_mutex_unlock(&s_tester.lock);
}

static void s_get_resource_callback(const struct aws_byte_buf *resource, int error_code, void *user_data) {
    (void)user_data;
    aws_mutex_lock(&s_tester.lock);
    s_tester.has_received_resource_callback = true;
    s_tester.error_code = error_code;
    if (resource && resource->len) {
        struct aws_byte_cursor cursor = aws_byte_cursor_from_buf(resource);
        aws_byte_buf_append_dynamic(&s_tester.resource, &cursor);
        s_tester.successful_requests++;
    }
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_imds_client_new_release(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(imds_client_new_release, s_imds_client_new_release);

AWS_STATIC_STRING_FROM_LITERAL(s_ec2_metadata_root, "/latest/meta-data");
AWS_STATIC_STRING_FROM_LITERAL(s_expected_imds_token_uri, "/latest/api/token");
AWS_STATIC_STRING_FROM_LITERAL(s_expected_imds_resource_uri, "/latest/meta-data/iam/security-credentials/test-role");
AWS_STATIC_STRING_FROM_LITERAL(s_test_imds_token, "A00XXF3H00ZZ==");

static int s_imds_client_connect_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.is_connection_acquire_successful = false;

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);

    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_ec2_metadata_root), s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_tester.has_received_resource_callback == true);
    ASSERT_TRUE(s_tester.resource.len == 0);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(imds_client_connect_failure, s_imds_client_connect_failure);

static void s_validate_token_ttl_header(const struct aws_http_message *request) {
    const struct aws_http_headers *headers = aws_http_message_get_const_headers(request);
    struct aws_byte_cursor ttl_header = aws_byte_cursor_from_c_str("x-aws-ec2-metadata-token-ttl-seconds");
    struct aws_byte_cursor ttl_value;
    int ret = aws_http_headers_get(headers, ttl_header, &ttl_value);
    if (ret == AWS_OP_SUCCESS) {
        s_tester.token_ttl_header_exist[s_tester.current_request] = true;
        if (aws_byte_cursor_eq_c_str_ignore_case(&ttl_value, "21600")) {
            s_tester.token_ttl_header_expected[s_tester.current_request] = true;
        } else {
            s_tester.token_ttl_header_expected[s_tester.current_request] = false;
        }
    } else {
        s_tester.token_ttl_header_exist[s_tester.current_request] = false;
    }
}

static void s_validate_token_header(const struct aws_http_message *request) {
    const struct aws_http_headers *headers = aws_http_message_get_const_headers(request);
    struct aws_byte_cursor token_header = aws_byte_cursor_from_c_str("x-aws-ec2-metadata-token");
    struct aws_byte_cursor token_value;
    int ret = aws_http_headers_get(headers, token_header, &token_value);
    if (ret == AWS_OP_SUCCESS) {
        s_tester.token_header_exist[s_tester.current_request] = true;
        if (aws_byte_cursor_eq_c_str_ignore_case(&token_value, "A00XXF3H00ZZ==")) {
            s_tester.token_header_expected[s_tester.current_request] = true;
        } else {
            s_tester.token_header_expected[s_tester.current_request] = false;
        }
    } else {
        s_tester.token_header_exist[s_tester.current_request] = false;
    }
}

static int s_validate_uri_path_and_resource(int expected_requests, bool get_resource) {

    ASSERT_UINT_EQUALS(expected_requests, s_tester.current_request);

    int idx = s_tester.token_request_idx;
    if (s_tester.current_request >= 1) {
        ASSERT_BIN_ARRAYS_EQUALS(
            s_tester.request_uris[idx].buffer,
            s_tester.request_uris[idx].len,
            s_expected_imds_token_uri->bytes,
            s_expected_imds_token_uri->len);
    }
    idx++;
    if (s_tester.current_request >= 2) {
        ASSERT_BIN_ARRAYS_EQUALS(
            s_tester.request_uris[idx].buffer,
            s_tester.request_uris[idx].len,
            s_expected_imds_resource_uri->bytes,
            s_expected_imds_resource_uri->len);
    }

    ASSERT_TRUE(s_tester.has_received_resource_callback == true);

    if (get_resource) {
        ASSERT_TRUE(s_tester.resource.len != 0);
    } else {
        ASSERT_TRUE(s_tester.resource.len == 0);
    }

    return 0;
}

static int s_validate_uri_path(int expected_requests, struct aws_byte_cursor resource_uri) {

    ASSERT_UINT_EQUALS(expected_requests, s_tester.current_request);

    int idx = s_tester.token_request_idx;
    if (s_tester.current_request >= 1) {
        ASSERT_BIN_ARRAYS_EQUALS(
            s_tester.request_uris[idx].buffer,
            s_tester.request_uris[idx].len,
            s_expected_imds_token_uri->bytes,
            s_expected_imds_token_uri->len);
    }
    idx++;
    if (s_tester.current_request >= 2) {
        ASSERT_BIN_ARRAYS_EQUALS(
            s_tester.request_uris[idx].buffer, s_tester.request_uris[idx].len, resource_uri.ptr, resource_uri.len);
    }
    return 0;
}

static int s_validate_uri_path_and_excpected_resource(
    int expected_requests,
    struct aws_byte_cursor resource_uri,
    struct aws_byte_cursor expected_resource) {

    ASSERT_UINT_EQUALS(expected_requests, s_tester.current_request);

    int idx = s_tester.token_request_idx;
    if (s_tester.current_request >= 1) {
        ASSERT_BIN_ARRAYS_EQUALS(
            s_tester.request_uris[idx].buffer,
            s_tester.request_uris[idx].len,
            s_expected_imds_token_uri->bytes,
            s_expected_imds_token_uri->len);
    }
    idx++;
    if (s_tester.current_request >= 2) {
        ASSERT_BIN_ARRAYS_EQUALS(
            s_tester.request_uris[idx].buffer, s_tester.request_uris[idx].len, resource_uri.ptr, resource_uri.len);
    }

    ASSERT_TRUE(s_tester.has_received_resource_callback == true);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_tester.resource.buffer, s_tester.resource.len, expected_resource.ptr, expected_resource.len);

    return 0;
}

static int s_imds_client_token_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_400_BAD_REQUEST;
    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);

    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_ec2_metadata_root), s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_tester.has_received_resource_callback == true);
    ASSERT_TRUE(s_tester.resource.len == 0);

    ASSERT_TRUE(s_validate_uri_path_and_resource(1, false /*no resource*/) == 0);
    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(imds_client_token_request_failure, s_imds_client_token_request_failure);

static int s_imds_client_insecure_fallback_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    /* secure data flow is not supported, fallback to insecurity */
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_403_FORBIDDEN;
    /* Insecurity fails as well */
    s_tester.response_code[1] = AWS_HTTP_STATUS_CODE_401_UNAUTHORIZED;
    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);

    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_expected_imds_resource_uri), s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_tester.has_received_resource_callback == true);
    ASSERT_TRUE(s_tester.resource.len == 0);

    ASSERT_TRUE(s_validate_uri_path_and_resource(2, false /*no resource*/) == 0);
    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_UINT_EQUALS(s_tester.error_code, AWS_AUTH_IMDS_CLIENT_SOURCE_FAILURE);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(imds_client_insecure_fallback_request_failure, s_imds_client_insecure_fallback_request_failure);

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response,
    "{\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"SecretAccessKey\":\"SuccessfulSecret\", \n  "
    "\"Token\":\"TokenSuccess\", \n \"Expiration\":\"2020-02-25T06:03:31Z\"}");
AWS_STATIC_STRING_FROM_LITERAL(s_access_key, "SuccessfulAccessKey");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_key, "SuccessfulSecret");
AWS_STATIC_STRING_FROM_LITERAL(s_token, "TokenSuccess");
AWS_STATIC_STRING_FROM_LITERAL(s_expiration, "2020-02-25T06:03:31Z");

static int s_imds_client_v1_fallback_disabled_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    /* secure data flow is not supported */
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_403_FORBIDDEN;
    /* v1 would have worked if fallback was not disabled */
    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .ec2_metadata_v1_disabled = true,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);

    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_expected_imds_resource_uri), s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_tester.has_received_resource_callback == true);
    ASSERT_TRUE(s_tester.resource.len == 0);
    /* There was no fallback so we didn't get any resource */
    ASSERT_TRUE(s_validate_uri_path_and_resource(1, false /*no resource*/) == 0);
    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_UINT_EQUALS(s_tester.error_code, AWS_AUTH_IMDS_CLIENT_SOURCE_FAILURE);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(imds_client_v1_fallback_disabled_failure, s_imds_client_v1_fallback_disabled_failure);

static int s_imds_client_resource_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_expected_imds_resource_uri), s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_validate_uri_path_and_resource(2, false /*no resource*/) == 0);
    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());
    return 0;
}

AWS_TEST_CASE(imds_client_resource_request_failure, s_imds_client_resource_request_failure);

static int s_imds_client_resource_request_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_expected_imds_resource_uri), s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_validate_uri_path_and_resource(2, true /*got resource*/) == 0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&s_tester.resource), s_good_response);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(imds_client_resource_request_success, s_imds_client_resource_request_success);

static int s_imds_client_insecure_resource_request_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_403_FORBIDDEN;

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_expected_imds_resource_uri), s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_validate_uri_path_and_resource(2, true /*got resource*/) == 0);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_FALSE(s_tester.token_header_exist[1]);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&s_tester.resource), s_good_response);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(imds_client_insecure_resource_request_success, s_imds_client_insecure_resource_request_success);

static int s_imds_client_insecure_then_secure_resource_request_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.token_request_idx = 1;
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_401_UNAUTHORIZED;
    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .imds_version = IMDS_PROTOCOL_V1,
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_expected_imds_resource_uri), s_get_resource_callback, NULL);
    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_tester.has_received_resource_callback == true);
    ASSERT_TRUE(s_validate_uri_path_and_resource(3, true /*no creds*/) == 0);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[1]);
    ASSERT_FALSE(s_tester.token_header_exist[1]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_expected[2]);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&s_tester.resource), s_good_response);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(
    imds_client_insecure_then_secure_resource_request_success,
    s_imds_client_insecure_then_secure_resource_request_success);

static int s_aws_http_stream_get_multiple_incoming_response_status_mock(
    const struct aws_http_stream *stream,
    int *out_status_code) {

    (void)stream;
    /* randomly return 403/200 */
    uint32_t rand_output;
    int ret[2] = {AWS_HTTP_STATUS_CODE_200_OK, AWS_HTTP_STATUS_CODE_403_FORBIDDEN};
    aws_device_random_u32(&rand_output);
    *out_status_code = ret[rand_output % 2];
    return AWS_OP_SUCCESS;
}

static struct aws_http_stream *s_aws_http_connection_make_multiple_requests_mock(
    struct aws_http_connection *client_connection,
    const struct aws_http_make_request_options *options) {

    (void)client_connection;
    (void)options;

    struct aws_byte_cursor path;
    AWS_ZERO_STRUCT(path);
    aws_http_message_get_request_path(options->request, &path);

    if (aws_byte_cursor_eq_c_str_ignore_case(&path, "/latest/api/token")) {
        s_validate_token_ttl_header(options->request);
        s_invoke_mock_request_callbacks(options, &s_tester.response_data_callbacks[0]);
    } else {
        s_validate_token_header(options->request);
        s_invoke_mock_request_callbacks(options, &s_tester.response_data_callbacks[1]);
    }
    return (struct aws_http_stream *)1;
}

static bool s_has_tester_received_expected_resources(void *user_data) {
    return s_tester.successful_requests == (*(int *)user_data);
}

static void s_aws_wait_for_all_resources(int expected_resources_cnt) {
    aws_mutex_lock(&s_tester.lock);
    aws_condition_variable_wait_pred(
        &s_tester.signal, &s_tester.lock, s_has_tester_received_expected_resources, &expected_resources_cnt);
    aws_mutex_unlock(&s_tester.lock);
}

static void s_multiple_request_get_resource_callback(
    const struct aws_byte_buf *resource,
    int error_code,
    void *user_data) {
    (void)user_data;
    (void)error_code;
    aws_mutex_lock(&s_tester.lock);
    s_tester.has_received_resource_callback = true;
    if (resource && resource->len) {
        struct aws_byte_cursor cursor = aws_byte_cursor_from_buf(resource);
        aws_byte_buf_reset(&s_tester.resource, true);
        aws_byte_buf_append_dynamic(&s_tester.resource, &cursor);
        s_tester.successful_requests++;
    }
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_imds_client_multiple_resource_requests_random_responses_finally_all_success(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .imds_version = IMDS_PROTOCOL_V2,
    };

    options.function_table->aws_http_stream_get_incoming_response_status =
        s_aws_http_stream_get_multiple_incoming_response_status_mock;
    options.function_table->aws_http_connection_make_request = s_aws_http_connection_make_multiple_requests_mock;

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    for (size_t i = 0; i < 5000; i++) {
        aws_imds_client_get_resource_async(
            client,
            aws_byte_cursor_from_string(s_expected_imds_resource_uri),
            s_multiple_request_get_resource_callback,
            NULL);
    }
    s_aws_wait_for_all_resources(5000);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&s_tester.resource), s_good_response);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(
    imds_client_multiple_resource_requests_random_responses_finally_all_success,
    s_imds_client_multiple_resource_requests_random_responses_finally_all_success);

static int s_imds_client_real_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct aws_logger_standard_options logger_options = {
        .level = AWS_LOG_LEVEL_TRACE,
        .file = stderr,
    };

    struct aws_logger logger;
    ASSERT_SUCCESS(aws_logger_init_standard(&logger, allocator, &logger_options));
    aws_logger_set(&logger);

    s_aws_imds_tester_init(allocator);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = s_tester.el_group,
        .max_entries = 8,
    };
    struct aws_host_resolver *resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = s_tester.el_group,
        .host_resolver = resolver,
    };
    struct aws_client_bootstrap *bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_resource_async(
        client, aws_byte_cursor_from_string(s_ec2_metadata_root), s_get_resource_callback, NULL);
    s_aws_wait_for_resource_result();

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    aws_auth_library_clean_up();

    aws_logger_set(NULL);
    aws_logger_clean_up(&logger);

    return 0;
}

AWS_TEST_CASE(imds_client_real_success, s_imds_client_real_success);

AWS_STATIC_STRING_FROM_LITERAL(s_test_ami_id, "ami-5b70e32");
static int s_imds_client_get_ami_id_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_test_ami_id);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_ami_id(client, s_get_resource_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(
        s_validate_uri_path_and_excpected_resource(
            2, aws_byte_cursor_from_c_str("/latest/meta-data/ami-id"), good_response_cursor) == 0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&s_tester.resource), s_test_ami_id);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(imds_client_get_ami_id_success, s_imds_client_get_ami_id_success);

AWS_STATIC_STRING_FROM_LITERAL(s_test_ancestor_ami_ids, "ami-5b70e32\nami-5b70e33\nami-5b70e34");
AWS_STATIC_STRING_FROM_LITERAL(s_test_ancestor_ami_id1, "ami-5b70e32");
AWS_STATIC_STRING_FROM_LITERAL(s_test_ancestor_ami_id2, "ami-5b70e33");
AWS_STATIC_STRING_FROM_LITERAL(s_test_ancestor_ami_id3, "ami-5b70e34");
static struct aws_byte_cursor s_newline_cursor = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\n");

static int s_assert_get_ancestor_ami_ids(const struct aws_array_list *array) {
    s_tester.has_received_resource_callback = true;
    size_t len = aws_array_list_length(array);
    ASSERT_TRUE(len == 3);
    struct aws_byte_cursor cursor[3];
    for (size_t i = 0; i < len; i++) {
        aws_array_list_get_at(array, &cursor[i], i);
        aws_byte_buf_append_dynamic(&s_tester.resource, &cursor[i]);
        aws_byte_buf_append_dynamic(&s_tester.resource, &s_newline_cursor);
    }
    s_tester.resource.len--;
    ASSERT_CURSOR_VALUE_STRING_EQUALS(cursor[0], s_test_ancestor_ami_id1);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(cursor[1], s_test_ancestor_ami_id2);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(cursor[2], s_test_ancestor_ami_id3);

    if (array) {
        s_tester.successful_requests++;
    }
    return 0;
}
static void s_get_ancestor_ami_ids_callback(const struct aws_array_list *array, int error_code, void *user_data) {
    (void)user_data;
    (void)error_code;
    aws_mutex_lock(&s_tester.lock);
    s_assert_get_ancestor_ami_ids(array);
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_imds_client_get_ancestor_ami_ids_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_test_ancestor_ami_ids);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_ancestor_ami_ids(client, s_get_ancestor_ami_ids_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(
        s_validate_uri_path_and_excpected_resource(
            2, aws_byte_cursor_from_c_str("/latest/meta-data/ancestor-ami-ids"), good_response_cursor) == 0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&s_tester.resource), s_test_ancestor_ami_ids);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(imds_client_get_ancestor_ami_ids_success, s_imds_client_get_ancestor_ami_ids_success);

AWS_STATIC_STRING_FROM_LITERAL(
    s_iam_profile,
    "{\"LastUpdated\" : \"2020-06-03T20:42:19Z\", \n "
    "\"InstanceProfileArn\" : \"arn:aws:iam::030535792909:instance-profile/CloudWatchAgentServerRole\", \n "
    "\"InstanceProfileId\" : \"AIPAQOHATHEGTGNQ5THQB\"}");

AWS_STATIC_STRING_FROM_LITERAL(s_test_last_updated, "2020-06-03T20:42:19Z");
AWS_STATIC_STRING_FROM_LITERAL(
    s_test_profile_arn,
    "arn:aws:iam::030535792909:instance-profile/CloudWatchAgentServerRole");
AWS_STATIC_STRING_FROM_LITERAL(s_test_profile_id, "AIPAQOHATHEGTGNQ5THQB");

static int s_assert_get_iam_profile(const struct aws_imds_iam_profile *iam) {
    s_tester.has_received_resource_callback = true;

    ASSERT_CURSOR_VALUE_STRING_EQUALS(iam->instance_profile_arn, s_test_profile_arn);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(iam->instance_profile_id, s_test_profile_id);
    struct aws_byte_buf buf;
    aws_byte_buf_init(&buf, s_tester.allocator, 100);
    aws_date_time_to_utc_time_str(&iam->last_updated, AWS_DATE_FORMAT_ISO_8601, &buf);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&buf), s_test_last_updated);
    aws_byte_buf_clean_up(&buf);
    if (iam) {
        s_tester.successful_requests++;
    }
    return 0;
}

static void s_get_iam_profile_callback(const struct aws_imds_iam_profile *iam, int error_code, void *user_data) {
    (void)user_data;
    (void)error_code;
    aws_mutex_lock(&s_tester.lock);
    s_assert_get_iam_profile(iam);
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_imds_client_get_iam_profile_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_iam_profile);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_iam_profile(client, s_get_iam_profile_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_validate_uri_path(2, aws_byte_cursor_from_c_str("/latest/meta-data/iam/info")) == 0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(imds_client_get_iam_profile_success, s_imds_client_get_iam_profile_success);

AWS_STATIC_STRING_FROM_LITERAL(
    s_instance_info,
    "{\"accountId\" : \"030535792909\", \n"
    "\"architecture\" : \"x86_64\", \n"
    "\"availabilityZone\" : \"us-west-2a\", \n"
    "\"billingProducts\" : [\"1234\", \"abcd\"], \n"
    "\"devpayProductCodes\" : null, \n"
    "\"marketplaceProductCodes\" : null, \n"
    "\"imageId\" : \"ami-5b70e323\", \n"
    "\"instanceId\" : \"i-022a93b5e640c0248\", \n"
    "\"instanceType\" : \"c4.8xlarge\", \n"
    "\"kernelId\" : null, \n"
    "\"pendingTime\" : \"2020-05-27T08:41:17Z\", \n"
    "\"privateIp\" : \"172.31.22.164\", \n"
    "\"ramdiskId\" : null, \n"
    "\"region\" : \"us-west-2\", \n"
    "\"version\" : \"2017-09-30\" \n}");

AWS_STATIC_STRING_FROM_LITERAL(s_account_id, "030535792909");
AWS_STATIC_STRING_FROM_LITERAL(s_architecture, "x86_64");
AWS_STATIC_STRING_FROM_LITERAL(s_availability_zone, "us-west-2a");
AWS_STATIC_STRING_FROM_LITERAL(s_image_id, "ami-5b70e323");
AWS_STATIC_STRING_FROM_LITERAL(s_instance_id, "i-022a93b5e640c0248");
AWS_STATIC_STRING_FROM_LITERAL(s_instance_type, "c4.8xlarge");
AWS_STATIC_STRING_FROM_LITERAL(s_pending_time, "2020-05-27T08:41:17Z");
AWS_STATIC_STRING_FROM_LITERAL(s_private_ip, "172.31.22.164");
AWS_STATIC_STRING_FROM_LITERAL(s_region, "us-west-2");
AWS_STATIC_STRING_FROM_LITERAL(s_version, "2017-09-30");
AWS_STATIC_STRING_FROM_LITERAL(s_billing_product1, "1234");
AWS_STATIC_STRING_FROM_LITERAL(s_billing_product2, "abcd");

static int s_assert_get_instance_info(const struct aws_imds_instance_info *instance) {
    s_tester.has_received_resource_callback = true;

    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->account_id, s_account_id);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->architecture, s_architecture);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->availability_zone, s_availability_zone);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->image_id, s_image_id);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->instance_id, s_instance_id);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->instance_type, s_instance_type);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->private_ip, s_private_ip);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->region, s_region);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->version, s_version);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(instance->availability_zone, s_availability_zone);

    ASSERT_TRUE(aws_array_list_length(&instance->billing_products) == 2);
    ASSERT_TRUE(aws_array_list_length(&instance->marketplace_product_codes) == 0);

    struct aws_byte_cursor cursor[2];
    for (size_t i = 0; i < 2; i++) {
        aws_array_list_get_at(&instance->billing_products, &cursor[i], i);
    }
    ASSERT_CURSOR_VALUE_STRING_EQUALS(cursor[0], s_billing_product1);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(cursor[1], s_billing_product2);

    struct aws_byte_buf buf;
    aws_byte_buf_init(&buf, s_tester.allocator, 100);
    aws_date_time_to_utc_time_str(&instance->pending_time, AWS_DATE_FORMAT_ISO_8601, &buf);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&buf), s_pending_time);
    aws_byte_buf_clean_up(&buf);
    if (instance) {
        s_tester.successful_requests++;
    }
    return 0;
}

static void s_get_instance_info_callback(
    const struct aws_imds_instance_info *instance,
    int error_code,
    void *user_data) {
    (void)user_data;
    (void)error_code;
    aws_mutex_lock(&s_tester.lock);
    s_assert_get_instance_info(instance);
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_imds_client_get_instance_info_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_instance_info);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_instance_info(client, s_get_instance_info_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(s_validate_uri_path(2, aws_byte_cursor_from_c_str("/latest/dynamic/instance-identity/document")) == 0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(imds_client_get_instance_info_success, s_imds_client_get_instance_info_success);

static int s_assert_get_credentials_info(const struct aws_credentials *creds) {
    s_tester.has_received_resource_callback = true;

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(creds), s_access_key);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_secret_access_key(creds), s_secret_key);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(creds), s_token);

    struct aws_byte_buf buf;
    aws_byte_buf_init(&buf, s_tester.allocator, 100);
    struct aws_date_time date;
    aws_date_time_init_epoch_secs(&date, aws_credentials_get_expiration_timepoint_seconds(creds));
    aws_date_time_to_utc_time_str(&date, AWS_DATE_FORMAT_ISO_8601, &buf);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_byte_cursor_from_buf(&buf), s_expiration);
    aws_byte_buf_clean_up(&buf);
    if (creds) {
        s_tester.successful_requests++;
    }
    return 0;
}

static void s_get_credentails_callback(const struct aws_credentials *creds, int error_code, void *user_data) {
    (void)user_data;
    (void)error_code;
    aws_mutex_lock(&s_tester.lock);
    s_assert_get_credentials_info(creds);
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_imds_client_get_credentials_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    aws_imds_client_get_credentials(client, aws_byte_cursor_from_c_str("test_role"), s_get_credentails_callback, NULL);

    s_aws_wait_for_resource_result();

    ASSERT_TRUE(
        s_validate_uri_path(2, aws_byte_cursor_from_c_str("/latest/meta-data/iam/security-credentials/test_role")) ==
        0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(imds_client_get_credentials_success, s_imds_client_get_credentials_success);

static int s_imds_client_cache_token_refresh(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    s_tester.timestamp = 0;

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);

    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &good_response_cursor);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor);
    aws_array_list_push_back(&s_tester.response_data_callbacks[3], &test_token_cursor);
    aws_array_list_push_back(&s_tester.response_data_callbacks[4], &good_response_cursor);

    struct aws_imds_client_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_imds_client *client = aws_imds_client_new(allocator, &options);
    /* 1. request a resource */
    aws_imds_client_get_credentials(client, aws_byte_cursor_from_c_str("test_role"), s_get_credentails_callback, NULL);
    s_aws_wait_for_resource_result();
    /* Currently have 2 requests, one to get the token, the other to fetch resource */
    ASSERT_UINT_EQUALS(2, s_tester.current_request);
    ASSERT_UINT_EQUALS(AWS_ERROR_SUCCESS, s_tester.error_code);

    /* 2. Request another resource without change the timestamp. So that the cached token should be used */
    s_tester.has_received_resource_callback = false;
    aws_imds_client_get_credentials(client, aws_byte_cursor_from_c_str("test_role"), s_get_credentails_callback, NULL);
    s_aws_wait_for_resource_result();
    /* Currently have 3 requests, only one more to fetch the resource as the cached token being used. */
    ASSERT_UINT_EQUALS(3, s_tester.current_request);
    ASSERT_UINT_EQUALS(AWS_ERROR_SUCCESS, s_tester.error_code);

    /* 3. Update the time to expired time. Request another resource without change the timestamp. So that the cached
     * token should be expired, and will fetch another token. */
    s_tester.has_received_resource_callback = false;
    s_tester.timestamp = aws_timestamp_convert(21600, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);
    aws_imds_client_get_credentials(client, aws_byte_cursor_from_c_str("test_role"), s_get_credentails_callback, NULL);
    s_aws_wait_for_resource_result();
    /* Currently have 3 requests, only one more to fetch the resource as the cached token being used. */
    ASSERT_UINT_EQUALS(5, s_tester.current_request);
    ASSERT_UINT_EQUALS(AWS_ERROR_SUCCESS, s_tester.error_code);
    aws_imds_client_release(client);

    s_aws_wait_for_imds_client_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(allocator, client);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}
AWS_TEST_CASE(imds_client_cache_token_refresh, s_imds_client_cache_token_refresh);
