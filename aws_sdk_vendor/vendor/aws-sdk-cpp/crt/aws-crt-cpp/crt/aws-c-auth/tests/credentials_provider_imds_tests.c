/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/credentials_utils.h>
#include <aws/common/clock.h>
#include <aws/common/condition_variable.h>
#include <aws/common/string.h>
#include <aws/common/thread.h>
#include <aws/http/request_response.h>
#include <aws/http/status_code.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/logging.h>
#include <aws/io/socket.h>

/**
 * The max requests SDK could make to IMDS V2 should be 4.
 * 1. query_role_name -> Get 401, unauthorized, then switch to secure way.
 * 2. query_token
 * 3. query role name
 * 4. query role
 *
 * By default, the requests made is 3.
 * 1. query_token (unless gets 400, no matter succeed or not, next step is query role name w/o token)
 * 2. query role name.
 * 3. query role
 *
 * Well, IMDS could act crazy then client would keep switching between secure and insecure way.
 * We will not handle this extreme case.
 */
#define IMDS_MAX_REQUESTS (8)
struct aws_mock_imds_tester {
    struct aws_allocator *allocator;
    struct aws_byte_buf request_uris[IMDS_MAX_REQUESTS];
    struct aws_array_list response_data_callbacks[IMDS_MAX_REQUESTS];

    int current_request;
    int response_code[IMDS_MAX_REQUESTS];
    int token_request_idx;

    bool is_connection_acquire_successful;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    struct aws_credentials *credentials;
    struct aws_event_loop_group *el_group;
    struct aws_client_bootstrap *bootstrap;
    bool has_received_credentials_callback;
    bool has_received_shutdown_callback;

    int error_code;

    bool token_ttl_header_exist[IMDS_MAX_REQUESTS];
    bool token_ttl_header_expected[IMDS_MAX_REQUESTS];
    bool token_header_exist[IMDS_MAX_REQUESTS];
    bool token_header_expected[IMDS_MAX_REQUESTS];
    bool alternate_closed_connections;
};

static struct aws_mock_imds_tester s_tester;

struct aws_credentials_provider_imds_impl {
    struct aws_imds_client *client;
};

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

static void s_aws_wait_for_provider_shutdown_callback(void) {
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
    return aws_high_res_clock_get_ticks(timestamp);
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

    for (size_t i = 0; i < IMDS_MAX_REQUESTS; i++) {
        if (aws_array_list_init_dynamic(
                &s_tester.response_data_callbacks[i], allocator, 10, sizeof(struct aws_byte_cursor))) {
            return AWS_OP_ERR;
        }
        if (aws_byte_buf_init(&s_tester.request_uris[i], allocator, 100)) {
            return AWS_OP_ERR;
        }
    }

    s_tester.allocator = allocator;
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

    return AWS_OP_SUCCESS;
}

static int s_aws_imds_tester_cleanup(void) {
    for (size_t i = 0; i < IMDS_MAX_REQUESTS; i++) {
        aws_array_list_clean_up(&s_tester.response_data_callbacks[i]);
        aws_byte_buf_clean_up(&s_tester.request_uris[i]);
    }

    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);

    aws_credentials_release(s_tester.credentials);

    aws_client_bootstrap_release(s_tester.bootstrap);
    aws_event_loop_group_release(s_tester.el_group);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

static bool s_has_tester_received_credentials_callback(void *user_data) {
    (void)user_data;

    return s_tester.has_received_credentials_callback;
}

static void s_aws_wait_for_credentials_result(void) {
    aws_mutex_lock(&s_tester.lock);
    aws_condition_variable_wait_pred(
        &s_tester.signal, &s_tester.lock, s_has_tester_received_credentials_callback, NULL);
    aws_mutex_unlock(&s_tester.lock);
}

static void s_get_credentials_callback(struct aws_credentials *credentials, int error_code, void *user_data) {
    (void)user_data;

    aws_mutex_lock(&s_tester.lock);
    s_tester.has_received_credentials_callback = true;
    s_tester.error_code = error_code;
    s_tester.credentials = credentials;
    aws_credentials_acquire(credentials);
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_credentials_provider_imds_new_destroy(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);
    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_new_destroy, s_credentials_provider_imds_new_destroy);

static int s_credentials_provider_imds_connect_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.is_connection_acquire_successful = false;

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials == NULL);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_connect_failure, s_credentials_provider_imds_connect_failure);

AWS_STATIC_STRING_FROM_LITERAL(s_expected_imds_token_uri, "/latest/api/token");
AWS_STATIC_STRING_FROM_LITERAL(s_expected_imds_base_uri, "/latest/meta-data/iam/security-credentials/");
AWS_STATIC_STRING_FROM_LITERAL(s_expected_imds_role_uri, "/latest/meta-data/iam/security-credentials/test-role");
AWS_STATIC_STRING_FROM_LITERAL(s_test_role_response, "test-role");
AWS_STATIC_STRING_FROM_LITERAL(s_test_imds_token, "A00XXF3H00ZZ==");

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

static int s_validate_uri_path_and_creds(int expected_requests, bool get_credentials) {

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
            s_expected_imds_base_uri->bytes,
            s_expected_imds_base_uri->len);
    }
    idx++;
    if (s_tester.current_request >= 3) {
        ASSERT_BIN_ARRAYS_EQUALS(
            s_tester.request_uris[idx].buffer,
            s_tester.request_uris[idx].len,
            s_expected_imds_role_uri->bytes,
            s_expected_imds_role_uri->len);
    }

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);

    if (get_credentials) {
        ASSERT_TRUE(s_tester.credentials != NULL);
    } else {
        ASSERT_TRUE(s_tester.credentials == NULL);
    }

    return 0;
}

static int s_credentials_provider_imds_token_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_400_BAD_REQUEST;
    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_validate_uri_path_and_creds(1, false /*no creds*/) == 0);
    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_token_request_failure, s_credentials_provider_imds_token_request_failure);

static int s_credentials_provider_imds_role_name_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_validate_uri_path_and_creds(2, false /*no creds*/) == 0);
    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_imds_role_name_request_failure,
    s_credentials_provider_imds_role_name_request_failure);

static int s_credentials_provider_imds_role_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor test_role_cursor = aws_byte_cursor_from_string(s_test_role_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_role_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_validate_uri_path_and_creds(3, false /*no creds*/) == 0);
    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_expected[2]);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_role_request_failure, s_credentials_provider_imds_role_request_failure);

AWS_STATIC_STRING_FROM_LITERAL(s_bad_document_response, "{\"NotTheExpectedDocumentFormat\":\"Error\"}");

static int s_credentials_provider_imds_bad_document_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor test_role_cursor = aws_byte_cursor_from_string(s_test_role_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_role_cursor);

    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_bad_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &bad_document_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_validate_uri_path_and_creds(3, false /*no creds*/) == 0);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_bad_document_failure, s_credentials_provider_imds_bad_document_failure);

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response,
    "{\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"SecretAccessKey\":\"SuccessfulSecret\", \n  "
    "\"Token\":\"TokenSuccess\", \n \"Expiration\":\"2020-02-25T06:03:31Z\"}");
AWS_STATIC_STRING_FROM_LITERAL(s_good_access_key_id, "SuccessfulAccessKey");
AWS_STATIC_STRING_FROM_LITERAL(s_good_secret_access_key, "SuccessfulSecret");
AWS_STATIC_STRING_FROM_LITERAL(s_good_session_token, "TokenSuccess");

static int s_verify_credentials(struct aws_credentials *credentials) {
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(credentials), s_good_access_key_id);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_secret_access_key(credentials), s_good_secret_access_key);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(credentials), s_good_session_token);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_imds_secure_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor test_role_cursor = aws_byte_cursor_from_string(s_test_role_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_role_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_validate_uri_path_and_creds(3, true /*got creds*/) == 0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_exist[1]);
    ASSERT_TRUE(s_tester.token_header_expected[1]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_expected[2]);

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_secure_success, s_credentials_provider_imds_secure_success);

static int s_credentials_provider_imds_connection_closed_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.alternate_closed_connections = true;

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);
    /* this one will fail, replay the body. */
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_token_cursor);

    struct aws_byte_cursor test_role_cursor = aws_byte_cursor_from_string(s_test_role_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &test_role_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[3], &good_response_cursor);
    /* this one will fail replay the body */
    aws_array_list_push_back(&s_tester.response_data_callbacks[4], &good_response_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_imds_connection_closed_success,
    s_credentials_provider_imds_connection_closed_success);

static int s_credentials_provider_imds_insecure_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_403_FORBIDDEN;

    struct aws_byte_cursor test_role_cursor = aws_byte_cursor_from_string(s_test_role_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_role_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_validate_uri_path_and_creds(3, true /*no creds*/) == 0);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[0]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[1]);
    ASSERT_FALSE(s_tester.token_header_exist[1]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[2]);
    ASSERT_FALSE(s_tester.token_header_exist[2]);

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_insecure_success, s_credentials_provider_imds_insecure_success);

static int s_credentials_provider_imds_insecure_then_secure_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);
    s_tester.token_request_idx = 1;
    s_tester.response_code[0] = AWS_HTTP_STATUS_CODE_401_UNAUTHORIZED;

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_token_cursor);

    struct aws_byte_cursor test_role_cursor = aws_byte_cursor_from_string(s_test_role_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &test_role_cursor);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[3], &good_response_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .imds_version = IMDS_PROTOCOL_V1,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();
    ASSERT_TRUE(s_validate_uri_path_and_creds(4, true /*no creds*/) == 0);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[0]);
    ASSERT_FALSE(s_tester.token_header_exist[0]);

    ASSERT_TRUE(s_tester.token_ttl_header_exist[1]);
    ASSERT_TRUE(s_tester.token_ttl_header_expected[1]);
    ASSERT_FALSE(s_tester.token_header_exist[1]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_exist[2]);
    ASSERT_TRUE(s_tester.token_header_expected[2]);

    ASSERT_FALSE(s_tester.token_ttl_header_exist[3]);
    ASSERT_TRUE(s_tester.token_header_exist[3]);
    ASSERT_TRUE(s_tester.token_header_expected[3]);

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_imds_insecure_then_secure_success,
    s_credentials_provider_imds_insecure_then_secure_success);

AWS_STATIC_STRING_FROM_LITERAL(s_test_role_response_first_half, "test-");
AWS_STATIC_STRING_FROM_LITERAL(s_test_role_response_second_half, "role");

static int s_credentials_provider_imds_success_multi_part_role_name(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor test_role_cursor1 = aws_byte_cursor_from_string(s_test_role_response_first_half);
    struct aws_byte_cursor test_role_cursor2 = aws_byte_cursor_from_string(s_test_role_response_second_half);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_role_cursor1);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_role_cursor2);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    s_validate_uri_path_and_creds(3, true);

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_imds_success_multi_part_role_name,
    s_credentials_provider_imds_success_multi_part_role_name);

AWS_STATIC_STRING_FROM_LITERAL(s_good_response_first_part, "{\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"Secret");
AWS_STATIC_STRING_FROM_LITERAL(s_good_response_second_part, "AccessKey\":\"SuccessfulSecr");
AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response_third_part,
    "et\", \n  \"Token\":\"TokenSuccess\"\n, \"Expiration\":\"2020-02-25T06:03:31Z\"}");

static int s_credentials_provider_imds_success_multi_part_doc(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_imds_tester_init(allocator);

    struct aws_byte_cursor test_token_cursor = aws_byte_cursor_from_string(s_test_imds_token);
    aws_array_list_push_back(&s_tester.response_data_callbacks[0], &test_token_cursor);

    struct aws_byte_cursor test_role_cursor = aws_byte_cursor_from_string(s_test_role_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks[1], &test_role_cursor);

    struct aws_byte_cursor good_response_cursor1 = aws_byte_cursor_from_string(s_good_response_first_part);
    struct aws_byte_cursor good_response_cursor2 = aws_byte_cursor_from_string(s_good_response_second_part);
    struct aws_byte_cursor good_response_cursor3 = aws_byte_cursor_from_string(s_good_response_third_part);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor1);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor2);
    aws_array_list_push_back(&s_tester.response_data_callbacks[2], &good_response_cursor3);

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    s_validate_uri_path_and_creds(3, true);

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    struct aws_credentials_provider_imds_impl *impl = provider->impl;
    aws_mem_release(provider->allocator, impl->client);
    aws_mem_release(provider->allocator, provider);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_success_multi_part_doc, s_credentials_provider_imds_success_multi_part_doc);

static int s_credentials_provider_imds_real_new_destroy(struct aws_allocator *allocator, void *ctx) {
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

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = bootstrap,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    aws_auth_library_clean_up();

    aws_logger_set(NULL);
    aws_logger_clean_up(&logger);

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_real_new_destroy, s_credentials_provider_imds_real_new_destroy);

static int s_credentials_provider_imds_real_success(struct aws_allocator *allocator, void *ctx) {
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

    struct aws_credentials_provider_imds_options options = {
        .bootstrap = bootstrap,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_imds(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_tester.credentials != NULL);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);

    ASSERT_SUCCESS(s_aws_imds_tester_cleanup());

    aws_auth_library_clean_up();

    aws_logger_set(NULL);
    aws_logger_clean_up(&logger);

    return 0;
}

AWS_TEST_CASE(credentials_provider_imds_real_success, s_credentials_provider_imds_real_success);
