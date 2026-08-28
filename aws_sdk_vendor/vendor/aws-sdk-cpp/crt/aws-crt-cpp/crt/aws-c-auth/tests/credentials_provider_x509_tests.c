/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/credentials_utils.h>
#include <aws/common/clock.h>
#include <aws/common/condition_variable.h>
#include <aws/common/date_time.h>
#include <aws/common/string.h>
#include <aws/common/thread.h>
#include <aws/http/request_response.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/logging.h>
#include <aws/io/socket.h>
#include <aws/io/tls_channel_handler.h>

struct aws_mock_x509_tester {
    struct aws_byte_buf request_uri;

    struct aws_array_list response_data_callbacks;
    bool is_connection_acquire_successful;
    bool is_request_successful;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    struct aws_credentials *credentials;
    bool has_received_credentials_callback;
    bool has_received_shutdown_callback;
    int error_code;

    struct aws_tls_ctx *ctx;
    struct aws_tls_connection_options tls_connection_options;
};

static struct aws_mock_x509_tester s_tester;

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
    (void)callback;
    (void)user_data;

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
    struct aws_array_list *data_callbacks,
    bool is_request_successful) {

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

    options->on_complete(
        (struct aws_http_stream *)1,
        is_request_successful ? AWS_ERROR_SUCCESS : AWS_ERROR_HTTP_UNKNOWN,
        options->user_data);
}

static struct aws_http_stream *s_aws_http_connection_make_request_mock(
    struct aws_http_connection *client_connection,
    const struct aws_http_make_request_options *options) {

    (void)client_connection;
    (void)options;

    struct aws_byte_cursor path;
    AWS_ZERO_STRUCT(path);
    aws_http_message_get_request_path(options->request, &path);

    aws_byte_buf_append_dynamic(&s_tester.request_uri, &path);
    s_invoke_mock_request_callbacks(options, &s_tester.response_data_callbacks, s_tester.is_request_successful);

    return (struct aws_http_stream *)1;
}

static int s_aws_http_stream_activate_mock(struct aws_http_stream *stream) {
    (void)stream;
    return AWS_OP_SUCCESS;
}

static int s_aws_http_stream_get_incoming_response_status_mock(
    const struct aws_http_stream *stream,
    int *out_status_code) {
    (void)stream;

    *out_status_code = 200;

    return AWS_OP_SUCCESS;
}

static void s_aws_http_stream_release_mock(struct aws_http_stream *stream) {
    (void)stream;
}

static void s_aws_http_connection_close_mock(struct aws_http_connection *connection) {
    (void)connection;
}

static struct aws_auth_http_system_vtable s_mock_function_table = {
    .aws_http_connection_manager_new = s_aws_http_connection_manager_new_mock,
    .aws_http_connection_manager_release = s_aws_http_connection_manager_release_mock,
    .aws_http_connection_manager_acquire_connection = s_aws_http_connection_manager_acquire_connection_mock,
    .aws_http_connection_manager_release_connection = s_aws_http_connection_manager_release_connection_mock,
    .aws_http_connection_make_request = s_aws_http_connection_make_request_mock,
    .aws_http_stream_activate = s_aws_http_stream_activate_mock,
    .aws_http_stream_get_incoming_response_status = s_aws_http_stream_get_incoming_response_status_mock,
    .aws_http_stream_release = s_aws_http_stream_release_mock,
    .aws_http_connection_close = s_aws_http_connection_close_mock};

static int s_aws_x509_tester_init(struct aws_allocator *allocator) {
    aws_auth_library_init(allocator);

    if (aws_array_list_init_dynamic(&s_tester.response_data_callbacks, allocator, 10, sizeof(struct aws_byte_cursor))) {
        return AWS_OP_ERR;
    }

    if (aws_byte_buf_init(&s_tester.request_uri, allocator, 100)) {
        return AWS_OP_ERR;
    }

    if (aws_mutex_init(&s_tester.lock)) {
        return AWS_OP_ERR;
    }

    if (aws_condition_variable_init(&s_tester.signal)) {
        return AWS_OP_ERR;
    }

    AWS_ZERO_STRUCT(s_tester.tls_connection_options);
    struct aws_tls_ctx_options tls_options;
    aws_tls_ctx_options_init_default_client(&tls_options, allocator);
    s_tester.ctx = aws_tls_client_ctx_new(allocator, &tls_options);
    aws_tls_ctx_options_clean_up(&tls_options);
    aws_tls_connection_options_init_from_ctx(&s_tester.tls_connection_options, s_tester.ctx);

    /* default to everything successful */
    s_tester.is_connection_acquire_successful = true;
    s_tester.is_request_successful = true;

    return AWS_OP_SUCCESS;
}

static void s_aws_x509_tester_cleanup(void) {
    aws_array_list_clean_up(&s_tester.response_data_callbacks);
    aws_byte_buf_clean_up(&s_tester.request_uri);
    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);
    aws_credentials_release(s_tester.credentials);
    aws_tls_ctx_release(s_tester.ctx);
    s_tester.ctx = NULL;

    aws_tls_connection_options_clean_up(&s_tester.tls_connection_options);
    aws_auth_library_clean_up();
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

static int s_credentials_provider_x509_new_destroy(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_x509_tester_init(allocator);

    struct aws_credentials_provider_x509_options options = {
        .bootstrap = NULL,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .endpoint = aws_byte_cursor_from_c_str("c2sakl5huz0afv.credentials.iot.us-east-1.amazonaws.com"),
        .thing_name = aws_byte_cursor_from_c_str("my_iot_thing_name"),
        .role_alias = aws_byte_cursor_from_c_str("my_test_role_alias"),
        .tls_connection_options = &s_tester.tls_connection_options,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_x509(allocator, &options);
    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_x509_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_x509_new_destroy, s_credentials_provider_x509_new_destroy);

static int s_credentials_provider_x509_connect_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_x509_tester_init(allocator);
    s_tester.is_connection_acquire_successful = false;

    struct aws_credentials_provider_x509_options options = {
        .bootstrap = NULL,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .endpoint = aws_byte_cursor_from_c_str("c2sakl5huz0afv.credentials.iot.us-east-1.amazonaws.com"),
        .thing_name = aws_byte_cursor_from_c_str("my_iot_thing_name"),
        .role_alias = aws_byte_cursor_from_c_str("my_test_role_alias"),
        .tls_connection_options = &s_tester.tls_connection_options,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_x509(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials == NULL);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_x509_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_x509_connect_failure, s_credentials_provider_x509_connect_failure);

AWS_STATIC_STRING_FROM_LITERAL(s_expected_x509_role_alias_path, "/role-aliases/my_test_role_alias/credentials");

static int s_credentials_provider_x509_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_x509_tester_init(allocator);
    s_tester.is_request_successful = false;

    struct aws_credentials_provider_x509_options options = {
        .bootstrap = NULL,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .endpoint = aws_byte_cursor_from_c_str("c2sakl5huz0afv.credentials.iot.us-east-1.amazonaws.com"),
        .thing_name = aws_byte_cursor_from_c_str("my_iot_thing_name"),
        .role_alias = aws_byte_cursor_from_c_str("my_test_role_alias"),
        .tls_connection_options = &s_tester.tls_connection_options,
    };
    struct aws_credentials_provider *provider = aws_credentials_provider_new_x509(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_BIN_ARRAYS_EQUALS(
        s_tester.request_uri.buffer,
        s_tester.request_uri.len,
        s_expected_x509_role_alias_path->bytes,
        s_expected_x509_role_alias_path->len);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials == NULL);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_x509_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_x509_request_failure, s_credentials_provider_x509_request_failure);

AWS_STATIC_STRING_FROM_LITERAL(s_bad_document_response, "{\"NotTheExpectedDocumentFormat\":\"Error\"}");

static int s_credentials_provider_x509_bad_document_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_x509_tester_init(allocator);

    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_bad_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &bad_document_cursor);

    struct aws_credentials_provider_x509_options options = {
        .bootstrap = NULL,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .endpoint = aws_byte_cursor_from_c_str("c2sakl5huz0afv.credentials.iot.us-east-1.amazonaws.com"),
        .thing_name = aws_byte_cursor_from_c_str("my_iot_thing_name"),
        .role_alias = aws_byte_cursor_from_c_str("my_test_role_alias"),
        .tls_connection_options = &s_tester.tls_connection_options,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_x509(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_BIN_ARRAYS_EQUALS(
        s_tester.request_uri.buffer,
        s_tester.request_uri.len,
        s_expected_x509_role_alias_path->bytes,
        s_expected_x509_role_alias_path->len);

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials == NULL);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_x509_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_x509_bad_document_failure, s_credentials_provider_x509_bad_document_failure);

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response,
    "{\"Credentials\": {\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"SecretAccessKey\":\"SuccessfulSecret\", \n  "
    "\"SessionToken\":\"TokenSuccess\", \n \"Expiration\":\"2020-02-25T06:03:31Z\"}}");
AWS_STATIC_STRING_FROM_LITERAL(s_good_access_key_id, "SuccessfulAccessKey");
AWS_STATIC_STRING_FROM_LITERAL(s_good_secret_access_key, "SuccessfulSecret");
AWS_STATIC_STRING_FROM_LITERAL(s_good_session_token, "TokenSuccess");
AWS_STATIC_STRING_FROM_LITERAL(s_good_response_expiration, "2020-02-25T06:03:31Z");

static int s_credentials_provider_x509_basic_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_x509_tester_init(allocator);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);

    struct aws_credentials_provider_x509_options options = {
        .bootstrap = NULL,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .endpoint = aws_byte_cursor_from_c_str("c2sakl5huz0afv.credentials.iot.us-east-1.amazonaws.com"),
        .thing_name = aws_byte_cursor_from_c_str("my_iot_thing_name"),
        .role_alias = aws_byte_cursor_from_c_str("my_test_role_alias"),
        .tls_connection_options = &s_tester.tls_connection_options,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_x509(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_BIN_ARRAYS_EQUALS(
        s_tester.request_uri.buffer,
        s_tester.request_uri.len,
        s_expected_x509_role_alias_path->bytes,
        s_expected_x509_role_alias_path->len);

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(s_tester.credentials), s_good_access_key_id);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(s_tester.credentials), s_good_secret_access_key);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(s_tester.credentials), s_good_session_token);

    struct aws_date_time expiration;
    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_good_response_expiration);
    aws_date_time_init_from_str_cursor(&expiration, &date_cursor, AWS_DATE_FORMAT_ISO_8601);
    ASSERT_TRUE(
        aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == (uint64_t)expiration.timestamp);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_x509_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_x509_basic_success, s_credentials_provider_x509_basic_success);

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response_first_part,
    "{\"Credentials\": {\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"Secret");
AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response_second_part,
    "AccessKey\":\"SuccessfulSecret\", \n  \"SessionToken\":\"Token");
AWS_STATIC_STRING_FROM_LITERAL(s_good_response_third_part, "Success\", \n \"Expiration\":\"2020-02-25T06:03:31Z\"}}");

static int s_credentials_provider_x509_success_multi_part_doc(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_x509_tester_init(allocator);

    struct aws_byte_cursor good_response_cursor1 = aws_byte_cursor_from_string(s_good_response_first_part);
    struct aws_byte_cursor good_response_cursor2 = aws_byte_cursor_from_string(s_good_response_second_part);
    struct aws_byte_cursor good_response_cursor3 = aws_byte_cursor_from_string(s_good_response_third_part);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor1);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor2);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor3);

    struct aws_credentials_provider_x509_options options = {
        .bootstrap = NULL,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .endpoint = aws_byte_cursor_from_c_str("c2sakl5huz0afv.credentials.iot.us-east-1.amazonaws.com"),
        .thing_name = aws_byte_cursor_from_c_str("my_iot_thing_name"),
        .role_alias = aws_byte_cursor_from_c_str("my_test_role_alias"),
        .tls_connection_options = &s_tester.tls_connection_options,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_x509(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_BIN_ARRAYS_EQUALS(
        s_tester.request_uri.buffer,
        s_tester.request_uri.len,
        s_expected_x509_role_alias_path->bytes,
        s_expected_x509_role_alias_path->len);

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(s_tester.credentials), s_good_access_key_id);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(s_tester.credentials), s_good_secret_access_key);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(s_tester.credentials), s_good_session_token);

    struct aws_date_time expiration;
    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_good_response_expiration);
    aws_date_time_init_from_str_cursor(&expiration, &date_cursor, AWS_DATE_FORMAT_ISO_8601);
    ASSERT_TRUE(
        aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == (uint64_t)expiration.timestamp);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_x509_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_x509_success_multi_part_doc, s_credentials_provider_x509_success_multi_part_doc);

static int s_credentials_provider_x509_real_new_destroy(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct aws_logger_standard_options logger_options = {
        .level = AWS_LOG_LEVEL_TRACE,
        .file = stderr,
    };

    struct aws_logger logger;
    ASSERT_SUCCESS(aws_logger_init_standard(&logger, allocator, &logger_options));
    aws_logger_set(&logger);

    s_aws_x509_tester_init(allocator);

    struct aws_event_loop_group *el_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = el_group,
        .max_entries = 8,
    };
    struct aws_host_resolver *resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = el_group,
        .host_resolver = resolver,
    };
    struct aws_client_bootstrap *bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    struct aws_credentials_provider_x509_options options = {
        .bootstrap = bootstrap,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .endpoint = aws_byte_cursor_from_c_str("c2sakl5huz0afv.credentials.iot.us-east-1.amazonaws.com"),
        .thing_name = aws_byte_cursor_from_c_str("my_iot_thing_name"),
        .role_alias = aws_byte_cursor_from_c_str("my_test_role_alias"),
        .tls_connection_options = &s_tester.tls_connection_options,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_x509(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);
    aws_event_loop_group_release(el_group);

    s_aws_x509_tester_cleanup();

    aws_auth_library_clean_up();

    aws_logger_set(NULL);
    aws_logger_clean_up(&logger);

    return 0;
}

AWS_TEST_CASE(credentials_provider_x509_real_new_destroy, s_credentials_provider_x509_real_new_destroy);
