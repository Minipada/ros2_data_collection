/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include "shared_credentials_test_definitions.h"

#include <aws/auth/credentials.h>
#include <aws/auth/private/credentials_utils.h>
#include <aws/common/clock.h>
#include <aws/common/condition_variable.h>
#include <aws/common/date_time.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>
#include <aws/common/thread.h>
#include <aws/common/uri.h>
#include <aws/http/request_response.h>
#include <aws/http/status_code.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/logging.h>
#include <aws/io/socket.h>
#include <aws/io/tls_channel_handler.h>

AWS_STATIC_STRING_FROM_LITERAL(s_ecs_creds_env_relative_uri, "AWS_CONTAINER_CREDENTIALS_RELATIVE_URI");
AWS_STATIC_STRING_FROM_LITERAL(s_ecs_creds_env_full_uri, "AWS_CONTAINER_CREDENTIALS_FULL_URI");
AWS_STATIC_STRING_FROM_LITERAL(s_ecs_creds_env_token_file, "AWS_CONTAINER_AUTHORIZATION_TOKEN_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_ecs_creds_env_token, "AWS_CONTAINER_AUTHORIZATION_TOKEN");

struct aws_mock_ecs_tester {
    struct aws_allocator *allocator;

    struct aws_string *request_path_and_query;
    struct aws_string *request_authorization_header;
    struct aws_string *selected_host;
    int attempts;

    struct aws_array_list response_data_callbacks;
    bool is_connection_acquire_successful;
    bool is_request_successful;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    struct aws_credentials *credentials;
    bool has_received_credentials_callback;
    bool has_received_shutdown_callback;
    bool selected_tls;
    uint32_t selected_port;

    int error_code;

    struct aws_event_loop_group *el_group;
    struct aws_host_resolver *host_resolver;
    struct aws_client_bootstrap *bootstrap;
    struct aws_tls_ctx *tls_ctx;
};

static struct aws_mock_ecs_tester s_tester;

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

    aws_mutex_lock(&s_tester.lock);
    s_tester.selected_host = aws_string_new_from_cursor(allocator, &options->host);
    s_tester.selected_port = options->port;
    s_tester.selected_tls = options->tls_connection_options != NULL;
    aws_mutex_unlock(&s_tester.lock);

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

    s_tester.attempts++;
    struct aws_byte_cursor path;
    AWS_ZERO_STRUCT(path);
    aws_http_message_get_request_path(options->request, &path);
    if (s_tester.request_path_and_query != NULL) {
        aws_string_destroy(s_tester.request_path_and_query);
        s_tester.request_path_and_query = NULL;
    }
    s_tester.request_path_and_query = aws_string_new_from_cursor(s_tester.allocator, &path);
    struct aws_byte_cursor authorization_header_value;
    AWS_ZERO_STRUCT(authorization_header_value);
    if (aws_http_headers_get(
            aws_http_message_get_headers(options->request),
            aws_byte_cursor_from_c_str("Authorization"),
            &authorization_header_value) == AWS_OP_SUCCESS) {
        if (s_tester.request_authorization_header != NULL) {
            aws_string_destroy(s_tester.request_authorization_header);
            s_tester.request_authorization_header = NULL;
        }
        s_tester.request_authorization_header =
            aws_string_new_from_cursor(s_tester.allocator, &authorization_header_value);
    }

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

    *out_status_code = AWS_HTTP_STATUS_CODE_200_OK;

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

static int s_aws_ecs_tester_init(struct aws_allocator *allocator) {
    aws_auth_library_init(allocator);
    s_tester.allocator = allocator;
    if (aws_array_list_init_dynamic(&s_tester.response_data_callbacks, allocator, 10, sizeof(struct aws_byte_cursor))) {
        return AWS_OP_ERR;
    }

    if (aws_mutex_init(&s_tester.lock)) {
        return AWS_OP_ERR;
    }

    if (aws_condition_variable_init(&s_tester.signal)) {
        return AWS_OP_ERR;
    }

    /* default to everything successful */
    s_tester.is_connection_acquire_successful = true;
    s_tester.is_request_successful = true;

    s_tester.el_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = s_tester.el_group,
        .max_entries = 8,
    };
    s_tester.host_resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = s_tester.el_group,
        .host_resolver = s_tester.host_resolver,
    };
    s_tester.bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    struct aws_tls_ctx_options tls_options;
    aws_tls_ctx_options_init_default_client(&tls_options, allocator);
    struct aws_tls_ctx *tls_ctx = aws_tls_client_ctx_new(allocator, &tls_options);
    ASSERT_NOT_NULL(tls_ctx);
    s_tester.tls_ctx = tls_ctx;
    aws_tls_ctx_options_clean_up(&tls_options);

    /* ensure pre-existing environment doesn't interfere with tests */
    aws_unset_environment_value(s_ecs_creds_env_relative_uri);
    aws_unset_environment_value(s_ecs_creds_env_full_uri);
    aws_unset_environment_value(s_ecs_creds_env_token_file);
    aws_unset_environment_value(s_ecs_creds_env_token);

    return AWS_OP_SUCCESS;
}

static void s_aws_ecs_tester_reset(void) {
    aws_array_list_clean_up(&s_tester.response_data_callbacks);
    aws_string_destroy(s_tester.request_path_and_query);
    aws_string_destroy(s_tester.request_authorization_header);
    aws_string_destroy(s_tester.selected_host);
    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);
    aws_credentials_release(s_tester.credentials);
    aws_client_bootstrap_release(s_tester.bootstrap);
    aws_host_resolver_release(s_tester.host_resolver);
    aws_event_loop_group_release(s_tester.el_group);
    aws_tls_ctx_release(s_tester.tls_ctx);
    AWS_ZERO_STRUCT(s_tester);
}

static void s_aws_ecs_tester_cleanup(void) {
    s_aws_ecs_tester_reset();
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

static int s_credentials_provider_ecs_new_destroy(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);
    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_new_destroy, s_credentials_provider_ecs_new_destroy);

static int s_credentials_provider_ecs_connect_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);
    s_tester.is_connection_acquire_successful = false;

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
        .tls_ctx = s_tester.tls_ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials == NULL);
    ASSERT_UINT_EQUALS(443, s_tester.selected_port);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_connect_failure, s_credentials_provider_ecs_connect_failure);

static int s_credentials_provider_ecs_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);
    s_tester.is_request_successful = false;

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
        .tls_ctx = s_tester.tls_ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_STR_EQUALS("/path/to/resource/?a=b&c=d", aws_string_c_str(s_tester.request_path_and_query));
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials == NULL);
    ASSERT_UINT_EQUALS(443, s_tester.selected_port);
    ASSERT_UINT_EQUALS(4, s_tester.attempts);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_request_failure, s_credentials_provider_ecs_request_failure);

AWS_STATIC_STRING_FROM_LITERAL(s_bad_document_response, "{\"NotTheExpectedDocumentFormat\":\"Error\"}");

static int s_credentials_provider_ecs_bad_document_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_bad_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &bad_document_cursor);

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
        .tls_ctx = s_tester.tls_ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_STR_EQUALS("/path/to/resource/?a=b&c=d", aws_string_c_str(s_tester.request_path_and_query));

    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials == NULL);
    ASSERT_UINT_EQUALS(443, s_tester.selected_port);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_bad_document_failure, s_credentials_provider_ecs_bad_document_failure);

static int s_credentials_provider_ecs_bad_host_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.google.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_NULL(s_tester.credentials);
    ASSERT_INT_EQUALS(AWS_AUTH_CREDENTIALS_PROVIDER_ECS_INVALID_HOST, s_tester.error_code);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_bad_host_failure, s_credentials_provider_ecs_bad_host_failure);

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response,
    "{\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"SecretAccessKey\":\"SuccessfulSecret\", \n  "
    "\"Token\":\"TokenSuccess\", \n \"Expiration\":\"2020-02-25T06:03:31Z\"}");
AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response_with_account_id,
    "{\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"SecretAccessKey\":\"SuccessfulSecret\", \n  "
    "\"Token\":\"TokenSuccess\", \n \"Expiration\":\"2020-02-25T06:03:31Z\", \"AccountId\":\"AccountId123\"}");
AWS_STATIC_STRING_FROM_LITERAL(s_good_access_key_id, "SuccessfulAccessKey");
AWS_STATIC_STRING_FROM_LITERAL(s_good_secret_access_key, "SuccessfulSecret");
AWS_STATIC_STRING_FROM_LITERAL(s_good_session_token, "TokenSuccess");
AWS_STATIC_STRING_FROM_LITERAL(s_good_account_id, "AccountId123");
AWS_STATIC_STRING_FROM_LITERAL(s_good_response_expiration, "2020-02-25T06:03:31Z");

/* Check that expected URI and Authorization token were used to make request.
 * URI must be super explicit, specifying scheme and port. */
static int s_check_ecs_tester_request_uri_and_authorization(const char *expected_uri_cstr, const char *expected_token) {
    struct aws_byte_cursor expected_uri_cursor = aws_byte_cursor_from_c_str(expected_uri_cstr);
    struct aws_uri expected_uri;
    ASSERT_SUCCESS(aws_uri_init_parse(&expected_uri, s_tester.allocator, &expected_uri_cursor));
    ASSERT_TRUE(aws_uri_scheme(&expected_uri)->len != 0);
    ASSERT_TRUE(aws_uri_port(&expected_uri) != 0);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(*aws_uri_host_name(&expected_uri), s_tester.selected_host);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(*aws_uri_path_and_query(&expected_uri), s_tester.request_path_and_query);
    ASSERT_INT_EQUALS(
        aws_byte_cursor_eq_c_str_ignore_case(aws_uri_scheme(&expected_uri), "https"), s_tester.selected_tls);
    ASSERT_UINT_EQUALS(aws_uri_port(&expected_uri), s_tester.selected_port);

    if (expected_token != NULL) {
        ASSERT_STR_EQUALS(expected_token, aws_string_c_str(s_tester.request_authorization_header));
    } else {
        ASSERT_NULL(s_tester.request_authorization_header);
    }

    aws_uri_clean_up(&expected_uri);
    return 0;
}

static int s_do_ecs_success_test(
    struct aws_allocator *allocator,
    struct aws_credentials_provider_ecs_options *options,
    const char *expected_uri,
    const char *expected_token,
    const struct aws_string *expected_account_id) {
    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_SUCCESS(s_check_ecs_tester_request_uri_and_authorization(expected_uri, expected_token));
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(s_tester.credentials), s_good_access_key_id);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(s_tester.credentials), s_good_secret_access_key);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(s_tester.credentials), s_good_session_token);
    if (expected_account_id) {
        ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_account_id(s_tester.credentials), expected_account_id);
    } else {
        ASSERT_TRUE(aws_credentials_get_account_id(s_tester.credentials).len == 0);
    }

    struct aws_date_time expiration;
    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_good_response_expiration);
    aws_date_time_init_from_str_cursor(&expiration, &date_cursor, AWS_DATE_FORMAT_ISO_8601);
    ASSERT_TRUE(
        aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == (uint64_t)expiration.timestamp);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    return AWS_OP_SUCCESS;
}

static int s_do_ecs_env_success_test(
    struct aws_allocator *allocator,
    struct aws_credentials_provider_ecs_environment_options *options,
    const char *relative_uri,
    const char *full_uri,
    const char *auth_token,
    const char *auth_token_file_content,
    const char *expected_uri,
    const char *expected_token) {

    struct aws_string *relative_uri_str = NULL;
    if (relative_uri != NULL) {
        relative_uri_str = aws_string_new_from_c_str(allocator, relative_uri);
        ASSERT_SUCCESS(aws_set_environment_value(s_ecs_creds_env_relative_uri, relative_uri_str));
    }
    struct aws_string *full_uri_str = NULL;
    if (full_uri != NULL) {
        full_uri_str = aws_string_new_from_c_str(allocator, full_uri);
        ASSERT_SUCCESS(aws_set_environment_value(s_ecs_creds_env_full_uri, full_uri_str));
    }
    struct aws_string *auth_token_str = NULL;
    if (auth_token != NULL) {
        auth_token_str = aws_string_new_from_c_str(allocator, auth_token);
        ASSERT_SUCCESS(aws_set_environment_value(s_ecs_creds_env_token, auth_token_str));
    }
    struct aws_string *auth_token_file_contents_str = NULL;
    struct aws_string *auth_token_file_path = NULL;
    if (auth_token_file_content != NULL) {
        auth_token_file_contents_str = aws_string_new_from_c_str(allocator, auth_token_file_content);

        auth_token_file_path = aws_create_process_unique_file_name(allocator);
        ASSERT_NOT_NULL(auth_token_file_path);
        ASSERT_TRUE(aws_create_profile_file(auth_token_file_path, auth_token_file_contents_str) == AWS_OP_SUCCESS);
        ASSERT_SUCCESS(aws_set_environment_value(s_ecs_creds_env_token_file, auth_token_file_path));
    }

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs_from_environment(allocator, options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_SUCCESS(s_check_ecs_tester_request_uri_and_authorization(expected_uri, expected_token));

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
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);
    if (auth_token_file_path != NULL) {
        aws_file_delete(auth_token_file_path);
    }
    aws_string_destroy(relative_uri_str);
    aws_string_destroy(full_uri_str);
    aws_string_destroy(auth_token_str);
    aws_string_destroy(auth_token_file_contents_str);
    aws_string_destroy(auth_token_file_path);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_ecs_basic_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
        .tls_ctx = s_tester.tls_ctx,
    };

    ASSERT_SUCCESS(s_do_ecs_success_test(
        allocator,
        &options,
        "https://www.xxx123321testmocknonexsitingawsservice.com:443/path/to/resource/?a=b&c=d" /*expected_uri*/,
        "test-token-1234-abcd" /*expected_token*/,
        NULL /*expected_account_id*/));

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_basic_success, s_credentials_provider_ecs_basic_success);

static int s_credentials_provider_ecs_basic_success_with_account_id(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response_with_account_id);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
        .tls_ctx = s_tester.tls_ctx,
    };

    ASSERT_SUCCESS(s_do_ecs_success_test(
        allocator,
        &options,
        "https://www.xxx123321testmocknonexsitingawsservice.com:443/path/to/resource/?a=b&c=d" /*expected_uri*/,
        "test-token-1234-abcd" /*expected_token*/,
        s_good_account_id /*expected_account_id*/));

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_ecs_basic_success_with_account_id,
    s_credentials_provider_ecs_basic_success_with_account_id);

static int s_credentials_provider_ecs_basic_success_token_file(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_string *auth_token = aws_string_new_from_c_str(allocator, "test-token-1234-abcd");
    struct aws_string *token_file_path = aws_create_process_unique_file_name(allocator);
    ASSERT_NOT_NULL(token_file_path);
    ASSERT_TRUE(aws_create_profile_file(token_file_path, auth_token) == AWS_OP_SUCCESS);

    /* test that static auth token is not preferred over file token */
    struct aws_string *bad_auth_token = aws_string_new_from_c_str(allocator, "badtoken");

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);
    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_string(auth_token),
        .auth_token_file_path = aws_byte_cursor_from_string(token_file_path),
        .tls_ctx = s_tester.tls_ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_SUCCESS(s_check_ecs_tester_request_uri_and_authorization(
        "https://www.xxx123321testmocknonexsitingawsservice.com:443/path/to/resource/?a=b&c=d",
        aws_string_c_str(auth_token)));

    s_tester.has_received_credentials_callback = false;
    aws_credentials_release(s_tester.credentials);

    aws_mutex_unlock(&s_tester.lock);

    /* update the file with updated token */
    struct aws_string *auth_token2 = aws_string_new_from_c_str(allocator, "test-token2-4321-qwer");
    ASSERT_TRUE(aws_create_profile_file(token_file_path, auth_token2) == AWS_OP_SUCCESS);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_SUCCESS(s_check_ecs_tester_request_uri_and_authorization(
        "https://www.xxx123321testmocknonexsitingawsservice.com:443/path/to/resource/?a=b&c=d",
        aws_string_c_str(auth_token2)));

    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);

    s_aws_ecs_tester_cleanup();
    aws_file_delete(token_file_path);
    aws_string_destroy(auth_token);
    aws_string_destroy(auth_token2);
    aws_string_destroy(token_file_path);
    aws_string_destroy(bad_auth_token);
    return 0;
}
AWS_TEST_CASE(credentials_provider_ecs_basic_success_token_file, s_credentials_provider_ecs_basic_success_token_file);

static int s_credentials_provider_ecs_basic_success_uri_env(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    const struct test_case {
        const char *relative_uri;
        const char *full_uri;
        const char *expected_uri;
        const char *auth_token;
        const char *auth_token_file_content;
        const char *expected_auth_token;
    } test_cases[] = {
        /* simple full uri*/
        {
            .full_uri = "http://127.0.0.1/credentials",
            .expected_uri = "http://127.0.0.1:80/credentials",
        },
        /* explicit port */
        {
            .full_uri = "http://127.0.0.1:8080/credentials",
            .expected_uri = "http://127.0.0.1:8080/credentials",
        },
        /* https */
        {
            .full_uri = "https://www.xxx123321testmocknonexsitingawsservice.com/credentials",
            .expected_uri = "https://www.xxx123321testmocknonexsitingawsservice.com:443/credentials",
        },
        /* path and query */
        {
            .full_uri = "http://127.0.0.1/path/to/resource/?a=b&c=d",
            .expected_uri = "http://127.0.0.1:80/path/to/resource/?a=b&c=d",
        },
        /* relative URI */
        {
            .relative_uri = "/path/to/resource/?a=b&c=d",
            .expected_uri = "http://169.254.170.2:80/path/to/resource/?a=b&c=d",
        },
        /* relative URI takes priority, when both RELATIVE and FULL are set */
        {
            .relative_uri = "/from-relative-uri",
            .full_uri = "http://127.0.0.1/from-full-uri",
            .expected_uri = "http://169.254.170.2:80/from-relative-uri",
        },
        /* auth token is properly set */
        {
            .full_uri = "http://127.0.0.1:8080/credentials",
            .expected_uri = "http://127.0.0.1:8080/credentials",
            .auth_token = "testToken",
            .expected_auth_token = "testToken",
        },
        /* auth_token is respected */
        {
            .full_uri = "http://127.0.0.1:8080/credentials",
            .expected_uri = "http://127.0.0.1:8080/credentials",
            .auth_token = "testToken",
            .expected_auth_token = "testToken",
        },
        /* auth_token_file_path is respected */
        {
            .full_uri = "http://127.0.0.1:8080/credentials",
            .expected_uri = "http://127.0.0.1:8080/credentials",
            .auth_token_file_content = "testToken",
            .expected_auth_token = "testToken",
        },
        /* auth_token_file_path is preferred */
        {
            .full_uri = "http://127.0.0.1:8080/credentials",
            .expected_uri = "http://127.0.0.1:8080/credentials",
            .auth_token = "BadToken",
            .auth_token_file_content = "testToken",
            .expected_auth_token = "testToken",
        },
        /* IPv4 loopback address */
        {
            .full_uri = "http://127.1.2.3:456/credentials",
            .expected_uri = "http://127.1.2.3:456/credentials",
        },
        /* IPv4 EKS container host address */
        {
            .full_uri = "http://169.254.170.23:8080/credentials",
            .expected_uri = "http://169.254.170.23:8080/credentials",
        },
        /* IPv4 ECS container host address */
        {
            .full_uri = "http://169.254.170.2:8080/credentials",
            .expected_uri = "http://169.254.170.2:8080/credentials",
        },
        /* IPv6 loopback address */
        {
            .full_uri = "http://[::1]:8080/credentials",
            .expected_uri = "http://[::1]:8080/credentials",
        },
        /* IPv6 EKS host */
        {
            .full_uri = "http://[fd00:ec2::23]:8080/credentials",
            .expected_uri = "http://[fd00:ec2::23]:8080/credentials",
        },
    };

    for (size_t case_idx = 0; case_idx < AWS_ARRAY_SIZE(test_cases); ++case_idx) {
        struct test_case case_i = test_cases[case_idx];
        printf(
            "CASE[%zu]: AWS_CONTAINER_CREDENTIALS_RELATIVE_URI=%s AWS_CONTAINER_CREDENTIALS_FULL_URI=%s\n, "
            "AWS_CONTAINER_AUTHORIZATION_TOKEN=%s\n, auth_token_file_content=%s\n",
            case_idx,
            case_i.relative_uri ? case_i.relative_uri : "<UNSET>",
            case_i.full_uri ? case_i.full_uri : "<UNSET>",
            case_i.auth_token ? case_i.auth_token : "<UNSET>",
            case_i.auth_token_file_content ? case_i.auth_token_file_content : "<UNSET>");

        /* This unsets previous env vars */
        ASSERT_SUCCESS(s_aws_ecs_tester_init(allocator));

        struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
        aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);

        struct aws_credentials_provider_ecs_environment_options options = {
            .bootstrap = s_tester.bootstrap,
            .function_table = &s_mock_function_table,
            .shutdown_options =
                {
                    .shutdown_callback = s_on_shutdown_complete,
                    .shutdown_user_data = NULL,
                },
            .tls_ctx = s_tester.tls_ctx,
        };

        ASSERT_SUCCESS(s_do_ecs_env_success_test(
            allocator,
            &options,
            case_i.relative_uri,
            case_i.full_uri,
            case_i.auth_token,
            case_i.auth_token_file_content,
            case_i.expected_uri,
            case_i.expected_auth_token));

        s_aws_ecs_tester_reset();
    }

    s_aws_ecs_tester_cleanup();
    return 0;
}
AWS_TEST_CASE(credentials_provider_ecs_basic_success_uri_env, s_credentials_provider_ecs_basic_success_uri_env);

static int s_credentials_provider_ecs_no_auth_token_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .tls_ctx = s_tester.tls_ctx,
    };

    ASSERT_SUCCESS(s_do_ecs_success_test(
        allocator,
        &options,
        "https://www.xxx123321testmocknonexsitingawsservice.com:443/path/to/resource/?a=b&c=d" /*expected_uri*/,
        NULL /*expected_token*/,
        NULL /*expected_account_id=*/));

    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_no_auth_token_success, s_credentials_provider_ecs_no_auth_token_success);

AWS_STATIC_STRING_FROM_LITERAL(s_good_response_first_part, "{\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"Secret");
AWS_STATIC_STRING_FROM_LITERAL(s_good_response_second_part, "AccessKey\":\"SuccessfulSecret\", \n  \"Token\":\"Token");
AWS_STATIC_STRING_FROM_LITERAL(s_good_response_third_part, "Success\", \n \"Expiration\":\"2020-02-25T06:03:31Z\"}");

static int s_credentials_provider_ecs_success_multi_part_doc(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_ecs_tester_init(allocator);

    struct aws_byte_cursor good_response_cursor1 = aws_byte_cursor_from_string(s_good_response_first_part);
    struct aws_byte_cursor good_response_cursor2 = aws_byte_cursor_from_string(s_good_response_second_part);
    struct aws_byte_cursor good_response_cursor3 = aws_byte_cursor_from_string(s_good_response_third_part);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor1);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor2);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor3);

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
        .tls_ctx = s_tester.tls_ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_STR_EQUALS("/path/to/resource/?a=b&c=d", aws_string_c_str(s_tester.request_path_and_query));

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
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    /* Because we mock the http connection manager, we never get a callback back from it */
    aws_mem_release(provider->allocator, provider);
    s_aws_ecs_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_success_multi_part_doc, s_credentials_provider_ecs_success_multi_part_doc);

static int s_credentials_provider_ecs_real_new_destroy(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    s_aws_ecs_tester_init(allocator);

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

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = bootstrap,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);
    aws_event_loop_group_release(el_group);

    s_aws_ecs_tester_cleanup();

    aws_auth_library_clean_up();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_real_new_destroy, s_credentials_provider_ecs_real_new_destroy);

static int s_credentials_provider_ecs_real_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    s_aws_ecs_tester_init(allocator);

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

    struct aws_credentials_provider_ecs_options options = {
        .bootstrap = bootstrap,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .host = aws_byte_cursor_from_c_str("www.xxx123321testmocknonexsitingawsservice.com"),
        .path_and_query = aws_byte_cursor_from_c_str("/path/to/resource/?a=b&c=d"),
        .auth_token = aws_byte_cursor_from_c_str("test-token-1234-abcd"),
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_ecs(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.credentials != NULL);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_ecs_tester_cleanup();

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);
    aws_event_loop_group_release(el_group);

    aws_auth_library_clean_up();

    return 0;
}

AWS_TEST_CASE(credentials_provider_ecs_real_success, s_credentials_provider_ecs_real_success);
