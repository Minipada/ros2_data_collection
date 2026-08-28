/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/credentials_utils.h>
#include <aws/common/condition_variable.h>
#include <aws/common/environment.h>
#include <aws/common/json.h>
#include <aws/common/string.h>
#include <aws/http/request_response.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/socket.h>
#include <aws/io/tls_channel_handler.h>

#include "aws/io/stream.h"

struct aws_mock_web_credential_provider_tester {
    struct aws_allocator *allocator;
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

    struct aws_event_loop_group *el_group;
    struct aws_host_resolver *resolver;
    struct aws_client_bootstrap *bootstrap;

    struct aws_http_connection_manager *mock_manager;
    struct aws_http_connection *mock_connection;
    struct aws_http_stream *mock_stream;

    size_t current_request_attempt_number;
    struct aws_http_make_request_options request_callback_options;

    struct aws_byte_buf request_body;

    void (*manager_destructor_fn)(void *);
    void *manager_destructor_user_data;
};

static struct aws_mock_web_credential_provider_tester s_tester;

static struct aws_http_connection_manager *s_aws_http_connection_manager_new_mock(
    struct aws_allocator *allocator,
    const struct aws_http_connection_manager_options *options) {

    (void)allocator;
    (void)options;

    s_tester.manager_destructor_fn = options->shutdown_complete_callback;
    s_tester.manager_destructor_user_data = options->shutdown_complete_user_data;

    return s_tester.mock_manager;
}

static void s_aws_http_connection_manager_release_mock(struct aws_http_connection_manager *manager) {
    (void)manager;

    s_tester.manager_destructor_fn(s_tester.manager_destructor_user_data);
}

static void s_aws_http_connection_manager_acquire_connection_mock(
    struct aws_http_connection_manager *manager,
    aws_http_connection_manager_on_connection_setup_fn *callback,
    void *user_data) {

    (void)manager;
    (void)callback;
    (void)user_data;

    if (s_tester.is_connection_acquire_successful) {
        callback(s_tester.mock_connection, AWS_ERROR_SUCCESS, user_data);
    } else {
        aws_raise_error(AWS_ERROR_HTTP_UNKNOWN);
        callback(NULL, AWS_ERROR_HTTP_UNKNOWN, user_data);
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

    if (options->on_response_headers) {
        options->on_response_headers(s_tester.mock_stream, AWS_HTTP_HEADER_BLOCK_MAIN, headers, 1, options->user_data);
    }

    if (options->on_response_header_block_done) {
        options->on_response_header_block_done(s_tester.mock_stream, data_callback_count > 0, options->user_data);
    }

    size_t response_count = aws_array_list_length(&s_tester.response_data_callbacks);
    if (response_count > 0) {
        size_t response_body_index = aws_min_size(s_tester.current_request_attempt_number, response_count - 1);
        struct aws_byte_cursor data_callback_cursor;
        aws_array_list_get_at(data_callbacks, &data_callback_cursor, response_body_index);
        options->on_response_body(s_tester.mock_stream, &data_callback_cursor, options->user_data);
    }

    ++s_tester.current_request_attempt_number;

    options->on_complete(
        s_tester.mock_stream, is_request_successful ? AWS_ERROR_SUCCESS : AWS_ERROR_HTTP_UNKNOWN, options->user_data);
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
    s_tester.request_callback_options = *options;

    aws_byte_buf_reset(&s_tester.request_body, false);

    struct aws_input_stream *body_stream = aws_http_message_get_body_stream(options->request);
    if (body_stream != NULL) {
        aws_input_stream_seek(body_stream, 0, AWS_SSB_BEGIN);

        int64_t body_length = 0;
        aws_input_stream_get_length(body_stream, &body_length);

        AWS_LOGF_DEBUG(AWS_LS_AUTH_GENERAL, "Request body length: %d", (int)body_length);

        aws_byte_buf_reserve(&s_tester.request_body, (size_t)body_length);

        aws_input_stream_read(body_stream, &s_tester.request_body);
    }

    return s_tester.mock_stream;
}

static int s_aws_http_stream_activate_mock(struct aws_http_stream *stream) {
    (void)stream;

    s_invoke_mock_request_callbacks(
        &s_tester.request_callback_options, &s_tester.response_data_callbacks, s_tester.is_request_successful);
    return AWS_OP_SUCCESS;
}

static int s_aws_http_stream_get_incoming_response_status_mock(
    const struct aws_http_stream *stream,
    int *out_status_code) {
    (void)stream;

    if (s_tester.is_request_successful) {
        *out_status_code = 200;
    } else {
        *out_status_code = 400;
    }

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

static int s_aws_cognito_tester_init(struct aws_allocator *allocator) {
    aws_auth_library_init(allocator);

    s_tester.allocator = allocator;

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

    s_tester.el_group = aws_event_loop_group_new_default(allocator, 0, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = s_tester.el_group,
        .max_entries = 8,
    };
    s_tester.resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = s_tester.el_group,
        .host_resolver = s_tester.resolver,
    };
    s_tester.bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    AWS_ZERO_STRUCT(s_tester.tls_connection_options);
    struct aws_tls_ctx_options tls_options;
    aws_tls_ctx_options_init_default_client(&tls_options, allocator);
    s_tester.ctx = aws_tls_client_ctx_new(allocator, &tls_options);
    ASSERT_NOT_NULL(s_tester.ctx);
    aws_tls_ctx_options_clean_up(&tls_options);
    aws_tls_connection_options_init_from_ctx(&s_tester.tls_connection_options, s_tester.ctx);

    /* default to everything successful */
    s_tester.is_connection_acquire_successful = true;
    s_tester.is_request_successful = true;

    /* I hate using 1 for mocks, let's instead point at valid addresses */
    s_tester.mock_manager = (void *)(&s_tester.mock_manager);
    s_tester.mock_connection = (void *)(&s_tester.mock_connection);
    s_tester.mock_stream = (void *)(&s_tester.mock_stream);

    aws_byte_buf_init(&s_tester.request_body, allocator, 100);

    return AWS_OP_SUCCESS;
}

static void s_aws_cognito_tester_cleanup(void) {
    aws_array_list_clean_up(&s_tester.response_data_callbacks);
    aws_byte_buf_clean_up(&s_tester.request_uri);
    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);
    aws_credentials_release(s_tester.credentials);
    aws_byte_buf_clean_up(&s_tester.request_body);

    aws_client_bootstrap_release(s_tester.bootstrap);
    aws_host_resolver_release(s_tester.resolver);
    aws_event_loop_group_release(s_tester.el_group);

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

static int s_credentials_provider_cognito_new_destroy(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);
    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_provider_cognito_new_destroy, s_credentials_provider_cognito_new_destroy);

static int s_credentials_provider_cognito_failure_connect_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);
    s_tester.is_connection_acquire_successful = false;

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_ERROR_HTTP_UNKNOWN);
    ASSERT_TRUE(s_tester.credentials == NULL);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_provider_cognito_failure_connect, s_credentials_provider_cognito_failure_connect_fn);

static int s_credentials_provider_cognito_failure_request_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);
    s_tester.is_request_successful = false;

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_AUTH_CREDENTIALS_PROVIDER_HTTP_STATUS_FAILURE);
    ASSERT_TRUE(s_tester.credentials == NULL);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_provider_cognito_failure_request, s_credentials_provider_cognito_failure_request_fn);

AWS_STATIC_STRING_FROM_LITERAL(s_bad_document_response, "{\"NotTheExpectedDocumentFormat\":\"Error\"}");

static int s_credentials_provider_cognito_failure_bad_document_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_bad_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &bad_document_cursor);

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code != AWS_ERROR_SUCCESS);
    ASSERT_TRUE(s_tester.credentials == NULL);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_cognito_failure_bad_document,
    s_credentials_provider_cognito_failure_bad_document_fn);

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_document_response,
    "{\"Credentials\":{\"AccessKeyId\":\"SomeAccessKeyIdValue\",\"SecretKey\":\"SomeSecretKeyValue\",\"SessionToken\":"
    "\"SomeSessionTokenValue\",\"Expiration\":1663003154}}");

AWS_STATIC_STRING_FROM_LITERAL(s_expected_access_key_id, "SomeAccessKeyIdValue");
AWS_STATIC_STRING_FROM_LITERAL(s_expected_secret_access_key, "SomeSecretKeyValue");
AWS_STATIC_STRING_FROM_LITERAL(s_expected_session_token, "SomeSessionTokenValue");

static int s_verify_credentials(struct aws_credentials *credentials) {
    struct aws_byte_cursor access_key_id = aws_credentials_get_access_key_id(credentials);
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_access_key_id->bytes, s_expected_access_key_id->len, access_key_id.ptr, access_key_id.len);

    struct aws_byte_cursor secret_access_key = aws_credentials_get_secret_access_key(credentials);
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_secret_access_key->bytes,
        s_expected_secret_access_key->len,
        secret_access_key.ptr,
        secret_access_key.len);

    struct aws_byte_cursor session_token = aws_credentials_get_session_token(credentials);
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_session_token->bytes, s_expected_session_token->len, session_token.ptr, session_token.len);

    uint64_t expiration = aws_credentials_get_expiration_timepoint_seconds(credentials);
    ASSERT_TRUE(expiration > 0);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_cognito_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    struct aws_byte_cursor good_document_cursor = aws_byte_cursor_from_string(s_good_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_document_cursor);

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_ERROR_SUCCESS);
    ASSERT_TRUE(s_tester.credentials != NULL);
    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_provider_cognito_success, s_credentials_provider_cognito_success_fn);

static int s_credentials_provider_cognito_success_after_retry_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    /* crummy response followed by a good one.  Verifies basic retry flow */
    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_bad_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &bad_document_cursor);
    struct aws_byte_cursor good_document_cursor = aws_byte_cursor_from_string(s_good_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_document_cursor);

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_ERROR_SUCCESS);
    ASSERT_TRUE(s_tester.credentials != NULL);
    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_provider_cognito_success_after_retry, s_credentials_provider_cognito_success_after_retry_fn);

AWS_STATIC_STRING_FROM_LITERAL(s_cognito_identity_environment_variable, "AWS_TESTING_COGNITO_IDENTITY");

static int s_credentials_provider_cognito_success_unauthenticated_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    struct aws_string *identity = NULL;
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_cognito_identity_environment_variable, &identity));
    ASSERT_NOT_NULL(identity);

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .endpoint = aws_byte_cursor_from_c_str("cognito-identity.us-east-1.amazonaws.com"),
        .identity = aws_byte_cursor_from_string(identity),
        .tls_ctx = s_tester.ctx,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_ERROR_SUCCESS);
    ASSERT_TRUE(s_tester.credentials != NULL);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    aws_string_destroy(identity);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_cognito_success_unauthenticated,
    s_credentials_provider_cognito_success_unauthenticated_fn);

static int s_get_token_pairs_sync_failure(
    void *get_token_pairs_user_data,
    aws_credentials_provider_cognito_get_token_pairs_completion_fn *completion_callback,
    void *completion_user_data) {

    (void)get_token_pairs_user_data;
    (void)completion_callback;
    (void)completion_user_data;

    return aws_raise_error(AWS_ERROR_UNKNOWN);
}

static int s_credentials_provider_cognito_failure_dynamic_token_pairs_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
        .get_token_pairs = s_get_token_pairs_sync_failure,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_ERROR_UNKNOWN);
    ASSERT_TRUE(s_tester.credentials == NULL);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_cognito_failure_dynamic_token_pairs,
    s_credentials_provider_cognito_failure_dynamic_token_pairs_fn);

static int s_get_token_pairs_completion_failure(
    void *get_token_pairs_user_data,
    aws_credentials_provider_cognito_get_token_pairs_completion_fn *completion_callback,
    void *completion_user_data) {

    (void)get_token_pairs_user_data;

    completion_callback(NULL, 0, AWS_ERROR_UNKNOWN, completion_user_data);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_cognito_failure_dynamic_token_pairs_completion_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
        .get_token_pairs = s_get_token_pairs_completion_failure,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_ERROR_UNKNOWN);
    ASSERT_TRUE(s_tester.credentials == NULL);
    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_cognito_failure_dynamic_token_pairs_completion,
    s_credentials_provider_cognito_failure_dynamic_token_pairs_completion_fn);

static int s_get_token_pairs_success(
    void *get_token_pairs_user_data,
    aws_credentials_provider_cognito_get_token_pairs_completion_fn *completion_callback,
    void *completion_user_data) {

    (void)get_token_pairs_user_data;

    struct aws_cognito_identity_provider_token_pair pairs[] = {
        {
            .identity_provider_name = aws_byte_cursor_from_c_str("dynamic-name"),
            .identity_provider_token = aws_byte_cursor_from_c_str("dynamic-value"),
        },
    };

    completion_callback(pairs, AWS_ARRAY_SIZE(pairs), AWS_ERROR_SUCCESS, completion_user_data);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_cognito_success_dynamic_token_pairs_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_cognito_tester_init(allocator);

    struct aws_byte_cursor good_document_cursor = aws_byte_cursor_from_string(s_good_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_document_cursor);

    struct aws_cognito_identity_provider_token_pair static_pairs[] = {
        {
            .identity_provider_name = aws_byte_cursor_from_c_str("static-name"),
            .identity_provider_token = aws_byte_cursor_from_c_str("static-value"),
        },
    };

    struct aws_credentials_provider_cognito_options options = {
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .endpoint = aws_byte_cursor_from_c_str("somewhere.amazonaws.com"),
        .identity = aws_byte_cursor_from_c_str("someone"),
        .tls_ctx = s_tester.ctx,
        .logins = static_pairs,
        .login_count = AWS_ARRAY_SIZE(static_pairs),
        .get_token_pairs = s_get_token_pairs_success,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_cognito(allocator, &options);
    ASSERT_NOT_NULL(provider);

    ASSERT_SUCCESS(aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL));

    s_aws_wait_for_credentials_result();

    aws_mutex_lock(&s_tester.lock);
    ASSERT_TRUE(s_tester.has_received_credentials_callback == true);
    ASSERT_TRUE(s_tester.error_code == AWS_ERROR_SUCCESS);
    ASSERT_TRUE(s_tester.credentials != NULL);

    // Check the request body and verify both a static and dynamic login pair appear in the JSON blob appropriately
    struct aws_json_value *json_body =
        aws_json_value_new_from_string(allocator, aws_byte_cursor_from_buf(&s_tester.request_body));
    ASSERT_NOT_NULL(json_body);

    struct aws_json_value *logins = aws_json_value_get_from_object(json_body, aws_byte_cursor_from_c_str("Logins"));
    ASSERT_NOT_NULL(logins);

    // Verify static login appears
    struct aws_json_value *static_login =
        aws_json_value_get_from_object(logins, aws_byte_cursor_from_c_str("static-name"));
    ASSERT_NOT_NULL(static_login);
    ASSERT_TRUE(aws_json_value_is_string(static_login));

    struct aws_byte_cursor static_value_cursor;
    ASSERT_SUCCESS(aws_json_value_get_string(static_login, &static_value_cursor));

    ASSERT_BIN_ARRAYS_EQUALS(
        static_pairs[0].identity_provider_token.ptr,
        static_pairs[0].identity_provider_token.len,
        static_value_cursor.ptr,
        static_value_cursor.len);

    // Verify dynamic login appears
    struct aws_json_value *dynamic_login =
        aws_json_value_get_from_object(logins, aws_byte_cursor_from_c_str("dynamic-name"));
    ASSERT_NOT_NULL(dynamic_login);
    ASSERT_TRUE(aws_json_value_is_string(dynamic_login));

    struct aws_byte_cursor dynamic_value_cursor;
    ASSERT_SUCCESS(aws_json_value_get_string(dynamic_login, &dynamic_value_cursor));

    struct aws_byte_cursor expected_dynamic_value = aws_byte_cursor_from_c_str("dynamic-value");
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_dynamic_value.ptr, expected_dynamic_value.len, dynamic_value_cursor.ptr, dynamic_value_cursor.len);

    aws_json_value_destroy(json_body);

    aws_mutex_unlock(&s_tester.lock);

    aws_credentials_provider_release(provider);

    s_aws_cognito_tester_cleanup();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_cognito_success_dynamic_token_pairs,
    s_credentials_provider_cognito_success_dynamic_token_pairs_fn);
