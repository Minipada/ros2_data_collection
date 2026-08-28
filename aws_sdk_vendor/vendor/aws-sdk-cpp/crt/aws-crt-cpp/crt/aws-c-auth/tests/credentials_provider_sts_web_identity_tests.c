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
#include <aws/http/request_response.h>
#include <aws/http/status_code.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/logging.h>
#include <aws/io/socket.h>
#include <aws/io/stream.h>
#include <aws/io/tls_channel_handler.h>
#include <aws/sdkutils/aws_profile.h>

static struct aws_mock_sts_web_identity_tester {
    struct aws_tls_ctx *tls_ctx;

    struct aws_byte_buf request_body;

    struct aws_array_list response_data_callbacks;
    bool is_connection_acquire_successful;
    bool is_request_successful;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    struct aws_credentials *credentials;
    bool has_received_credentials_callback;
    bool has_received_shutdown_callback;

    int attempts;
    int response_code;
    int error_code;
} s_tester;

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

struct mock_connection_manager {
    struct aws_allocator *allocator;
    aws_http_connection_manager_shutdown_complete_fn *shutdown_complete_callback;
    void *shutdown_complete_user_data;
};

static struct aws_http_connection_manager *s_aws_http_connection_manager_new_mock(
    struct aws_allocator *allocator,
    const struct aws_http_connection_manager_options *options) {

    struct mock_connection_manager *mock_manager = aws_mem_calloc(allocator, 1, sizeof(struct mock_connection_manager));
    mock_manager->allocator = allocator;
    mock_manager->shutdown_complete_callback = options->shutdown_complete_callback;
    mock_manager->shutdown_complete_user_data = options->shutdown_complete_user_data;
    return (struct aws_http_connection_manager *)mock_manager;
}

static void s_aws_http_connection_manager_release_mock(struct aws_http_connection_manager *manager) {
    struct mock_connection_manager *mock_manager = (struct mock_connection_manager *)manager;
    mock_manager->shutdown_complete_callback(mock_manager->shutdown_complete_user_data);
    aws_mem_release(mock_manager->allocator, mock_manager);
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
    struct aws_input_stream *body_stream = aws_http_message_get_body_stream(options->request);
    struct aws_allocator *allocator = s_tester.request_body.allocator;
    aws_byte_buf_clean_up(&s_tester.request_body);
    aws_byte_buf_init(&s_tester.request_body, allocator, 256);
    aws_input_stream_read(body_stream, &s_tester.request_body);
    s_invoke_mock_request_callbacks(options, &s_tester.response_data_callbacks, s_tester.is_request_successful);

    s_tester.attempts++;
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

    if (s_tester.response_code) {
        *out_status_code = s_tester.response_code;
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

static struct aws_http_connection *s_aws_http_stream_get_connection_mock(const struct aws_http_stream *stream) {
    (void)stream;
    return (struct aws_http_connection *)1;
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
    .aws_http_connection_close = s_aws_http_connection_close_mock};

AWS_STATIC_STRING_FROM_LITERAL(s_sts_web_identity_foo_profile, "foo");
AWS_STATIC_STRING_FROM_LITERAL(s_sts_web_identity_region_env, "AWS_DEFAULT_REGION");
AWS_STATIC_STRING_FROM_LITERAL(s_sts_web_identity_role_arn_env, "AWS_ROLE_ARN");
AWS_STATIC_STRING_FROM_LITERAL(s_sts_web_identity_role_session_name_env, "AWS_ROLE_SESSION_NAME");
AWS_STATIC_STRING_FROM_LITERAL(s_sts_web_identity_token_file_path_env, "AWS_WEB_IDENTITY_TOKEN_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_sts_web_identity_token_contents, "my-test-token-contents-123-abc-xyz");

static int s_aws_sts_web_identity_test_unset_env_parameters(void) {
    ASSERT_TRUE(aws_unset_environment_value(s_sts_web_identity_region_env) == AWS_OP_SUCCESS);
    ASSERT_TRUE(aws_unset_environment_value(s_sts_web_identity_role_arn_env) == AWS_OP_SUCCESS);
    ASSERT_TRUE(aws_unset_environment_value(s_sts_web_identity_role_session_name_env) == AWS_OP_SUCCESS);
    ASSERT_TRUE(aws_unset_environment_value(s_sts_web_identity_token_file_path_env) == AWS_OP_SUCCESS);

    return AWS_OP_SUCCESS;
}

static int s_aws_sts_web_identity_test_init_env_parameters(
    struct aws_allocator *allocator,
    const char *region,
    const char *role_arn,
    const char *role_session_name,
    const char *web_identity_token_file) {

    struct aws_string *region_str = aws_string_new_from_c_str(allocator, region);
    ASSERT_TRUE(region_str != NULL);
    ASSERT_TRUE(aws_set_environment_value(s_sts_web_identity_region_env, region_str) == AWS_OP_SUCCESS);
    aws_string_destroy(region_str);

    struct aws_string *role_arn_str = aws_string_new_from_c_str(allocator, role_arn);
    ASSERT_TRUE(role_arn_str != NULL);
    ASSERT_TRUE(aws_set_environment_value(s_sts_web_identity_role_arn_env, role_arn_str) == AWS_OP_SUCCESS);
    aws_string_destroy(role_arn_str);

    struct aws_string *role_session_name_str = aws_string_new_from_c_str(allocator, role_session_name);
    ASSERT_TRUE(role_session_name_str != NULL);
    ASSERT_TRUE(
        aws_set_environment_value(s_sts_web_identity_role_session_name_env, role_session_name_str) == AWS_OP_SUCCESS);
    aws_string_destroy(role_session_name_str);

    struct aws_string *web_identity_token_file_str = aws_string_new_from_c_str(allocator, web_identity_token_file);
    ASSERT_TRUE(web_identity_token_file_str != NULL);
    ASSERT_TRUE(
        aws_set_environment_value(s_sts_web_identity_token_file_path_env, web_identity_token_file_str) ==
        AWS_OP_SUCCESS);
    aws_string_destroy(web_identity_token_file_str);

    return AWS_OP_SUCCESS;
}

static int s_aws_sts_web_identity_test_init_config_profile(
    struct aws_allocator *allocator,
    const struct aws_string *config_contents) {

    struct aws_string *config_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(config_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(config_file_path_str, config_contents) == AWS_OP_SUCCESS);

    ASSERT_TRUE(
        aws_set_environment_value(s_default_config_path_env_variable_name, config_file_path_str) == AWS_OP_SUCCESS);

    ASSERT_TRUE(
        aws_set_environment_value(s_default_profile_env_variable_name, s_sts_web_identity_foo_profile) ==
        AWS_OP_SUCCESS);

    aws_string_destroy(config_file_path_str);
    return AWS_OP_SUCCESS;
}

static int s_aws_sts_web_identity_tester_init(struct aws_allocator *allocator) {
    aws_auth_library_init(allocator);

    struct aws_tls_ctx_options tls_options;
    aws_tls_ctx_options_init_default_client(&tls_options, allocator);
    s_tester.tls_ctx = aws_tls_client_ctx_new(allocator, &tls_options);
    ASSERT_NOT_NULL(s_tester.tls_ctx);

    if (aws_array_list_init_dynamic(&s_tester.response_data_callbacks, allocator, 10, sizeof(struct aws_byte_cursor))) {
        return AWS_OP_ERR;
    }

    if (aws_byte_buf_init(&s_tester.request_body, allocator, 256)) {
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

    return AWS_OP_SUCCESS;
}

static void s_aws_sts_web_identity_tester_cleanup(void) {
    aws_tls_ctx_release(s_tester.tls_ctx);
    aws_array_list_clean_up(&s_tester.response_data_callbacks);
    aws_byte_buf_clean_up(&s_tester.request_body);
    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);
    aws_credentials_release(s_tester.credentials);
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
    s_tester.credentials = credentials;
    s_tester.error_code = error_code;
    if (credentials != NULL) {
        aws_credentials_acquire(credentials);
    }
    aws_condition_variable_notify_one(&s_tester.signal);
    aws_mutex_unlock(&s_tester.lock);
}

static int s_credentials_provider_sts_web_identity_new_destroy_from_parameters(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .region = aws_byte_cursor_from_c_str("us-east-1"),
        .role_arn = aws_byte_cursor_from_c_str("arn:aws:iam::1234567890:role/test-arn"),
        .role_session_name = aws_byte_cursor_from_c_str("9876543210"),
        .token_file_path = aws_byte_cursor_from_string(token_file_path_str),
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);
    ASSERT_NOT_NULL(provider);
    aws_string_destroy(token_file_path_str);
    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_new_destroy_from_parameters,
    s_credentials_provider_sts_web_identity_new_destroy_from_parameters);

static int s_credentials_provider_sts_web_identity_new_destroy_from_env(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);
    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_new_destroy_from_env,
    s_credentials_provider_sts_web_identity_new_destroy_from_env);

AWS_STATIC_STRING_FROM_LITERAL(
    s_sts_web_identity_config_file_contents,
    "[profile default]\n"
    "region=us-east-1\n"
    "role_arn=arn:aws:iam::1111111111:role/test-arn\n"
    "role_session_name=2222222222\n"
    "web_identity_token_file=/some/unreachable/path/toklen_file\n"
    "[profile foo]\n"
    "region=us-west-2\n"
    "role_arn=arn:aws:iam::3333333333:role/test-arn\n"
    "role_session_name=4444444444\n"
    "web_identity_token_file=");

static int s_credentials_provider_sts_web_identity_new_destroy_from_config(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    struct aws_byte_buf content_buf;
    struct aws_byte_buf existing_content =
        aws_byte_buf_from_c_str(aws_string_c_str(s_sts_web_identity_config_file_contents));
    aws_byte_buf_init_copy(&content_buf, allocator, &existing_content);
    struct aws_byte_cursor cursor = aws_byte_cursor_from_string(token_file_path_str);
    ASSERT_TRUE(aws_byte_buf_append_dynamic(&content_buf, &cursor) == AWS_OP_SUCCESS);
    cursor = aws_byte_cursor_from_c_str("\n");
    ASSERT_TRUE(aws_byte_buf_append_dynamic(&content_buf, &cursor) == AWS_OP_SUCCESS);
    aws_string_destroy(token_file_path_str);

    struct aws_string *config_file_contents = aws_string_new_from_array(allocator, content_buf.buffer, content_buf.len);
    ASSERT_TRUE(config_file_contents != NULL);
    aws_byte_buf_clean_up(&content_buf);

    s_aws_sts_web_identity_test_init_config_profile(allocator, config_file_contents);
    aws_string_destroy(config_file_contents);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);
    ASSERT_NOT_NULL(provider);
    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_new_destroy_from_config,
    s_credentials_provider_sts_web_identity_new_destroy_from_config);

AWS_STATIC_STRING_FROM_LITERAL(
    s_basic_config_file,
    "[profile foo]\n"
    "region=us-east-1\n");

static int s_credentials_provider_sts_web_identity_fail_with_empty_config_and_env(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    struct aws_byte_buf content_buf = aws_byte_buf_from_c_str(aws_string_c_str(s_basic_config_file));

    struct aws_string *config_file_contents = aws_string_new_from_array(allocator, content_buf.buffer, content_buf.len);
    ASSERT_TRUE(config_file_contents != NULL);
    aws_byte_buf_clean_up(&content_buf);

    s_aws_sts_web_identity_test_init_config_profile(allocator, config_file_contents);
    aws_string_destroy(config_file_contents);

    s_aws_sts_web_identity_test_init_env_parameters(allocator, "", "", "", "");

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    ASSERT_NULL(aws_credentials_provider_new_sts_web_identity(allocator, &options));
    s_aws_sts_web_identity_tester_cleanup();
    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_fail_with_empty_config_and_env,
    s_credentials_provider_sts_web_identity_fail_with_empty_config_and_env);

static int s_credentials_provider_sts_web_identity_new_destroy_from_cached_config(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    /* create a config file */
    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    struct aws_byte_buf content_buf;
    struct aws_byte_buf existing_content =
        aws_byte_buf_from_c_str(aws_string_c_str(s_sts_web_identity_config_file_contents));
    aws_byte_buf_init_copy(&content_buf, allocator, &existing_content);
    struct aws_byte_cursor cursor = aws_byte_cursor_from_string(token_file_path_str);
    ASSERT_TRUE(aws_byte_buf_append_dynamic(&content_buf, &cursor) == AWS_OP_SUCCESS);
    cursor = aws_byte_cursor_from_c_str("\n");
    ASSERT_TRUE(aws_byte_buf_append_dynamic(&content_buf, &cursor) == AWS_OP_SUCCESS);
    aws_string_destroy(token_file_path_str);

    struct aws_string *config_file_contents = aws_string_new_from_array(allocator, content_buf.buffer, content_buf.len);
    ASSERT_TRUE(config_file_contents != NULL);
    aws_byte_buf_clean_up(&content_buf);

    s_aws_sts_web_identity_test_init_config_profile(allocator, config_file_contents);
    aws_string_destroy(config_file_contents);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    /* read the config files */
    struct aws_profile_collection *config_profiles = NULL;
    struct aws_string *config_file_path = NULL;

    ASSERT_TRUE(
        aws_get_environment_value(allocator, s_default_config_path_env_variable_name, &config_file_path) ==
        AWS_OP_SUCCESS);

    config_profiles = aws_profile_collection_new_from_file(allocator, config_file_path, AWS_PST_CONFIG);
    ASSERT_NOT_NULL(config_profiles);

    options.config_profile_collection_cached = config_profiles;

    /* unset environment and config file*/
    struct aws_string *empty_content = aws_string_new_from_c_str(allocator, "");
    ASSERT_TRUE(empty_content != NULL);
    s_aws_sts_web_identity_test_init_config_profile(allocator, empty_content);
    aws_string_destroy(empty_content);

    s_aws_sts_web_identity_test_unset_env_parameters();

    ASSERT_TRUE(aws_unset_environment_value(s_default_profile_env_variable_name) == AWS_OP_SUCCESS);

    /* assert we can create sts web identity from cached config file */
    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);
    ASSERT_NOT_NULL(provider);
    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();
    aws_string_destroy(config_file_path);
    aws_profile_collection_release(config_profiles);
    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_new_destroy_from_cached_config,
    s_credentials_provider_sts_web_identity_new_destroy_from_cached_config);

static int s_credentials_provider_sts_web_identity_new_failed_without_env_and_config(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    struct aws_string *empty_content = aws_string_new_from_c_str(allocator, "");
    ASSERT_TRUE(empty_content != NULL);
    s_aws_sts_web_identity_test_init_config_profile(allocator, empty_content);
    aws_string_destroy(empty_content);

    s_aws_sts_web_identity_test_unset_env_parameters();

    ASSERT_TRUE(aws_unset_environment_value(s_default_profile_env_variable_name) == AWS_OP_SUCCESS);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);
    ASSERT_TRUE(provider == NULL);

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_sts_web_identity_new_failed_without_env_and_config,
    s_credentials_provider_sts_web_identity_new_failed_without_env_and_config);

AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_sts_web_identity_body_message,
    "Action=AssumeRoleWithWebIdentity&Version=2011-06-15"
    "&RoleArn=arn%3Aaws%3Aiam%3A%3A1234567890%3Arole%2Ftest-arn&RoleSessionName=9876543210&WebIdentityToken=my-test-"
    "token-contents-123-abc-xyz");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_sts_web_identity_body_message_config,
    "Action=AssumeRoleWithWebIdentity&Version=2011-06-15"
    "&RoleArn=arn%3Aaws%3Aiam%3A%3A3333333333%3Arole%2Ftest-arn&RoleSessionName=4444444444&WebIdentityToken=my-test-"
    "token-contents-123-abc-xyz");

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response,
    "<AssumeRoleWithWebIdentityResponse>"
    "    <AssumeRoleWithWebIdentityResult>"
    "        <AssumedRoleUser>"
    "           <Arn>arn:aws:sts::123456789012:assumed-role/FederatedWebIdentityRole/app1</Arn>"
    "           <AssumedRoleId>AROACLKWSDQRAOEXAMPLE:app1</AssumedRoleId>"
    "        </AssumedRoleUser>"
    "        <Credentials>"
    "            <SessionToken>TokenSuccess</SessionToken>"
    "           <SecretAccessKey>SuccessfulSecret</SecretAccessKey>"
    "            <Expiration>2020-02-25T06:03:31Z</Expiration>"
    "           <AccessKeyId>SuccessfulAccessKey</AccessKeyId>"
    "        </Credentials>"
    "       <Provider>www.amazon.com</Provider>"
    "    </AssumeRoleWithWebIdentityResult>"
    "   <ResponseMetadata>"
    "        <RequestId>ad4156e9-bce1-11e2-82e6-6b6efEXAMPLE</RequestId>"
    "   </ResponseMetadata>"
    "</AssumeRoleWithWebIdentityResponse>");
AWS_STATIC_STRING_FROM_LITERAL(s_good_access_key_id, "SuccessfulAccessKey");
AWS_STATIC_STRING_FROM_LITERAL(s_good_secret_access_key, "SuccessfulSecret");
AWS_STATIC_STRING_FROM_LITERAL(s_good_session_token, "TokenSuccess");
AWS_STATIC_STRING_FROM_LITERAL(s_good_response_expiration, "2020-02-25T06:03:31Z");
AWS_STATIC_STRING_FROM_LITERAL(s_good_account_id_response, "123456789012");

static int s_verify_credentials(bool request_made, bool from_config, bool got_credentials, int expected_attempts) {

    if (request_made) {
        if (from_config) {
            ASSERT_CURSOR_VALUE_STRING_EQUALS(
                aws_byte_cursor_from_buf(&s_tester.request_body), s_expected_sts_web_identity_body_message_config);
        } else {
            ASSERT_CURSOR_VALUE_STRING_EQUALS(
                aws_byte_cursor_from_buf(&s_tester.request_body), s_expected_sts_web_identity_body_message);
        }
    }

    ASSERT_TRUE(s_tester.has_received_credentials_callback);

    if (got_credentials) {
        ASSERT_TRUE(s_tester.credentials != NULL);
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_access_key_id(s_tester.credentials), s_good_access_key_id);
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_secret_access_key(s_tester.credentials), s_good_secret_access_key);
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_session_token(s_tester.credentials), s_good_session_token);
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_account_id(s_tester.credentials), s_good_account_id_response);
    } else {
        ASSERT_TRUE(s_tester.credentials == NULL);
    }

    ASSERT_TRUE(s_tester.attempts == expected_attempts);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_sts_web_identity_connect_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);
    s_tester.is_connection_acquire_successful = false;

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(
        false /*no request*/, false /*from config*/, false /*get creds*/, 0 /*expected attempts*/));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_connect_failure,
    s_credentials_provider_sts_web_identity_connect_failure);

static int s_credentials_provider_sts_web_identity_request_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);
    s_tester.is_request_successful = false;

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(
        true /*request made*/, false /*from config*/, false /*get creds*/, 1 /*expected attempts*/));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_request_failure,
    s_credentials_provider_sts_web_identity_request_failure);

AWS_STATIC_STRING_FROM_LITERAL(
    s_bad_document_response,
    "<AssumeRoleWithWebIdentityResponse xmlns=\"Not the right doc\">Test</AssumeRoleWithWebIdentityResponse>");

static int s_credentials_provider_sts_web_identity_bad_document_failure(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_bad_document_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &bad_document_cursor);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(
        true /*request made*/, false /*from config*/, false /*get creds*/, 1 /*expected attempts*/));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_bad_document_failure,
    s_credentials_provider_sts_web_identity_bad_document_failure);

AWS_STATIC_STRING_FROM_LITERAL(
    s_retryable_error_response_1,
    "<Error>"
    "<Code>IDPCommunicationError</Code>"
    "<Message>XXX</Message>"
    "<Resource>YYY</Resource>"
    "<RequestId>4442587FB7D0A2F9</RequestId>"
    "</Error>");

AWS_STATIC_STRING_FROM_LITERAL(
    s_retryable_error_response_2,
    "<Error>"
    "<Code>InvalidIdentityToken</Code>"
    "<Message>XXX</Message>"
    "<Resource>YYY</Resource>"
    "<RequestId>4442587FB7D0A2F9</RequestId>"
    "</Error>");

static int s_credentials_provider_sts_web_identity_test_retry_error1(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);
    s_tester.response_code = AWS_HTTP_STATUS_CODE_400_BAD_REQUEST;
    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_retryable_error_response_1);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &bad_document_cursor);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(
        true /*request made*/, false /*from config*/, false /*get creds*/, 3 /*expected attempts*/));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_test_retry_error1,
    s_credentials_provider_sts_web_identity_test_retry_error1);

static int s_credentials_provider_sts_web_identity_test_retry_error2(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);
    s_tester.response_code = AWS_HTTP_STATUS_CODE_400_BAD_REQUEST;
    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_byte_cursor bad_document_cursor = aws_byte_cursor_from_string(s_retryable_error_response_2);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &bad_document_cursor);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(
        true /*request made*/, false /*from config*/, false /*get creds*/, 3 /*expected attempts*/));

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_test_retry_error2,
    s_credentials_provider_sts_web_identity_test_retry_error2);

static int s_credentials_provider_sts_web_identity_basic_success_env(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(
        true /*request made*/, false /*from config*/, true /*get creds*/, 1 /*expected attempts*/));

    struct aws_date_time expiration;
    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_good_response_expiration);
    aws_date_time_init_from_str_cursor(&expiration, &date_cursor, AWS_DATE_FORMAT_ISO_8601);
    ASSERT_INT_EQUALS(
        aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials), (uint64_t)expiration.timestamp);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_sts_web_identity_basic_success_env,
    s_credentials_provider_sts_web_identity_basic_success_env);

static int s_credentials_provider_sts_web_identity_basic_success_config(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    struct aws_byte_buf content_buf;
    struct aws_byte_buf existing_content =
        aws_byte_buf_from_c_str(aws_string_c_str(s_sts_web_identity_config_file_contents));
    aws_byte_buf_init_copy(&content_buf, allocator, &existing_content);
    struct aws_byte_cursor cursor = aws_byte_cursor_from_string(token_file_path_str);
    ASSERT_TRUE(aws_byte_buf_append_dynamic(&content_buf, &cursor) == AWS_OP_SUCCESS);
    cursor = aws_byte_cursor_from_c_str("\n");
    ASSERT_TRUE(aws_byte_buf_append_dynamic(&content_buf, &cursor) == AWS_OP_SUCCESS);
    aws_string_destroy(token_file_path_str);

    struct aws_string *config_file_contents = aws_string_new_from_array(allocator, content_buf.buffer, content_buf.len);
    ASSERT_TRUE(config_file_contents != NULL);
    aws_byte_buf_clean_up(&content_buf);

    s_aws_sts_web_identity_test_init_config_profile(allocator, config_file_contents);
    aws_string_destroy(config_file_contents);

    struct aws_byte_cursor good_response_cursor = aws_byte_cursor_from_string(s_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(
        s_verify_credentials(true /*request made*/, true /*from config*/, true /*get creds*/, 1 /*expected attempts*/));

    struct aws_date_time expiration;
    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_good_response_expiration);
    aws_date_time_init_from_str_cursor(&expiration, &date_cursor, AWS_DATE_FORMAT_ISO_8601);
    ASSERT_INT_EQUALS(
        aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials), (uint64_t)expiration.timestamp);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_sts_web_identity_basic_success_config,
    s_credentials_provider_sts_web_identity_basic_success_config);

AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response_first_part,
    "<AssumeRoleWithWebIdentityResponse>"
    "    <AssumeRoleWithWebIdentityResult>"
    "        <AssumedRoleUser>"
    "            <Arn>arn:aws:sts::123456789012:assumed-role/FederatedWebIdentityRole/app1</Arn>"
    "           <AssumedRoleId>AROACLKWSDQRAOEXAMPLE:app1</AssumedRoleId>"
    "        </AssumedRoleUser>"
    "        <Credentials>");
AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response_second_part,
    "            <SessionToken>TokenSuccess</SessionToken>"
    "           <SecretAccessKey>SuccessfulSecret</SecretAccessKey>"
    "            <Expiration>2020-02-25T06:03:31Z</Expiration>"
    "           <AccessKeyId>SuccessfulAccessKey</AccessKeyId>"
    "        </Credentials>"
    "       <Provider>www.amazon.com</Provider>"
    "    </AssumeRoleWithWebIdentityResult>"
    "   <ResponseMetadata>");
AWS_STATIC_STRING_FROM_LITERAL(
    s_good_response_third_part,

    "        <RequestId>ad4156e9-bce1-11e2-82e6-6b6efEXAMPLE</RequestId>"
    "   </ResponseMetadata>"
    "</AssumeRoleWithWebIdentityResponse>");

static int s_credentials_provider_sts_web_identity_success_multi_part_doc(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_web_identity_tester_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    struct aws_byte_cursor good_response_cursor1 = aws_byte_cursor_from_string(s_good_response_first_part);
    struct aws_byte_cursor good_response_cursor2 = aws_byte_cursor_from_string(s_good_response_second_part);
    struct aws_byte_cursor good_response_cursor3 = aws_byte_cursor_from_string(s_good_response_third_part);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor1);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor2);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &good_response_cursor3);

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = NULL,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(
        true /*request made*/, false /*from config*/, true /*get creds*/, 1 /*expected attempts*/));

    struct aws_date_time expiration;
    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_good_response_expiration);
    aws_date_time_init_from_str_cursor(&expiration, &date_cursor, AWS_DATE_FORMAT_ISO_8601);
    ASSERT_INT_EQUALS(
        aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials), (uint64_t)expiration.timestamp);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_sts_web_identity_tester_cleanup();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_success_multi_part_doc,
    s_credentials_provider_sts_web_identity_success_multi_part_doc);

static int s_credentials_provider_sts_web_identity_real_new_destroy(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    s_aws_sts_web_identity_test_unset_env_parameters();

    struct aws_string *token_file_path_str = aws_create_process_unique_file_name(allocator);
    ASSERT_TRUE(token_file_path_str != NULL);
    ASSERT_TRUE(aws_create_profile_file(token_file_path_str, s_sts_web_identity_token_contents) == AWS_OP_SUCCESS);

    s_aws_sts_web_identity_test_init_env_parameters(
        allocator,
        "us-east-1",
        "arn:aws:iam::1234567890:role/test-arn",
        "9876543210",
        aws_string_c_str(token_file_path_str));
    aws_string_destroy(token_file_path_str);

    s_aws_sts_web_identity_tester_init(allocator);

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

    struct aws_credentials_provider_sts_web_identity_options options = {
        .bootstrap = bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_sts_web_identity(allocator, &options);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);
    aws_event_loop_group_release(el_group);

    s_aws_sts_web_identity_tester_cleanup();

    aws_auth_library_clean_up();

    return 0;
}
AWS_TEST_CASE(
    credentials_provider_sts_web_identity_real_new_destroy,
    s_credentials_provider_sts_web_identity_real_new_destroy);
