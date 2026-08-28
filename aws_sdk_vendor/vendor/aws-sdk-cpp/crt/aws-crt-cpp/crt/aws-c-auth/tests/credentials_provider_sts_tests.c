/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/auth/credentials.h>
#include <aws/common/clock.h>
#include <aws/common/condition_variable.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/stream.h>
#include <aws/io/tls_channel_handler.h>

#include <aws/http/connection.h>
#include <aws/http/connection_manager.h>
#include <aws/http/request_response.h>

#include <aws/auth/private/credentials_utils.h>
#include <aws/testing/aws_test_harness.h>

#include "aws/common/byte_buf.h"
#include "credentials_provider_utils.h"
#include "shared_credentials_test_definitions.h"

struct aws_mock_http_request {
    struct aws_byte_buf path;
    struct aws_byte_buf method;
    struct aws_byte_buf host_header;
    struct aws_byte_buf body;
    bool had_auth_header;
    struct aws_byte_buf auth_header;
    int response_code;
};

#define MAX_REQUEST 10
struct aws_mock_sts_tester {
    struct aws_allocator *allocator;
    struct aws_mock_http_request mocked_requests[MAX_REQUEST];
    int num_request;

    int mock_response_code;
    int mock_failure_code;
    size_t fail_operations;

    struct aws_array_list response_data_callbacks;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    struct aws_credentials *credentials;
    bool has_received_credentials_callback;
    int mocked_connection_manager_shutdown_callback_count;
    int expected_connection_manager_shutdown_callback_count;
    int error_code;

    bool fail_connection;

    int provider_shutdown_callback_count;

    struct aws_event_loop_group *el_group;

    struct aws_host_resolver *resolver;

    struct aws_client_bootstrap *bootstrap;

    struct aws_tls_ctx *tls_ctx;
};

static struct aws_mock_sts_tester s_tester;

static void s_on_connection_manager_shutdown_complete(void *user_data) {
    (void)user_data;

    aws_mutex_lock(&s_tester.lock);
    s_tester.mocked_connection_manager_shutdown_callback_count++;
    aws_mutex_unlock(&s_tester.lock);

    aws_condition_variable_notify_one(&s_tester.signal);
}

static bool s_has_tester_received_connection_manager_shutdown_callback(void *user_data) {
    (void)user_data;

    return s_tester.mocked_connection_manager_shutdown_callback_count ==
           s_tester.expected_connection_manager_shutdown_callback_count;
}

static void s_aws_wait_for_connection_manager_shutdown_callback(void) {
    aws_mutex_lock(&s_tester.lock);
    aws_condition_variable_wait_pred(
        &s_tester.signal, &s_tester.lock, s_has_tester_received_connection_manager_shutdown_callback, NULL);
    aws_mutex_unlock(&s_tester.lock);
}

static void s_on_provider_shutdown(void *user_data) {
    (void)user_data;

    aws_mutex_lock(&s_tester.lock);
    s_tester.provider_shutdown_callback_count++;
    aws_mutex_unlock(&s_tester.lock);

    aws_condition_variable_notify_one(&s_tester.signal);
}

static bool s_has_tester_received_provider_shutdown_callback(void *user_data) {
    (void)user_data;

    return s_tester.provider_shutdown_callback_count;
}

static void s_aws_wait_for_provider_shutdown_callback(void) {
    aws_mutex_lock(&s_tester.lock);
    aws_condition_variable_wait_pred(
        &s_tester.signal, &s_tester.lock, s_has_tester_received_provider_shutdown_callback, NULL);
    aws_mutex_unlock(&s_tester.lock);
}

static struct aws_http_connection_manager *s_aws_http_connection_manager_new_mock(
    struct aws_allocator *allocator,
    const struct aws_http_connection_manager_options *options) {
    /* copy the shutdown callback */
    struct aws_shutdown_callback_options *shutdown_callback =
        aws_mem_calloc(allocator, 1, sizeof(struct aws_shutdown_callback_options));
    shutdown_callback->shutdown_callback_fn = options->shutdown_complete_callback;
    shutdown_callback->shutdown_callback_user_data = options->shutdown_complete_user_data;
    return (struct aws_http_connection_manager *)shutdown_callback;
}

static void s_aws_http_connection_manager_release_mock(struct aws_http_connection_manager *manager) {
    (void)manager;
    /* invoke the shutdown callback */
    struct aws_shutdown_callback_options *shutdown_callback = (struct aws_shutdown_callback_options *)manager;
    if (shutdown_callback->shutdown_callback_fn != NULL) {
        shutdown_callback->shutdown_callback_fn(shutdown_callback->shutdown_callback_user_data);
    }
    aws_mem_release(s_tester.allocator, shutdown_callback);
    s_on_connection_manager_shutdown_complete(NULL);
}

static void s_aws_http_connection_manager_acquire_connection_mock(
    struct aws_http_connection_manager *manager,
    aws_http_connection_manager_on_connection_setup_fn *callback,
    void *user_data) {

    (void)manager;
    (void)callback;
    (void)user_data;

    if (!s_tester.fail_connection) {
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
    bool is_request_successful) {

    struct aws_http_header headers[1];
    AWS_ZERO_ARRAY(headers);

    headers[0].name = aws_byte_cursor_from_c_str("some-header");
    headers[0].value = aws_byte_cursor_from_c_str("value");

    if (options->on_response_headers) {
        options->on_response_headers(
            (struct aws_http_stream *)1, AWS_HTTP_HEADER_BLOCK_MAIN, headers, 1, options->user_data);
    }

    if (options->on_response_header_block_done) {
        options->on_response_header_block_done((struct aws_http_stream *)1, true, options->user_data);
    }

    struct aws_byte_cursor data_callback_cur;
    aws_array_list_get_at(&s_tester.response_data_callbacks, &data_callback_cur, s_tester.num_request - 1);
    options->on_response_body((struct aws_http_stream *)1, &data_callback_cur, options->user_data);

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
    struct aws_mock_http_request *mocked_request = &s_tester.mocked_requests[s_tester.num_request++];
    AWS_ZERO_STRUCT(*mocked_request);
    struct aws_byte_cursor path;
    AWS_ZERO_STRUCT(path);
    aws_http_message_get_request_path(options->request, &path);
    aws_byte_buf_clean_up(&mocked_request->path);
    aws_byte_buf_init_copy_from_cursor(&mocked_request->path, s_tester.allocator, path);

    struct aws_byte_cursor method;
    AWS_ZERO_STRUCT(method);
    aws_http_message_get_request_method(options->request, &method);
    aws_byte_buf_clean_up(&mocked_request->method);
    aws_byte_buf_init_copy_from_cursor(&mocked_request->method, s_tester.allocator, method);

    size_t header_count = aws_http_message_get_header_count(options->request);

    for (size_t i = 0; i < header_count; ++i) {
        struct aws_http_header header;
        AWS_ZERO_STRUCT(header);

        aws_http_message_get_header(options->request, &header, i);

        if (aws_byte_cursor_eq_c_str_ignore_case(&header.name, "host")) {
            aws_byte_buf_clean_up(&mocked_request->host_header);
            aws_byte_buf_init_copy_from_cursor(&mocked_request->host_header, s_tester.allocator, header.value);
        }

        if (aws_byte_cursor_eq_c_str_ignore_case(&header.name, "authorization")) {
            mocked_request->had_auth_header = true;
            aws_byte_buf_init_copy_from_cursor(&mocked_request->auth_header, s_tester.allocator, header.value);
        }
    }

    struct aws_input_stream *input_stream = aws_http_message_get_body_stream(options->request);
    int64_t body_len = 0;
    if (input_stream != NULL) {
        aws_input_stream_get_length(input_stream, &body_len);
        aws_byte_buf_clean_up(&mocked_request->body);
        aws_byte_buf_init(&mocked_request->body, s_tester.allocator, (size_t)body_len);
        aws_input_stream_read(input_stream, &mocked_request->body);
    }
    bool fail_request = false;

    if (s_tester.fail_operations) {
        fail_request = true;
        s_tester.fail_operations--;
        mocked_request->response_code = s_tester.mock_failure_code;
    } else {
        mocked_request->response_code = s_tester.mock_response_code;
    }
    s_invoke_mock_request_callbacks(options, !fail_request);

    return (struct aws_http_stream *)1;
}

static int s_aws_http_stream_get_incoming_response_status_mock(
    const struct aws_http_stream *stream,
    int *out_status_code) {
    (void)stream;

    *out_status_code = s_tester.mocked_requests[s_tester.num_request - 1].response_code;

    return AWS_OP_SUCCESS;
}

static int s_aws_http_stream_activate_mock(struct aws_http_stream *stream) {
    (void)stream;
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

AWS_STATIC_STRING_FROM_LITERAL(s_config_file_path_env_variable_name, "AWS_CONFIG_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_region_env_variable_name, "AWS_REGION");
AWS_STATIC_STRING_FROM_LITERAL(s_region_default_env_variable_name, "AWS_DEFAULT_REGION");

static int s_aws_sts_tester_init(struct aws_allocator *allocator) {
    AWS_ZERO_STRUCT(s_tester);
    s_tester.allocator = allocator;
    s_tester.expected_connection_manager_shutdown_callback_count = 1;
    aws_auth_library_init(allocator);

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

    struct aws_tls_ctx_options tls_options;
    aws_tls_ctx_options_init_default_client(&tls_options, allocator);
    s_tester.tls_ctx = aws_tls_client_ctx_new(allocator, &tls_options);
    ASSERT_NOT_NULL(s_tester.tls_ctx);
    aws_tls_ctx_options_clean_up(&tls_options);

    if (aws_array_list_init_dynamic(&s_tester.response_data_callbacks, allocator, 10, sizeof(struct aws_byte_cursor))) {
        return AWS_OP_ERR;
    }

    /*
     * set the environment so that it doesn't mess with tests
     */
    aws_unset_environment_value(s_region_env_variable_name);
    aws_unset_environment_value(s_region_default_env_variable_name);
    struct aws_string *cur_directory = aws_string_new_from_c_str(allocator, ".");
    aws_set_environment_value(s_config_file_path_env_variable_name, cur_directory);
    aws_string_destroy(cur_directory);

    return AWS_OP_SUCCESS;
}

static void s_cleanup_creds_callback_data(void) {
    aws_mutex_lock(&s_tester.lock);
    s_tester.has_received_credentials_callback = false;

    if (s_tester.credentials) {
        aws_credentials_release(s_tester.credentials);
        s_tester.credentials = NULL;
    }

    for (int i = 0; i < MAX_REQUEST; i++) {
        aws_byte_buf_clean_up(&s_tester.mocked_requests[i].path);
        aws_byte_buf_clean_up(&s_tester.mocked_requests[i].method);
        aws_byte_buf_clean_up(&s_tester.mocked_requests[i].host_header);
        aws_byte_buf_clean_up(&s_tester.mocked_requests[i].auth_header);
        aws_byte_buf_clean_up(&s_tester.mocked_requests[i].body);
    }

    aws_mutex_unlock(&s_tester.lock);
}

static int s_aws_sts_tester_cleanup(void) {

    s_cleanup_creds_callback_data();

    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);

    aws_array_list_clean_up(&s_tester.response_data_callbacks);

    aws_client_bootstrap_release(s_tester.bootstrap);
    aws_host_resolver_release(s_tester.resolver);
    aws_event_loop_group_release(s_tester.el_group);
    aws_tls_ctx_release(s_tester.tls_ctx);

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

static struct aws_byte_cursor s_access_key_cur = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("accessKey12345");
static struct aws_byte_cursor s_secret_key_cur = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("secretKey12345");
static struct aws_byte_cursor s_session_token_cur = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("sessionToken123456789");
static struct aws_byte_cursor s_role_arn_cur =
    AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("arn:aws:iam::67895:role/test_role");
static struct aws_byte_cursor s_session_name_cur = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("test_session");
static struct aws_byte_cursor s_external_id_cur = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("externalId123_+=,.@:\\/-");

static struct aws_byte_cursor s_success_creds_doc =
    AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("<AssumeRoleResponse xmlns=\"who cares\">\n"
                                          "     <AssumeRoleResult>\n"
                                          "         <fuzzMePlz>\n"
                                          "         </fuzzMePlz>\n"
                                          "         <Credentials>\n"
                                          "             <AccessKeyId>accessKeyIdResp</AccessKeyId>\n"
                                          "             <SecretAccessKey>secretKeyResp</SecretAccessKey>\n"
                                          "             <SessionToken>sessionTokenResp</SessionToken>\n"
                                          "         </Credentials>\n"
                                          "         <AssumedRoleUser>\n"
                                          "             <AssumedRoleId>assumeRoleIdResp</AssumedRoleId>\n"
                                          "             <Arn>arn:aws:sts::123456789012:assumed-role</Arn>\n"
                                          "         </AssumedRoleUser>\n"
                                          "         ... more stuff we don't care about\n"
                                          "      </AssumeRoleResult>\n"
                                          "</AssumeRoleResponse>");

static struct aws_byte_cursor s_expected_payload =
    AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67895%"
                                          "3Arole%2Ftest_role&RoleSessionName=test_session&DurationSeconds=900");

static struct aws_byte_cursor s_expected_payload_with_external_id =
    AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15"
                                          "&Action=AssumeRole"
                                          "&RoleArn=arn%3Aaws%3Aiam%3A%3A67895%3Arole%2Ftest_role"
                                          "&RoleSessionName=test_session"
                                          "&ExternalId=externalId123_%2B%3D%2C.%40%3A%5C%2F-"
                                          "&DurationSeconds=900");

AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_response, "accessKeyIdResp");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_response, "secretKeyResp");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_response, "sessionTokenResp");
AWS_STATIC_STRING_FROM_LITERAL(s_account_id_response, "123456789012");

static int s_verify_credentials(struct aws_credentials *credentials) {
    ASSERT_NOT_NULL(credentials);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(credentials), s_access_key_id_response);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_secret_access_key(credentials), s_secret_access_key_response);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(credentials), s_session_token_response);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_account_id(credentials), s_account_id_response);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_sts_direct_config_succeeds_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 900);

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);
    const char *expected_auth_header =
        "AWS4-HMAC-SHA256 Credential=accessKey12345/19700101/us-east-1/sts/aws4_request, "
        "SignedHeaders=content-length;content-type;host;x-amz-date;x-amz-security-token, "
        "Signature=06e941853a2cc010ed8ac7072754e0089be8aae59e319924fb0105b20dda77cd";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_auth_header,
        strlen(expected_auth_header),
        s_tester.mocked_requests[0].auth_header.buffer,
        s_tester.mocked_requests[0].auth_header.len);

    const char *expected_host_header = "sts.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_provider_sts_direct_config_succeeds, s_credentials_provider_sts_direct_config_succeeds_fn)

static int s_credentials_provider_sts_direct_config_with_external_id_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .external_id = s_external_id_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload_with_external_id.ptr,
        s_expected_payload_with_external_id.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_with_external_id_succeeds,
    s_credentials_provider_sts_direct_config_with_external_id_succeeds_fn)

static int s_credentials_provider_sts_direct_config_with_region_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);
    /* verify that it picks the s_region_env_variable_name if both are set */
    struct aws_string *region = aws_string_new_from_c_str(allocator, "us-west-2");
    struct aws_string *default_region = aws_string_new_from_c_str(allocator, "us-east-1");
    aws_set_environment_value(s_region_env_variable_name, region);
    aws_set_environment_value(s_region_default_env_variable_name, default_region);
    aws_string_destroy(default_region);
    aws_string_destroy(region);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 900);

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);

    const char *expected_host_header = "sts.us-west-2.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    const char *expected_auth_header =
        "AWS4-HMAC-SHA256 Credential=accessKey12345/19700101/us-west-2/sts/aws4_request, "
        "SignedHeaders=content-length;content-type;host;x-amz-date;x-amz-security-token, "
        "Signature=a4a164e00acb40795ea491092dfcfbbfcda817b6b0bb5d06d328f535ec163ac6";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_auth_header,
        strlen(expected_auth_header),
        s_tester.mocked_requests[0].auth_header.buffer,
        s_tester.mocked_requests[0].auth_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_with_region_succeeds,
    s_credentials_provider_sts_direct_config_with_region_succeeds_fn)

static int s_credentials_provider_sts_direct_config_with_default_region_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);
    struct aws_string *default_region = aws_string_new_from_c_str(allocator, "us-east-1");
    aws_set_environment_value(s_region_default_env_variable_name, default_region);
    aws_string_destroy(default_region);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 900);

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);

    const char *expected_host_header = "sts.us-east-1.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_with_default_region_succeeds,
    s_credentials_provider_sts_direct_config_with_default_region_succeeds_fn)

static int s_credentials_provider_sts_direct_config_with_region_from_config_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_string *config_contents = aws_string_new_from_c_str(allocator, "[profile myProfile]\nregion=cn-north-1");

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("myProfile"),
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 900);

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);
    const char *expected_host_header = "sts.cn-north-1.amazonaws.com.cn";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    aws_file_delete(config_file_str);
    aws_string_destroy(config_file_str);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_with_region_from_config_succeeds,
    s_credentials_provider_sts_direct_config_with_region_from_config_succeeds_fn)

static int s_credentials_provider_sts_direct_config_succeeds_after_retry_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);
    s_tester.mock_response_code = 200;
    s_tester.mock_failure_code = 429;
    s_tester.fail_operations = 2;
    int expected_num_requests = 3;
    for (int i = 0; i < expected_num_requests; i++) {
        aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    }

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 900);

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);

    const char *expected_host_header = "sts.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_succeeds_after_retry,
    s_credentials_provider_sts_direct_config_succeeds_after_retry_fn)

static struct aws_byte_cursor s_malformed_creds_doc =
    AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("<AssumeRoleResponse xmlns=\"who cares\">\n"
                                          "     <AssumeRoleResult>\n"
                                          "         <AssumeRoleUser>\n"
                                          "             <Credentials>\n"
                                          "                 <AccessKeyId>accessKeyIdResp</AccessKeyId>\n"
                                          "                ");

static int s_credentials_provider_sts_direct_config_invalid_doc_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_malformed_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_NULL(s_tester.credentials);

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);

    const char *expected_host_header = "sts.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_invalid_doc,
    s_credentials_provider_sts_direct_config_invalid_doc_fn)

static int s_credentials_provider_sts_direct_config_connection_failed_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);

    s_tester.fail_connection = true;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_NULL(s_tester.credentials);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_connection_failed,
    s_credentials_provider_sts_direct_config_connection_failed_fn)

static int s_credentials_provider_sts_direct_config_service_fails_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    mock_aws_set_system_time(0);
    s_tester.mock_response_code = 529;
    int expected_num_requests = 4;
    for (int i = 0; i < expected_num_requests; i++) {
        aws_array_list_push_back(&s_tester.response_data_callbacks, &s_malformed_creds_doc);
    }

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    aws_credentials_provider_get_credentials(sts_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_NULL(s_tester.credentials);

    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_direct_config_service_fails,
    s_credentials_provider_sts_direct_config_service_fails_fn)

static const char *s_source_profile_config_file = "[default]\n"
                                                  "aws_access_key_id=BLAHBLAH\n"
                                                  "aws_secret_access_key=BLAHBLAHBLAH\n"
                                                  "\n"
                                                  "[roletest]\n"
                                                  "role_arn=arn:aws:iam::67895:role/test_role\n"
                                                  "source_profile=default\n"
                                                  "role_session_name=test_session";

static const char *s_source_profile_external_id_config_file = "[default]\n"
                                                              "aws_access_key_id=BLAHBLAH\n"
                                                              "aws_secret_access_key=BLAHBLAHBLAH\n"
                                                              "\n"
                                                              "[roletest]\n"
                                                              "role_arn=arn:aws:iam::67895:role/test_role\n"
                                                              "source_profile=default\n"
                                                              "role_session_name=test_session\n"
                                                              "external_id=externalId123_+=,.@:\\/-\n";

static const char *s_source_profile_chain_config_file = "[default]\n"
                                                        "aws_access_key_id=BLAHBLAH\n"
                                                        "aws_secret_access_key=BLAHBLAHBLAH\n"
                                                        "\n"
                                                        "[roletest]\n"
                                                        "role_arn=arn:aws:iam::67895:role/test_role\n"
                                                        "source_profile=roletest2\n"
                                                        "role_session_name=test_session\n"
                                                        "[roletest2]\n"
                                                        "role_arn=arn:aws:iam::67896:role/test_role\n"
                                                        "source_profile=roletest3\n"
                                                        "role_session_name=test_session2\n"
                                                        "[roletest3]\n"
                                                        "role_arn=arn:aws:iam::67897:role/test_role\n"
                                                        "source_profile=default\n"
                                                        "role_session_name=test_session3\n";

static int s_credentials_provider_sts_from_profile_config_with_chain_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);
    s_tester.expected_connection_manager_shutdown_callback_count = 3;
    struct aws_string *config_contents = aws_string_new_from_c_str(allocator, s_source_profile_chain_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
        .shutdown_options =
            {
                .shutdown_callback = s_on_provider_shutdown,
            },
    };
    int expected_num_requests = 3;
    for (int i = 0; i < expected_num_requests; i++) {
        aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    }
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    ASSERT_INT_EQUALS(expected_num_requests, s_tester.num_request);
    static struct aws_byte_cursor s_expected_request_body[] = {
        AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67897%"
                                              "3Arole%2Ftest_role&RoleSessionName=test_session3&DurationSeconds=900"),
        AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67896%"
                                              "3Arole%2Ftest_role&RoleSessionName=test_session2&DurationSeconds=900"),
        AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67895%"
                                              "3Arole%2Ftest_role&RoleSessionName=test_session&DurationSeconds=900"),
    };
    const char *expected_method = "POST";
    const char *expected_path = "/";
    const char *expected_host_header = "sts.amazonaws.com";

    for (int i = 0; i < s_tester.num_request; i++) {
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_method,
            strlen(expected_method),
            s_tester.mocked_requests[i].method.buffer,
            s_tester.mocked_requests[i].method.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_path,
            strlen(expected_path),
            s_tester.mocked_requests[i].path.buffer,
            s_tester.mocked_requests[i].path.len);
        ASSERT_TRUE(s_tester.mocked_requests[i].had_auth_header);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_host_header,
            strlen(expected_host_header),
            s_tester.mocked_requests[i].host_header.buffer,
            s_tester.mocked_requests[i].host_header.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            s_expected_request_body[i].ptr,
            s_expected_request_body[i].len,
            s_tester.mocked_requests[i].body.buffer,
            s_tester.mocked_requests[i].body.len);
    }

    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    s_aws_wait_for_provider_shutdown_callback();
    /* There used to be a bug that triggered the shutdown callback multiple times. Sleep for a few seconds
     * and validate that we don't trigger the shutdown callback multiple times */
    aws_thread_current_sleep(3000000000);
    ASSERT_INT_EQUALS(1, s_tester.provider_shutdown_callback_count);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_with_chain,
    s_credentials_provider_sts_from_profile_config_with_chain_fn)

AWS_STATIC_STRING_FROM_LITERAL(s_ecs_creds_env_relative_uri, "AWS_CONTAINER_CREDENTIALS_RELATIVE_URI");
static const char *s_source_credentials_ecs_config_file = "[default]\n"
                                                          "role_arn=arn:aws:iam::67895:role/test_role\n"
                                                          "credential_source=EcsContainer\n"
                                                          "role_session_name=test_session\n";
static struct aws_byte_cursor s_ecs_good_response = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
    "{\"AccessKeyId\":\"SuccessfulAccessKey\", \n  \"SecretAccessKey\":\"SuccessfulSecret\", \n  "
    "\"Token\":\"TokenSuccess\", \n \"Expiration\":\"2020-02-25T06:03:31Z\"}");
static int s_credentials_provider_sts_from_profile_config_with_ecs_credentials_source_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    /* set the relative uri environment variable*/
    char *relative_uri = "/test";
    struct aws_string *relative_uri_str = aws_string_new_from_c_str(allocator, relative_uri);
    ASSERT_SUCCESS(aws_set_environment_value(s_ecs_creds_env_relative_uri, relative_uri_str));

    s_aws_sts_tester_init(allocator);
    /* one for ecs provider and one for sts provider */
    s_tester.expected_connection_manager_shutdown_callback_count = 2;
    struct aws_string *config_contents = aws_string_new_from_c_str(allocator, s_source_credentials_ecs_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
    };
    int expected_num_requests = 2;
    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_ecs_good_response);
    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);

    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    ASSERT_INT_EQUALS(expected_num_requests, s_tester.num_request);
    struct aws_mock_http_request expected_mocked_request[2];
    /* expected ecs request*/
    expected_mocked_request[0] = (struct aws_mock_http_request){
        .method = aws_byte_buf_from_c_str("GET"),
        .host_header = aws_byte_buf_from_c_str("169.254.170.2"),
        .path = aws_byte_buf_from_c_str(relative_uri),
        .body = aws_byte_buf_from_c_str(""),
        .had_auth_header = false,
    };
    /* expected sts request */
    expected_mocked_request[1] = (struct aws_mock_http_request){
        .method = aws_byte_buf_from_c_str("POST"),
        .host_header = aws_byte_buf_from_c_str("sts.amazonaws.com"),
        .path = aws_byte_buf_from_c_str("/"),
        .body = aws_byte_buf_from_c_str("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67895%"
                                        "3Arole%2Ftest_role&RoleSessionName=test_session&DurationSeconds=900"),
        .had_auth_header = true,
    };

    for (int i = 0; i < expected_num_requests; i++) {
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_mocked_request[i].method.buffer,
            expected_mocked_request[i].method.len,
            s_tester.mocked_requests[i].method.buffer,
            s_tester.mocked_requests[i].method.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_mocked_request[i].path.buffer,
            expected_mocked_request[i].path.len,
            s_tester.mocked_requests[i].path.buffer,
            s_tester.mocked_requests[i].path.len);
        ASSERT_TRUE(s_tester.mocked_requests[i].had_auth_header == expected_mocked_request[i].had_auth_header);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_mocked_request[i].host_header.buffer,
            expected_mocked_request[i].host_header.len,
            s_tester.mocked_requests[i].host_header.buffer,
            s_tester.mocked_requests[i].host_header.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_mocked_request[i].body.buffer,
            expected_mocked_request[i].body.len,
            s_tester.mocked_requests[i].body.buffer,
            s_tester.mocked_requests[i].body.len);
    }

    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_string_destroy(relative_uri_str);
    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_with_ecs_credentials_source,
    s_credentials_provider_sts_from_profile_config_with_ecs_credentials_source_fn)

static const char *s_source_profile_chain_and_profile_config_file = "[default]\n"
                                                                    "aws_access_key_id=BLAHBLAH\n"
                                                                    "aws_secret_access_key=BLAHBLAHBLAH\n"
                                                                    "\n"
                                                                    "[roletest]\n"
                                                                    "role_arn=arn:aws:iam::67895:role/test_role\n"
                                                                    "source_profile=roletest2\n"
                                                                    "role_session_name=test_session\n"
                                                                    "[roletest2]\n"
                                                                    "role_arn=arn:aws:iam::67896:role/test_role\n"
                                                                    "source_profile=roletest3\n"
                                                                    "role_session_name=test_session2\n"
                                                                    "[roletest3]\n"
                                                                    "role_arn=arn:aws:iam::67897:role/test_role\n"
                                                                    "source_profile=default\n"
                                                                    "role_session_name=test_session3\n"
                                                                    "aws_access_key_id = BLAH\n"
                                                                    "aws_secret_access_key = BLAHBLAH\n";
static int s_credentials_provider_sts_from_profile_config_with_chain_and_profile_creds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);
    s_tester.expected_connection_manager_shutdown_callback_count = 2;
    struct aws_string *config_contents =
        aws_string_new_from_c_str(allocator, s_source_profile_chain_and_profile_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
    };

    int expected_num_requests = 2;
    for (int i = 0; i < expected_num_requests; i++) {
        aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    }
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    ASSERT_INT_EQUALS(expected_num_requests, s_tester.num_request);
    static struct aws_byte_cursor s_expected_request_body[] = {
        AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67896%"
                                              "3Arole%2Ftest_role&RoleSessionName=test_session2&DurationSeconds=900"),
        AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67895%"
                                              "3Arole%2Ftest_role&RoleSessionName=test_session&DurationSeconds=900"),
    };
    const char *expected_method = "POST";
    const char *expected_path = "/";
    const char *expected_host_header = "sts.amazonaws.com";

    for (int i = 0; i < s_tester.num_request; i++) {
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_method,
            strlen(expected_method),
            s_tester.mocked_requests[i].method.buffer,
            s_tester.mocked_requests[i].method.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_path,
            strlen(expected_path),
            s_tester.mocked_requests[i].path.buffer,
            s_tester.mocked_requests[i].path.len);
        ASSERT_TRUE(s_tester.mocked_requests[i].had_auth_header);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_host_header,
            strlen(expected_host_header),
            s_tester.mocked_requests[i].host_header.buffer,
            s_tester.mocked_requests[i].host_header.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            s_expected_request_body[i].ptr,
            s_expected_request_body[i].len,
            s_tester.mocked_requests[i].body.buffer,
            s_tester.mocked_requests[i].body.len);
    }

    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_with_chain_and_profile_creds,
    s_credentials_provider_sts_from_profile_config_with_chain_and_profile_creds_fn)

static const char *s_source_profile_chain_and_partial_profile_config_file =
    "[default]\n"
    "aws_access_key_id=BLAHBLAH\n"
    "aws_secret_access_key=BLAHBLAHBLAH\n"
    "\n"
    "[roletest]\n"
    "role_arn=arn:aws:iam::67895:role/test_role\n"
    "source_profile=roletest2\n"
    "role_session_name=test_session\n"
    "[roletest2]\n"
    "role_arn=arn:aws:iam::67896:role/test_role\n"
    "source_profile=roletest3\n"
    "role_session_name=test_session2\n"
    "[roletest3]\n"
    "role_arn=arn:aws:iam::67897:role/test_role\n"
    "source_profile=default\n"
    "role_session_name=test_session3\n"
    "aws_access_key_id = BLAH\n";
static int s_credentials_provider_sts_from_profile_config_with_chain_and_partial_profile_creds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);
    s_tester.expected_connection_manager_shutdown_callback_count = 2;
    struct aws_string *config_contents =
        aws_string_new_from_c_str(allocator, s_source_profile_chain_and_partial_profile_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
    };

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_NULL(s_tester.credentials);
    ASSERT_INT_EQUALS(s_tester.error_code, AWS_AUTH_SIGNING_NO_CREDENTIALS);
    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_with_chain_and_partial_profile_creds,
    s_credentials_provider_sts_from_profile_config_with_chain_and_partial_profile_creds_fn)

static const char *s_source_profile_self_assume_role_config_file = "[default]\n"
                                                                   "aws_access_key_id=BLAHBLAH\n"
                                                                   "aws_secret_access_key=BLAHBLAHBLAH\n"
                                                                   "\n"
                                                                   "[roletest]\n"
                                                                   "role_arn=arn:aws:iam::67895:role/test_role\n"
                                                                   "source_profile=roletest\n"
                                                                   "role_session_name=test_session\n"
                                                                   "aws_access_key_id = BLAH\n"
                                                                   "aws_secret_access_key = BLAHBLAH\n";
static int s_credentials_provider_sts_from_self_referencing_profile_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);
    struct aws_string *config_contents =
        aws_string_new_from_c_str(allocator, s_source_profile_self_assume_role_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
    };

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    ASSERT_INT_EQUALS(1, s_tester.num_request);
    static struct aws_byte_cursor s_expected_request_body[] = {
        AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Version=2011-06-15&Action=AssumeRole&RoleArn=arn%3Aaws%3Aiam%3A%3A67895%"
                                              "3Arole%2Ftest_role&RoleSessionName=test_session&DurationSeconds=900"),
    };
    const char *expected_method = "POST";
    const char *expected_path = "/";
    const char *expected_host_header = "sts.amazonaws.com";

    for (int i = 0; i < s_tester.num_request; i++) {
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_method,
            strlen(expected_method),
            s_tester.mocked_requests[i].method.buffer,
            s_tester.mocked_requests[i].method.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_path,
            strlen(expected_path),
            s_tester.mocked_requests[i].path.buffer,
            s_tester.mocked_requests[i].path.len);
        ASSERT_TRUE(s_tester.mocked_requests[i].had_auth_header);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_host_header,
            strlen(expected_host_header),
            s_tester.mocked_requests[i].host_header.buffer,
            s_tester.mocked_requests[i].host_header.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            s_expected_request_body[i].ptr,
            s_expected_request_body[i].len,
            s_tester.mocked_requests[i].body.buffer,
            s_tester.mocked_requests[i].body.len);
    }

    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_self_referencing_profile,
    s_credentials_provider_sts_from_self_referencing_profile_fn)

static const char *s_source_profile_chain_cycle_config_file = "[default]\n"
                                                              "aws_access_key_id=BLAHBLAH\n"
                                                              "aws_secret_access_key=BLAHBLAHBLAH\n"
                                                              "\n"
                                                              "[roletest]\n"
                                                              "role_arn=arn:aws:iam::67895:role/test_role\n"
                                                              "source_profile=roletest2\n"
                                                              "role_session_name=test_session\n"
                                                              "[roletest2]\n"
                                                              "role_arn=arn:aws:iam::67896:role/test_role\n"
                                                              "source_profile=roletest3\n"
                                                              "role_session_name=test_session2\n"
                                                              "[roletest3]\n"
                                                              "role_arn=arn:aws:iam::67897:role/test_role\n"
                                                              "source_profile=roletest2\n"
                                                              "role_session_name=test_session3\n";

static int s_credentials_provider_sts_from_profile_config_with_chain_cycle_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);

    struct aws_string *config_contents = aws_string_new_from_c_str(allocator, s_source_profile_chain_cycle_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
    };

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NULL(provider);
    ASSERT_INT_EQUALS(AWS_AUTH_PROFILE_STS_CREDENTIALS_PROVIDER_CYCLE_FAILURE, aws_last_error());

    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());
    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_with_chain_cycle,
    s_credentials_provider_sts_from_profile_config_with_chain_cycle_fn)

static const char *s_source_profile_chain_cycle_and_static_creds_config_file =
    "[roletest]\n"
    "role_arn=arn:aws:iam::67895:role/test_role\n"
    "source_profile=roletest2\n"
    "role_session_name=test_session\n"
    "aws_access_key_id=BLAHBLAH\n"
    "aws_secret_access_key=BLAHBLAHBLAH\n"
    "[roletest2]\n"
    "role_arn=arn:aws:iam::67896:role/test_role\n"
    "source_profile=roletest3\n"
    "role_session_name=test_session2\n"
    "[roletest3]\n"
    "role_arn=arn:aws:iam::67897:role/test_role\n"
    "source_profile=roletest\n"
    "role_session_name=test_session3\n";

static int s_credentials_provider_sts_from_profile_config_with_chain_cycle_and_profile_creds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);

    struct aws_string *config_contents =
        aws_string_new_from_c_str(allocator, s_source_profile_chain_cycle_and_static_creds_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .function_table = &s_mock_function_table,
    };

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NULL(provider);
    ASSERT_INT_EQUALS(AWS_AUTH_PROFILE_STS_CREDENTIALS_PROVIDER_CYCLE_FAILURE, aws_last_error());

    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());
    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_with_chain_cycle_and_profile_creds,
    s_credentials_provider_sts_from_profile_config_with_chain_cycle_and_profile_creds_fn)

static int s_credentials_provider_sts_from_profile_config_succeeds(
    struct aws_allocator *allocator,
    void *ctx,
    bool manual_tls) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);

    struct aws_string *config_contents = aws_string_new_from_c_str(allocator, s_source_profile_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        /* tls_ctx is optional, test it both ways */
        .tls_ctx = manual_tls ? s_tester.tls_ctx : NULL,
        .function_table = &s_mock_function_table,
    };

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);

    const char *expected_host_header = "sts.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_sts_from_profile_config_succeeds_fn(struct aws_allocator *allocator, void *ctx) {
    return s_credentials_provider_sts_from_profile_config_succeeds(allocator, ctx, false /*manual_tls*/);
}

AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_succeeds,
    s_credentials_provider_sts_from_profile_config_succeeds_fn)

static int credentials_provider_sts_from_profile_config_manual_tls_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    return s_credentials_provider_sts_from_profile_config_succeeds(allocator, ctx, true /*manual_tls*/);
}

AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_manual_tls_succeeds,
    credentials_provider_sts_from_profile_config_manual_tls_succeeds_fn)

static int s_credentials_provider_sts_from_profile_config_with_external_id_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    s_aws_sts_tester_init(allocator);

    struct aws_string *config_contents = aws_string_new_from_c_str(allocator, s_source_profile_external_id_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
    };

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload_with_external_id.ptr,
        s_expected_payload_with_external_id.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_with_external_id,
    s_credentials_provider_sts_from_profile_config_with_external_id_fn)

static const char *s_env_source_config_file = "[default]\n"
                                              "aws_access_key_id=BLAHBLAH\n"
                                              "aws_secret_access_key=BLAHBLAHBLAH\n"
                                              "\n"
                                              "[roletest]\n"
                                              "role_arn=arn:aws:iam::67895:role/test_role\n"
                                              "credential_source=Environment\n"
                                              "role_session_name=test_session";

AWS_STRING_FROM_LITERAL(s_env_access_key_val, "EnvAccessKeyId");
AWS_STRING_FROM_LITERAL(s_env_secret_access_key_val, "EnvSecretAccessKeyId");

static int s_credentials_provider_sts_from_profile_config_environment_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_unset_environment_value(s_default_profile_env_variable_name);
    aws_unset_environment_value(s_default_config_path_env_variable_name);
    aws_unset_environment_value(s_default_credentials_path_env_variable_name);

    aws_set_environment_value(s_access_key_id_env_var, s_env_access_key_val);
    aws_set_environment_value(s_secret_access_key_env_var, s_env_secret_access_key_val);

    s_aws_sts_tester_init(allocator);

    struct aws_string *config_contents = aws_string_new_from_c_str(allocator, s_env_source_config_file);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    ASSERT_SUCCESS(aws_create_profile_file(creds_file_str, config_contents));
    aws_string_destroy(config_contents);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_c_str("roletest"),
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .function_table = &s_mock_function_table,
    };

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    aws_string_destroy(creds_file_str);
    aws_string_destroy(config_file_str);

    aws_credentials_provider_get_credentials(provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);

    const char *expected_host_header = "sts.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    aws_credentials_provider_release(provider);
    s_aws_wait_for_connection_manager_shutdown_callback();

    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    credentials_provider_sts_from_profile_config_environment_succeeds,
    s_credentials_provider_sts_from_profile_config_environment_succeeds_fn)

#define HIGH_RES_BASE_TIME_NS 101000000000ULL

/*
 * In this test, we set up a cached provider with a longer and out-of-sync refresh period than the sts
 * provider that it wraps.  We verify that the cached provider factors in the shorter-lived sts credentials
 * properly and refreshes when the credentials expire and not when the cache would expire.
 */
static int s_credentials_provider_sts_cache_expiration_conflict(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_sts_tester_init(allocator);

    struct aws_credentials_provider_static_options static_options = {
        .access_key_id = s_access_key_cur,
        .secret_access_key = s_secret_key_cur,
        .session_token = s_session_token_cur,
    };
    struct aws_credentials_provider *static_provider = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider_sts_options options = {
        .creds_provider = static_provider,
        .bootstrap = s_tester.bootstrap,
        .tls_ctx = s_tester.tls_ctx,
        .role_arn = s_role_arn_cur,
        .session_name = s_session_name_cur,
        .duration_seconds = 0,
        .function_table = &s_mock_function_table,
        .system_clock_fn = mock_aws_get_system_time,
    };

    /* make sure high res time and system time are sufficiently diverged that a mistake in the
     * respective calculations would fail the test */
    mock_aws_set_system_time(0);
    mock_aws_set_high_res_time(HIGH_RES_BASE_TIME_NS);

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    s_tester.mock_response_code = 200;

    struct aws_credentials_provider *sts_provider = aws_credentials_provider_new_sts(allocator, &options);

    struct aws_credentials_provider_cached_options cached_options = {
        .system_clock_fn = mock_aws_get_system_time,
        .high_res_clock_fn = mock_aws_get_high_res_time,
        .refresh_time_in_milliseconds = 1200 * 1000,
        .source = sts_provider,
    };
    struct aws_credentials_provider *cached_provider = aws_credentials_provider_new_cached(allocator, &cached_options);

    aws_credentials_provider_get_credentials(cached_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();

    ASSERT_SUCCESS(s_verify_credentials(s_tester.credentials));
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 900);

    const char *expected_method = "POST";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_method,
        strlen(expected_method),
        s_tester.mocked_requests[0].method.buffer,
        s_tester.mocked_requests[0].method.len);

    const char *expected_path = "/";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_path,
        strlen(expected_path),
        s_tester.mocked_requests[0].path.buffer,
        s_tester.mocked_requests[0].path.len);

    ASSERT_TRUE(s_tester.mocked_requests[0].had_auth_header);

    const char *expected_host_header = "sts.amazonaws.com";
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_host_header,
        strlen(expected_host_header),
        s_tester.mocked_requests[0].host_header.buffer,
        s_tester.mocked_requests[0].host_header.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_payload.ptr,
        s_expected_payload.len,
        s_tester.mocked_requests[0].body.buffer,
        s_tester.mocked_requests[0].body.len);

    /* advance each time to a little before expiration, verify we get creds with the same expiration */
    uint64_t before_expiration_time = aws_timestamp_convert(599, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);
    mock_aws_set_system_time(before_expiration_time);
    mock_aws_set_high_res_time(HIGH_RES_BASE_TIME_NS + before_expiration_time);

    s_cleanup_creds_callback_data();

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    aws_credentials_provider_get_credentials(cached_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 900);

    /* advance each time to after expiration but before cached provider timeout, verify we get new creds */
    uint64_t after_expiration_time = aws_timestamp_convert(901, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);
    mock_aws_set_system_time(after_expiration_time);
    mock_aws_set_high_res_time(HIGH_RES_BASE_TIME_NS + after_expiration_time);

    s_cleanup_creds_callback_data();

    aws_array_list_push_back(&s_tester.response_data_callbacks, &s_success_creds_doc);
    aws_credentials_provider_get_credentials(cached_provider, s_get_credentials_callback, NULL);

    s_aws_wait_for_credentials_result();
    ASSERT_TRUE(aws_credentials_get_expiration_timepoint_seconds(s_tester.credentials) == 1801);

    aws_credentials_provider_release(cached_provider);
    aws_credentials_provider_release(sts_provider);
    s_aws_wait_for_connection_manager_shutdown_callback();
    aws_credentials_provider_release(static_provider);

    ASSERT_SUCCESS(s_aws_sts_tester_cleanup());

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_provider_sts_cache_expiration_conflict, s_credentials_provider_sts_cache_expiration_conflict)
