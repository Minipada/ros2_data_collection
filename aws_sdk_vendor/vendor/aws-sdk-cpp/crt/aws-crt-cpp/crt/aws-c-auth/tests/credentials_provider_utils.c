/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "credentials_provider_utils.h"
#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/credentials_utils.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>
#include <aws/common/task_scheduler.h>
#include <aws/common/uuid.h>
#include <aws/http/status_code.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/file_utils.h>
#include <aws/io/stream.h>
#include <aws/io/tls_channel_handler.h>

#include <errno.h>

AWS_STATIC_STRING_FROM_LITERAL(s_home_env_var, "HOME");

/*
 * Support for async get testing
 */
void aws_get_credentials_test_callback_result_init(
    struct aws_get_credentials_test_callback_result *result,
    int required_count) {
    AWS_ZERO_STRUCT(*result);
    aws_mutex_init(&result->sync);
    aws_condition_variable_init(&result->signal);
    result->required_count = required_count;
}

void aws_get_credentials_test_callback_result_clean_up(struct aws_get_credentials_test_callback_result *result) {

    if (result->credentials) {
        aws_credentials_release(result->credentials);
    }

    aws_condition_variable_clean_up(&result->signal);
    aws_mutex_clean_up(&result->sync);
}

void aws_test_get_credentials_async_callback(struct aws_credentials *credentials, int error_code, void *user_data) {
    struct aws_get_credentials_test_callback_result *result =
        (struct aws_get_credentials_test_callback_result *)user_data;

    aws_mutex_lock(&result->sync);

    result->count++;
    result->last_error = error_code;

    if (result->credentials != NULL) {
        aws_credentials_release(result->credentials);
    }

    result->credentials = credentials;
    if (credentials != NULL) {
        aws_credentials_acquire(credentials);
    }

    aws_condition_variable_notify_one(&result->signal);

    aws_mutex_unlock(&result->sync);
}

static bool s_sync_credentials_predicate(void *context) {
    struct aws_get_credentials_test_callback_result *result =
        (struct aws_get_credentials_test_callback_result *)context;

    return result->count == result->required_count;
}

void aws_wait_on_credentials_callback(struct aws_get_credentials_test_callback_result *result) {
    bool done = false;
    while (!done) {
        aws_mutex_lock(&result->sync);

        aws_condition_variable_wait_pred(&result->signal, &result->sync, s_sync_credentials_predicate, result);

        done = result->count == result->required_count;
        aws_mutex_unlock(&result->sync);
    }
}

/*
 * Mock provider
 */
struct aws_credentials_provider_mock_impl {
    struct aws_array_list results;
    size_t next_result;
};

static int s_mock_credentials_provider_get_credentials_async(
    struct aws_credentials_provider *provider,
    aws_on_get_credentials_callback_fn callback,
    void *user_data) {
    struct aws_credentials_provider_mock_impl *impl = (struct aws_credentials_provider_mock_impl *)provider->impl;

    if (impl->next_result < aws_array_list_length(&impl->results)) {
        struct get_credentials_mock_result result;
        if (aws_array_list_get_at(&impl->results, &result, impl->next_result)) {
            AWS_FATAL_ASSERT(false);
        } else {
            callback(result.credentials, result.error_code, user_data);
        }
        impl->next_result++;
    } else {
        AWS_FATAL_ASSERT(false);
    }

    return AWS_OP_SUCCESS;
}

static void s_mock_credentials_provider_destroy(struct aws_credentials_provider *provider) {
    struct aws_credentials_provider_mock_impl *impl = (struct aws_credentials_provider_mock_impl *)provider->impl;

    aws_credentials_provider_invoke_shutdown_callback(provider);

    aws_array_list_clean_up(&impl->results);

    aws_mem_release(provider->allocator, provider);
}

static struct aws_credentials_provider_vtable s_aws_credentials_provider_mock_vtable = {
    .get_credentials = s_mock_credentials_provider_get_credentials_async,
    .destroy = s_mock_credentials_provider_destroy,
};

struct aws_credentials_provider *aws_credentials_provider_new_mock(
    struct aws_allocator *allocator,
    struct get_credentials_mock_result *results,
    size_t result_count,
    struct aws_credentials_provider_shutdown_options *shutdown_options) {

    struct aws_credentials_provider *provider = NULL;
    struct aws_credentials_provider_mock_impl *impl = NULL;

    aws_mem_acquire_many(
        allocator,
        2,
        &provider,
        sizeof(struct aws_credentials_provider),
        &impl,
        sizeof(struct aws_credentials_provider_mock_impl));

    if (!provider) {
        return NULL;
    }

    AWS_ZERO_STRUCT(*provider);
    AWS_ZERO_STRUCT(*impl);

    if (aws_array_list_init_dynamic(
            &impl->results, allocator, result_count, sizeof(struct get_credentials_mock_result))) {
        goto on_init_result_list_failure;
    }

    for (size_t i = 0; i < result_count; ++i) {
        aws_array_list_push_back(&impl->results, results + i);
    }

    provider->allocator = allocator;
    provider->vtable = &s_aws_credentials_provider_mock_vtable;
    provider->impl = impl;
    if (shutdown_options) {
        provider->shutdown_options = *shutdown_options;
    }
    aws_atomic_store_int(&provider->ref_count, 1);

    return provider;

on_init_result_list_failure:
    aws_mem_release(allocator, provider);

    return NULL;
}

/*
 * Mock async provider
 */

struct aws_credentials_provider_mock_async_impl {
    struct aws_event_loop_group *event_loop_group;
    struct aws_mutex sync;
    struct aws_array_list queries;
    struct aws_array_list mock_results;
    size_t next_result;
};

static int s_async_mock_credentials_provider_get_credentials_async(
    struct aws_credentials_provider *provider,
    aws_on_get_credentials_callback_fn callback,
    void *user_data) {
    struct aws_credentials_provider_mock_async_impl *impl =
        (struct aws_credentials_provider_mock_async_impl *)provider->impl;

    aws_mutex_lock(&impl->sync);

    struct aws_credentials_query query;
    AWS_ZERO_STRUCT(query);

    aws_credentials_query_init(&query, provider, callback, user_data);

    aws_array_list_push_back(&impl->queries, &query);

    aws_mutex_unlock(&impl->sync);

    return AWS_OP_SUCCESS;
}

static void s_async_mock_credentials_provider_destroy(struct aws_credentials_provider *provider) {
    struct aws_credentials_provider_mock_async_impl *impl =
        (struct aws_credentials_provider_mock_async_impl *)provider->impl;

    aws_array_list_clean_up(&impl->queries);
    aws_array_list_clean_up(&impl->mock_results);

    aws_event_loop_group_release(impl->event_loop_group);

    aws_mutex_clean_up(&impl->sync);

    aws_credentials_provider_invoke_shutdown_callback(provider);

    aws_mem_release(provider->allocator, provider);
}

static struct aws_credentials_provider_vtable s_aws_credentials_provider_mock_async_vtable = {
    .get_credentials = s_async_mock_credentials_provider_get_credentials_async,
    .destroy = s_async_mock_credentials_provider_destroy,
};

static void s_async_mock_credentials_provider_fire_callbacks_task(
    struct aws_task *task,
    void *arg,
    enum aws_task_status status) {

    (void)status;
    struct aws_credentials_provider *provider = arg;
    struct aws_credentials_provider_mock_async_impl *impl = provider->impl;

    aws_mem_release(provider->allocator, task);

    aws_mutex_lock(&impl->sync);

    /*
     * We need to make all of our callbacks outside the lock, in order to avoid deadlock
     * To make that easier, we keep this array list around and swap the callbacks we need to make into it
     */
    struct aws_array_list temp_queries;
    AWS_FATAL_ASSERT(
        aws_array_list_init_dynamic(&temp_queries, impl->queries.alloc, 10, sizeof(struct aws_credentials_query)) ==
        AWS_OP_SUCCESS);

    struct get_credentials_mock_result result;
    AWS_ZERO_STRUCT(result);

    size_t callback_count = aws_array_list_length(&impl->queries);
    if (callback_count != 0) {
        size_t result_count = aws_array_list_length(&impl->mock_results);
        if (impl->next_result >= result_count ||
            aws_array_list_get_at(&impl->mock_results, &result, impl->next_result)) {
            AWS_FATAL_ASSERT(false);
        }
        impl->next_result++;

        /*
         * move the callbacks we need to complete into the temporary list so that we can
         * safely use them outside the lock (we cannot safely use impl->queries outside the lock)
         */
        aws_array_list_swap_contents(&impl->queries, &temp_queries);
    }
    aws_mutex_unlock(&impl->sync);

    /* make the callbacks, not holding the lock */
    for (size_t i = 0; i < callback_count; ++i) {
        struct aws_credentials_query query;
        AWS_ZERO_STRUCT(query);
        if (aws_array_list_get_at(&temp_queries, &query, i)) {
            continue;
        }

        AWS_FATAL_ASSERT(query.callback != NULL);
        query.callback(result.credentials, result.error_code, query.user_data);

        aws_credentials_query_clean_up(&query);
    }

    aws_array_list_clean_up(&temp_queries);
    aws_credentials_provider_release(provider);
}

void aws_credentials_provider_mock_async_fire_callbacks(struct aws_credentials_provider *provider) {
    struct aws_credentials_provider_mock_async_impl *impl = provider->impl;

    struct aws_task *task = aws_mem_calloc(provider->allocator, 1, sizeof(struct aws_task));
    AWS_FATAL_ASSERT(task);
    aws_task_init(
        task,
        s_async_mock_credentials_provider_fire_callbacks_task,
        provider,
        "async_mock_credentials_provider_fire_callbacks_task");

    /* keep provider alive until task runs */
    aws_credentials_provider_acquire(provider);

    struct aws_event_loop *loop = aws_event_loop_group_get_next_loop(impl->event_loop_group);
    aws_event_loop_schedule_task_now(loop, task);
}

struct aws_credentials_provider *aws_credentials_provider_new_mock_async(
    struct aws_allocator *allocator,
    struct get_credentials_mock_result *results,
    size_t result_count,
    struct aws_event_loop_group *elg,
    struct aws_credentials_provider_shutdown_options *shutdown_options) {

    struct aws_credentials_provider *provider = NULL;
    struct aws_credentials_provider_mock_async_impl *impl = NULL;

    aws_mem_acquire_many(
        allocator,
        2,
        &provider,
        sizeof(struct aws_credentials_provider),
        &impl,
        sizeof(struct aws_credentials_provider_mock_async_impl));

    if (!provider) {
        return NULL;
    }

    AWS_ZERO_STRUCT(*provider);
    AWS_ZERO_STRUCT(*impl);

    if (aws_mutex_init(&impl->sync)) {
        goto on_lock_init_failure;
    }

    if (aws_array_list_init_dynamic(&impl->queries, allocator, 10, sizeof(struct aws_credentials_query))) {
        goto on_query_list_init_failure;
    }

    if (aws_array_list_init_dynamic(
            &impl->mock_results, allocator, result_count, sizeof(struct get_credentials_mock_result))) {
        goto on_mock_result_list_init_failure;
    }

    for (size_t i = 0; i < result_count; ++i) {
        aws_array_list_push_back(&impl->mock_results, results + i);
    }

    impl->event_loop_group = aws_event_loop_group_acquire(elg);

    provider->allocator = allocator;
    provider->vtable = &s_aws_credentials_provider_mock_async_vtable;
    provider->impl = impl;
    if (shutdown_options) {
        provider->shutdown_options = *shutdown_options;
    }
    aws_atomic_store_int(&provider->ref_count, 1);

    return provider;

on_mock_result_list_init_failure:
    aws_array_list_clean_up(&impl->queries);

on_query_list_init_failure:
    aws_mutex_clean_up(&impl->sync);

on_lock_init_failure:
    aws_mem_release(allocator, provider);

    return NULL;
}

/*
 * mock system clock
 */

static struct aws_mutex system_clock_sync = AWS_MUTEX_INIT;
static uint64_t system_clock_time = 0;

int mock_aws_get_system_time(uint64_t *current_time) {
    aws_mutex_lock(&system_clock_sync);

    *current_time = system_clock_time;

    aws_mutex_unlock(&system_clock_sync);

    return AWS_OP_SUCCESS;
}

void mock_aws_set_system_time(uint64_t current_time) {
    aws_mutex_lock(&system_clock_sync);

    system_clock_time = current_time;

    aws_mutex_unlock(&system_clock_sync);
}

/*
 * mock high res clock
 */

static struct aws_mutex high_res_clock_sync = AWS_MUTEX_INIT;
static uint64_t high_res_clock_time = 0;

int mock_aws_get_high_res_time(uint64_t *current_time) {
    aws_mutex_lock(&high_res_clock_sync);

    *current_time = high_res_clock_time;

    aws_mutex_unlock(&high_res_clock_sync);

    return AWS_OP_SUCCESS;
}

void mock_aws_set_high_res_time(uint64_t current_time) {
    aws_mutex_lock(&high_res_clock_sync);

    high_res_clock_time = current_time;

    aws_mutex_unlock(&high_res_clock_sync);
}

/*
 * Null provider impl
 */

static int s_credentials_provider_null_get_credentials_async(
    struct aws_credentials_provider *provider,
    aws_on_get_credentials_callback_fn callback,
    void *user_data) {
    (void)provider;
    callback(NULL, AWS_ERROR_UNKNOWN, user_data);

    return AWS_OP_SUCCESS;
}

static void s_credentials_provider_null_destroy(struct aws_credentials_provider *provider) {
    aws_credentials_provider_invoke_shutdown_callback(provider);

    aws_mem_release(provider->allocator, provider);
}

static struct aws_credentials_provider_vtable s_aws_credentials_provider_null_vtable = {
    .get_credentials = s_credentials_provider_null_get_credentials_async,
    .destroy = s_credentials_provider_null_destroy,
};

struct aws_credentials_provider *aws_credentials_provider_new_null(
    struct aws_allocator *allocator,
    struct aws_credentials_provider_shutdown_options *shutdown_options) {
    struct aws_credentials_provider *provider =
        (struct aws_credentials_provider *)aws_mem_acquire(allocator, sizeof(struct aws_credentials_provider));
    if (provider == NULL) {
        return NULL;
    }

    AWS_ZERO_STRUCT(*provider);

    provider->allocator = allocator;
    provider->vtable = &s_aws_credentials_provider_null_vtable;
    provider->impl = NULL;
    if (shutdown_options) {
        provider->shutdown_options = *shutdown_options;
    }
    aws_atomic_store_int(&provider->ref_count, 1);

    return provider;
}

int aws_create_directory_components(struct aws_allocator *allocator, const struct aws_string *path) {
    const char local_platform_separator = aws_get_platform_directory_separator();

    /* Create directory components and ensure use of platform separator at the same time. */
    for (size_t i = 0; i < path->len; ++i) {
        if (aws_is_any_directory_separator((char)path->bytes[i])) {
            ((char *)path->bytes)[i] = local_platform_separator;

            struct aws_string *segment = aws_string_new_from_array(allocator, path->bytes, i);
            int rc = aws_directory_create(segment);
            aws_string_destroy(segment);

            if (rc != AWS_OP_SUCCESS) {
                return rc;
            }
        }
    }
    return AWS_OP_SUCCESS;
}

int aws_create_random_home_directory(struct aws_allocator *allocator, struct aws_string **out_path) {
    struct aws_byte_buf path_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&path_buf, allocator, 256));

    struct aws_byte_cursor prefix = aws_byte_cursor_from_c_str("./home-");
    ASSERT_SUCCESS(aws_byte_buf_append(&path_buf, &prefix));

    struct aws_uuid uuid;
    ASSERT_SUCCESS(aws_uuid_init(&uuid));
    ASSERT_SUCCESS(aws_uuid_to_str(&uuid, &path_buf));

    ASSERT_SUCCESS(aws_byte_buf_append_byte_dynamic(&path_buf, '/'));

    struct aws_string *path_str = aws_string_new_from_buf(allocator, &path_buf);
    ASSERT_SUCCESS(aws_create_directory_components(allocator, path_str));

    ASSERT_SUCCESS(aws_set_environment_value(s_home_env_var, path_str));

    aws_byte_buf_clean_up(&path_buf);
    *out_path = path_str;
    return AWS_OP_SUCCESS;
}

/*
 * Mocked HTTP connection manager for tests
 */

struct aws_auth_http_system_vtable aws_credentials_provider_http_mock_function_table = {
    .aws_http_connection_manager_new = aws_credentials_provider_http_mock_connection_manager_new,
    .aws_http_connection_manager_release = aws_credentials_provider_http_mock_connection_manager_release,
    .aws_http_connection_manager_acquire_connection =
        aws_credentials_provider_http_mock_connection_manager_acquire_connection,
    .aws_http_connection_manager_release_connection =
        aws_credentials_provider_http_mock_connection_manager_release_connection,
    .aws_http_connection_make_request = aws_credentials_provider_http_mock_make_request,
    .aws_http_stream_activate = aws_credentials_provider_http_mock_stream_activate,
    .aws_http_stream_get_connection = aws_credentials_provider_http_mock_stream_get_connection,
    .aws_http_stream_get_incoming_response_status =
        aws_credentials_provider_http_mock_stream_get_incoming_response_status,
    .aws_http_stream_release = aws_credentials_provider_http_mock_stream_release,
    .aws_http_connection_close = aws_credentials_provider_http_mock_connection_close};

struct aws_credentials_provider_http_mock_tester credentials_provider_http_mock_tester;

int aws_credentials_provider_http_mock_tester_init(struct aws_allocator *allocator) {
    aws_auth_library_init(allocator);

    AWS_ZERO_STRUCT(credentials_provider_http_mock_tester);

    struct aws_tls_ctx_options tls_ctx_options;
    aws_tls_ctx_options_init_default_client(&tls_ctx_options, allocator);
    credentials_provider_http_mock_tester.tls_ctx = aws_tls_client_ctx_new(allocator, &tls_ctx_options);
    ASSERT_NOT_NULL(credentials_provider_http_mock_tester.tls_ctx);

    credentials_provider_http_mock_tester.el_group = aws_event_loop_group_new_default(allocator, 0, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = credentials_provider_http_mock_tester.el_group,
        .max_entries = 8,
    };
    credentials_provider_http_mock_tester.resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = credentials_provider_http_mock_tester.el_group,
        .host_resolver = credentials_provider_http_mock_tester.resolver,
    };
    credentials_provider_http_mock_tester.bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    if (aws_array_list_init_dynamic(
            &credentials_provider_http_mock_tester.response_data_callbacks,
            allocator,
            10,
            sizeof(struct aws_byte_cursor))) {
        return AWS_OP_ERR;
    }

    if (aws_byte_buf_init(&credentials_provider_http_mock_tester.request_path, allocator, 256)) {
        return AWS_OP_ERR;
    }
    if (aws_byte_buf_init(&credentials_provider_http_mock_tester.request_body, allocator, 256)) {
        return AWS_OP_ERR;
    }

    if (aws_mutex_init(&credentials_provider_http_mock_tester.lock)) {
        return AWS_OP_ERR;
    }

    if (aws_condition_variable_init(&credentials_provider_http_mock_tester.signal)) {
        return AWS_OP_ERR;
    }

    /* default to everything successful */
    credentials_provider_http_mock_tester.is_connection_acquire_successful = true;
    credentials_provider_http_mock_tester.is_request_successful = true;

    return AWS_OP_SUCCESS;
}

void aws_credentials_provider_http_mock_tester_cleanup(void) {
    aws_tls_ctx_release(credentials_provider_http_mock_tester.tls_ctx);
    aws_client_bootstrap_release(credentials_provider_http_mock_tester.bootstrap);
    aws_host_resolver_release(credentials_provider_http_mock_tester.resolver);
    aws_event_loop_group_release(credentials_provider_http_mock_tester.el_group);
    aws_array_list_clean_up(&credentials_provider_http_mock_tester.response_data_callbacks);
    aws_byte_buf_clean_up(&credentials_provider_http_mock_tester.request_path);
    aws_byte_buf_clean_up(&credentials_provider_http_mock_tester.request_body);
    aws_condition_variable_clean_up(&credentials_provider_http_mock_tester.signal);
    aws_mutex_clean_up(&credentials_provider_http_mock_tester.lock);
    aws_credentials_release(credentials_provider_http_mock_tester.credentials);
    aws_auth_library_clean_up();
}

void aws_credentials_provider_http_mock_on_shutdown_complete(void *user_data) {
    (void)user_data;
    aws_mutex_lock(&credentials_provider_http_mock_tester.lock);
    credentials_provider_http_mock_tester.has_received_shutdown_callback = true;
    aws_mutex_unlock(&credentials_provider_http_mock_tester.lock);

    aws_condition_variable_notify_one(&credentials_provider_http_mock_tester.signal);
}

bool aws_credentials_provider_http_mock_has_received_shutdown_callback(void *user_data) {
    (void)user_data;

    return credentials_provider_http_mock_tester.has_received_shutdown_callback;
}

void aws_credentials_provider_http_mock_wait_for_shutdown_callback(void) {
    aws_mutex_lock(&credentials_provider_http_mock_tester.lock);
    aws_condition_variable_wait_pred(
        &credentials_provider_http_mock_tester.signal,
        &credentials_provider_http_mock_tester.lock,
        aws_credentials_provider_http_mock_has_received_shutdown_callback,
        NULL);
    aws_mutex_unlock(&credentials_provider_http_mock_tester.lock);
}

struct mock_connection_manager {
    struct aws_allocator *allocator;
    aws_http_connection_manager_shutdown_complete_fn *shutdown_complete_callback;
    void *shutdown_complete_user_data;
};

struct aws_http_connection_manager *aws_credentials_provider_http_mock_connection_manager_new(
    struct aws_allocator *allocator,
    const struct aws_http_connection_manager_options *options) {

    struct mock_connection_manager *mock_manager = aws_mem_calloc(allocator, 1, sizeof(struct mock_connection_manager));
    mock_manager->allocator = allocator;
    mock_manager->shutdown_complete_callback = options->shutdown_complete_callback;
    mock_manager->shutdown_complete_user_data = options->shutdown_complete_user_data;
    return (struct aws_http_connection_manager *)mock_manager;
}

void aws_credentials_provider_http_mock_connection_manager_release(struct aws_http_connection_manager *manager) {
    struct mock_connection_manager *mock_manager = (struct mock_connection_manager *)manager;
    mock_manager->shutdown_complete_callback(mock_manager->shutdown_complete_user_data);
    aws_mem_release(mock_manager->allocator, mock_manager);
}

void aws_credentials_provider_http_mock_connection_manager_acquire_connection(
    struct aws_http_connection_manager *manager,
    aws_http_connection_manager_on_connection_setup_fn *callback,
    void *user_data) {

    (void)manager;
    (void)callback;
    (void)user_data;

    if (credentials_provider_http_mock_tester.is_connection_acquire_successful) {
        callback((struct aws_http_connection *)1, AWS_OP_SUCCESS, user_data);
    } else {
        aws_raise_error(AWS_ERROR_HTTP_UNKNOWN);
        callback(NULL, AWS_OP_ERR, user_data);
    }
}

int aws_credentials_provider_http_mock_connection_manager_release_connection(
    struct aws_http_connection_manager *manager,
    struct aws_http_connection *connection) {

    (void)manager;
    (void)connection;

    return AWS_OP_SUCCESS;
}

void aws_credentials_provider_http_mock_invoke_request_callbacks(
    const struct aws_http_make_request_options *options,
    struct aws_array_list *data_callbacks,
    bool is_request_successful) {

    size_t data_callback_count = aws_array_list_length(data_callbacks);

    struct aws_http_header headers[1];
    AWS_ZERO_ARRAY(headers);

    headers[0].name = aws_byte_cursor_from_c_str("some-header");
    headers[0].value = aws_byte_cursor_from_c_str("value");
    if (options->on_response_headers) {
        options->on_response_headers(
            (struct aws_http_stream *)1, AWS_HTTP_HEADER_BLOCK_MAIN, headers, 1, options->user_data);
    }
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

struct aws_http_stream *aws_credentials_provider_http_mock_make_request(
    struct aws_http_connection *client_connection,
    const struct aws_http_make_request_options *options) {

    (void)client_connection;
    (void)options;

    struct aws_byte_cursor path;
    AWS_ZERO_STRUCT(path);
    struct aws_input_stream *body_stream = aws_http_message_get_body_stream(options->request);
    struct aws_allocator *allocator = credentials_provider_http_mock_tester.request_body.allocator;
    aws_byte_buf_clean_up(&credentials_provider_http_mock_tester.request_body);
    aws_byte_buf_init(&credentials_provider_http_mock_tester.request_body, allocator, 256);
    if (body_stream) {
        aws_input_stream_read(body_stream, &credentials_provider_http_mock_tester.request_body);
    }
    aws_byte_buf_clean_up(&credentials_provider_http_mock_tester.request_path);

    struct aws_byte_cursor request_path_cursor;
    aws_http_message_get_request_path(options->request, &request_path_cursor);
    aws_byte_buf_init_copy_from_cursor(
        &credentials_provider_http_mock_tester.request_path, allocator, request_path_cursor);
    credentials_provider_http_mock_tester.attempts++;
    credentials_provider_http_mock_tester.request_options = *options;

    return (struct aws_http_stream *)1;
}

int aws_credentials_provider_http_mock_stream_activate(struct aws_http_stream *stream) {
    (void)stream;
    aws_credentials_provider_http_mock_invoke_request_callbacks(
        &credentials_provider_http_mock_tester.request_options,
        &credentials_provider_http_mock_tester.response_data_callbacks,
        credentials_provider_http_mock_tester.is_request_successful);
    return AWS_OP_SUCCESS;
}

int aws_credentials_provider_http_mock_stream_get_incoming_response_status(
    const struct aws_http_stream *stream,
    int *out_status_code) {
    (void)stream;

    if (credentials_provider_http_mock_tester.failure_count) {
        credentials_provider_http_mock_tester.failure_count--;
        *out_status_code = credentials_provider_http_mock_tester.failure_response_code;
    } else if (credentials_provider_http_mock_tester.response_code) {
        *out_status_code = credentials_provider_http_mock_tester.response_code;
    } else {
        *out_status_code = AWS_HTTP_STATUS_CODE_200_OK;
    }

    return AWS_OP_SUCCESS;
}

void aws_credentials_provider_http_mock_stream_release(struct aws_http_stream *stream) {
    (void)stream;
}

void aws_credentials_provider_http_mock_connection_close(struct aws_http_connection *connection) {
    (void)connection;
}

struct aws_http_connection *aws_credentials_provider_http_mock_stream_get_connection(
    const struct aws_http_stream *stream) {
    (void)stream;
    return (struct aws_http_connection *)1;
}

bool aws_credentials_provider_http_mock_has_received_credentials_callback(void *user_data) {
    (void)user_data;

    return credentials_provider_http_mock_tester.has_received_credentials_callback;
}

void aws_credentials_provider_http_mock_wait_for_credentials_result(void) {
    aws_mutex_lock(&credentials_provider_http_mock_tester.lock);
    aws_condition_variable_wait_pred(
        &credentials_provider_http_mock_tester.signal,
        &credentials_provider_http_mock_tester.lock,
        aws_credentials_provider_http_mock_has_received_credentials_callback,
        NULL);
    aws_mutex_unlock(&credentials_provider_http_mock_tester.lock);
}

void aws_credentials_provider_http_mock_get_credentials_callback(
    struct aws_credentials *credentials,
    int error_code,
    void *user_data) {
    (void)user_data;

    aws_mutex_lock(&credentials_provider_http_mock_tester.lock);
    credentials_provider_http_mock_tester.has_received_credentials_callback = true;
    credentials_provider_http_mock_tester.credentials = credentials;
    credentials_provider_http_mock_tester.error_code = error_code;
    if (credentials != NULL) {
        aws_credentials_acquire(credentials);
    }
    aws_condition_variable_notify_one(&credentials_provider_http_mock_tester.signal);
    aws_mutex_unlock(&credentials_provider_http_mock_tester.lock);
}
