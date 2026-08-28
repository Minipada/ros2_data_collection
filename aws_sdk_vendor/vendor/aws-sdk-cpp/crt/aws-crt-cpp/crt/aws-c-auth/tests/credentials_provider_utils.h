#ifndef AWS_AUTH_CREDENTIALS_PROVIDER_MOCK_H
#define AWS_AUTH_CREDENTIALS_PROVIDER_MOCK_H

/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/auth/private/aws_profile.h>
#include <aws/auth/private/credentials_utils.h>

#include <aws/common/condition_variable.h>
#include <aws/common/mutex.h>
#include <aws/http/connection_manager.h>
#include <aws/http/request_response.h>

struct aws_credentials;
struct aws_credentials_provider;
struct aws_credentials_provider_shutdown_options;
struct aws_event_loop_group;
struct aws_string;

/*
 * This file contains a number of helper functions and data structures
 * that let us verify async behavior within the credentials provider.
 *
 * It includes multiple provider mocks (one synchronous, one background-thread
 * based and externally controllable), a synchronizing controller that uses
 * concurrency primitives to ensure we can perform operations at troublesome
 * time points (freeze the cached background query so that we can queue up
 * multiple pending queries, for example), and misc supporting functions like
 * time function mocks.
 */

/*
 * test helper struct to correctly wait on async credentials callbacks
 */
struct aws_get_credentials_test_callback_result {
    struct aws_mutex sync;
    struct aws_condition_variable signal;
    struct aws_credentials *credentials;
    int count;
    int required_count;
    int last_error;
};

void aws_get_credentials_test_callback_result_init(
    struct aws_get_credentials_test_callback_result *result,
    int required_count);
void aws_get_credentials_test_callback_result_clean_up(struct aws_get_credentials_test_callback_result *result);

void aws_wait_on_credentials_callback(struct aws_get_credentials_test_callback_result *result);

void aws_test_get_credentials_async_callback(struct aws_credentials *credentials, int error_code, void *user_data);

struct get_credentials_mock_result {
    int error_code;
    struct aws_credentials *credentials;
};

/*
 * Mock credentials provider, synchronous
 */
struct aws_credentials_provider *aws_credentials_provider_new_mock(
    struct aws_allocator *allocator,
    struct get_credentials_mock_result *results,
    size_t result_count,
    struct aws_credentials_provider_shutdown_options *shutdown_options);

struct aws_credentials_provider *aws_credentials_provider_new_mock_async(
    struct aws_allocator *allocator,
    struct get_credentials_mock_result *results,
    size_t result_count,
    struct aws_event_loop_group *elg,
    struct aws_credentials_provider_shutdown_options *shutdown_options);

/* If any pending queries, deliver the next mock-result to all of them from another thread.
 * If no pending queries, nothing happens. */
void aws_credentials_provider_mock_async_fire_callbacks(struct aws_credentials_provider *provider);

/*
 * Simple global clock mocks
 */
int mock_aws_get_system_time(uint64_t *current_time);
void mock_aws_set_system_time(uint64_t current_time);

int mock_aws_get_high_res_time(uint64_t *current_time);
void mock_aws_set_high_res_time(uint64_t current_time);

/*
 * Credentials provider that always returns NULL.  Useful for chain tests.
 */
struct aws_credentials_provider *aws_credentials_provider_new_null(
    struct aws_allocator *allocator,
    struct aws_credentials_provider_shutdown_options *shutdown_options);

/**
 * Create the directory components of @path:
 * - if @path ends in a path separator, create every directory component;
 * - else, stop at the last path separator (parent directory of @path).
 */
int aws_create_directory_components(struct aws_allocator *allocator, const struct aws_string *path);

/**
 * Create a new directory (under current working dir) and set $HOME env variable.
 */
int aws_create_random_home_directory(struct aws_allocator *allocator, struct aws_string **out_path);

/**
 * Mocked HTTP connection manager for tests
 */
struct aws_credentials_provider_http_mock_tester {
    struct aws_tls_ctx *tls_ctx;
    struct aws_event_loop_group *el_group;
    struct aws_host_resolver *resolver;
    struct aws_client_bootstrap *bootstrap;

    struct aws_byte_buf request_path;
    struct aws_byte_buf request_body;
    struct aws_http_make_request_options request_options;

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
    int failure_response_code;
    int failure_count;
};

extern struct aws_credentials_provider_http_mock_tester credentials_provider_http_mock_tester;
int aws_credentials_provider_http_mock_tester_init(struct aws_allocator *allocator);
void aws_credentials_provider_http_mock_tester_cleanup(void);
void aws_credentials_provider_http_mock_on_shutdown_complete(void *user_data);
bool aws_credentials_provider_http_mock_has_received_shutdown_callback(void *user_data);
void aws_credentials_provider_http_mock_wait_for_shutdown_callback(void);
struct aws_http_connection_manager *aws_credentials_provider_http_mock_connection_manager_new(
    struct aws_allocator *allocator,
    const struct aws_http_connection_manager_options *options);
void aws_credentials_provider_http_mock_connection_manager_release(struct aws_http_connection_manager *manager);
void aws_credentials_provider_http_mock_connection_manager_acquire_connection(
    struct aws_http_connection_manager *manager,
    aws_http_connection_manager_on_connection_setup_fn *callback,
    void *user_data);
int aws_credentials_provider_http_mock_connection_manager_release_connection(
    struct aws_http_connection_manager *manager,
    struct aws_http_connection *connection);
void aws_credentials_provider_http_mock_invoke_request_callbacks(
    const struct aws_http_make_request_options *options,
    struct aws_array_list *data_callbacks,
    bool is_request_successful);
struct aws_http_stream *aws_credentials_provider_http_mock_make_request(
    struct aws_http_connection *client_connection,
    const struct aws_http_make_request_options *options);
int aws_credentials_provider_http_mock_stream_activate(struct aws_http_stream *stream);
int aws_credentials_provider_http_mock_stream_get_incoming_response_status(
    const struct aws_http_stream *stream,
    int *out_status_code);
void aws_credentials_provider_http_mock_stream_release(struct aws_http_stream *stream);
void aws_credentials_provider_http_mock_connection_close(struct aws_http_connection *connection);
struct aws_http_connection *aws_credentials_provider_http_mock_stream_get_connection(
    const struct aws_http_stream *stream);
bool aws_credentials_provider_http_mock_has_received_credentials_callback(void *user_data);
void aws_credentials_provider_http_mock_wait_for_credentials_result(void);
void aws_credentials_provider_http_mock_get_credentials_callback(
    struct aws_credentials *credentials,
    int error_code,
    void *user_data);
extern struct aws_auth_http_system_vtable aws_credentials_provider_http_mock_function_table;

#endif /* AWS_AUTH_CREDENTIALS_PROVIDER_MOCK_H */
