/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/event-stream/event_stream_rpc_client.h>
#include <aws/event-stream/private/event_stream_rpc_test_helper.h>

#include <aws/common/condition_variable.h>
#include <aws/common/device_random.h>
#include <aws/common/macros.h>
#include <aws/common/mutex.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/socket.h>

#include <aws/testing/aws_test_harness.h>

static const char *s_test_host_name = "127.0.0.1";

struct client_test_data {
    struct aws_allocator *allocator;
    int received_message_flags;
    struct aws_mutex sync_lock;
    struct aws_condition_variable sync_cvar;
    enum aws_event_stream_rpc_message_type received_message_type;
    struct aws_byte_buf received_payload;
    struct aws_event_stream_rpc_server_continuation_token *server_token;
    struct aws_byte_buf last_seen_operation_name;
    bool client_message_sent;
    bool client_message_received;
    bool server_message_sent;
    bool server_message_received;
    bool client_token_closed;
    bool server_token_closed;
};

struct test_data {
    struct aws_allocator *allocator;
    struct aws_event_loop_group *el_group;
    struct aws_server_bootstrap *server_bootstrap;
    struct aws_client_bootstrap *client_bootstrap;
    struct aws_host_resolver *resolver;
    struct aws_event_stream_rpc_server_listener *listener;
    struct aws_event_stream_rpc_server_connection *server_connection;
    struct aws_event_stream_rpc_client_connection *client_connection;
    aws_event_stream_rpc_server_connection_protocol_message_fn *on_server_message_received;
    aws_event_stream_rpc_server_on_incoming_stream_fn *on_new_server_stream;
    aws_event_stream_rpc_server_stream_continuation_fn *on_new_server_continuation;
    aws_event_stream_rpc_server_stream_continuation_closed_fn *on_server_continuation_closed;

    aws_event_stream_rpc_client_connection_protocol_message_fn *on_client_message_received;

    struct client_test_data client_test_data;

    void *user_data;
    void *server_continuation_user_data;
    struct aws_mutex shutdown_lock;
    struct aws_condition_variable shutdown_cvar;
    struct aws_mutex setup_lock;
    struct aws_condition_variable setup_cvar;
    bool client_setup_completed;
    bool server_setup_completed;
    bool client_connection_shutdown;
    bool server_connection_shutdown_completed;
    bool event_loop_shutdown_completed;
    bool listener_shutdown_completed;
    bool resolver_shutdown_completed;
};

static struct test_data s_test_data;

static void s_fixture_on_server_protocol_message(
    struct aws_event_stream_rpc_server_connection *connection,
    const struct aws_event_stream_rpc_message_args *message_args,
    void *user_data) {
    struct test_data *test_data = user_data;
    test_data->on_server_message_received(connection, message_args, test_data->user_data);
}

static void s_on_stream_server_continuation_shim(
    struct aws_event_stream_rpc_server_continuation_token *token,
    const struct aws_event_stream_rpc_message_args *message_args,
    void *user_data) {
    struct test_data *test_data = user_data;
    test_data->on_new_server_continuation(token, message_args, test_data->server_continuation_user_data);
}

static void s_stream_server_continuation_closed_shim(
    struct aws_event_stream_rpc_server_continuation_token *token,
    void *user_data) {
    struct test_data *test_data = user_data;
    test_data->on_server_continuation_closed(token, test_data->server_continuation_user_data);
}

static int s_on_server_incoming_stream_shim(
    struct aws_event_stream_rpc_server_connection *connection,
    struct aws_event_stream_rpc_server_continuation_token *token,
    struct aws_byte_cursor operation_name,
    struct aws_event_stream_rpc_server_stream_continuation_options *continuation_options,
    void *user_data) {
    struct test_data *test_data = user_data;

    continuation_options->on_continuation = s_on_stream_server_continuation_shim;
    continuation_options->on_continuation_closed = s_stream_server_continuation_closed_shim;
    continuation_options->user_data = test_data;

    if (test_data->on_new_server_stream) {
        test_data->on_new_server_stream(
            connection, token, operation_name, continuation_options, test_data->server_continuation_user_data);
    }

    return AWS_OP_SUCCESS;
}

static int s_fixture_on_new_server_connection(
    struct aws_event_stream_rpc_server_connection *connection,
    int error_code,
    struct aws_event_stream_rpc_connection_options *connection_options,
    void *user_data) {
    (void)error_code;

    struct test_data *test_data = user_data;
    test_data->server_connection = connection;
    aws_mutex_lock(&test_data->setup_lock);
    test_data->server_setup_completed = true;
    aws_mutex_unlock(&test_data->setup_lock);
    aws_event_stream_rpc_server_connection_acquire(connection);

    connection_options->on_connection_protocol_message = s_fixture_on_server_protocol_message;
    connection_options->on_incoming_stream = s_on_server_incoming_stream_shim;
    connection_options->user_data = user_data;

    aws_condition_variable_notify_one(&test_data->setup_cvar);

    return AWS_OP_SUCCESS;
}

static void s_fixture_on_server_connection_shutdown(
    struct aws_event_stream_rpc_server_connection *connection,
    int error_code,
    void *user_data) {
    (void)connection;
    (void)error_code;

    struct test_data *test_data = user_data;

    aws_mutex_lock(&test_data->shutdown_lock);
    test_data->server_connection_shutdown_completed = true;
    aws_mutex_unlock(&test_data->shutdown_lock);
    aws_condition_variable_notify_one(&test_data->shutdown_cvar);
}

static bool s_server_connection_shutdown_completed(void *args) {
    struct test_data *test_data = args;
    return test_data->server_connection_shutdown_completed;
}

static void s_on_listener_destroy(struct aws_event_stream_rpc_server_listener *server, void *user_data) {
    (void)server;
    struct test_data *test_data = user_data;
    aws_mutex_lock(&test_data->shutdown_lock);
    test_data->listener_shutdown_completed = true;
    aws_mutex_unlock(&test_data->shutdown_lock);
    aws_condition_variable_notify_one(&test_data->shutdown_cvar);
}

static bool s_listener_shutdown_pred(void *arg) {
    struct test_data *test_data = arg;
    return test_data->listener_shutdown_completed;
}

static void s_event_loop_shutdown_callback(void *user_data) {
    struct test_data *test_data = user_data;
    aws_mutex_lock(&test_data->shutdown_lock);
    test_data->event_loop_shutdown_completed = true;
    aws_mutex_unlock(&test_data->shutdown_lock);
    aws_condition_variable_notify_one(&test_data->shutdown_cvar);
}

static bool s_event_loop_shutdown_pred(void *arg) {
    struct test_data *test_data = arg;
    return test_data->event_loop_shutdown_completed;
}

static void s_client_on_connection_setup(
    struct aws_event_stream_rpc_client_connection *connection,
    int error_code,
    void *user_data) {
    (void)error_code;
    struct test_data *test_data = user_data;

    aws_mutex_lock(&test_data->setup_lock);
    test_data->client_connection = connection;

    if (connection) {
        aws_event_stream_rpc_client_connection_acquire(connection);
    }
    test_data->client_setup_completed = true;
    aws_mutex_unlock(&test_data->setup_lock);
    aws_condition_variable_notify_one(&test_data->setup_cvar);
}

static void s_client_on_connection_shutdown(
    struct aws_event_stream_rpc_client_connection *connection,
    int error_code,
    void *user_data) {
    (void)connection;
    (void)error_code;

    struct test_data *test_data = user_data;

    aws_mutex_lock(&test_data->shutdown_lock);
    test_data->client_connection_shutdown = true;
    aws_mutex_unlock(&test_data->shutdown_lock);
    aws_condition_variable_notify_one(&test_data->shutdown_cvar);
}

static bool s_client_connection_shutdown_completed(void *args) {
    struct test_data *test_data = args;
    return test_data->client_connection_shutdown;
}

static void s_client_connection_protocol_message(
    struct aws_event_stream_rpc_client_connection *connection,
    const struct aws_event_stream_rpc_message_args *message_args,
    void *user_data) {
    struct test_data *test_data = user_data;
    test_data->on_client_message_received(connection, message_args, test_data->user_data);
}

static bool s_setup_completed_pred(void *arg) {
    struct test_data *test_data = arg;

    return test_data->client_setup_completed && test_data->server_setup_completed;
}

static void s_resolver_shutdown_completion_callback(void *arg) {
    struct test_data *test_data = arg;

    aws_mutex_lock(&test_data->shutdown_lock);
    test_data->resolver_shutdown_completed = true;
    aws_mutex_unlock(&test_data->shutdown_lock);
    aws_condition_variable_notify_one(&test_data->shutdown_cvar);
}

static bool s_resolver_shutdown_completed_pred(void *arg) {
    struct test_data *test_data = arg;

    return test_data->resolver_shutdown_completed;
}

static int s_fixture_setup(struct aws_allocator *allocator, void *ctx) {
    aws_event_stream_library_init(allocator);
    struct test_data *test_data = ctx;
    AWS_ZERO_STRUCT(*test_data);
    aws_mutex_init(&test_data->setup_lock);
    aws_mutex_init(&test_data->shutdown_lock);
    aws_condition_variable_init(&test_data->setup_cvar);
    aws_condition_variable_init(&test_data->shutdown_cvar);

    struct aws_shutdown_callback_options el_shutdown_options = {
        .shutdown_callback_fn = s_event_loop_shutdown_callback,
        .shutdown_callback_user_data = test_data,
    };
    test_data->el_group = aws_event_loop_group_new_default(allocator, 1, &el_shutdown_options);
    ASSERT_NOT_NULL(test_data->el_group);
    test_data->server_bootstrap = aws_server_bootstrap_new(allocator, test_data->el_group);
    ASSERT_NOT_NULL(test_data->server_bootstrap);

    struct aws_shutdown_callback_options host_resolver_shutdown_options = {
        .shutdown_callback_fn = s_resolver_shutdown_completion_callback,
        .shutdown_callback_user_data = test_data,
    };

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = test_data->el_group,
        .max_entries = 1,
        .shutdown_options = &host_resolver_shutdown_options,
    };

    test_data->resolver = aws_host_resolver_new_default(allocator, &resolver_options);
    ASSERT_NOT_NULL(test_data->resolver);

    struct aws_client_bootstrap_options client_bootstrap_options = {
        .user_data = test_data,
        .event_loop_group = test_data->el_group,
        .host_resolver = test_data->resolver,
    };
    test_data->client_bootstrap = aws_client_bootstrap_new(allocator, &client_bootstrap_options);
    ASSERT_NOT_NULL(test_data->client_bootstrap);

    ASSERT_SUCCESS(aws_mutex_init(&test_data->shutdown_lock));
    ASSERT_SUCCESS(aws_condition_variable_init(&test_data->shutdown_cvar));

    ASSERT_SUCCESS(aws_mutex_init(&test_data->client_test_data.sync_lock));
    ASSERT_SUCCESS(aws_condition_variable_init(&test_data->client_test_data.sync_cvar));
    test_data->client_test_data.allocator = allocator;

    struct aws_socket_options socket_options = {
        .connect_timeout_ms = 3000,
        .domain = AWS_SOCKET_IPV4,
        .type = AWS_SOCKET_STREAM,
    };

    /* Find a random open port */
    uint16_t test_port = 0;
    while (!test_data->listener) {
        aws_device_random_u16(&test_port);
        test_port |= 0x8000; /* Use high numbers */

        struct aws_event_stream_rpc_server_listener_options listener_options = {
            .socket_options = &socket_options,
            .host_name = s_test_host_name,
            .port = test_port,
            .bootstrap = test_data->server_bootstrap,
            .user_data = test_data,
            .on_new_connection = s_fixture_on_new_server_connection,
            .on_connection_shutdown = s_fixture_on_server_connection_shutdown,
            .on_destroy_callback = s_on_listener_destroy,
        };

        test_data->listener = aws_event_stream_rpc_server_new_listener(allocator, &listener_options);
        if (!test_data->listener) {
            int error_code = aws_last_error();
            ASSERT_TRUE(error_code == AWS_IO_SOCKET_ADDRESS_IN_USE || error_code == AWS_ERROR_NO_PERMISSION);
        }
    }

    test_data->allocator = allocator;

    struct aws_event_stream_rpc_client_connection_options connection_options = {
        .socket_options = &socket_options,
        .user_data = test_data,
        .bootstrap = test_data->client_bootstrap,
        .host_name = s_test_host_name,
        .port = test_port,
        .on_connection_setup = s_client_on_connection_setup,
        .on_connection_shutdown = s_client_on_connection_shutdown,
        .on_connection_protocol_message = s_client_connection_protocol_message,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_connect(allocator, &connection_options));

    aws_mutex_lock(&test_data->setup_lock);
    aws_condition_variable_wait_pred(&test_data->setup_cvar, &test_data->setup_lock, s_setup_completed_pred, test_data);
    aws_mutex_unlock(&test_data->setup_lock);

    ASSERT_NOT_NULL(&test_data->client_connection);
    return AWS_OP_SUCCESS;
}

static int s_fixture_shutdown(struct aws_allocator *allocator, int setup_result, void *ctx) {
    (void)allocator;
    struct test_data *test_data = ctx;

    if (!setup_result) {
        aws_mutex_lock(&test_data->shutdown_lock);
        aws_condition_variable_wait_pred(
            &test_data->shutdown_cvar, &test_data->shutdown_lock, s_server_connection_shutdown_completed, test_data);
        aws_condition_variable_wait_pred(
            &test_data->shutdown_cvar, &test_data->shutdown_lock, s_client_connection_shutdown_completed, test_data);
        aws_event_stream_rpc_client_connection_release(test_data->client_connection);
        aws_event_stream_rpc_server_connection_release(test_data->server_connection);
        aws_event_stream_rpc_server_listener_release(test_data->listener);
        aws_condition_variable_wait_pred(
            &test_data->shutdown_cvar, &test_data->shutdown_lock, s_listener_shutdown_pred, test_data);
        aws_server_bootstrap_release(test_data->server_bootstrap);
        aws_client_bootstrap_release(test_data->client_bootstrap);
        aws_host_resolver_release(test_data->resolver);
        aws_condition_variable_wait_pred(
            &test_data->shutdown_cvar, &test_data->shutdown_lock, s_resolver_shutdown_completed_pred, test_data);
        aws_event_loop_group_release(test_data->el_group);
        aws_condition_variable_wait_pred(
            &test_data->shutdown_cvar, &test_data->shutdown_lock, s_event_loop_shutdown_pred, test_data);
        aws_mutex_unlock(&test_data->shutdown_lock);
        aws_thread_join_all_managed();
        aws_mutex_clean_up(&test_data->shutdown_lock);
        aws_condition_variable_clean_up(&test_data->shutdown_cvar);
        aws_mutex_clean_up(&test_data->setup_lock);
        aws_condition_variable_clean_up(&test_data->setup_cvar);
        aws_mutex_clean_up(&test_data->client_test_data.sync_lock);
        aws_condition_variable_clean_up(&test_data->client_test_data.sync_cvar);
    }

    aws_event_stream_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_test_event_stream_rpc_client_connection_setup_and_teardown(struct aws_allocator *allocator, void *ctx) {
    struct test_data *test_data = ctx;
    (void)allocator;
    /* just let setup and shutdown run to make sure the basic init/cleanup flow references are properly counted without
     * having to worry about continuation reference counts. */
    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_setup_and_teardown,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_setup_and_teardown,
    s_fixture_shutdown,
    &s_test_data)

static void s_rpc_client_message_flush(int error_code, void *user_data) {
    (void)error_code;

    struct client_test_data *client_test_data = user_data;
    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->client_message_sent = true;
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
    /* make these pessimistic to prevent a cleanup race. */
    aws_mutex_unlock(&client_test_data->sync_lock);
}

static void s_rpc_server_message_flush(int error_code, void *user_data) {
    (void)error_code;

    struct client_test_data *client_test_data = user_data;
    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->server_message_sent = true;
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
    /* make these pessimistic to prevent a cleanup race. */
    aws_mutex_unlock(&client_test_data->sync_lock);
}

static bool s_rpc_client_message_transmission_completed_pred(void *arg) {
    struct client_test_data *client_test_data = arg;
    return client_test_data->client_message_sent && client_test_data->server_message_received;
}

static bool s_rpc_server_message_transmission_completed_pred(void *arg) {
    struct client_test_data *client_test_data = arg;
    return client_test_data->server_message_sent && client_test_data->client_message_received;
}

static void s_rpc_server_connection_protocol_message(
    struct aws_event_stream_rpc_server_connection *connection,
    const struct aws_event_stream_rpc_message_args *message_args,
    void *user_data) {
    (void)connection;

    struct client_test_data *client_test_data = user_data;
    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->server_message_received = true;
    client_test_data->received_message_type = message_args->message_type;
    aws_byte_buf_init_copy(&client_test_data->received_payload, client_test_data->allocator, message_args->payload);
    aws_mutex_unlock(&client_test_data->sync_lock);
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
}

static void s_rpc_client_connection_protocol_message(
    struct aws_event_stream_rpc_client_connection *connection,
    const struct aws_event_stream_rpc_message_args *message_args,
    void *user_data) {
    (void)connection;
    struct client_test_data *client_test_data = user_data;

    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->client_message_received = true;
    client_test_data->received_message_type = message_args->message_type;
    client_test_data->received_message_flags = message_args->message_flags;
    aws_byte_buf_init_copy(&client_test_data->received_payload, client_test_data->allocator, message_args->payload);
    aws_mutex_unlock(&client_test_data->sync_lock);
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
}

static int s_test_event_stream_rpc_client_connection_connect(struct aws_allocator *allocator, void *ctx) {

    struct test_data *test_data = ctx;
    test_data->on_server_message_received = s_rpc_server_connection_protocol_message;
    test_data->on_client_message_received = s_rpc_client_connection_protocol_message;

    struct client_test_data client_test_data = {
        .allocator = allocator,
        .sync_cvar = AWS_CONDITION_VARIABLE_INIT,
        .sync_lock = AWS_MUTEX_INIT,
    };

    test_data->user_data = &client_test_data;

    struct aws_byte_buf connect_payload = aws_byte_buf_from_c_str("{ \"message\": \" connect message \" }");
    struct aws_event_stream_rpc_message_args connect_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT,
        .payload = &connect_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_send_protocol_message(
        test_data->client_connection, &connect_args, s_rpc_client_message_flush, &client_test_data));

    aws_mutex_lock(&client_test_data.sync_lock);
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    connect_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK;
    connect_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_CONNECTION_ACCEPTED;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_connection_send_protocol_message(
        test_data->server_connection, &connect_args, s_rpc_server_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);

    aws_mutex_unlock(&client_test_data.sync_lock);
    aws_mutex_clean_up(&client_test_data.sync_lock);
    aws_condition_variable_clean_up(&client_test_data.sync_cvar);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_connect,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_connect,
    s_fixture_shutdown,
    &s_test_data)

static int s_test_event_stream_rpc_client_connection_message_before_connect(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)allocator;

    struct test_data *test_data = ctx;

    struct aws_byte_buf ping_payload = aws_byte_buf_from_c_str("{ \"message\": \" ping message \" }");
    struct aws_event_stream_rpc_message_args connect_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_PING,
        .payload = &ping_payload,
    };

    ASSERT_ERROR(
        AWS_ERROR_EVENT_STREAM_RPC_PROTOCOL_ERROR,
        aws_event_stream_rpc_client_connection_send_protocol_message(
            test_data->client_connection, &connect_args, s_rpc_client_message_flush, NULL));

    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_message_before_connect,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_message_before_connect,
    s_fixture_shutdown,
    &s_test_data)

static int s_test_event_stream_rpc_client_connection_protocol_message(struct aws_allocator *allocator, void *ctx) {

    struct test_data *test_data = ctx;
    test_data->on_server_message_received = s_rpc_server_connection_protocol_message;
    test_data->on_client_message_received = s_rpc_client_connection_protocol_message;

    struct client_test_data client_test_data = {
        .allocator = allocator,
        .sync_cvar = AWS_CONDITION_VARIABLE_INIT,
        .sync_lock = AWS_MUTEX_INIT,
    };

    test_data->user_data = &client_test_data;

    struct aws_byte_buf connect_payload = aws_byte_buf_from_c_str("{ \"message\": \" connect message \" }");
    struct aws_event_stream_rpc_message_args connect_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT,
        .payload = &connect_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_send_protocol_message(
        test_data->client_connection, &connect_args, s_rpc_client_message_flush, &client_test_data));

    aws_mutex_lock(&client_test_data.sync_lock);
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    connect_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK;
    connect_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_CONNECTION_ACCEPTED;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_connection_send_protocol_message(
        test_data->server_connection, &connect_args, s_rpc_server_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    struct aws_byte_buf ping_payload =
        aws_byte_buf_from_c_str("{ \"message\": \"hello device that will further isolate humans from each other "
                                "into an ever increasing digital dystopia.\" }");

    struct aws_event_stream_rpc_message_args ping_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_PING,
        .payload = &ping_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_send_protocol_message(
        test_data->client_connection, &ping_args, s_rpc_client_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_PING, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        ping_payload.buffer,
        ping_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    ping_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_PING_RESPONSE;

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_connection_send_protocol_message(
        test_data->server_connection, &ping_args, s_rpc_server_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_PING_RESPONSE, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        ping_payload.buffer,
        ping_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);

    aws_mutex_unlock(&client_test_data.sync_lock);
    aws_mutex_clean_up(&client_test_data.sync_lock);
    aws_condition_variable_clean_up(&client_test_data.sync_cvar);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_protocol_message,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_protocol_message,
    s_fixture_shutdown,
    &s_test_data)

static void s_rpc_client_stream_continuation(
    struct aws_event_stream_rpc_client_continuation_token *token,
    const struct aws_event_stream_rpc_message_args *message_args,
    void *user_data) {
    (void)token;
    struct client_test_data *client_test_data = user_data;

    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->client_message_received = true;
    client_test_data->received_message_type = message_args->message_type;
    aws_byte_buf_init_copy(&client_test_data->received_payload, client_test_data->allocator, message_args->payload);
    aws_mutex_unlock(&client_test_data->sync_lock);
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
}

static void s_rpc_client_stream_continuation_closed(
    struct aws_event_stream_rpc_client_continuation_token *token,
    void *user_data) {
    (void)token;

    struct client_test_data *client_test_data = user_data;
    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->client_token_closed = true;
    aws_mutex_unlock(&client_test_data->sync_lock);
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
}

static bool s_rpc_client_continuation_token_closed_pred(void *arg) {
    struct client_test_data *client_test_data = arg;
    return client_test_data->client_token_closed && client_test_data->server_token_closed;
}

static int s_rpc_server_on_incoming_stream(
    struct aws_event_stream_rpc_server_connection *connection,
    struct aws_event_stream_rpc_server_continuation_token *token,
    struct aws_byte_cursor operation_name,
    struct aws_event_stream_rpc_server_stream_continuation_options *continuation_options,
    void *user_data) {
    (void)connection;
    (void)continuation_options;

    struct client_test_data *client_test_data = user_data;
    client_test_data->server_token = token;
    aws_byte_buf_init_copy_from_cursor(
        &client_test_data->last_seen_operation_name, client_test_data->allocator, operation_name);

    return AWS_OP_SUCCESS;
}

static void s_rpc_server_stream_continuation(
    struct aws_event_stream_rpc_server_continuation_token *token,
    const struct aws_event_stream_rpc_message_args *message_args,
    void *user_data) {
    (void)token;

    struct client_test_data *client_test_data = user_data;
    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->server_message_received = true;
    client_test_data->received_message_type = message_args->message_type;
    aws_byte_buf_init_copy(&client_test_data->received_payload, client_test_data->allocator, message_args->payload);
    aws_mutex_unlock(&client_test_data->sync_lock);
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
}

static void s_rpc_server_stream_continuation_closed(
    struct aws_event_stream_rpc_server_continuation_token *token,
    void *user_data) {
    (void)token;
    struct client_test_data *client_test_data = user_data;
    aws_mutex_lock(&client_test_data->sync_lock);
    client_test_data->server_token_closed = true;
    aws_mutex_unlock(&client_test_data->sync_lock);
    aws_condition_variable_notify_one(&client_test_data->sync_cvar);
}

static int s_test_event_stream_rpc_client_connection_continuation_flow(struct aws_allocator *allocator, void *ctx) {

    struct test_data *test_data = ctx;
    test_data->on_server_message_received = s_rpc_server_connection_protocol_message;
    test_data->on_client_message_received = s_rpc_client_connection_protocol_message;
    test_data->on_new_server_stream = s_rpc_server_on_incoming_stream;
    test_data->on_new_server_continuation = s_rpc_server_stream_continuation;
    test_data->on_server_continuation_closed = s_rpc_server_stream_continuation_closed;

    struct client_test_data client_test_data = {
        .allocator = allocator,
        .sync_cvar = AWS_CONDITION_VARIABLE_INIT,
        .sync_lock = AWS_MUTEX_INIT,
    };

    test_data->user_data = &client_test_data;
    test_data->server_continuation_user_data = &client_test_data;

    struct aws_byte_buf connect_payload = aws_byte_buf_from_c_str("{ \"message\": \" connect message \" }");
    struct aws_event_stream_rpc_message_args connect_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT,
        .payload = &connect_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_send_protocol_message(
        test_data->client_connection, &connect_args, s_rpc_client_message_flush, &client_test_data));

    aws_mutex_lock(&client_test_data.sync_lock);
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    connect_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK;
    connect_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_CONNECTION_ACCEPTED;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_connection_send_protocol_message(
        test_data->server_connection, &connect_args, s_rpc_server_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    struct aws_event_stream_rpc_client_stream_continuation_options continuation_options = {
        .user_data = &client_test_data,
        .on_continuation = s_rpc_client_stream_continuation,
        .on_continuation_closed = s_rpc_client_stream_continuation_closed,
    };

    struct aws_event_stream_rpc_client_continuation_token *client_token =
        aws_event_stream_rpc_client_connection_new_stream(test_data->client_connection, &continuation_options);
    ASSERT_NOT_NULL(client_token);

    struct aws_byte_cursor operation_name = aws_byte_cursor_from_c_str("test_operation");

    struct aws_byte_buf operation_payload = aws_byte_buf_from_c_str("{ \"message\": \" operation payload \" }");
    struct aws_event_stream_rpc_message_args operation_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_MESSAGE,
        .payload = &operation_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_continuation_activate(
        client_token, operation_name, &operation_args, s_rpc_client_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_MESSAGE, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        operation_payload.buffer,
        operation_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    ASSERT_BIN_ARRAYS_EQUALS(
        operation_name.ptr,
        operation_name.len,
        client_test_data.last_seen_operation_name.buffer,
        client_test_data.last_seen_operation_name.len);
    aws_byte_buf_clean_up(&client_test_data.last_seen_operation_name);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    operation_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_TERMINATE_STREAM;
    operation_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_ERROR;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_continuation_send_message(
        client_test_data.server_token, &operation_args, s_rpc_server_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_continuation_token_closed_pred,
        &client_test_data);

    ASSERT_BIN_ARRAYS_EQUALS(
        operation_payload.buffer,
        operation_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_ERROR, client_test_data.received_message_type);

    aws_byte_buf_clean_up(&client_test_data.received_payload);
    aws_event_stream_rpc_client_continuation_release(client_token);

    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);

    aws_mutex_unlock(&client_test_data.sync_lock);
    aws_mutex_clean_up(&client_test_data.sync_lock);
    aws_condition_variable_clean_up(&client_test_data.sync_cvar);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_continuation_flow,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_continuation_flow,
    s_fixture_shutdown,
    &s_test_data)

static int s_test_event_stream_rpc_client_connection_unactivated_continuation_fails(
    struct aws_allocator *allocator,
    void *ctx) {

    struct test_data *test_data = ctx;
    test_data->on_server_message_received = s_rpc_server_connection_protocol_message;
    test_data->on_client_message_received = s_rpc_client_connection_protocol_message;
    test_data->on_new_server_stream = s_rpc_server_on_incoming_stream;
    test_data->on_new_server_continuation = s_rpc_server_stream_continuation;
    test_data->on_server_continuation_closed = s_rpc_server_stream_continuation_closed;

    struct client_test_data client_test_data = {
        .allocator = allocator,
        .sync_cvar = AWS_CONDITION_VARIABLE_INIT,
        .sync_lock = AWS_MUTEX_INIT,
    };

    test_data->user_data = &client_test_data;
    test_data->server_continuation_user_data = &client_test_data;

    struct aws_byte_buf connect_payload = aws_byte_buf_from_c_str("{ \"message\": \" connect message \" }");
    struct aws_event_stream_rpc_message_args connect_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT,
        .payload = &connect_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_send_protocol_message(
        test_data->client_connection, &connect_args, s_rpc_client_message_flush, &client_test_data));

    aws_mutex_lock(&client_test_data.sync_lock);
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    connect_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK;
    connect_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_CONNECTION_ACCEPTED;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_connection_send_protocol_message(
        test_data->server_connection, &connect_args, s_rpc_server_message_flush, &client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    struct aws_event_stream_rpc_client_stream_continuation_options continuation_options = {
        .user_data = &client_test_data,
        .on_continuation = s_rpc_client_stream_continuation,
        .on_continuation_closed = s_rpc_client_stream_continuation_closed,
    };

    struct aws_event_stream_rpc_client_continuation_token *client_token =
        aws_event_stream_rpc_client_connection_new_stream(test_data->client_connection, &continuation_options);
    ASSERT_NOT_NULL(client_token);

    struct aws_byte_buf operation_payload = aws_byte_buf_from_c_str("{ \"message\": \" operation payload \" }");
    struct aws_event_stream_rpc_message_args operation_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_MESSAGE,
        .payload = &operation_payload,
    };

    ASSERT_ERROR(
        AWS_ERROR_EVENT_STREAM_RPC_STREAM_NOT_ACTIVATED,
        aws_event_stream_rpc_client_continuation_send_message(
            client_token, &operation_args, s_rpc_client_message_flush, &client_test_data));

    aws_event_stream_rpc_client_continuation_release(client_token);

    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);

    aws_mutex_unlock(&client_test_data.sync_lock);
    aws_mutex_clean_up(&client_test_data.sync_lock);
    aws_condition_variable_clean_up(&client_test_data.sync_cvar);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_unactivated_continuation_fails,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_unactivated_continuation_fails,
    s_fixture_shutdown,
    &s_test_data)

static int s_test_event_stream_rpc_client_connection_continuation_send_message_on_closed_fails(
    struct aws_allocator *allocator,
    void *ctx) {

    struct test_data *test_data = ctx;
    test_data->on_server_message_received = s_rpc_server_connection_protocol_message;
    test_data->on_client_message_received = s_rpc_client_connection_protocol_message;
    test_data->on_new_server_stream = s_rpc_server_on_incoming_stream;
    test_data->on_new_server_continuation = s_rpc_server_stream_continuation;
    test_data->on_server_continuation_closed = s_rpc_server_stream_continuation_closed;

    struct client_test_data client_test_data = {
        .allocator = allocator,
        .sync_cvar = AWS_CONDITION_VARIABLE_INIT,
        .sync_lock = AWS_MUTEX_INIT,
    };

    test_data->user_data = &client_test_data;
    test_data->server_continuation_user_data = &client_test_data;

    /* client sends CONNECT */
    struct aws_byte_buf connect_payload = aws_byte_buf_from_c_str("{ \"message\": \" connect message \" }");
    struct aws_event_stream_rpc_message_args connect_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT,
        .payload = &connect_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_send_protocol_message(
        test_data->client_connection, &connect_args, s_rpc_client_message_flush, &client_test_data));

    aws_mutex_lock(&client_test_data.sync_lock);

    /* ...wait until sent and received... */
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    /* server sends CONNECT_ACK */
    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    connect_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK;
    connect_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_CONNECTION_ACCEPTED;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_connection_send_protocol_message(
        test_data->server_connection, &connect_args, s_rpc_server_message_flush, &client_test_data));

    /* ...wait until sent and received... */
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    /* client sends message creating new stream */
    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    struct aws_event_stream_rpc_client_stream_continuation_options continuation_options = {
        .user_data = &client_test_data,
        .on_continuation = s_rpc_client_stream_continuation,
        .on_continuation_closed = s_rpc_client_stream_continuation_closed,
    };

    struct aws_event_stream_rpc_client_continuation_token *client_token =
        aws_event_stream_rpc_client_connection_new_stream(test_data->client_connection, &continuation_options);
    ASSERT_NOT_NULL(client_token);

    struct aws_byte_cursor operation_name = aws_byte_cursor_from_c_str("test_operation");

    struct aws_byte_buf operation_payload = aws_byte_buf_from_c_str("{ \"message\": \" operation payload \" }");
    struct aws_event_stream_rpc_message_args operation_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_MESSAGE,
        .payload = &operation_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_continuation_activate(
        client_token, operation_name, &operation_args, s_rpc_client_message_flush, &client_test_data));

    /* ...wait until sent and received... */
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        &client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_MESSAGE, client_test_data.received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        operation_payload.buffer,
        operation_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);
    aws_byte_buf_clean_up(&client_test_data.received_payload);

    ASSERT_BIN_ARRAYS_EQUALS(
        operation_name.ptr,
        operation_name.len,
        client_test_data.last_seen_operation_name.buffer,
        client_test_data.last_seen_operation_name.len);
    aws_byte_buf_clean_up(&client_test_data.last_seen_operation_name);

    /* server sends response with TERMINATE_STREAM flag set */
    client_test_data.received_message_type = 0;
    client_test_data.client_message_received = false;
    client_test_data.client_message_sent = false;
    client_test_data.server_message_received = false;
    client_test_data.server_message_sent = false;

    operation_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_TERMINATE_STREAM;
    operation_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_ERROR;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_continuation_send_message(
        client_test_data.server_token, &operation_args, s_rpc_server_message_flush, &client_test_data));

    /* ...wait until sent and received... */
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        &client_test_data);

    /* ...wait until client stream closed... */
    aws_condition_variable_wait_pred(
        &client_test_data.sync_cvar,
        &client_test_data.sync_lock,
        s_rpc_client_continuation_token_closed_pred,
        &client_test_data);

    ASSERT_BIN_ARRAYS_EQUALS(
        operation_payload.buffer,
        operation_payload.len,
        client_test_data.received_payload.buffer,
        client_test_data.received_payload.len);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_ERROR, client_test_data.received_message_type);

    aws_byte_buf_clean_up(&client_test_data.received_payload);

    /* should not be allowed to send further stream messages */
    ASSERT_ERROR(
        AWS_ERROR_EVENT_STREAM_RPC_STREAM_CLOSED,
        aws_event_stream_rpc_client_continuation_send_message(
            client_token, &operation_args, s_rpc_client_message_flush, &client_test_data));

    aws_event_stream_rpc_client_continuation_release(client_token);

    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);

    aws_mutex_unlock(&client_test_data.sync_lock);
    aws_mutex_clean_up(&client_test_data.sync_lock);
    aws_condition_variable_clean_up(&client_test_data.sync_cvar);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_continuation_send_message_on_closed_fails,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_continuation_send_message_on_closed_fails,
    s_fixture_shutdown,
    &s_test_data)

static int s_test_event_stream_rpc_client_connection_continuation_duplicated_activate_fails(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)allocator;

    struct test_data *test_data = ctx;
    test_data->on_server_message_received = s_rpc_server_connection_protocol_message;
    test_data->on_client_message_received = s_rpc_client_connection_protocol_message;
    test_data->on_new_server_stream = s_rpc_server_on_incoming_stream;
    test_data->on_new_server_continuation = s_rpc_server_stream_continuation;
    test_data->on_server_continuation_closed = s_rpc_server_stream_continuation_closed;

    struct client_test_data *client_test_data = &test_data->client_test_data;

    test_data->user_data = client_test_data;
    test_data->server_continuation_user_data = client_test_data;

    struct aws_byte_buf connect_payload = aws_byte_buf_from_c_str("{ \"message\": \" connect message \" }");
    struct aws_event_stream_rpc_message_args connect_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT,
        .payload = &connect_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_connection_send_protocol_message(
        test_data->client_connection, &connect_args, s_rpc_client_message_flush, client_test_data));

    aws_mutex_lock(&client_test_data->sync_lock);
    aws_condition_variable_wait_pred(
        &client_test_data->sync_cvar,
        &client_test_data->sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT, client_test_data->received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data->received_payload.buffer,
        client_test_data->received_payload.len);
    aws_byte_buf_clean_up(&client_test_data->received_payload);

    client_test_data->received_message_type = 0;
    client_test_data->client_message_received = false;
    client_test_data->client_message_sent = false;
    client_test_data->server_message_received = false;
    client_test_data->server_message_sent = false;

    connect_args.message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK;
    connect_args.message_flags = AWS_EVENT_STREAM_RPC_MESSAGE_FLAG_CONNECTION_ACCEPTED;

    ASSERT_SUCCESS(aws_event_stream_rpc_server_connection_send_protocol_message(
        test_data->server_connection, &connect_args, s_rpc_server_message_flush, client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data->sync_cvar,
        &client_test_data->sync_lock,
        s_rpc_server_message_transmission_completed_pred,
        client_test_data);

    ASSERT_INT_EQUALS(AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_CONNECT_ACK, client_test_data->received_message_type);
    ASSERT_BIN_ARRAYS_EQUALS(
        connect_payload.buffer,
        connect_payload.len,
        client_test_data->received_payload.buffer,
        client_test_data->received_payload.len);
    aws_byte_buf_clean_up(&client_test_data->received_payload);

    client_test_data->received_message_type = 0;
    client_test_data->client_message_received = false;
    client_test_data->client_message_sent = false;
    client_test_data->server_message_received = false;
    client_test_data->server_message_sent = false;

    struct aws_event_stream_rpc_client_stream_continuation_options continuation_options = {
        .user_data = client_test_data,
        .on_continuation = s_rpc_client_stream_continuation,
        .on_continuation_closed = s_rpc_client_stream_continuation_closed,
    };

    struct aws_event_stream_rpc_client_continuation_token *client_token =
        aws_event_stream_rpc_client_connection_new_stream(test_data->client_connection, &continuation_options);
    ASSERT_NOT_NULL(client_token);

    struct aws_byte_cursor operation_name = aws_byte_cursor_from_c_str("test_operation");

    struct aws_byte_buf operation_payload = aws_byte_buf_from_c_str("{ \"message\": \" operation payload \" }");
    struct aws_event_stream_rpc_message_args operation_args = {
        .headers_count = 0,
        .headers = NULL,
        .message_type = AWS_EVENT_STREAM_RPC_MESSAGE_TYPE_APPLICATION_MESSAGE,
        .payload = &operation_payload,
    };

    ASSERT_SUCCESS(aws_event_stream_rpc_client_continuation_activate(
        client_token, operation_name, &operation_args, s_rpc_client_message_flush, client_test_data));

    aws_condition_variable_wait_pred(
        &client_test_data->sync_cvar,
        &client_test_data->sync_lock,
        s_rpc_client_message_transmission_completed_pred,
        client_test_data);
    aws_byte_buf_clean_up(&client_test_data->received_payload);
    aws_byte_buf_clean_up(&client_test_data->last_seen_operation_name);

    ASSERT_ERROR(
        AWS_ERROR_INVALID_STATE,
        aws_event_stream_rpc_client_continuation_activate(
            client_token, operation_name, &operation_args, s_rpc_client_message_flush, client_test_data));

    aws_event_stream_rpc_client_continuation_release(client_token);

    aws_event_stream_rpc_client_connection_close(test_data->client_connection, AWS_ERROR_SUCCESS);
    aws_event_stream_rpc_server_connection_close(test_data->server_connection, AWS_ERROR_SUCCESS);

    aws_condition_variable_wait_pred(
        &client_test_data->sync_cvar,
        &client_test_data->sync_lock,
        s_rpc_client_continuation_token_closed_pred,
        client_test_data);

    aws_mutex_unlock(&client_test_data->sync_lock);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_event_stream_rpc_client_connection_continuation_duplicated_activate_fails,
    s_fixture_setup,
    s_test_event_stream_rpc_client_connection_continuation_duplicated_activate_fails,
    s_fixture_shutdown,
    &s_test_data)
