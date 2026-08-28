/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt311_testing_utils.h"

#include <aws/common/math.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/host_resolver.h>
#include <aws/mqtt/private/client_impl.h>
#include <aws/testing/aws_test_harness.h>

#include "mqtt_mock_server_handler.h"

static void s_on_incoming_channel_setup_fn(
    struct aws_server_bootstrap *bootstrap,
    int error_code,
    struct aws_channel *channel,
    void *user_data) {
    (void)bootstrap;
    struct mqtt_connection_state_test *state_test_data = user_data;

    state_test_data->error = error_code;

    if (!error_code) {
        aws_mutex_lock(&state_test_data->lock);
        state_test_data->server_disconnect_completed = false;
        aws_mutex_unlock(&state_test_data->lock);
        AWS_LOGF_DEBUG(TEST_LOG_SUBJECT, "server channel setup completed");

        state_test_data->server_channel = channel;
        struct aws_channel_slot *test_handler_slot = aws_channel_slot_new(channel);
        aws_channel_slot_insert_end(channel, test_handler_slot);
        mqtt_mock_server_handler_update_slot(state_test_data->mock_server, test_handler_slot);
        aws_channel_slot_set_handler(test_handler_slot, state_test_data->mock_server);
    }
}

static void s_on_incoming_channel_shutdown_fn(
    struct aws_server_bootstrap *bootstrap,
    int error_code,
    struct aws_channel *channel,
    void *user_data) {
    (void)bootstrap;
    (void)error_code;
    (void)channel;
    struct mqtt_connection_state_test *state_test_data = user_data;
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->server_disconnect_completed = true;
    AWS_LOGF_DEBUG(TEST_LOG_SUBJECT, "server channel shutdown completed");
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static void s_on_listener_destroy(struct aws_server_bootstrap *bootstrap, void *user_data) {
    (void)bootstrap;
    struct mqtt_connection_state_test *state_test_data = user_data;
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->listener_destroyed = true;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_listener_destroyed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->listener_destroyed;
}

static void s_wait_on_listener_cleanup(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_listener_destroyed, state_test_data);
    aws_mutex_unlock(&state_test_data->lock);
}

static void s_on_connection_interrupted(struct aws_mqtt_client_connection *connection, int error_code, void *userdata) {
    (void)connection;
    (void)error_code;
    struct mqtt_connection_state_test *state_test_data = userdata;

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->connection_interrupted = true;
    state_test_data->interruption_error = error_code;
    aws_mutex_unlock(&state_test_data->lock);
    AWS_LOGF_DEBUG(TEST_LOG_SUBJECT, "connection interrupted");
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_connection_interrupted(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->connection_interrupted;
}

void aws_test311_wait_for_interrupt_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_connection_interrupted, state_test_data);
    state_test_data->connection_interrupted = false;
    aws_mutex_unlock(&state_test_data->lock);
}

static void s_on_connection_resumed(
    struct aws_mqtt_client_connection *connection,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {
    (void)connection;
    (void)return_code;
    (void)session_present;
    AWS_LOGF_DEBUG(TEST_LOG_SUBJECT, "reconnect completed");

    struct mqtt_connection_state_test *state_test_data = userdata;

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->connection_resumed = true;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_connection_resumed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->connection_resumed;
}

void aws_test311_wait_for_reconnect_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_connection_resumed, state_test_data);
    state_test_data->connection_resumed = false;
    aws_mutex_unlock(&state_test_data->lock);
}

static void s_on_connection_success(
    struct aws_mqtt_client_connection *connection,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {
    (void)connection;
    struct mqtt_connection_state_test *state_test_data = userdata;
    aws_mutex_lock(&state_test_data->lock);

    state_test_data->session_present = session_present;
    state_test_data->mqtt_return_code = return_code;
    state_test_data->connection_success = true;
    aws_mutex_unlock(&state_test_data->lock);

    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static void s_on_connection_failure(struct aws_mqtt_client_connection *connection, int error_code, void *userdata) {
    (void)connection;
    struct mqtt_connection_state_test *state_test_data = userdata;
    aws_mutex_lock(&state_test_data->lock);

    state_test_data->error = error_code;
    state_test_data->connection_failure = true;
    aws_mutex_unlock(&state_test_data->lock);

    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_connection_succeed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->connection_success;
}

static bool s_is_connection_failed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->connection_failure;
}

void aws_test311_wait_for_connection_to_succeed(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_connection_succeed, state_test_data);
    state_test_data->connection_success = false;
    aws_mutex_unlock(&state_test_data->lock);
}

void aws_test311_wait_for_connection_to_fail(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_connection_failed, state_test_data);
    state_test_data->connection_failure = false;
    aws_mutex_unlock(&state_test_data->lock);
}

static bool s_is_termination_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->connection_terminated;
}

static void s_wait_for_termination_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_termination_completed, state_test_data);
    state_test_data->connection_terminated = false;
    aws_mutex_unlock(&state_test_data->lock);
}

void aws_test311_on_connection_termination_fn(void *userdata) {
    struct mqtt_connection_state_test *state_test_data = (struct mqtt_connection_state_test *)userdata;

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->connection_termination_calls += 1;
    state_test_data->connection_terminated = true;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static void s_on_any_publish_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata) {
    (void)connection;
    struct mqtt_connection_state_test *state_test_data = userdata;

    struct aws_byte_buf payload_cp;
    aws_byte_buf_init_copy_from_cursor(&payload_cp, state_test_data->allocator, *payload);
    struct aws_byte_buf topic_cp;
    aws_byte_buf_init_copy_from_cursor(&topic_cp, state_test_data->allocator, *topic);
    struct received_publish_packet received_packet = {
        .payload = payload_cp,
        .topic = topic_cp,
        .dup = dup,
        .qos = qos,
        .retain = retain,
    };

    aws_mutex_lock(&state_test_data->lock);
    aws_array_list_push_back(&state_test_data->any_published_messages, &received_packet);
    state_test_data->any_publishes_received++;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

/**
 * sets up a unix domain socket server and socket options. Creates an mqtt connection configured to use
 * the domain socket.
 */
int aws_test311_setup_mqtt_server_fn(struct aws_allocator *allocator, void *ctx) {
    aws_mqtt_library_init(allocator);

    struct mqtt_connection_state_test *state_test_data = ctx;

    AWS_ZERO_STRUCT(*state_test_data);

    state_test_data->allocator = allocator;
    state_test_data->el_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    state_test_data->mock_server = new_mqtt_mock_server(allocator);
    ASSERT_NOT_NULL(state_test_data->mock_server);

    state_test_data->server_bootstrap = aws_server_bootstrap_new(allocator, state_test_data->el_group);
    ASSERT_NOT_NULL(state_test_data->server_bootstrap);

    struct aws_socket_options socket_options = {
        .connect_timeout_ms = 100,
        .domain = AWS_SOCKET_LOCAL,
    };

    state_test_data->socket_options = socket_options;
    ASSERT_SUCCESS(aws_condition_variable_init(&state_test_data->cvar));
    ASSERT_SUCCESS(aws_mutex_init(&state_test_data->lock));

    aws_socket_endpoint_init_local_address_for_test(&state_test_data->endpoint);

    struct aws_server_socket_channel_bootstrap_options server_bootstrap_options = {
        .bootstrap = state_test_data->server_bootstrap,
        .host_name = state_test_data->endpoint.address,
        .port = state_test_data->endpoint.port,
        .socket_options = &state_test_data->socket_options,
        .incoming_callback = s_on_incoming_channel_setup_fn,
        .shutdown_callback = s_on_incoming_channel_shutdown_fn,
        .destroy_callback = s_on_listener_destroy,
        .user_data = state_test_data,
    };
    state_test_data->listener = aws_server_bootstrap_new_socket_listener(&server_bootstrap_options);

    ASSERT_NOT_NULL(state_test_data->listener);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = state_test_data->el_group,
        .max_entries = 1,
    };
    state_test_data->host_resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = state_test_data->el_group,
        .user_data = state_test_data,
        .host_resolver = state_test_data->host_resolver,
    };

    state_test_data->client_bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    state_test_data->mqtt_client = aws_mqtt_client_new(allocator, state_test_data->client_bootstrap);
    state_test_data->mqtt_connection = aws_mqtt_client_connection_new(state_test_data->mqtt_client);
    ASSERT_NOT_NULL(state_test_data->mqtt_connection);

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_connection_interruption_handlers(
        state_test_data->mqtt_connection,
        s_on_connection_interrupted,
        state_test_data,
        s_on_connection_resumed,
        state_test_data));

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_connection_result_handlers(
        state_test_data->mqtt_connection,
        s_on_connection_success,
        state_test_data,
        s_on_connection_failure,
        state_test_data));

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_on_any_publish_handler(
        state_test_data->mqtt_connection, s_on_any_publish_received, state_test_data));

    ASSERT_SUCCESS(aws_array_list_init_dynamic(
        &state_test_data->published_messages, allocator, 4, sizeof(struct received_publish_packet)));
    ASSERT_SUCCESS(aws_array_list_init_dynamic(
        &state_test_data->any_published_messages, allocator, 4, sizeof(struct received_publish_packet)));
    ASSERT_SUCCESS(aws_array_list_init_dynamic(&state_test_data->qos_returned, allocator, 2, sizeof(uint8_t)));

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_connection_termination_handler(
        state_test_data->mqtt_connection, aws_test311_on_connection_termination_fn, state_test_data));

    return AWS_OP_SUCCESS;
}

static void s_received_publish_packet_list_clean_up(struct aws_array_list *list) {
    for (size_t i = 0; i < aws_array_list_length(list); ++i) {
        struct received_publish_packet *val_ptr = NULL;
        aws_array_list_get_at_ptr(list, (void **)&val_ptr, i);
        aws_byte_buf_clean_up(&val_ptr->payload);
        aws_byte_buf_clean_up(&val_ptr->topic);
    }
    aws_array_list_clean_up(list);
}

int aws_test311_clean_up_mqtt_server_fn(struct aws_allocator *allocator, int setup_result, void *ctx) {
    (void)allocator;

    if (!setup_result) {
        struct mqtt_connection_state_test *state_test_data = ctx;

        s_received_publish_packet_list_clean_up(&state_test_data->published_messages);
        s_received_publish_packet_list_clean_up(&state_test_data->any_published_messages);
        aws_array_list_clean_up(&state_test_data->qos_returned);
        aws_mqtt_client_connection_release(state_test_data->mqtt_connection);

        s_wait_for_termination_to_complete(state_test_data);
        ASSERT_UINT_EQUALS(1, state_test_data->connection_termination_calls);

        aws_mqtt_client_release(state_test_data->mqtt_client);
        aws_client_bootstrap_release(state_test_data->client_bootstrap);
        aws_host_resolver_release(state_test_data->host_resolver);
        aws_server_bootstrap_destroy_socket_listener(state_test_data->server_bootstrap, state_test_data->listener);
        s_wait_on_listener_cleanup(state_test_data);
        aws_server_bootstrap_release(state_test_data->server_bootstrap);
        aws_event_loop_group_release(state_test_data->el_group);
        destroy_mqtt_mock_server(state_test_data->mock_server);
    }

    aws_mqtt_library_clean_up();
    return AWS_OP_SUCCESS;
}

void aws_test311_on_connection_complete_fn(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {
    (void)connection;
    struct mqtt_connection_state_test *state_test_data = userdata;
    aws_mutex_lock(&state_test_data->lock);

    state_test_data->session_present = session_present;
    state_test_data->mqtt_return_code = return_code;
    state_test_data->error = error_code;
    state_test_data->connection_completed = true;
    aws_mutex_unlock(&state_test_data->lock);

    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_connection_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->connection_completed;
}

void aws_test311_wait_for_connection_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_connection_completed, state_test_data);
    state_test_data->connection_completed = false;
    aws_mutex_unlock(&state_test_data->lock);
}

void aws_test311_on_disconnect_fn(struct aws_mqtt_client_connection *connection, void *userdata) {
    (void)connection;
    struct mqtt_connection_state_test *state_test_data = userdata;
    AWS_LOGF_DEBUG(TEST_LOG_SUBJECT, "disconnect completed");
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->client_disconnect_completed = true;
    aws_mutex_unlock(&state_test_data->lock);

    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_disconnect_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->client_disconnect_completed && state_test_data->server_disconnect_completed;
}

void aws_test311_wait_for_disconnect_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_disconnect_completed, state_test_data);
    state_test_data->client_disconnect_completed = false;
    state_test_data->server_disconnect_completed = false;
    aws_mutex_unlock(&state_test_data->lock);
}

static bool s_is_any_publish_received(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->any_publishes_received == state_test_data->expected_any_publishes;
}

void aws_test311_wait_for_any_publish(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_any_publish_received, state_test_data);
    state_test_data->any_publishes_received = 0;
    state_test_data->expected_any_publishes = 0;
    aws_mutex_unlock(&state_test_data->lock);
}

void aws_test311_on_publish_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata) {

    (void)connection;
    (void)topic;
    struct mqtt_connection_state_test *state_test_data = userdata;

    struct aws_byte_buf payload_cp;
    aws_byte_buf_init_copy_from_cursor(&payload_cp, state_test_data->allocator, *payload);
    struct aws_byte_buf topic_cp;
    aws_byte_buf_init_copy_from_cursor(&topic_cp, state_test_data->allocator, *topic);
    struct received_publish_packet received_packet = {
        .payload = payload_cp,
        .topic = topic_cp,
        .dup = dup,
        .qos = qos,
        .retain = retain,
    };

    aws_mutex_lock(&state_test_data->lock);
    aws_array_list_push_back(&state_test_data->published_messages, &received_packet);
    state_test_data->publishes_received++;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_publish_received(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->publishes_received == state_test_data->expected_publishes;
}

void aws_test311_wait_for_publish(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_publish_received, state_test_data);
    state_test_data->publishes_received = 0;
    state_test_data->expected_publishes = 0;
    aws_mutex_unlock(&state_test_data->lock);
}

void aws_test311_on_suback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_byte_cursor *topic,
    enum aws_mqtt_qos qos,
    int error_code,
    void *userdata) {
    (void)connection;
    (void)packet_id;
    (void)topic;

    struct mqtt_connection_state_test *state_test_data = userdata;

    aws_mutex_lock(&state_test_data->lock);
    if (!error_code) {
        aws_array_list_push_back(&state_test_data->qos_returned, &qos);
    }
    state_test_data->subscribe_completed = true;
    state_test_data->subscribe_complete_error = error_code;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_subscribe_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->subscribe_completed;
}

void aws_test311_wait_for_subscribe_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_is_subscribe_completed, state_test_data);
    state_test_data->subscribe_completed = false;
    aws_mutex_unlock(&state_test_data->lock);
}

void aws_test311_on_multi_suback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_array_list *topic_subacks, /* contains aws_mqtt_topic_subscription pointers */
    int error_code,
    void *userdata) {
    (void)connection;
    (void)packet_id;
    (void)topic_subacks;
    (void)error_code;

    struct mqtt_connection_state_test *state_test_data = userdata;

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->subscribe_completed = true;
    state_test_data->subscribe_complete_error = error_code;

    if (!error_code) {
        size_t length = aws_array_list_length(topic_subacks);
        for (size_t i = 0; i < length; ++i) {
            struct aws_mqtt_topic_subscription *subscription = NULL;
            aws_array_list_get_at(topic_subacks, &subscription, i);
            aws_array_list_push_back(&state_test_data->qos_returned, &subscription->qos);
        }
    }
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

void aws_test311_on_op_complete(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    int error_code,
    void *userdata) {
    (void)connection;
    (void)packet_id;

    struct mqtt_connection_state_test *state_test_data = userdata;
    AWS_LOGF_DEBUG(TEST_LOG_SUBJECT, "pub op completed");
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->ops_completed++;
    state_test_data->op_complete_error = error_code;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_is_ops_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->ops_completed == state_test_data->expected_ops_completed;
}

void aws_test311_wait_for_ops_completed(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_for_pred(
        &state_test_data->cvar, &state_test_data->lock, 10000000000, s_is_ops_completed, state_test_data);
    aws_mutex_unlock(&state_test_data->lock);
}