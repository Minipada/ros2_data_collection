/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt_mock_server_handler.h"

#include <aws/mqtt/private/client_impl.h>

#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/host_resolver.h>

#include <aws/common/clock.h>
#include <aws/common/condition_variable.h>

#include <aws/testing/aws_test_harness.h>

static const int TEST_LOG_SUBJECT = 60000;
static const int ONE_SEC = 1000000000;

struct received_publish_packet {
    struct aws_byte_buf topic;
    struct aws_byte_buf payload;
    bool dup;
    enum aws_mqtt_qos qos;
    bool retain;
};

struct mqtt_connection_state_test {
    struct aws_allocator *allocator;
    struct aws_channel *server_channel;
    struct aws_channel_handler *mock_server;
    struct aws_client_bootstrap *client_bootstrap;
    struct aws_server_bootstrap *server_bootstrap;
    struct aws_event_loop_group *el_group;
    struct aws_host_resolver *host_resolver;
    struct aws_socket_endpoint endpoint;
    struct aws_socket *listener;
    struct aws_mqtt_client *mqtt_client;
    struct aws_mqtt_client_connection *mqtt_connection;
    struct aws_socket_options socket_options;
    bool session_present;
    bool connection_completed;
    bool client_disconnect_completed;
    bool server_disconnect_completed;
    bool connection_interrupted;
    bool connection_resumed;
    bool subscribe_completed;
    bool listener_destroyed;
    int interruption_error;
    int subscribe_complete_error;
    int op_complete_error;
    enum aws_mqtt_connect_return_code mqtt_return_code;
    int error;
    struct aws_condition_variable cvar;
    struct aws_mutex lock;
    /* any published messages from mock server, that you may not subscribe to. (Which should not happen in real life) */
    struct aws_array_list any_published_messages; /* list of struct received_publish_packet */
    size_t any_publishes_received;
    size_t expected_any_publishes;
    /* the published messages from mock server, that you did subscribe to. */
    struct aws_array_list published_messages; /* list of struct received_publish_packet */
    size_t publishes_received;
    size_t expected_publishes;
    /* The returned QoS from mock server */
    struct aws_array_list qos_returned; /* list of uint_8 */
    size_t ops_completed;
    size_t expected_ops_completed;
};

static struct mqtt_connection_state_test test_data = {0};

/* ========== HELPER FUNCTIONS FOR THE TEST ========== */

static void s_operation_statistics_on_any_publish_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata);

static void s_operation_statistics_on_incoming_channel_setup_fn(
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

static void s_operation_statistics_on_incoming_channel_shutdown_fn(
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

static void s_operation_statistics_on_listener_destroy(struct aws_server_bootstrap *bootstrap, void *user_data) {
    (void)bootstrap;
    struct mqtt_connection_state_test *state_test_data = user_data;
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->listener_destroyed = true;
    aws_mutex_unlock(&state_test_data->lock);
    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_operation_statistics_is_listener_destroyed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->listener_destroyed;
}

static void s_operation_statistics_wait_on_listener_cleanup(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_operation_statistics_is_listener_destroyed, state_test_data);
    aws_mutex_unlock(&state_test_data->lock);
}

static void s_operation_statistics_on_connection_interrupted(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    void *userdata) {
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

static void s_operation_statistics_on_connection_resumed(
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

/** sets up a unix domain socket server and socket options. Creates an mqtt connection configured to use
 * the domain socket.
 */
static int s_operation_statistics_setup_mqtt_server_fn(struct aws_allocator *allocator, void *ctx) {
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
        .incoming_callback = s_operation_statistics_on_incoming_channel_setup_fn,
        .shutdown_callback = s_operation_statistics_on_incoming_channel_shutdown_fn,
        .destroy_callback = s_operation_statistics_on_listener_destroy,
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
        s_operation_statistics_on_connection_interrupted,
        state_test_data,
        s_operation_statistics_on_connection_resumed,
        state_test_data));

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_on_any_publish_handler(
        state_test_data->mqtt_connection, s_operation_statistics_on_any_publish_received, state_test_data));

    ASSERT_SUCCESS(aws_array_list_init_dynamic(
        &state_test_data->published_messages, allocator, 4, sizeof(struct received_publish_packet)));
    ASSERT_SUCCESS(aws_array_list_init_dynamic(
        &state_test_data->any_published_messages, allocator, 4, sizeof(struct received_publish_packet)));
    ASSERT_SUCCESS(aws_array_list_init_dynamic(&state_test_data->qos_returned, allocator, 2, sizeof(uint8_t)));
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

static int s_operation_statistics_clean_up_mqtt_server_fn(
    struct aws_allocator *allocator,
    int setup_result,
    void *ctx) {
    (void)allocator;

    if (!setup_result) {
        struct mqtt_connection_state_test *state_test_data = ctx;

        s_received_publish_packet_list_clean_up(&state_test_data->published_messages);
        s_received_publish_packet_list_clean_up(&state_test_data->any_published_messages);
        aws_array_list_clean_up(&state_test_data->qos_returned);
        aws_mqtt_client_connection_release(state_test_data->mqtt_connection);
        aws_mqtt_client_release(state_test_data->mqtt_client);
        aws_client_bootstrap_release(state_test_data->client_bootstrap);
        aws_host_resolver_release(state_test_data->host_resolver);
        aws_server_bootstrap_destroy_socket_listener(state_test_data->server_bootstrap, state_test_data->listener);
        s_operation_statistics_wait_on_listener_cleanup(state_test_data);
        aws_server_bootstrap_release(state_test_data->server_bootstrap);
        aws_event_loop_group_release(state_test_data->el_group);
        destroy_mqtt_mock_server(state_test_data->mock_server);
    }

    aws_mqtt_library_clean_up();
    return AWS_OP_SUCCESS;
}

static void s_operation_statistics_on_connection_complete_fn(
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

static bool s_operation_statistics_is_connection_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->connection_completed;
}

static void s_operation_statistics_wait_for_connection_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar,
        &state_test_data->lock,
        s_operation_statistics_is_connection_completed,
        state_test_data);
    state_test_data->connection_completed = false;
    aws_mutex_unlock(&state_test_data->lock);
}

void s_operation_statistics_on_disconnect_fn(struct aws_mqtt_client_connection *connection, void *userdata) {
    (void)connection;
    struct mqtt_connection_state_test *state_test_data = userdata;
    AWS_LOGF_DEBUG(TEST_LOG_SUBJECT, "disconnect completed");
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->client_disconnect_completed = true;
    aws_mutex_unlock(&state_test_data->lock);

    aws_condition_variable_notify_one(&state_test_data->cvar);
}

static bool s_operation_statistics_is_disconnect_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->client_disconnect_completed && state_test_data->server_disconnect_completed;
}

static void s_operation_statistics_wait_for_disconnect_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar,
        &state_test_data->lock,
        s_operation_statistics_is_disconnect_completed,
        state_test_data);
    state_test_data->client_disconnect_completed = false;
    state_test_data->server_disconnect_completed = false;
    aws_mutex_unlock(&state_test_data->lock);
}

static void s_operation_statistics_on_any_publish_received(
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

static void s_operation_statistics_on_publish_received(
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

static void s_operation_statistics_on_suback(
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

static void s_on_op_complete(
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

static void s_wait_for_ops_completed(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_for_pred(
        &state_test_data->cvar, &state_test_data->lock, 10000000000, s_is_ops_completed, state_test_data);
    aws_mutex_unlock(&state_test_data->lock);
}

static bool s_operation_statistics_is_subscribe_completed(void *arg) {
    struct mqtt_connection_state_test *state_test_data = arg;
    return state_test_data->subscribe_completed;
}

static void s_operation_statistics_wait_for_subscribe_to_complete(struct mqtt_connection_state_test *state_test_data) {
    aws_mutex_lock(&state_test_data->lock);
    aws_condition_variable_wait_pred(
        &state_test_data->cvar, &state_test_data->lock, s_operation_statistics_is_subscribe_completed, state_test_data);
    state_test_data->subscribe_completed = false;
    aws_mutex_unlock(&state_test_data->lock);
}

/* ========== PUBLISH TESTS ========== */

/**
 * Make a connection, tell the server NOT to send Acks, publish and immediately check the statistics to make sure
 * it is incomplete, then wait a little bit and check that it was properly marked as UnAcked, then send the PubAck
 * confirm statistics are zero, and then disconnect
 */
static int s_test_mqtt_operation_statistics_simple_publish(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message");

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* Stop ACKS so we make sure the operation statistics has time to allow us to identify we sent a packet */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* We want to wait for 1 operation to complete */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* Publish a packet */
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        s_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    /* Wait a little bit to allow the code to put the packet into the socket from the queue, allowing it
     * to be unacked. If we check right away, we may or may not see it in the un-acked statistics */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Make sure the sizes are correct and there is only one operation waiting
     * (The size of the topic, the size of the payload, 2 for the header, 2 for the packet ID) */
    uint64_t expected_packet_size = pub_topic.len + payload_1.len + 4;
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(1, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(1, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    /* Send the PubAck and wait for the client to receive it */
    mqtt_mock_server_send_puback(state_test_data->mock_server, packet_id_1);
    s_wait_for_ops_completed(state_test_data);

    /* Make sure the operation values are back to zero now that the publish went out */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_simple_publish,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_simple_publish,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Make five publishes offline, confirm they are in the incomplete statistics, make a connection to a server
 * that does not send ACKs, send ConnAck, confirm five publishes are in unacked statistics, send PubAcks,
 * confirm operation statistics are zero, and then disconnect
 */
static int s_test_mqtt_operation_statistics_offline_publish(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic_1");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Test Message");

    /* Stop ACKS so we make sure the operation statistics has time to allow us to identify we sent a packet */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    uint16_t pub_packet_id_1 = 0;
    uint16_t pub_packet_id_2 = 0;
    uint16_t pub_packet_id_3 = 0;
    uint16_t pub_packet_id_4 = 0;
    uint16_t pub_packet_id_5 = 0;

    /* Publish the five packets */
    for (int i = 0; i < 5; i++) {
        uint16_t packet = aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload,
            s_on_op_complete,
            state_test_data);
        ASSERT_TRUE(packet > 0);

        if (i == 0) {
            pub_packet_id_1 = packet;
        } else if (i == 1) {
            pub_packet_id_2 = packet;
        } else if (i == 2) {
            pub_packet_id_3 = packet;
        } else if (i == 3) {
            pub_packet_id_4 = packet;
        } else {
            pub_packet_id_5 = packet;
        }
    }

    /* Wait a little bit to make sure the client has processed them (and NOT put them in the un-acked statistics) */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Make sure the sizes are correct and there is five operations waiting
     * Each packet size = (The size of the topic, the size of the payload, 2 for the header, 2 for the packet ID) */
    uint64_t expected_packet_size = (pub_topic.len + payload.len + 4) * 5;
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(5, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    // The UnAcked operations should be zero, because we are NOT connected
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* We want to wait for 5 operations to complete */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 5;
    aws_mutex_unlock(&state_test_data->lock);

    /* Send the PubAck for each packet and wait for the client to receive it */
    mqtt_mock_server_send_puback(state_test_data->mock_server, pub_packet_id_1);
    mqtt_mock_server_send_puback(state_test_data->mock_server, pub_packet_id_2);
    mqtt_mock_server_send_puback(state_test_data->mock_server, pub_packet_id_3);
    mqtt_mock_server_send_puback(state_test_data->mock_server, pub_packet_id_4);
    mqtt_mock_server_send_puback(state_test_data->mock_server, pub_packet_id_5);
    s_wait_for_ops_completed(state_test_data);

    /* Make sure the operation values are back to zero now that the publish went out */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_offline_publish,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_offline_publish,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Make five publishes offline and confirm the operation statistics properly tracks them only in the incomplete
 * operations, before connecting and confirming post-connect they are also in unacked in the operation statistics.
 * Then disconnect and confirm the operation statistics are still correct post-disconnect
 */
static int s_test_mqtt_operation_statistics_disconnect_publish(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic_1");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Test Message");

    /* Stop ACKS so we make sure the operation statistics has time to allow us to identify we sent a packet */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* Publish the five packets */
    for (int i = 0; i < 5; i++) {
        uint16_t packet = aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload,
            s_on_op_complete,
            state_test_data);
        ASSERT_TRUE(packet > 0);
    }

    /* Wait a little bit to make sure the client has processed them (and NOT put them in the un-acked statistics) */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Make sure the sizes are correct and there is five operations waiting
     * Each packet size = (The size of the topic, the size of the payload, 2 for the header, 2 for the packet ID) */
    uint64_t expected_packet_size = (pub_topic.len + payload.len + 4) * 5;
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(5, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    // The UnAcked operations should be zero, because we are NOT connected
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* Wait a little bit to make sure the client has had a chance to put the publishes out */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Confirm the UnAcked operations are now correct as well */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(5, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(5, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    /* Wait a little bit just to make sure the client has fully processed the shutdown */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Confirm the operation statistics are still correctly tracking post-disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(5, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(5, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_disconnect_publish,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_disconnect_publish,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Makes a publish offline, checks operation statistics, connects to non-ACK sending server, checks operation
 * statistics, makes another publish while online, checks operation statistics, disconnects, makes another publish, and
 * finally checks operation statistics one last time.
 */
static int s_test_mqtt_operation_statistics_reconnect_publish(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic_1");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Test Message");

    /* Stop ACKS so we make sure the operation statistics has time to allow us to identify we sent a packet */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* First publish! */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint64_t pub_packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_on_op_complete,
        state_test_data);
    ASSERT_TRUE(pub_packet_id_1 > 0);
    s_wait_for_ops_completed(state_test_data);

    /* Make sure the sizes are correct and there is five operations waiting
     * Each packet size = (The size of the topic, the size of the payload, 2 for the header, 2 for the packet ID) */
    uint64_t expected_packet_size = (pub_topic.len + payload.len + 4);
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(1, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    // The UnAcked operations should be zero, because we are NOT connected
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* Wait a second to give the MQTT311 client time to move the offline publish to unacked */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Confirm the UnAcked operations are now correct as well */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(1, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(1, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    /* Second publish! */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint64_t pub_packet_id_2 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_on_op_complete,
        state_test_data);
    ASSERT_TRUE(pub_packet_id_2 > 0);
    s_wait_for_ops_completed(state_test_data);

    /* Confirm both publishes are correct across all statistics */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(2, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size * 2, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(2, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size * 2, operation_statistics.unacked_operation_size);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    /* Third publish! */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint64_t pub_packet_id_3 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_on_op_complete,
        state_test_data);
    ASSERT_TRUE(pub_packet_id_3 > 0);
    s_wait_for_ops_completed(state_test_data);

    /* Confirm all three publishes are in the incomplete statistics, but only two are in unacked */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(3, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size * 3, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(2, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size * 2, operation_statistics.unacked_operation_size);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_reconnect_publish,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_reconnect_publish,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)

/* ========== SUBSCRIBE TESTS ========== */

/**
 * Make a connection, tell the server NOT to send Acks, subscribe and check the statistics to make sure
 * it is incomplete, then wait a little bit and check that it was properly marked as UnAcked, then send the SubAck
 * confirm statistics are zero, and then disconnect
 */
static int s_test_mqtt_operation_statistics_simple_subscribe(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor sub_topic = aws_byte_cursor_from_c_str("/test/topic");

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* We want to wait for 1 operation */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* Stop ACKS so we make sure the operation statistics has time to allow us to identify we sent a packet */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    // Send a subscribe packet
    uint16_t packet_id_1 = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_operation_statistics_on_publish_received,
        state_test_data,
        NULL,
        s_operation_statistics_on_suback,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    /* Wait a little bit to allow the code to put the packet into the socket from the queue, allowing it
     * to be unacked. If we check right away, we may or may not see it in the un-acked statistics */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Make sure the sizes are correct and there is only one operation waiting
     * (The size of the topic + 3 (QoS, MSB, LSB), 2 for the header, 2 for the packet ID) */
    uint64_t expected_packet_size = sub_topic.len + 7;
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(1, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(1, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    /* Send the SubAck and wait for the client to get the ACK */
    mqtt_mock_server_send_single_suback(state_test_data->mock_server, packet_id_1, AWS_MQTT_QOS_AT_LEAST_ONCE);
    s_wait_for_ops_completed(state_test_data);

    /* Make sure the operation statistics are empty */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_simple_subscribe,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_simple_subscribe,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)

/* ========== UNSUBSCRIBE TESTS ========== */

/**
 * Make a connection, tell the server NOT to send Acks, publish and immediately check the statistics to make sure
 * it is incomplete, then wait a little bit and check that it was properly marked as UnAcked, then send the UnsubAck
 * confirm statistics are zero, and then disconnect
 */
static int s_test_mqtt_operation_statistics_simple_unsubscribe(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor unsub_topic = aws_byte_cursor_from_c_str("/test/topic");

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* We want to wait for 1 operation */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* Stop ACKS so we make sure the operation statistics has time to allow us to identify we sent a packet */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* Send a subscribe packet */
    uint16_t packet_id_1 = aws_mqtt_client_connection_unsubscribe(
        state_test_data->mqtt_connection, &unsub_topic, s_on_op_complete, state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    /* Wait a little bit to allow the code to put the packet into the socket from the queue, allowing it
     * to be unacked. If we check right away, we may or may not see it in the un-acked statistics */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Make sure the sizes are correct and there is only one operation waiting
     * (The size of the topic, 2 for the header, 2 for the packet ID) */
    uint64_t expected_packet_size = unsub_topic.len + 4;
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(1, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(1, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    /* Send the UnsubAck and wait for the client to get the ACK */
    mqtt_mock_server_send_unsuback(state_test_data->mock_server, packet_id_1);
    s_wait_for_ops_completed(state_test_data);

    /* Make sure the operation statistics are empty */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_simple_unsubscribe,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_simple_unsubscribe,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)

/* ========== RESUBSCRIBE TESTS ========== */

/**
 * Subscribe to multiple topics prior to connection, make a CONNECT, unsubscribe to a topic, disconnect with the broker,
 * make a connection with clean_session set to true, then call resubscribe (without the server being able to send ACKs)
 * and confirm the operation statistics size is correct, then resubscribe and finally disconnect.
 */
static int s_test_mqtt_operation_statistics_simple_resubscribe(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor sub_topic_1 = aws_byte_cursor_from_c_str("/test/topic1");
    struct aws_byte_cursor sub_topic_2 = aws_byte_cursor_from_c_str("/test/topic2");
    struct aws_byte_cursor sub_topic_3 = aws_byte_cursor_from_c_str("/test/topic3");

    /* We want to wait for 3 operations */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 3;
    aws_mutex_unlock(&state_test_data->lock);

    /* Subscribe to the three topics */
    uint16_t sub_packet_id_1 = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic_1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_operation_statistics_on_publish_received,
        state_test_data,
        NULL,
        s_operation_statistics_on_suback,
        state_test_data);
    ASSERT_TRUE(sub_packet_id_1 > 0);
    uint16_t sub_packet_id_2 = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic_2,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_operation_statistics_on_publish_received,
        state_test_data,
        NULL,
        s_operation_statistics_on_suback,
        state_test_data);
    ASSERT_TRUE(sub_packet_id_2 > 0);
    uint16_t sub_packet_id_3 = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic_3,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_operation_statistics_on_publish_received,
        state_test_data,
        NULL,
        s_operation_statistics_on_suback,
        state_test_data);
    ASSERT_TRUE(sub_packet_id_3 > 0);

    /* Wait a little bit to allow the code to put the packet into the socket from the queue, allowing it
     * to be unacked. If we check right away, we may or may not see it in the un-acked statistics */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Confirm the 3 subscribes are both pending and unacked, and confirm their byte size
     * The size = each subscribe: 4 (fixed header + packet ID) + topic filter + 3 (QoS, MSB and LSB length) */
    uint64_t expected_packet_size = 12; // fixed packet headers and IDs
    expected_packet_size += sub_topic_1.len + 3;
    expected_packet_size += sub_topic_2.len + 3;
    expected_packet_size += sub_topic_3.len + 3;
    /* Check the size (Note: UnAcked will be ZERO because we are not connected) */
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(3, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* Wait for the subscribes to complete */
    s_wait_for_ops_completed(state_test_data);

    /* We want to wait for 1 operation */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    /* unsubscribe to the first topic */
    uint16_t unsub_packet_id = aws_mqtt_client_connection_unsubscribe(
        state_test_data->mqtt_connection, &sub_topic_1, s_on_op_complete, state_test_data);
    ASSERT_TRUE(unsub_packet_id > 0);
    s_wait_for_ops_completed(state_test_data);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);
    /* Note: The client is still subscribed to both topic_2 and topic_3 */

    /* Reconnect to the same server */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* Get all the packets out of the way */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    // Stop ACKs - we want to determine the size
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* Resubscribes to topic_2 & topic_3 (Note: we do not need a callback for the purpose of this test) */
    uint16_t resub_packet_id =
        aws_mqtt_resubscribe_existing_topics(state_test_data->mqtt_connection, NULL, state_test_data);
    ASSERT_TRUE(resub_packet_id > 0);

    /* Wait a little bit to allow the code to put the packet into the socket from the queue, allowing it
     * to be unacked. If we check right away, we may or may not see it in the un-acked statistics */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    // Make sure the resubscribe packet size is correct and there is only one resubscribe waiting
    // The size = 4 (fixed header + packet ID) + [for each topic](topic filter size + 3 (QoS, MSB and LSB length))
    expected_packet_size = 4;
    expected_packet_size += sub_topic_2.len + 3;
    expected_packet_size += sub_topic_3.len + 3;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(1, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(1, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    /* Send the resubscribe */
    mqtt_mock_server_send_single_suback(state_test_data->mock_server, resub_packet_id, AWS_MQTT_QOS_AT_LEAST_ONCE);
    s_operation_statistics_wait_for_subscribe_to_complete(state_test_data);

    /* Enable ACKs again, and then disconnect */
    mqtt_mock_server_enable_auto_ack(state_test_data->mock_server);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_simple_resubscribe,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_simple_resubscribe,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)

/* ========== OTHER TESTS ========== */

static void s_test_operation_statistics_simple_callback(
    struct aws_mqtt_client_connection_311_impl *connection,
    void *userdata) {
    struct aws_atomic_var *statistics_count = (struct aws_atomic_var *)userdata;
    aws_atomic_fetch_add(statistics_count, 1);

    // Confirm we can get the operation statistics from the callback
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    aws_mqtt_client_connection_get_stats(&connection->base, &operation_statistics);
    (void)operation_statistics;
}

/**
 * Tests the operation statistics callback to make sure it is being called as expected. This is a very simple
 * test that just ensures the callback is being called multiple times AND that you can access the operation
 * statistics from within the callback.
 */
static int s_test_mqtt_operation_statistics_simple_callback(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = s_operation_statistics_on_connection_complete_fn,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message");

    /* Connect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    s_operation_statistics_wait_for_connection_to_complete(state_test_data);

    /* Set the operation statistics callback */
    struct aws_atomic_var statistics_count;
    aws_atomic_store_int(&statistics_count, 0);
    aws_mqtt_client_connection_set_on_operation_statistics_handler(
        state_test_data->mqtt_connection->impl, s_test_operation_statistics_simple_callback, &statistics_count);

    // /* Stop ACKS so we make sure the operation statistics has time to allow us to identify we sent a packet */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    // /* We want to wait for 1 operation to complete */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* Publish a packet */
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        s_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    /* Wait a little bit to allow the code to put the packet into the socket from the queue, allowing it
     * to be unacked. If we check right away, we may or may not see it in the un-acked statistics */
    aws_thread_current_sleep((uint64_t)ONE_SEC);

    /* Make sure the sizes are correct and there is only one operation waiting
     * (The size of the topic, the size of the payload, 2 for the header, 2 for the packet ID) */
    uint64_t expected_packet_size = pub_topic.len + payload_1.len + 4;
    struct aws_mqtt_connection_operation_statistics operation_statistics;
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(1, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(1, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(expected_packet_size, operation_statistics.unacked_operation_size);

    /* Assert the callback was called twice (first for putting in incomplete, second for putting in unacked) */
    ASSERT_INT_EQUALS(2, aws_atomic_load_int(&statistics_count));

    /* Send the PubAck and wait for the client to receive it */
    mqtt_mock_server_send_puback(state_test_data->mock_server, packet_id_1);
    s_wait_for_ops_completed(state_test_data);

    // /* Assert the callback was called */
    aws_thread_current_sleep((uint64_t)ONE_SEC);
    ASSERT_INT_EQUALS(3, aws_atomic_load_int(&statistics_count));

    /* Make sure the operation values are back to zero now that the publish went out */
    ASSERT_SUCCESS(aws_mqtt_client_connection_get_stats(state_test_data->mqtt_connection, &operation_statistics));
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.incomplete_operation_size);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_count);
    ASSERT_INT_EQUALS(0, operation_statistics.unacked_operation_size);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, s_operation_statistics_on_disconnect_fn, state_test_data));
    s_operation_statistics_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_operation_statistics_simple_callback,
    s_operation_statistics_setup_mqtt_server_fn,
    s_test_mqtt_operation_statistics_simple_callback,
    s_operation_statistics_clean_up_mqtt_server_fn,
    &test_data)
