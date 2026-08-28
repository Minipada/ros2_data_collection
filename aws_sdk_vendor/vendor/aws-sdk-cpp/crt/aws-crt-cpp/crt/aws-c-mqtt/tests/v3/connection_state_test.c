/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/mqtt.h>

#include "mqtt311_testing_utils.h"
#include "mqtt_mock_server_handler.h"

#include <aws/common/clock.h>
#include <aws/mqtt/private/client_impl.h>
#include <aws/testing/aws_test_harness.h>

static struct mqtt_connection_state_test test_data = {0};

static int s_setup_mqtt_server_fn(struct aws_allocator *allocator, void *ctx) {
    return aws_test311_setup_mqtt_server_fn(allocator, ctx);
}

static int s_clean_up_mqtt_server_fn(struct aws_allocator *allocator, int setup_result, void *ctx) {
    return aws_test311_clean_up_mqtt_server_fn(allocator, setup_result, ctx);
}

/*
 * Makes an Mqtt connect call, then a disconnect. Then verifies a CONNECT and DISCONNECT were sent.
 */
static int s_test_mqtt_connect_disconnect_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(2, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_disconnect,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connect_disconnect_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/*
 * Makes an Mqtt connect call, and set will and login information for the connection. Validate the those information are
 * correctly included in the CONNECT package.
 */
static int s_test_mqtt_connect_set_will_login_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_byte_cursor will_payload = aws_byte_cursor_from_c_str("this is a will.");
    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("test_topic");
    struct aws_byte_cursor username = aws_byte_cursor_from_c_str("user name");
    struct aws_byte_cursor password = aws_byte_cursor_from_c_str("password");
    enum aws_mqtt_qos will_qos = AWS_MQTT_QOS_AT_LEAST_ONCE;

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_will(
        state_test_data->mqtt_connection, &topic, will_qos, true /*retain*/, &will_payload));

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_login(state_test_data->mqtt_connection, &username, &password));

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(2, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));

    /* CONNECT packet */
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));
    /* validate the received will */
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->will_message, &will_payload));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->will_topic, &topic));
    ASSERT_UINT_EQUALS(will_qos, received_packet->will_qos);
    ASSERT_TRUE(true == received_packet->will_retain);
    /* validate the received login information */
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->username, &username));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->password, &password));

    /* DISCONNECT packet */
    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    /* Connect to the mock server again. If set will&loggin message is not called before the next connect, the
     * will&loggin message will still be there and be sent to the server again */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    /* The second CONNECT packet */
    received_packet = mqtt_mock_server_get_latest_decoded_packet(state_test_data->mock_server);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));
    /* validate the received will */
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->will_message, &will_payload));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->will_topic, &topic));
    ASSERT_UINT_EQUALS(will_qos, received_packet->will_qos);
    ASSERT_TRUE(true == received_packet->will_retain);
    /* validate the received login information */
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->username, &username));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->password, &password));

    /* disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* set new will & loggin message, before next connect, the next CONNECT packet will contain the new information */
    struct aws_byte_cursor new_will_payload = aws_byte_cursor_from_c_str("this is a new will.");
    struct aws_byte_cursor new_topic = aws_byte_cursor_from_c_str("test_topic_New");
    struct aws_byte_cursor new_username = aws_byte_cursor_from_c_str("new user name");
    struct aws_byte_cursor new_password = aws_byte_cursor_from_c_str("new password");
    enum aws_mqtt_qos new_will_qos = AWS_MQTT_QOS_AT_MOST_ONCE;

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_will(
        state_test_data->mqtt_connection, &new_topic, new_will_qos, true /*retain*/, &new_will_payload));

    ASSERT_SUCCESS(
        aws_mqtt_client_connection_set_login(state_test_data->mqtt_connection, &new_username, &new_password));

    /* connect again */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    /* The third CONNECT packet */
    received_packet = mqtt_mock_server_get_latest_decoded_packet(state_test_data->mock_server);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));
    /* validate the received will */
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->will_message, &new_will_payload));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->will_topic, &new_topic));
    ASSERT_UINT_EQUALS(new_will_qos, received_packet->will_qos);
    ASSERT_TRUE(true == received_packet->will_retain);
    /* validate the received login information */
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->username, &new_username));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->password, &new_password));

    /* disconnect. FINISHED */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_set_will_login,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connect_set_will_login_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

#define MIN_RECONNECT_DELAY_SECONDS 5
#define MAX_RECONNECT_DELAY_SECONDS 120

/*
 * Makes a CONNECT, then the server hangs up, tests that the client reconnects on its own, then sends a DISCONNECT.
 * Also checks that the minimum reconnect time delay is honored.
 */
static int s_test_mqtt_connection_interrupted_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    aws_mqtt_client_connection_set_reconnect_timeout(
        state_test_data->mqtt_connection, MIN_RECONNECT_DELAY_SECONDS, MAX_RECONNECT_DELAY_SECONDS);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* shut it down and make sure the client automatically reconnects.*/
    uint64_t now = 0;
    aws_high_res_clock_get_ticks(&now);
    uint64_t start_shutdown = now;

    aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
    aws_test311_wait_for_reconnect_to_complete(state_test_data);

    aws_high_res_clock_get_ticks(&now);
    uint64_t reconnect_complete = now;

    uint64_t elapsed_time = reconnect_complete - start_shutdown;
    ASSERT_TRUE(
        aws_timestamp_convert(elapsed_time, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_SECS, NULL) >=
        MIN_RECONNECT_DELAY_SECONDS);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(3, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_interrupted,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_interrupted_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Makes a CONNECT, with a 1 second keep alive ping interval, the mock is configured to not reply to the PING,
 * this should cause a timeout, then the PING responses are turned back on, and the client should automatically
 * reconnect. Then send a DISCONNECT. */
static int s_test_mqtt_connection_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
    };

    mqtt_mock_server_set_max_ping_resp(state_test_data->mock_server, 0);
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* this should take about 1.1 seconds for the timeout and reconnect.*/
    aws_test311_wait_for_reconnect_to_complete(state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    ASSERT_INT_EQUALS(AWS_ERROR_MQTT_TIMEOUT, state_test_data->interruption_error);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_timeout,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_timeout_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Test set on_any_publish handler. User can set on_any_publish handler to be called whenever any publish packet is
 * received */
static int s_test_mqtt_connection_any_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor topic_1 = aws_byte_cursor_from_c_str("/test/topic1");
    struct aws_byte_cursor topic_2 = aws_byte_cursor_from_c_str("/test/topic2");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* NOTE: mock server sends to client with no subscription at all, which should not happen in the real world! */
    state_test_data->expected_any_publishes = 2;
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &topic_1,
        &payload_1,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &topic_2,
        &payload_2,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));

    aws_test311_wait_for_any_publish(state_test_data);
    mqtt_mock_server_wait_for_pubacks(state_test_data->mock_server, 2);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    /* CONNECT two PUBACK DISCONNECT */
    ASSERT_UINT_EQUALS(4, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 3);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    /* Check the received publish packet from the client side */
    ASSERT_UINT_EQUALS(2, aws_array_list_length(&state_test_data->any_published_messages));
    struct received_publish_packet *publish_msg = NULL;
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->any_published_messages, (void **)&publish_msg, 0));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&topic_1, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_1, &publish_msg->payload));
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->any_published_messages, (void **)&publish_msg, 1));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&topic_2, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_2, &publish_msg->payload));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_any_publish,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_any_publish_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Makes a CONNECT, channel is successfully setup, but the server never sends a connack, make sure we timeout. */
static int s_test_mqtt_connection_connack_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
    };

    mqtt_mock_server_set_max_connack(state_test_data->mock_server, 0);
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    ASSERT_INT_EQUALS(AWS_ERROR_MQTT_TIMEOUT, state_test_data->error);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_connack_timeout,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_connack_timeout_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Use the connack timeout to test the connection failure callback */
static int s_test_mqtt_connection_failure_callback_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
    };

    mqtt_mock_server_set_max_connack(state_test_data->mock_server, 0);
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_fail(state_test_data);

    ASSERT_INT_EQUALS(AWS_ERROR_MQTT_TIMEOUT, state_test_data->error);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_failure_callback,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_failure_callback_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Quick test the connection succeed callback */
static int s_test_mqtt_connection_success_callback_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_succeed(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(1, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    // Disconnect and finish
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_success_callback,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_success_callback_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Subscribe to a topic prior to connection, make a CONNECT, have the server send PUBLISH messages,
 * make sure they're received, then send a DISCONNECT. */
static int s_test_mqtt_subscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor sub_topic = aws_byte_cursor_from_c_str("/test/topic");

    uint16_t packet_id = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        aws_test311_on_publish_received,
        state_test_data,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(packet_id > 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_test311_wait_for_subscribe_to_complete(state_test_data);

    state_test_data->expected_publishes = 2;
    state_test_data->expected_any_publishes = 2;
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic,
        &payload_1,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        true /*retain*/));
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic,
        &payload_2,
        true /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));

    aws_test311_wait_for_publish(state_test_data);
    aws_test311_wait_for_any_publish(state_test_data);
    mqtt_mock_server_wait_for_pubacks(state_test_data->mock_server, 2);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(5, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_SUBSCRIBE, received_packet->type);
    ASSERT_UINT_EQUALS(1, aws_array_list_length(&received_packet->sub_topic_filters));
    struct aws_mqtt_subscription val;
    ASSERT_SUCCESS(aws_array_list_front(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &sub_topic));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_UINT_EQUALS(packet_id, received_packet->packet_identifier);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 3);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 4);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    /* Check PUBLISH packets received via subscription callback */
    ASSERT_UINT_EQUALS(2, aws_array_list_length(&state_test_data->published_messages));

    struct received_publish_packet *publish_msg = NULL;
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, 0));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_1, &publish_msg->payload));
    ASSERT_FALSE(publish_msg->dup);
    ASSERT_TRUE(publish_msg->retain);

    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, 1));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_2, &publish_msg->payload));
    ASSERT_TRUE(publish_msg->dup);
    ASSERT_FALSE(publish_msg->retain);

    /* Check PUBLISH packets received via on_any_publish callback */
    ASSERT_UINT_EQUALS(2, aws_array_list_length(&state_test_data->any_published_messages));

    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->any_published_messages, (void **)&publish_msg, 0));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_1, &publish_msg->payload));
    ASSERT_FALSE(publish_msg->dup);
    ASSERT_TRUE(publish_msg->retain);

    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->any_published_messages, (void **)&publish_msg, 1));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_2, &publish_msg->payload));
    ASSERT_TRUE(publish_msg->dup);
    ASSERT_FALSE(publish_msg->retain);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_subscribe,
    s_setup_mqtt_server_fn,
    s_test_mqtt_subscribe_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_test_mqtt_subscribe_incoming_dup_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor subscribed_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor any_topic = aws_byte_cursor_from_c_str("/a/b/c");

    uint16_t packet_id = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &subscribed_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        aws_test311_on_publish_received,
        state_test_data,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(packet_id > 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    aws_test311_wait_for_subscribe_to_complete(state_test_data);

    state_test_data->expected_publishes = 4;
    state_test_data->expected_any_publishes = 8;

    struct aws_byte_cursor subscribed_payload = aws_byte_cursor_from_c_str("Subscribed");
    for (size_t i = 0; i < 4; ++i) {
        ASSERT_SUCCESS(mqtt_mock_server_send_publish_by_id(
            state_test_data->mock_server,
            1111,
            &subscribed_topic,
            &subscribed_payload,
            i > 0 /*dup*/,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            true /*retain*/));
    }

    struct aws_byte_cursor any_payload = aws_byte_cursor_from_c_str("Not subscribed.  On-any only.");
    for (size_t i = 0; i < 4; ++i) {
        ASSERT_SUCCESS(mqtt_mock_server_send_publish_by_id(
            state_test_data->mock_server,
            1234,
            &any_topic,
            &any_payload,
            i > 0 /*dup*/,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false /*retain*/));
    }

    aws_test311_wait_for_publish(state_test_data);
    aws_test311_wait_for_any_publish(state_test_data);
    mqtt_mock_server_wait_for_pubacks(state_test_data->mock_server, 8);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(11, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_SUBSCRIBE, received_packet->type);
    ASSERT_UINT_EQUALS(1, aws_array_list_length(&received_packet->sub_topic_filters));
    struct aws_mqtt_subscription val;
    ASSERT_SUCCESS(aws_array_list_front(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &subscribed_topic));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_UINT_EQUALS(packet_id, received_packet->packet_identifier);

    for (size_t i = 0; i < 8; ++i) {
        received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2 + i);
        ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);
    }

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 10);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    /* Check PUBLISH packets received via subscription callback */
    ASSERT_UINT_EQUALS(4, aws_array_list_length(&state_test_data->published_messages));

    for (size_t i = 0; i < 4; ++i) {
        struct received_publish_packet *publish_msg = NULL;
        ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, i));
        ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&subscribed_topic, &publish_msg->topic));
        ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&subscribed_payload, &publish_msg->payload));
        ASSERT_INT_EQUALS((i != 0) ? 1 : 0, publish_msg->dup ? 1 : 0);
        ASSERT_TRUE(publish_msg->retain);
    }

    /* Check PUBLISH packets received via on_any_publish callback */
    ASSERT_UINT_EQUALS(8, aws_array_list_length(&state_test_data->any_published_messages));

    for (size_t i = 0; i < 4; ++i) {
        struct received_publish_packet *publish_msg = NULL;
        ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->any_published_messages, (void **)&publish_msg, i));
        ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&subscribed_topic, &publish_msg->topic));
        ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&subscribed_payload, &publish_msg->payload));
        ASSERT_INT_EQUALS((i > 0) ? 1 : 0, publish_msg->dup ? 1 : 0);
        ASSERT_TRUE(publish_msg->retain);
    }

    for (size_t i = 4; i < 8; ++i) {
        struct received_publish_packet *publish_msg = NULL;
        ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->any_published_messages, (void **)&publish_msg, i));
        ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&any_topic, &publish_msg->topic));
        ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&any_payload, &publish_msg->payload));
        ASSERT_INT_EQUALS((i > 4) ? 1 : 0, publish_msg->dup ? 1 : 0);
        ASSERT_FALSE(publish_msg->retain);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_subscribe_incoming_dup,
    s_setup_mqtt_server_fn,
    s_test_mqtt_subscribe_incoming_dup_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Subscribe to a topic and broker returns a SUBACK with failure return code, the subscribe should fail */
static int s_test_mqtt_connect_subscribe_fail_from_broker_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, we will send failure SUBACK */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    struct aws_byte_cursor sub_topic = aws_byte_cursor_from_c_str("/test/topic");

    uint16_t packet_id = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        aws_test311_on_publish_received,
        state_test_data,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(packet_id > 0);

    ASSERT_SUCCESS(mqtt_mock_server_send_single_suback(state_test_data->mock_server, packet_id, AWS_MQTT_QOS_FAILURE));
    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    /* Check the subscribe returned QoS is failure */
    size_t length = aws_array_list_length(&state_test_data->qos_returned);
    ASSERT_UINT_EQUALS(1, length);
    uint8_t qos = 0;
    ASSERT_SUCCESS(aws_array_list_get_at(&state_test_data->qos_returned, &qos, 0));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_FAILURE, qos);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_subscribe_fail_from_broker,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connect_subscribe_fail_from_broker_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Subscribe to multiple topics prior to connection, make a CONNECT, have the server send PUBLISH messages,
 * make sure they're received, then send a DISCONNECT. */
static int s_test_mqtt_subscribe_multi_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor sub_topic_1 = aws_byte_cursor_from_c_str("/test/topic1");
    struct aws_byte_cursor sub_topic_2 = aws_byte_cursor_from_c_str("/test/topic2");

    struct aws_mqtt_topic_subscription sub1 = {
        .topic = sub_topic_1,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };
    struct aws_mqtt_topic_subscription sub2 = {
        .topic = sub_topic_2,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };

    struct aws_array_list topic_filters;
    size_t list_len = 2;
    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, static_buf, list_len * sizeof(struct aws_mqtt_topic_subscription));
    aws_array_list_init_static(&topic_filters, static_buf, list_len, sizeof(struct aws_mqtt_topic_subscription));

    aws_array_list_push_back(&topic_filters, &sub1);
    aws_array_list_push_back(&topic_filters, &sub2);

    uint16_t packet_id = aws_mqtt_client_connection_subscribe_multiple(
        state_test_data->mqtt_connection, &topic_filters, aws_test311_on_multi_suback, state_test_data);
    ASSERT_TRUE(packet_id > 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    /* Check the subscribe returned QoS is expected */
    size_t length = aws_array_list_length(&state_test_data->qos_returned);
    ASSERT_UINT_EQUALS(2, length);
    uint8_t qos = 0;
    ASSERT_SUCCESS(aws_array_list_get_at(&state_test_data->qos_returned, &qos, 0));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_EXACTLY_ONCE, qos);
    ASSERT_SUCCESS(aws_array_list_get_at(&state_test_data->qos_returned, &qos, 1));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_EXACTLY_ONCE, qos);

    state_test_data->expected_publishes = 2;
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic_1,
        &payload_1,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic_2,
        &payload_2,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    aws_test311_wait_for_publish(state_test_data);

    /* Let's do another publish on a topic that is not subscribed by client.
     * This can happen if the Server automatically assigned a subscription to the Client */
    state_test_data->expected_any_publishes = 3;
    struct aws_byte_cursor payload_3 = aws_byte_cursor_from_c_str("Test Message 3");
    struct aws_byte_cursor topic_3 = aws_byte_cursor_from_c_str("/test/topic3");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &topic_3,
        &payload_3,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    aws_test311_wait_for_any_publish(state_test_data);

    mqtt_mock_server_wait_for_pubacks(state_test_data->mock_server, 3);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(6, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_SUBSCRIBE, received_packet->type);
    ASSERT_UINT_EQUALS(2, aws_array_list_length(&received_packet->sub_topic_filters));
    struct aws_mqtt_subscription val;
    ASSERT_SUCCESS(aws_array_list_front(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &sub_topic_1));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_SUCCESS(aws_array_list_back(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &sub_topic_2));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_UINT_EQUALS(packet_id, received_packet->packet_identifier);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);
    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 3);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);
    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 4);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 5);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    /* Only two packets should be recorded by the published_messages, but all the three packets will be recorded by
     * any_published_messages */
    ASSERT_UINT_EQUALS(2, aws_array_list_length(&state_test_data->published_messages));
    ASSERT_UINT_EQUALS(3, aws_array_list_length(&state_test_data->any_published_messages));

    struct received_publish_packet *publish_msg = NULL;
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, 0));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic_1, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_1, &publish_msg->payload));
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, 1));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic_2, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_2, &publish_msg->payload));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_subscribe_multi,
    s_setup_mqtt_server_fn,
    s_test_mqtt_subscribe_multi_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Subscribe to multiple topics prior to connection, make a CONNECT, have the server send PUBLISH messages, unsubscribe
 * to a topic, have the server send PUBLISH messages again, make sure the unsubscribed topic callback will not be fired
 */
static int s_test_mqtt_unsubscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor sub_topic_1 = aws_byte_cursor_from_c_str("/test/topic1");
    struct aws_byte_cursor sub_topic_2 = aws_byte_cursor_from_c_str("/test/topic2");

    struct aws_mqtt_topic_subscription sub1 = {
        .topic = sub_topic_1,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };
    struct aws_mqtt_topic_subscription sub2 = {
        .topic = sub_topic_2,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };

    struct aws_array_list topic_filters;
    size_t list_len = 2;
    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, static_buf, list_len * sizeof(struct aws_mqtt_topic_subscription));
    aws_array_list_init_static(&topic_filters, static_buf, list_len, sizeof(struct aws_mqtt_topic_subscription));

    aws_array_list_push_back(&topic_filters, &sub1);
    aws_array_list_push_back(&topic_filters, &sub2);

    uint16_t sub_packet_id = aws_mqtt_client_connection_subscribe_multiple(
        state_test_data->mqtt_connection, &topic_filters, aws_test311_on_multi_suback, state_test_data);
    ASSERT_TRUE(sub_packet_id > 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_test311_wait_for_subscribe_to_complete(state_test_data);

    state_test_data->expected_any_publishes = 2;
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic_1,
        &payload_1,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic_2,
        &payload_2,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    aws_test311_wait_for_any_publish(state_test_data);
    mqtt_mock_server_wait_for_pubacks(state_test_data->mock_server, 2);

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    /* unsubscribe to the first topic */
    uint16_t unsub_packet_id = aws_mqtt_client_connection_unsubscribe(
        state_test_data->mqtt_connection, &sub_topic_1, aws_test311_on_op_complete, state_test_data);
    ASSERT_TRUE(unsub_packet_id > 0);
    /* Even when the UNSUBACK has not received, the client will not invoke the on_pub callback for that topic */
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic_1,
        &payload_1,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    ASSERT_SUCCESS(mqtt_mock_server_send_publish(
        state_test_data->mock_server,
        &sub_topic_2,
        &payload_2,
        false /*dup*/,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false /*retain*/));
    state_test_data->expected_any_publishes = 2;
    aws_test311_wait_for_any_publish(state_test_data);
    mqtt_mock_server_wait_for_pubacks(state_test_data->mock_server, 2);
    aws_test311_wait_for_ops_completed(state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_SUBSCRIBE, received_packet->type);
    ASSERT_UINT_EQUALS(2, aws_array_list_length(&received_packet->sub_topic_filters));
    struct aws_mqtt_subscription val;
    ASSERT_SUCCESS(aws_array_list_front(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &sub_topic_1));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_SUCCESS(aws_array_list_back(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &sub_topic_2));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_UINT_EQUALS(sub_packet_id, received_packet->packet_identifier);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);
    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 3);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 4);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_UNSUBSCRIBE, received_packet->type);
    ASSERT_UINT_EQUALS(1, aws_array_list_length(&received_packet->unsub_topic_filters));
    struct aws_byte_cursor val_cur;
    ASSERT_SUCCESS(aws_array_list_front(&received_packet->unsub_topic_filters, &val_cur));
    ASSERT_TRUE(aws_byte_cursor_eq(&val_cur, &sub_topic_1));
    ASSERT_UINT_EQUALS(unsub_packet_id, received_packet->packet_identifier);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 5);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);
    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 6);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBACK, received_packet->type);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 7);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    /* Only three packets should be recorded by the published_messages, but all the four packets will be recorded by
     * any_published_messages */
    ASSERT_UINT_EQUALS(3, aws_array_list_length(&state_test_data->published_messages));
    ASSERT_UINT_EQUALS(4, aws_array_list_length(&state_test_data->any_published_messages));

    struct received_publish_packet *publish_msg = NULL;
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, 0));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic_1, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_1, &publish_msg->payload));
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, 1));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic_2, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_2, &publish_msg->payload));
    ASSERT_SUCCESS(aws_array_list_get_at_ptr(&state_test_data->published_messages, (void **)&publish_msg, 2));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&sub_topic_2, &publish_msg->topic));
    ASSERT_TRUE(aws_byte_cursor_eq_byte_buf(&payload_2, &publish_msg->payload));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_unsubscribe,
    s_setup_mqtt_server_fn,
    s_test_mqtt_unsubscribe_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Subscribe to multiple topics prior to connection, make a CONNECT, have the server send PUBLISH messages, unsubscribe
 * to a topic, disconnect with the broker, make a connection with clean_session true, then call resubscribe, client will
 * successfully resubscribe to the old topics.
 */
static int s_test_mqtt_resubscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor sub_topic_1 = aws_byte_cursor_from_c_str("/test/topic1");
    struct aws_byte_cursor sub_topic_2 = aws_byte_cursor_from_c_str("/test/topic2");
    struct aws_byte_cursor sub_topic_3 = aws_byte_cursor_from_c_str("/test/topic3");

    struct aws_mqtt_topic_subscription sub1 = {
        .topic = sub_topic_1,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };
    struct aws_mqtt_topic_subscription sub2 = {
        .topic = sub_topic_2,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };
    struct aws_mqtt_topic_subscription sub3 = {
        .topic = sub_topic_3,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };

    struct aws_array_list topic_filters;
    size_t list_len = 3;
    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, static_buf, list_len * sizeof(struct aws_mqtt_topic_subscription));
    aws_array_list_init_static(&topic_filters, static_buf, list_len, sizeof(struct aws_mqtt_topic_subscription));

    aws_array_list_push_back(&topic_filters, &sub1);
    aws_array_list_push_back(&topic_filters, &sub2);
    aws_array_list_push_back(&topic_filters, &sub3);

    uint16_t sub_packet_id = aws_mqtt_client_connection_subscribe_multiple(
        state_test_data->mqtt_connection, &topic_filters, aws_test311_on_multi_suback, state_test_data);
    ASSERT_TRUE(sub_packet_id > 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    /* unsubscribe to the first topic */
    uint16_t unsub_packet_id = aws_mqtt_client_connection_unsubscribe(
        state_test_data->mqtt_connection, &sub_topic_1, aws_test311_on_op_complete, state_test_data);
    ASSERT_TRUE(unsub_packet_id > 0);

    aws_test311_wait_for_ops_completed(state_test_data);
    /* client still subscribes to topic_2 & topic_3 */

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* reconnection to the same server */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Get all the packets out of the way */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));
    size_t packets_count = mqtt_mock_server_decoded_packets_count(state_test_data->mock_server);
    struct mqtt_decoded_packet *t_received_packet =
        mqtt_mock_server_get_latest_decoded_packet(state_test_data->mock_server);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, t_received_packet->type);

    /* resubscribes to topic_2 & topic_3 */
    uint16_t resub_packet_id = aws_mqtt_resubscribe_existing_topics(
        state_test_data->mqtt_connection, aws_test311_on_multi_suback, state_test_data);
    ASSERT_TRUE(resub_packet_id > 0);
    aws_test311_wait_for_subscribe_to_complete(state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, packets_count);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_SUBSCRIBE, received_packet->type);

    ASSERT_UINT_EQUALS(2, aws_array_list_length(&received_packet->sub_topic_filters));
    struct aws_mqtt_subscription val;
    ASSERT_SUCCESS(aws_array_list_front(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &sub_topic_3));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_SUCCESS(aws_array_list_back(&received_packet->sub_topic_filters, &val));
    ASSERT_TRUE(aws_byte_cursor_eq(&val.topic_filter, &sub_topic_2));
    ASSERT_UINT_EQUALS(AWS_MQTT_QOS_AT_LEAST_ONCE, val.qos);
    ASSERT_UINT_EQUALS(resub_packet_id, received_packet->packet_identifier);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_resubscribe,
    s_setup_mqtt_server_fn,
    s_test_mqtt_resubscribe_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Make a CONNECT, PUBLISH to a topic, make sure server received, then send a DISCONNECT. */
static int s_test_mqtt_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 3;
    aws_mutex_unlock(&state_test_data->lock);
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);
    uint16_t packet_id_2 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_2,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_2 > 0);

    /* Null payload case */
    uint16_t packet_id_3 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        NULL,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_3 > 0);

    aws_test311_wait_for_ops_completed(state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(5, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBLISH, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->topic_name, &pub_topic));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->publish_payload, &payload_1));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBLISH, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->topic_name, &pub_topic));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->publish_payload, &payload_2));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 3);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBLISH, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->topic_name, &pub_topic));
    ASSERT_INT_EQUALS(0, received_packet->publish_payload.len);

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 4);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_publish,
    s_setup_mqtt_server_fn,
    s_test_mqtt_publish_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Make a CONNECT, PUBLISH to a topic, free the payload before the publish completes to make sure it's safe */
static int s_test_mqtt_publish_payload_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_buf buf_payload;
    struct aws_byte_cursor ori_payload = aws_byte_cursor_from_c_str("Test Message 1");
    ASSERT_SUCCESS(aws_byte_buf_init_copy_from_cursor(&buf_payload, allocator, ori_payload));
    struct aws_byte_cursor payload_curser = aws_byte_cursor_from_buf(&buf_payload);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint16_t packet_id = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_curser,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id > 0);

    /* clean up the payload buf, as user don't need to manage the buf and keep it valid until publish completes */
    aws_byte_buf_clean_up(&buf_payload);
    aws_test311_wait_for_ops_completed(state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));

    ASSERT_UINT_EQUALS(3, mqtt_mock_server_decoded_packets_count(state_test_data->mock_server));
    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBLISH, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->topic_name, &pub_topic));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->publish_payload, &ori_payload));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 2);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connect_publish_payload,
    s_setup_mqtt_server_fn,
    s_test_mqtt_publish_payload_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * CONNECT, force the server to hang up after a successful connection and block all CONNACKS, send PUBLISH messages
 * let the server send CONNACKS, make sure when the client reconnects automatically, it sends the PUBLISH messages
 * that were sent during offline mode. Then send a DISCONNECT.
 */
static int s_test_mqtt_connection_offline_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    mqtt_mock_server_set_max_connack(state_test_data->mock_server, 0);

    /* shut it down and make sure the client automatically reconnects.*/
    aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
    aws_test311_wait_for_interrupt_to_complete(state_test_data);

    state_test_data->server_disconnect_completed = false;

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 2;
    aws_mutex_unlock(&state_test_data->lock);

    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload_1,
            aws_test311_on_op_complete,
            state_test_data) > 0);
    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload_2,
            aws_test311_on_op_complete,
            state_test_data) > 0);

    aws_mutex_lock(&state_test_data->lock);
    ASSERT_FALSE(state_test_data->connection_resumed);
    aws_mutex_unlock(&state_test_data->lock);
    mqtt_mock_server_set_max_connack(state_test_data->mock_server, SIZE_MAX);
    aws_test311_wait_for_ops_completed(state_test_data);
    aws_mutex_lock(&state_test_data->lock);
    ASSERT_TRUE(state_test_data->connection_resumed);
    aws_mutex_unlock(&state_test_data->lock);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Decode all received packets by mock server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));
    size_t packets_count = mqtt_mock_server_decoded_packets_count(state_test_data->mock_server);
    ASSERT_TRUE(packets_count >= 5 && packets_count <= 6);

    struct mqtt_decoded_packet *received_packet =
        mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 0);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, 1);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
    ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));

    /* if message count is 6 there was an extra connect message due to the automatic reconnect behavior and timing. */
    size_t index = 2;
    if (packets_count == 6) {
        received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, index++);
        ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_CONNECT, received_packet->type);
        ASSERT_UINT_EQUALS(connection_options.clean_session, received_packet->clean_session);
        ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->client_identifier, &connection_options.client_id));
    }

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, index++);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBLISH, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->topic_name, &pub_topic));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->publish_payload, &payload_1));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, index++);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_PUBLISH, received_packet->type);
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->topic_name, &pub_topic));
    ASSERT_TRUE(aws_byte_cursor_eq(&received_packet->publish_payload, &payload_2));

    received_packet = mqtt_mock_server_get_decoded_packet_by_index(state_test_data->mock_server, index++);
    ASSERT_UINT_EQUALS(AWS_MQTT_PACKET_DISCONNECT, received_packet->type);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_offline_publish,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_offline_publish_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * CONNECT, force the server to hang up after a successful connection and block all CONNACKS, DISCONNECT while client is
 * reconnecting. Resource and pending requests are cleaned up correctly
 */
static int s_test_mqtt_connection_disconnect_while_reconnecting(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    mqtt_mock_server_set_max_connack(state_test_data->mock_server, 0);

    /* shut it down and the client automatically reconnects.*/
    aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
    aws_test311_wait_for_interrupt_to_complete(state_test_data);

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 2;
    aws_mutex_unlock(&state_test_data->lock);

    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload_1,
            aws_test311_on_op_complete,
            state_test_data) > 0);
    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload_2,
            aws_test311_on_op_complete,
            state_test_data) > 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);
    aws_mqtt_client_connection_release(state_test_data->mqtt_connection);
    state_test_data->mqtt_connection = NULL;
    aws_test311_wait_for_ops_completed(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_disconnect_while_reconnecting,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_disconnect_while_reconnecting,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Regression test: Once upon a time there was a bug caused by race condition on the state of connection.
 * The scenario is the server/broker closes the connection, while the client is making requests.
 * The race condition between the eventloop thread closes the connection and the main thread makes request could cause a
 * bug.
 * Solution: put a lock for the state of connection, protect it from accessing by multiple threads at the same time.
 */

static int s_test_mqtt_connection_closes_while_making_requests_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* shutdown the channel for some error */
    aws_channel_shutdown(state_test_data->server_channel, AWS_ERROR_INVALID_STATE);

    /* While the shutdown is still in process, making a publish request */
    /* It may not 100% trigger the crash, the crash only happens when the s_mqtt_client_shutdown and mqtt_create_request
     * happen at the same time. Like the slot is removed by shutdown, and the create request still think the slot is
     * there and try to access it. Crash happens. It's not possible to trigger the crash 100% without changing the
     * implementation. */
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    aws_test311_wait_for_reconnect_to_complete(state_test_data);
    aws_test311_wait_for_ops_completed(state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_closes_while_making_requests,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_closes_while_making_requests_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* helper to make sure packets are received/resent in expected order and duplicat flag is appropriately set */
static int s_check_resend_packets(
    struct aws_channel_handler *handler,
    size_t search_start_idx,
    bool duplicate_publish_expected,
    uint16_t *packet_ids,
    size_t packet_id_count) {
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(handler));

    if (packet_id_count == 0) {
        return AWS_OP_SUCCESS;
    }

    size_t previous_index = 0;
    struct mqtt_decoded_packet *previous_packet =
        mqtt_mock_server_find_decoded_packet_by_id(handler, search_start_idx, packet_ids[0], &previous_index);
    if (previous_packet->type == AWS_MQTT_PACKET_PUBLISH) {
        ASSERT_INT_EQUALS(duplicate_publish_expected, previous_packet->duplicate);
    }

    for (size_t i = 1; i < packet_id_count; ++i) {
        size_t current_index = 0;
        struct mqtt_decoded_packet *current_packet =
            mqtt_mock_server_find_decoded_packet_by_id(handler, search_start_idx, packet_ids[i], &current_index);
        if (current_packet->type == AWS_MQTT_PACKET_PUBLISH) {
            ASSERT_INT_EQUALS(duplicate_publish_expected, current_packet->duplicate);
        }

        ASSERT_TRUE(current_index > previous_index);
        previous_packet = current_packet;
        previous_index = current_index;
    }

    return AWS_OP_SUCCESS;
}

/**
 * Test that when the response is not back from the server, and the connection lost, the sent packets will be retried
 * in the same order as they are sent before.
 */
static int s_test_mqtt_connection_resend_packets_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
    };

    struct aws_byte_cursor sub_topic = aws_byte_cursor_from_c_str("/test/topic/sub/#");
    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    struct aws_byte_cursor payload_2 = aws_byte_cursor_from_c_str("Test Message 2");
    struct aws_byte_cursor payload_3 = aws_byte_cursor_from_c_str("Test Message 3");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    uint16_t packet_ids[5];

    packet_ids[0] = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection, &pub_topic, AWS_MQTT_QOS_AT_LEAST_ONCE, false, &payload_1, NULL, NULL);
    ASSERT_TRUE(packet_ids[0] > 0);

    packet_ids[1] = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        aws_test311_on_publish_received,
        state_test_data,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(packet_ids[1] > 0);

    packet_ids[2] = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection, &pub_topic, AWS_MQTT_QOS_AT_LEAST_ONCE, false, &payload_2, NULL, NULL);
    ASSERT_TRUE(packet_ids[2] > 0);

    packet_ids[3] = aws_mqtt_client_connection_unsubscribe(state_test_data->mqtt_connection, &sub_topic, NULL, NULL);
    ASSERT_TRUE(packet_ids[3] > 0);

    packet_ids[4] = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection, &pub_topic, AWS_MQTT_QOS_AT_LEAST_ONCE, false, &payload_3, NULL, NULL);
    ASSERT_TRUE(packet_ids[4] > 0);

    /* Wait for 1 sec. ensure all the publishes have been received by the server */
    aws_thread_current_sleep(ONE_SEC);
    ASSERT_SUCCESS(
        s_check_resend_packets(state_test_data->mock_server, 0, false, packet_ids, AWS_ARRAY_SIZE(packet_ids)));

    size_t packet_count = mqtt_mock_server_decoded_packets_count(state_test_data->mock_server);

    /* shutdown the channel for some error */
    aws_channel_shutdown(state_test_data->server_channel, AWS_ERROR_INVALID_STATE);
    aws_test311_wait_for_reconnect_to_complete(state_test_data);
    /* Wait again, and ensure the publishes have been resent */
    aws_thread_current_sleep(ONE_SEC);
    ASSERT_SUCCESS(s_check_resend_packets(
        state_test_data->mock_server, packet_count, true, packet_ids, AWS_ARRAY_SIZE(packet_ids)));

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_resend_packets,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_resend_packets_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection lost before the publish QoS 0 ever sent, publish QoS 0 will not be retried.
 */
static int s_test_mqtt_connection_not_retry_publish_QoS_0_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* kill the connection */
    aws_channel_shutdown(state_test_data->server_channel, AWS_ERROR_INVALID_STATE);
    /* TODO: only one eventloop thread in the el group. Most likely the test will fail, the outgoing task will not be
     * cancelled because of connection lost */
    /* make a publish with QoS 0 immediate. */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    /* publish should complete after the shutdown */
    aws_test311_wait_for_ops_completed(state_test_data);
    /* wait for reconnect */
    aws_test311_wait_for_reconnect_to_complete(state_test_data);

    /* Check all received packets, no publish packets ever received by the server. Because the connection lost before it
     * ever get sent. */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));
    ASSERT_NULL(
        mqtt_mock_server_find_decoded_packet_by_type(state_test_data->mock_server, 0, AWS_MQTT_PACKET_PUBLISH, NULL));
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_not_retry_publish_QoS_0,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_not_retry_publish_QoS_0_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that the retry policy is consistent, which means either the request has been sent or not, when the connection
 * lost/resume, the request will be retried.
 */
static int s_test_mqtt_connection_consistent_retry_policy_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    struct aws_byte_cursor sub_topic = aws_byte_cursor_from_c_str("/test/topic");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    struct aws_channel_handler *handler = state_test_data->mock_server;

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(handler);

    /* kill the connection */
    aws_channel_shutdown(state_test_data->server_channel, AWS_ERROR_INVALID_STATE);
    /* There is a hidden race condition between channel shutdown from eventloop and publish/subscribe from main thread,
     * but either way, it should work as they are retried in the end.  */
    /* make a publish with QoS 1 immediate. */
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);
    /* make another subscribe */
    uint16_t packet_id_2 = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        NULL,
        NULL,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(packet_id_2 > 0);

    /* wait for reconnect */
    aws_test311_wait_for_reconnect_to_complete(state_test_data);
    /* Wait for 1 sec. ensure all the requests have been received by the server */
    aws_thread_current_sleep(ONE_SEC);

    /* Check all received packets, subscribe and publish has been received */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(handler));
    size_t packet_count = mqtt_mock_server_decoded_packets_count(handler);
    /* the latest packet should be subscribe */
    ASSERT_NOT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_SUBSCRIBE, NULL));
    ASSERT_NOT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_PUBLISH, NULL));

    /* Re-enable the auto ack to finish the requests */
    mqtt_mock_server_enable_auto_ack(handler);
    /* Kill the connection again, the requests will be retried since the response has not been received yet. */
    aws_channel_shutdown(state_test_data->server_channel, AWS_ERROR_INVALID_STATE);
    aws_test311_wait_for_reconnect_to_complete(state_test_data);
    /* subscribe should be able to completed now */
    aws_test311_wait_for_subscribe_to_complete(state_test_data);

    /* Check all received packets, subscribe and publish has been resent */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(handler));
    ASSERT_TRUE(mqtt_mock_server_decoded_packets_count(handler) > packet_count);
    /* the latest packet should be subscribe */
    ASSERT_NOT_NULL(
        mqtt_mock_server_find_decoded_packet_by_type(handler, packet_count, AWS_MQTT_PACKET_SUBSCRIBE, NULL));
    ASSERT_NOT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, packet_count, AWS_MQTT_PACKET_PUBLISH, NULL));

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_consistent_retry_policy,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_consistent_retry_policy_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that the request will not be retried even the response has not received for a while.
 */
static int s_test_mqtt_connection_not_resend_packets_on_healthy_connection_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    struct aws_byte_cursor sub_topic = aws_byte_cursor_from_c_str("/test/topic");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    struct aws_channel_handler *handler = state_test_data->mock_server;

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(handler);

    /* make a publish with QoS 1 */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);
    /* make another subscribe */
    uint16_t packet_id_2 = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &sub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        NULL,
        NULL,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(packet_id_2 > 0);

    /* Wait for 3 sec. ensure no duplicate requests will be sent */
    aws_thread_current_sleep((uint64_t)ONE_SEC * 3);
    /* Check all received packets, only one publish and subscribe received */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(state_test_data->mock_server));
    size_t pre_index = SIZE_MAX;
    ASSERT_NOT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_PUBLISH, &pre_index));
    if (pre_index + 1 < mqtt_mock_server_decoded_packets_count(handler)) {
        /* If it's not the last packet, search again, and result should be NULL. */
        ASSERT_NULL(
            mqtt_mock_server_find_decoded_packet_by_type(handler, pre_index + 1, AWS_MQTT_PACKET_PUBLISH, NULL));
    }
    ASSERT_NOT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_SUBSCRIBE, &pre_index));
    if (pre_index + 1 < mqtt_mock_server_decoded_packets_count(handler)) {
        ASSERT_NULL(
            mqtt_mock_server_find_decoded_packet_by_type(handler, pre_index + 1, AWS_MQTT_PACKET_SUBSCRIBE, NULL));
    }

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_not_resend_packets_on_healthy_connection,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_not_resend_packets_on_healthy_connection_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Make requests during offline, and destory the connection before ever online, the resource should be cleaned up
 * properly */
static int s_test_mqtt_connection_destory_pending_requests_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload,
            aws_test311_on_op_complete,
            state_test_data) > 0);
    ASSERT_TRUE(
        aws_mqtt_client_connection_subscribe(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            aws_test311_on_publish_received,
            state_test_data,
            NULL,
            aws_test311_on_suback,
            state_test_data) > 0);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_destory_pending_requests,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_destory_pending_requests_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Make a clean session connection, all the request will not be retried when the connection lost */
static int s_test_mqtt_clean_session_not_retry_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true, /* make a clean_session connection */
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    struct aws_channel_handler *handler = state_test_data->mock_server;
    mqtt_mock_server_disable_auto_ack(handler);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload,
            aws_test311_on_op_complete,
            state_test_data) > 0);
    ASSERT_TRUE(
        aws_mqtt_client_connection_subscribe(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            aws_test311_on_publish_received,
            state_test_data,
            NULL,
            aws_test311_on_suback,
            state_test_data) > 0);
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    /* Shutdown the connection */
    aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
    aws_test311_wait_for_interrupt_to_complete(state_test_data);

    /* Once the connection lost, the requests will fail */
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_MQTT_CANCELLED_FOR_CLEAN_SESSION);
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_MQTT_CANCELLED_FOR_CLEAN_SESSION);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_clean_session_not_retry,
    s_setup_mqtt_server_fn,
    s_test_mqtt_clean_session_not_retry_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Make a clean session connection, the previous session will be discard when a new connection with clean session true
 * is created */
static int s_test_mqtt_clean_session_discard_previous_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true, /* make a clean_session connection */
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Test Message 1");
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    /* Requests made now will be considered as the previous session. */
    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload,
            aws_test311_on_op_complete,
            state_test_data) > 0);
    ASSERT_TRUE(
        aws_mqtt_client_connection_subscribe(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            aws_test311_on_publish_received,
            state_test_data,
            NULL,
            aws_test311_on_suback,
            state_test_data) > 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    struct aws_channel_handler *handler = state_test_data->mock_server;

    aws_test311_wait_for_ops_completed(state_test_data);
    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_MQTT_CANCELLED_FOR_CLEAN_SESSION);
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_MQTT_CANCELLED_FOR_CLEAN_SESSION);

    /* Check no request is received by the server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(handler));
    ASSERT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_PUBLISH, NULL));
    ASSERT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_SUBSCRIBE, NULL));

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_clean_session_discard_previous,
    s_setup_mqtt_server_fn,
    s_test_mqtt_clean_session_discard_previous_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Make a clean session connection, the requests after the connect function will be considered as the next session */
static int s_test_mqtt_clean_session_keep_next_session_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true, /* make a clean_session connection */
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Test Message 1");
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    /* Requests made after the connect function will be considered as the next session, and will be sent eventually */
    ASSERT_TRUE(
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload,
            aws_test311_on_op_complete,
            state_test_data) > 0);
    ASSERT_TRUE(
        aws_mqtt_client_connection_subscribe(
            state_test_data->mqtt_connection,
            &topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            aws_test311_on_publish_received,
            state_test_data,
            NULL,
            aws_test311_on_suback,
            state_test_data) > 0);
    aws_test311_wait_for_connection_to_complete(state_test_data);
    struct aws_channel_handler *handler = state_test_data->mock_server;

    aws_test311_wait_for_ops_completed(state_test_data);
    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_SUCCESS);
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_SUCCESS);

    /* Check no request is received by the server */
    ASSERT_SUCCESS(mqtt_mock_server_decode_packets(handler));
    ASSERT_NOT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_PUBLISH, NULL));
    ASSERT_NOT_NULL(mqtt_mock_server_find_decoded_packet_by_type(handler, 0, AWS_MQTT_PACKET_SUBSCRIBE, NULL));

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_clean_session_keep_next_session,
    s_setup_mqtt_server_fn,
    s_test_mqtt_clean_session_keep_next_session_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the publish failed.
 */
static int s_test_mqtt_connection_publish_QoS1_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* make a publish with QoS 1 immediate. */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    /* publish should complete after the shutdown */
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Check the publish has been completed with timeout error */
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_MQTT_TIMEOUT);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_publish_QoS1_timeout,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_publish_QoS1_timeout_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens we still send ping reqs
 */
static int s_test_mqtt_connection_publish_QoS1_timeout_with_ping_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = 99,
        .protocol_operation_timeout_ms = 20,
        .keep_alive_time_secs = 3, // connection->keep_alive_time_ns, pushoff the timestamp by that amount
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* make a publish with QoS 1 immediate. */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 10;
    aws_mutex_unlock(&state_test_data->lock);

    for (int i = 0; i < 10; i++) {
        uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload_1,
            aws_test311_on_op_complete,
            state_test_data);
        ASSERT_TRUE(packet_id_1 > 0);

        // sleep 1 second
        aws_thread_current_sleep(1000000000);
    }

    // Wait for 3 seconds
    aws_thread_current_sleep(3000000000);

    // make sure we are still receiveing pings when the connection is down, in other words ping pushoff is not happening
    ASSERT_TRUE(mqtt_mock_server_get_ping_count(state_test_data->mock_server) > 1);

    aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);

    /* publish should complete after the shutdown */
    aws_test311_wait_for_ops_completed(state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_publish_QoS1_timeout_with_ping,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_publish_QoS1_timeout_with_ping_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the publish failed.
 */
static int s_test_mqtt_connection_publish_QoS1_timeout_override_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* make a publish with QoS 1 immediate. */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint16_t packet_id_1 = aws_mqtt_client_connection_311_publish(
        state_test_data->mqtt_connection->impl,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data,
        aws_timestamp_convert(3, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    ASSERT_TRUE(packet_id_1 > 0);

    /* publish should complete after the shutdown */
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Check the publish has been completed with timeout error */
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_MQTT_TIMEOUT);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_publish_QoS1_timeout_override,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_publish_QoS1_timeout_override_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the unsubscribe failed.
 */
static int s_test_mqtt_connection_unsubscribe_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* unsubscribe to the first topic */
    uint16_t unsub_packet_id = aws_mqtt_client_connection_unsubscribe(
        state_test_data->mqtt_connection, &pub_topic, aws_test311_on_op_complete, state_test_data);
    ASSERT_TRUE(unsub_packet_id > 0);

    /* unsubscribe should complete after the timeout */
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Check that the unsubscribe has been completed with a timeout error */
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_MQTT_TIMEOUT);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_unsubscribe_timeout,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_unsubscribe_timeout_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the unsubscribe failed.
 */
static int s_test_mqtt_connection_unsubscribe_timeout_override_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* unsubscribe to the first topic */
    uint16_t unsub_packet_id = aws_mqtt_client_connection_311_unsubscribe(
        state_test_data->mqtt_connection->impl,
        &pub_topic,
        aws_test311_on_op_complete,
        state_test_data,
        aws_timestamp_convert(3, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    ASSERT_TRUE(unsub_packet_id > 0);

    /* unsubscribe should complete after the timeout */
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Check that the unsubscribe has been completed with a timeout error */
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_MQTT_TIMEOUT);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_unsubscribe_timeout_override,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_unsubscribe_timeout_override_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the subscribe failed.
 */
static int s_test_mqtt_connection_subscribe_single_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* subscribe */
    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    uint16_t sub_packet_id = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        aws_test311_on_publish_received,
        state_test_data,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(sub_packet_id > 0);

    /* subscribe should complete after the timeout */
    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    /* Check that the subscribe has been completed with a timeout error */
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_MQTT_TIMEOUT);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_subscribe_single_timeout,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_subscribe_single_timeout_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the subscribe failed.
 */
static int s_test_mqtt_connection_subscribe_single_timeout_override_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* subscribe */
    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    uint16_t sub_packet_id = aws_mqtt_client_connection_311_subscribe(
        state_test_data->mqtt_connection->impl,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        aws_test311_on_publish_received,
        state_test_data,
        NULL,
        aws_test311_on_suback,
        state_test_data,
        aws_timestamp_convert(3, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    ASSERT_TRUE(sub_packet_id > 0);

    /* subscribe should complete after the timeout */
    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    /* Check that the subscribe has been completed with a timeout error */
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_MQTT_TIMEOUT);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_subscribe_single_timeout_override,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_subscribe_single_timeout_override_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the multi-subscribe
 * failed.
 */
static int s_test_mqtt_connection_subscribe_multi_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    struct aws_byte_cursor sub_topic_1 = aws_byte_cursor_from_c_str("/test/topic1");
    struct aws_byte_cursor sub_topic_2 = aws_byte_cursor_from_c_str("/test/topic2");

    struct aws_mqtt_topic_subscription sub1 = {
        .topic = sub_topic_1,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };
    struct aws_mqtt_topic_subscription sub2 = {
        .topic = sub_topic_2,
        .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        .on_publish = aws_test311_on_publish_received,
        .on_cleanup = NULL,
        .on_publish_ud = state_test_data,
    };

    struct aws_array_list topic_filters;
    size_t list_len = 2;
    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, static_buf, list_len * sizeof(struct aws_mqtt_topic_subscription));
    aws_array_list_init_static(&topic_filters, static_buf, list_len, sizeof(struct aws_mqtt_topic_subscription));

    aws_array_list_push_back(&topic_filters, &sub1);
    aws_array_list_push_back(&topic_filters, &sub2);

    uint16_t sub_packet_id = aws_mqtt_client_connection_subscribe_multiple(
        state_test_data->mqtt_connection, &topic_filters, aws_test311_on_multi_suback, state_test_data);

    ASSERT_TRUE(sub_packet_id > 0);

    /* subscribe should complete after the timeout */
    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    /* Check that the subscribe has been completed with a timeout error */
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_MQTT_TIMEOUT);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_subscribe_multi_timeout,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_subscribe_multi_timeout_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and timeout happens and the subscribe failed.
 */
static int s_test_mqtt_connection_resubscribe_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* subscribe */
    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    uint16_t sub_packet_id = aws_mqtt_client_connection_subscribe(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        aws_test311_on_publish_received,
        state_test_data,
        NULL,
        aws_test311_on_suback,
        state_test_data);
    ASSERT_TRUE(sub_packet_id > 0);

    /* subscribe should complete after the timeout */
    aws_test311_wait_for_subscribe_to_complete(state_test_data);
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_SUCCESS);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* reconnection to the same server */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    uint16_t resub_packet_id = aws_mqtt_resubscribe_existing_topics(
        state_test_data->mqtt_connection, aws_test311_on_multi_suback, state_test_data);
    ASSERT_TRUE(resub_packet_id > 0);
    aws_test311_wait_for_subscribe_to_complete(state_test_data);

    /* Check that the resubscribe has been completed with a timeout error */
    ASSERT_UINT_EQUALS(state_test_data->subscribe_complete_error, AWS_ERROR_MQTT_TIMEOUT);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_resubscribe_timeout,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_resubscribe_timeout_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that connection is healthy, user set the timeout for request, and connection lost will reset timeout.
 */
static int s_test_mqtt_connection_publish_QoS1_timeout_connection_lost_reset_time_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Disable the auto ACK packets sent by the server, which blocks the requests to complete */
    mqtt_mock_server_disable_auto_ack(state_test_data->mock_server);

    /* make a publish with QoS 1 immediate. */
    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);

    /* sleep for 2 sec, close the connection */
    aws_thread_current_sleep((uint64_t)ONE_SEC * 2);
    /* Kill the connection, the requests will be retried and the timeout will be reset. */
    aws_channel_shutdown(state_test_data->server_channel, AWS_ERROR_INVALID_STATE);
    aws_test311_wait_for_reconnect_to_complete(state_test_data);
    /* sleep for 2 sec again, in total the response has not received for more than 4 sec, timeout should happen if the
     * lost of connection not reset the timeout */
    aws_thread_current_sleep((uint64_t)ONE_SEC * 2);
    /* send a puback */
    ASSERT_SUCCESS(mqtt_mock_server_send_puback(state_test_data->mock_server, packet_id_1));

    /* publish should complete after the shutdown */
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Check the publish has been completed successfully since the lost of the connection reset the timeout */
    ASSERT_UINT_EQUALS(state_test_data->op_complete_error, AWS_ERROR_SUCCESS);
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_publish_QoS1_timeout_connection_lost_reset_time,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_publish_QoS1_timeout_connection_lost_reset_time_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/* Function called for testing the on_connection_closed callback */
static void s_on_connection_closed_fn(
    struct aws_mqtt_client_connection *connection,
    struct on_connection_closed_data *data,
    void *userdata) {
    (void)connection;
    (void)data;

    struct mqtt_connection_state_test *state_test_data = (struct mqtt_connection_state_test *)userdata;

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->connection_close_calls += 1;
    aws_mutex_unlock(&state_test_data->lock);
}

/**
 * Test that the connection close callback is fired only once and when the connection was closed
 */
static int s_test_mqtt_connection_close_callback_simple_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };
    aws_mqtt_client_connection_set_connection_closed_handler(
        state_test_data->mqtt_connection, s_on_connection_closed_fn, state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* sleep for 2 sec, just to make sure the connection is stable */
    aws_thread_current_sleep((uint64_t)ONE_SEC * 2);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Make sure the callback was called and the value is what we expect */
    ASSERT_UINT_EQUALS(1, state_test_data->connection_close_calls);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_close_callback_simple,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_close_callback_simple_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that the connection close callback is NOT fired during an interrupt
 */
static int s_test_mqtt_connection_close_callback_interrupted_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };
    aws_mqtt_client_connection_set_connection_closed_handler(
        state_test_data->mqtt_connection, s_on_connection_closed_fn, state_test_data);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    /* Kill the connection */
    aws_channel_shutdown(state_test_data->server_channel, AWS_ERROR_INVALID_STATE);
    aws_test311_wait_for_reconnect_to_complete(state_test_data);

    /* sleep for 2 sec, just to make sure the connection is stable */
    aws_thread_current_sleep((uint64_t)ONE_SEC * 2);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* Make sure the callback was called only ONCE and the value is what we expect */
    ASSERT_UINT_EQUALS(1, state_test_data->connection_close_calls);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_close_callback_interrupted,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_close_callback_interrupted_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that the connection close callback is called every time a disconnect happens, if it happens multiple times
 */
static int s_test_mqtt_connection_close_callback_multi_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };
    aws_mqtt_client_connection_set_connection_closed_handler(
        state_test_data->mqtt_connection, s_on_connection_closed_fn, state_test_data);

    int disconnect_amount = 10;
    for (int i = 0; i < disconnect_amount; i++) {
        ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
        aws_test311_wait_for_connection_to_complete(state_test_data);

        /* Disconnect */
        ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
            state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
        aws_test311_wait_for_disconnect_to_complete(state_test_data);
    }

    /* Make sure the callback was called disconnect_amount times */
    ASSERT_UINT_EQUALS(disconnect_amount, state_test_data->connection_close_calls);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_close_callback_multi,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_close_callback_multi_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_test_mqtt_connection_reconnection_backoff_stable(struct aws_allocator *allocator, void *ctx) {

    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,

        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    uint64_t time_before = 0;
    uint64_t time_after = 0;
    for (int i = 0; i < 3; i++) {
        /* sleep for AWS_RESET_RECONNECT_BACKOFF_DELAY_SECONDS to make sure our connection is successful */
        aws_thread_current_sleep(
            (uint64_t)ONE_SEC * AWS_RESET_RECONNECT_BACKOFF_DELAY_SECONDS +
            RECONNECT_BACKOFF_DELAY_ERROR_MARGIN_NANO_SECONDS);

        aws_high_res_clock_get_ticks(&time_before);

        /* shut it down and make sure the client automatically reconnects.*/
        aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
        aws_test311_wait_for_reconnect_to_complete(state_test_data);

        aws_high_res_clock_get_ticks(&time_after);

        uint64_t reconnection_backoff_time = time_after - time_before;
        uint64_t remainder = 0;
        ASSERT_TRUE(
            aws_timestamp_convert(reconnection_backoff_time, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_SECS, &remainder) ==
            DEFAULT_MIN_RECONNECT_DELAY_SECONDS);
        ASSERT_TRUE(remainder <= RECONNECT_BACKOFF_DELAY_ERROR_MARGIN_NANO_SECONDS);
    }

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_reconnection_backoff_stable,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_reconnection_backoff_stable,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_test_mqtt_connection_reconnection_backoff_unstable(struct aws_allocator *allocator, void *ctx) {

    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,

        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    uint64_t time_before = 0;
    uint64_t time_after = 0;
    uint64_t expected_reconnect_backoff = 1;
    for (int i = 0; i < 3; i++) {

        aws_high_res_clock_get_ticks(&time_before);

        /* shut it down and make sure the client automatically reconnects.*/
        aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
        aws_test311_wait_for_reconnect_to_complete(state_test_data);

        aws_high_res_clock_get_ticks(&time_after);

        uint64_t reconnection_backoff = time_after - time_before;
        uint64_t remainder = 0;
        ASSERT_TRUE(
            aws_timestamp_convert(reconnection_backoff, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_SECS, &remainder) ==
            expected_reconnect_backoff);
        ASSERT_TRUE(remainder <= RECONNECT_BACKOFF_DELAY_ERROR_MARGIN_NANO_SECONDS);

        // Increase the exponential backoff
        expected_reconnect_backoff = aws_min_u64(expected_reconnect_backoff * 2, 10);
    }

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_reconnection_backoff_unstable,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_reconnection_backoff_unstable,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_test_mqtt_connection_reconnection_backoff_reset(struct aws_allocator *allocator, void *ctx) {

    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,

        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    uint64_t time_before = 0;
    uint64_t time_after = 0;
    uint64_t expected_reconnect_backoff = 1;
    uint64_t reconnection_backoff = 0;
    for (int i = 0; i < 3; i++) {

        aws_high_res_clock_get_ticks(&time_before);

        /* shut it down and make sure the client automatically reconnects.*/
        aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
        aws_test311_wait_for_reconnect_to_complete(state_test_data);

        aws_high_res_clock_get_ticks(&time_after);
        reconnection_backoff = time_after - time_before;
        ASSERT_TRUE(
            aws_timestamp_convert(reconnection_backoff, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_SECS, NULL) >=
            expected_reconnect_backoff);

        expected_reconnect_backoff = aws_min_u64(expected_reconnect_backoff * 2, 10);
    }

    /* sleep for AWS_RESET_RECONNECT_BACKOFF_DELAY_SECONDS to make sure our connection is successful */
    aws_thread_current_sleep(
        (uint64_t)ONE_SEC * AWS_RESET_RECONNECT_BACKOFF_DELAY_SECONDS +
        RECONNECT_BACKOFF_DELAY_ERROR_MARGIN_NANO_SECONDS);

    aws_high_res_clock_get_ticks(&time_before);

    /* shut it down and make sure the client automatically reconnects.*/
    aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
    aws_test311_wait_for_reconnect_to_complete(state_test_data);

    aws_high_res_clock_get_ticks(&time_after);
    reconnection_backoff = time_after - time_before;
    uint64_t remainder = 0;
    ASSERT_TRUE(
        aws_timestamp_convert(reconnection_backoff, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_SECS, &remainder) ==
        DEFAULT_MIN_RECONNECT_DELAY_SECONDS);
    ASSERT_TRUE(remainder <= RECONNECT_BACKOFF_DELAY_ERROR_MARGIN_NANO_SECONDS);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_reconnection_backoff_reset,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_reconnection_backoff_reset,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_test_mqtt_connection_reconnection_backoff_reset_after_disconnection(
    struct aws_allocator *allocator,
    void *ctx) {

    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,

        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    uint64_t time_before = 0;
    uint64_t time_after = 0;
    uint64_t expected_reconnect_backoff = 1;
    uint64_t reconnection_backoff = 0;
    for (int i = 0; i < 3; i++) {
        aws_high_res_clock_get_ticks(&time_before);

        /* shut it down and make sure the client automatically reconnects.*/
        aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
        aws_test311_wait_for_reconnect_to_complete(state_test_data);

        aws_high_res_clock_get_ticks(&time_after);
        reconnection_backoff = time_after - time_before;
        ASSERT_TRUE(
            aws_timestamp_convert(reconnection_backoff, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_SECS, NULL) >=
            expected_reconnect_backoff);

        expected_reconnect_backoff = aws_min_u64(expected_reconnect_backoff * 2, 10);
    }
    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    /* connect again */
    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    aws_high_res_clock_get_ticks(&time_before);

    aws_channel_shutdown(state_test_data->server_channel, AWS_OP_SUCCESS);
    aws_test311_wait_for_reconnect_to_complete(state_test_data);

    aws_high_res_clock_get_ticks(&time_after);
    reconnection_backoff = time_after - time_before;
    uint64_t remainder = 0;
    ASSERT_TRUE(
        aws_timestamp_convert(reconnection_backoff, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_SECS, &remainder) ==
        DEFAULT_MIN_RECONNECT_DELAY_SECONDS);
    ASSERT_TRUE(remainder <= RECONNECT_BACKOFF_DELAY_ERROR_MARGIN_NANO_SECONDS);

    /* Disconnect */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_reconnection_backoff_reset_after_disconnection,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_reconnection_backoff_reset_after_disconnection,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Makes a CONNECT, with 1 second keep alive ping interval, does nothing for roughly 4 seconds, ensures 4 pings are sent
 */
static int s_test_mqtt_connection_ping_norm_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = 1,
        .ping_timeout_ms = 100,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));

    /* Wait for 4.5 seconds (to account for slight drift/jitter) */
    aws_thread_current_sleep(4500000000);

    /* Ensure the server got 4 PING packets */
    ASSERT_INT_EQUALS(4, mqtt_mock_server_get_ping_count(state_test_data->mock_server));

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_ping_norm,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_ping_norm_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Makes a CONNECT, with 1 second keep alive ping interval. Publish QOS1 message for 4.5 seconds and then ensure NO
 * pings were sent. (The ping time will be push off on ack )
 */
static int s_test_mqtt_connection_ping_no_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = 1,
        .ping_timeout_ms = 100,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    uint64_t begin_timestamp = 0;
    uint64_t elapsed_time = 0;
    uint64_t now = 0;
    aws_high_res_clock_get_ticks(&begin_timestamp);
    uint64_t test_duration = (uint64_t)4 * AWS_TIMESTAMP_NANOS;

    // Make sure we publish for 4 seconds;
    while (elapsed_time < test_duration) {
        /* Publish qos1*/
        uint16_t packet_id = aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload_1,
            aws_test311_on_op_complete,
            state_test_data);
        ASSERT_TRUE(packet_id > 0);

        aws_thread_current_sleep(500000000); /* Sleep 0.5 seconds to avoid spamming*/

        aws_high_res_clock_get_ticks(&now);
        elapsed_time = now - begin_timestamp;
    }

    aws_thread_current_sleep(250000000); /* Sleep 0.25 seconds to consider jitter*/

    /* Ensure the server got 0 PING packets */
    ASSERT_INT_EQUALS(0, mqtt_mock_server_get_ping_count(state_test_data->mock_server));

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_ping_no,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_ping_no_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Makes a CONNECT, with 1 second keep alive ping interval, publish a qos0 messages for 4.5 seconds.
 * We should send a total of 4 pings
 */
static int s_test_mqtt_connection_ping_noack_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = 1,
        .ping_timeout_ms = 100,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");

    uint64_t begin_timestamp = 0;
    uint64_t elapsed_time = 0;
    uint64_t now = 0;
    aws_high_res_clock_get_ticks(&begin_timestamp);
    uint64_t test_duration = (uint64_t)4 * AWS_TIMESTAMP_NANOS;

    // Make sure we publish for 4 seconds;
    while (elapsed_time < test_duration) {
        /* Publish qos0*/
        uint16_t packet_id = aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &pub_topic,
            AWS_MQTT_QOS_AT_MOST_ONCE,
            false,
            &payload_1,
            aws_test311_on_op_complete,
            state_test_data);
        ASSERT_TRUE(packet_id > 0);

        aws_thread_current_sleep(500000000); /* Sleep 0.5 seconds to avoid spamming*/
        aws_high_res_clock_get_ticks(&now);
        elapsed_time = now - begin_timestamp;
    }

    aws_thread_current_sleep(250000000); /* Sleep 0.25 seconds to consider jitter*/

    /* Ensure the server got 4 PING packets */
    ASSERT_INT_EQUALS(4, mqtt_mock_server_get_ping_count(state_test_data->mock_server));

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_ping_noack,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_ping_noack_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test to make sure the PING timing is correct if a publish/packet is sent near the end of the keep alive time.
 * Note: Because of socket write jitter and scheduling jitter, the times have a 0.25 (quarter of a second) delta range.
 *
 * To test this, this test has a keep alive at 4 seconds and makes a publish after 3 seconds. This resets the ping
 * task and will reschedule it for 4 seconds from the publish (the PING will be scheduled for 3 seconds after the 4
 * second task is invoked). This test then waits a second, makes sure a PING has NOT been sent (with no ping reschedule,
 * it would have) and then waits 3 seconds to ensure and checks that a PING has been sent. Finally, it waits 4 seconds
 * to ensure a second PING was sent at the correct time.
 */
static int s_test_mqtt_connection_ping_basic_scenario_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = 4,
        .ping_timeout_ms = 100,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    /* PING will be in 4 seconds */

    aws_thread_current_sleep(3000000000); /* Wait 3 seconds */

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* Make a publish */
    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Publish packet written at 3 seconds */

    aws_thread_current_sleep(1250000000); /* Wait 1.25 second (the extra 0.25 is to account for jitter) */
    /* PING task has executed and been rescheduled at 3 seconds (1 second passed) */

    /* Ensure the server has gotten 0 PING packets so far */
    ASSERT_INT_EQUALS(0, mqtt_mock_server_get_ping_count(state_test_data->mock_server));

    aws_thread_current_sleep(
        3000000000); /* Wait 3 seconds more (no jitter needed because we already added 0.25 in the prior sleep) */
    /* PING task (from publish) has been executed */

    /* Ensure the server has gotten only 1 PING packet */
    ASSERT_INT_EQUALS(1, mqtt_mock_server_get_ping_count(state_test_data->mock_server));

    aws_thread_current_sleep(4000000000); /* Wait 4 seconds (since we didn't publish or anything, it should go back to
                                             normal keep alive time) */

    /* Ensure the server has gotten 2 PING packets */
    ASSERT_INT_EQUALS(2, mqtt_mock_server_get_ping_count(state_test_data->mock_server));

    /* Disconnect and finish! */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_ping_basic_scenario,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_ping_basic_scenario_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * The test is the same as above (s_test_mqtt_connection_ping_basic_scenario_fn) but after the first publish, it waits
 * 1 second and makes another publish, before waiting 4 seconds from that point and ensures only a single PING was sent.
 */
static int s_test_mqtt_connection_ping_double_scenario_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = 4,
        .ping_timeout_ms = 100,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);
    /* PING will be in 4 seconds */

    aws_thread_current_sleep(3000000000); /* Wait 3 seconds */

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 1;
    aws_mutex_unlock(&state_test_data->lock);

    /* Make a publish */
    struct aws_byte_cursor pub_topic = aws_byte_cursor_from_c_str("/test/topic");
    struct aws_byte_cursor payload_1 = aws_byte_cursor_from_c_str("Test Message 1");
    uint16_t packet_id_1 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_1 > 0);
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Publish packet written at 3 seconds */

    aws_thread_current_sleep(1250000000); /* Wait 1.25 second (the extra 0.25 is to account for jitter) */
    /* PING task has executed and been rescheduled at 3 seconds (1 second passed) */

    /* Ensure the server has gotten 0 PING packets so far */
    ASSERT_INT_EQUALS(0, mqtt_mock_server_get_ping_count(state_test_data->mock_server));

    aws_thread_current_sleep(750000000); /* wait 0.75 seconds */

    aws_mutex_lock(&state_test_data->lock);
    state_test_data->expected_ops_completed = 2;
    aws_mutex_unlock(&state_test_data->lock);

    /* Make as second publish */
    uint16_t packet_id_2 = aws_mqtt_client_connection_publish(
        state_test_data->mqtt_connection,
        &pub_topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload_1,
        aws_test311_on_op_complete,
        state_test_data);
    ASSERT_TRUE(packet_id_2 > 0);
    aws_test311_wait_for_ops_completed(state_test_data);
    /* Publish packet written at 2 seconds (relative to PING that was scheduled above) */

    aws_thread_current_sleep(4250000000); /* Wait 4.25 (the extra 0.25 is to account for jitter) seconds */
    /**
     * Note: The extra 2 seconds are to account for the time it takes to publish on the socket. Despite best efforts,
     * I cannot get it to trigger right away in the test suite...
     * If you read the logs though, the scheduled PINGs should be 4 seconds, 3 seconds, 2 seconds, 4 seconds
     */

    /* Ensure the server has gotten only 1 PING packet */
    ASSERT_INT_EQUALS(1, mqtt_mock_server_get_ping_count(state_test_data->mock_server));
    /**
     * At this point a new PING task is scheduled for 4 seconds, but we do not care anymore for the
     * purposes of this test.
     */

    /* Disconnect and finish! */
    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_ping_double_scenario,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_ping_double_scenario_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/**
 * Test that the connection termination callback is fired for the connection that was not actually connected ever.
 * \note Other tests use on_connection_termination callback as well, so one simple dedicated case is enough.
 */
static int s_test_mqtt_connection_termination_callback_simple_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_connection_termination_handler(
        state_test_data->mqtt_connection, aws_test311_on_connection_termination_fn, state_test_data));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_connection_termination_callback_simple,
    s_setup_mqtt_server_fn,
    s_test_mqtt_connection_termination_callback_simple_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/*
 * Verifies that calling publish with a bad qos results in a validation failure
 */
static int s_test_mqtt_validation_failure_publish_qos_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("a/b");
    ASSERT_INT_EQUALS(
        0,
        aws_mqtt_client_connection_publish(
            state_test_data->mqtt_connection,
            &topic,
            (enum aws_mqtt_qos)3,
            true,
            NULL,
            aws_test311_on_op_complete,
            state_test_data));
    int error_code = aws_last_error();
    ASSERT_INT_EQUALS(AWS_ERROR_MQTT_INVALID_QOS, error_code);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_validation_failure_publish_qos,
    s_setup_mqtt_server_fn,
    s_test_mqtt_validation_failure_publish_qos_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/*
 * Verifies that calling subscribe_multiple with no topics causes a validation failure
 */
static int s_test_mqtt_validation_failure_subscribe_empty_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    struct aws_array_list topic_filters;
    size_t list_len = 2;
    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, static_buf, list_len * sizeof(struct aws_mqtt_topic_subscription));
    aws_array_list_init_static(&topic_filters, static_buf, list_len, sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_INT_EQUALS(
        0,
        aws_mqtt_client_connection_subscribe_multiple(
            state_test_data->mqtt_connection, &topic_filters, aws_test311_on_multi_suback, state_test_data));
    int error_code = aws_last_error();
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, error_code);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_validation_failure_subscribe_empty,
    s_setup_mqtt_server_fn,
    s_test_mqtt_validation_failure_subscribe_empty_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

/*
 * Verifies that calling unsubscribe with a null topic causes a validation failure (not a crash)
 */
static int s_test_mqtt_validation_failure_unsubscribe_null_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(state_test_data);

    ASSERT_INT_EQUALS(
        0,
        aws_mqtt_client_connection_unsubscribe(
            state_test_data->mqtt_connection, NULL, aws_test311_on_op_complete, state_test_data));
    int error_code = aws_last_error();
    ASSERT_INT_EQUALS(AWS_ERROR_MQTT_INVALID_TOPIC, error_code);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        state_test_data->mqtt_connection, aws_test311_on_disconnect_fn, state_test_data));
    aws_test311_wait_for_disconnect_to_complete(state_test_data);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_validation_failure_unsubscribe_null,
    s_setup_mqtt_server_fn,
    s_test_mqtt_validation_failure_unsubscribe_null_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

static struct aws_byte_cursor s_bad_client_id_utf8 = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\x41\xED\xA0\x80\x41");
static struct aws_byte_cursor s_bad_username_utf8 = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\x41\x00\x41");
static struct aws_byte_cursor s_bad_will_topic_utf8 = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\x41\xED\xBF\xBF");

static int s_test_mqtt_validation_failure_connect_invalid_client_id_utf8_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .user_data = state_test_data,
        .clean_session = false,
        .client_id = s_bad_client_id_utf8,
        .host_name = aws_byte_cursor_from_c_str(state_test_data->endpoint.address),
        .socket_options = &state_test_data->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_FAILS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    int error_code = aws_last_error();
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, error_code);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_validation_failure_connect_invalid_client_id_utf8,
    s_setup_mqtt_server_fn,
    s_test_mqtt_validation_failure_connect_invalid_client_id_utf8_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_test_mqtt_validation_failure_invalid_will_topic_utf8_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_byte_cursor will_topic_cursor = s_bad_will_topic_utf8;
    ASSERT_FAILS(aws_mqtt_client_connection_set_will(
        state_test_data->mqtt_connection, &will_topic_cursor, AWS_MQTT_QOS_AT_MOST_ONCE, false, &will_topic_cursor));
    int error_code = aws_last_error();
    ASSERT_INT_EQUALS(AWS_ERROR_MQTT_INVALID_TOPIC, error_code);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_validation_failure_invalid_will_topic_utf8,
    s_setup_mqtt_server_fn,
    s_test_mqtt_validation_failure_invalid_will_topic_utf8_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_mqtt_validation_failure_invalid_will_qos_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_byte_cursor will_topic_cursor = aws_byte_cursor_from_c_str("a/b");

    ASSERT_FAILS(aws_mqtt_client_connection_set_will(
        state_test_data->mqtt_connection, &will_topic_cursor, 12, false, &will_topic_cursor));
    int error_code = aws_last_error();
    ASSERT_INT_EQUALS(AWS_ERROR_MQTT_INVALID_QOS, error_code);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_validation_failure_invalid_will_qos,
    s_setup_mqtt_server_fn,
    s_mqtt_validation_failure_invalid_will_qos_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

static int s_test_mqtt_validation_failure_invalid_username_utf8_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_byte_cursor login_cursor = s_bad_username_utf8;
    ASSERT_FAILS(aws_mqtt_client_connection_set_login(state_test_data->mqtt_connection, &login_cursor, NULL));
    int error_code = aws_last_error();
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, error_code);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_validation_failure_invalid_username_utf8,
    s_setup_mqtt_server_fn,
    s_test_mqtt_validation_failure_invalid_username_utf8_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)

static void s_mqtt_client_test_websocket_failed_transform(
    struct aws_http_message *request,
    void *user_data,
    aws_mqtt_transform_websocket_handshake_complete_fn *complete_fn,
    void *complete_ctx) {

    struct mqtt_connection_state_test *state_test_data = user_data;
    struct aws_mqtt_client_connection_311_impl *mqtt311_connection = state_test_data->mqtt_connection->impl;
    /* This test should also perform a regression check: websocket transform failed on a non-eventloop thread does not
     * cause crash. */
    AWS_FATAL_ASSERT(!aws_event_loop_thread_is_callers_thread(mqtt311_connection->loop));

    (*complete_fn)(request, AWS_ERROR_INVALID_STATE, complete_ctx);
}

/* Connection failure test where websocket MQTT channel establishment fails due to handshake transform failure */
static int s_test_mqtt_websocket_failed_transform_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct mqtt_connection_state_test *state_test_data = ctx;

    struct aws_mqtt_connection_options connection_options = {
        .socket_options = &state_test_data->socket_options,
    };

    aws_mqtt_client_connection_use_websockets(
        state_test_data->mqtt_connection, s_mqtt_client_test_websocket_failed_transform, state_test_data, NULL, NULL);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(state_test_data->mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_fail(state_test_data);

    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_STATE, state_test_data->error);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    mqtt_websocket_failed_transform,
    s_setup_mqtt_server_fn,
    s_test_mqtt_websocket_failed_transform_fn,
    s_clean_up_mqtt_server_fn,
    &test_data)
