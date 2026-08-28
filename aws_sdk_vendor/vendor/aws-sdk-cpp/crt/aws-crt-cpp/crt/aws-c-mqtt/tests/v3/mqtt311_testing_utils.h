/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AWS_C_MQTT_MQTT311_TESTING_UTILS_H
#define AWS_C_MQTT_MQTT311_TESTING_UTILS_H

#include <aws/mqtt/mqtt.h>

#include <aws/common/array_list.h>
#include <aws/common/condition_variable.h>
#include <aws/common/mutex.h>
#include <aws/io/socket.h>
#include <aws/mqtt/client.h>

#include <inttypes.h>

#define TEST_LOG_SUBJECT 60000
#define ONE_SEC 1000000000
// The value is extract from aws-c-mqtt/source/client.c
#define AWS_RESET_RECONNECT_BACKOFF_DELAY_SECONDS 10
#define RECONNECT_BACKOFF_DELAY_ERROR_MARGIN_NANO_SECONDS 500000000
#define DEFAULT_MIN_RECONNECT_DELAY_SECONDS 1

#define DEFAULT_TEST_PING_TIMEOUT_MS 1000
#define DEFAULT_TEST_KEEP_ALIVE_S 2

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
    bool connection_success;
    bool connection_failure;
    bool client_disconnect_completed;
    bool server_disconnect_completed;
    bool connection_interrupted;
    bool connection_resumed;
    bool subscribe_completed;
    bool listener_destroyed;
    bool connection_terminated;
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
    size_t connection_close_calls; /* All of the times on_connection_closed has been called */

    size_t connection_termination_calls; /* How many times on_connection_termination has been called, should be 1 */
};

AWS_EXTERN_C_BEGIN

int aws_test311_setup_mqtt_server_fn(struct aws_allocator *allocator, void *ctx);

int aws_test311_clean_up_mqtt_server_fn(struct aws_allocator *allocator, int setup_result, void *ctx);

void aws_test311_wait_for_interrupt_to_complete(struct mqtt_connection_state_test *state_test_data);

void aws_test311_wait_for_reconnect_to_complete(struct mqtt_connection_state_test *state_test_data);

void aws_test311_wait_for_connection_to_succeed(struct mqtt_connection_state_test *state_test_data);

void aws_test311_wait_for_connection_to_fail(struct mqtt_connection_state_test *state_test_data);

void aws_test311_on_connection_complete_fn(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata);

void aws_test311_wait_for_connection_to_complete(struct mqtt_connection_state_test *state_test_data);

void aws_test311_on_disconnect_fn(struct aws_mqtt_client_connection *connection, void *userdata);

void aws_test311_wait_for_disconnect_to_complete(struct mqtt_connection_state_test *state_test_data);

void aws_test311_wait_for_any_publish(struct mqtt_connection_state_test *state_test_data);

void aws_test311_on_publish_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata);

void aws_test311_wait_for_publish(struct mqtt_connection_state_test *state_test_data);

void aws_test311_on_suback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_byte_cursor *topic,
    enum aws_mqtt_qos qos,
    int error_code,
    void *userdata);

void aws_test311_wait_for_subscribe_to_complete(struct mqtt_connection_state_test *state_test_data);

void aws_test311_on_multi_suback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_array_list *topic_subacks,
    int error_code,
    void *userdata);

void aws_test311_on_op_complete(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    int error_code,
    void *userdata);

void aws_test311_wait_for_ops_completed(struct mqtt_connection_state_test *state_test_data);

void aws_test311_on_connection_termination_fn(void *userdata);

AWS_EXTERN_C_END

#endif // AWS_C_MQTT_MQTT311_TESTING_UTILS_H
