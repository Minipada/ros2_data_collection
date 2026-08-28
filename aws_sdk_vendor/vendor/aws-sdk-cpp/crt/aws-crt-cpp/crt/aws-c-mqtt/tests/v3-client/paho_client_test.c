/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/private/client_impl.h>

#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/packets.h>

#include <aws/io/channel.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/socket.h>
#include <aws/io/socket_channel_handler.h>

#include <aws/common/condition_variable.h>
#include <aws/common/device_random.h>
#include <aws/common/mutex.h>
#include <aws/common/string.h>
#include <aws/common/thread.h>

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#    include <windows.h>
#    define sleep Sleep
#else
#    include <unistd.h>
#endif

/* Note: to successfully run this test, a broker on localhost:1883 is required, eg: mosquitto */

static struct aws_byte_cursor s_client_id = {
    .ptr = (uint8_t *)"MyClientId1",
    .len = 11,
};
AWS_STATIC_STRING_FROM_LITERAL(s_subscribe_topic, "a/b");
AWS_STATIC_STRING_FROM_LITERAL(s_hostname, "localhost");

enum { PAYLOAD_LEN = 20000 };
static uint8_t s_payload[PAYLOAD_LEN];

struct test_context {
    struct aws_allocator *allocator;

    struct aws_mutex lock;
    struct aws_condition_variable signal;
    struct aws_event_loop_group *el_group;
    struct aws_host_resolver *resolver;
    struct aws_client_bootstrap *bootstrap;
    struct aws_mqtt_client *client;
    struct aws_mqtt_client_connection *connection;
    bool retained_packet_received;
    bool on_any_publish_fired;
    bool connection_complete;
    bool received_pub_ack;
    bool received_unsub_ack;
    bool on_disconnect_received;
};

static void s_on_packet_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *user_data) {

    (void)connection;
    (void)topic;
    (void)dup;
    (void)qos;
    (void)retain;

    struct aws_byte_cursor expected_payload = {
        .ptr = s_payload,
        .len = PAYLOAD_LEN,
    };
    (void)expected_payload;
    AWS_FATAL_ASSERT(aws_byte_cursor_eq(payload, &expected_payload));

    printf("2 started\n");

    struct test_context *tester = user_data;

    aws_mutex_lock(&tester->lock);
    tester->retained_packet_received = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_on_any_packet_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *user_data) {

    (void)connection;
    (void)topic;
    (void)dup;
    (void)qos;
    (void)retain;

    struct aws_byte_cursor expected_payload = {
        .ptr = s_payload,
        .len = PAYLOAD_LEN,
    };
    (void)expected_payload;
    AWS_FATAL_ASSERT(aws_byte_cursor_eq(payload, &expected_payload));

    struct test_context *tester = user_data;

    aws_mutex_lock(&tester->lock);
    tester->on_any_publish_fired = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_mqtt_on_puback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;
    (void)error_code;

    AWS_FATAL_ASSERT(error_code == AWS_OP_SUCCESS);

    struct test_context *tester = userdata;

    printf("2 started\n");

    aws_mutex_lock(&tester->lock);
    tester->received_pub_ack = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_mqtt_on_connection_complete(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *user_data) {

    (void)connection;
    (void)error_code;
    (void)return_code;
    (void)session_present;

    AWS_FATAL_ASSERT(error_code == AWS_OP_SUCCESS);
    AWS_FATAL_ASSERT(return_code == AWS_MQTT_CONNECT_ACCEPTED);
    AWS_FATAL_ASSERT(session_present == false);

    struct test_context *tester = user_data;

    printf("1 started\n");

    aws_mutex_lock(&tester->lock);
    tester->connection_complete = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_mqtt_on_interrupted(struct aws_mqtt_client_connection *connection, int error_code, void *userdata) {

    (void)connection;
    (void)error_code;
    (void)userdata;

    printf("Connection offline\n");
}

static void s_mqtt_on_resumed(
    struct aws_mqtt_client_connection *connection,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {

    (void)connection;
    (void)return_code;
    (void)session_present;
    (void)userdata;

    printf("Connection resumed\n");
}

static void s_mqtt_on_unsuback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;
    (void)error_code;

    AWS_FATAL_ASSERT(error_code == AWS_OP_SUCCESS);

    struct test_context *tester = userdata;

    printf("2 started\n");

    aws_mutex_lock(&tester->lock);
    tester->received_unsub_ack = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_mqtt_on_disconnect(struct aws_mqtt_client_connection *connection, void *user_data) {

    (void)connection;

    struct test_context *tester = user_data;

    printf("3 started\n");

    aws_mutex_lock(&tester->lock);
    tester->on_disconnect_received = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static bool s_is_connection_complete_pred(void *user_data) {
    struct test_context *tester = user_data;
    return tester->connection_complete;
}

static bool s_received_pub_ack_pred(void *user_data) {
    struct test_context *tester = user_data;
    return tester->received_pub_ack;
}

static bool s_on_disconnect_received(void *user_data) {
    struct test_context *tester = user_data;
    return tester->on_disconnect_received;
}

static bool s_retained_and_any_publish_pred(void *user_data) {
    struct test_context *tester = user_data;
    return tester->retained_packet_received && tester->on_any_publish_fired;
}
static bool s_received_unsub_ack_pred(void *user_data) {
    struct test_context *tester = user_data;
    return tester->received_unsub_ack;
}

static void s_wait_on_tester_predicate(struct test_context *tester, bool (*predicate)(void *)) {
    aws_mutex_lock(&tester->lock);
    aws_condition_variable_wait_pred(&tester->signal, &tester->lock, predicate, tester);
    aws_mutex_unlock(&tester->lock);
}

static int s_initialize_test(
    struct test_context *tester,
    struct aws_allocator *allocator,
    struct aws_mqtt_connection_options *conn_options) {
    aws_mqtt_library_init(allocator);

    aws_mutex_init(&tester->lock);
    aws_condition_variable_init(&tester->signal);

    tester->el_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = tester->el_group,
        .max_entries = 8,
    };
    tester->resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = tester->el_group,
        .host_resolver = tester->resolver,
    };
    tester->bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    tester->client = aws_mqtt_client_new(allocator, tester->bootstrap);
    tester->connection = aws_mqtt_client_connection_new(tester->client);

    aws_mqtt_client_connection_set_connection_interruption_handlers(
        tester->connection, s_mqtt_on_interrupted, NULL, s_mqtt_on_resumed, NULL);

    aws_mqtt_client_connection_set_on_any_publish_handler(tester->connection, s_on_any_packet_received, tester);

    AWS_FATAL_ASSERT(aws_mqtt_client_connection_connect(tester->connection, conn_options) == AWS_OP_SUCCESS);

    return AWS_OP_SUCCESS;
}

static void s_cleanup_test(struct test_context *tester) {

    aws_mqtt_client_connection_release(tester->connection);
    aws_mqtt_client_release(tester->client);

    aws_client_bootstrap_release(tester->bootstrap);
    aws_host_resolver_release(tester->resolver);
    aws_event_loop_group_release(tester->el_group);

    aws_thread_join_all_managed();

    aws_mutex_clean_up(&tester->lock);
    aws_condition_variable_clean_up(&tester->signal);

    aws_mqtt_library_clean_up();
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    struct aws_allocator *allocator = aws_mem_tracer_new(aws_default_allocator(), NULL, AWS_MEMTRACE_BYTES, 0);

    struct test_context tester;
    AWS_ZERO_STRUCT(tester);

    struct aws_byte_cursor host_name_cur = aws_byte_cursor_from_string(s_hostname);

    struct aws_socket_options options;
    AWS_ZERO_STRUCT(options);
    options.connect_timeout_ms = 3000;
    options.type = AWS_SOCKET_STREAM;
    options.domain = AWS_SOCKET_IPV4;

    struct aws_mqtt_connection_options conn_options = {
        .host_name = host_name_cur,
        .port = 1883,
        .socket_options = &options,
        .tls_options = NULL,
        .client_id = s_client_id,
        .keep_alive_time_secs = 0,
        .ping_timeout_ms = 0,
        .on_connection_complete = s_mqtt_on_connection_complete,
        .user_data = &tester,
        .clean_session = true,
    };

    AWS_FATAL_ASSERT(AWS_OP_SUCCESS == s_initialize_test(&tester, allocator, &conn_options));

    /* Wait for connack */
    s_wait_on_tester_predicate(&tester, s_is_connection_complete_pred);

    printf("1 done\n");

    struct aws_byte_cursor subscribe_topic_cur = aws_byte_cursor_from_string(s_subscribe_topic);

    /* Populate the payload */
    struct aws_byte_cursor payload_cur = aws_byte_cursor_from_array(s_payload, PAYLOAD_LEN);

    aws_mqtt_client_connection_publish(
        tester.connection,
        &subscribe_topic_cur,
        AWS_MQTT_QOS_EXACTLY_ONCE,
        true,
        &payload_cur,
        &s_mqtt_on_puback,
        &tester);

    /* Wait for puback */
    s_wait_on_tester_predicate(&tester, s_received_pub_ack_pred);

    printf("2 done\n");

    aws_mqtt_client_connection_disconnect(tester.connection, s_mqtt_on_disconnect, &tester);

    /* Wait for disconnack */
    s_wait_on_tester_predicate(&tester, s_on_disconnect_received);

    printf("3 done\n");

    aws_mutex_lock(&tester.lock);
    tester.connection_complete = false;
    tester.on_disconnect_received = false;
    aws_mutex_unlock(&tester.lock);

    AWS_FATAL_ASSERT(AWS_OP_SUCCESS == aws_mqtt_client_connection_connect(tester.connection, &conn_options));

    /* Wait for connack */
    s_wait_on_tester_predicate(&tester, s_is_connection_complete_pred);

    printf("1 done\n");

    /* Subscribe (no on_suback, the on_message received will trigger the cv) */
    aws_mqtt_client_connection_subscribe(
        tester.connection,
        &subscribe_topic_cur,
        AWS_MQTT_QOS_EXACTLY_ONCE,
        &s_on_packet_received,
        &tester,
        NULL,
        NULL,
        NULL);

    /* Wait for PUBLISH */
    s_wait_on_tester_predicate(&tester, s_retained_and_any_publish_pred);

    printf("2 done\n");

    struct aws_byte_cursor topic_filter =
        aws_byte_cursor_from_array(aws_string_bytes(s_subscribe_topic), s_subscribe_topic->len);
    aws_mqtt_client_connection_unsubscribe(tester.connection, &topic_filter, &s_mqtt_on_unsuback, &tester);

    /* Wait for UNSUBACK */
    s_wait_on_tester_predicate(&tester, s_received_unsub_ack_pred);

    printf("3 done\n");

    sleep(4);

    aws_mqtt_client_connection_disconnect(tester.connection, s_mqtt_on_disconnect, &tester);

    s_wait_on_tester_predicate(&tester, s_on_disconnect_received);

    s_cleanup_test(&tester);

    AWS_FATAL_ASSERT(0 == aws_mem_tracer_count(allocator));
    allocator = aws_mem_tracer_destroy(allocator);

    return AWS_OP_SUCCESS;
}
