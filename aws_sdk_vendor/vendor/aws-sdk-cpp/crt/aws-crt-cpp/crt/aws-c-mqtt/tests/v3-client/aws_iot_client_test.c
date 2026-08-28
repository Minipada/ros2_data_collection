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
#include <aws/io/logging.h>
#include <aws/io/socket.h>
#include <aws/io/socket_channel_handler.h>
#include <aws/io/tls_channel_handler.h>

#include <aws/common/condition_variable.h>
#include <aws/common/device_random.h>
#include <aws/common/mutex.h>
#include <aws/common/string.h>
#include <aws/common/thread.h>
#include <aws/common/uuid.h>

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#    include <windows.h>
#    define sleep Sleep
#else
#    include <unistd.h>
#endif

const char s_client_id_prefix[] = "sdk-c-v2-";
AWS_STATIC_STRING_FROM_LITERAL(s_subscribe_topic, "sdk/test/c");

enum { PUBLISHES = 20 };

enum { PAYLOAD_LEN = 200 };
static uint8_t s_payload[PAYLOAD_LEN];

static uint8_t s_will_payload[] = "The client has gone offline!";
enum { WILL_PAYLOAD_LEN = sizeof(s_will_payload) };

struct test_context {
    struct aws_allocator *allocator;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    struct aws_logger logger;
    struct aws_tls_ctx *tls_ctx;
    struct aws_event_loop_group *el_group;
    struct aws_host_resolver *resolver;
    struct aws_client_bootstrap *bootstrap;
    struct aws_mqtt_client *client;
    struct aws_mqtt_client_connection *connection;
    struct aws_tls_connection_options tls_connection_options;

    size_t pubacks_gotten;
    size_t packets_gotten;

    bool received_on_connection_complete;
    bool received_on_suback;
    bool received_on_disconnect;
};

static void s_on_puback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;
    (void)error_code;

    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);

    struct test_context *tester = userdata;

    aws_mutex_lock(&tester->lock);
    ++tester->pubacks_gotten;
    aws_mutex_unlock(&tester->lock);
}

static void s_on_packet_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata) {

    (void)connection;
    (void)topic;
    (void)dup;
    (void)qos;
    (void)retain;

    AWS_FATAL_ASSERT(payload->len == PAYLOAD_LEN);
    AWS_FATAL_ASSERT(0 == memcmp(payload->ptr, s_payload, PAYLOAD_LEN));

    bool notify = false;
    struct test_context *tester = userdata;

    aws_mutex_lock(&tester->lock);
    ++tester->packets_gotten;
    if (tester->packets_gotten == PUBLISHES) {
        notify = true;
    }
    aws_mutex_unlock(&tester->lock);

    if (notify) {
        aws_condition_variable_notify_one(&tester->signal);
    }
}

static bool s_all_packets_received_cond(void *userdata) {
    struct test_context *tester = userdata;
    return tester->packets_gotten == PUBLISHES;
}

static void s_mqtt_on_connection_complete(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {

    (void)connection;
    (void)error_code;
    (void)return_code;
    (void)session_present;

    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);
    AWS_FATAL_ASSERT(return_code == AWS_MQTT_CONNECT_ACCEPTED);
    AWS_FATAL_ASSERT(session_present == false);

    struct test_context *tester = userdata;

    aws_mutex_lock(&tester->lock);
    tester->received_on_connection_complete = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_mqtt_on_suback(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_byte_cursor *topic,
    enum aws_mqtt_qos qos,
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;
    (void)topic;
    (void)qos;
    (void)error_code;

    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);
    AWS_FATAL_ASSERT(qos != AWS_MQTT_QOS_FAILURE);

    struct test_context *tester = userdata;

    aws_mutex_lock(&tester->lock);
    tester->received_on_suback = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_on_connection_interrupted(struct aws_mqtt_client_connection *connection, int error_code, void *userdata) {

    (void)connection;
    (void)userdata;
    printf("CONNECTION INTERRUPTED error_code=%d\n", error_code);
}

static void s_on_resubscribed(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_array_list *topic_subacks, /* contains aws_mqtt_topic_subscription pointers */
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;
    (void)userdata;

    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);

    size_t num_topics = aws_array_list_length(topic_subacks);
    printf("RESUBSCRIBE_COMPLETE. error_code=%d num_topics=%zu\n", error_code, num_topics);
    for (size_t i = 0; i < num_topics; ++i) {
        struct aws_mqtt_topic_subscription sub_i;
        aws_array_list_get_at(topic_subacks, &sub_i, i);
        printf("  topic=" PRInSTR " qos=%d\n", AWS_BYTE_CURSOR_PRI(sub_i.topic), sub_i.qos);
        AWS_FATAL_ASSERT(sub_i.qos != AWS_MQTT_QOS_FAILURE);
    }
}

static void s_on_connection_resumed(
    struct aws_mqtt_client_connection *connection,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {

    (void)connection;
    (void)userdata;

    printf("CONNECTION RESUMED return_code=%d session_present=%d\n", return_code, session_present);
    if (!session_present) {
        printf("RESUBSCRIBING...");
        uint16_t packet_id = aws_mqtt_resubscribe_existing_topics(connection, s_on_resubscribed, NULL);
        AWS_FATAL_ASSERT(packet_id);
    }
}

static void s_mqtt_on_disconnect(struct aws_mqtt_client_connection *connection, void *userdata) {
    (void)connection;

    struct test_context *tester = userdata;

    aws_mutex_lock(&tester->lock);
    tester->received_on_disconnect = true;
    aws_mutex_unlock(&tester->lock);
    aws_condition_variable_notify_one(&tester->signal);
}

static void s_wait_on_tester_predicate(struct test_context *tester, bool (*predicate)(void *)) {
    aws_mutex_lock(&tester->lock);
    aws_condition_variable_wait_pred(&tester->signal, &tester->lock, predicate, tester);
    aws_mutex_unlock(&tester->lock);
}

static bool s_received_on_disconnect_pred(void *user_data) {
    struct test_context *tester = user_data;
    return tester->received_on_disconnect;
}

static bool s_received_on_suback_pred(void *user_data) {
    struct test_context *tester = user_data;
    return tester->received_on_suback;
}

static bool s_connection_complete_pred(void *user_data) {
    struct test_context *tester = user_data;
    return tester->received_on_connection_complete;
}

int s_initialize_test(
    struct test_context *tester,
    struct aws_allocator *allocator,
    const char *cert,
    const char *private_key,
    const char *root_ca,
    const char *endpoint) {
    aws_mqtt_library_init(allocator);

    struct aws_logger_standard_options logger_options = {
        .level = AWS_LL_TRACE,
        .file = stdout,
    };

    aws_logger_init_standard(&tester->logger, allocator, &logger_options);
    aws_logger_set(&tester->logger);

    tester->allocator = allocator;
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

    struct aws_tls_ctx_options tls_ctx_opt;
    AWS_FATAL_ASSERT(
        AWS_OP_SUCCESS == aws_tls_ctx_options_init_client_mtls_from_path(&tls_ctx_opt, allocator, cert, private_key));
    AWS_FATAL_ASSERT(AWS_OP_SUCCESS == aws_tls_ctx_options_set_alpn_list(&tls_ctx_opt, "x-amzn-mqtt-ca"));
    AWS_FATAL_ASSERT(
        AWS_OP_SUCCESS == aws_tls_ctx_options_override_default_trust_store_from_path(&tls_ctx_opt, NULL, root_ca));

    tester->tls_ctx = aws_tls_client_ctx_new(allocator, &tls_ctx_opt);
    AWS_FATAL_ASSERT(tester->tls_ctx != NULL);

    aws_tls_ctx_options_clean_up(&tls_ctx_opt);

    aws_tls_connection_options_init_from_ctx(&tester->tls_connection_options, tester->tls_ctx);

    tester->client = aws_mqtt_client_new(allocator, tester->bootstrap);
    tester->connection = aws_mqtt_client_connection_new(tester->client);

    struct aws_socket_options socket_options;
    AWS_ZERO_STRUCT(socket_options);
    socket_options.connect_timeout_ms = 3000;
    socket_options.type = AWS_SOCKET_STREAM;
    socket_options.domain = AWS_SOCKET_IPV6;

    struct aws_byte_cursor host_name_cur = aws_byte_cursor_from_c_str(endpoint);

    /* Generate a random clientid */
    char client_id[128];
    struct aws_byte_buf client_id_buf = aws_byte_buf_from_empty_array(client_id, AWS_ARRAY_SIZE(client_id));

    aws_byte_buf_write(&client_id_buf, (const uint8_t *)s_client_id_prefix, AWS_ARRAY_SIZE(s_client_id_prefix));

    struct aws_uuid uuid;
    aws_uuid_init(&uuid);
    aws_uuid_to_str(&uuid, &client_id_buf);

    struct aws_byte_cursor client_id_cur = aws_byte_cursor_from_buf(&client_id_buf);

    struct aws_mqtt_connection_options conn_options = {
        .host_name = host_name_cur,
        .port = 8883,
        .socket_options = &socket_options,
        .tls_options = &tester->tls_connection_options,
        .client_id = client_id_cur,
        .keep_alive_time_secs = 0,
        .ping_timeout_ms = 0,
        .on_connection_complete = s_mqtt_on_connection_complete,
        .user_data = tester,
        .clean_session = true,
    };

    AWS_FATAL_ASSERT(
        AWS_OP_SUCCESS == aws_mqtt_client_connection_set_connection_interruption_handlers(
                              tester->connection, s_on_connection_interrupted, NULL, s_on_connection_resumed, NULL));

    struct aws_byte_cursor subscribe_topic_cur = aws_byte_cursor_from_string(s_subscribe_topic);
    struct aws_byte_cursor will_cur = aws_byte_cursor_from_array(s_will_payload, WILL_PAYLOAD_LEN);
    aws_mqtt_client_connection_set_will(tester->connection, &subscribe_topic_cur, 1, false, &will_cur);

    aws_mqtt_client_connection_connect(tester->connection, &conn_options);

    return AWS_OP_SUCCESS;
}

static void s_cleanup_test(struct test_context *tester) {
    aws_tls_connection_options_clean_up(&tester->tls_connection_options);

    aws_mqtt_client_connection_release(tester->connection);
    aws_mqtt_client_release(tester->client);

    aws_tls_ctx_release(tester->tls_ctx);

    aws_client_bootstrap_release(tester->bootstrap);

    aws_host_resolver_release(tester->resolver);
    aws_event_loop_group_release(tester->el_group);

    aws_thread_join_all_managed();

    aws_logger_clean_up(&tester->logger);

    aws_mutex_clean_up(&tester->lock);
    aws_condition_variable_clean_up(&tester->signal);

    aws_mqtt_library_clean_up();
}

int main(int argc, char **argv) {

    if (argc < 5) {
        printf(
            "4 args required, only %d passed. Usage:\n"
            "aws-c-mqtt-iot-client [endpoint] [certificate] [private_key] [root_ca]\n",
            argc - 1);
        return 1;
    }

    const char *endpoint = argv[1];
    const char *cert = argv[2];
    const char *private_key = argv[3];
    const char *root_ca = argv[4];

    struct aws_allocator *allocator = aws_mem_tracer_new(aws_default_allocator(), NULL, AWS_MEMTRACE_BYTES, 0);

    struct test_context tester;
    AWS_ZERO_STRUCT(tester);

    AWS_FATAL_ASSERT(s_initialize_test(&tester, allocator, cert, private_key, root_ca, endpoint) == AWS_OP_SUCCESS);

    struct aws_byte_cursor subscribe_topic_cur = aws_byte_cursor_from_string(s_subscribe_topic);

    s_wait_on_tester_predicate(&tester, s_connection_complete_pred);

    aws_mqtt_client_connection_subscribe(
        tester.connection,
        &subscribe_topic_cur,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        &s_on_packet_received,
        &tester,
        NULL,
        s_mqtt_on_suback,
        &tester);

    s_wait_on_tester_predicate(&tester, s_received_on_suback_pred);

    /* Populate the payload */
    struct aws_byte_buf payload_buf = aws_byte_buf_from_empty_array(s_payload, PAYLOAD_LEN);
    aws_device_random_buffer(&payload_buf);
    struct aws_byte_cursor payload_cur = aws_byte_cursor_from_buf(&payload_buf);

    for (int i = 0; i < PUBLISHES; ++i) {
        aws_mqtt_client_connection_publish(
            tester.connection,
            &subscribe_topic_cur,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            &payload_cur,
            &s_on_puback,
            &tester);

        /* Keep the service endpoint from throttling the connection */
        if (i != 0 && i % 100 == 0) {
            sleep(1);
        }
    }

    s_wait_on_tester_predicate(&tester, s_all_packets_received_cond);

    AWS_FATAL_ASSERT(PUBLISHES == tester.packets_gotten);

    aws_mqtt_client_connection_disconnect(tester.connection, s_mqtt_on_disconnect, &tester);

    s_wait_on_tester_predicate(&tester, s_received_on_disconnect_pred);

    s_cleanup_test(&tester);

    AWS_FATAL_ASSERT(0 == aws_mem_tracer_count(allocator));
    allocator = aws_mem_tracer_destroy(allocator);

    return AWS_OP_SUCCESS;
}
