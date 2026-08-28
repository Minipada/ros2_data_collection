/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt5_testing_utils.h"

#include <aws/common/clock.h>
#include <aws/common/string.h>
#include <aws/http/websocket.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/v5/mqtt5_utils.h>
#include <aws/mqtt/v5/mqtt5_client.h>
#include <aws/mqtt/v5/mqtt5_listener.h>

#include <aws/testing/aws_test_harness.h>

#include <math.h>
/* The delay margin accounts for system-level latencies, particularly when scheduling tasks on dispatch queues. In
 * typical test environments, we observe delays of around 150–170ms before the task begins execution. To ensure
 * stability, we define a margin of 200ms (AWS_MQTT5_TESTING_DELAY_NS). If related tests fail, it may indicate this
 * margin is insufficient—consider increasing the value. */
#define AWS_MQTT5_TESTING_DELAY_NS 200 * 1000 * 1000 /*200ms*/

static bool s_is_within_percentage_of(uint64_t expected_time, uint64_t actual_time, double percentage) {
    double actual_percent = 1.0 - (double)actual_time / (double)expected_time;
    return fabs(actual_percent) <= percentage;
}

/* The function is used to validate the time interval for the expected and actual time. We used a user determined
 * percentage for margin and a minimal of AWS_MQTT5_TESTING_DELAY_NS considering the system delay. */
static bool s_validate_time_interval(uint64_t expected_time, uint64_t actual_time, double percentage) {
    uint64_t delta = actual_time > expected_time ? actual_time - expected_time : expected_time - actual_time;
    return delta < AWS_MQTT5_TESTING_DELAY_NS || s_is_within_percentage_of(expected_time, actual_time, percentage);
}

static bool s_check_time_lower_bound(uint64_t expected_time, uint64_t actual_time) {
    return actual_time >= expected_time;
}

int aws_mqtt5_mock_server_handle_connect_always_succeed(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

static int s_aws_mqtt5_mock_server_handle_pingreq_always_respond(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PINGRESP, NULL);
}

static int s_aws_mqtt5_mock_server_handle_disconnect(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)connection;
    (void)user_data;

    return AWS_OP_SUCCESS;
}

void s_lifecycle_event_callback(const struct aws_mqtt5_client_lifecycle_event *event) {
    (void)event;
}

void s_publish_received_callback(const struct aws_mqtt5_packet_publish_view *publish, void *user_data) {
    (void)publish;
    (void)user_data;
}

AWS_STRING_FROM_LITERAL(g_default_client_id, "HelloWorld");

void aws_mqtt5_client_test_init_default_options(struct mqtt5_client_test_options *test_options) {

    struct aws_mqtt5_client_topic_alias_options local_topic_aliasing_options = {
        .outbound_topic_alias_behavior = AWS_MQTT5_COTABT_DISABLED,
    };

    test_options->topic_aliasing_options = local_topic_aliasing_options;

    struct aws_mqtt5_packet_connect_view local_connect_options = {
        .keep_alive_interval_seconds = 30,
        .client_id = aws_byte_cursor_from_string(g_default_client_id),
        .clean_start = true,
    };

    test_options->connect_options = local_connect_options;

    struct aws_mqtt5_client_options local_client_options = {
        .connect_options = &test_options->connect_options,
        .session_behavior = AWS_MQTT5_CSBT_CLEAN,
        .lifecycle_event_handler = s_lifecycle_event_callback,
        .lifecycle_event_handler_user_data = NULL,
        .max_reconnect_delay_ms = 120000,
        .min_connected_time_to_reset_reconnect_delay_ms = 30000,
        .min_reconnect_delay_ms = 1000,
        .ping_timeout_ms = 10000,
        .publish_received_handler = s_publish_received_callback,
        .ack_timeout_seconds = 0,
        .topic_aliasing_options = &test_options->topic_aliasing_options,
    };

    test_options->client_options = local_client_options;

    struct aws_mqtt5_mock_server_vtable local_server_function_table = {
        .packet_handlers = {
            NULL,                                                   /* RESERVED = 0 */
            &aws_mqtt5_mock_server_handle_connect_always_succeed,   /* CONNECT */
            NULL,                                                   /* CONNACK */
            NULL,                                                   /* PUBLISH */
            NULL,                                                   /* PUBACK */
            NULL,                                                   /* PUBREC */
            NULL,                                                   /* PUBREL */
            NULL,                                                   /* PUBCOMP */
            NULL,                                                   /* SUBSCRIBE */
            NULL,                                                   /* SUBACK */
            NULL,                                                   /* UNSUBSCRIBE */
            NULL,                                                   /* UNSUBACK */
            &s_aws_mqtt5_mock_server_handle_pingreq_always_respond, /* PINGREQ */
            NULL,                                                   /* PINGRESP */
            &s_aws_mqtt5_mock_server_handle_disconnect,             /* DISCONNECT */
            NULL                                                    /* AUTH */
        }};

    test_options->server_function_table = local_server_function_table;
}

static int s_aws_mqtt5_client_test_init_default_connect_storage(
    struct aws_mqtt5_packet_connect_storage *storage,
    struct aws_allocator *allocator) {

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds = 30,
        .client_id = aws_byte_cursor_from_string(g_default_client_id),
        .clean_start = true,
    };

    return aws_mqtt5_packet_connect_storage_init(storage, allocator, &connect_view);
}

static int s_aws_mqtt5_client_test_init_default_disconnect_storage(
    struct aws_mqtt5_packet_disconnect_storage *storage,
    struct aws_allocator *allocator) {
    struct aws_mqtt5_packet_disconnect_view disconnect_view = {
        .reason_code = AWS_MQTT5_DRC_NORMAL_DISCONNECTION,
    };

    return aws_mqtt5_packet_disconnect_storage_init(storage, allocator, &disconnect_view);
}

static bool s_last_life_cycle_event_is(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    enum aws_mqtt5_client_lifecycle_event_type event_type) {
    size_t event_count = aws_array_list_length(&test_fixture->lifecycle_events);
    if (event_count == 0) {
        return false;
    }

    struct aws_mqtt5_lifecycle_event_record *record = NULL;
    aws_array_list_get_at(&test_fixture->lifecycle_events, &record, event_count - 1);

    return record->event.event_type == event_type;
}

static bool s_last_mock_server_packet_received_is(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    enum aws_mqtt5_packet_type packet_type) {
    size_t packet_count = aws_array_list_length(&test_fixture->server_received_packets);
    if (packet_count == 0) {
        return false;
    }

    struct aws_mqtt5_mock_server_packet_record *packet = NULL;
    aws_array_list_get_at_ptr(&test_fixture->server_received_packets, (void **)&packet, packet_count - 1);

    return packet_type == packet->packet_type;
}

static bool s_last_mock_server_packet_received_is_disconnect(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;

    return s_last_mock_server_packet_received_is(test_fixture, AWS_MQTT5_PT_DISCONNECT);
}

static void s_wait_for_mock_server_to_receive_disconnect_packet(
    struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_last_mock_server_packet_received_is_disconnect, test_context);
    aws_mutex_unlock(&test_context->lock);
}

static bool s_last_lifecycle_event_is_connected(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;

    return s_last_life_cycle_event_is(test_fixture, AWS_MQTT5_CLET_CONNECTION_SUCCESS);
}

void aws_wait_for_connected_lifecycle_event(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_last_lifecycle_event_is_connected, test_context);
    aws_mutex_unlock(&test_context->lock);
}

static bool s_last_lifecycle_event_is_stopped(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;

    return s_last_life_cycle_event_is(test_fixture, AWS_MQTT5_CLET_STOPPED);
}

void aws_wait_for_stopped_lifecycle_event(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_last_lifecycle_event_is_stopped, test_context);
    aws_mutex_unlock(&test_context->lock);
}

static bool s_has_lifecycle_event(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    enum aws_mqtt5_client_lifecycle_event_type event_type) {

    size_t record_count = aws_array_list_length(&test_fixture->lifecycle_events);
    for (size_t i = 0; i < record_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *record = NULL;
        aws_array_list_get_at(&test_fixture->lifecycle_events, &record, i);
        if (record->event.event_type == event_type) {
            return true;
        }
    }

    return false;
}

static bool s_has_connection_failure_event(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;

    return s_has_lifecycle_event(test_fixture, AWS_MQTT5_CLET_CONNECTION_FAILURE);
}

static void s_wait_for_connection_failure_lifecycle_event(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_has_connection_failure_event, test_context);
    aws_mutex_unlock(&test_context->lock);
}

static bool s_has_disconnect_event(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;

    return s_has_lifecycle_event(test_fixture, AWS_MQTT5_CLET_DISCONNECTION);
}

static void s_wait_for_disconnection_lifecycle_event(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(&test_context->signal, &test_context->lock, s_has_disconnect_event, test_context);
    aws_mutex_unlock(&test_context->lock);
}

static bool s_disconnect_completion_invoked(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;

    return test_fixture->disconnect_completion_callback_invoked;
}

static void s_on_disconnect_completion(int error_code, void *user_data) {
    (void)error_code;

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = user_data;

    aws_mutex_lock(&test_fixture->lock);
    test_fixture->disconnect_completion_callback_invoked = true;
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

static void s_wait_for_disconnect_completion(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_disconnect_completion_invoked, test_context);
    aws_mutex_unlock(&test_context->lock);
}

int aws_verify_client_state_sequence(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    enum aws_mqtt5_client_state *expected_states,
    size_t expected_states_count) {
    aws_mutex_lock(&test_context->lock);

    size_t actual_states_count = aws_array_list_length(&test_context->client_states);
    ASSERT_TRUE(actual_states_count >= expected_states_count);

    for (size_t i = 0; i < expected_states_count; ++i) {
        enum aws_mqtt5_client_state state = AWS_MCS_STOPPED;
        aws_array_list_get_at(&test_context->client_states, &state, i);

        ASSERT_INT_EQUALS(expected_states[i], state);
    }

    aws_mutex_unlock(&test_context->lock);

    return AWS_OP_SUCCESS;
}

static int s_verify_simple_lifecycle_event_sequence(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    struct aws_mqtt5_client_lifecycle_event *expected_events,
    size_t expected_events_count) {
    aws_mutex_lock(&test_context->lock);

    size_t actual_events_count = aws_array_list_length(&test_context->lifecycle_events);
    ASSERT_TRUE(actual_events_count >= expected_events_count);

    for (size_t i = 0; i < expected_events_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *lifecycle_event = NULL;
        aws_array_list_get_at(&test_context->lifecycle_events, &lifecycle_event, i);

        struct aws_mqtt5_client_lifecycle_event *expected_event = &expected_events[i];
        ASSERT_INT_EQUALS(expected_event->event_type, lifecycle_event->event.event_type);
        ASSERT_INT_EQUALS(expected_event->error_code, lifecycle_event->event.error_code);
    }

    aws_mutex_unlock(&test_context->lock);

    return AWS_OP_SUCCESS;
}

int aws_verify_received_packet_sequence(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    struct aws_mqtt5_mock_server_packet_record *expected_packets,
    size_t expected_packets_count) {
    aws_mutex_lock(&test_context->lock);

    size_t actual_packets_count = aws_array_list_length(&test_context->server_received_packets);
    ASSERT_TRUE(actual_packets_count >= expected_packets_count);

    for (size_t i = 0; i < expected_packets_count; ++i) {
        struct aws_mqtt5_mock_server_packet_record *actual_packet = NULL;
        aws_array_list_get_at_ptr(&test_context->server_received_packets, (void **)&actual_packet, i);

        struct aws_mqtt5_mock_server_packet_record *expected_packet = &expected_packets[i];

        ASSERT_INT_EQUALS(expected_packet->packet_type, actual_packet->packet_type);

        /* a NULL storage means we don't care about verifying it on a field-by-field basis */
        if (expected_packet->packet_storage != NULL) {
            ASSERT_TRUE(aws_mqtt5_client_test_are_packets_equal(
                expected_packet->packet_type, expected_packet->packet_storage, actual_packet->packet_storage));
        }
    }

    aws_mutex_unlock(&test_context->lock);

    return AWS_OP_SUCCESS;
}

/*
 * Basic successful connect/disconnect test.  We check expected lifecycle events, internal client state changes,
 * and server received packets.
 */
static int s_mqtt5_client_direct_connect_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_disconnect_view disconnect_options = {
        .reason_code = AWS_MQTT5_DRC_DISCONNECT_WITH_WILL_MESSAGE,
    };

    struct aws_mqtt5_disconnect_completion_options completion_options = {
        .completion_callback = s_on_disconnect_completion,
        .completion_user_data = &test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, &disconnect_options, &completion_options));

    aws_wait_for_stopped_lifecycle_event(&test_context);
    s_wait_for_disconnect_completion(&test_context);
    s_wait_for_mock_server_to_receive_disconnect_packet(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_SUCCESS,
        },
        {
            .event_type = AWS_MQTT5_CLET_DISCONNECTION,
            .error_code = AWS_ERROR_MQTT5_USER_REQUESTED_STOP,
        },
        {
            .event_type = AWS_MQTT5_CLET_STOPPED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CONNECTED,
        AWS_MCS_CLEAN_DISCONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
        AWS_MCS_STOPPED,
    };

    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    struct aws_mqtt5_packet_connect_storage expected_connect_storage;
    ASSERT_SUCCESS(s_aws_mqtt5_client_test_init_default_connect_storage(&expected_connect_storage, allocator));

    struct aws_mqtt5_packet_disconnect_storage expected_disconnect_storage;
    ASSERT_SUCCESS(s_aws_mqtt5_client_test_init_default_disconnect_storage(&expected_disconnect_storage, allocator));
    expected_disconnect_storage.storage_view.reason_code = AWS_MQTT5_DRC_DISCONNECT_WITH_WILL_MESSAGE;

    struct aws_mqtt5_mock_server_packet_record expected_packets[] = {
        {
            .packet_type = AWS_MQTT5_PT_CONNECT,
            .packet_storage = &expected_connect_storage,
        },
        {
            .packet_type = AWS_MQTT5_PT_DISCONNECT,
            .packet_storage = &expected_disconnect_storage,
        },
    };
    ASSERT_SUCCESS(
        aws_verify_received_packet_sequence(&test_context, expected_packets, AWS_ARRAY_SIZE(expected_packets)));

    aws_mqtt5_packet_connect_storage_clean_up(&expected_connect_storage);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_direct_connect_success, s_mqtt5_client_direct_connect_success_fn)

/*
 * Connection failure test infrastructure.  Supplied callbacks are used to modify the way in which the connection
 * establishment fails.
 */
static int s_mqtt5_client_simple_failure_test_fn(
    struct aws_allocator *allocator,
    void (*change_client_test_config_fn)(struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *config),
    void (*change_client_vtable_fn)(struct aws_mqtt5_client_vtable *)) {

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    if (change_client_test_config_fn != NULL) {
        (*change_client_test_config_fn)(&test_fixture_options);
    }

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    struct aws_mqtt5_client_vtable vtable = *client->vtable;
    if (change_client_vtable_fn != NULL) {
        (*change_client_vtable_fn)(&vtable);
        aws_mqtt5_client_set_vtable(client, &vtable);
    }

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    s_wait_for_connection_failure_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_INVALID_STATE,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_PENDING_RECONNECT,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_test_synchronous_socket_channel_failure_fn(
    struct aws_socket_channel_bootstrap_options *options) {
    (void)options;

    return aws_raise_error(AWS_ERROR_INVALID_STATE);
}

static void s_change_client_vtable_synchronous_direct_failure(struct aws_mqtt5_client_vtable *vtable) {
    vtable->client_bootstrap_new_socket_channel_fn = s_mqtt5_client_test_synchronous_socket_channel_failure_fn;
}

/* Connection failure test where direct MQTT channel establishment fails synchronously */
static int s_mqtt5_client_direct_connect_sync_channel_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(
        s_mqtt5_client_simple_failure_test_fn(allocator, NULL, s_change_client_vtable_synchronous_direct_failure));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_direct_connect_sync_channel_failure, s_mqtt5_client_direct_connect_sync_channel_failure_fn)

struct socket_channel_failure_wrapper {
    struct aws_socket_channel_bootstrap_options bootstrap_options;
    struct aws_task task;
};

static struct socket_channel_failure_wrapper s_socket_channel_failure_wrapper;

void s_socket_channel_async_failure_task_fn(struct aws_task *task, void *arg, enum aws_task_status status) {
    (void)task;
    if (status != AWS_TASK_STATUS_RUN_READY) {
        return;
    }

    struct socket_channel_failure_wrapper *wrapper = arg;
    struct aws_socket_channel_bootstrap_options *options = &wrapper->bootstrap_options;

    (*wrapper->bootstrap_options.setup_callback)(options->bootstrap, AWS_ERROR_INVALID_STATE, NULL, options->user_data);
}

static int s_mqtt5_client_test_asynchronous_socket_channel_failure_fn(
    struct aws_socket_channel_bootstrap_options *options) {
    aws_task_init(
        &s_socket_channel_failure_wrapper.task,
        s_socket_channel_async_failure_task_fn,
        &s_socket_channel_failure_wrapper,
        "asynchronous_socket_channel_failure");
    s_socket_channel_failure_wrapper.bootstrap_options = *options;

    struct aws_mqtt5_client *client = options->user_data;
    aws_event_loop_schedule_task_now(client->loop, &s_socket_channel_failure_wrapper.task);

    return AWS_OP_SUCCESS;
}

static void s_change_client_vtable_asynchronous_direct_failure(struct aws_mqtt5_client_vtable *vtable) {
    vtable->client_bootstrap_new_socket_channel_fn = s_mqtt5_client_test_asynchronous_socket_channel_failure_fn;
}

/* Connection failure test where direct MQTT channel establishment fails asynchronously */
static int s_mqtt5_client_direct_connect_async_channel_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(
        s_mqtt5_client_simple_failure_test_fn(allocator, NULL, s_change_client_vtable_asynchronous_direct_failure));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_direct_connect_async_channel_failure, s_mqtt5_client_direct_connect_async_channel_failure_fn)

static int s_mqtt5_client_test_synchronous_websocket_failure_fn(
    const struct aws_websocket_client_connection_options *options) {
    (void)options;

    return aws_raise_error(AWS_ERROR_INVALID_STATE);
}

static void s_change_client_vtable_synchronous_websocket_failure(struct aws_mqtt5_client_vtable *vtable) {
    vtable->websocket_connect_fn = s_mqtt5_client_test_synchronous_websocket_failure_fn;
}

static void s_mqtt5_client_test_websocket_successful_transform(
    struct aws_http_message *request,
    void *user_data,
    aws_mqtt5_transform_websocket_handshake_complete_fn *complete_fn,
    void *complete_ctx) {
    (void)user_data;

    (*complete_fn)(request, AWS_ERROR_SUCCESS, complete_ctx);
}

static void s_change_client_options_to_websockets(struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *config) {
    config->client_options->websocket_handshake_transform = s_mqtt5_client_test_websocket_successful_transform;
}

/* Connection failure test where websocket MQTT channel establishment fails synchronously */
static int s_mqtt5_client_websocket_connect_sync_channel_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_mqtt5_client_simple_failure_test_fn(
        allocator, s_change_client_options_to_websockets, s_change_client_vtable_synchronous_websocket_failure));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_websocket_connect_sync_channel_failure,
    s_mqtt5_client_websocket_connect_sync_channel_failure_fn)

struct websocket_channel_failure_wrapper {
    struct aws_websocket_client_connection_options websocket_options;
    struct aws_task task;
};

static struct websocket_channel_failure_wrapper s_websocket_channel_failure_wrapper;

void s_websocket_channel_async_failure_task_fn(struct aws_task *task, void *arg, enum aws_task_status status) {
    (void)task;
    if (status != AWS_TASK_STATUS_RUN_READY) {
        return;
    }

    struct websocket_channel_failure_wrapper *wrapper = arg;
    struct aws_websocket_client_connection_options *options = &wrapper->websocket_options;

    struct aws_websocket_on_connection_setup_data websocket_setup = {.error_code = AWS_ERROR_INVALID_STATE};
    (*wrapper->websocket_options.on_connection_setup)(&websocket_setup, options->user_data);
}

static int s_mqtt5_client_test_asynchronous_websocket_failure_fn(
    const struct aws_websocket_client_connection_options *options) {
    aws_task_init(
        &s_websocket_channel_failure_wrapper.task,
        s_websocket_channel_async_failure_task_fn,
        &s_websocket_channel_failure_wrapper,
        "asynchronous_websocket_channel_failure");
    s_websocket_channel_failure_wrapper.websocket_options = *options;

    struct aws_mqtt5_client *client = options->user_data;
    aws_event_loop_schedule_task_now(client->loop, &s_websocket_channel_failure_wrapper.task);

    return AWS_OP_SUCCESS;
}

static void s_change_client_vtable_asynchronous_websocket_failure(struct aws_mqtt5_client_vtable *vtable) {
    vtable->websocket_connect_fn = s_mqtt5_client_test_asynchronous_websocket_failure_fn;
}

/* Connection failure test where websocket MQTT channel establishment fails asynchronously */
static int s_mqtt5_client_websocket_connect_async_channel_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_mqtt5_client_simple_failure_test_fn(
        allocator, s_change_client_options_to_websockets, s_change_client_vtable_asynchronous_websocket_failure));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_websocket_connect_async_channel_failure,
    s_mqtt5_client_websocket_connect_async_channel_failure_fn)

static void s_mqtt5_client_test_websocket_failed_transform(
    struct aws_http_message *request,
    void *user_data,
    aws_mqtt5_transform_websocket_handshake_complete_fn *complete_fn,
    void *complete_ctx) {

    (void)user_data;

    (*complete_fn)(request, AWS_ERROR_INVALID_STATE, complete_ctx);
}

static void s_change_client_options_to_failed_websocket_transform(
    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *config) {
    config->client_options->websocket_handshake_transform = s_mqtt5_client_test_websocket_failed_transform;
}

/* Connection failure test where websocket MQTT channel establishment fails due to handshake transform failure */
static int s_mqtt5_client_websocket_connect_handshake_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(
        s_mqtt5_client_simple_failure_test_fn(allocator, s_change_client_options_to_failed_websocket_transform, NULL));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_websocket_connect_handshake_failure, s_mqtt5_client_websocket_connect_handshake_failure_fn)

int aws_mqtt5_mock_server_handle_connect_always_fail(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    connack_view.reason_code = AWS_MQTT5_CRC_BANNED;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

/* Connection failure test where overall connection fails due to a CONNACK error code */
static int s_mqtt5_client_direct_connect_connack_refusal_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_always_fail;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    s_wait_for_connection_failure_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_direct_connect_connack_refusal, s_mqtt5_client_direct_connect_connack_refusal_fn)

/* Connection failure test where overall connection fails because there's no response to the CONNECT packet */
static int s_mqtt5_client_direct_connect_connack_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* fast CONNACK timeout and don't response to the CONNECT packet */
    test_options.client_options.connack_timeout_ms = 2000;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] = NULL;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    s_wait_for_connection_failure_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_TIMEOUT,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_direct_connect_connack_timeout, s_mqtt5_client_direct_connect_connack_timeout_fn)

struct aws_mqtt5_server_disconnect_test_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
    bool disconnect_sent;
    bool connack_sent;
};

static void s_server_disconnect_service_fn(
    struct aws_mqtt5_server_mock_connection_context *mock_server,
    void *user_data) {

    struct aws_mqtt5_server_disconnect_test_context *test_context = user_data;
    if (test_context->disconnect_sent || !test_context->connack_sent) {
        return;
    }

    test_context->disconnect_sent = true;

    struct aws_mqtt5_packet_disconnect_view disconnect = {
        .reason_code = AWS_MQTT5_DRC_PACKET_TOO_LARGE,
    };

    aws_mqtt5_mock_server_send_packet(mock_server, AWS_MQTT5_PT_DISCONNECT, &disconnect);
}

static int s_aws_mqtt5_server_disconnect_on_connect(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {

    /*
     * We intercept the CONNECT in order to correctly set the connack_sent test state.  Otherwise we risk sometimes
     * sending the DISCONNECT before the CONNACK
     */
    int result = aws_mqtt5_mock_server_handle_connect_always_succeed(packet, connection, user_data);

    struct aws_mqtt5_server_disconnect_test_context *test_context = user_data;
    test_context->connack_sent = true;

    return result;
}

/* Connection test where we succeed and then the server sends a DISCONNECT */
static int s_mqtt5_client_direct_connect_from_server_disconnect_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* mock server sends a DISCONNECT packet back to the client after a successful CONNECTION establishment */
    test_options.server_function_table.service_task_fn = s_server_disconnect_service_fn;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] = s_aws_mqtt5_server_disconnect_on_connect;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    s_wait_for_disconnection_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_SUCCESS,
        },
        {
            .event_type = AWS_MQTT5_CLET_DISCONNECTION,
            .error_code = AWS_ERROR_MQTT5_DISCONNECT_RECEIVED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CONNECTED,
        AWS_MCS_CHANNEL_SHUTDOWN,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_direct_connect_from_server_disconnect,
    s_mqtt5_client_direct_connect_from_server_disconnect_fn)

struct aws_mqtt5_client_test_wait_for_n_context {
    size_t required_event_count;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
};

static size_t s_count_pingreqs(struct aws_mqtt5_client_mock_test_fixture *test_fixture) {
    size_t ping_count = 0;
    size_t packet_count = aws_array_list_length(&test_fixture->server_received_packets);
    for (size_t i = 0; i < packet_count; ++i) {
        struct aws_mqtt5_mock_server_packet_record *record = NULL;
        aws_array_list_get_at_ptr(&test_fixture->server_received_packets, (void **)&record, i);

        if (record->packet_type == AWS_MQTT5_PT_PINGREQ) {
            ping_count++;
        }
    }

    return ping_count;
}

static bool s_received_at_least_n_pingreqs(void *arg) {
    struct aws_mqtt5_client_test_wait_for_n_context *ping_context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = ping_context->test_fixture;

    size_t ping_count = s_count_pingreqs(test_fixture);

    return ping_count >= ping_context->required_event_count;
}

static void s_wait_for_n_pingreqs(struct aws_mqtt5_client_test_wait_for_n_context *ping_context) {

    struct aws_mqtt5_client_mock_test_fixture *test_context = ping_context->test_fixture;
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_received_at_least_n_pingreqs, ping_context);
    aws_mutex_unlock(&test_context->lock);
}

#define TEST_PING_INTERVAL_MS 2000

static int s_verify_ping_sequence_timing(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);

    uint64_t last_packet_time = 0;

    size_t packet_count = aws_array_list_length(&test_context->server_received_packets);
    for (size_t i = 0; i < packet_count; ++i) {
        struct aws_mqtt5_mock_server_packet_record *record = NULL;
        aws_array_list_get_at_ptr(&test_context->server_received_packets, (void **)&record, i);

        if (i == 0) {
            ASSERT_INT_EQUALS(record->packet_type, AWS_MQTT5_PT_CONNECT);
            last_packet_time = record->timestamp;
        } else {
            if (record->packet_type == AWS_MQTT5_PT_PINGREQ) {
                uint64_t time_delta_ns = record->timestamp - last_packet_time;
                uint64_t time_delta_millis =
                    aws_timestamp_convert(time_delta_ns, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_MILLIS, NULL);

                ASSERT_TRUE(s_is_within_percentage_of(TEST_PING_INTERVAL_MS, time_delta_millis, .3));

                last_packet_time = record->timestamp;
            }
        }
    }
    aws_mutex_unlock(&test_context->lock);

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_client_test_init_ping_test_connect_storage(
    struct aws_mqtt5_packet_connect_storage *storage,
    struct aws_allocator *allocator) {

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds =
            (uint16_t)aws_timestamp_convert(TEST_PING_INTERVAL_MS, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_SECS, NULL),
        .client_id = aws_byte_cursor_from_string(g_default_client_id),
        .clean_start = true,
    };

    return aws_mqtt5_packet_connect_storage_init(storage, allocator, &connect_view);
}

/*
 * Test that the client sends pings at regular intervals to the server
 *
 * This is a low-keep-alive variant of the basic success test that waits for N pingreqs to be received by the server
 * and validates the approximate time intervals between them.
 */
static int s_mqtt5_client_ping_sequence_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* fast keep alive in order keep tests reasonably short */
    uint16_t keep_alive_seconds =
        (uint16_t)aws_timestamp_convert(TEST_PING_INTERVAL_MS, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_SECS, NULL);
    test_options.connect_options.keep_alive_interval_seconds = keep_alive_seconds;

    /* faster ping timeout */
    test_options.client_options.ping_timeout_ms = 750;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_client_test_wait_for_n_context ping_context = {
        .required_event_count = 5,
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &ping_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);
    s_wait_for_n_pingreqs(&ping_context);

    /*
     * There's a really unpleasant race condition where we can stop the client so fast (based on the mock
     * server receiving PINGREQs that the mock server's socket gets closed underneath it as it is trying to
     * write the PINGRESP back to the client, which in turn triggers channel shutdown where no further data
     * is read from the socket, so we never see the DISCONNECT that the client actually sent.
     *
     * We're not able to wait on the PINGRESP because we have no insight into when it's received.  So for now,
     * we'll insert an artificial sleep before stopping the client.  We should try and come up with a more
     * elegant solution.
     */

    aws_thread_current_sleep(aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    struct aws_mqtt5_packet_disconnect_view disconnect_view = {
        .reason_code = AWS_MQTT5_DRC_NORMAL_DISCONNECTION,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, &disconnect_view, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);
    s_wait_for_mock_server_to_receive_disconnect_packet(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_SUCCESS,
        },
        {
            .event_type = AWS_MQTT5_CLET_DISCONNECTION,
            .error_code = AWS_ERROR_MQTT5_USER_REQUESTED_STOP,
        },
        {
            .event_type = AWS_MQTT5_CLET_STOPPED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CONNECTED,
        AWS_MCS_CLEAN_DISCONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
        AWS_MCS_STOPPED,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    struct aws_mqtt5_packet_connect_storage expected_connect_storage;
    ASSERT_SUCCESS(s_aws_mqtt5_client_test_init_ping_test_connect_storage(&expected_connect_storage, allocator));

    struct aws_mqtt5_packet_disconnect_storage expected_disconnect_storage;
    ASSERT_SUCCESS(s_aws_mqtt5_client_test_init_default_disconnect_storage(&expected_disconnect_storage, allocator));

    struct aws_mqtt5_mock_server_packet_record expected_packets[] = {
        {
            .packet_type = AWS_MQTT5_PT_CONNECT,
            .packet_storage = &expected_connect_storage,
        },
        {
            .packet_type = AWS_MQTT5_PT_PINGREQ,
        },
        {
            .packet_type = AWS_MQTT5_PT_PINGREQ,
        },
        {
            .packet_type = AWS_MQTT5_PT_PINGREQ,
        },
        {
            .packet_type = AWS_MQTT5_PT_PINGREQ,
        },
        {
            .packet_type = AWS_MQTT5_PT_PINGREQ,
        },
        {
            .packet_type = AWS_MQTT5_PT_DISCONNECT,
            .packet_storage = &expected_disconnect_storage,
        },
    };
    ASSERT_SUCCESS(
        aws_verify_received_packet_sequence(&test_context, expected_packets, AWS_ARRAY_SIZE(expected_packets)));

    ASSERT_SUCCESS(s_verify_ping_sequence_timing(&test_context));

    aws_mqtt5_packet_connect_storage_clean_up(&expected_connect_storage);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_ping_sequence, s_mqtt5_client_ping_sequence_fn)

/*
 * Test that sending other data to the server pushes out the client ping timer
 *
 * This is a low-keep-alive variant of the basic success test that writes UNSUBSCRIBEs to the server at fast intervals.
 * Verify the server doesn't receive any PINGREQs until the right amount of time after we stop sending the CONNECTs to
 * it.
 *
 * TODO: we can't write this test until we have proper operation handling during CONNECTED state
 */
static int s_mqtt5_client_ping_write_pushout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_ping_write_pushout, s_mqtt5_client_ping_write_pushout_fn)

#define TIMEOUT_TEST_PING_INTERVAL_MS ((uint64_t)10000)

static int s_verify_ping_timeout_interval(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    uint64_t expected_connected_time_ms) {
    aws_mutex_lock(&test_context->lock);

    uint64_t connected_time = 0;
    uint64_t disconnected_time = 0;

    size_t event_count = aws_array_list_length(&test_context->lifecycle_events);
    for (size_t i = 0; i < event_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *record = NULL;
        aws_array_list_get_at(&test_context->lifecycle_events, &record, i);
        if (connected_time == 0 && record->event.event_type == AWS_MQTT5_CLET_CONNECTION_SUCCESS) {
            connected_time = record->timestamp;
        }

        if (disconnected_time == 0 && record->event.event_type == AWS_MQTT5_CLET_DISCONNECTION) {
            disconnected_time = record->timestamp;
        }
    }

    aws_mutex_unlock(&test_context->lock);

    ASSERT_TRUE(connected_time > 0 && disconnected_time > 0 && disconnected_time > connected_time);

    uint64_t connected_interval_ms =
        aws_timestamp_convert(disconnected_time - connected_time, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_MILLIS, NULL);

    ASSERT_TRUE(s_is_within_percentage_of(expected_connected_time_ms, connected_interval_ms, .3));

    return AWS_OP_SUCCESS;
}

/*
 * Test that not receiving a PINGRESP causes a disconnection
 *
 * This is a low-keep-alive variant of the basic success test where the mock server does not respond to a PINGREQ.
 */
static int s_mqtt5_client_ping_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* fast keep alive in order keep tests reasonably short */
    uint16_t keep_alive_seconds =
        (uint16_t)aws_timestamp_convert(TIMEOUT_TEST_PING_INTERVAL_MS, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_SECS, NULL);
    test_options.connect_options.keep_alive_interval_seconds = keep_alive_seconds;

    /* don't respond to PINGREQs */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PINGREQ] = NULL;

    /* faster ping timeout */
    test_options.client_options.ping_timeout_ms = 5000;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);
    s_wait_for_disconnection_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_SUCCESS,
        },
        {
            .event_type = AWS_MQTT5_CLET_DISCONNECTION,
            .error_code = AWS_ERROR_MQTT5_PING_RESPONSE_TIMEOUT,
        },
        {
            .event_type = AWS_MQTT5_CLET_STOPPED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    uint64_t expected_connected_time_ms =
        TIMEOUT_TEST_PING_INTERVAL_MS + (uint64_t)test_context.client->config->ping_timeout_ms;
    ASSERT_SUCCESS(s_verify_ping_timeout_interval(&test_context, expected_connected_time_ms));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CONNECTED,
        AWS_MCS_CLEAN_DISCONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_ping_timeout, s_mqtt5_client_ping_timeout_fn)

/*
 * A variant of the basic ping timeout test that uses a timeout that is larger than the keep alive.  Previously,
 * we forbid this because taken literally, it leads to broken behavior.  We now clamp the ping timeout dynamically
 * based on the connection's established keep alive.
 */
static int s_mqtt5_client_ping_timeout_with_keep_alive_conflict_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* fast keep alive in order keep tests reasonably short */
    uint16_t keep_alive_seconds =
        (uint16_t)aws_timestamp_convert(TIMEOUT_TEST_PING_INTERVAL_MS, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_SECS, NULL);
    test_options.connect_options.keep_alive_interval_seconds = keep_alive_seconds;

    /* don't respond to PINGREQs */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PINGREQ] = NULL;

    /* ping timeout slower than keep alive */
    test_options.client_options.ping_timeout_ms = 2 * TIMEOUT_TEST_PING_INTERVAL_MS;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);
    s_wait_for_disconnection_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_SUCCESS,
        },
        {
            .event_type = AWS_MQTT5_CLET_DISCONNECTION,
            .error_code = AWS_ERROR_MQTT5_PING_RESPONSE_TIMEOUT,
        },
        {
            .event_type = AWS_MQTT5_CLET_STOPPED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    uint64_t expected_connected_time_ms = 3 * TIMEOUT_TEST_PING_INTERVAL_MS / 2;
    ASSERT_SUCCESS(s_verify_ping_timeout_interval(&test_context, expected_connected_time_ms));

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CONNECTED,
        AWS_MCS_CLEAN_DISCONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_ping_timeout_with_keep_alive_conflict,
    s_mqtt5_client_ping_timeout_with_keep_alive_conflict_fn)

/*
 * Set up a zero keep alive and verify no pings get sent over an interval of time.
 */
static int s_mqtt5_client_disabled_keep_alive_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* no keep alive at all */
    test_options.connect_options.keep_alive_interval_seconds = 0;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    uint16_t negotiated_keep_alive = 65535;
    aws_mutex_lock(&test_context.lock);
    size_t event_count = aws_array_list_length(&test_context.lifecycle_events);
    struct aws_mqtt5_lifecycle_event_record *record = NULL;
    aws_array_list_get_at(&test_context.lifecycle_events, &record, event_count - 1);
    ASSERT_TRUE(AWS_MQTT5_CLET_CONNECTION_SUCCESS == record->event.event_type);
    negotiated_keep_alive = record->settings_storage.server_keep_alive;
    aws_mutex_unlock(&test_context.lock);

    ASSERT_INT_EQUALS(0, negotiated_keep_alive);

    // zzz
    aws_thread_current_sleep(aws_timestamp_convert(5, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    // verify the mock server did not get any PINGREQs
    aws_mutex_lock(&test_context.lock);
    size_t pingreq_count = s_count_pingreqs(&test_context);
    aws_mutex_unlock(&test_context.lock);
    ASSERT_INT_EQUALS(0, pingreq_count);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_disabled_keep_alive, s_mqtt5_client_disabled_keep_alive_fn)

struct aws_lifecycle_event_wait_context {
    enum aws_mqtt5_client_lifecycle_event_type type;
    size_t count;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
};

static bool s_received_at_least_n_events(void *arg) {
    struct aws_lifecycle_event_wait_context *context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = context->test_fixture;

    size_t actual_count = 0;
    size_t event_count = aws_array_list_length(&test_fixture->lifecycle_events);
    for (size_t i = 0; i < event_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *record = NULL;
        aws_array_list_get_at(&test_fixture->lifecycle_events, &record, i);

        if (record->event.event_type == context->type) {
            actual_count++;
        }
    }

    return actual_count >= context->count;
}

void aws_mqtt5_wait_for_n_lifecycle_events(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    enum aws_mqtt5_client_lifecycle_event_type type,
    size_t count) {
    struct aws_lifecycle_event_wait_context context = {
        .type = type,
        .count = count,
        .test_fixture = test_context,
    };

    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_received_at_least_n_events, &context);
    aws_mutex_unlock(&test_context->lock);
}

int aws_verify_reconnection_exponential_backoff_timestamps(struct aws_mqtt5_client_mock_test_fixture *test_fixture) {
    aws_mutex_lock(&test_fixture->lock);

    size_t event_count = aws_array_list_length(&test_fixture->lifecycle_events);
    uint64_t last_timestamp = 0;
    uint64_t expected_backoff = RECONNECT_TEST_MIN_BACKOFF;

    for (size_t i = 0; i < event_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *record = NULL;
        aws_array_list_get_at(&test_fixture->lifecycle_events, &record, i);

        if (record->event.event_type == AWS_MQTT5_CLET_CONNECTION_FAILURE) {
            if (last_timestamp == 0) {
                last_timestamp = record->timestamp;
            } else {
                uint64_t time_diff = aws_timestamp_convert(
                    record->timestamp - last_timestamp, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_MILLIS, NULL);

                if (!s_check_time_lower_bound(expected_backoff, time_diff)) {
                    return AWS_OP_ERR;
                }

                expected_backoff = aws_min_u64(expected_backoff * 2, RECONNECT_TEST_MAX_BACKOFF);
                last_timestamp = record->timestamp;
            }
        }
    }

    aws_mutex_unlock(&test_fixture->lock);

    return AWS_OP_SUCCESS;
}

/*
 * Always-fail variant that waits for 6 connection failures and then checks the timestamps between them against
 * what we'd expect with exponential backoff and no jitter
 */
static int s_mqtt5_client_reconnect_failure_backoff_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* backoff delay sequence: 500, 1000, 2000, 4000, 5000, ... */
    test_options.client_options.retry_jitter_mode = AWS_EXPONENTIAL_BACKOFF_JITTER_NONE;
    test_options.client_options.min_reconnect_delay_ms = RECONNECT_TEST_MIN_BACKOFF;
    test_options.client_options.max_reconnect_delay_ms = RECONNECT_TEST_MAX_BACKOFF;
    test_options.client_options.min_connected_time_to_reset_reconnect_delay_ms = RECONNECT_TEST_BACKOFF_RESET_DELAY;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_always_fail;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_mqtt5_wait_for_n_lifecycle_events(&test_context, AWS_MQTT5_CLET_CONNECTION_FAILURE, 6);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    /* 6 (connecting, connection failure) pairs */
    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    ASSERT_SUCCESS(aws_verify_reconnection_exponential_backoff_timestamps(&test_context));

    /* 6 (connecting, mqtt_connect, channel_shutdown, pending_reconnect) tuples (minus the final pending_reconnect) */
    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING, AWS_MCS_MQTT_CONNECT, AWS_MCS_CHANNEL_SHUTDOWN, AWS_MCS_PENDING_RECONNECT,
        AWS_MCS_CONNECTING, AWS_MCS_MQTT_CONNECT, AWS_MCS_CHANNEL_SHUTDOWN, AWS_MCS_PENDING_RECONNECT,
        AWS_MCS_CONNECTING, AWS_MCS_MQTT_CONNECT, AWS_MCS_CHANNEL_SHUTDOWN, AWS_MCS_PENDING_RECONNECT,
        AWS_MCS_CONNECTING, AWS_MCS_MQTT_CONNECT, AWS_MCS_CHANNEL_SHUTDOWN, AWS_MCS_PENDING_RECONNECT,
        AWS_MCS_CONNECTING, AWS_MCS_MQTT_CONNECT, AWS_MCS_CHANNEL_SHUTDOWN, AWS_MCS_PENDING_RECONNECT,
        AWS_MCS_CONNECTING, AWS_MCS_MQTT_CONNECT, AWS_MCS_CHANNEL_SHUTDOWN,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_reconnect_failure_backoff, s_mqtt5_client_reconnect_failure_backoff_fn)

int aws_mqtt5_mock_server_handle_connect_succeed_on_nth(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct aws_mqtt5_mock_server_reconnect_state *context = user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    if (context->connection_attempts == context->required_connection_count_threshold) {
        connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
        aws_high_res_clock_get_ticks(&context->connect_timestamp);
    } else {
        connack_view.reason_code = AWS_MQTT5_CRC_NOT_AUTHORIZED;
    }

    ++context->connection_attempts;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

static void s_aws_mqtt5_mock_server_disconnect_after_n_ms(
    struct aws_mqtt5_server_mock_connection_context *mock_server,
    void *user_data) {

    struct aws_mqtt5_mock_server_reconnect_state *context = user_data;
    if (context->connect_timestamp == 0) {
        return;
    }

    uint64_t now = 0;
    aws_high_res_clock_get_ticks(&now);

    if (now < context->connect_timestamp) {
        return;
    }

    uint64_t elapsed_ms =
        aws_timestamp_convert(now - context->connect_timestamp, AWS_TIMESTAMP_NANOS, AWS_TIMESTAMP_MILLIS, NULL);
    if (elapsed_ms > context->successful_connection_disconnect_delay_ms) {

        struct aws_mqtt5_packet_disconnect_view disconnect = {
            .reason_code = AWS_MQTT5_DRC_PACKET_TOO_LARGE,
        };

        aws_mqtt5_mock_server_send_packet(mock_server, AWS_MQTT5_PT_DISCONNECT, &disconnect);
        context->connect_timestamp = 0;
    }
}

static int s_verify_reconnection_after_success_used_backoff(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    uint64_t expected_reconnect_delay_ms) {

    aws_mutex_lock(&test_fixture->lock);

    size_t event_count = aws_array_list_length(&test_fixture->lifecycle_events);

    uint64_t disconnect_after_success_timestamp = 0;
    uint64_t reconnect_failure_after_disconnect_timestamp = 0;

    for (size_t i = 0; i < event_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *record = NULL;
        aws_array_list_get_at(&test_fixture->lifecycle_events, &record, i);

        if (record->event.event_type == AWS_MQTT5_CLET_DISCONNECTION) {
            ASSERT_INT_EQUALS(0, disconnect_after_success_timestamp);
            disconnect_after_success_timestamp = record->timestamp;
        } else if (record->event.event_type == AWS_MQTT5_CLET_CONNECTION_FAILURE) {
            if (reconnect_failure_after_disconnect_timestamp == 0 && disconnect_after_success_timestamp > 0) {
                reconnect_failure_after_disconnect_timestamp = record->timestamp;
            }
        }
    }

    aws_mutex_unlock(&test_fixture->lock);

    ASSERT_TRUE(disconnect_after_success_timestamp > 0 && reconnect_failure_after_disconnect_timestamp > 0);

    uint64_t post_success_reconnect_time_ms = aws_timestamp_convert(
        reconnect_failure_after_disconnect_timestamp - disconnect_after_success_timestamp,
        AWS_TIMESTAMP_NANOS,
        AWS_TIMESTAMP_MILLIS,
        NULL);

    /* The reconnect delay should be at least the expected backoff time.
     * Meanwhile, to make sure the backoff time is reset to minimal properly, we also would like to validate the upper
     * bound of the reconnect time. With function `s_validate_time_interval()`, we used a 30% margin and a minimal of
     * AWS_MQTT5_TESTING_DELAY_NS for system delay to validate the upper bound.
     */
    if (!s_check_time_lower_bound(expected_reconnect_delay_ms, post_success_reconnect_time_ms) &&
        !s_validate_time_interval(expected_reconnect_delay_ms, post_success_reconnect_time_ms, .3)) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

/*
 * Fail-until-max-backoff variant, followed by a success that then quickly disconnects.  Verify the next reconnect
 * attempt still uses the maximum backoff because we weren't connected long enough to reset it.
 */
static int s_mqtt5_client_reconnect_backoff_insufficient_reset_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_mock_server_reconnect_state mock_server_state = {
        .required_connection_count_threshold = 6,
        /* quick disconnect should not reset reconnect delay */
        .successful_connection_disconnect_delay_ms = RECONNECT_TEST_BACKOFF_RESET_DELAY / 5,
    };

    /* backoff delay sequence: 500, 1000, 2000, 4000, 5000, ... */
    test_options.client_options.retry_jitter_mode = AWS_EXPONENTIAL_BACKOFF_JITTER_NONE;
    test_options.client_options.min_reconnect_delay_ms = RECONNECT_TEST_MIN_BACKOFF;
    test_options.client_options.max_reconnect_delay_ms = RECONNECT_TEST_MAX_BACKOFF;
    test_options.client_options.min_connected_time_to_reset_reconnect_delay_ms = RECONNECT_TEST_BACKOFF_RESET_DELAY;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_succeed_on_nth;
    test_options.server_function_table.service_task_fn = s_aws_mqtt5_mock_server_disconnect_after_n_ms;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &mock_server_state,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_mqtt5_wait_for_n_lifecycle_events(&test_context, AWS_MQTT5_CLET_CONNECTION_FAILURE, 7);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    /* 6 (connecting, connection failure) pairs, followed by a successful connection, then a disconnect and reconnect */
    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_SUCCESS,
        },
        {
            .event_type = AWS_MQTT5_CLET_DISCONNECTION,
            .error_code = AWS_ERROR_MQTT5_DISCONNECT_RECEIVED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    ASSERT_SUCCESS(s_verify_reconnection_after_success_used_backoff(&test_context, RECONNECT_TEST_MAX_BACKOFF));
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_reconnect_backoff_insufficient_reset, s_mqtt5_client_reconnect_backoff_insufficient_reset_fn)

/*
 * Fail-until-max-backoff variant, followed by a success that disconnects after enough time has passed that the backoff
 * should be reset.  Verify that the next reconnect is back to using the minimum backoff value.
 */
static int s_mqtt5_client_reconnect_backoff_sufficient_reset_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_mock_server_reconnect_state mock_server_state = {
        .required_connection_count_threshold = 6,
        /* slow disconnect should reset reconnect delay */
        .successful_connection_disconnect_delay_ms = RECONNECT_TEST_BACKOFF_RESET_DELAY * 2,
    };

    /* backoff delay sequence: 500, 1000, 2000, 4000, 5000, ... */
    test_options.client_options.retry_jitter_mode = AWS_EXPONENTIAL_BACKOFF_JITTER_NONE;
    test_options.client_options.min_reconnect_delay_ms = RECONNECT_TEST_MIN_BACKOFF;
    test_options.client_options.max_reconnect_delay_ms = RECONNECT_TEST_MAX_BACKOFF;
    test_options.client_options.min_connected_time_to_reset_reconnect_delay_ms = RECONNECT_TEST_BACKOFF_RESET_DELAY;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_succeed_on_nth;
    test_options.server_function_table.service_task_fn = s_aws_mqtt5_mock_server_disconnect_after_n_ms;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &mock_server_state,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_mqtt5_wait_for_n_lifecycle_events(&test_context, AWS_MQTT5_CLET_CONNECTION_FAILURE, 7);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    /* 6 (connecting, connection failure) pairs, followed by a successful connection, then a disconnect and reconnect */
    struct aws_mqtt5_client_lifecycle_event expected_events[] = {
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_SUCCESS,
        },
        {
            .event_type = AWS_MQTT5_CLET_DISCONNECTION,
            .error_code = AWS_ERROR_MQTT5_DISCONNECT_RECEIVED,
        },
        {
            .event_type = AWS_MQTT5_CLET_ATTEMPTING_CONNECT,
        },
        {
            .event_type = AWS_MQTT5_CLET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT5_CONNACK_CONNECTION_REFUSED,
        },
    };
    ASSERT_SUCCESS(
        s_verify_simple_lifecycle_event_sequence(&test_context, expected_events, AWS_ARRAY_SIZE(expected_events)));

    ASSERT_SUCCESS(s_verify_reconnection_after_success_used_backoff(&test_context, RECONNECT_TEST_MIN_BACKOFF));
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_reconnect_backoff_sufficient_reset, s_mqtt5_client_reconnect_backoff_sufficient_reset_fn)

static const char s_topic_filter1[] = "some/topic/but/letsmakeit/longer/soIcanfailpacketsizetests/+";

static struct aws_mqtt5_subscription_view s_subscriptions[] = {
    {
        .topic_filter =
            {
                .ptr = (uint8_t *)s_topic_filter1,
                .len = AWS_ARRAY_SIZE(s_topic_filter1) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .no_local = false,
        .retain_as_published = false,
        .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE,
    },
};

static enum aws_mqtt5_suback_reason_code s_suback_reason_codes[] = {
    AWS_MQTT5_SARC_GRANTED_QOS_1,
};

void s_aws_mqtt5_subscribe_complete_fn(
    const struct aws_mqtt5_packet_suback_view *suback,
    int error_code,
    void *complete_ctx) {

    (void)suback;
    (void)error_code;

    struct aws_mqtt5_client_mock_test_fixture *test_context = complete_ctx;

    aws_mutex_lock(&test_context->lock);
    test_context->subscribe_complete = true;
    aws_mutex_unlock(&test_context->lock);
    aws_condition_variable_notify_all(&test_context->signal);
}

static bool s_received_suback(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_context = arg;

    return test_context->subscribe_complete;
}

static void s_wait_for_suback_received(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(&test_context->signal, &test_context->lock, s_received_suback, test_context);
    aws_mutex_unlock(&test_context->lock);
}

int aws_mqtt5_server_send_suback_on_subscribe(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_subscribe_view *subscribe_view = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_view->packet_id,
        .reason_code_count = AWS_ARRAY_SIZE(s_suback_reason_codes),
        .reason_codes = s_suback_reason_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

/* Connection test where we succeed, send a SUBSCRIBE, and wait for a SUBACK */
static int s_mqtt5_client_subscribe_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        aws_mqtt5_server_send_suback_on_subscribe;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .subscriptions = s_subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(s_subscriptions),
    };

    struct aws_mqtt5_subscribe_completion_options completion_options = {
        .completion_callback = s_aws_mqtt5_subscribe_complete_fn,
        .completion_user_data = &test_context,
    };
    aws_mqtt5_client_subscribe(client, &subscribe_view, &completion_options);

    s_wait_for_suback_received(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_subscribe_success, s_mqtt5_client_subscribe_success_fn)

static int s_aws_mqtt5_mock_server_handle_connect_succeed_maximum_packet_size(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    uint32_t maximum_packet_size = 50;

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    connack_view.maximum_packet_size = &maximum_packet_size;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

void s_aws_mqtt5_subscribe_complete_packet_size_too_small_fn(
    const struct aws_mqtt5_packet_suback_view *suback,
    int error_code,
    void *complete_ctx) {

    AWS_FATAL_ASSERT(suback == NULL);
    AWS_FATAL_ASSERT(AWS_ERROR_MQTT5_PACKET_VALIDATION == error_code);

    struct aws_mqtt5_client_mock_test_fixture *test_context = complete_ctx;

    aws_mutex_lock(&test_context->lock);
    test_context->subscribe_complete = true;
    aws_mutex_unlock(&test_context->lock);
    aws_condition_variable_notify_all(&test_context->signal);
}

static int s_mqtt5_client_subscribe_fail_packet_too_big_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_succeed_maximum_packet_size;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .subscriptions = s_subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(s_subscriptions),
    };

    struct aws_mqtt5_subscribe_completion_options completion_options = {
        .completion_callback = s_aws_mqtt5_subscribe_complete_packet_size_too_small_fn,
        .completion_user_data = &test_context,
    };
    aws_mqtt5_client_subscribe(client, &subscribe_view, &completion_options);

    s_wait_for_suback_received(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_subscribe_fail_packet_too_big, s_mqtt5_client_subscribe_fail_packet_too_big_fn)

static void s_aws_mqtt5_disconnect_failure_completion_fn(int error_code, void *complete_ctx) {
    AWS_FATAL_ASSERT(error_code == AWS_ERROR_MQTT5_PACKET_VALIDATION);

    s_on_disconnect_completion(error_code, complete_ctx);
}

static int s_mqtt5_client_disconnect_fail_packet_too_big_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_succeed_maximum_packet_size;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_byte_cursor long_reason_string_cursor = aws_byte_cursor_from_c_str(
        "Not valid because it includes the 0-terminator but we don't check utf-8 so who cares");

    struct aws_mqtt5_packet_disconnect_view disconnect_view = {
        .reason_string = &long_reason_string_cursor,
    };

    struct aws_mqtt5_disconnect_completion_options completion_options = {
        .completion_callback = s_aws_mqtt5_disconnect_failure_completion_fn,
        .completion_user_data = &test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, &disconnect_view, &completion_options));

    s_wait_for_disconnect_completion(&test_context);
    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_disconnect_fail_packet_too_big, s_mqtt5_client_disconnect_fail_packet_too_big_fn)

static uint8_t s_topic[] = "Hello/world";

#define RECEIVE_MAXIMUM_PUBLISH_COUNT 30
#define TEST_RECEIVE_MAXIMUM 3

static int s_aws_mqtt5_mock_server_handle_connect_succeed_receive_maximum(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    uint16_t receive_maximum = TEST_RECEIVE_MAXIMUM;

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    connack_view.receive_maximum = &receive_maximum;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

struct send_puback_task {
    struct aws_allocator *allocator;
    struct aws_task task;
    struct aws_mqtt5_server_mock_connection_context *connection;
    uint16_t packet_id;
};

void send_puback_fn(struct aws_task *task, void *arg, enum aws_task_status status) {
    (void)task;

    struct send_puback_task *puback_response_task = arg;
    if (status == AWS_TASK_STATUS_CANCELED) {
        goto done;
    }

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = puback_response_task->connection->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    --test_fixture->server_current_inflight_publishes;
    aws_mutex_unlock(&test_fixture->lock);

    struct aws_mqtt5_packet_puback_view puback_view = {
        .packet_id = puback_response_task->packet_id,
    };

    aws_mqtt5_mock_server_send_packet(puback_response_task->connection, AWS_MQTT5_PT_PUBACK, &puback_view);

done:

    aws_mem_release(puback_response_task->allocator, puback_response_task);
}

static int s_aws_mqtt5_mock_server_handle_publish_delayed_puback(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {

    (void)user_data;

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = connection->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    ++test_fixture->server_current_inflight_publishes;
    test_fixture->server_maximum_inflight_publishes =
        aws_max_u32(test_fixture->server_current_inflight_publishes, test_fixture->server_maximum_inflight_publishes);
    aws_mutex_unlock(&test_fixture->lock);

    struct aws_mqtt5_packet_publish_view *publish_view = packet;

    struct send_puback_task *puback_response_task =
        aws_mem_calloc(connection->allocator, 1, sizeof(struct send_puback_task));
    puback_response_task->allocator = connection->allocator;
    puback_response_task->connection = connection;
    puback_response_task->packet_id = publish_view->packet_id;

    aws_task_init(&puback_response_task->task, send_puback_fn, puback_response_task, "delayed_puback_response");

    struct aws_event_loop *event_loop = aws_channel_get_event_loop(connection->slot->channel);

    uint64_t now = 0;
    aws_event_loop_current_clock_time(event_loop, &now);

    uint64_t min_delay = aws_timestamp_convert(250, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_NANOS, NULL);
    uint64_t max_delay = aws_timestamp_convert(500, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_NANOS, NULL);
    uint64_t delay_nanos = aws_mqtt5_client_random_in_range(min_delay, max_delay);

    uint64_t puback_time = aws_add_u64_saturating(now, delay_nanos);
    aws_event_loop_schedule_task_future(event_loop, &puback_response_task->task, puback_time);

    return AWS_OP_SUCCESS;
}

static void s_receive_maximum_publish_completion_fn(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *complete_ctx) {

    if (packet_type != AWS_MQTT5_PT_PUBACK) {
        return;
    }

    const struct aws_mqtt5_packet_puback_view *puback = packet;
    struct aws_mqtt5_client_mock_test_fixture *test_context = complete_ctx;

    uint64_t now = 0;
    aws_high_res_clock_get_ticks(&now);

    aws_mutex_lock(&test_context->lock);

    ++test_context->total_pubacks_received;
    if (error_code == AWS_ERROR_SUCCESS && puback->reason_code < 128) {
        ++test_context->successful_pubacks_received;
    }

    aws_mutex_unlock(&test_context->lock);
    aws_condition_variable_notify_all(&test_context->signal);
}

static bool s_received_n_successful_publishes(void *arg) {
    struct aws_mqtt5_client_test_wait_for_n_context *context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = context->test_fixture;

    return test_fixture->successful_pubacks_received >= context->required_event_count;
}

static void s_wait_for_n_successful_publishes(struct aws_mqtt5_client_test_wait_for_n_context *context) {

    struct aws_mqtt5_client_mock_test_fixture *test_context = context->test_fixture;
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_received_n_successful_publishes, context);
    aws_mutex_unlock(&test_context->lock);
}

static int s_mqtt5_client_flow_control_receive_maximum_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* send delayed pubacks */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_mock_server_handle_publish_delayed_puback;

    /* establish a low receive maximum */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_succeed_receive_maximum;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    /* send a bunch of publishes */
    for (size_t i = 0; i < RECEIVE_MAXIMUM_PUBLISH_COUNT; ++i) {
        struct aws_mqtt5_packet_publish_view qos1_publish_view = {
            .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
            .topic =
                {
                    .ptr = s_topic,
                    .len = AWS_ARRAY_SIZE(s_topic) - 1,
                },
        };

        struct aws_mqtt5_publish_completion_options completion_options = {
            .completion_callback = s_receive_maximum_publish_completion_fn,
            .completion_user_data = &test_context,
        };

        ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish_view, &completion_options));
    }

    /* wait for all publishes to succeed */
    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = RECEIVE_MAXIMUM_PUBLISH_COUNT,
    };
    s_wait_for_n_successful_publishes(&wait_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    /*
     * verify that the maximum number of in-progress qos1 publishes on the server was never more than what the
     * server said its maximum was
     */
    aws_mutex_lock(&test_context.lock);
    uint32_t max_inflight_publishes = test_context.server_maximum_inflight_publishes;
    aws_mutex_unlock(&test_context.lock);

    ASSERT_TRUE(max_inflight_publishes <= TEST_RECEIVE_MAXIMUM);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_flow_control_receive_maximum, s_mqtt5_client_flow_control_receive_maximum_fn)

static void s_publish_timeout_publish_completion_fn(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *complete_ctx) {
    (void)packet;
    (void)packet_type;

    struct aws_mqtt5_client_mock_test_fixture *test_context = complete_ctx;

    aws_mutex_lock(&test_context->lock);

    if (error_code == AWS_ERROR_MQTT_TIMEOUT) {
        ++test_context->timeouts_received;
    }

    aws_mutex_unlock(&test_context->lock);
    aws_condition_variable_notify_all(&test_context->signal);
}

static bool s_received_n_publish_timeouts(void *arg) {
    struct aws_mqtt5_client_test_wait_for_n_context *context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = context->test_fixture;

    return test_fixture->timeouts_received >= context->required_event_count;
}

static void s_wait_for_n_publish_timeouts(struct aws_mqtt5_client_test_wait_for_n_context *context) {
    struct aws_mqtt5_client_mock_test_fixture *test_context = context->test_fixture;
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_received_n_publish_timeouts, context);
    aws_mutex_unlock(&test_context->lock);
}

static bool s_sent_n_timeout_publish_packets(void *arg) {
    struct aws_mqtt5_client_test_wait_for_n_context *context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = context->test_fixture;

    return test_fixture->publishes_received >= context->required_event_count;
}

static int s_aws_mqtt5_mock_server_handle_timeout_publish(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {

    (void)packet;
    (void)user_data;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = connection->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    ++connection->test_fixture->publishes_received;
    aws_mutex_unlock(&test_fixture->lock);

    return AWS_OP_SUCCESS;
}

static void s_wait_for_n_successful_server_timeout_publishes(struct aws_mqtt5_client_test_wait_for_n_context *context) {
    struct aws_mqtt5_client_mock_test_fixture *test_context = context->test_fixture;
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_sent_n_timeout_publish_packets, context);
    aws_mutex_unlock(&test_context->lock);
}

/*
 * Test that not receiving a PUBACK causes the PUBLISH waiting for the PUBACK to timeout
 */
static int s_mqtt5_client_publish_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_mock_server_handle_timeout_publish;

    /* fast publish timeout */
    test_options.client_options.ack_timeout_seconds = 5;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_publish_completion_options completion_options = {
        .completion_callback = &s_publish_timeout_publish_completion_fn,
        .completion_user_data = &test_context,
    };

    struct aws_mqtt5_packet_publish_view packet_publish_view = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic =
            {
                .ptr = s_topic,
                .len = AWS_ARRAY_SIZE(s_topic) - 1,
            },
    };

    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = (size_t)aws_mqtt5_client_random_in_range(3, 20),
    };

    /* Send semi-random number of publishes that will not be acked */
    for (size_t publish_count = 0; publish_count < wait_context.required_event_count; ++publish_count) {
        ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &packet_publish_view, &completion_options));
    }

    s_wait_for_n_successful_server_timeout_publishes(&wait_context);

    s_wait_for_n_publish_timeouts(&wait_context);

    ASSERT_INT_EQUALS(wait_context.required_event_count, test_context.timeouts_received);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_publish_timeout, s_mqtt5_client_publish_timeout_fn)

int aws_mqtt5_mock_server_handle_publish_puback(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {

    (void)user_data;

    struct aws_mqtt5_packet_publish_view *publish_view = packet;
    if (publish_view->qos != AWS_MQTT5_QOS_AT_LEAST_ONCE) {
        return AWS_OP_SUCCESS;
    }

    struct aws_mqtt5_packet_puback_view puback_view = {
        .packet_id = publish_view->packet_id,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBACK, &puback_view);
}

#define IOT_CORE_THROUGHPUT_PACKETS 21

static uint8_t s_large_packet_payload[127 * 1024];

static int s_do_iot_core_throughput_test(struct aws_allocator *allocator, bool use_iot_core_limits) {

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* send pubacks */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback;

    if (use_iot_core_limits) {
        test_options.client_options.extended_validation_and_flow_control_options =
            AWS_MQTT5_EVAFCO_AWS_IOT_CORE_DEFAULTS;
    }

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    /* send a bunch of large publishes */
    aws_secure_zero(s_large_packet_payload, AWS_ARRAY_SIZE(s_large_packet_payload));

    for (size_t i = 0; i < IOT_CORE_THROUGHPUT_PACKETS; ++i) {
        struct aws_mqtt5_packet_publish_view qos1_publish_view = {
            .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
            .topic =
                {
                    .ptr = s_topic,
                    .len = AWS_ARRAY_SIZE(s_topic) - 1,
                },
            .payload = aws_byte_cursor_from_array(s_large_packet_payload, AWS_ARRAY_SIZE(s_large_packet_payload)),
        };

        struct aws_mqtt5_publish_completion_options completion_options = {
            .completion_callback = s_receive_maximum_publish_completion_fn, /* can reuse receive_maximum callback */
            .completion_user_data = &test_context,
        };

        ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish_view, &completion_options));
    }

    /* wait for all publishes to succeed */
    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = IOT_CORE_THROUGHPUT_PACKETS,
    };
    s_wait_for_n_successful_publishes(&wait_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_thread_join_all_managed();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_flow_control_iot_core_throughput_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    uint64_t start_time = 0;
    aws_high_res_clock_get_ticks(&start_time);

    ASSERT_SUCCESS(s_do_iot_core_throughput_test(allocator, true));

    uint64_t end_time = 0;
    aws_high_res_clock_get_ticks(&end_time);

    uint64_t test_time = end_time - start_time;

    /*
     * We expect the throttled version to take around 5 seconds, since we're sending 21 almost-max size (127k) packets
     * against a limit of 512KB/s.  Since the packets are submitted immediately on CONNACK, the rate limiter
     * token bucket is starting at zero and so will give us immediate throttling.
     */
    ASSERT_TRUE(test_time > 5 * (uint64_t)AWS_TIMESTAMP_NANOS);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_flow_control_iot_core_throughput, s_mqtt5_client_flow_control_iot_core_throughput_fn)

#define IOT_CORE_PUBLISH_TPS_PACKETS 650

static int s_do_iot_core_publish_tps_test(
    struct aws_allocator *allocator,
    bool use_iot_core_limits,
    uint64_t *test_time) {

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* send pubacks */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback;

    if (use_iot_core_limits) {
        test_options.client_options.extended_validation_and_flow_control_options =
            AWS_MQTT5_EVAFCO_AWS_IOT_CORE_DEFAULTS;
    }

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    /* send a bunch of tiny publishes */
    for (size_t i = 0; i < IOT_CORE_PUBLISH_TPS_PACKETS; ++i) {
        struct aws_mqtt5_packet_publish_view qos1_publish_view = {
            .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
            .topic =
                {
                    .ptr = s_topic,
                    .len = AWS_ARRAY_SIZE(s_topic) - 1,
                },
        };

        struct aws_mqtt5_publish_completion_options completion_options = {
            .completion_callback = s_receive_maximum_publish_completion_fn, /* can reuse receive_maximum callback */
            .completion_user_data = &test_context,
        };

        ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish_view, &completion_options));
    }

    /* wait for all publishes to succeed */
    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = IOT_CORE_PUBLISH_TPS_PACKETS,
    };
    s_wait_for_n_successful_publishes(&wait_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_high_res_clock_get_ticks(test_time);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_thread_join_all_managed();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_flow_control_iot_core_publish_tps_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    uint64_t start_time1 = 0;
    aws_high_res_clock_get_ticks(&start_time1);
    uint64_t end_time1 = 0;

    ASSERT_SUCCESS(s_do_iot_core_publish_tps_test(allocator, false, &end_time1));

    uint64_t test_time1 = end_time1 - start_time1;

    uint64_t start_time2 = 0;
    aws_high_res_clock_get_ticks(&start_time2);

    uint64_t end_time2 = 0;
    ASSERT_SUCCESS(s_do_iot_core_publish_tps_test(allocator, true, &end_time2));

    uint64_t test_time2 = end_time2 - start_time2;

    /* We expect the unthrottled test to complete quickly */
    ASSERT_TRUE(test_time1 < AWS_TIMESTAMP_NANOS);

    /*
     * We expect the throttled version to take over 6 seconds, since we're sending over 650 tiny publish packets
     * against a limit of 100TPS.  Since the packets are submitted immediately on CONNACK, the rate limiter
     * token bucket is starting at zero and so will give us immediate throttling.
     */
    ASSERT_TRUE(test_time2 > 6 * (uint64_t)AWS_TIMESTAMP_NANOS);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_flow_control_iot_core_publish_tps, s_mqtt5_client_flow_control_iot_core_publish_tps_fn)

static int s_aws_mqtt5_mock_server_handle_connect_honor_session_after_success(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connect_view *connect_packet = packet;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    /* Only resume a connection if the client has already connected to the server before */
    if (connection->test_fixture->client->has_connected_successfully) {
        connack_view.session_present = !connect_packet->clean_start;
    }

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

int aws_mqtt5_mock_server_handle_connect_honor_session_unconditional(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connect_view *connect_packet = packet;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    connack_view.session_present = !connect_packet->clean_start;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

struct aws_mqtt5_wait_for_n_lifecycle_events_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
    enum aws_mqtt5_client_lifecycle_event_type event_type;
    size_t expected_event_count;
};

static bool s_received_n_lifecycle_events(void *arg) {
    struct aws_mqtt5_wait_for_n_lifecycle_events_context *context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = context->test_fixture;

    size_t matching_events = 0;
    size_t event_count = aws_array_list_length(&test_fixture->lifecycle_events);
    for (size_t i = 0; i < event_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *record = NULL;
        aws_array_list_get_at(&test_fixture->lifecycle_events, &record, i);

        if (record->event.event_type == context->event_type) {
            ++matching_events;
        }
    }

    return matching_events >= context->expected_event_count;
}

void aws_wait_for_n_lifecycle_events(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    enum aws_mqtt5_client_lifecycle_event_type event_type,
    size_t expected_event_count) {
    struct aws_mqtt5_wait_for_n_lifecycle_events_context wait_context = {
        .test_fixture = test_fixture,
        .event_type = event_type,
        .expected_event_count = expected_event_count,
    };

    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(
        &test_fixture->signal, &test_fixture->lock, s_received_n_lifecycle_events, &wait_context);
    aws_mutex_unlock(&test_fixture->lock);
}

static bool s_compute_expected_rejoined_session(
    enum aws_mqtt5_client_session_behavior_type session_behavior,
    size_t connect_index) {
    switch (session_behavior) {
        case AWS_MQTT5_CSBT_CLEAN:
            return false;

        case AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS:
            return connect_index > 0;

        case AWS_MQTT5_CSBT_REJOIN_ALWAYS:
        default:
            return true;
    }
}

static int s_aws_mqtt5_client_test_init_resume_session_connect_storage(
    struct aws_mqtt5_packet_connect_storage *storage,
    struct aws_allocator *allocator) {

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds = 30,
        .client_id = aws_byte_cursor_from_string(g_default_client_id),
        .clean_start = false,
    };

    return aws_mqtt5_packet_connect_storage_init(storage, allocator, &connect_view);
}

#define SESSION_RESUMPTION_CONNECT_COUNT 5

static int s_do_mqtt5_client_session_resumption_test(
    struct aws_allocator *allocator,
    enum aws_mqtt5_client_session_behavior_type session_behavior) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.client_options.session_behavior = session_behavior;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_honor_session_unconditional;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    for (size_t i = 0; i < SESSION_RESUMPTION_CONNECT_COUNT; ++i) {
        ASSERT_SUCCESS(aws_mqtt5_client_start(client));
        aws_wait_for_n_lifecycle_events(&test_context, AWS_MQTT5_CLET_CONNECTION_SUCCESS, i + 1);

        /* not technically truly safe to query depending on memory model.  Remove if it becomes a problem. */
        bool expected_rejoined_session = s_compute_expected_rejoined_session(session_behavior, i);
        ASSERT_INT_EQUALS(expected_rejoined_session, client->negotiated_settings.rejoined_session);

        /* can't use stop as that wipes session state */
        aws_channel_shutdown(test_context.server_channel, AWS_ERROR_UNKNOWN);

        aws_wait_for_n_lifecycle_events(&test_context, AWS_MQTT5_CLET_DISCONNECTION, i + 1);
    }

    struct aws_mqtt5_packet_connect_storage clean_start_connect_storage;
    ASSERT_SUCCESS(s_aws_mqtt5_client_test_init_default_connect_storage(&clean_start_connect_storage, allocator));

    struct aws_mqtt5_packet_connect_storage resume_session_connect_storage;
    ASSERT_SUCCESS(
        s_aws_mqtt5_client_test_init_resume_session_connect_storage(&resume_session_connect_storage, allocator));

    struct aws_mqtt5_mock_server_packet_record expected_packets[SESSION_RESUMPTION_CONNECT_COUNT];
    for (size_t i = 0; i < SESSION_RESUMPTION_CONNECT_COUNT; ++i) {
        expected_packets[i].packet_type = AWS_MQTT5_PT_CONNECT;
        if (s_compute_expected_rejoined_session(session_behavior, i)) {
            expected_packets[i].packet_storage = &resume_session_connect_storage;
        } else {
            expected_packets[i].packet_storage = &clean_start_connect_storage;
        }
    }

    ASSERT_SUCCESS(
        aws_verify_received_packet_sequence(&test_context, expected_packets, AWS_ARRAY_SIZE(expected_packets)));

    aws_mqtt5_packet_connect_storage_clean_up(&clean_start_connect_storage);
    aws_mqtt5_packet_connect_storage_clean_up(&resume_session_connect_storage);
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_session_resumption_clean_start_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5_client_session_resumption_test(allocator, AWS_MQTT5_CSBT_CLEAN));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_session_resumption_clean_start, s_mqtt5_client_session_resumption_clean_start_fn)

static int s_mqtt5_client_session_resumption_post_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5_client_session_resumption_test(allocator, AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_session_resumption_post_success, s_mqtt5_client_session_resumption_post_success_fn)

static int s_mqtt5_client_session_resumption_always_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5_client_session_resumption_test(allocator, AWS_MQTT5_CSBT_REJOIN_ALWAYS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_session_resumption_always, s_mqtt5_client_session_resumption_always_fn)

static uint8_t s_sub_pub_unsub_topic_filter[] = "hello/+";

static struct aws_mqtt5_subscription_view s_sub_pub_unsub_subscriptions[] = {
    {
        .topic_filter =
            {
                .ptr = (uint8_t *)s_sub_pub_unsub_topic_filter,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_topic_filter) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .no_local = false,
        .retain_as_published = false,
        .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE,
    },
};

static struct aws_byte_cursor s_sub_pub_unsub_topic_filters[] = {
    {
        .ptr = s_sub_pub_unsub_topic_filter,
        .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_topic_filter) - 1,
    },
};

struct aws_mqtt5_sub_pub_unsub_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;

    bool subscribe_complete;
    bool publish_complete;
    bool publish_received;
    bool unsubscribe_complete;
    size_t publishes_received;

    size_t subscribe_failures;
    size_t publish_failures;
    size_t unsubscribe_failures;

    struct aws_mqtt5_packet_publish_storage publish_storage;
};

static void s_sub_pub_unsub_context_clean_up(struct aws_mqtt5_sub_pub_unsub_context *context) {
    aws_mqtt5_packet_publish_storage_clean_up(&context->publish_storage);
}

void s_sub_pub_unsub_subscribe_complete_fn(
    const struct aws_mqtt5_packet_suback_view *suback,
    int error_code,
    void *complete_ctx) {

    AWS_FATAL_ASSERT(suback != NULL);
    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);

    struct aws_mqtt5_sub_pub_unsub_context *test_context = complete_ctx;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    test_context->subscribe_complete = true;
    if (error_code != AWS_ERROR_SUCCESS) {
        ++test_context->subscribe_failures;
    }
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

static bool s_sub_pub_unsub_received_suback(void *arg) {
    struct aws_mqtt5_sub_pub_unsub_context *test_context = arg;

    return test_context->subscribe_complete && test_context->subscribe_failures == 0;
}

static void s_sub_pub_unsub_wait_for_suback_received(struct aws_mqtt5_sub_pub_unsub_context *test_context) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(
        &test_fixture->signal, &test_fixture->lock, s_sub_pub_unsub_received_suback, test_context);
    aws_mutex_unlock(&test_fixture->lock);
}

static int s_mqtt5_client_sub_pub_unsub_subscribe(
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context,
    struct aws_mqtt5_packet_subscribe_storage *expected_subscribe_storage) {
    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .subscriptions = s_sub_pub_unsub_subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(s_sub_pub_unsub_subscriptions),
    };

    struct aws_mqtt5_subscribe_completion_options completion_options = {
        .completion_callback = s_sub_pub_unsub_subscribe_complete_fn,
        .completion_user_data = full_test_context,
    };

    struct aws_mqtt5_client *client = full_test_context->test_fixture->client;
    ASSERT_SUCCESS(aws_mqtt5_client_subscribe(client, &subscribe_view, &completion_options));

    s_sub_pub_unsub_wait_for_suback_received(full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_packet_subscribe_storage_init(
        expected_subscribe_storage, full_test_context->test_fixture->allocator, &subscribe_view));

    return AWS_OP_SUCCESS;
}

void s_sub_pub_unsub_unsubscribe_complete_fn(
    const struct aws_mqtt5_packet_unsuback_view *unsuback,
    int error_code,
    void *complete_ctx) {

    AWS_FATAL_ASSERT(unsuback != NULL);
    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);

    struct aws_mqtt5_sub_pub_unsub_context *test_context = complete_ctx;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    test_context->unsubscribe_complete = true;
    if (error_code != AWS_ERROR_SUCCESS) {
        ++test_context->unsubscribe_failures;
    }
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

static bool s_sub_pub_unsub_received_unsuback(void *arg) {
    struct aws_mqtt5_sub_pub_unsub_context *test_context = arg;

    return test_context->unsubscribe_complete && test_context->unsubscribe_failures == 0;
}

static void s_sub_pub_unsub_wait_for_unsuback_received(struct aws_mqtt5_sub_pub_unsub_context *test_context) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(
        &test_fixture->signal, &test_fixture->lock, s_sub_pub_unsub_received_unsuback, test_context);
    aws_mutex_unlock(&test_fixture->lock);
}

static int s_mqtt5_client_sub_pub_unsub_unsubscribe(
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context,
    struct aws_mqtt5_packet_unsubscribe_storage *expected_unsubscribe_storage) {
    struct aws_mqtt5_packet_unsubscribe_view unsubscribe_view = {
        .topic_filters = s_sub_pub_unsub_topic_filters,
        .topic_filter_count = AWS_ARRAY_SIZE(s_sub_pub_unsub_topic_filters),
    };

    struct aws_mqtt5_unsubscribe_completion_options completion_options = {
        .completion_callback = s_sub_pub_unsub_unsubscribe_complete_fn,
        .completion_user_data = full_test_context,
    };

    struct aws_mqtt5_client *client = full_test_context->test_fixture->client;
    ASSERT_SUCCESS(aws_mqtt5_client_unsubscribe(client, &unsubscribe_view, &completion_options));

    s_sub_pub_unsub_wait_for_unsuback_received(full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_packet_unsubscribe_storage_init(
        expected_unsubscribe_storage, full_test_context->test_fixture->allocator, &unsubscribe_view));

    return AWS_OP_SUCCESS;
}

void s_sub_pub_unsub_publish_received_fn(const struct aws_mqtt5_packet_publish_view *publish, void *complete_ctx) {

    AWS_FATAL_ASSERT(publish != NULL);

    struct aws_mqtt5_sub_pub_unsub_context *test_context = complete_ctx;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    test_context->publish_received = true;
    aws_mqtt5_packet_publish_storage_init(&test_context->publish_storage, test_fixture->allocator, publish);

    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

static bool s_sub_pub_unsub_received_publish(void *arg) {
    struct aws_mqtt5_sub_pub_unsub_context *test_context = arg;

    return test_context->publish_received && test_context->publish_failures == 0;
}

static void s_sub_pub_unsub_wait_for_publish_received(struct aws_mqtt5_sub_pub_unsub_context *test_context) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(
        &test_fixture->signal, &test_fixture->lock, s_sub_pub_unsub_received_publish, test_context);
    aws_mutex_unlock(&test_fixture->lock);
}

int aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_unsubscribe_view *unsubscribe_view = packet;

    AWS_VARIABLE_LENGTH_ARRAY(
        enum aws_mqtt5_unsuback_reason_code, mqtt5_unsuback_codes, unsubscribe_view->topic_filter_count);
    for (size_t i = 0; i < unsubscribe_view->topic_filter_count; ++i) {
        enum aws_mqtt5_unsuback_reason_code *reason_code_ptr = &mqtt5_unsuback_codes[i];
        *reason_code_ptr = AWS_MQTT5_UARC_SUCCESS;
    }

    struct aws_mqtt5_packet_unsuback_view unsuback_view = {
        .packet_id = unsubscribe_view->packet_id,
        .reason_code_count = unsubscribe_view->topic_filter_count,
        .reason_codes = mqtt5_unsuback_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_UNSUBACK, &unsuback_view);
}

#define FORWARDED_PUBLISH_PACKET_ID 32768

int aws_mqtt5_mock_server_handle_publish_puback_and_forward(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_publish_view *publish_view = packet;

    /* send a PUBACK? */
    if (publish_view->qos == AWS_MQTT5_QOS_AT_LEAST_ONCE) {
        struct aws_mqtt5_packet_puback_view puback_view = {
            .packet_id = publish_view->packet_id,
            .reason_code = AWS_MQTT5_PARC_SUCCESS,
        };

        if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBACK, &puback_view)) {
            return AWS_OP_ERR;
        }
    }

    /* assume we're subscribed, reflect the publish back to the test client */
    struct aws_mqtt5_packet_publish_view reflect_publish_view = *publish_view;
    if (publish_view->qos == AWS_MQTT5_QOS_AT_LEAST_ONCE) {
        reflect_publish_view.packet_id = FORWARDED_PUBLISH_PACKET_ID;
    }

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &reflect_publish_view);
}

void s_sub_pub_unsub_publish_complete_fn(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *complete_ctx) {

    (void)packet;
    (void)packet_type;

    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);

    struct aws_mqtt5_sub_pub_unsub_context *test_context = complete_ctx;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    test_context->publish_complete = true;
    if (error_code != AWS_ERROR_SUCCESS) {
        ++test_context->publish_failures;
    }
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

static bool s_sub_pub_unsub_publish_complete(void *arg) {
    struct aws_mqtt5_sub_pub_unsub_context *test_context = arg;

    return test_context->publish_complete;
}

static void s_sub_pub_unsub_wait_for_publish_complete(struct aws_mqtt5_sub_pub_unsub_context *test_context) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(
        &test_fixture->signal, &test_fixture->lock, s_sub_pub_unsub_publish_complete, test_context);
    aws_mutex_unlock(&test_fixture->lock);
}

static uint8_t s_sub_pub_unsub_publish_topic[] = "hello/world";
static uint8_t s_sub_pub_unsub_publish_payload[] = "PublishPayload";

static int s_mqtt5_client_sub_pub_unsub_publish(
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context,
    enum aws_mqtt5_qos qos,
    struct aws_mqtt5_packet_publish_storage *expected_publish_storage,
    struct aws_mqtt5_packet_puback_storage *expected_puback_storage) {
    struct aws_mqtt5_packet_publish_view publish_view = {
        .qos = qos,
        .topic =
            {
                .ptr = s_sub_pub_unsub_publish_topic,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
            },
        .payload =
            {
                .ptr = s_sub_pub_unsub_publish_payload,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_payload) - 1,
            },
    };

    struct aws_mqtt5_publish_completion_options completion_options = {
        .completion_callback = s_sub_pub_unsub_publish_complete_fn,
        .completion_user_data = full_test_context,
    };

    struct aws_mqtt5_client *client = full_test_context->test_fixture->client;
    ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &publish_view, &completion_options));

    if (qos != AWS_MQTT5_QOS_AT_MOST_ONCE) {
        s_sub_pub_unsub_wait_for_publish_complete(full_test_context);
    }

    struct aws_allocator *allocator = full_test_context->test_fixture->allocator;
    ASSERT_SUCCESS(aws_mqtt5_packet_publish_storage_init(expected_publish_storage, allocator, &publish_view));

    struct aws_mqtt5_packet_puback_view puback_view = {
        .packet_id = FORWARDED_PUBLISH_PACKET_ID,
        .reason_code = AWS_MQTT5_PARC_SUCCESS,
    };
    ASSERT_SUCCESS(aws_mqtt5_packet_puback_storage_init(expected_puback_storage, allocator, &puback_view));

    return AWS_OP_SUCCESS;
}

static int s_do_sub_pub_unsub_test(struct aws_allocator *allocator, enum aws_mqtt5_qos qos) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    test_options.client_options.publish_received_handler = s_sub_pub_unsub_publish_received_fn;
    test_options.client_options.publish_received_handler_user_data = &full_test_context;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        aws_mqtt5_server_send_suback_on_subscribe;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_subscribe_storage expected_subscribe_storage;
    AWS_ZERO_STRUCT(expected_subscribe_storage);
    struct aws_mqtt5_packet_publish_storage expected_publish_storage;
    AWS_ZERO_STRUCT(expected_publish_storage);
    struct aws_mqtt5_packet_puback_storage expected_puback_storage;
    AWS_ZERO_STRUCT(expected_puback_storage);
    struct aws_mqtt5_packet_unsubscribe_storage expected_unsubscribe_storage;
    AWS_ZERO_STRUCT(expected_unsubscribe_storage);

    ASSERT_SUCCESS(s_mqtt5_client_sub_pub_unsub_subscribe(&full_test_context, &expected_subscribe_storage));
    ASSERT_SUCCESS(s_mqtt5_client_sub_pub_unsub_publish(
        &full_test_context, qos, &expected_publish_storage, &expected_puback_storage));
    s_sub_pub_unsub_wait_for_publish_received(&full_test_context);
    ASSERT_SUCCESS(s_mqtt5_client_sub_pub_unsub_unsubscribe(&full_test_context, &expected_unsubscribe_storage));

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    /* verify packets that server received: connect,subscribe, publish, puback(if qos1), unsubscribe */
    struct aws_array_list expected_packets;
    aws_array_list_init_dynamic(&expected_packets, allocator, 5, sizeof(struct aws_mqtt5_mock_server_packet_record));

    struct aws_mqtt5_mock_server_packet_record connect_record = {
        .packet_type = AWS_MQTT5_PT_CONNECT,
        .packet_storage = NULL,
    };
    aws_array_list_push_back(&expected_packets, &connect_record);

    struct aws_mqtt5_mock_server_packet_record subscribe_record = {
        .packet_type = AWS_MQTT5_PT_SUBSCRIBE,
        .packet_storage = &expected_subscribe_storage,
    };
    aws_array_list_push_back(&expected_packets, &subscribe_record);

    struct aws_mqtt5_mock_server_packet_record publish_record = {
        .packet_type = AWS_MQTT5_PT_PUBLISH,
        .packet_storage = &expected_publish_storage,
    };
    aws_array_list_push_back(&expected_packets, &publish_record);

    if (qos == AWS_MQTT5_QOS_AT_LEAST_ONCE) {
        struct aws_mqtt5_mock_server_packet_record puback_record = {
            .packet_type = AWS_MQTT5_PT_PUBACK,
            .packet_storage = &expected_puback_storage,
        };
        aws_array_list_push_back(&expected_packets, &puback_record);
    }

    struct aws_mqtt5_mock_server_packet_record unsubscribe_record = {
        .packet_type = AWS_MQTT5_PT_UNSUBSCRIBE,
        .packet_storage = &expected_unsubscribe_storage,
    };
    aws_array_list_push_back(&expected_packets, &unsubscribe_record);

    ASSERT_SUCCESS(aws_verify_received_packet_sequence(
        &test_context, expected_packets.data, aws_array_list_length(&expected_packets)));

    /* verify client received the publish that we sent */
    const struct aws_mqtt5_packet_publish_view *received_publish = &full_test_context.publish_storage.storage_view;
    ASSERT_TRUE((received_publish->packet_id != 0) == (qos == AWS_MQTT5_QOS_AT_LEAST_ONCE));
    ASSERT_INT_EQUALS((uint32_t)qos, (uint32_t)received_publish->qos);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_sub_pub_unsub_publish_topic,
        AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
        received_publish->topic.ptr,
        received_publish->topic.len);

    ASSERT_BIN_ARRAYS_EQUALS(
        s_sub_pub_unsub_publish_payload,
        AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_payload) - 1,
        received_publish->payload.ptr,
        received_publish->payload.len);

    aws_mqtt5_packet_subscribe_storage_clean_up(&expected_subscribe_storage);
    aws_mqtt5_packet_publish_storage_clean_up(&expected_publish_storage);
    aws_mqtt5_packet_puback_storage_clean_up(&expected_puback_storage);
    aws_mqtt5_packet_unsubscribe_storage_clean_up(&expected_unsubscribe_storage);
    aws_array_list_clean_up(&expected_packets);

    s_sub_pub_unsub_context_clean_up(&full_test_context);
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_sub_pub_unsub_qos0_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_sub_pub_unsub_test(allocator, AWS_MQTT5_QOS_AT_MOST_ONCE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_sub_pub_unsub_qos0, s_mqtt5_client_sub_pub_unsub_qos0_fn)

static int s_mqtt5_client_sub_pub_unsub_qos1_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_sub_pub_unsub_test(allocator, AWS_MQTT5_QOS_AT_LEAST_ONCE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_sub_pub_unsub_qos1, s_mqtt5_client_sub_pub_unsub_qos1_fn)

static enum aws_mqtt5_unsuback_reason_code s_unsubscribe_success_reason_codes[] = {
    AWS_MQTT5_UARC_NO_SUBSCRIPTION_EXISTED,
};

static int s_aws_mqtt5_server_send_not_subscribe_unsuback_on_unsubscribe(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)user_data;

    struct aws_mqtt5_packet_unsubscribe_view *unsubscribe_view = packet;

    struct aws_mqtt5_packet_unsuback_view unsuback_view = {
        .packet_id = unsubscribe_view->packet_id,
        .reason_code_count = AWS_ARRAY_SIZE(s_unsubscribe_success_reason_codes),
        .reason_codes = s_unsubscribe_success_reason_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_UNSUBACK, &unsuback_view);
}

static int s_mqtt5_client_unsubscribe_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        s_aws_mqtt5_server_send_not_subscribe_unsuback_on_unsubscribe;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_unsubscribe_view unsubscribe_view = {
        .topic_filters = s_sub_pub_unsub_topic_filters,
        .topic_filter_count = AWS_ARRAY_SIZE(s_sub_pub_unsub_topic_filters),
    };

    struct aws_mqtt5_unsubscribe_completion_options completion_options = {
        .completion_callback = s_sub_pub_unsub_unsubscribe_complete_fn,
        .completion_user_data = &full_test_context,
    };
    aws_mqtt5_client_unsubscribe(client, &unsubscribe_view, &completion_options);

    s_sub_pub_unsub_wait_for_unsuback_received(&full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_unsubscribe_success, s_mqtt5_client_unsubscribe_success_fn)

static aws_mqtt5_packet_id_t s_puback_packet_id = 183;

struct aws_mqtt5_server_send_qos1_publish_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
    bool publish_sent;
    bool connack_sent;
    bool connack_checked;
};

static int s_aws_mqtt5_mock_server_handle_puback(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)user_data;

    struct aws_mqtt5_packet_puback_view *puback_view = packet;

    ASSERT_INT_EQUALS(puback_view->packet_id, s_puback_packet_id);
    ASSERT_TRUE(puback_view->reason_code == AWS_MQTT5_PARC_SUCCESS);

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = connection->test_fixture;
    struct aws_mqtt5_server_send_qos1_publish_context *publish_context =
        connection->test_fixture->mock_server_user_data;
    aws_mutex_lock(&test_fixture->lock);
    publish_context->connack_checked = true;
    aws_mutex_unlock(&test_fixture->lock);

    return AWS_OP_SUCCESS;
}

static void s_aws_mqtt5_mock_server_send_qos1_publish(
    struct aws_mqtt5_server_mock_connection_context *mock_server,
    void *user_data) {

    struct aws_mqtt5_server_send_qos1_publish_context *test_context = user_data;
    if (test_context->publish_sent || !test_context->connack_sent) {
        return;
    }

    test_context->publish_sent = true;

    struct aws_mqtt5_packet_publish_view qos1_publish_view = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic =
            {
                .ptr = s_topic,
                .len = AWS_ARRAY_SIZE(s_topic) - 1,
            },
        .packet_id = s_puback_packet_id,
    };

    aws_mqtt5_mock_server_send_packet(mock_server, AWS_MQTT5_PT_PUBLISH, &qos1_publish_view);
}

static int s_aws_mqtt5_server_send_qos1_publish_on_connect(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    int result = aws_mqtt5_mock_server_handle_connect_always_succeed(packet, connection, user_data);

    struct aws_mqtt5_server_send_qos1_publish_context *test_context = user_data;
    test_context->connack_sent = true;

    return result;
}

static bool s_publish_qos1_puback(void *arg) {
    struct aws_mqtt5_server_send_qos1_publish_context *test_context = arg;
    return test_context->connack_checked;
}

static void s_publish_qos1_wait_for_puback(struct aws_mqtt5_server_send_qos1_publish_context *test_context) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;
    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(&test_fixture->signal, &test_fixture->lock, s_publish_qos1_puback, test_context);
    aws_mutex_unlock(&test_fixture->lock);
}

/* When client receives a QoS1 PUBLISH it must send a valid PUBACK with packet id */
static int mqtt5_client_receive_qos1_return_puback_test_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* mock server sends a PUBLISH packet to the client */
    test_options.server_function_table.service_task_fn = s_aws_mqtt5_mock_server_send_qos1_publish;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBACK] = s_aws_mqtt5_mock_server_handle_puback;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_server_send_qos1_publish_on_connect;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_send_qos1_publish_context publish_context = {
        .test_fixture = &test_context,
        .publish_sent = false,
        .connack_sent = false,
        .connack_checked = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &publish_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    s_publish_qos1_wait_for_puback(&publish_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_receive_qos1_return_puback_test, mqtt5_client_receive_qos1_return_puback_test_fn)

static int s_aws_mqtt5_mock_server_handle_connect_session_present(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    connack_view.session_present = true;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

/* When client receives a CONNACK with existing session state when one isn't present it should disconnect */
static int mqtt5_client_receive_nonexisting_session_state_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* mock server returns a CONNACK indicating a session is being resumed */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_session_present;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_client_mock_test_fixture test_context;
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    s_wait_for_connection_failure_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
    };
    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_receive_nonexisting_session_state, mqtt5_client_receive_nonexisting_session_state_fn)

static const char *s_receive_assigned_client_id_client_id = "Assigned_Client_ID";

struct aws_mqtt5_server_receive_assigned_client_id_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
    bool assigned_client_id_checked;
};

static int s_aws_mqtt5_mock_server_handle_connect_assigned_client_id(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)user_data;

    struct aws_mqtt5_packet_connect_view *connect_view = packet;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    struct aws_byte_cursor assigned_client_id = aws_byte_cursor_from_c_str(s_receive_assigned_client_id_client_id);

    /* Server behavior sets the Assigned Client Identifier on a CONNECT packet with an empty Client ID */
    if (connect_view->client_id.len == 0) {
        connack_view.assigned_client_identifier = &assigned_client_id;
    } else {
        ASSERT_BIN_ARRAYS_EQUALS(
            assigned_client_id.ptr, assigned_client_id.len, connect_view->client_id.ptr, connect_view->client_id.len);
        struct aws_mqtt5_server_receive_assigned_client_id_context *test_context = user_data;
        test_context->assigned_client_id_checked = true;
    }

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

/*
 * When client connects with a zero length Client ID, server provides one.
 * The client should then use the assigned Client ID on reconnection attempts.
 */
static int mqtt5_client_receive_assigned_client_id_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* Empty the Client ID for connect */
    test_options.connect_options.client_id.len = 0;

    /* mock server checks for a client ID and if it's missing sends an assigned one */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_assigned_client_id;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_server_receive_assigned_client_id_context assinged_id_context = {
        .test_fixture = &test_context,
        .assigned_client_id_checked = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &assinged_id_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_byte_cursor assigned_client_id = aws_byte_cursor_from_c_str(s_receive_assigned_client_id_client_id);
    struct aws_byte_cursor negotiated_settings_client_id =
        aws_byte_cursor_from_buf(&client->negotiated_settings.client_id_storage);
    /* Test that Assigned Client ID is stored */
    ASSERT_BIN_ARRAYS_EQUALS(
        assigned_client_id.ptr,
        assigned_client_id.len,
        negotiated_settings_client_id.ptr,
        negotiated_settings_client_id.len);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    /* Check for Assigned Client ID on reconnect */
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    ASSERT_TRUE(assinged_id_context.assigned_client_id_checked);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_receive_assigned_client_id, mqtt5_client_receive_assigned_client_id_fn);

#define TEST_PUBLISH_COUNT 10

static int s_aws_mqtt5_mock_server_handle_publish_no_puback_on_first_connect(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)user_data;

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = connection->test_fixture;
    struct aws_mqtt5_packet_publish_view *publish_view = packet;

    aws_mutex_lock(&test_fixture->lock);
    ++connection->test_fixture->publishes_received;
    aws_mutex_unlock(&test_fixture->lock);

    /* Only send the PUBACK on the second attempt after a reconnect and restored session */
    if (publish_view->duplicate) {
        struct aws_mqtt5_packet_puback_view puback_view = {
            .packet_id = publish_view->packet_id,
        };
        return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBACK, &puback_view);
    }

    return AWS_OP_SUCCESS;
}

static void s_receive_stored_session_publish_completion_fn(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *complete_ctx) {

    if (packet_type != AWS_MQTT5_PT_PUBACK) {
        return;
    }

    const struct aws_mqtt5_packet_puback_view *puback = packet;

    struct aws_mqtt5_client_mock_test_fixture *test_context = complete_ctx;

    aws_mutex_lock(&test_context->lock);

    if (error_code == AWS_ERROR_SUCCESS && puback->reason_code < 128) {
        ++test_context->successful_pubacks_received;
    }

    aws_mutex_unlock(&test_context->lock);
    aws_condition_variable_notify_all(&test_context->signal);
}

static bool s_received_n_unacked_publishes(void *arg) {
    struct aws_mqtt5_client_test_wait_for_n_context *context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = context->test_fixture;

    return test_fixture->publishes_received >= context->required_event_count;
}

static void s_wait_for_n_unacked_publishes(struct aws_mqtt5_client_test_wait_for_n_context *context) {

    struct aws_mqtt5_client_mock_test_fixture *test_context = context->test_fixture;
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_received_n_unacked_publishes, context);
    aws_mutex_unlock(&test_context->lock);
}

static int mqtt5_client_no_session_after_client_stop_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* Set to rejoin */
    test_options.client_options.session_behavior = AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS;

    /* mock server will not send PUBACKS on initial connect */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_mock_server_handle_publish_no_puback_on_first_connect;
    /* Simulate reconnecting to an existing connection */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_honor_session_after_success;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    for (size_t i = 0; i < TEST_PUBLISH_COUNT; ++i) {
        struct aws_mqtt5_packet_publish_view qos1_publish_view = {
            .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
            .topic =
                {
                    .ptr = s_topic,
                    .len = AWS_ARRAY_SIZE(s_topic) - 1,
                },
        };

        struct aws_mqtt5_publish_completion_options completion_options = {
            .completion_callback = s_receive_stored_session_publish_completion_fn,
            .completion_user_data = &test_context,
        };

        ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish_view, &completion_options));
    }

    /* Wait for publishes to have gone out from client */
    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = TEST_PUBLISH_COUNT,
    };
    s_wait_for_n_unacked_publishes(&wait_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    aws_mutex_lock(&test_context.lock);
    size_t event_count = aws_array_list_length(&test_context.lifecycle_events);
    struct aws_mqtt5_lifecycle_event_record *record = NULL;
    aws_array_list_get_at(&test_context.lifecycle_events, &record, event_count - 1);
    aws_mutex_unlock(&test_context.lock);

    ASSERT_FALSE(record->connack_storage.storage_view.session_present);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));
    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_no_session_after_client_stop, mqtt5_client_no_session_after_client_stop_fn);

static int mqtt5_client_restore_session_on_ping_timeout_reconnect_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* Set to rejoin */
    test_options.client_options.session_behavior = AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS;
    /* faster ping timeout */
    test_options.client_options.ping_timeout_ms = 3000;
    test_options.connect_options.keep_alive_interval_seconds = 5;

    /* don't respond to PINGREQs */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PINGREQ] = NULL;
    /* mock server will not send PUBACKS on initial connect */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_mock_server_handle_publish_no_puback_on_first_connect;
    /* Simulate reconnecting to an existing connection */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_honor_session_after_success;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    for (size_t i = 0; i < TEST_PUBLISH_COUNT; ++i) {
        struct aws_mqtt5_packet_publish_view qos1_publish_view = {
            .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
            .topic =
                {
                    .ptr = s_topic,
                    .len = AWS_ARRAY_SIZE(s_topic) - 1,
                },
        };

        struct aws_mqtt5_publish_completion_options completion_options = {
            .completion_callback = s_receive_stored_session_publish_completion_fn,
            .completion_user_data = &test_context,
        };

        ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish_view, &completion_options));
    }

    /* Wait for publishes to have gone out from client */
    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = TEST_PUBLISH_COUNT,
    };
    s_wait_for_n_unacked_publishes(&wait_context);

    /* disconnect due to failed ping */
    s_wait_for_disconnection_lifecycle_event(&test_context);

    /* Reconnect from a disconnect automatically */
    aws_wait_for_connected_lifecycle_event(&test_context);

    s_wait_for_n_successful_publishes(&wait_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));
    aws_wait_for_stopped_lifecycle_event(&test_context);

    enum aws_mqtt5_client_state expected_states[] = {
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CONNECTED,
        AWS_MCS_CLEAN_DISCONNECT,
        AWS_MCS_CHANNEL_SHUTDOWN,
        AWS_MCS_PENDING_RECONNECT,
        AWS_MCS_CONNECTING,
        AWS_MCS_MQTT_CONNECT,
        AWS_MCS_CONNECTED,
        AWS_MCS_CHANNEL_SHUTDOWN,
        AWS_MCS_STOPPED,
    };

    ASSERT_SUCCESS(aws_verify_client_state_sequence(&test_context, expected_states, AWS_ARRAY_SIZE(expected_states)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_restore_session_on_ping_timeout_reconnect,
    mqtt5_client_restore_session_on_ping_timeout_reconnect_fn);

/* If the server returns a Clean Session, client must discard any existing Session and start a new Session */
static int mqtt5_client_discard_session_on_server_clean_start_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* Set to rejoin */
    test_options.client_options.session_behavior = AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS;

    /* mock server will not send PUBACKS on initial connect */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_mock_server_handle_publish_no_puback_on_first_connect;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    for (size_t i = 0; i < TEST_PUBLISH_COUNT; ++i) {
        struct aws_mqtt5_packet_publish_view qos1_publish_view = {
            .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
            .topic =
                {
                    .ptr = s_topic,
                    .len = AWS_ARRAY_SIZE(s_topic) - 1,
                },
        };

        struct aws_mqtt5_publish_completion_options completion_options = {
            .completion_callback = s_receive_stored_session_publish_completion_fn,
            .completion_user_data = &test_context,
        };

        ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish_view, &completion_options));
    }

    /* Wait for QoS1 publishes to have gone out from client */
    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = TEST_PUBLISH_COUNT,
    };
    s_wait_for_n_unacked_publishes(&wait_context);

    /* Disconnect with unacked publishes */
    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));
    aws_wait_for_stopped_lifecycle_event(&test_context);

    /* Reconnect with a Client Stored Session */
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));
    aws_wait_for_connected_lifecycle_event(&test_context);

    /* Provide time for Client to process any queued operations */
    aws_thread_current_sleep(1000000000);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));
    aws_wait_for_stopped_lifecycle_event(&test_context);

    /* Check that no publishes were resent after the initial batch on first connect */
    ASSERT_INT_EQUALS(test_context.publishes_received, TEST_PUBLISH_COUNT);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_discard_session_on_server_clean_start,
    mqtt5_client_discard_session_on_server_clean_start_fn);

static int s_verify_zero_statistics(struct aws_mqtt5_client_operation_statistics *stats) {
    ASSERT_INT_EQUALS(stats->incomplete_operation_size, 0);
    ASSERT_INT_EQUALS(stats->incomplete_operation_count, 0);
    ASSERT_INT_EQUALS(stats->unacked_operation_size, 0);
    ASSERT_INT_EQUALS(stats->unacked_operation_count, 0);

    return AWS_OP_SUCCESS;
}

static int s_verify_statistics_equal(
    struct aws_mqtt5_client_operation_statistics *expected_stats,
    struct aws_mqtt5_client_operation_statistics *actual_stats) {
    ASSERT_INT_EQUALS(expected_stats->incomplete_operation_size, actual_stats->incomplete_operation_size);
    ASSERT_INT_EQUALS(expected_stats->incomplete_operation_count, actual_stats->incomplete_operation_count);
    ASSERT_INT_EQUALS(expected_stats->unacked_operation_size, actual_stats->unacked_operation_size);
    ASSERT_INT_EQUALS(expected_stats->unacked_operation_size, actual_stats->unacked_operation_size);

    return AWS_OP_SUCCESS;
}

static int s_verify_client_statistics(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    struct aws_mqtt5_client_operation_statistics *expected_stats,
    size_t expected_stats_count) {
    struct aws_array_list *actual_stats = &test_context->client_statistics;
    size_t actual_stats_count = aws_array_list_length(actual_stats);

    /* we expect the last stats to be zero, the expected stats represent the stats before that */
    ASSERT_INT_EQUALS(actual_stats_count, expected_stats_count + 1);

    struct aws_mqtt5_client_operation_statistics *current_stats = NULL;
    aws_array_list_get_at_ptr(actual_stats, (void **)&current_stats, actual_stats_count - 1);
    ASSERT_SUCCESS(s_verify_zero_statistics(current_stats));

    for (size_t i = 0; i < expected_stats_count; ++i) {
        aws_array_list_get_at_ptr(actual_stats, (void **)&current_stats, i);
        ASSERT_SUCCESS(s_verify_statistics_equal(&expected_stats[i], current_stats));
    }

    return AWS_OP_SUCCESS;
}

static struct aws_mqtt5_client_operation_statistics s_subscribe_test_statistics[] = {
    {
        .incomplete_operation_size = 68,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 0,
        .unacked_operation_count = 0,
    },
    {
        .incomplete_operation_size = 68,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 68,
        .unacked_operation_count = 1,
    },
};

static int s_mqtt5_client_statistics_subscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        aws_mqtt5_server_send_suback_on_subscribe;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_server_disconnect_test_context disconnect_context = {
        .test_fixture = &test_context,
        .disconnect_sent = false,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &disconnect_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .subscriptions = s_subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(s_subscriptions),
    };

    struct aws_mqtt5_subscribe_completion_options completion_options = {
        .completion_callback = s_aws_mqtt5_subscribe_complete_fn,
        .completion_user_data = &test_context,
    };
    aws_mqtt5_client_subscribe(client, &subscribe_view, &completion_options);

    s_wait_for_suback_received(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    ASSERT_SUCCESS(s_verify_client_statistics(
        &test_context, s_subscribe_test_statistics, AWS_ARRAY_SIZE(s_subscribe_test_statistics)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_statistics_subscribe, s_mqtt5_client_statistics_subscribe_fn)

static struct aws_mqtt5_client_operation_statistics s_unsubscribe_test_statistics[] = {
    {
        .incomplete_operation_size = 14,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 0,
        .unacked_operation_count = 0,
    },
    {
        .incomplete_operation_size = 14,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 14,
        .unacked_operation_count = 1,
    },
};

static int s_mqtt5_client_statistics_unsubscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        s_aws_mqtt5_server_send_not_subscribe_unsuback_on_unsubscribe;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_unsubscribe_view unsubscribe_view = {
        .topic_filters = s_sub_pub_unsub_topic_filters,
        .topic_filter_count = AWS_ARRAY_SIZE(s_sub_pub_unsub_topic_filters),
    };

    struct aws_mqtt5_unsubscribe_completion_options completion_options = {
        .completion_callback = s_sub_pub_unsub_unsubscribe_complete_fn,
        .completion_user_data = &full_test_context,
    };
    aws_mqtt5_client_unsubscribe(client, &unsubscribe_view, &completion_options);

    s_sub_pub_unsub_wait_for_unsuback_received(&full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    ASSERT_SUCCESS(s_verify_client_statistics(
        &test_context, s_unsubscribe_test_statistics, AWS_ARRAY_SIZE(s_unsubscribe_test_statistics)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_statistics_unsubscribe, s_mqtt5_client_statistics_unsubscribe_fn)

static struct aws_mqtt5_client_operation_statistics s_publish_qos1_test_statistics[] = {
    {
        .incomplete_operation_size = 30,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 0,
        .unacked_operation_count = 0,
    },
    {
        .incomplete_operation_size = 30,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 30,
        .unacked_operation_count = 1,
    },
};

static int s_do_mqtt5_client_statistics_publish_test(
    struct aws_allocator *allocator,
    enum aws_mqtt5_qos qos,
    struct aws_mqtt5_client_operation_statistics *expected_stats,
    size_t expected_stats_count) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_publish_view publish_view = {
        .qos = qos,
        .topic =
            {
                .ptr = s_sub_pub_unsub_publish_topic,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
            },
        .payload =
            {
                .ptr = s_sub_pub_unsub_publish_payload,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_payload) - 1,
            },
    };

    struct aws_mqtt5_publish_completion_options completion_options = {
        .completion_callback = s_sub_pub_unsub_publish_complete_fn,
        .completion_user_data = &full_test_context,
    };
    aws_mqtt5_client_publish(client, &publish_view, &completion_options);

    s_sub_pub_unsub_wait_for_publish_complete(&full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    ASSERT_SUCCESS(s_verify_client_statistics(&test_context, expected_stats, expected_stats_count));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_statistics_publish_qos1_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5_client_statistics_publish_test(
        allocator,
        AWS_MQTT5_QOS_AT_LEAST_ONCE,
        s_publish_qos1_test_statistics,
        AWS_ARRAY_SIZE(s_publish_qos1_test_statistics)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_statistics_publish_qos1, s_mqtt5_client_statistics_publish_qos1_fn)

static struct aws_mqtt5_client_operation_statistics s_publish_qos0_test_statistics[] = {
    {
        .incomplete_operation_size = 30,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 0,
        .unacked_operation_count = 0,
    },
};

static int s_mqtt5_client_statistics_publish_qos0_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5_client_statistics_publish_test(
        allocator,
        AWS_MQTT5_QOS_AT_MOST_ONCE,
        s_publish_qos0_test_statistics,
        AWS_ARRAY_SIZE(s_publish_qos0_test_statistics)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_statistics_publish_qos0, s_mqtt5_client_statistics_publish_qos0_fn)

static struct aws_mqtt5_client_operation_statistics s_publish_qos1_requeue_test_statistics[] = {
    {
        .incomplete_operation_size = 30,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 0,
        .unacked_operation_count = 0,
    },
    {
        .incomplete_operation_size = 30,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 30,
        .unacked_operation_count = 1,
    },
    {
        .incomplete_operation_size = 30,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 0,
        .unacked_operation_count = 0,
    },
    {
        .incomplete_operation_size = 30,
        .incomplete_operation_count = 1,
        .unacked_operation_size = 30,
        .unacked_operation_count = 1,
    },
};

static int s_aws_mqtt5_server_disconnect_on_first_publish_puback_after(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct aws_mqtt5_sub_pub_unsub_context *test_context = user_data;
    ++test_context->publishes_received;
    if (test_context->publishes_received == 1) {
        return AWS_OP_ERR;
    }

    struct aws_mqtt5_packet_publish_view *publish_view = packet;

    /* send a PUBACK? */
    if (publish_view->qos == AWS_MQTT5_QOS_AT_LEAST_ONCE) {
        struct aws_mqtt5_packet_puback_view puback_view = {
            .packet_id = publish_view->packet_id,
            .reason_code = AWS_MQTT5_PARC_SUCCESS,
        };

        if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBACK, &puback_view)) {
            return AWS_OP_ERR;
        }
    }

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_statistics_publish_qos1_requeue_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.client_options.session_behavior = AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_server_disconnect_on_first_publish_puback_after;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_honor_session_after_success;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_publish_view publish_view = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic =
            {
                .ptr = s_sub_pub_unsub_publish_topic,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
            },
        .payload =
            {
                .ptr = s_sub_pub_unsub_publish_payload,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_payload) - 1,
            },
    };

    struct aws_mqtt5_publish_completion_options completion_options = {
        .completion_callback = s_sub_pub_unsub_publish_complete_fn,
        .completion_user_data = &full_test_context,
    };
    aws_mqtt5_client_publish(client, &publish_view, &completion_options);

    s_sub_pub_unsub_wait_for_publish_complete(&full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    ASSERT_SUCCESS(s_verify_client_statistics(
        &test_context, s_publish_qos1_requeue_test_statistics, AWS_ARRAY_SIZE(s_publish_qos1_requeue_test_statistics)));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_statistics_publish_qos1_requeue, s_mqtt5_client_statistics_publish_qos1_requeue_fn)

#define PUBACK_ORDERING_PUBLISH_COUNT 5

static int s_aws_mqtt5_server_send_multiple_publishes(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    /* in order: send PUBLISH packet ids 1, 2, 3, 4, 5 */
    for (size_t i = 0; i < PUBACK_ORDERING_PUBLISH_COUNT; ++i) {
        struct aws_mqtt5_packet_publish_view publish_view = {
            .packet_id = (uint16_t)i + 1,
            .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
            .topic =
                {
                    .ptr = s_sub_pub_unsub_publish_topic,
                    .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
                },
            .payload =
                {
                    .ptr = s_sub_pub_unsub_publish_payload,
                    .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_payload) - 1,
                },
        };

        if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &publish_view)) {
            return AWS_OP_ERR;
        }
    }

    return AWS_OP_SUCCESS;
}

static bool s_server_received_all_pubacks(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;

    size_t pubacks_received = 0;
    size_t packet_count = aws_array_list_length(&test_fixture->server_received_packets);
    for (size_t i = 0; i < packet_count; ++i) {
        struct aws_mqtt5_mock_server_packet_record *packet = NULL;
        aws_array_list_get_at_ptr(&test_fixture->server_received_packets, (void **)&packet, i);
        if (packet->packet_type == AWS_MQTT5_PT_PUBACK) {
            ++pubacks_received;
        }
    }

    return pubacks_received == PUBACK_ORDERING_PUBLISH_COUNT;
}

static void s_wait_for_mock_server_pubacks(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(
        &test_context->signal, &test_context->lock, s_server_received_all_pubacks, test_context);
    aws_mutex_unlock(&test_context->lock);
}

static int s_verify_mock_puback_order(struct aws_mqtt5_client_mock_test_fixture *test_context) {

    aws_mutex_lock(&test_context->lock);

    uint16_t expected_packet_id = 1;
    size_t packet_count = aws_array_list_length(&test_context->server_received_packets);

    /* in order: received PUBACK packet ids 1, 2, 3, 4, 5 */
    for (size_t i = 0; i < packet_count; ++i) {
        struct aws_mqtt5_mock_server_packet_record *packet = NULL;
        aws_array_list_get_at_ptr(&test_context->server_received_packets, (void **)&packet, i);
        if (packet->packet_type == AWS_MQTT5_PT_PUBACK) {
            struct aws_mqtt5_packet_puback_view *puback_view =
                &((struct aws_mqtt5_packet_puback_storage *)(packet->packet_storage))->storage_view;
            ASSERT_INT_EQUALS(expected_packet_id, puback_view->packet_id);
            ++expected_packet_id;
        }
    }

    ASSERT_INT_EQUALS(PUBACK_ORDERING_PUBLISH_COUNT + 1, expected_packet_id);

    aws_mutex_unlock(&test_context->lock);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_puback_ordering_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_server_send_multiple_publishes;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_publish_view publish_view = {
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .topic =
            {
                .ptr = s_sub_pub_unsub_publish_topic,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
            },
        .payload =
            {
                .ptr = s_sub_pub_unsub_publish_payload,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_payload) - 1,
            },
    };

    aws_mqtt5_client_publish(client, &publish_view, NULL);

    s_wait_for_mock_server_pubacks(&test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    ASSERT_SUCCESS(s_verify_mock_puback_order(&test_context));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_puback_ordering, s_mqtt5_client_puback_ordering_fn)

enum aws_mqtt5_listener_test_publish_received_callback_type {
    AWS_MQTT5_LTPRCT_DEFAULT,
    AWS_MQTT5_LTPRCT_LISTENER,
};

struct aws_mqtt5_listeners_test_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
    struct aws_array_list publish_received_callback_types;
    struct aws_mutex lock;
    struct aws_condition_variable signal;
};

static void s_aws_mqtt5_listeners_test_context_init(
    struct aws_mqtt5_listeners_test_context *listener_test_context,
    struct aws_allocator *allocator) {
    AWS_ZERO_STRUCT(*listener_test_context);

    aws_array_list_init_dynamic(
        &listener_test_context->publish_received_callback_types,
        allocator,
        0,
        sizeof(enum aws_mqtt5_listener_test_publish_received_callback_type));
    aws_mutex_init(&listener_test_context->lock);
    aws_condition_variable_init(&listener_test_context->signal);
}

static void s_aws_mqtt5_listeners_test_context_clean_up(
    struct aws_mqtt5_listeners_test_context *listener_test_context) {
    aws_condition_variable_clean_up(&listener_test_context->signal);
    aws_mutex_clean_up(&listener_test_context->lock);
    aws_array_list_clean_up(&listener_test_context->publish_received_callback_types);
}

static int s_aws_mqtt5_mock_server_reflect_publish(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)user_data;

    struct aws_mqtt5_packet_publish_view *publish_view = packet;
    struct aws_mqtt5_packet_publish_view reflected_view = *publish_view;

    if (reflected_view.qos != AWS_MQTT5_QOS_AT_MOST_ONCE) {
        reflected_view.packet_id = 1;
    }

    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &reflected_view)) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

static void s_aws_mqtt5_listeners_test_publish_received_default_handler(
    const struct aws_mqtt5_packet_publish_view *publish_view,
    void *user_data) {
    (void)publish_view;

    struct aws_mqtt5_listeners_test_context *context = user_data;

    aws_mutex_lock(&context->lock);
    enum aws_mqtt5_listener_test_publish_received_callback_type callback_type = AWS_MQTT5_LTPRCT_DEFAULT;
    aws_array_list_push_back(&context->publish_received_callback_types, &callback_type);
    aws_mutex_unlock(&context->lock);
    aws_condition_variable_notify_all(&context->signal);
}

static bool s_aws_mqtt5_listeners_test_publish_received_listener_handler(
    const struct aws_mqtt5_packet_publish_view *publish_view,
    void *user_data) {
    struct aws_mqtt5_listeners_test_context *context = user_data;

    aws_mutex_lock(&context->lock);
    enum aws_mqtt5_listener_test_publish_received_callback_type callback_type = AWS_MQTT5_LTPRCT_LISTENER;
    aws_array_list_push_back(&context->publish_received_callback_types, &callback_type);
    aws_mutex_unlock(&context->lock);
    aws_condition_variable_notify_all(&context->signal);

    return publish_view->qos == AWS_MQTT5_QOS_AT_LEAST_ONCE;
}

struct aws_mqtt5_listeners_test_wait_context {
    size_t callback_count;
    struct aws_mqtt5_listeners_test_context *test_fixture;
};

static bool s_aws_mqtt5_listeners_test_wait_on_callback_count(void *context) {
    struct aws_mqtt5_listeners_test_wait_context *wait_context = context;
    return wait_context->callback_count ==
           aws_array_list_length(&wait_context->test_fixture->publish_received_callback_types);
}

static int s_aws_mqtt5_listeners_test_wait_on_and_verify_callbacks(
    struct aws_mqtt5_listeners_test_context *context,
    size_t callback_count,
    enum aws_mqtt5_listener_test_publish_received_callback_type *callback_types) {
    struct aws_mqtt5_listeners_test_wait_context wait_context = {
        .callback_count = callback_count,
        .test_fixture = context,
    };

    aws_mutex_lock(&context->lock);
    aws_condition_variable_wait_pred(
        &context->signal, &context->lock, s_aws_mqtt5_listeners_test_wait_on_callback_count, &wait_context);
    for (size_t i = 0; i < callback_count; ++i) {
        enum aws_mqtt5_listener_test_publish_received_callback_type callback_type;
        aws_array_list_get_at(&context->publish_received_callback_types, &callback_type, i);

        ASSERT_INT_EQUALS(callback_types[i], callback_type);
    }
    aws_mutex_unlock(&context->lock);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_listeners_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] = s_aws_mqtt5_mock_server_reflect_publish;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_listeners_test_context full_test_context = {
        .test_fixture = &test_context,
    };
    s_aws_mqtt5_listeners_test_context_init(&full_test_context, allocator);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    test_fixture_options.client_options->publish_received_handler =
        s_aws_mqtt5_listeners_test_publish_received_default_handler;
    test_fixture_options.client_options->publish_received_handler_user_data = &full_test_context;

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_publish_view qos0_publish_view = {
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .topic =
            {
                .ptr = s_sub_pub_unsub_publish_topic,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
            },
    };

    struct aws_mqtt5_packet_publish_view qos1_publish_view = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic =
            {
                .ptr = s_sub_pub_unsub_publish_topic,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
            },
    };

    // send a qos 0 publish, wait for it to reflect, verify it's the default handler
    aws_mqtt5_client_publish(client, &qos0_publish_view, NULL);

    enum aws_mqtt5_listener_test_publish_received_callback_type first_callback_type_array[] = {
        AWS_MQTT5_LTPRCT_DEFAULT,
    };
    ASSERT_SUCCESS(s_aws_mqtt5_listeners_test_wait_on_and_verify_callbacks(
        &full_test_context, AWS_ARRAY_SIZE(first_callback_type_array), first_callback_type_array));

    // attach a listener at the beginning of the handler chain
    struct aws_mqtt5_listener_config listener_config = {
        .client = client,
        .listener_callbacks = {
            .listener_publish_received_handler_user_data = &full_test_context,
            .listener_publish_received_handler = s_aws_mqtt5_listeners_test_publish_received_listener_handler,
        }};
    struct aws_mqtt5_listener *listener = aws_mqtt5_listener_new(allocator, &listener_config);

    // send a qos 0 publish, wait for it to reflect, verify both handlers were invoked in the proper order
    aws_mqtt5_client_publish(client, &qos0_publish_view, NULL);

    enum aws_mqtt5_listener_test_publish_received_callback_type second_callback_type_array[] = {
        AWS_MQTT5_LTPRCT_DEFAULT,
        AWS_MQTT5_LTPRCT_LISTENER,
        AWS_MQTT5_LTPRCT_DEFAULT,
    };
    ASSERT_SUCCESS(s_aws_mqtt5_listeners_test_wait_on_and_verify_callbacks(
        &full_test_context, AWS_ARRAY_SIZE(second_callback_type_array), second_callback_type_array));

    // send a qos1 publish (which is short-circuited by the listener), verify just the listener was notified
    aws_mqtt5_client_publish(client, &qos1_publish_view, NULL);

    enum aws_mqtt5_listener_test_publish_received_callback_type third_callback_type_array[] = {
        AWS_MQTT5_LTPRCT_DEFAULT,
        AWS_MQTT5_LTPRCT_LISTENER,
        AWS_MQTT5_LTPRCT_DEFAULT,
        AWS_MQTT5_LTPRCT_LISTENER,
    };
    ASSERT_SUCCESS(s_aws_mqtt5_listeners_test_wait_on_and_verify_callbacks(
        &full_test_context, AWS_ARRAY_SIZE(third_callback_type_array), third_callback_type_array));

    // remove the listener
    aws_mqtt5_listener_release(listener);

    // send a qos1 publish, wait for it to reflect, verify it's the default handler
    aws_mqtt5_client_publish(client, &qos1_publish_view, NULL);

    enum aws_mqtt5_listener_test_publish_received_callback_type fourth_callback_type_array[] = {
        AWS_MQTT5_LTPRCT_DEFAULT,
        AWS_MQTT5_LTPRCT_LISTENER,
        AWS_MQTT5_LTPRCT_DEFAULT,
        AWS_MQTT5_LTPRCT_LISTENER,
        AWS_MQTT5_LTPRCT_DEFAULT,
    };
    ASSERT_SUCCESS(s_aws_mqtt5_listeners_test_wait_on_and_verify_callbacks(
        &full_test_context, AWS_ARRAY_SIZE(fourth_callback_type_array), fourth_callback_type_array));

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    s_aws_mqtt5_listeners_test_context_clean_up(&full_test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_listeners, s_mqtt5_client_listeners_fn)

static void s_on_offline_publish_completion(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *user_data) {
    (void)packet_type;
    (void)packet;

    struct aws_mqtt5_sub_pub_unsub_context *full_test_context = user_data;

    aws_mutex_lock(&full_test_context->test_fixture->lock);

    if (error_code == AWS_ERROR_MQTT5_OPERATION_FAILED_DUE_TO_OFFLINE_QUEUE_POLICY) {
        ++full_test_context->publish_failures;
    } else if (error_code == 0) {
        full_test_context->publish_complete = true;
    }

    aws_mutex_unlock(&full_test_context->test_fixture->lock);
    aws_condition_variable_notify_all(&full_test_context->test_fixture->signal);
}

static bool s_has_failed_publishes(void *user_data) {
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context = user_data;

    return full_test_context->publish_failures > 0;
}

static void s_aws_mqtt5_wait_for_publish_failure(struct aws_mqtt5_sub_pub_unsub_context *full_test_context) {
    aws_mutex_lock(&full_test_context->test_fixture->lock);
    aws_condition_variable_wait_pred(
        &full_test_context->test_fixture->signal,
        &full_test_context->test_fixture->lock,
        s_has_failed_publishes,
        full_test_context);
    aws_mutex_unlock(&full_test_context->test_fixture->lock);
}

static int s_offline_publish(
    struct aws_mqtt5_client *client,
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context,
    enum aws_mqtt5_qos qos) {
    struct aws_mqtt5_packet_publish_view publish_view = {
        .qos = qos,
        .topic =
            {
                .ptr = s_sub_pub_unsub_publish_topic,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_topic) - 1,
            },
        .payload =
            {
                .ptr = s_sub_pub_unsub_publish_payload,
                .len = AWS_ARRAY_SIZE(s_sub_pub_unsub_publish_payload) - 1,
            },
    };

    struct aws_mqtt5_publish_completion_options completion_options = {
        .completion_callback = s_on_offline_publish_completion,
        .completion_user_data = full_test_context,
    };

    return aws_mqtt5_client_publish(client, &publish_view, &completion_options);
}

static int s_verify_offline_publish_failure(
    struct aws_mqtt5_client *client,
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context,
    enum aws_mqtt5_qos qos) {

    aws_mutex_lock(&full_test_context->test_fixture->lock);
    full_test_context->publish_failures = 0;
    aws_mutex_unlock(&full_test_context->test_fixture->lock);

    ASSERT_SUCCESS(s_offline_publish(client, full_test_context, qos));

    s_aws_mqtt5_wait_for_publish_failure(full_test_context);

    return AWS_OP_SUCCESS;
}

/* There's no signalable event for internal queue changes so we have to spin-poll this in a dumb manner */
static void s_aws_mqtt5_wait_for_offline_queue_size(
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context,
    size_t expected_queue_size) {
    bool done = false;
    struct aws_mqtt5_client *client = full_test_context->test_fixture->client;
    while (!done) {
        aws_thread_current_sleep(aws_timestamp_convert(100, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_NANOS, NULL));

        struct aws_mqtt5_client_operation_statistics stats;
        AWS_ZERO_STRUCT(stats);

        aws_mqtt5_client_get_stats(client, &stats);

        done = stats.incomplete_operation_count == expected_queue_size;
    }
}

static void s_on_offline_subscribe_completion(
    const struct aws_mqtt5_packet_suback_view *suback_view,
    int error_code,
    void *user_data) {
    (void)suback_view;

    struct aws_mqtt5_sub_pub_unsub_context *full_test_context = user_data;

    aws_mutex_lock(&full_test_context->test_fixture->lock);
    if (error_code == AWS_ERROR_MQTT5_OPERATION_FAILED_DUE_TO_OFFLINE_QUEUE_POLICY) {
        ++full_test_context->subscribe_failures;
    }
    aws_mutex_unlock(&full_test_context->test_fixture->lock);
    aws_condition_variable_notify_all(&full_test_context->test_fixture->signal);
}

static bool s_has_failed_subscribes(void *user_data) {
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context = user_data;

    return full_test_context->subscribe_failures > 0;
}

static void s_aws_mqtt5_wait_for_subscribe_failure(struct aws_mqtt5_sub_pub_unsub_context *full_test_context) {
    aws_mutex_lock(&full_test_context->test_fixture->lock);
    aws_condition_variable_wait_pred(
        &full_test_context->test_fixture->signal,
        &full_test_context->test_fixture->lock,
        s_has_failed_subscribes,
        full_test_context);
    aws_mutex_unlock(&full_test_context->test_fixture->lock);
}

static int s_offline_subscribe(
    struct aws_mqtt5_client *client,
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context) {
    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .subscriptions = s_subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(s_subscriptions),
    };

    struct aws_mqtt5_subscribe_completion_options completion_options = {
        .completion_callback = s_on_offline_subscribe_completion,
        .completion_user_data = full_test_context,
    };

    return aws_mqtt5_client_subscribe(client, &subscribe_view, &completion_options);
}

static int s_verify_offline_subscribe_failure(
    struct aws_mqtt5_client *client,
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context) {
    aws_mutex_lock(&full_test_context->test_fixture->lock);
    full_test_context->subscribe_failures = 0;
    aws_mutex_unlock(&full_test_context->test_fixture->lock);

    ASSERT_SUCCESS(s_offline_subscribe(client, full_test_context));

    s_aws_mqtt5_wait_for_subscribe_failure(full_test_context);

    return AWS_OP_SUCCESS;
}

static void s_on_offline_unsubscribe_completion(
    const struct aws_mqtt5_packet_unsuback_view *unsuback_view,
    int error_code,
    void *user_data) {
    (void)unsuback_view;

    struct aws_mqtt5_sub_pub_unsub_context *full_test_context = user_data;

    aws_mutex_lock(&full_test_context->test_fixture->lock);
    if (error_code == AWS_ERROR_MQTT5_OPERATION_FAILED_DUE_TO_OFFLINE_QUEUE_POLICY) {
        ++full_test_context->unsubscribe_failures;
    }
    aws_mutex_unlock(&full_test_context->test_fixture->lock);
    aws_condition_variable_notify_all(&full_test_context->test_fixture->signal);
}

static bool s_has_failed_unsubscribes(void *user_data) {
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context = user_data;

    return full_test_context->unsubscribe_failures > 0;
}

static void s_aws_mqtt5_wait_for_unsubscribe_failure(struct aws_mqtt5_sub_pub_unsub_context *full_test_context) {
    aws_mutex_lock(&full_test_context->test_fixture->lock);
    aws_condition_variable_wait_pred(
        &full_test_context->test_fixture->signal,
        &full_test_context->test_fixture->lock,
        s_has_failed_unsubscribes,
        full_test_context);
    aws_mutex_unlock(&full_test_context->test_fixture->lock);
}

static int s_offline_unsubscribe(
    struct aws_mqtt5_client *client,
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context) {
    struct aws_mqtt5_packet_unsubscribe_view unsubscribe_view = {
        .topic_filters = s_sub_pub_unsub_topic_filters,
        .topic_filter_count = AWS_ARRAY_SIZE(s_sub_pub_unsub_topic_filters),
    };

    struct aws_mqtt5_unsubscribe_completion_options completion_options = {
        .completion_callback = s_on_offline_unsubscribe_completion,
        .completion_user_data = full_test_context,
    };

    return aws_mqtt5_client_unsubscribe(client, &unsubscribe_view, &completion_options);
}

static int s_verify_offline_unsubscribe_failure(
    struct aws_mqtt5_client *client,
    struct aws_mqtt5_sub_pub_unsub_context *full_test_context) {
    aws_mutex_lock(&full_test_context->test_fixture->lock);
    full_test_context->unsubscribe_failures = 0;
    aws_mutex_unlock(&full_test_context->test_fixture->lock);

    ASSERT_SUCCESS(s_offline_unsubscribe(client, full_test_context));

    s_aws_mqtt5_wait_for_unsubscribe_failure(full_test_context);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_offline_operation_submission_fail_all_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_ALL_ON_DISCONNECT;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    /* everything should fail on submission */
    ASSERT_SUCCESS(s_verify_offline_publish_failure(client, &full_test_context, AWS_MQTT5_QOS_AT_MOST_ONCE));
    ASSERT_SUCCESS(s_verify_offline_publish_failure(client, &full_test_context, AWS_MQTT5_QOS_AT_LEAST_ONCE));
    ASSERT_SUCCESS(s_verify_offline_subscribe_failure(client, &full_test_context));
    ASSERT_SUCCESS(s_verify_offline_unsubscribe_failure(client, &full_test_context));

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_offline_operation_submission_fail_all,
    s_mqtt5_client_offline_operation_submission_fail_all_fn)

static int s_mqtt5_client_offline_operation_submission_fail_qos0_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_QOS0_PUBLISH_ON_DISCONNECT;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    /* qos 0 publish should fail on submission */
    ASSERT_SUCCESS(s_verify_offline_publish_failure(client, &full_test_context, AWS_MQTT5_QOS_AT_MOST_ONCE));

    /* qos 1 publish, subscribe, and unsubscribe should queue on submission */
    ASSERT_SUCCESS(s_offline_publish(client, &full_test_context, AWS_MQTT5_QOS_AT_LEAST_ONCE));
    s_aws_mqtt5_wait_for_offline_queue_size(&full_test_context, 1);
    ASSERT_SUCCESS(s_offline_subscribe(client, &full_test_context));
    s_aws_mqtt5_wait_for_offline_queue_size(&full_test_context, 2);
    ASSERT_SUCCESS(s_offline_unsubscribe(client, &full_test_context));
    s_aws_mqtt5_wait_for_offline_queue_size(&full_test_context, 3);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_offline_operation_submission_fail_qos0,
    s_mqtt5_client_offline_operation_submission_fail_qos0_fn)

static int s_mqtt5_client_offline_operation_submission_fail_non_qos1_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_NON_QOS1_PUBLISH_ON_DISCONNECT;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    /* qos0 publish, subscribe, and unsubscribe should fail on submission */
    ASSERT_SUCCESS(s_verify_offline_publish_failure(client, &full_test_context, AWS_MQTT5_QOS_AT_MOST_ONCE));
    ASSERT_SUCCESS(s_verify_offline_subscribe_failure(client, &full_test_context));
    ASSERT_SUCCESS(s_verify_offline_unsubscribe_failure(client, &full_test_context));

    /* qos1 publish should queue on submission */
    ASSERT_SUCCESS(s_offline_publish(client, &full_test_context, AWS_MQTT5_QOS_AT_LEAST_ONCE));
    s_aws_mqtt5_wait_for_offline_queue_size(&full_test_context, 1);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_offline_operation_submission_fail_non_qos1,
    s_mqtt5_client_offline_operation_submission_fail_non_qos1_fn)

static int s_mqtt5_client_offline_operation_submission_then_connect_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;

    test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_NON_QOS1_PUBLISH_ON_DISCONNECT;

    struct aws_mqtt5_client_mock_test_fixture test_context;
    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;

    /* qos1 publish should queue on submission */
    ASSERT_SUCCESS(s_offline_publish(client, &full_test_context, AWS_MQTT5_QOS_AT_LEAST_ONCE));
    s_aws_mqtt5_wait_for_offline_queue_size(&full_test_context, 1);

    /* start the client, it should connect and immediately send the publish */
    aws_mqtt5_client_start(client);

    s_sub_pub_unsub_wait_for_publish_complete(&full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_offline_operation_submission_then_connect,
    s_mqtt5_client_offline_operation_submission_then_connect_fn)

#define ALIASED_PUBLISH_SEQUENCE_COUNT 4

struct aws_mqtt5_aliased_publish_sequence_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;
    struct aws_allocator *allocator;

    struct aws_array_list publishes_received;
};

static int s_aws_mqtt5_aliased_publish_sequence_context_init(
    struct aws_mqtt5_aliased_publish_sequence_context *context,
    struct aws_allocator *allocator) {
    AWS_ZERO_STRUCT(*context);

    context->allocator = allocator;
    return aws_array_list_init_dynamic(
        &context->publishes_received, allocator, 10, sizeof(struct aws_mqtt5_packet_publish_storage *));
}

static void s_aws_mqtt5_aliased_publish_sequence_context_clean_up(
    struct aws_mqtt5_aliased_publish_sequence_context *context) {
    for (size_t i = 0; i < aws_array_list_length(&context->publishes_received); ++i) {
        struct aws_mqtt5_packet_publish_storage *storage = NULL;
        aws_array_list_get_at(&context->publishes_received, &storage, i);

        aws_mqtt5_packet_publish_storage_clean_up(storage);
        aws_mem_release(context->allocator, storage);
    }
    aws_array_list_clean_up(&context->publishes_received);
}

void s_aliased_publish_received_fn(const struct aws_mqtt5_packet_publish_view *publish, void *complete_ctx) {

    AWS_FATAL_ASSERT(publish != NULL);

    struct aws_mqtt5_aliased_publish_sequence_context *full_test_context = complete_ctx;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = full_test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);

    struct aws_mqtt5_packet_publish_storage *storage =
        aws_mem_calloc(full_test_context->allocator, 1, sizeof(struct aws_mqtt5_packet_publish_storage));
    aws_mqtt5_packet_publish_storage_init(storage, full_test_context->allocator, publish);

    aws_array_list_push_back(&full_test_context->publishes_received, &storage);

    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

static enum aws_mqtt5_suback_reason_code s_alias_reason_codes[] = {
    AWS_MQTT5_SARC_GRANTED_QOS_1,
    AWS_MQTT5_SARC_GRANTED_QOS_1,
};
static uint8_t s_alias_topic1[] = "alias/first/topic";
static uint8_t s_alias_topic2[] = "alias/second/topic";

static int s_aws_mqtt5_server_send_aliased_publish_sequence(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)user_data;

    struct aws_mqtt5_packet_subscribe_view *subscribe_view = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_view->packet_id,
        .reason_code_count = 1,
        .reason_codes = s_alias_reason_codes,
    };

    // just to be thorough, send a suback
    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view)) {
        return AWS_OP_ERR;
    }

    uint16_t alias_id = 1;
    struct aws_mqtt5_packet_publish_view publish_view = {
        .packet_id = 1,
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic =
            {
                .ptr = s_alias_topic1,
                .len = AWS_ARRAY_SIZE(s_alias_topic1) - 1,
            },
        .topic_alias = &alias_id,
    };

    // establish an alias with id 1
    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &publish_view)) {
        return AWS_OP_ERR;
    }

    // alias alone
    AWS_ZERO_STRUCT(publish_view.topic);
    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &publish_view)) {
        return AWS_OP_ERR;
    }

    // establish a new alias with id 1
    publish_view.topic.ptr = s_alias_topic2;
    publish_view.topic.len = AWS_ARRAY_SIZE(s_alias_topic2) - 1;

    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &publish_view)) {
        return AWS_OP_ERR;
    }

    // alias alone
    AWS_ZERO_STRUCT(publish_view.topic);
    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &publish_view)) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

static bool s_client_received_aliased_publish_sequence(void *arg) {
    struct aws_mqtt5_aliased_publish_sequence_context *full_test_context = arg;

    return aws_array_list_length(&full_test_context->publishes_received) == ALIASED_PUBLISH_SEQUENCE_COUNT;
}

static void s_wait_for_aliased_publish_sequence(struct aws_mqtt5_aliased_publish_sequence_context *full_test_context) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = full_test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(
        &test_fixture->signal, &test_fixture->lock, s_client_received_aliased_publish_sequence, full_test_context);
    aws_mutex_unlock(&test_fixture->lock);
}

static int s_verify_aliased_publish_sequence(struct aws_mqtt5_aliased_publish_sequence_context *full_test_context) {

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = full_test_context->test_fixture;
    aws_mutex_lock(&test_fixture->lock);

    for (size_t i = 0; i < aws_array_list_length(&full_test_context->publishes_received); ++i) {
        struct aws_mqtt5_packet_publish_storage *publish_storage = NULL;
        aws_array_list_get_at(&full_test_context->publishes_received, &publish_storage, i);

        struct aws_byte_cursor topic_cursor = publish_storage->storage_view.topic;

        if (i < 2) {
            // the first two publishes should be the first topic
            ASSERT_BIN_ARRAYS_EQUALS(
                s_alias_topic1, AWS_ARRAY_SIZE(s_alias_topic1) - 1, topic_cursor.ptr, topic_cursor.len);
        } else {
            // the last two publishes should be the second topic
            ASSERT_BIN_ARRAYS_EQUALS(
                s_alias_topic2, AWS_ARRAY_SIZE(s_alias_topic2) - 1, topic_cursor.ptr, topic_cursor.len);
        }
    }

    aws_mutex_unlock(&test_fixture->lock);

    return AWS_OP_SUCCESS;
}

static struct aws_mqtt5_subscription_view s_alias_subscriptions[] = {
    {
        .topic_filter =
            {
                .ptr = (uint8_t *)s_alias_topic1,
                .len = AWS_ARRAY_SIZE(s_alias_topic1) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
    },
    {
        .topic_filter =
            {
                .ptr = (uint8_t *)s_alias_topic2,
                .len = AWS_ARRAY_SIZE(s_alias_topic2) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
    },
};

static int s_mqtt5_client_inbound_alias_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_aws_mqtt5_server_send_aliased_publish_sequence;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_aliased_publish_sequence_context full_test_context;
    ASSERT_SUCCESS(s_aws_mqtt5_aliased_publish_sequence_context_init(&full_test_context, allocator));
    full_test_context.test_fixture = &test_context;

    struct aws_mqtt5_client_topic_alias_options aliasing_config = {
        .inbound_alias_cache_size = 10,
        .inbound_topic_alias_behavior = AWS_MQTT5_CITABT_ENABLED,
    };

    test_options.client_options.topic_aliasing_options = &aliasing_config;
    test_options.client_options.publish_received_handler = s_aliased_publish_received_fn;
    test_options.client_options.publish_received_handler_user_data = &full_test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_subscribe_view subscribe = {
        .subscriptions = s_alias_subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(s_alias_subscriptions),
    };

    ASSERT_SUCCESS(aws_mqtt5_client_subscribe(client, &subscribe, NULL));

    s_wait_for_aliased_publish_sequence(&full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    ASSERT_SUCCESS(s_verify_aliased_publish_sequence(&full_test_context));

    s_aws_mqtt5_aliased_publish_sequence_context_clean_up(&full_test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_inbound_alias_success, s_mqtt5_client_inbound_alias_success_fn)

enum aws_mqtt5_test_inbound_alias_failure_type {
    AWS_MTIAFT_DISABLED,
    AWS_MTIAFT_ZERO_ID,
    AWS_MTIAFT_TOO_LARGE_ID,
    AWS_MTIAFT_UNBOUND_ID
};

struct aws_mqtt5_test_inbound_alias_failure_context {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture;

    enum aws_mqtt5_test_inbound_alias_failure_type failure_type;
};

static int s_aws_mqtt5_server_send_aliased_publish_failure(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {

    struct aws_mqtt5_test_inbound_alias_failure_context *test_context = user_data;

    struct aws_mqtt5_packet_subscribe_view *subscribe_view = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_view->packet_id,
        .reason_code_count = 1,
        .reason_codes = s_alias_reason_codes,
    };

    // just to be thorough, send a suback
    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view)) {
        return AWS_OP_ERR;
    }

    uint16_t alias_id = 0;
    struct aws_byte_cursor topic_cursor = {
        .ptr = s_alias_topic1,
        .len = AWS_ARRAY_SIZE(s_alias_topic1) - 1,
    };

    switch (test_context->failure_type) {
        case AWS_MTIAFT_TOO_LARGE_ID:
            alias_id = 100;
            break;

        case AWS_MTIAFT_UNBOUND_ID:
            AWS_ZERO_STRUCT(topic_cursor);
            alias_id = 1;
            break;

        default:
            break;
    }

    struct aws_mqtt5_packet_publish_view publish_view = {
        .packet_id = 1, .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE, .topic = topic_cursor, .topic_alias = &alias_id};

    // establish an alias with id 1
    if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &publish_view)) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

static bool s_has_decoding_error_disconnect_event(void *arg) {
    struct aws_mqtt5_test_inbound_alias_failure_context *test_context = arg;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    size_t record_count = aws_array_list_length(&test_fixture->lifecycle_events);
    for (size_t i = 0; i < record_count; ++i) {
        struct aws_mqtt5_lifecycle_event_record *record = NULL;
        aws_array_list_get_at(&test_fixture->lifecycle_events, &record, i);
        if (record->event.event_type == AWS_MQTT5_CLET_DISCONNECTION) {
            if (record->event.error_code == AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR) {
                return true;
            }
        }
    }

    return false;
}

static void s_wait_for_decoding_error_disconnect(struct aws_mqtt5_test_inbound_alias_failure_context *test_context) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(
        &test_fixture->signal, &test_fixture->lock, s_has_decoding_error_disconnect_event, test_context);
    aws_mutex_unlock(&test_fixture->lock);
}

static int s_do_inbound_alias_failure_test(
    struct aws_allocator *allocator,
    enum aws_mqtt5_test_inbound_alias_failure_type test_failure_type) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_aws_mqtt5_server_send_aliased_publish_failure;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_test_inbound_alias_failure_context full_test_context = {
        .test_fixture = &test_context,
        .failure_type = test_failure_type,
    };

    struct aws_mqtt5_client_topic_alias_options aliasing_config = {
        .inbound_alias_cache_size = 10,
        .inbound_topic_alias_behavior =
            (test_failure_type == AWS_MTIAFT_DISABLED) ? AWS_MQTT5_CITABT_DISABLED : AWS_MQTT5_CITABT_ENABLED,
    };

    test_options.client_options.topic_aliasing_options = &aliasing_config;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &full_test_context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    struct aws_mqtt5_packet_subscribe_view subscribe = {
        .subscriptions = s_alias_subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(s_alias_subscriptions),
    };

    ASSERT_SUCCESS(aws_mqtt5_client_subscribe(client, &subscribe, NULL));

    s_wait_for_decoding_error_disconnect(&full_test_context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_inbound_alias_failure_disabled_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_inbound_alias_failure_test(allocator, AWS_MTIAFT_DISABLED));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_inbound_alias_failure_disabled, s_mqtt5_client_inbound_alias_failure_disabled_fn)

static int s_mqtt5_client_inbound_alias_failure_zero_id_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_inbound_alias_failure_test(allocator, AWS_MTIAFT_ZERO_ID));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_inbound_alias_failure_zero_id, s_mqtt5_client_inbound_alias_failure_zero_id_fn)

static int s_mqtt5_client_inbound_alias_failure_too_large_id_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_inbound_alias_failure_test(allocator, AWS_MTIAFT_TOO_LARGE_ID));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_inbound_alias_failure_too_large_id, s_mqtt5_client_inbound_alias_failure_too_large_id_fn)

static int s_mqtt5_client_inbound_alias_failure_unbound_id_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_inbound_alias_failure_test(allocator, AWS_MTIAFT_UNBOUND_ID));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_inbound_alias_failure_unbound_id, s_mqtt5_client_inbound_alias_failure_unbound_id_fn)

void s_outbound_alias_failure_publish_complete_fn(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *complete_ctx) {

    (void)packet_type;
    (void)packet;
    AWS_FATAL_ASSERT(error_code != AWS_ERROR_SUCCESS);

    struct aws_mqtt5_sub_pub_unsub_context *test_context = complete_ctx;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = test_context->test_fixture;

    aws_mutex_lock(&test_fixture->lock);
    test_context->publish_failures++;
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

#define SEQUENCE_TEST_CACHE_SIZE 2

static int s_aws_mqtt5_mock_server_handle_connect_allow_aliasing(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    uint16_t topic_alias_maximum = SEQUENCE_TEST_CACHE_SIZE;
    connack_view.topic_alias_maximum = &topic_alias_maximum;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

static int s_do_mqtt5_client_outbound_alias_failure_test(
    struct aws_allocator *allocator,
    enum aws_mqtt5_client_outbound_topic_alias_behavior_type behavior_type) {

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_allow_aliasing;

    test_options.topic_aliasing_options.outbound_topic_alias_behavior = behavior_type;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_sub_pub_unsub_context full_test_context = {
        .test_fixture = &test_context,
    };
    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    uint16_t topic_alias = 1;
    struct aws_mqtt5_packet_publish_view packet_publish_view = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic =
            {
                .ptr = s_topic,
                .len = AWS_ARRAY_SIZE(s_topic) - 1,
            },
        .topic_alias = &topic_alias,
    };

    if (behavior_type == AWS_MQTT5_COTABT_MANUAL) {
        AWS_ZERO_STRUCT(packet_publish_view.topic);
    }

    struct aws_mqtt5_publish_completion_options completion_options = {
        .completion_callback = s_outbound_alias_failure_publish_complete_fn,
        .completion_user_data = &full_test_context,
    };

    /* should result in an immediate validation failure or a subsequent dynamic validation failure */
    if (aws_mqtt5_client_publish(client, &packet_publish_view, &completion_options) == AWS_OP_SUCCESS) {
        s_aws_mqtt5_wait_for_publish_failure(&full_test_context);
    }

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_client_outbound_alias_manual_failure_empty_topic_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5_client_outbound_alias_failure_test(allocator, AWS_MQTT5_COTABT_MANUAL));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_outbound_alias_manual_failure_empty_topic,
    s_mqtt5_client_outbound_alias_manual_failure_empty_topic_fn)

struct outbound_alias_publish {
    struct aws_byte_cursor topic;
    uint16_t topic_alias;

    size_t expected_alias_id;
    bool expected_reuse;
};

#define DEFINE_OUTBOUND_ALIAS_PUBLISH(topic_suffix, desired_alias, expected_alias_index, reused)                       \
    {                                                                                                                  \
        .topic = aws_byte_cursor_from_string(s_topic_##topic_suffix),                                                  \
        .topic_alias = desired_alias,                                                                                  \
        .expected_alias_id = expected_alias_index,                                                                     \
        .expected_reuse = reused,                                                                                      \
    }

static void s_outbound_alias_publish_completion_fn(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *complete_ctx) {

    AWS_FATAL_ASSERT(packet_type == AWS_MQTT5_PT_PUBACK);
    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);

    const struct aws_mqtt5_packet_puback_view *puback = packet;
    struct aws_mqtt5_client_mock_test_fixture *test_context = complete_ctx;

    aws_mutex_lock(&test_context->lock);

    ++test_context->total_pubacks_received;
    if (error_code == AWS_ERROR_SUCCESS && puback->reason_code < 128) {
        ++test_context->successful_pubacks_received;
    }

    aws_mutex_unlock(&test_context->lock);
    aws_condition_variable_notify_all(&test_context->signal);
}

static int s_perform_outbound_alias_publish(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    struct outbound_alias_publish *publish) {

    struct aws_mqtt5_client *client = test_fixture->client;

    uint16_t alias_id = publish->topic_alias;
    struct aws_mqtt5_packet_publish_view publish_view = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic = publish->topic,
    };

    if (alias_id != 0) {
        publish_view.topic_alias = &alias_id;
    }

    struct aws_mqtt5_publish_completion_options completion_options = {
        .completion_callback = s_outbound_alias_publish_completion_fn,
        .completion_user_data = test_fixture,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &publish_view, &completion_options));

    return AWS_OP_SUCCESS;
}

static int s_perform_outbound_alias_sequence(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    struct outbound_alias_publish *publishes,
    size_t publish_count) {

    for (size_t i = 0; i < publish_count; ++i) {
        struct outbound_alias_publish *publish = &publishes[i];
        ASSERT_SUCCESS(s_perform_outbound_alias_publish(test_fixture, publish));
    }

    return AWS_OP_SUCCESS;
}

static int s_perform_outbound_alias_sequence_test(
    struct aws_allocator *allocator,
    enum aws_mqtt5_client_outbound_topic_alias_behavior_type behavior_type,
    struct outbound_alias_publish *publishes,
    size_t publish_count) {

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_allow_aliasing;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback;

    test_options.topic_aliasing_options.outbound_topic_alias_behavior = behavior_type;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));
    test_context.maximum_inbound_topic_aliases = SEQUENCE_TEST_CACHE_SIZE;

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    ASSERT_SUCCESS(s_perform_outbound_alias_sequence(&test_context, publishes, publish_count));

    struct aws_mqtt5_client_test_wait_for_n_context wait_context = {
        .test_fixture = &test_context,
        .required_event_count = publish_count,
    };
    s_wait_for_n_successful_publishes(&wait_context);

    aws_mutex_lock(&test_context.lock);
    size_t packet_count = aws_array_list_length(&test_context.server_received_packets);
    ASSERT_INT_EQUALS(1 + publish_count, packet_count); // N publishes, 1 connect

    /* start at 1 and skip the connect */
    for (size_t i = 1; i < packet_count; ++i) {
        struct aws_mqtt5_mock_server_packet_record *packet = NULL;
        aws_array_list_get_at_ptr(&test_context.server_received_packets, (void **)&packet, i);

        ASSERT_INT_EQUALS(AWS_MQTT5_PT_PUBLISH, packet->packet_type);
        struct aws_mqtt5_packet_publish_storage *publish_storage = packet->packet_storage;
        struct aws_mqtt5_packet_publish_view *publish_view = &publish_storage->storage_view;

        struct outbound_alias_publish *publish = &publishes[i - 1];
        ASSERT_NOT_NULL(publish_view->topic_alias);
        ASSERT_INT_EQUALS(publish->expected_alias_id, *publish_view->topic_alias);

        /*
         * Unfortunately, the decoder fails unless it has an inbound resolver and the inbound resolver will always
         * resolve the topics first.  So we can't actually check that an empty topic was sent.  It would be nice to
         * harden this up in the future.
         */
        ASSERT_BIN_ARRAYS_EQUALS(
            publish->topic.ptr, publish->topic.len, publish_view->topic.ptr, publish_view->topic.len);
    }

    aws_mutex_unlock(&test_context.lock);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_STATIC_STRING_FROM_LITERAL(s_topic_a, "topic/a");
AWS_STATIC_STRING_FROM_LITERAL(s_topic_b, "b/topic");
AWS_STATIC_STRING_FROM_LITERAL(s_topic_c, "topic/c");

static int s_mqtt5_client_outbound_alias_manual_success_a_b_ar_br_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct outbound_alias_publish test_publishes[] = {
        DEFINE_OUTBOUND_ALIAS_PUBLISH(a, 1, 1, false),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(b, 2, 2, false),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(a, 1, 1, true),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(b, 2, 2, true),
    };

    ASSERT_SUCCESS(s_perform_outbound_alias_sequence_test(
        allocator, AWS_MQTT5_COTABT_MANUAL, test_publishes, AWS_ARRAY_SIZE(test_publishes)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_outbound_alias_manual_success_a_b_ar_br,
    s_mqtt5_client_outbound_alias_manual_success_a_b_ar_br_fn)

static int s_mqtt5_client_outbound_alias_lru_success_a_b_c_br_cr_a_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct outbound_alias_publish test_publishes[] = {
        DEFINE_OUTBOUND_ALIAS_PUBLISH(a, 0, 1, false),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(b, 0, 2, false),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(c, 0, 1, false),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(b, 0, 2, true),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(c, 0, 1, true),
        DEFINE_OUTBOUND_ALIAS_PUBLISH(a, 0, 2, false),
    };

    ASSERT_SUCCESS(s_perform_outbound_alias_sequence_test(
        allocator, AWS_MQTT5_COTABT_LRU, test_publishes, AWS_ARRAY_SIZE(test_publishes)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_client_outbound_alias_lru_success_a_b_c_br_cr_a,
    s_mqtt5_client_outbound_alias_lru_success_a_b_c_br_cr_a_fn)

struct mqtt5_operation_timeout_completion_callback {
    enum aws_mqtt5_packet_type type;
    uint64_t timepoint_ns;
    int error_code;
};

struct mqtt5_dynamic_operation_timeout_test_context {
    struct aws_allocator *allocator;
    struct aws_mqtt5_client_mock_test_fixture *fixture;
    struct aws_array_list completion_callbacks;

    const struct mqtt5_operation_timeout_completion_callback *expected_callbacks;
    size_t expected_callback_count;
};

static void s_mqtt5_dynamic_operation_timeout_test_context_init(
    struct mqtt5_dynamic_operation_timeout_test_context *context,
    struct aws_allocator *allocator,
    struct aws_mqtt5_client_mock_test_fixture *fixture) {
    AWS_ZERO_STRUCT(*context);

    context->allocator = allocator;
    context->fixture = fixture;
    aws_array_list_init_dynamic(
        &context->completion_callbacks, allocator, 5, sizeof(struct mqtt5_operation_timeout_completion_callback));
}

static void s_mqtt5_dynamic_operation_timeout_test_context_cleanup(
    struct mqtt5_dynamic_operation_timeout_test_context *context) {
    aws_array_list_clean_up(&context->completion_callbacks);
}

static bool s_mqtt5_dynamic_operation_timeout_test_context_callback_sequence_equals_expected(
    struct mqtt5_dynamic_operation_timeout_test_context *context) {

    if (context->expected_callback_count != aws_array_list_length(&context->completion_callbacks)) {
        return false;
    }

    for (size_t i = 0; i < context->expected_callback_count; ++i) {
        const struct mqtt5_operation_timeout_completion_callback *expected_callback = &context->expected_callbacks[i];
        struct mqtt5_operation_timeout_completion_callback *callback = NULL;
        aws_array_list_get_at_ptr(&context->completion_callbacks, (void **)&callback, i);

        if (callback->type != expected_callback->type) {
            return false;
        }

        if (callback->error_code != expected_callback->error_code) {
            return false;
        }
    }

    return true;
}

static void s_add_completion_callback(
    struct mqtt5_dynamic_operation_timeout_test_context *context,
    enum aws_mqtt5_packet_type type,
    int error_code) {
    aws_mutex_lock(&context->fixture->lock);

    struct mqtt5_operation_timeout_completion_callback callback_entry = {
        .type = type, .error_code = error_code, .timepoint_ns = 0};

    aws_high_res_clock_get_ticks(&callback_entry.timepoint_ns);

    aws_array_list_push_back(&context->completion_callbacks, &callback_entry);

    aws_mutex_unlock(&context->fixture->lock);

    aws_condition_variable_notify_all(&context->fixture->signal);
}

static void s_timeout_test_publish_completion_fn(
    enum aws_mqtt5_packet_type packet_type,
    const void *packet,
    int error_code,
    void *complete_ctx) {
    (void)packet_type;
    (void)packet;

    s_add_completion_callback(complete_ctx, AWS_MQTT5_PT_PUBLISH, error_code);
}

static void s_timeout_test_subscribe_completion_fn(
    const struct aws_mqtt5_packet_suback_view *suback,
    int error_code,
    void *complete_ctx) {
    (void)suback;

    s_add_completion_callback(complete_ctx, AWS_MQTT5_PT_SUBSCRIBE, error_code);
}

static void s_timeout_test_unsubscribe_completion_fn(
    const struct aws_mqtt5_packet_unsuback_view *unsuback,
    int error_code,
    void *complete_ctx) {
    (void)unsuback;

    s_add_completion_callback(complete_ctx, AWS_MQTT5_PT_UNSUBSCRIBE, error_code);
}

static bool s_all_timeout_operations_complete(void *arg) {
    struct mqtt5_dynamic_operation_timeout_test_context *context = arg;

    return aws_array_list_length(&context->completion_callbacks) == context->expected_callback_count;
}

static void s_wait_for_all_operation_timeouts(struct mqtt5_dynamic_operation_timeout_test_context *context) {
    aws_mutex_lock(&context->fixture->lock);
    aws_condition_variable_wait_pred(
        &context->fixture->signal, &context->fixture->lock, s_all_timeout_operations_complete, context);
    aws_mutex_unlock(&context->fixture->lock);
}

/*
 * Tests a mixture of qos 0 publish, qos 1 publish, subscribe, and unsubscribe with override ack timeouts.
 *
 * qos 1 publish with 3 second timeout
 * subscribe with 2 second timeout
 * qos 0 publish
 * unsubscribe with 4 second timeout
 * qos 1 publish with 1 second timeout
 *
 * We expect to see callbacks in sequence:
 *
 *  qos 0 publish success
 *  qos 1 publish failure by timeout after 1 second
 *  subscribe failure by timeout after 2 seconds
 *  qos 1 publish failure by timeout after 3 seconds
 *  unsubscribe failure by timeout after 4 seconds
 */
static int s_mqtt5_client_dynamic_operation_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_mock_server_handle_timeout_publish;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct mqtt5_dynamic_operation_timeout_test_context context;
    s_mqtt5_dynamic_operation_timeout_test_context_init(&context, allocator, &test_context);

    struct mqtt5_operation_timeout_completion_callback expected_callbacks[] = {
        {
            .type = AWS_MQTT5_PT_PUBLISH,
            .error_code = AWS_ERROR_SUCCESS,
        },
        {
            .type = AWS_MQTT5_PT_PUBLISH,
            .error_code = AWS_ERROR_MQTT_TIMEOUT,
        },
        {
            .type = AWS_MQTT5_PT_SUBSCRIBE,
            .error_code = AWS_ERROR_MQTT_TIMEOUT,
        },
        {
            .type = AWS_MQTT5_PT_PUBLISH,
            .error_code = AWS_ERROR_MQTT_TIMEOUT,
        },
        {
            .type = AWS_MQTT5_PT_UNSUBSCRIBE,
            .error_code = AWS_ERROR_MQTT_TIMEOUT,
        },
    };

    context.expected_callbacks = expected_callbacks;
    context.expected_callback_count = AWS_ARRAY_SIZE(expected_callbacks);

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    uint64_t operation_start = 0;
    aws_high_res_clock_get_ticks(&operation_start);

    struct aws_byte_cursor topic = {
        .ptr = s_topic,
        .len = AWS_ARRAY_SIZE(s_topic) - 1,
    };

    // qos 1 publish - 3 second timeout
    struct aws_mqtt5_packet_publish_view qos1_publish = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic = topic,
    };

    struct aws_mqtt5_publish_completion_options qos1_publish_options = {
        .completion_callback = s_timeout_test_publish_completion_fn,
        .completion_user_data = &context,
        .ack_timeout_seconds_override = 3,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish, &qos1_publish_options));

    // subscribe - 2 seconds timeout
    struct aws_mqtt5_subscription_view subscriptions[] = {{
        .topic_filter = topic,
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
    }};

    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .subscriptions = subscriptions,
        .subscription_count = AWS_ARRAY_SIZE(subscriptions),
    };

    struct aws_mqtt5_subscribe_completion_options subscribe_options = {
        .completion_callback = s_timeout_test_subscribe_completion_fn,
        .completion_user_data = &context,
        .ack_timeout_seconds_override = 2,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_subscribe(client, &subscribe_view, &subscribe_options));

    // qos 0 publish
    struct aws_mqtt5_packet_publish_view qos0_publish = {
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .topic = topic,
    };

    struct aws_mqtt5_publish_completion_options qos0_publish_options = {
        .completion_callback = s_timeout_test_publish_completion_fn,
        .completion_user_data = &context,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos0_publish, &qos0_publish_options));

    // unsubscribe - 4 second timeout
    struct aws_byte_cursor topic_filters[] = {
        topic,
    };

    struct aws_mqtt5_packet_unsubscribe_view unsubscribe = {
        .topic_filters = topic_filters,
        .topic_filter_count = AWS_ARRAY_SIZE(topic_filters),
    };

    struct aws_mqtt5_unsubscribe_completion_options unsubscribe_options = {
        .completion_callback = s_timeout_test_unsubscribe_completion_fn,
        .completion_user_data = &context,
        .ack_timeout_seconds_override = 4,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_unsubscribe(client, &unsubscribe, &unsubscribe_options));

    // qos 1 publish - 1 second timeout
    qos1_publish_options.ack_timeout_seconds_override = 1;

    ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish, &qos1_publish_options));

    s_wait_for_all_operation_timeouts(&context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mutex_lock(&test_context.lock);
    s_mqtt5_dynamic_operation_timeout_test_context_callback_sequence_equals_expected(&context);
    aws_mutex_unlock(&test_context.lock);

    /*
     * Finally, do a minimum time elapsed check:
     *   each operation after the first (the qos 0 publish which did not time out) should have a completion
     *   timepoint N seconds or later after the start of the test (where N is the operation's index in the sequence)
     */
    for (size_t i = 1; i < context.expected_callback_count; ++i) {
        struct mqtt5_operation_timeout_completion_callback *callback = NULL;
        aws_array_list_get_at_ptr(&context.completion_callbacks, (void **)&callback, i);

        uint64_t delta_ns = callback->timepoint_ns - operation_start;
        ASSERT_TRUE(delta_ns >= aws_timestamp_convert(i, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    }

    s_mqtt5_dynamic_operation_timeout_test_context_cleanup(&context);
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_dynamic_operation_timeout, s_mqtt5_client_dynamic_operation_timeout_fn)

#define DYNAMIC_TIMEOUT_DEFAULT_SECONDS 2

/*
 * Checks that using a override operation timeout of zero results in using the client's default timeout
 */
static int s_mqtt5_client_dynamic_operation_timeout_default_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);
    test_options.client_options.ack_timeout_seconds = DYNAMIC_TIMEOUT_DEFAULT_SECONDS;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        s_aws_mqtt5_mock_server_handle_timeout_publish;

    struct aws_mqtt5_client_mock_test_fixture test_context;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_context, allocator, &test_fixture_options));

    struct mqtt5_dynamic_operation_timeout_test_context context;
    s_mqtt5_dynamic_operation_timeout_test_context_init(&context, allocator, &test_context);

    struct mqtt5_operation_timeout_completion_callback expected_callbacks[] = {
        {
            .type = AWS_MQTT5_PT_PUBLISH,
            .error_code = AWS_ERROR_MQTT_TIMEOUT,
        },
    };

    context.expected_callbacks = expected_callbacks;
    context.expected_callback_count = AWS_ARRAY_SIZE(expected_callbacks);

    struct aws_mqtt5_client *client = test_context.client;
    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_context);

    uint64_t operation_start = 0;
    aws_high_res_clock_get_ticks(&operation_start);

    struct aws_byte_cursor topic = {
        .ptr = s_topic,
        .len = AWS_ARRAY_SIZE(s_topic) - 1,
    };

    struct aws_mqtt5_packet_publish_view qos1_publish = {
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .topic = topic,
    };

    struct aws_mqtt5_publish_completion_options qos1_publish_options = {
        .completion_callback = s_timeout_test_publish_completion_fn,
        .completion_user_data = &context,
        .ack_timeout_seconds_override = 0,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_publish(client, &qos1_publish, &qos1_publish_options));

    s_wait_for_all_operation_timeouts(&context);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

    aws_mutex_lock(&test_context.lock);
    s_mqtt5_dynamic_operation_timeout_test_context_callback_sequence_equals_expected(&context);
    aws_mutex_unlock(&test_context.lock);

    /*
     * Finally, do a minimum time elapsed check:
     */
    for (size_t i = 0; i < context.expected_callback_count; ++i) {
        struct mqtt5_operation_timeout_completion_callback *callback = NULL;
        aws_array_list_get_at_ptr(&context.completion_callbacks, (void **)&callback, i);

        uint64_t delta_ns = callback->timepoint_ns - operation_start;
        ASSERT_TRUE(
            delta_ns >=
            aws_timestamp_convert(DYNAMIC_TIMEOUT_DEFAULT_SECONDS, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    }

    s_mqtt5_dynamic_operation_timeout_test_context_cleanup(&context);
    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_client_dynamic_operation_timeout_default, s_mqtt5_client_dynamic_operation_timeout_default_fn)