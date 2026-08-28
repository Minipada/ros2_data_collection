/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/mqtt.h>

#include <aws/mqtt/private/mqtt311_listener.h>

#include "mqtt311_testing_utils.h"
#include "mqtt_mock_server_handler.h"

#include <aws/testing/aws_test_harness.h>

struct mqtt311_listener_connection_success_record {
    bool joined_session;
};

struct mqtt311_listener_connection_interrupted_record {
    int error_code;
};

struct mqtt311_listener_publish_record {
    struct aws_byte_buf topic;
    struct aws_byte_buf payload;
};

struct mqtt311_listener_test_context {
    struct aws_allocator *allocator;

    struct aws_mqtt311_listener *listener;

    struct mqtt_connection_state_test *mqtt311_test_context;
    int mqtt311_test_context_setup_result;

    struct aws_array_list connection_success_events;
    struct aws_array_list connection_interrupted_events;
    struct aws_array_list publish_events;
    size_t disconnect_event_count;
    bool terminated;

    struct aws_mutex lock;
    struct aws_condition_variable signal;
};

static void s_311_listener_test_on_publish_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata) {
    (void)connection;
    (void)dup;
    (void)qos;
    (void)retain;

    struct mqtt311_listener_test_context *context = userdata;

    struct mqtt311_listener_publish_record publish_record;
    AWS_ZERO_STRUCT(publish_record);

    aws_byte_buf_init_copy_from_cursor(&publish_record.topic, context->allocator, *topic);
    aws_byte_buf_init_copy_from_cursor(&publish_record.payload, context->allocator, *payload);

    aws_mutex_lock(&context->lock);
    aws_array_list_push_back(&context->publish_events, &publish_record);
    aws_mutex_unlock(&context->lock);
    aws_condition_variable_notify_all(&context->signal);
}

static void s_311_listener_test_on_connection_success(
    struct aws_mqtt_client_connection *connection,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {
    (void)connection;
    (void)return_code;

    struct mqtt311_listener_test_context *context = userdata;

    struct mqtt311_listener_connection_success_record connection_success_record = {
        .joined_session = session_present,
    };

    aws_mutex_lock(&context->lock);
    aws_array_list_push_back(&context->connection_success_events, &connection_success_record);
    aws_mutex_unlock(&context->lock);
    aws_condition_variable_notify_all(&context->signal);
}

static void s_311_listener_test_on_connection_interrupted(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    void *userdata) {
    (void)connection;

    struct mqtt311_listener_test_context *context = userdata;

    struct mqtt311_listener_connection_interrupted_record connection_interrupted_record = {
        .error_code = error_code,
    };

    aws_mutex_lock(&context->lock);
    aws_array_list_push_back(&context->connection_interrupted_events, &connection_interrupted_record);
    aws_mutex_unlock(&context->lock);
    aws_condition_variable_notify_all(&context->signal);
}

static void s_311_listener_test_on_disconnect(struct aws_mqtt_client_connection *connection, void *userdata) {
    (void)connection;

    struct mqtt311_listener_test_context *context = userdata;

    aws_mutex_lock(&context->lock);
    ++context->disconnect_event_count;
    aws_mutex_unlock(&context->lock);
    aws_condition_variable_notify_all(&context->signal);
}

static void s_311_listener_test_on_termination(void *complete_ctx) {
    struct mqtt311_listener_test_context *context = complete_ctx;

    aws_mutex_lock(&context->lock);
    context->terminated = true;
    aws_mutex_unlock(&context->lock);
    aws_condition_variable_notify_all(&context->signal);
}

static int mqtt311_listener_test_context_init(
    struct mqtt311_listener_test_context *context,
    struct aws_allocator *allocator,
    struct mqtt_connection_state_test *mqtt311_test_context) {
    AWS_ZERO_STRUCT(*context);

    context->allocator = allocator;
    context->mqtt311_test_context = mqtt311_test_context;

    aws_array_list_init_dynamic(
        &context->connection_success_events, allocator, 10, sizeof(struct mqtt311_listener_connection_success_record));
    aws_array_list_init_dynamic(
        &context->connection_interrupted_events,
        allocator,
        10,
        sizeof(struct mqtt311_listener_connection_interrupted_record));
    aws_array_list_init_dynamic(
        &context->publish_events, allocator, 10, sizeof(struct mqtt311_listener_publish_record));

    aws_mutex_init(&context->lock);
    aws_condition_variable_init(&context->signal);

    context->mqtt311_test_context_setup_result = aws_test311_setup_mqtt_server_fn(allocator, mqtt311_test_context);
    ASSERT_SUCCESS(context->mqtt311_test_context_setup_result);

    struct aws_mqtt311_listener_config listener_config = {
        .connection = mqtt311_test_context->mqtt_connection,
        .listener_callbacks =
            {
                .publish_received_handler = s_311_listener_test_on_publish_received,
                .connection_success_handler = s_311_listener_test_on_connection_success,
                .connection_interrupted_handler = s_311_listener_test_on_connection_interrupted,
                .disconnect_handler = s_311_listener_test_on_disconnect,
                .user_data = context,
            },
        .termination_callback = s_311_listener_test_on_termination,
        .termination_callback_user_data = context,
    };

    context->listener = aws_mqtt311_listener_new(allocator, &listener_config);

    return AWS_OP_SUCCESS;
}

static bool s_is_listener_terminated(void *userdata) {
    struct mqtt311_listener_test_context *context = userdata;

    return context->terminated;
}

static void s_wait_for_listener_termination_callback(struct mqtt311_listener_test_context *context) {
    aws_mutex_lock(&context->lock);
    aws_condition_variable_wait_pred(&context->signal, &context->lock, s_is_listener_terminated, context);
    aws_mutex_unlock(&context->lock);
}

static void mqtt311_listener_test_context_clean_up(struct mqtt311_listener_test_context *context) {
    aws_mqtt311_listener_release(context->listener);
    s_wait_for_listener_termination_callback(context);

    aws_test311_clean_up_mqtt_server_fn(
        context->allocator, context->mqtt311_test_context_setup_result, context->mqtt311_test_context);

    aws_mutex_clean_up(&context->lock);
    aws_condition_variable_clean_up(&context->signal);

    aws_array_list_clean_up(&context->connection_success_events);
    aws_array_list_clean_up(&context->connection_interrupted_events);

    for (size_t i = 0; i < aws_array_list_length(&context->publish_events); ++i) {
        struct mqtt311_listener_publish_record publish_record;
        AWS_ZERO_STRUCT(publish_record);

        aws_array_list_get_at(&context->publish_events, &publish_record, i);

        aws_byte_buf_clean_up(&publish_record.topic);
        aws_byte_buf_clean_up(&publish_record.payload);
    }

    aws_array_list_clean_up(&context->publish_events);
}

struct connection_success_event_test_context {
    struct mqtt311_listener_test_context *context;
    bool joined_session;
    size_t expected_count;
};

static bool s_contains_connection_success_events(void *userdata) {
    struct connection_success_event_test_context *wait_context = userdata;
    struct mqtt311_listener_test_context *context = wait_context->context;

    size_t found = 0;
    for (size_t i = 0; i < aws_array_list_length(&context->connection_success_events); ++i) {
        struct mqtt311_listener_connection_success_record record;
        aws_array_list_get_at(&context->connection_success_events, &record, i);

        if (record.joined_session == wait_context->joined_session) {
            ++found;
        }
    }

    return found >= wait_context->expected_count;
}

static void s_wait_for_connection_success_events(
    struct mqtt311_listener_test_context *context,
    bool joined_session,
    size_t expected_count) {
    struct connection_success_event_test_context wait_context = {
        .context = context,
        .joined_session = joined_session,
        .expected_count = expected_count,
    };

    aws_mutex_lock(&context->lock);
    aws_condition_variable_wait_pred(
        &context->signal, &context->lock, s_contains_connection_success_events, &wait_context);
    aws_mutex_unlock(&context->lock);
}

struct connection_interrupted_event_test_context {
    struct mqtt311_listener_test_context *context;
    int error_code;
    size_t expected_count;
};

static bool s_contains_connection_interrupted_events(void *userdata) {
    struct connection_interrupted_event_test_context *wait_context = userdata;
    struct mqtt311_listener_test_context *context = wait_context->context;

    size_t found = 0;
    for (size_t i = 0; i < aws_array_list_length(&context->connection_interrupted_events); ++i) {
        struct mqtt311_listener_connection_interrupted_record record;
        aws_array_list_get_at(&context->connection_interrupted_events, &record, i);

        if (record.error_code != wait_context->error_code) {
            continue;
        }

        ++found;
    }

    return found >= wait_context->expected_count;
}

static void s_wait_for_connection_interrupted_events(
    struct mqtt311_listener_test_context *context,
    int error_code,
    size_t expected_count) {
    struct connection_interrupted_event_test_context wait_context = {
        .context = context,
        .error_code = error_code,
        .expected_count = expected_count,
    };

    aws_mutex_lock(&context->lock);
    aws_condition_variable_wait_pred(
        &context->signal, &context->lock, s_contains_connection_interrupted_events, &wait_context);
    aws_mutex_unlock(&context->lock);
}

struct disconnect_event_test_context {
    struct mqtt311_listener_test_context *context;
    size_t expected_count;
};

static bool s_contains_disconnect_events(void *userdata) {
    struct disconnect_event_test_context *wait_context = userdata;

    return wait_context->context->disconnect_event_count >= wait_context->expected_count;
}

static void s_wait_for_disconnect_events(struct mqtt311_listener_test_context *context, size_t expected_count) {
    struct disconnect_event_test_context wait_context = {
        .context = context,
        .expected_count = expected_count,
    };

    aws_mutex_lock(&context->lock);
    aws_condition_variable_wait_pred(&context->signal, &context->lock, s_contains_disconnect_events, &wait_context);
    aws_mutex_unlock(&context->lock);
}

static int s_do_mqtt311_listener_connection_events_test(struct aws_allocator *allocator, bool session_present) {
    aws_mqtt_library_init(allocator);

    struct mqtt_connection_state_test mqtt311_context;
    AWS_ZERO_STRUCT(mqtt311_context);

    struct mqtt311_listener_test_context test_context;
    ASSERT_SUCCESS(mqtt311_listener_test_context_init(&test_context, allocator, &mqtt311_context));

    mqtt_mock_server_set_session_present(mqtt311_context.mock_server, session_present);

    struct aws_mqtt_connection_options connection_options = {
        .user_data = &mqtt311_context,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(mqtt311_context.endpoint.address),
        .socket_options = &mqtt311_context.socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .keep_alive_time_secs = DEFAULT_TEST_KEEP_ALIVE_S,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(mqtt311_context.mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(&mqtt311_context);

    s_wait_for_connection_success_events(&test_context, session_present, 1);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        mqtt311_context.mqtt_connection, aws_test311_on_disconnect_fn, &mqtt311_context));
    aws_test311_wait_for_disconnect_to_complete(&mqtt311_context);

    s_wait_for_disconnect_events(&test_context, 1);

    mqtt_mock_server_set_max_ping_resp(mqtt311_context.mock_server, 0);

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(mqtt311_context.mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(&mqtt311_context);

    // the ping configuration leads to an interruption event after 3 seconds due to ping timeout
    s_wait_for_connection_interrupted_events(&test_context, AWS_ERROR_MQTT_TIMEOUT, 1);

    aws_test311_wait_for_reconnect_to_complete(&mqtt311_context);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        mqtt311_context.mqtt_connection, aws_test311_on_disconnect_fn, &mqtt311_context));
    aws_test311_wait_for_disconnect_to_complete(&mqtt311_context);

    s_wait_for_disconnect_events(&test_context, 2);

    mqtt311_listener_test_context_clean_up(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt311_listener_connection_events_no_session_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_mqtt311_listener_connection_events_test(allocator, false);
}

AWS_TEST_CASE(mqtt311_listener_connection_events_no_session, s_mqtt311_listener_connection_events_no_session_fn)

static int s_mqtt311_listener_connection_events_with_session_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_mqtt311_listener_connection_events_test(allocator, true);
}

AWS_TEST_CASE(mqtt311_listener_connection_events_with_session, s_mqtt311_listener_connection_events_with_session_fn)

struct publish_event_test_context {
    struct mqtt311_listener_test_context *context;
    struct aws_byte_cursor expected_topic;
    struct aws_byte_cursor expected_payload;
    size_t expected_count;
};

static bool s_contains_publish_events(void *userdata) {
    struct publish_event_test_context *wait_context = userdata;
    struct mqtt311_listener_test_context *context = wait_context->context;

    size_t found = 0;
    for (size_t i = 0; i < aws_array_list_length(&context->publish_events); ++i) {
        struct mqtt311_listener_publish_record record;
        aws_array_list_get_at(&context->publish_events, &record, i);

        struct aws_byte_cursor actual_topic = aws_byte_cursor_from_buf(&record.topic);
        if (!aws_byte_cursor_eq(&wait_context->expected_topic, &actual_topic)) {
            continue;
        }

        struct aws_byte_cursor actual_payload = aws_byte_cursor_from_buf(&record.payload);
        if (!aws_byte_cursor_eq(&wait_context->expected_payload, &actual_payload)) {
            continue;
        }

        ++found;
    }

    return found >= wait_context->expected_count;
}

static void s_wait_for_publish_events(
    struct mqtt311_listener_test_context *context,
    struct aws_byte_cursor topic,
    struct aws_byte_cursor payload,
    size_t expected_count) {
    struct publish_event_test_context wait_context = {
        .context = context,
        .expected_topic = topic,
        .expected_payload = payload,
        .expected_count = expected_count,
    };

    aws_mutex_lock(&context->lock);
    aws_condition_variable_wait_pred(&context->signal, &context->lock, s_contains_publish_events, &wait_context);
    aws_mutex_unlock(&context->lock);
}

static int s_mqtt311_listener_publish_event_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_connection_state_test mqtt311_context;
    AWS_ZERO_STRUCT(mqtt311_context);

    struct mqtt311_listener_test_context test_context;
    ASSERT_SUCCESS(mqtt311_listener_test_context_init(&test_context, allocator, &mqtt311_context));

    mqtt_mock_server_set_publish_reflection(mqtt311_context.mock_server, true);

    struct aws_mqtt_connection_options connection_options = {
        .user_data = &mqtt311_context,
        .clean_session = true,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(mqtt311_context.endpoint.address),
        .socket_options = &mqtt311_context.socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(mqtt311_context.mqtt_connection, &connection_options));
    aws_test311_wait_for_connection_to_complete(&mqtt311_context);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("hello/world/1");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("payload1");
    aws_mqtt_client_connection_publish(
        mqtt311_context.mqtt_connection, &topic1, AWS_MQTT_QOS_AT_LEAST_ONCE, false, &payload1, NULL, NULL);

    s_wait_for_publish_events(&test_context, topic1, payload1, 1);

    struct aws_byte_cursor topic2 = aws_byte_cursor_from_c_str("nothing/important");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("somethingneeddoing?");
    aws_mqtt_client_connection_publish(
        mqtt311_context.mqtt_connection, &topic2, AWS_MQTT_QOS_AT_LEAST_ONCE, false, &payload2, NULL, NULL);
    aws_mqtt_client_connection_publish(
        mqtt311_context.mqtt_connection, &topic2, AWS_MQTT_QOS_AT_LEAST_ONCE, false, &payload2, NULL, NULL);

    s_wait_for_publish_events(&test_context, topic2, payload2, 2);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(
        mqtt311_context.mqtt_connection, aws_test311_on_disconnect_fn, &mqtt311_context));
    aws_test311_wait_for_disconnect_to_complete(&mqtt311_context);

    mqtt311_listener_test_context_clean_up(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt311_listener_publish_event, s_mqtt311_listener_publish_event_fn)