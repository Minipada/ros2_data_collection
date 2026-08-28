/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt5_testing_utils.h"

#include <aws/common/clock.h>
#include <aws/common/string.h>
#include <aws/http/websocket.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/mqtt/client.h>
#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/v5/mqtt5_to_mqtt3_adapter_impl.h>
#include <aws/mqtt/v5/mqtt5_client.h>

#include <aws/testing/aws_test_harness.h>

#include <inttypes.h>

enum aws_mqtt3_lifecycle_event_type {
    AWS_MQTT3_LET_CONNECTION_COMPLETE,
    AWS_MQTT3_LET_INTERRUPTED,
    AWS_MQTT3_LET_RESUMED,
    AWS_MQTT3_LET_CLOSED,
    AWS_MQTT3_LET_DISCONNECTION_COMPLETE,
    AWS_MQTT3_LET_CONNECTION_SUCCESS,
    AWS_MQTT3_LET_CONNECTION_FAILURE,
    AWS_MQTT3_LET_TERMINATION,
};

struct aws_mqtt3_lifecycle_event {
    enum aws_mqtt3_lifecycle_event_type type;

    uint64_t timestamp;
    int error_code;
    enum aws_mqtt_connect_return_code return_code;
    bool session_present;

    bool skip_error_code_equality;
};

enum aws_mqtt3_operation_event_type {
    AWS_MQTT3_OET_PUBLISH_COMPLETE,
    AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
    AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE,
    AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
    AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
};

struct aws_mqtt3_operation_event {
    enum aws_mqtt3_operation_event_type type;

    uint64_t timestamp;
    int error_code;

    // publish received properties
    enum aws_mqtt_qos qos;
    struct aws_byte_buf topic;
    struct aws_byte_cursor topic_cursor;
    struct aws_byte_buf payload;
    struct aws_byte_cursor payload_cursor;

    // subscribe complete properties
    struct aws_array_list granted_subscriptions;

    struct aws_byte_buf topic_storage;

    /*
     * Not a part of recorded events, instead used to help verification check number of occurrences.  Only
     * used by the "contains" verification function which wouldn't otherise be able to check if an exact
     * event appears multiple times in the set.
     */
    size_t expected_count;
};

static void s_aws_mqtt3_operation_event_clean_up(struct aws_mqtt3_operation_event *event) {
    if (event == NULL) {
        return;
    }

    aws_byte_buf_clean_up(&event->topic);
    aws_byte_buf_clean_up(&event->payload);
    aws_byte_buf_clean_up(&event->topic_storage);

    aws_array_list_clean_up(&event->granted_subscriptions);
}

static bool s_aws_mqtt3_operation_event_equals(
    struct aws_mqtt3_operation_event *expected,
    struct aws_mqtt3_operation_event *actual) {
    if (expected->type != actual->type) {
        return false;
    }

    if (expected->error_code != actual->error_code) {
        return false;
    }

    if (expected->qos != actual->qos) {
        return false;
    }

    if (expected->topic_cursor.len != actual->topic_cursor.len) {
        return false;
    }

    if (expected->topic_cursor.len > 0) {
        if (memcmp(expected->topic_cursor.ptr, actual->topic_cursor.ptr, expected->topic_cursor.len) != 0) {
            return false;
        }
    }

    if (expected->payload_cursor.len != actual->payload_cursor.len) {
        return false;
    }

    if (expected->payload_cursor.len > 0) {
        if (memcmp(expected->payload_cursor.ptr, actual->payload_cursor.ptr, expected->payload_cursor.len) != 0) {
            return false;
        }
    }

    if (aws_array_list_length(&expected->granted_subscriptions) !=
        aws_array_list_length(&actual->granted_subscriptions)) {
        return false;
    }

    for (size_t i = 0; i < aws_array_list_length(&expected->granted_subscriptions); ++i) {

        struct aws_mqtt_topic_subscription expected_sub;
        aws_array_list_get_at(&expected->granted_subscriptions, &expected_sub, i);

        bool found_match = false;
        for (size_t j = 0; j < aws_array_list_length(&actual->granted_subscriptions); ++j) {
            struct aws_mqtt_topic_subscription actual_sub;
            aws_array_list_get_at(&actual->granted_subscriptions, &actual_sub, j);

            if (expected_sub.qos != actual_sub.qos) {
                continue;
            }

            if (expected_sub.topic.len != actual_sub.topic.len) {
                continue;
            }

            if (expected_sub.topic.len > 0) {
                if (memcmp(expected_sub.topic.ptr, actual_sub.topic.ptr, expected_sub.topic.len) != 0) {
                    continue;
                }
            }

            found_match = true;
            break;
        }

        if (!found_match) {
            return false;
        }
    }

    return true;
}

struct aws_mqtt5_to_mqtt3_adapter_test_fixture {
    struct aws_mqtt5_client_mock_test_fixture mqtt5_fixture;

    struct aws_mqtt_client_connection *connection;

    struct aws_array_list lifecycle_events;

    struct aws_array_list operation_events;

    struct aws_mutex lock;
    struct aws_condition_variable signal;
};

static void s_init_adapter_connection_options_from_fixture(
    struct aws_mqtt_connection_options *connection_options,
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture) {
    AWS_ZERO_STRUCT(*connection_options);

    connection_options->host_name = aws_byte_cursor_from_c_str(fixture->mqtt5_fixture.endpoint.address);
    connection_options->port = fixture->mqtt5_fixture.endpoint.port;
    connection_options->socket_options = &fixture->mqtt5_fixture.socket_options;
    connection_options->keep_alive_time_secs = 30;
    connection_options->ping_timeout_ms = 10000;
    connection_options->clean_session = true;
}

static int s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    size_t expected_event_count,
    struct aws_mqtt3_operation_event *expected_events,
    size_t maximum_event_count) {

    aws_mutex_lock(&fixture->lock);

    size_t actual_event_count = aws_array_list_length(&fixture->operation_events);
    ASSERT_TRUE(expected_event_count <= actual_event_count);
    ASSERT_TRUE(actual_event_count <= maximum_event_count);

    for (size_t i = 0; i < expected_event_count; ++i) {
        struct aws_mqtt3_operation_event *expected_event = expected_events + i;
        struct aws_mqtt3_operation_event *actual_event = NULL;
        aws_array_list_get_at_ptr(&fixture->operation_events, (void **)(&actual_event), i);

        ASSERT_TRUE(s_aws_mqtt3_operation_event_equals(expected_event, actual_event));
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    size_t expected_event_count,
    struct aws_mqtt3_operation_event *expected_events) {

    aws_mutex_lock(&fixture->lock);

    size_t actual_event_count = aws_array_list_length(&fixture->operation_events);

    for (size_t i = 0; i < expected_event_count; ++i) {
        struct aws_mqtt3_operation_event *expected_event = expected_events + i;
        size_t match_count = 0;

        for (size_t j = 0; j < actual_event_count; ++j) {
            struct aws_mqtt3_operation_event *actual_event = NULL;
            aws_array_list_get_at_ptr(&fixture->operation_events, (void **)(&actual_event), j);

            if (s_aws_mqtt3_operation_event_equals(expected_event, actual_event)) {
                ++match_count;
            }
        }

        if (expected_event->expected_count == 0) {
            ASSERT_INT_EQUALS(1, match_count);
        } else {
            ASSERT_INT_EQUALS(expected_event->expected_count, match_count);
        }
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

struct n_operation_event_wait_context {
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture;
    enum aws_mqtt3_operation_event_type type;
    size_t count;
};

static bool s_wait_for_n_adapter_operation_events_predicate(void *context) {
    struct n_operation_event_wait_context *wait_context = context;
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = wait_context->fixture;

    size_t actual_count = 0;
    size_t event_count = aws_array_list_length(&fixture->operation_events);
    for (size_t i = 0; i < event_count; ++i) {
        struct aws_mqtt3_operation_event *actual_event = NULL;
        aws_array_list_get_at_ptr(&fixture->operation_events, (void **)(&actual_event), i);
        if (actual_event->type == wait_context->type) {
            ++actual_count;
        }
    }

    return actual_count >= wait_context->count;
}

static void s_wait_for_n_adapter_operation_events(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    enum aws_mqtt3_operation_event_type type,
    size_t count) {
    struct n_operation_event_wait_context wait_context = {
        .fixture = fixture,
        .type = type,
        .count = count,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(
        &fixture->signal, &fixture->lock, s_wait_for_n_adapter_operation_events_predicate, &wait_context);
    aws_mutex_unlock(&fixture->lock);
}

struct n_lifeycle_event_wait_context {
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture;
    enum aws_mqtt3_lifecycle_event_type type;
    size_t count;
};

static bool s_wait_for_n_adapter_lifecycle_events_predicate(void *context) {
    struct n_lifeycle_event_wait_context *wait_context = context;
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = wait_context->fixture;

    size_t actual_count = 0;
    size_t event_count = aws_array_list_length(&fixture->lifecycle_events);
    for (size_t i = 0; i < event_count; ++i) {
        struct aws_mqtt3_lifecycle_event *actual_event = NULL;
        aws_array_list_get_at_ptr(&fixture->lifecycle_events, (void **)(&actual_event), i);
        if (actual_event->type == wait_context->type) {
            ++actual_count;
        }
    }

    return actual_count >= wait_context->count;
}

static void s_wait_for_n_adapter_lifecycle_events(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    enum aws_mqtt3_lifecycle_event_type type,
    size_t count) {
    struct n_lifeycle_event_wait_context wait_context = {
        .fixture = fixture,
        .type = type,
        .count = count,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(
        &fixture->signal, &fixture->lock, s_wait_for_n_adapter_lifecycle_events_predicate, &wait_context);
    aws_mutex_unlock(&fixture->lock);
}

static int s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_event(
    struct aws_mqtt3_lifecycle_event *expected_event,
    struct aws_mqtt3_lifecycle_event *actual_event) {
    ASSERT_INT_EQUALS(actual_event->type, expected_event->type);
    if (expected_event->skip_error_code_equality) {
        /* some error scenarios lead to different values cross-platform, so just verify yes/no in that case */
        ASSERT_TRUE((actual_event->error_code != 0) == (expected_event->error_code != 0));
    } else {
        ASSERT_INT_EQUALS(actual_event->error_code, expected_event->error_code);
    }

    ASSERT_INT_EQUALS(actual_event->return_code, expected_event->return_code);
    ASSERT_TRUE(actual_event->session_present == expected_event->session_present);

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    size_t expected_event_count,
    struct aws_mqtt3_lifecycle_event *expected_events,
    size_t maximum_event_count) {

    aws_mutex_lock(&fixture->lock);

    size_t actual_event_count = aws_array_list_length(&fixture->lifecycle_events);
    ASSERT_TRUE(expected_event_count <= actual_event_count);
    ASSERT_TRUE(actual_event_count <= maximum_event_count);

    for (size_t i = 0; i < expected_event_count; ++i) {
        struct aws_mqtt3_lifecycle_event *expected_event = expected_events + i;
        struct aws_mqtt3_lifecycle_event *actual_event = NULL;
        aws_array_list_get_at_ptr(&fixture->lifecycle_events, (void **)(&actual_event), i);

        ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_event(expected_event, actual_event));
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence_starts_with(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    size_t expected_event_count,
    struct aws_mqtt3_lifecycle_event *expected_events) {

    aws_mutex_lock(&fixture->lock);

    size_t actual_event_count = aws_array_list_length(&fixture->lifecycle_events);
    ASSERT_TRUE(expected_event_count <= actual_event_count);

    for (size_t i = 0; i < expected_event_count; ++i) {
        struct aws_mqtt3_lifecycle_event *expected_event = expected_events + i;
        struct aws_mqtt3_lifecycle_event *actual_event = NULL;
        aws_array_list_get_at_ptr(&fixture->lifecycle_events, (void **)(&actual_event), i);

        ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_event(expected_event, actual_event));
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence_ends_with(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    size_t expected_event_count,
    struct aws_mqtt3_lifecycle_event *expected_events) {

    aws_mutex_lock(&fixture->lock);

    size_t actual_event_count = aws_array_list_length(&fixture->lifecycle_events);
    ASSERT_TRUE(expected_event_count <= actual_event_count);

    for (size_t i = 0; i < expected_event_count; ++i) {
        struct aws_mqtt3_lifecycle_event *expected_event = expected_events + i;

        size_t actual_index = i + (actual_event_count - expected_event_count);
        struct aws_mqtt3_lifecycle_event *actual_event = NULL;
        aws_array_list_get_at_ptr(&fixture->lifecycle_events, (void **)(&actual_event), actual_index);

        ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_event(expected_event, actual_event));
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_closed_handler(
    struct aws_mqtt_client_connection *connection,
    struct on_connection_closed_data *data,
    void *userdata) {

    (void)connection;
    (void)data;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    /* record the event */
    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_CLOSED;
    aws_high_res_clock_get_ticks(&event.timestamp);

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_termination_handler(void *userdata) {
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    /* record the event */
    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_TERMINATION;
    aws_high_res_clock_get_ticks(&event.timestamp);

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_interrupted_handler(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    void *userdata) {

    (void)connection;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    /* record the event */
    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_INTERRUPTED;
    aws_high_res_clock_get_ticks(&event.timestamp);
    event.error_code = error_code;

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_resumed_handler(
    struct aws_mqtt_client_connection *connection,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {

    (void)connection;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    /* record the event */
    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_RESUMED;
    aws_high_res_clock_get_ticks(&event.timestamp);
    event.return_code = return_code;
    event.session_present = session_present;

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_connection_failure_handler(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    void *userdata) {

    (void)connection;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    /* record the event */
    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_CONNECTION_FAILURE;
    aws_high_res_clock_get_ticks(&event.timestamp);
    event.error_code = error_code;

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_connection_success_handler(
    struct aws_mqtt_client_connection *connection,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *userdata) {

    (void)connection;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    /* record the event */
    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_CONNECTION_SUCCESS;
    aws_high_res_clock_get_ticks(&event.timestamp);
    event.return_code = return_code;
    event.session_present = session_present;

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete(
    struct aws_mqtt_client_connection *connection,
    int error_code,
    enum aws_mqtt_connect_return_code return_code,
    bool session_present,
    void *user_data) {
    (void)connection;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = user_data;

    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_CONNECTION_COMPLETE;
    aws_high_res_clock_get_ticks(&event.timestamp);
    event.error_code = error_code;
    event.return_code = return_code;
    event.session_present = session_present;

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete(
    struct aws_mqtt_client_connection *connection,
    void *user_data) {
    (void)connection;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = user_data;

    struct aws_mqtt3_lifecycle_event event;
    AWS_ZERO_STRUCT(event);

    event.type = AWS_MQTT3_LET_DISCONNECTION_COMPLETE;
    aws_high_res_clock_get_ticks(&event.timestamp);

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->lifecycle_events, &event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

int aws_mqtt5_to_mqtt3_adapter_test_fixture_init(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    struct aws_allocator *allocator,
    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *mqtt5_fixture_config) {
    AWS_ZERO_STRUCT(*fixture);

    if (aws_mqtt5_client_mock_test_fixture_init(&fixture->mqtt5_fixture, allocator, mqtt5_fixture_config)) {
        return AWS_OP_ERR;
    }

    fixture->connection = aws_mqtt_client_connection_new_from_mqtt5_client(fixture->mqtt5_fixture.client);
    if (fixture->connection == NULL) {
        return AWS_OP_ERR;
    }

    aws_array_list_init_dynamic(&fixture->lifecycle_events, allocator, 10, sizeof(struct aws_mqtt3_lifecycle_event));
    aws_array_list_init_dynamic(&fixture->operation_events, allocator, 10, sizeof(struct aws_mqtt3_operation_event));

    aws_mutex_init(&fixture->lock);
    aws_condition_variable_init(&fixture->signal);

    aws_mqtt_client_connection_set_connection_termination_handler(
        fixture->connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_termination_handler, fixture);
    aws_mqtt_client_connection_set_connection_closed_handler(
        fixture->connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_closed_handler, fixture);
    aws_mqtt_client_connection_set_connection_interruption_handlers(
        fixture->connection,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_interrupted_handler,
        fixture,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_resumed_handler,
        fixture);
    aws_mqtt_client_connection_set_connection_result_handlers(
        fixture->connection,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_connection_success_handler,
        fixture,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_connection_failure_handler,
        fixture);

    return AWS_OP_SUCCESS;
}

void aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture) {
    aws_mqtt_client_connection_release(fixture->connection);

    s_wait_for_n_adapter_lifecycle_events(fixture, AWS_MQTT3_LET_TERMINATION, 1);

    aws_mqtt5_client_mock_test_fixture_clean_up(&fixture->mqtt5_fixture);

    aws_array_list_clean_up(&fixture->lifecycle_events);

    size_t operation_event_count = aws_array_list_length(&fixture->operation_events);
    for (size_t i = 0; i < operation_event_count; ++i) {
        struct aws_mqtt3_operation_event *event = NULL;
        aws_array_list_get_at_ptr(&fixture->operation_events, (void **)(&event), i);

        s_aws_mqtt3_operation_event_clean_up(event);
    }
    aws_array_list_clean_up(&fixture->operation_events);

    aws_mutex_clean_up(&fixture->lock);
    aws_condition_variable_clean_up(&fixture->signal);
}

void s_mqtt5to3_lifecycle_event_callback(const struct aws_mqtt5_client_lifecycle_event *event) {
    (void)event;
}

void s_mqtt5to3_publish_received_callback(const struct aws_mqtt5_packet_publish_view *publish, void *user_data) {
    (void)publish;
    (void)user_data;
}

static int s_do_mqtt5to3_adapter_create_destroy(struct aws_allocator *allocator, uint64_t sleep_nanos) {
    aws_mqtt_library_init(allocator);

    struct aws_mqtt5_packet_connect_view local_connect_options = {
        .keep_alive_interval_seconds = 30,
        .clean_start = true,
    };

    struct aws_mqtt5_client_options client_options = {
        .connect_options = &local_connect_options,
        .lifecycle_event_handler = s_mqtt5to3_lifecycle_event_callback,
        .lifecycle_event_handler_user_data = NULL,
        .publish_received_handler = s_mqtt5to3_publish_received_callback,
        .publish_received_handler_user_data = NULL,
        .ping_timeout_ms = 10000,
    };

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_config = {
        .client_options = &client_options,
    };

    struct aws_mqtt5_client_mock_test_fixture test_fixture;
    AWS_ZERO_STRUCT(test_fixture);

    ASSERT_SUCCESS(aws_mqtt5_client_mock_test_fixture_init(&test_fixture, allocator, &test_fixture_config));

    struct aws_mqtt_client_connection *connection =
        aws_mqtt_client_connection_new_from_mqtt5_client(test_fixture.client);

    if (sleep_nanos > 0) {
        /* sleep a little just to let the listener attachment resolve */
        aws_thread_current_sleep(aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    }

    aws_mqtt_client_connection_release(connection);

    if (sleep_nanos > 0) {
        /* sleep a little just to let the listener detachment resolve */
        aws_thread_current_sleep(aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    }

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_mqtt5to3_adapter_create_destroy_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5to3_adapter_create_destroy(allocator, 0));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_create_destroy, s_mqtt5to3_adapter_create_destroy_fn)

static int s_mqtt5to3_adapter_create_destroy_delayed_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5to3_adapter_create_destroy(
        allocator, aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_create_destroy_delayed, s_mqtt5to3_adapter_create_destroy_delayed_fn)

typedef int (*mqtt5to3_adapter_config_test_setup_fn)(
    struct aws_allocator *allocator,
    struct aws_mqtt_client_connection *adapter,
    struct aws_mqtt5_packet_connect_storage *expected_connect);

static int s_do_mqtt5to3_adapter_config_test(
    struct aws_allocator *allocator,
    mqtt5to3_adapter_config_test_setup_fn setup_fn) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture test_fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&test_fixture, allocator, &test_fixture_options));

    struct aws_mqtt5_client *client = test_fixture.mqtt5_fixture.client;

    struct aws_mqtt_client_connection *adapter = test_fixture.connection;

    struct aws_mqtt5_packet_connect_storage expected_connect_storage;
    ASSERT_SUCCESS((*setup_fn)(allocator, adapter, &expected_connect_storage));

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&test_fixture.mqtt5_fixture);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_fixture.mqtt5_fixture);

    struct aws_mqtt5_mock_server_packet_record expected_packets[] = {
        {
            .packet_type = AWS_MQTT5_PT_CONNECT,
            .packet_storage = &expected_connect_storage,
        },
    };
    ASSERT_SUCCESS(aws_verify_received_packet_sequence(
        &test_fixture.mqtt5_fixture, expected_packets, AWS_ARRAY_SIZE(expected_packets)));

    aws_mqtt5_packet_connect_storage_clean_up(&expected_connect_storage);

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&test_fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_STATIC_STRING_FROM_LITERAL(s_simple_topic, "Hello/World");
AWS_STATIC_STRING_FROM_LITERAL(s_simple_payload, "A Payload");

static int s_mqtt5to3_adapter_set_will_setup(
    struct aws_allocator *allocator,
    struct aws_mqtt_client_connection *adapter,
    struct aws_mqtt5_packet_connect_storage *expected_connect) {

    struct aws_byte_cursor topic_cursor = aws_byte_cursor_from_string(s_simple_topic);
    struct aws_byte_cursor payload_cursor = aws_byte_cursor_from_string(s_simple_payload);

    ASSERT_SUCCESS(
        aws_mqtt_client_connection_set_will(adapter, &topic_cursor, AWS_MQTT_QOS_AT_LEAST_ONCE, true, &payload_cursor));

    struct aws_mqtt5_packet_publish_view expected_will = {
        .payload = payload_cursor,
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .retain = true,
        .topic = topic_cursor,
    };

    struct aws_mqtt5_packet_connect_view expected_connect_view = {
        .client_id = aws_byte_cursor_from_string(g_default_client_id),
        .keep_alive_interval_seconds = 30,
        .clean_start = true,
        .will = &expected_will,
    };

    ASSERT_SUCCESS(aws_mqtt5_packet_connect_storage_init(expected_connect, allocator, &expected_connect_view));

    return AWS_OP_SUCCESS;
}

static int s_mqtt5to3_adapter_set_will_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5to3_adapter_config_test(allocator, s_mqtt5to3_adapter_set_will_setup));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_set_will, s_mqtt5to3_adapter_set_will_fn)

AWS_STATIC_STRING_FROM_LITERAL(s_username, "MyUsername");
AWS_STATIC_STRING_FROM_LITERAL(s_password, "TopTopSecret");

static int s_mqtt5to3_adapter_set_login_setup(
    struct aws_allocator *allocator,
    struct aws_mqtt_client_connection *adapter,
    struct aws_mqtt5_packet_connect_storage *expected_connect) {

    struct aws_byte_cursor username_cursor = aws_byte_cursor_from_string(s_username);
    struct aws_byte_cursor password_cursor = aws_byte_cursor_from_string(s_password);

    ASSERT_SUCCESS(aws_mqtt_client_connection_set_login(adapter, &username_cursor, &password_cursor));

    struct aws_mqtt5_packet_connect_view expected_connect_view = {
        .client_id = aws_byte_cursor_from_string(g_default_client_id),
        .keep_alive_interval_seconds = 30,
        .clean_start = true,
        .username = &username_cursor,
        .password = &password_cursor,
    };

    ASSERT_SUCCESS(aws_mqtt5_packet_connect_storage_init(expected_connect, allocator, &expected_connect_view));

    return AWS_OP_SUCCESS;
}

static int s_mqtt5to3_adapter_set_login_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5to3_adapter_config_test(allocator, s_mqtt5to3_adapter_set_login_setup));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_set_login, s_mqtt5to3_adapter_set_login_fn)

static int s_mqtt5to3_adapter_set_reconnect_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    /*
     * This is a variant of the mqtt5_client_reconnect_failure_backoff test.
     *
     * The primary change is that we configure the mqtt5 client with "wrong" (fast) reconnect delays and then use
     * the adapter API to configure with the "right" ones that will let the test pass.
     */
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* backoff delay sequence: 500, 1000, 2000, 4000, 5000, ... */
    test_options.client_options.retry_jitter_mode = AWS_EXPONENTIAL_BACKOFF_JITTER_NONE;
    test_options.client_options.min_reconnect_delay_ms = 10;
    test_options.client_options.max_reconnect_delay_ms = 50;
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

    struct aws_mqtt_client_connection *adapter = aws_mqtt_client_connection_new_from_mqtt5_client(client);

    aws_mqtt_client_connection_set_reconnect_timeout(adapter, RECONNECT_TEST_MIN_BACKOFF, RECONNECT_TEST_MAX_BACKOFF);

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_mqtt5_wait_for_n_lifecycle_events(&test_context, AWS_MQTT5_CLET_CONNECTION_FAILURE, 6);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    aws_wait_for_stopped_lifecycle_event(&test_context);

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

    aws_mqtt_client_connection_release(adapter);

    aws_mqtt5_client_mock_test_fixture_clean_up(&test_context);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_set_reconnect_timeout, s_mqtt5to3_adapter_set_reconnect_timeout_fn)

/*
 * Basic successful connection test
 */
static int s_mqtt5to3_adapter_connect_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_mqtt3_lifecycle_event expected_events[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_connect_success, s_mqtt5to3_adapter_connect_success_fn)

static int s_do_mqtt5to3_adapter_connect_success_disconnect_success_cycle(
    struct aws_allocator *allocator,
    size_t iterations) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    for (size_t i = 0; i < iterations; ++i) {
        struct aws_mqtt_connection_options connection_options;
        s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

        connection_options.on_connection_complete =
            s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
        connection_options.user_data = &fixture;

        aws_mqtt_client_connection_connect(adapter, &connection_options);

        s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, i + 1);

        aws_mqtt_client_connection_disconnect(
            adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

        s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, i + 1);
        s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CLOSED, i + 1);

        struct aws_mqtt3_lifecycle_event expected_event_sequence[] = {
            {
                .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
            },
            {
                .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
            },
            {
                .type = AWS_MQTT3_LET_DISCONNECTION_COMPLETE,
            },
            {
                .type = AWS_MQTT3_LET_CLOSED,
            },
        };
        size_t sequence_size = AWS_ARRAY_SIZE(expected_event_sequence);

        size_t expected_event_count = (i + 1) * sequence_size;
        struct aws_mqtt3_lifecycle_event *expected_events =
            aws_mem_calloc(allocator, expected_event_count, sizeof(struct aws_mqtt3_lifecycle_event));
        for (size_t j = 0; j < i + 1; ++j) {
            for (size_t k = 0; k < sequence_size; ++k) {
                *(expected_events + j * sequence_size + k) = expected_event_sequence[k];
            }
        }

        ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
            &fixture, expected_event_count, expected_events, expected_event_count));

        aws_mem_release(allocator, expected_events);
    }

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

/*
 * A couple of simple connect-disconnect cycle tests.  The first does a single cycle while the second does several.
 * Verifies proper lifecycle event sequencing.
 */
static int s_mqtt5to3_adapter_connect_success_disconnect_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5to3_adapter_connect_success_disconnect_success_cycle(allocator, 1));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_success_disconnect_success,
    s_mqtt5to3_adapter_connect_success_disconnect_success_fn)

static int s_mqtt5to3_adapter_connect_success_disconnect_success_thrice_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_mqtt5to3_adapter_connect_success_disconnect_success_cycle(allocator, 3));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_success_disconnect_success_thrice,
    s_mqtt5to3_adapter_connect_success_disconnect_success_thrice_fn)

/*
 * Verifies that calling connect() while connected yields a connection completion callback with the
 * appropriate already-connected error code.  Note that in the mqtt311 impl, this error is synchronous.
 */
static int s_mqtt5to3_adapter_connect_success_connect_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 2);

    struct aws_mqtt3_lifecycle_event expected_events[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
            .error_code = AWS_ERROR_MQTT_ALREADY_CONNECTED,
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_connect_success_connect_failure, s_mqtt5to3_adapter_connect_success_connect_failure_fn)

/*
 * A non-deterministic test that starts the connect process and immediately drops the last external adapter
 * reference.  Intended to stochastically shake out shutdown race conditions.
 */
static int s_mqtt5to3_adapter_connect_success_sloppy_shutdown_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_connect_success_sloppy_shutdown, s_mqtt5to3_adapter_connect_success_sloppy_shutdown_fn)

static int s_aws_mqtt5_server_disconnect_after_connect(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    aws_mqtt5_mock_server_handle_connect_always_succeed(packet, connection, user_data);

    struct aws_mqtt5_packet_disconnect_view disconnect = {
        .reason_code = AWS_MQTT5_DRC_SERVER_SHUTTING_DOWN,
    };

    int result = aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_DISCONNECT, &disconnect);

    return result;
}

static int s_verify_bad_connectivity_callbacks(struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture) {
    struct aws_mqtt3_lifecycle_event expected_events_start[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
        {
            .type = AWS_MQTT3_LET_INTERRUPTED,
            .error_code = AWS_ERROR_MQTT_UNEXPECTED_HANGUP,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_RESUMED,
        },
        {
            .type = AWS_MQTT3_LET_INTERRUPTED,
            .error_code = AWS_ERROR_MQTT_UNEXPECTED_HANGUP,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_RESUMED,
        },
        {
            .type = AWS_MQTT3_LET_INTERRUPTED,
            .error_code = AWS_ERROR_MQTT_UNEXPECTED_HANGUP,
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence_starts_with(
        fixture, AWS_ARRAY_SIZE(expected_events_start), expected_events_start));

    struct aws_mqtt3_lifecycle_event expected_events_end[] = {
        {
            .type = AWS_MQTT3_LET_DISCONNECTION_COMPLETE,
        },
        {
            .type = AWS_MQTT3_LET_CLOSED,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence_ends_with(
        fixture, AWS_ARRAY_SIZE(expected_events_end), expected_events_end));

    return AWS_OP_SUCCESS;
}

static int s_do_bad_connectivity_basic_test(struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture) {
    struct aws_mqtt_client_connection *adapter = fixture->connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(fixture, AWS_MQTT3_LET_INTERRUPTED, 3);

    aws_mqtt_client_connection_disconnect(
        adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, fixture);

    s_wait_for_n_adapter_lifecycle_events(fixture, AWS_MQTT3_LET_CLOSED, 1);

    ASSERT_SUCCESS(s_verify_bad_connectivity_callbacks(fixture));

    return AWS_OP_SUCCESS;
}

/*
 * A test where each successful connection is immediately dropped after the connack is sent.  Allows us to verify
 * proper interrupt/resume sequencing.
 */
static int s_mqtt5to3_adapter_connect_bad_connectivity_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* So that the test doesn't get excessively slow due to all the reconnects with backoff */
    test_options.client_options.min_reconnect_delay_ms = 500;
    test_options.client_options.max_reconnect_delay_ms = 1000;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_server_disconnect_after_connect;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    ASSERT_SUCCESS(s_do_bad_connectivity_basic_test(&fixture));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_connect_bad_connectivity, s_mqtt5to3_adapter_connect_bad_connectivity_fn)

/*
 * A variant of the bad connectivity test where we restart the mqtt5 client after the main test is over and verify
 * we don't get any interrupt/resume callbacks.
 */
static int s_mqtt5to3_adapter_connect_bad_connectivity_with_mqtt5_restart_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* So that the test doesn't get excessively slow due to all the reconnects with backoff */
    test_options.client_options.min_reconnect_delay_ms = 500;
    test_options.client_options.max_reconnect_delay_ms = 1000;

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_server_disconnect_after_connect;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    ASSERT_SUCCESS(s_do_bad_connectivity_basic_test(&fixture));

    /*
     * Now restart the 5 client, wait for a few more connection success/disconnect cycles, and then verify that no
     * further adapter callbacks were invoked because of this.
     */
    aws_mqtt5_client_start(fixture.mqtt5_fixture.client);

    aws_mqtt5_wait_for_n_lifecycle_events(&fixture.mqtt5_fixture, AWS_MQTT5_CLET_CONNECTION_SUCCESS, 6);

    aws_thread_current_sleep(aws_timestamp_convert(2, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    ASSERT_SUCCESS(s_verify_bad_connectivity_callbacks(&fixture));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_bad_connectivity_with_mqtt5_restart,
    s_mqtt5to3_adapter_connect_bad_connectivity_with_mqtt5_restart_fn)

int aws_mqtt5_mock_server_handle_connect_succeed_on_or_after_nth(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct aws_mqtt5_mock_server_reconnect_state *context = user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    if (context->connection_attempts >= context->required_connection_count_threshold) {
        connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
        aws_high_res_clock_get_ticks(&context->connect_timestamp);
    } else {
        connack_view.reason_code = AWS_MQTT5_CRC_NOT_AUTHORIZED;
    }

    ++context->connection_attempts;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

/*
 * Test where the initial connect is rejected, which should put the adapter to sleep.  Meanwhile followup attempts
 * are successful and the mqtt5 client itself becomes connected.
 */
static int s_mqtt5to3_adapter_connect_failure_connect_success_via_mqtt5_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt5_mock_server_reconnect_state mock_server_state = {
        .required_connection_count_threshold = 1,
    };

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_succeed_on_or_after_nth;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &mock_server_state,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    // wait for and verify a connection failure
    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_mqtt3_lifecycle_event expected_events[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT_PROTOCOL_ERROR,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
            .error_code = AWS_ERROR_MQTT_PROTOCOL_ERROR,
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    // wait for the mqtt5 client to successfully connect on the second try
    aws_mqtt5_wait_for_n_lifecycle_events(&fixture.mqtt5_fixture, AWS_MQTT5_CLET_CONNECTION_SUCCESS, 1);

    // verify we didn't get any callbacks on the adapter
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    // "connect" on the adapter, wait for and verify success
    aws_mqtt_client_connection_connect(adapter, &connection_options);
    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 2);

    struct aws_mqtt3_lifecycle_event expected_reconnect_events[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT_PROTOCOL_ERROR,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
            .error_code = AWS_ERROR_MQTT_PROTOCOL_ERROR,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture,
        AWS_ARRAY_SIZE(expected_reconnect_events),
        expected_reconnect_events,
        AWS_ARRAY_SIZE(expected_reconnect_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_failure_connect_success_via_mqtt5,
    s_mqtt5to3_adapter_connect_failure_connect_success_via_mqtt5_fn)

AWS_STATIC_STRING_FROM_LITERAL(s_bad_host_name, "derpity_derp");

/*
 * Fails to connect with a bad config.  Follow up with a good config.  Verifies that config is re-evaluated with
 * each connect() invocation.
 */
static int s_mqtt5to3_adapter_connect_failure_bad_config_success_good_config_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);
    struct aws_byte_cursor good_host_name = connection_options.host_name;
    connection_options.host_name = aws_byte_cursor_from_string(s_bad_host_name);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    // wait for and verify a connection failure
    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_mqtt3_lifecycle_event expected_events[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_FILE_INVALID_PATH,
            .skip_error_code_equality = true, /* the error code here is platform-dependent */
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
            .error_code = AWS_ERROR_FILE_INVALID_PATH,
            .skip_error_code_equality = true, /* the error code here is platform-dependent */
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    // reconnect with a good host the adapter, wait for and verify success
    connection_options.host_name = good_host_name;
    aws_mqtt_client_connection_connect(adapter, &connection_options);
    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 2);

    struct aws_mqtt3_lifecycle_event expected_reconnect_events[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_FILE_INVALID_PATH,
            .skip_error_code_equality = true, /* the error code here is platform-dependent */
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
            .error_code = AWS_ERROR_FILE_INVALID_PATH,
            .skip_error_code_equality = true, /* the error code here is platform-dependent */
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture,
        AWS_ARRAY_SIZE(expected_reconnect_events),
        expected_reconnect_events,
        AWS_ARRAY_SIZE(expected_reconnect_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_failure_bad_config_success_good_config,
    s_mqtt5to3_adapter_connect_failure_bad_config_success_good_config_fn)

int aws_mqtt5_mock_server_handle_connect_fail_on_or_after_nth(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct aws_mqtt5_mock_server_reconnect_state *context = user_data;

    bool send_disconnect = false;
    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    if (context->connection_attempts >= context->required_connection_count_threshold) {
        connack_view.reason_code = AWS_MQTT5_CRC_NOT_AUTHORIZED;
    } else {
        connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
        aws_high_res_clock_get_ticks(&context->connect_timestamp);
        send_disconnect = true;
    }

    ++context->connection_attempts;

    aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);

    if (send_disconnect) {
        struct aws_mqtt5_packet_disconnect_view disconnect = {
            .reason_code = AWS_MQTT5_DRC_SERVER_SHUTTING_DOWN,
        };

        aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_DISCONNECT, &disconnect);
    }

    return AWS_OP_SUCCESS;
}

/*
 * Establishes a successful connection then drops it followed by a perma-failure loop, verify we receive
 * the new connection failure callbacks.
 */
static int s_mqtt5to3_adapter_connect_reconnect_failures_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt5_mock_server_reconnect_state mock_server_state = {
        .required_connection_count_threshold = 1,
    };

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_fail_on_or_after_nth;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
        .mock_server_user_data = &mock_server_state,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);
    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_FAILURE, 3);

    aws_mqtt_client_connection_disconnect(adapter, NULL, NULL);

    struct aws_mqtt3_lifecycle_event expected_events[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
        {
            .type = AWS_MQTT3_LET_INTERRUPTED,
            .error_code = AWS_ERROR_MQTT_UNEXPECTED_HANGUP,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT_PROTOCOL_ERROR,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT_PROTOCOL_ERROR,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_FAILURE,
            .error_code = AWS_ERROR_MQTT_PROTOCOL_ERROR,
        },
    };
    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence_starts_with(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_connect_reconnect_failures, s_mqtt5to3_adapter_connect_reconnect_failures_fn)

/*
 * Connect successfully then disconnect followed by a connect with no intervening wait.  Verifies simple reliable
 * action and event sequencing.
 */
static int s_mqtt5to3_adapter_connect_success_disconnect_connect_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    aws_mqtt_client_connection_disconnect(
        adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 1);
    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 2);

    /*
     * depending on timing there may or may not be a closed event in between, so just check beginning and end for
     * expected events
     */

    struct aws_mqtt3_lifecycle_event expected_sequence_beginning[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
        {
            .type = AWS_MQTT3_LET_DISCONNECTION_COMPLETE,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence_starts_with(
        &fixture, AWS_ARRAY_SIZE(expected_sequence_beginning), expected_sequence_beginning));

    struct aws_mqtt3_lifecycle_event expected_sequence_ending[] = {
        {
            .type = AWS_MQTT3_LET_CONNECTION_SUCCESS,
        },
        {
            .type = AWS_MQTT3_LET_CONNECTION_COMPLETE,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence_ends_with(
        &fixture, AWS_ARRAY_SIZE(expected_sequence_ending), expected_sequence_ending));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_success_disconnect_connect,
    s_mqtt5to3_adapter_connect_success_disconnect_connect_fn)

/*
 * Calls disconnect() on an adapter that successfully connected but then had the mqtt5 client stopped behind the
 * adapter's back.  Verifies that we still get a completion callback.
 */
static int s_mqtt5to3_adapter_connect_success_stop_mqtt5_disconnect_success_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    aws_mqtt5_client_stop(fixture.mqtt5_fixture.client, NULL, NULL);

    aws_wait_for_stopped_lifecycle_event(&fixture.mqtt5_fixture);

    aws_mqtt_client_connection_disconnect(
        adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 1);

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_success_stop_mqtt5_disconnect_success,
    s_mqtt5to3_adapter_connect_success_stop_mqtt5_disconnect_success_fn)

/*
 * Call disconnect on a newly-created adapter.  Verifies that we get a completion callback.
 */
static int s_mqtt5to3_adapter_disconnect_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    aws_mqtt_client_connection_disconnect(
        adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 1);

    struct aws_mqtt3_lifecycle_event expected_events[] = {
        {
            .type = AWS_MQTT3_LET_DISCONNECTION_COMPLETE,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_lifecycle_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_disconnect_success, s_mqtt5to3_adapter_disconnect_success_fn)

/*
 * Use the adapter to successfully connect then call disconnect multiple times.  Verify that all disconnect
 * invocations generate expected lifecycle events.  Verifies that disconnects after a disconnect are properly handled.
 */
static int s_mqtt5to3_adapter_connect_success_disconnect_success_disconnect_success_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *adapter = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(adapter, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    aws_mqtt5_client_stop(fixture.mqtt5_fixture.client, NULL, NULL);

    aws_wait_for_stopped_lifecycle_event(&fixture.mqtt5_fixture);

    aws_mqtt_client_connection_disconnect(
        adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 1);

    aws_mqtt_client_connection_disconnect(
        adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    aws_mqtt_client_connection_disconnect(
        adapter, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 3);
    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CLOSED, 1);

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_connect_success_disconnect_success_disconnect_success,
    s_mqtt5to3_adapter_connect_success_disconnect_success_disconnect_success_fn)

#define SIMPLE_ALLOCATION_COUNT 10

static int s_mqtt5to3_adapter_operation_allocation_simple_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);
    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;
    struct aws_mqtt_client_connection_5_impl *adapter = connection->impl;
    struct aws_mqtt5_to_mqtt3_adapter_operation_table *operational_state = &adapter->operational_state;

    for (size_t i = 0; i < SIMPLE_ALLOCATION_COUNT; ++i) {
        struct aws_mqtt5_to_mqtt3_adapter_publish_options publish_options = {
            .adapter = adapter,
            .topic = aws_byte_cursor_from_c_str("derp"),
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        };

        struct aws_mqtt5_to_mqtt3_adapter_operation_publish *publish =
            aws_mqtt5_to_mqtt3_adapter_operation_new_publish(allocator, &publish_options);
        ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_operation_table_add_operation(operational_state, &publish->base));

        ASSERT_INT_EQUALS(i + 1, (size_t)(publish->base.id));
    }

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_operation_allocation_simple, s_mqtt5to3_adapter_operation_allocation_simple_fn)

#define ALLOCATION_WRAP_AROUND_ID_START 100

static int s_mqtt5to3_adapter_operation_allocation_wraparound_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);
    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;
    struct aws_mqtt_client_connection_5_impl *adapter = connection->impl;
    struct aws_mqtt5_to_mqtt3_adapter_operation_table *operational_state = &adapter->operational_state;

    operational_state->next_id = ALLOCATION_WRAP_AROUND_ID_START;

    for (size_t i = 0; i < UINT16_MAX + 50; ++i) {
        struct aws_mqtt5_to_mqtt3_adapter_publish_options publish_options = {
            .adapter = adapter,
            .topic = aws_byte_cursor_from_c_str("derp"),
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        };

        struct aws_mqtt5_to_mqtt3_adapter_operation_publish *publish =
            aws_mqtt5_to_mqtt3_adapter_operation_new_publish(allocator, &publish_options);
        ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_operation_table_add_operation(operational_state, &publish->base));

        size_t expected_id = (i + ALLOCATION_WRAP_AROUND_ID_START) % 65536;
        if (i > UINT16_MAX - ALLOCATION_WRAP_AROUND_ID_START) {
            ++expected_id;
        }

        ASSERT_INT_EQUALS(expected_id, (size_t)(publish->base.id));

        aws_mqtt5_to_mqtt3_adapter_operation_table_remove_operation(operational_state, publish->base.id);
    }

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_operation_allocation_wraparound, s_mqtt5to3_adapter_operation_allocation_wraparound_fn)

static int s_mqtt5to3_adapter_operation_allocation_exhaustion_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);
    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;
    struct aws_mqtt_client_connection_5_impl *adapter = connection->impl;
    struct aws_mqtt5_to_mqtt3_adapter_operation_table *operational_state = &adapter->operational_state;

    operational_state->next_id = ALLOCATION_WRAP_AROUND_ID_START;

    for (size_t i = 0; i < UINT16_MAX + 50; ++i) {
        struct aws_mqtt5_to_mqtt3_adapter_publish_options publish_options = {
            .adapter = adapter,
            .topic = aws_byte_cursor_from_c_str("derp"),
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        };

        struct aws_mqtt5_to_mqtt3_adapter_operation_publish *publish =
            aws_mqtt5_to_mqtt3_adapter_operation_new_publish(allocator, &publish_options);
        if (i >= UINT16_MAX) {
            ASSERT_FAILS(aws_mqtt5_to_mqtt3_adapter_operation_table_add_operation(operational_state, &publish->base));
            aws_mqtt5_to_mqtt3_adapter_operation_release(&publish->base);
            continue;
        }

        ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_operation_table_add_operation(operational_state, &publish->base));

        size_t expected_id = (i + ALLOCATION_WRAP_AROUND_ID_START) % 65536;
        if (i > UINT16_MAX - ALLOCATION_WRAP_AROUND_ID_START) {
            ++expected_id;
        }

        ASSERT_INT_EQUALS(expected_id, (size_t)(publish->base.id));
    }

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_operation_allocation_exhaustion, s_mqtt5to3_adapter_operation_allocation_exhaustion_fn)

static int s_aws_mqtt5_mock_server_handle_connect_succeed_with_small_payload(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    uint32_t maximum_packet_size = 1024;

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    connack_view.maximum_packet_size = &maximum_packet_size;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    struct aws_mqtt3_operation_event operation_event = {
        .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
        .error_code = error_code,
    };

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->operation_events, &operation_event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static int s_mqtt5to3_adapter_publish_failure_invalid_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* only allow small payloads to force a publish error */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_aws_mqtt5_mock_server_handle_connect_succeed_with_small_payload;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    uint8_t payload_array[2 * 1024];
    struct aws_byte_cursor payload = aws_byte_cursor_from_array(payload_array, AWS_ARRAY_SIZE(payload_array));

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {{
        .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
        .error_code = AWS_ERROR_MQTT5_PACKET_VALIDATION,
    }};
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_publish_failure_invalid, s_mqtt5to3_adapter_publish_failure_invalid_fn)

static int s_mqtt5to3_adapter_publish_failure_offline_queue_policy_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_QOS0_PUBLISH_ON_DISCONNECT;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    aws_mqtt_client_connection_disconnect(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        false,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {{
        .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
        .error_code = AWS_ERROR_MQTT5_OPERATION_FAILED_DUE_TO_OFFLINE_QUEUE_POLICY,
    }};
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_publish_failure_offline_queue_policy,
    s_mqtt5to3_adapter_publish_failure_offline_queue_policy_fn)

static int s_mqtt5to3_adapter_publish_success_qos0_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        false,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {{
        .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
        .error_code = AWS_ERROR_SUCCESS,
    }};
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_publish_success_qos0, s_mqtt5to3_adapter_publish_success_qos0_fn)

static int s_mqtt5to3_adapter_publish_success_qos1_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {{
        .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
        .error_code = AWS_ERROR_SUCCESS,
    }};
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_publish_success_qos1, s_mqtt5to3_adapter_publish_success_qos1_fn)

int aws_mqtt5_mock_server_handle_not_authorized_publish_puback(
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
        .reason_code = AWS_MQTT5_PARC_NOT_AUTHORIZED,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBACK, &puback_view);
}

static int s_mqtt5to3_adapter_publish_qos1_fail_ack_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* Return a fail qos1 puback */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_not_authorized_publish_puback;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {{
        .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
        .error_code = AWS_ERROR_MQTT_ACK_REASON_CODE_FAILURE,
    }};
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_publish_qos1_fail_ack, s_mqtt5to3_adapter_publish_qos1_fail_ack_fn)

static int s_mqtt5to3_adapter_publish_no_ack_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* Ignore publishes, triggering client-side timeout */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] = NULL;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;
    connection_options.protocol_operation_timeout_ms = 5000;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {{
        .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
        .error_code = AWS_ERROR_MQTT_TIMEOUT,
    }};
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_publish_no_ack, s_mqtt5to3_adapter_publish_no_ack_fn)

static int s_mqtt5to3_adapter_publish_interrupted_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    /* Ignore publishes */
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] = NULL;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    /*
     * wait for a little bit, we aren't going to get a response, shutdown while the operation is still pending
     * While we don't verify anything afterwards, consequent race conditions and leaks would show up.
     */
    aws_thread_current_sleep(aws_timestamp_convert(2, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_MILLIS, NULL));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_publish_interrupted, s_mqtt5to3_adapter_publish_interrupted_fn)

void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_byte_cursor *topic,
    enum aws_mqtt_qos qos,
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;
    (void)topic;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    struct aws_mqtt3_operation_event operation_event = {
        .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
        .error_code = error_code,
    };

    aws_array_list_init_dynamic(
        &operation_event.granted_subscriptions,
        fixture->mqtt5_fixture.allocator,
        1,
        sizeof(struct aws_mqtt_topic_subscription));

    aws_byte_buf_init_copy_from_cursor(&operation_event.topic_storage, fixture->mqtt5_fixture.allocator, *topic);

    /*
     * technically it's not safe to persist the topic cursor but they way the tests are built, the cursor will stay
     * valid until the events are checked (as long as we don't delete the subscription internally)
     */
    struct aws_mqtt_topic_subscription sub = {
        .topic = aws_byte_cursor_from_buf(&operation_event.topic_storage),
        .qos = qos,
    };
    aws_array_list_push_back(&operation_event.granted_subscriptions, (void *)&sub);

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->operation_events, &operation_event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_received(
    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture,
    enum aws_mqtt3_operation_event_type event_type,
    struct aws_byte_cursor topic,
    struct aws_byte_cursor payload,
    enum aws_mqtt_qos qos) {

    struct aws_mqtt3_operation_event operation_event = {
        .type = event_type,
        .qos = qos,
    };

    aws_byte_buf_init_copy_from_cursor(&operation_event.topic, fixture->mqtt5_fixture.allocator, topic);
    operation_event.topic_cursor = aws_byte_cursor_from_buf(&operation_event.topic);
    aws_byte_buf_init_copy_from_cursor(&operation_event.payload, fixture->mqtt5_fixture.allocator, payload);
    operation_event.payload_cursor = aws_byte_cursor_from_buf(&operation_event.payload);

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->operation_events, &operation_event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata) {

    (void)connection;
    (void)dup;
    (void)retain;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_received(
        fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY, *topic, *payload, qos);
}

void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata) {

    (void)connection;
    (void)dup;
    (void)retain;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;
    s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_received(
        fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED, *topic, *payload, qos);
}

static int s_mqtt5_mock_server_handle_subscribe_suback_success(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {

    (void)user_data;

    struct aws_mqtt5_packet_subscribe_view *subscribe_view = packet;

    AWS_VARIABLE_LENGTH_ARRAY(
        enum aws_mqtt5_suback_reason_code, mqtt5_suback_codes, subscribe_view->subscription_count);
    for (size_t i = 0; i < subscribe_view->subscription_count; ++i) {
        enum aws_mqtt5_suback_reason_code *reason_code_ptr = &mqtt5_suback_codes[i];
        *reason_code_ptr = (enum aws_mqtt5_suback_reason_code)subscribe_view->subscriptions[i].qos;
    }

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_view->packet_id,
        .reason_code_count = subscribe_view->subscription_count,
        .reason_codes = mqtt5_suback_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

static int s_mqtt5to3_adapter_subscribe_single_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    struct aws_mqtt_topic_subscription expected_subs[1] = {
        {
            .topic = topic,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        },
    };

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    aws_array_list_init_static_from_initialized(
        &expected_events[0].granted_subscriptions,
        (void *)expected_subs,
        1,
        sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_single_success, s_mqtt5to3_adapter_subscribe_single_success_fn)

/*
 * This function tests receiving a subscribe acknowledge after disconnecting from
 * the server.
 * it expects a AWS_MQTT_QOS_FAILURE return
 */
static int s_mqtt5to3_adapter_subscribe_single_null_suback_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    struct aws_mqtt_topic_subscription expected_subs[1] = {
        {
            .topic = topic,
            .qos = AWS_MQTT_QOS_FAILURE,
        },
    };

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
            .error_code = AWS_ERROR_MQTT5_USER_REQUESTED_STOP,
        },
    };
    aws_array_list_init_static_from_initialized(
        &expected_events[0].granted_subscriptions,
        (void *)expected_subs,
        1,
        sizeof(struct aws_mqtt_topic_subscription));

    aws_mqtt_client_connection_disconnect(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 1);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_single_null_suback, s_mqtt5to3_adapter_subscribe_single_null_suback_fn)

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_multi_complete(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    const struct aws_array_list *topic_subacks, /* contains aws_mqtt_topic_subscription pointers */
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    struct aws_mqtt3_operation_event operation_event = {
        .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
        .error_code = error_code,
    };

    if (error_code == AWS_ERROR_SUCCESS) {
        size_t granted_count = aws_array_list_length(topic_subacks);

        aws_array_list_init_dynamic(
            &operation_event.granted_subscriptions,
            fixture->mqtt5_fixture.allocator,
            granted_count,
            sizeof(struct aws_mqtt_topic_subscription));

        size_t topic_length = 0;
        for (size_t i = 0; i < granted_count; ++i) {
            struct aws_mqtt_topic_subscription *granted_sub = NULL;
            aws_array_list_get_at(topic_subacks, &granted_sub, i);

            aws_array_list_push_back(&operation_event.granted_subscriptions, (void *)granted_sub);
            topic_length += granted_sub->topic.len;
        }

        aws_byte_buf_init(&operation_event.topic_storage, fixture->mqtt5_fixture.allocator, topic_length);

        for (size_t i = 0; i < granted_count; ++i) {
            struct aws_mqtt_topic_subscription *granted_sub = NULL;
            aws_array_list_get_at_ptr(&operation_event.granted_subscriptions, (void **)&granted_sub, i);

            aws_byte_buf_append_and_update(&operation_event.topic_storage, &granted_sub->topic);
        }
    }

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->operation_events, &operation_event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static int s_mqtt5to3_adapter_subscribe_multi_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_mqtt_topic_subscription subscriptions[] = {
        {
            .topic = aws_byte_cursor_from_c_str("topic/1"),
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        },
        {
            .topic = aws_byte_cursor_from_c_str("topic/2"),
            .qos = AWS_MQTT_QOS_AT_MOST_ONCE,
        },
    };

    struct aws_array_list subscription_list;
    aws_array_list_init_static_from_initialized(
        &subscription_list, subscriptions, 2, sizeof(struct aws_mqtt_topic_subscription));

    aws_mqtt_client_connection_subscribe_multiple(
        connection,
        &subscription_list,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_multi_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    aws_array_list_init_static_from_initialized(
        &expected_events[0].granted_subscriptions,
        (void *)subscriptions,
        2,
        sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_multi_success, s_mqtt5to3_adapter_subscribe_multi_success_fn)

/*
 * This function tests receiving a subscribe acknowledge after disconnecting from
 * the server.
 * it expects a AWS_MQTT_QOS_FAILURE return
 */
static int s_mqtt5to3_adapter_subscribe_multi_null_suback_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_mqtt_topic_subscription subscriptions[] = {
        {
            .topic = aws_byte_cursor_from_c_str("topic/1"),
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        },
        {
            .topic = aws_byte_cursor_from_c_str("topic/2"),
            .qos = AWS_MQTT_QOS_AT_MOST_ONCE,
        },
    };

    struct aws_array_list subscription_list;
    aws_array_list_init_static_from_initialized(
        &subscription_list, subscriptions, 2, sizeof(struct aws_mqtt_topic_subscription));

    aws_mqtt_client_connection_subscribe_multiple(
        connection,
        &subscription_list,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_multi_complete,
        &fixture);

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
            .error_code = AWS_ERROR_MQTT5_USER_REQUESTED_STOP,
        },
    };

    aws_mqtt_client_connection_disconnect(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_disconnection_complete, &fixture);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_DISCONNECTION_COMPLETE, 1);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_multi_null_suback, s_mqtt5to3_adapter_subscribe_multi_null_suback_fn)

static int s_mqtt5_mock_server_handle_subscribe_suback_failure(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {

    (void)user_data;

    struct aws_mqtt5_packet_subscribe_view *subscribe_view = packet;

    AWS_VARIABLE_LENGTH_ARRAY(
        enum aws_mqtt5_suback_reason_code, mqtt5_suback_codes, subscribe_view->subscription_count);
    for (size_t i = 0; i < subscribe_view->subscription_count; ++i) {
        enum aws_mqtt5_suback_reason_code *reason_code_ptr = &mqtt5_suback_codes[i];
        *reason_code_ptr = (i % 2) ? (enum aws_mqtt5_suback_reason_code)subscribe_view->subscriptions[i].qos
                                   : AWS_MQTT5_SARC_QUOTA_EXCEEDED;
    }

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_view->packet_id,
        .reason_code_count = subscribe_view->subscription_count,
        .reason_codes = mqtt5_suback_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

static int s_mqtt5to3_adapter_subscribe_single_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_failure;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    struct aws_mqtt_topic_subscription expected_subs[1] = {
        {
            .topic = topic,
            .qos = AWS_MQTT_QOS_FAILURE,
        },
    };

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };

    aws_array_list_init_static_from_initialized(
        &expected_events[0].granted_subscriptions,
        (void *)expected_subs,
        1,
        sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_single_failure, s_mqtt5to3_adapter_subscribe_single_failure_fn)

static int s_mqtt5to3_adapter_subscribe_single_invalid_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor bad_topic = aws_byte_cursor_from_c_str("#/derp");

    ASSERT_INT_EQUALS(
        0,
        aws_mqtt_client_connection_subscribe(
            connection,
            &bad_topic,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
            &fixture,
            NULL,
            s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
            &fixture));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_single_invalid, s_mqtt5to3_adapter_subscribe_single_invalid_fn)

static int s_mqtt5to3_adapter_subscribe_multi_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_failure;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_mqtt_topic_subscription subscriptions[] = {
        {
            .topic = aws_byte_cursor_from_c_str("topic/1"),
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        },
        {
            .topic = aws_byte_cursor_from_c_str("topic/2"),
            .qos = AWS_MQTT_QOS_AT_MOST_ONCE,
        },
    };

    struct aws_array_list subscription_list;
    aws_array_list_init_static_from_initialized(
        &subscription_list, subscriptions, 2, sizeof(struct aws_mqtt_topic_subscription));

    aws_mqtt_client_connection_subscribe_multiple(
        connection,
        &subscription_list,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_multi_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    /* reuse the subscriptions array for validation, but the first one will fail */
    subscriptions[0].qos = AWS_MQTT_QOS_FAILURE;

    struct aws_mqtt3_operation_event expected_events[] = {{
        .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
        .error_code = AWS_ERROR_SUCCESS,
    }};
    aws_array_list_init_static_from_initialized(
        &expected_events[0].granted_subscriptions,
        (void *)subscriptions,
        2,
        sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events, AWS_ARRAY_SIZE(expected_events)));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_multi_failure, s_mqtt5to3_adapter_subscribe_multi_failure_fn)

static int s_mqtt5to3_adapter_subscribe_multi_invalid_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_failure;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_mqtt_topic_subscription subscriptions[] = {
        {
            .topic = aws_byte_cursor_from_c_str("topic/1"),
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        },
        {
            .topic = aws_byte_cursor_from_c_str("#/#"),
            .qos = AWS_MQTT_QOS_AT_MOST_ONCE,
        },
    };

    struct aws_array_list subscription_list;
    aws_array_list_init_static_from_initialized(
        &subscription_list, subscriptions, 2, sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_INT_EQUALS(
        0,
        aws_mqtt_client_connection_subscribe_multiple(
            connection,
            &subscription_list,
            s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_multi_complete,
            &fixture));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_multi_invalid, s_mqtt5to3_adapter_subscribe_multi_invalid_fn)

static int s_mqtt5to3_adapter_subscribe_single_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    aws_mqtt_client_connection_set_on_any_publish_handler(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish, &fixture);

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("derp");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Payload!");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY, 1);

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
            .error_code = AWS_ERROR_SUCCESS,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic,
            .payload_cursor = payload,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic,
            .payload_cursor = payload,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_subscribe_single_publish, s_mqtt5to3_adapter_subscribe_single_publish_fn)

static int s_mqtt5to3_adapter_subscribe_multi_overlapping_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    aws_mqtt_client_connection_set_on_any_publish_handler(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish, &fixture);

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("hello/world");
    struct aws_byte_cursor topic2 = aws_byte_cursor_from_c_str("hello/+");
    struct aws_byte_cursor topic3 = aws_byte_cursor_from_c_str("derp");

    ASSERT_TRUE(
        0 != aws_mqtt_client_connection_subscribe(
                 connection,
                 &topic1,
                 AWS_MQTT_QOS_AT_LEAST_ONCE,
                 s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
                 &fixture,
                 NULL,
                 s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
                 &fixture));

    ASSERT_TRUE(
        0 != aws_mqtt_client_connection_subscribe(
                 connection,
                 &topic2,
                 AWS_MQTT_QOS_AT_MOST_ONCE,
                 s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
                 &fixture,
                 NULL,
                 s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
                 &fixture));

    ASSERT_TRUE(
        0 != aws_mqtt_client_connection_subscribe(
                 connection,
                 &topic3,
                 AWS_MQTT_QOS_AT_LEAST_ONCE,
                 s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
                 &fixture,
                 NULL,
                 s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
                 &fixture));

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 3);

    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload 1!");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload 2!");

    aws_mqtt_client_connection_publish(
        connection,
        &topic1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload1,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);
    aws_mqtt_client_connection_publish(
        connection,
        &topic3,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload2,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 2);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY, 2);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED, 3);

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic1,
            .payload_cursor = payload1,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic3,
            .payload_cursor = payload2,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic1,
            .payload_cursor = payload1,
            .expected_count = 2,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic3,
            .payload_cursor = payload2,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_subscribe_multi_overlapping_publish,
    s_mqtt5to3_adapter_subscribe_multi_overlapping_publish_fn)

static void s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_unsubscribe_complete(
    struct aws_mqtt_client_connection *connection,
    uint16_t packet_id,
    int error_code,
    void *userdata) {

    (void)connection;
    (void)packet_id;

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture *fixture = userdata;

    struct aws_mqtt3_operation_event operation_event = {
        .type = AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE,
        .error_code = error_code,
    };

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->operation_events, &operation_event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static int s_mqtt5to3_adapter_unsubscribe_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    aws_mqtt_client_connection_set_on_any_publish_handler(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish, &fixture);

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("hello/world");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("Payload 1!");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED, 1);

    struct aws_mqtt3_operation_event expected_events_before[] = {
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic,
            .payload_cursor = payload,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic,
            .payload_cursor = payload,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(expected_events_before), expected_events_before));

    aws_mqtt_client_connection_unsubscribe(
        connection, &topic, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_unsubscribe_complete, &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE, 1);

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY, 2);

    struct aws_mqtt3_operation_event expected_events_after[] = {
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic,
            .payload_cursor = payload,
            .expected_count = 2,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic,
            .payload_cursor = payload,
        },
        {
            .type = AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(expected_events_after), expected_events_after));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_unsubscribe_success, s_mqtt5to3_adapter_unsubscribe_success_fn)

static int s_mqtt5_mock_server_handle_unsubscribe_unsuback_failure(
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
        *reason_code_ptr = AWS_MQTT5_UARC_IMPLEMENTATION_SPECIFIC_ERROR;
    }

    struct aws_mqtt5_packet_unsuback_view unsuback_view = {
        .packet_id = unsubscribe_view->packet_id,
        .reason_code_count = AWS_ARRAY_SIZE(mqtt5_unsuback_codes),
        .reason_codes = mqtt5_unsuback_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_UNSUBACK, &unsuback_view);
}

static int s_mqtt5to3_adapter_unsubscribe_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        s_mqtt5_mock_server_handle_unsubscribe_unsuback_failure;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("hello/world");

    aws_mqtt_client_connection_unsubscribe(
        connection, &topic, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_unsubscribe_complete, &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE, 1);

    struct aws_mqtt3_operation_event expected_events[] = {
        {
            .type = AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(expected_events), expected_events));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_unsubscribe_failure, s_mqtt5to3_adapter_unsubscribe_failure_fn)

static int s_mqtt5to3_adapter_unsubscribe_invalid_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        s_mqtt5_mock_server_handle_unsubscribe_unsuback_failure;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("#/bad");

    ASSERT_INT_EQUALS(
        0,
        aws_mqtt_client_connection_unsubscribe(
            connection, &topic, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_unsubscribe_complete, &fixture));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_unsubscribe_invalid, s_mqtt5to3_adapter_unsubscribe_invalid_fn)

static int s_mqtt5to3_adapter_unsubscribe_overlapped_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    aws_mqtt_client_connection_set_on_any_publish_handler(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish, &fixture);

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("hello/world");
    struct aws_byte_cursor topic2 = aws_byte_cursor_from_c_str("hello/+");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic2,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 2);

    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload 1!");

    aws_mqtt_client_connection_publish(
        connection,
        &topic1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload1,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED, 2);

    struct aws_mqtt3_operation_event expected_events_before[] = {
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic1,
            .payload_cursor = payload1,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic1,
            .payload_cursor = payload1,
            .expected_count = 2,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(expected_events_before), expected_events_before));

    /* drop the wildcard subscription and publish again, should only get one more publish received subscribed */
    aws_mqtt_client_connection_unsubscribe(
        connection, &topic2, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_unsubscribe_complete, &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE, 1);

    aws_mqtt_client_connection_publish(
        connection,
        &topic1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload1,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 2);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY, 2);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED, 3);

    struct aws_mqtt3_operation_event expected_events_after[] = {
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_ANY,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic1,
            .payload_cursor = payload1,
            .expected_count = 2,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_RECEIVED_SUBSCRIBED,
            .error_code = AWS_ERROR_SUCCESS,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
            .topic_cursor = topic1,
            .payload_cursor = payload1,
            .expected_count = 3,
        },
        {
            .type = AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE,
        },
    };

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(expected_events_after), expected_events_after));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_unsubscribe_overlapped, s_mqtt5to3_adapter_unsubscribe_overlapped_fn)

static int s_mqtt5_mock_server_handle_connect_publish_throttled(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_connack_view connack_view;
    AWS_ZERO_STRUCT(connack_view);

    uint16_t receive_maximum = 1;

    connack_view.reason_code = AWS_MQTT5_CRC_SUCCESS;
    connack_view.receive_maximum = &receive_maximum;

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_CONNACK, &connack_view);
}

/*
 * In this test, we configure the server to only allow a single unacked QoS1 publish and to not respond to
 * publishes.  Then we throw three QoS 1 publishes at the client.  This leads to the client being "paralyzed" waiting
 * for a PUBACK for the first publish.  We then query the client stats and expected to see one unacked operation and
 * two additional (for a total of three) incomplete operations.
 */
static int s_mqtt5to3_adapter_get_stats_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        s_mqtt5_mock_server_handle_connect_publish_throttled;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] = NULL;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    aws_mqtt_client_connection_set_on_any_publish_handler(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish, &fixture);

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("hi/there");
    struct aws_byte_cursor payload = aws_byte_cursor_from_c_str("something");

    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);
    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);
    aws_mqtt_client_connection_publish(
        connection,
        &topic,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    aws_thread_current_sleep(aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    struct aws_mqtt_connection_operation_statistics stats;
    aws_mqtt_client_connection_get_stats(connection, &stats);

    ASSERT_INT_EQUALS(1, stats.unacked_operation_count);
    ASSERT_INT_EQUALS(3, stats.incomplete_operation_count);
    ASSERT_TRUE(stats.unacked_operation_size > 0);
    ASSERT_TRUE(stats.incomplete_operation_size > 0);
    ASSERT_INT_EQUALS(stats.unacked_operation_size * 3, stats.incomplete_operation_size);

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_get_stats, s_mqtt5to3_adapter_get_stats_fn)

/*
 * In this test we invoke a resubscribe while there are no active subscriptions.  This hits a degenerate pathway
 * that we have to handle specially inside the adapter since an empty subscribe is invalid.
 */
static int s_mqtt5to3_adapter_resubscribe_nothing_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    aws_mqtt_client_connection_set_on_any_publish_handler(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish, &fixture);

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    aws_mqtt_resubscribe_existing_topics(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_multi_complete, &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);

    struct aws_mqtt3_operation_event resubscribe_ack[] = {{
        .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, .error_code = AWS_ERROR_MQTT_CONNECTION_RESUBSCRIBE_NO_TOPICS,
        /* no granted subscriptions */
    }};

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(resubscribe_ack), resubscribe_ack));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_resubscribe_nothing, s_mqtt5to3_adapter_resubscribe_nothing_fn)

/*
 * In this test we subscribe individually to three separate topics, wait, then invoke resubscribe on the client and
 * verify that we record 4 subacks, 3 for the individual and one 3-sized multi-sub for appropriate topics.
 */
static int s_mqtt5to3_adapter_resubscribe_something_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_mqtt5_mock_server_handle_subscribe_suback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    aws_mqtt_client_connection_set_on_any_publish_handler(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_on_any_publish, &fixture);

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("hello/world");
    struct aws_byte_cursor topic2 = aws_byte_cursor_from_c_str("foo/bar");
    struct aws_byte_cursor topic3 = aws_byte_cursor_from_c_str("a/b/c");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic2,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic3,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 3);

    aws_mqtt_resubscribe_existing_topics(
        connection, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_multi_complete, &fixture);

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 4);

    struct aws_mqtt3_operation_event resubscribe_ack[] = {{
        .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
    }};

    struct aws_mqtt_topic_subscription subscriptions[] = {
        {
            .topic = topic1,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        },
        {
            .topic = topic2,
            .qos = AWS_MQTT_QOS_AT_MOST_ONCE,
        },
        {
            .topic = topic3,
            .qos = AWS_MQTT_QOS_AT_LEAST_ONCE,
        },
    };

    aws_array_list_init_static_from_initialized(
        &resubscribe_ack[0].granted_subscriptions,
        (void *)subscriptions,
        AWS_ARRAY_SIZE(subscriptions),
        sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(resubscribe_ack), resubscribe_ack));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5to3_adapter_resubscribe_something, s_mqtt5to3_adapter_resubscribe_something_fn)

static int s_mqtt5to3_adapter_operation_callbacks_after_shutdown_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] = NULL;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] = NULL;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] = NULL;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_mqtt5_to_mqtt3_adapter_test_fixture fixture;
    ASSERT_SUCCESS(aws_mqtt5_to_mqtt3_adapter_test_fixture_init(&fixture, allocator, &test_fixture_options));

    struct aws_mqtt_client_connection *connection = fixture.connection;

    struct aws_mqtt_connection_options connection_options;
    s_init_adapter_connection_options_from_fixture(&connection_options, &fixture);

    connection_options.on_connection_complete = s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_connection_complete;
    connection_options.user_data = &fixture;

    aws_mqtt_client_connection_connect(connection, &connection_options);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_CONNECTION_COMPLETE, 1);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("hello/world");
    struct aws_byte_cursor topic2 = aws_byte_cursor_from_c_str("hello/+");

    aws_mqtt_client_connection_subscribe(
        connection,
        &topic1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_topic_specific_publish,
        &fixture,
        NULL,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_subscribe_complete,
        &fixture);

    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload 1!");

    aws_mqtt_client_connection_publish(
        connection,
        &topic1,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        false,
        &payload1,
        s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_publish_complete,
        &fixture);

    aws_mqtt_client_connection_unsubscribe(
        connection, &topic2, s_aws_mqtt5_to_mqtt3_adapter_test_fixture_record_unsubscribe_complete, &fixture);

    aws_mqtt_client_connection_release(connection);

    s_wait_for_n_adapter_lifecycle_events(&fixture, AWS_MQTT3_LET_TERMINATION, 1);
    fixture.connection = NULL;

    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_SUBSCRIBE_COMPLETE, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_PUBLISH_COMPLETE, 1);
    s_wait_for_n_adapter_operation_events(&fixture, AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE, 1);

    struct aws_mqtt3_operation_event failed_ops[] = {
        {
            .type = AWS_MQTT3_OET_SUBSCRIBE_COMPLETE,
            .error_code = AWS_ERROR_MQTT_CONNECTION_DESTROYED,
        },
        {
            .type = AWS_MQTT3_OET_PUBLISH_COMPLETE,
            .error_code = AWS_ERROR_MQTT_CONNECTION_DESTROYED,
        },
        {
            .type = AWS_MQTT3_OET_UNSUBSCRIBE_COMPLETE,
            .error_code = AWS_ERROR_MQTT_CONNECTION_DESTROYED,
        },
    };

    struct aws_mqtt_topic_subscription failed_subscriptions[] = {
        {
            .topic = topic1,
            .qos = AWS_MQTT_QOS_FAILURE,
        },
    };

    aws_array_list_init_static_from_initialized(
        &failed_ops[0].granted_subscriptions,
        (void *)failed_subscriptions,
        AWS_ARRAY_SIZE(failed_subscriptions),
        sizeof(struct aws_mqtt_topic_subscription));

    ASSERT_SUCCESS(s_aws_mqtt5_to_mqtt3_adapter_test_fixture_verify_operation_sequence_contains(
        &fixture, AWS_ARRAY_SIZE(failed_ops), failed_ops));

    aws_mqtt5_to_mqtt3_adapter_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5to3_adapter_operation_callbacks_after_shutdown,
    s_mqtt5to3_adapter_operation_callbacks_after_shutdown_fn)
