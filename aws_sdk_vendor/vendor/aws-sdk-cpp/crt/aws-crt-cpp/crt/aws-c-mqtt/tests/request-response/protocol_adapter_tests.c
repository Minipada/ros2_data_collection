/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "../v3/mqtt311_testing_utils.h"
#include "../v3/mqtt_mock_server_handler.h"
#include "../v5/mqtt5_testing_utils.h"
#include <aws/common/common.h>
#include <aws/mqtt/request-response/request_response_client.h>

#include "aws/mqtt/private/request-response/protocol_adapter.h"

#include <aws/testing/aws_test_harness.h>

struct request_response_protocol_adapter_incoming_publish_event_record {
    struct aws_byte_buf topic;
    struct aws_byte_buf payload;
};

static void s_request_response_protocol_adapter_incoming_publish_event_record_init(
    struct request_response_protocol_adapter_incoming_publish_event_record *record,
    struct aws_allocator *allocator,
    struct aws_byte_cursor topic,
    struct aws_byte_cursor payload) {

    aws_byte_buf_init_copy_from_cursor(&record->topic, allocator, topic);
    aws_byte_buf_init_copy_from_cursor(&record->payload, allocator, payload);
}

static void s_request_response_protocol_adapter_incoming_publish_event_record_clean_up(
    struct request_response_protocol_adapter_incoming_publish_event_record *record) {
    aws_byte_buf_clean_up(&record->topic);
    aws_byte_buf_clean_up(&record->payload);
}

struct request_response_protocol_adapter_subscription_event_record {
    enum aws_protocol_adapter_subscription_event_type event_type;
    struct aws_byte_buf topic_filter;
    int error_code;
    bool retryable;
};

static void s_request_response_protocol_adapter_subscription_event_record_init(
    struct request_response_protocol_adapter_subscription_event_record *record,
    struct aws_allocator *allocator,
    enum aws_protocol_adapter_subscription_event_type event_type,
    struct aws_byte_cursor topic_filter,
    int error_code,
    bool retryable) {

    AWS_ZERO_STRUCT(*record);

    record->event_type = event_type;
    record->error_code = error_code;
    record->retryable = retryable;
    aws_byte_buf_init_copy_from_cursor(&record->topic_filter, allocator, topic_filter);
}

static void s_request_response_protocol_adapter_subscription_event_record_cleanup(
    struct request_response_protocol_adapter_subscription_event_record *record) {
    aws_byte_buf_clean_up(&record->topic_filter);
}

enum aws_protocol_adapter_protocol_type {
    AWS_PAPT_MQTT5,
    AWS_PAPT_MQTT311,
};

struct mqtt311_protocol_testing_context {
    struct mqtt_connection_state_test mqtt311_test_context;
    int mqtt311_test_context_setup_result;
};

struct aws_request_response_protocol_adapter_test_fixture {
    struct aws_allocator *allocator;

    enum aws_protocol_adapter_protocol_type protocol_type;

    union {
        struct aws_mqtt5_client_mock_test_fixture mqtt5_fixture;
        struct mqtt311_protocol_testing_context mqtt311_fixture;
    } protocol_context;

    struct aws_mqtt_protocol_adapter *protocol_adapter;

    struct aws_array_list incoming_publish_events;
    struct aws_array_list connection_events;
    struct aws_array_list subscription_events;
    struct aws_array_list publish_results;

    bool adapter_terminated;

    struct aws_mutex lock;
    struct aws_condition_variable signal;
};

static void s_rr_mqtt_protocol_adapter_test_on_subscription_event(
    const struct aws_protocol_adapter_subscription_event *event,
    void *user_data) {
    struct aws_request_response_protocol_adapter_test_fixture *fixture = user_data;

    struct request_response_protocol_adapter_subscription_event_record record;
    s_request_response_protocol_adapter_subscription_event_record_init(
        &record, fixture->allocator, event->event_type, event->topic_filter, event->error_code, event->retryable);

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->subscription_events, &record);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_rr_mqtt_protocol_adapter_test_on_incoming_publish(
    const struct aws_mqtt_rr_incoming_publish_event *publish,
    void *user_data) {
    struct aws_request_response_protocol_adapter_test_fixture *fixture = user_data;

    struct request_response_protocol_adapter_incoming_publish_event_record record;
    AWS_ZERO_STRUCT(record);
    s_request_response_protocol_adapter_incoming_publish_event_record_init(
        &record, fixture->allocator, publish->topic, publish->payload);

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->incoming_publish_events, &record);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_rr_mqtt_protocol_adapter_test_on_terminate_callback(void *user_data) {
    struct aws_request_response_protocol_adapter_test_fixture *fixture = user_data;

    aws_mutex_lock(&fixture->lock);
    fixture->adapter_terminated = true;
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_rr_mqtt_protocol_adapter_test_on_connection_event(
    const struct aws_protocol_adapter_connection_event *event,
    void *user_data) {
    struct aws_request_response_protocol_adapter_test_fixture *fixture = user_data;

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->connection_events, event);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_rr_mqtt_protocol_adapter_test_on_publish_result(int error_code, void *user_data) {
    struct aws_request_response_protocol_adapter_test_fixture *fixture = user_data;

    aws_mutex_lock(&fixture->lock);
    aws_array_list_push_back(&fixture->publish_results, &error_code);
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static int s_aws_request_response_protocol_adapter_test_fixture_init_shared(
    struct aws_request_response_protocol_adapter_test_fixture *fixture,
    enum aws_protocol_adapter_protocol_type protocol_type) {

    struct aws_allocator *allocator = fixture->allocator;
    fixture->protocol_type = protocol_type;

    struct aws_mqtt_protocol_adapter_options protocol_adapter_options = {
        .subscription_event_callback = s_rr_mqtt_protocol_adapter_test_on_subscription_event,
        .incoming_publish_callback = s_rr_mqtt_protocol_adapter_test_on_incoming_publish,
        .terminate_callback = s_rr_mqtt_protocol_adapter_test_on_terminate_callback,
        .connection_event_callback = s_rr_mqtt_protocol_adapter_test_on_connection_event,
        .user_data = fixture};

    if (protocol_type == AWS_PAPT_MQTT5) {
        fixture->protocol_adapter = aws_mqtt_protocol_adapter_new_from_5(
            allocator, &protocol_adapter_options, fixture->protocol_context.mqtt5_fixture.client);
    } else {
        fixture->protocol_adapter = aws_mqtt_protocol_adapter_new_from_311(
            allocator,
            &protocol_adapter_options,
            fixture->protocol_context.mqtt311_fixture.mqtt311_test_context.mqtt_connection);
    }
    AWS_FATAL_ASSERT(fixture->protocol_adapter != NULL);

    aws_array_list_init_dynamic(
        &fixture->incoming_publish_events,
        allocator,
        10,
        sizeof(struct request_response_protocol_adapter_incoming_publish_event_record));
    aws_array_list_init_dynamic(
        &fixture->connection_events, allocator, 10, sizeof(struct aws_protocol_adapter_connection_event));
    aws_array_list_init_dynamic(
        &fixture->subscription_events,
        allocator,
        10,
        sizeof(struct request_response_protocol_adapter_subscription_event_record));
    aws_array_list_init_dynamic(&fixture->publish_results, allocator, 10, sizeof(int));

    aws_mutex_init(&fixture->lock);
    aws_condition_variable_init(&fixture->signal);

    return AWS_OP_SUCCESS;
}

static int s_aws_request_response_protocol_adapter_test_fixture_init_mqtt5(
    struct aws_request_response_protocol_adapter_test_fixture *fixture,
    struct aws_allocator *allocator,
    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *mqtt5_fixture_config) {

    AWS_ZERO_STRUCT(*fixture);

    fixture->allocator = allocator;

    if (aws_mqtt5_client_mock_test_fixture_init(
            &fixture->protocol_context.mqtt5_fixture, allocator, mqtt5_fixture_config)) {
        return AWS_OP_ERR;
    }

    return s_aws_request_response_protocol_adapter_test_fixture_init_shared(fixture, AWS_PAPT_MQTT5);
}

static int s_aws_request_response_protocol_adapter_test_fixture_init_mqtt311(
    struct aws_request_response_protocol_adapter_test_fixture *fixture,
    struct aws_allocator *allocator) {

    AWS_ZERO_STRUCT(*fixture);

    fixture->allocator = allocator;

    int result =
        aws_test311_setup_mqtt_server_fn(allocator, &fixture->protocol_context.mqtt311_fixture.mqtt311_test_context);
    ASSERT_SUCCESS(result);
    fixture->protocol_context.mqtt311_fixture.mqtt311_test_context_setup_result = result;

    return s_aws_request_response_protocol_adapter_test_fixture_init_shared(fixture, AWS_PAPT_MQTT311);
}

static bool s_is_adapter_terminated(void *context) {
    struct aws_request_response_protocol_adapter_test_fixture *fixture = context;

    return fixture->adapter_terminated;
}

static void s_aws_request_response_protocol_adapter_test_fixture_destroy_adapters(
    struct aws_request_response_protocol_adapter_test_fixture *fixture) {
    if (fixture->protocol_adapter != NULL) {
        aws_mqtt_protocol_adapter_destroy(fixture->protocol_adapter);

        aws_mutex_lock(&fixture->lock);
        aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_is_adapter_terminated, fixture);
        aws_mutex_unlock(&fixture->lock);
        fixture->protocol_adapter = NULL;
    }
}

static void s_aws_request_response_protocol_adapter_test_fixture_clean_up(
    struct aws_request_response_protocol_adapter_test_fixture *fixture) {

    s_aws_request_response_protocol_adapter_test_fixture_destroy_adapters(fixture);

    if (fixture->protocol_type == AWS_PAPT_MQTT5) {
        aws_mqtt5_client_mock_test_fixture_clean_up(&fixture->protocol_context.mqtt5_fixture);
    } else {
        aws_test311_clean_up_mqtt_server_fn(
            fixture->allocator,
            fixture->protocol_context.mqtt311_fixture.mqtt311_test_context_setup_result,
            &fixture->protocol_context.mqtt311_fixture.mqtt311_test_context);
    }

    for (size_t i = 0; i < aws_array_list_length(&fixture->subscription_events); ++i) {
        struct request_response_protocol_adapter_subscription_event_record record;
        aws_array_list_get_at(&fixture->subscription_events, &record, i);
        s_request_response_protocol_adapter_subscription_event_record_cleanup(&record);
    }
    aws_array_list_clean_up(&fixture->subscription_events);

    for (size_t i = 0; i < aws_array_list_length(&fixture->incoming_publish_events); ++i) {
        struct request_response_protocol_adapter_incoming_publish_event_record record;
        aws_array_list_get_at(&fixture->incoming_publish_events, &record, i);
        s_request_response_protocol_adapter_incoming_publish_event_record_clean_up(&record);
    }
    aws_array_list_clean_up(&fixture->incoming_publish_events);

    aws_array_list_clean_up(&fixture->connection_events);
    aws_array_list_clean_up(&fixture->publish_results);

    aws_mutex_clean_up(&fixture->lock);
    aws_condition_variable_clean_up(&fixture->signal);
}

struct test_subscription_event_wait_context {
    struct request_response_protocol_adapter_subscription_event_record *expected_event;
    size_t expected_count;
    struct aws_request_response_protocol_adapter_test_fixture *fixture;
};

static bool s_do_subscription_events_contain(void *context) {
    struct test_subscription_event_wait_context *wait_context = context;

    size_t found = 0;

    size_t num_events = aws_array_list_length(&wait_context->fixture->subscription_events);
    for (size_t i = 0; i < num_events; ++i) {
        struct request_response_protocol_adapter_subscription_event_record record;
        aws_array_list_get_at(&wait_context->fixture->subscription_events, &record, i);

        if (record.event_type != wait_context->expected_event->event_type) {
            continue;
        }

        if (record.error_code != wait_context->expected_event->error_code) {
            continue;
        }

        struct aws_byte_cursor record_topic_filter = aws_byte_cursor_from_buf(&record.topic_filter);
        struct aws_byte_cursor expected_topic_filter =
            aws_byte_cursor_from_buf(&wait_context->expected_event->topic_filter);
        if (!aws_byte_cursor_eq(&record_topic_filter, &expected_topic_filter)) {
            continue;
        }

        if (record.retryable != wait_context->expected_event->retryable) {
            continue;
        }

        ++found;
    }

    return found >= wait_context->expected_count;
}

static void s_wait_for_subscription_events_contains(
    struct aws_request_response_protocol_adapter_test_fixture *fixture,
    struct request_response_protocol_adapter_subscription_event_record *expected_event,
    size_t expected_count) {

    struct test_subscription_event_wait_context context = {
        .expected_event = expected_event,
        .expected_count = expected_count,
        .fixture = fixture,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_do_subscription_events_contain, &context);
    aws_mutex_unlock(&fixture->lock);
}

struct test_connection_event_wait_context {
    struct aws_protocol_adapter_connection_event *expected_event;
    size_t expected_count;
    struct aws_request_response_protocol_adapter_test_fixture *fixture;
};

static bool s_do_connection_events_contain(void *context) {
    struct test_connection_event_wait_context *wait_context = context;

    size_t found = 0;

    size_t num_events = aws_array_list_length(&wait_context->fixture->connection_events);
    for (size_t i = 0; i < num_events; ++i) {
        struct aws_protocol_adapter_connection_event record;
        aws_array_list_get_at(&wait_context->fixture->connection_events, &record, i);

        if (record.event_type != wait_context->expected_event->event_type) {
            continue;
        }

        if (record.joined_session != wait_context->expected_event->joined_session) {
            continue;
        }

        ++found;
    }

    return found >= wait_context->expected_count;
}

static void s_wait_for_connection_events_contains(
    struct aws_request_response_protocol_adapter_test_fixture *fixture,
    struct aws_protocol_adapter_connection_event *expected_event,
    size_t expected_count) {

    struct test_connection_event_wait_context context = {
        .expected_event = expected_event,
        .expected_count = expected_count,
        .fixture = fixture,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_do_connection_events_contain, &context);
    aws_mutex_unlock(&fixture->lock);
}

struct test_incoming_publish_event_wait_context {
    struct request_response_protocol_adapter_incoming_publish_event_record *expected_event;
    size_t expected_count;
    struct aws_request_response_protocol_adapter_test_fixture *fixture;
};

static bool s_do_incoming_publish_events_contain(void *context) {
    struct test_incoming_publish_event_wait_context *wait_context = context;

    size_t found = 0;

    size_t num_events = aws_array_list_length(&wait_context->fixture->incoming_publish_events);
    for (size_t i = 0; i < num_events; ++i) {
        struct request_response_protocol_adapter_incoming_publish_event_record record;
        aws_array_list_get_at(&wait_context->fixture->incoming_publish_events, &record, i);

        struct aws_byte_cursor record_topic = aws_byte_cursor_from_buf(&record.topic);
        struct aws_byte_cursor expected_topic = aws_byte_cursor_from_buf(&wait_context->expected_event->topic);
        if (!aws_byte_cursor_eq(&record_topic, &expected_topic)) {
            continue;
        }

        struct aws_byte_cursor record_payload = aws_byte_cursor_from_buf(&record.payload);
        struct aws_byte_cursor expected_payload = aws_byte_cursor_from_buf(&wait_context->expected_event->payload);
        if (!aws_byte_cursor_eq(&record_payload, &expected_payload)) {
            continue;
        }

        ++found;
    }

    return found >= wait_context->expected_count;
}

static void s_wait_for_incoming_publish_events_contains(
    struct aws_request_response_protocol_adapter_test_fixture *fixture,
    struct request_response_protocol_adapter_incoming_publish_event_record *expected_event,
    size_t expected_count) {

    struct test_incoming_publish_event_wait_context context = {
        .expected_event = expected_event,
        .expected_count = expected_count,
        .fixture = fixture,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_do_incoming_publish_events_contain, &context);
    aws_mutex_unlock(&fixture->lock);
}

struct test_publish_result_wait_context {
    int expected_error_code;
    size_t expected_count;
    struct aws_request_response_protocol_adapter_test_fixture *fixture;
};

static bool s_do_publish_results_contain(void *context) {
    struct test_publish_result_wait_context *wait_context = context;

    size_t found = 0;

    size_t num_events = aws_array_list_length(&wait_context->fixture->publish_results);
    for (size_t i = 0; i < num_events; ++i) {
        int error_code = AWS_ERROR_SUCCESS;
        aws_array_list_get_at(&wait_context->fixture->publish_results, &error_code, i);

        if (error_code == wait_context->expected_error_code) {
            ++found;
        }
    }

    return found >= wait_context->expected_count;
}

static void s_wait_for_publish_results_contains(
    struct aws_request_response_protocol_adapter_test_fixture *fixture,
    int expected_error_code,
    size_t expected_count) {

    struct test_publish_result_wait_context context = {
        .expected_error_code = expected_error_code,
        .expected_count = expected_count,
        .fixture = fixture,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_do_publish_results_contain, &context);
    aws_mutex_unlock(&fixture->lock);
}

enum protocol_adapter_operation_test_type {
    PAOTT_SUCCESS,
    PAOTT_FAILURE_TIMEOUT,
    PAOTT_FAILURE_REASON_CODE_RETRYABLE,
    PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE,
    PAOTT_FAILURE_ERROR_CODE,
};

static enum aws_mqtt5_suback_reason_code s_failed_suback_reason_codes[] = {
    AWS_MQTT5_SARC_NOT_AUTHORIZED,
};

static int s_aws_mqtt5_server_send_failed_suback_on_subscribe(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_subscribe_view *subscribe_view = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_view->packet_id,
        .reason_code_count = AWS_ARRAY_SIZE(s_failed_suback_reason_codes),
        .reason_codes = s_failed_suback_reason_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

static int s_test_type_to_expected_error_code(
    enum protocol_adapter_operation_test_type test_type,
    enum aws_protocol_adapter_protocol_type protocol_type) {
    if (protocol_type == AWS_PAPT_MQTT5) {
        switch (test_type) {
            case PAOTT_FAILURE_TIMEOUT:
                return AWS_ERROR_MQTT_TIMEOUT;
            case PAOTT_FAILURE_REASON_CODE_RETRYABLE:
            case PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE:
                return AWS_ERROR_MQTT_PROTOCOL_ADAPTER_FAILING_REASON_CODE;
            case PAOTT_FAILURE_ERROR_CODE:
                return AWS_ERROR_MQTT5_OPERATION_FAILED_DUE_TO_OFFLINE_QUEUE_POLICY;
            default:
                return AWS_ERROR_SUCCESS;
        }
    } else {
        switch (test_type) {
            case PAOTT_FAILURE_TIMEOUT:
                return AWS_ERROR_MQTT_TIMEOUT;
            case PAOTT_FAILURE_REASON_CODE_RETRYABLE:
            case PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE:
                return AWS_ERROR_MQTT_PROTOCOL_ADAPTER_FAILING_REASON_CODE;
            case PAOTT_FAILURE_ERROR_CODE:
                return AWS_ERROR_MQTT5_OPERATION_FAILED_DUE_TO_OFFLINE_QUEUE_POLICY;
            default:
                return AWS_ERROR_SUCCESS;
        }
    }
}

static int s_do_request_response_mqtt5_protocol_adapter_subscribe_test(
    struct aws_allocator *allocator,
    enum protocol_adapter_operation_test_type test_type) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    if (test_type == PAOTT_SUCCESS) {
        test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
            aws_mqtt5_server_send_suback_on_subscribe;
    } else if (
        test_type == PAOTT_FAILURE_REASON_CODE_RETRYABLE || test_type == PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE) {
        test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
            s_aws_mqtt5_server_send_failed_suback_on_subscribe;
    }

    if (test_type == PAOTT_FAILURE_ERROR_CODE) {
        test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_ALL_ON_DISCONNECT;
    }

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options mqtt5_test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt5(
        &fixture, allocator, &mqtt5_test_fixture_options));

    struct aws_mqtt5_client *client = fixture.protocol_context.mqtt5_fixture.client;

    if (test_type != PAOTT_FAILURE_ERROR_CODE) {
        ASSERT_SUCCESS(aws_mqtt5_client_start(client));

        aws_wait_for_connected_lifecycle_event(&fixture.protocol_context.mqtt5_fixture);
    }

    int expected_error_code = s_test_type_to_expected_error_code(test_type, AWS_PAPT_MQTT5);

    struct request_response_protocol_adapter_subscription_event_record expected_outcome;
    s_request_response_protocol_adapter_subscription_event_record_init(
        &expected_outcome,
        allocator,
        AWS_PASET_SUBSCRIBE,
        aws_byte_cursor_from_c_str("hello/world"),
        expected_error_code,
        test_type != PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE);

    struct aws_protocol_adapter_subscribe_options subscribe_options = {
        .topic_filter = aws_byte_cursor_from_buf(&expected_outcome.topic_filter),
        .ack_timeout_seconds = 2,
    };

    aws_mqtt_protocol_adapter_subscribe(fixture.protocol_adapter, &subscribe_options);

    s_wait_for_subscription_events_contains(&fixture, &expected_outcome, 1);

    s_request_response_protocol_adapter_subscription_event_record_cleanup(&expected_outcome);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_request_response_mqtt5_protocol_adapter_subscribe_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_subscribe_test(allocator, PAOTT_SUCCESS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_subscribe_success,
    s_request_response_mqtt5_protocol_adapter_subscribe_success_fn)

static int s_request_response_mqtt5_protocol_adapter_subscribe_failure_timeout_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_subscribe_test(allocator, PAOTT_FAILURE_TIMEOUT));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_subscribe_failure_timeout,
    s_request_response_mqtt5_protocol_adapter_subscribe_failure_timeout_fn)

static int s_request_response_mqtt5_protocol_adapter_subscribe_failure_reason_code_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_subscribe_test(
        allocator, PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_subscribe_failure_reason_code,
    s_request_response_mqtt5_protocol_adapter_subscribe_failure_reason_code_fn)

static int s_request_response_mqtt5_protocol_adapter_subscribe_failure_error_code_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_subscribe_test(allocator, PAOTT_FAILURE_ERROR_CODE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_subscribe_failure_error_code,
    s_request_response_mqtt5_protocol_adapter_subscribe_failure_error_code_fn)

static enum aws_mqtt5_unsuback_reason_code s_retryable_failed_unsuback_reason_codes[] = {
    AWS_MQTT5_UARC_IMPLEMENTATION_SPECIFIC_ERROR,
};

static int s_aws_mqtt5_server_send_retryable_failed_unsuback_on_unsubscribe(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_subscribe_view *unsubscribe_view = packet;

    struct aws_mqtt5_packet_unsuback_view unsuback_view = {
        .packet_id = unsubscribe_view->packet_id,
        .reason_code_count = AWS_ARRAY_SIZE(s_retryable_failed_unsuback_reason_codes),
        .reason_codes = s_retryable_failed_unsuback_reason_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_UNSUBACK, &unsuback_view);
}

static enum aws_mqtt5_unsuback_reason_code s_not_retryable_failed_unsuback_reason_codes[] = {
    AWS_MQTT5_UARC_NOT_AUTHORIZED,
};

static int s_aws_mqtt5_server_send_not_retryable_failed_unsuback_on_unsubscribe(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_subscribe_view *unsubscribe_view = packet;

    struct aws_mqtt5_packet_unsuback_view unsuback_view = {
        .packet_id = unsubscribe_view->packet_id,
        .reason_code_count = AWS_ARRAY_SIZE(s_not_retryable_failed_unsuback_reason_codes),
        .reason_codes = s_not_retryable_failed_unsuback_reason_codes,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_UNSUBACK, &unsuback_view);
}

static int s_do_request_response_mqtt5_protocol_adapter_unsubscribe_test(
    struct aws_allocator *allocator,
    enum protocol_adapter_operation_test_type test_type) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    if (test_type == PAOTT_SUCCESS) {
        test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
            aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success;
    } else if (test_type == PAOTT_FAILURE_REASON_CODE_RETRYABLE) {
        test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
            s_aws_mqtt5_server_send_retryable_failed_unsuback_on_unsubscribe;
    } else if (test_type == PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE) {
        test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
            s_aws_mqtt5_server_send_not_retryable_failed_unsuback_on_unsubscribe;
    }

    if (test_type == PAOTT_FAILURE_ERROR_CODE) {
        test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_ALL_ON_DISCONNECT;
    }

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options mqtt5_test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt5(
        &fixture, allocator, &mqtt5_test_fixture_options));

    struct aws_mqtt5_client *client = fixture.protocol_context.mqtt5_fixture.client;

    if (test_type != PAOTT_FAILURE_ERROR_CODE) {
        ASSERT_SUCCESS(aws_mqtt5_client_start(client));

        aws_wait_for_connected_lifecycle_event(&fixture.protocol_context.mqtt5_fixture);
    }

    int expected_error_code = s_test_type_to_expected_error_code(test_type, AWS_PAPT_MQTT5);

    struct request_response_protocol_adapter_subscription_event_record expected_outcome;
    s_request_response_protocol_adapter_subscription_event_record_init(
        &expected_outcome,
        allocator,
        AWS_PASET_UNSUBSCRIBE,
        aws_byte_cursor_from_c_str("hello/world"),
        expected_error_code,
        test_type == PAOTT_FAILURE_REASON_CODE_RETRYABLE || test_type == PAOTT_FAILURE_TIMEOUT);

    struct aws_protocol_adapter_unsubscribe_options unsubscribe_options = {
        .topic_filter = aws_byte_cursor_from_buf(&expected_outcome.topic_filter),
        .ack_timeout_seconds = 2,
    };

    aws_mqtt_protocol_adapter_unsubscribe(fixture.protocol_adapter, &unsubscribe_options);

    s_wait_for_subscription_events_contains(&fixture, &expected_outcome, 1);

    s_request_response_protocol_adapter_subscription_event_record_cleanup(&expected_outcome);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_request_response_mqtt5_protocol_adapter_unsubscribe_success_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_unsubscribe_test(allocator, PAOTT_SUCCESS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_unsubscribe_success,
    s_request_response_mqtt5_protocol_adapter_unsubscribe_success_fn)

static int s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_timeout_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_unsubscribe_test(allocator, PAOTT_FAILURE_TIMEOUT));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_unsubscribe_failure_timeout,
    s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_timeout_fn)

static int s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_reason_code_retryable_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(
        s_do_request_response_mqtt5_protocol_adapter_unsubscribe_test(allocator, PAOTT_FAILURE_REASON_CODE_RETRYABLE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_unsubscribe_failure_reason_code_retryable,
    s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_reason_code_retryable_fn)

static int s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_reason_code_not_retryable_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_unsubscribe_test(
        allocator, PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_unsubscribe_failure_reason_code_not_retryable,
    s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_reason_code_not_retryable_fn)

static int s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_error_code_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_unsubscribe_test(allocator, PAOTT_FAILURE_ERROR_CODE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_unsubscribe_failure_error_code,
    s_request_response_mqtt5_protocol_adapter_unsubscribe_failure_error_code_fn)

static int s_aws_mqtt5_server_send_failed_puback_on_publish(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;
    (void)user_data;

    struct aws_mqtt5_packet_publish_view *publish_view = packet;

    if (publish_view->qos == AWS_MQTT5_QOS_AT_LEAST_ONCE) {
        struct aws_mqtt5_packet_puback_view puback_view = {
            .packet_id = publish_view->packet_id,
            .reason_code = AWS_MQTT5_PARC_NOT_AUTHORIZED,
        };

        return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBACK, &puback_view);
    }

    return AWS_OP_SUCCESS;
}

static int s_do_request_response_mqtt5_protocol_adapter_publish_test(
    struct aws_allocator *allocator,
    enum protocol_adapter_operation_test_type test_type) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    if (test_type == PAOTT_SUCCESS) {
        test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
            aws_mqtt5_mock_server_handle_publish_puback;
    } else if (
        test_type == PAOTT_FAILURE_REASON_CODE_RETRYABLE || test_type == PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE) {
        test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
            s_aws_mqtt5_server_send_failed_puback_on_publish;
    }

    if (test_type == PAOTT_FAILURE_ERROR_CODE) {
        test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_ALL_ON_DISCONNECT;
    }

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options mqtt5_test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt5(
        &fixture, allocator, &mqtt5_test_fixture_options));

    struct aws_mqtt5_client *client = fixture.protocol_context.mqtt5_fixture.client;

    if (test_type != PAOTT_FAILURE_ERROR_CODE) {
        ASSERT_SUCCESS(aws_mqtt5_client_start(client));

        aws_wait_for_connected_lifecycle_event(&fixture.protocol_context.mqtt5_fixture);
    }

    struct aws_protocol_adapter_publish_options publish_options = {
        .topic = aws_byte_cursor_from_c_str("hello/world"),
        .payload = aws_byte_cursor_from_c_str("SomePayload"),
        .ack_timeout_seconds = 2,
        .completion_callback_fn = s_rr_mqtt_protocol_adapter_test_on_publish_result,
        .user_data = &fixture,
    };

    aws_mqtt_protocol_adapter_publish(fixture.protocol_adapter, &publish_options);

    int expected_error_code = s_test_type_to_expected_error_code(test_type, AWS_PAPT_MQTT5);
    s_wait_for_publish_results_contains(&fixture, expected_error_code, 1);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_request_response_mqtt5_protocol_adapter_publish_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_publish_test(allocator, PAOTT_SUCCESS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_publish_success,
    s_request_response_mqtt5_protocol_adapter_publish_success_fn)

static int s_request_response_mqtt5_protocol_adapter_publish_failure_timeout_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_publish_test(allocator, PAOTT_FAILURE_TIMEOUT));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_publish_failure_timeout,
    s_request_response_mqtt5_protocol_adapter_publish_failure_timeout_fn)

static int s_request_response_mqtt5_protocol_adapter_publish_failure_reason_code_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(
        s_do_request_response_mqtt5_protocol_adapter_publish_test(allocator, PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_publish_failure_reason_code,
    s_request_response_mqtt5_protocol_adapter_publish_failure_reason_code_fn)

static int s_request_response_mqtt5_protocol_adapter_publish_failure_error_code_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt5_protocol_adapter_publish_test(allocator, PAOTT_FAILURE_ERROR_CODE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_publish_failure_error_code,
    s_request_response_mqtt5_protocol_adapter_publish_failure_error_code_fn)

static int s_do_request_response_mqtt5_protocol_adapter_connection_event_connect_test(
    struct aws_allocator *allocator,
    bool rejoin_session) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_honor_session_unconditional;
    test_options.client_options.session_behavior = rejoin_session ? AWS_MQTT5_CSBT_REJOIN_ALWAYS : AWS_MQTT5_CSBT_CLEAN;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options mqtt5_test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt5(
        &fixture, allocator, &mqtt5_test_fixture_options));

    struct aws_mqtt5_client *client = fixture.protocol_context.mqtt5_fixture.client;

    struct aws_protocol_adapter_connection_event expected_connect_record = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = rejoin_session,
    };

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));
    s_wait_for_connection_events_contains(&fixture, &expected_connect_record, 1);

    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));
    aws_wait_for_stopped_lifecycle_event(&fixture.protocol_context.mqtt5_fixture);

    struct aws_protocol_adapter_connection_event expected_disconnect_record = {
        .event_type = AWS_PACET_DISCONNECTED,
    };

    s_wait_for_connection_events_contains(&fixture, &expected_disconnect_record, 1);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_request_response_mqtt5_protocol_adapter_connection_event_connect_no_session_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_request_response_mqtt5_protocol_adapter_connection_event_connect_test(allocator, false);
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_connection_event_connect_no_session,
    s_request_response_mqtt5_protocol_adapter_connection_event_connect_no_session_fn)

static int s_request_response_mqtt5_protocol_adapter_connection_event_connect_session_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_request_response_mqtt5_protocol_adapter_connection_event_connect_test(allocator, true);
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_connection_event_connect_session,
    s_request_response_mqtt5_protocol_adapter_connection_event_connect_session_fn)

static int s_request_response_mqtt5_protocol_adapter_incoming_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_puback_and_forward;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options mqtt5_test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt5(
        &fixture, allocator, &mqtt5_test_fixture_options));

    struct aws_mqtt5_client *client = fixture.protocol_context.mqtt5_fixture.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&fixture.protocol_context.mqtt5_fixture);

    struct request_response_protocol_adapter_incoming_publish_event_record expected_publish;
    AWS_ZERO_STRUCT(expected_publish);

    s_request_response_protocol_adapter_incoming_publish_event_record_init(
        &expected_publish,
        allocator,
        aws_byte_cursor_from_c_str("hello/world"),
        aws_byte_cursor_from_c_str("SomePayload"));

    struct aws_protocol_adapter_publish_options publish_options = {
        .topic = aws_byte_cursor_from_buf(&expected_publish.topic),
        .payload = aws_byte_cursor_from_buf(&expected_publish.payload),
        .ack_timeout_seconds = 2,
        .completion_callback_fn = s_rr_mqtt_protocol_adapter_test_on_publish_result,
        .user_data = &fixture,
    };

    aws_mqtt_protocol_adapter_publish(fixture.protocol_adapter, &publish_options);

    s_wait_for_incoming_publish_events_contains(&fixture, &expected_publish, 1);

    s_request_response_protocol_adapter_incoming_publish_event_record_clean_up(&expected_publish);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_incoming_publish,
    s_request_response_mqtt5_protocol_adapter_incoming_publish_fn)

static int s_request_response_mqtt5_protocol_adapter_shutdown_while_pending_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options test_options;
    aws_mqtt5_client_test_init_default_options(&test_options);

    // don't respond to anything
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] = NULL;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] = NULL;
    test_options.server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] = NULL;

    test_options.client_options.offline_queue_behavior = AWS_MQTT5_COQBT_FAIL_ALL_ON_DISCONNECT;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options mqtt5_test_fixture_options = {
        .client_options = &test_options.client_options,
        .server_function_table = &test_options.server_function_table,
    };

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt5(
        &fixture, allocator, &mqtt5_test_fixture_options));

    struct aws_mqtt5_client *client = fixture.protocol_context.mqtt5_fixture.client;

    ASSERT_SUCCESS(aws_mqtt5_client_start(client));

    aws_wait_for_connected_lifecycle_event(&fixture.protocol_context.mqtt5_fixture);

    // publish
    struct aws_protocol_adapter_publish_options publish_options = {
        .topic = aws_byte_cursor_from_c_str("hello/world"),
        .payload = aws_byte_cursor_from_c_str("SomePayload"),
        .ack_timeout_seconds = 5,
        .completion_callback_fn = s_rr_mqtt_protocol_adapter_test_on_publish_result,
        .user_data = &fixture,
    };

    aws_mqtt_protocol_adapter_publish(fixture.protocol_adapter, &publish_options);

    // subscribe
    struct aws_protocol_adapter_subscribe_options subscribe_options = {
        .topic_filter = aws_byte_cursor_from_c_str("hello/world"),
        .ack_timeout_seconds = 5,
    };

    aws_mqtt_protocol_adapter_subscribe(fixture.protocol_adapter, &subscribe_options);

    // unsubscribe
    struct aws_protocol_adapter_unsubscribe_options unsubscribe_options = {
        .topic_filter = aws_byte_cursor_from_c_str("hello/world"),
        .ack_timeout_seconds = 5,
    };

    aws_mqtt_protocol_adapter_unsubscribe(fixture.protocol_adapter, &unsubscribe_options);

    // tear down the adapter, leaving the in-progress operations with nothing to call back into
    s_aws_request_response_protocol_adapter_test_fixture_destroy_adapters(&fixture);

    // stop the mqtt client, which fails the pending MQTT operations
    ASSERT_SUCCESS(aws_mqtt5_client_stop(client, NULL, NULL));

    // wait for the stop to complete, which implies all the operations have been completed without calling back
    // into a deleted adapter
    aws_wait_for_n_lifecycle_events(&fixture.protocol_context.mqtt5_fixture, AWS_MQTT5_CLET_STOPPED, 1);

    // nothing to verify, we just don't want to crash

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt5_protocol_adapter_shutdown_while_pending,
    s_request_response_mqtt5_protocol_adapter_shutdown_while_pending_fn)

static int s_do_request_response_mqtt311_protocol_adapter_subscribe_test(
    struct aws_allocator *allocator,
    enum protocol_adapter_operation_test_type test_type) {
    aws_mqtt_library_init(allocator);

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt311(&fixture, allocator));

    struct aws_mqtt_client_connection *connection =
        fixture.protocol_context.mqtt311_fixture.mqtt311_test_context.mqtt_connection;

    struct mqtt_connection_state_test *test_context_311 =
        &fixture.protocol_context.mqtt311_fixture.mqtt311_test_context;

    switch (test_type) {
        case PAOTT_SUCCESS:
            mqtt_mock_server_enable_auto_ack(test_context_311->mock_server);
            break;
        case PAOTT_FAILURE_REASON_CODE_RETRYABLE:
        case PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE:
            mqtt_mock_server_enable_auto_ack(test_context_311->mock_server);
            mqtt_mock_server_suback_reason_code(test_context_311->mock_server, 128);
            break;
        case PAOTT_FAILURE_ERROR_CODE:
            // there is no reasonable way to generate an async error-code-based failure; so skip this case
            AWS_FATAL_ASSERT(false);
            break;
        case PAOTT_FAILURE_TIMEOUT:
            mqtt_mock_server_disable_auto_ack(test_context_311->mock_server);
        default:
            break;
    }

    struct aws_mqtt_connection_options connection_options = {
        .user_data = test_context_311,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(test_context_311->endpoint.address),
        .socket_options = &test_context_311->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(connection, &connection_options));

    struct aws_protocol_adapter_connection_event connection_success_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = false,
    };

    s_wait_for_connection_events_contains(&fixture, &connection_success_event, 1);

    int expected_error_code = s_test_type_to_expected_error_code(test_type, AWS_PAPT_MQTT311);

    struct request_response_protocol_adapter_subscription_event_record expected_outcome;
    s_request_response_protocol_adapter_subscription_event_record_init(
        &expected_outcome,
        allocator,
        AWS_PASET_SUBSCRIBE,
        aws_byte_cursor_from_c_str("hello/world"),
        expected_error_code,
        true);

    struct aws_protocol_adapter_subscribe_options subscribe_options = {
        .topic_filter = aws_byte_cursor_from_buf(&expected_outcome.topic_filter),
        .ack_timeout_seconds = 2,
    };

    aws_mqtt_protocol_adapter_subscribe(fixture.protocol_adapter, &subscribe_options);

    s_wait_for_subscription_events_contains(&fixture, &expected_outcome, 1);

    s_request_response_protocol_adapter_subscription_event_record_cleanup(&expected_outcome);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(connection, aws_test311_on_disconnect_fn, test_context_311));

    struct aws_protocol_adapter_connection_event interruption_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };

    s_wait_for_connection_events_contains(&fixture, &interruption_event, 1);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_request_response_mqtt311_protocol_adapter_subscribe_success_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt311_protocol_adapter_subscribe_test(allocator, PAOTT_SUCCESS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_subscribe_success,
    s_request_response_mqtt311_protocol_adapter_subscribe_success_fn)

static int s_request_response_mqtt311_protocol_adapter_subscribe_failure_timeout_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt311_protocol_adapter_subscribe_test(allocator, PAOTT_FAILURE_TIMEOUT));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_subscribe_failure_timeout,
    s_request_response_mqtt311_protocol_adapter_subscribe_failure_timeout_fn)

static int s_request_response_mqtt311_protocol_adapter_subscribe_failure_reason_code_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt311_protocol_adapter_subscribe_test(
        allocator, PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_subscribe_failure_reason_code,
    s_request_response_mqtt311_protocol_adapter_subscribe_failure_reason_code_fn)

static int s_do_request_response_mqtt311_protocol_adapter_unsubscribe_test(
    struct aws_allocator *allocator,
    enum protocol_adapter_operation_test_type test_type) {
    aws_mqtt_library_init(allocator);

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt311(&fixture, allocator));

    struct aws_mqtt_client_connection *connection =
        fixture.protocol_context.mqtt311_fixture.mqtt311_test_context.mqtt_connection;

    struct mqtt_connection_state_test *test_context_311 =
        &fixture.protocol_context.mqtt311_fixture.mqtt311_test_context;

    switch (test_type) {
        case PAOTT_SUCCESS:
            mqtt_mock_server_enable_auto_ack(test_context_311->mock_server);
            break;
        case PAOTT_FAILURE_REASON_CODE_RETRYABLE:
        case PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE:
            // 311 does not have unsuback reason codes or a way to fail
            AWS_FATAL_ASSERT(false);
            break;
        case PAOTT_FAILURE_ERROR_CODE:
            // there is no reasonable way to generate an async error-code-based failure; so skip this case
            AWS_FATAL_ASSERT(false);
            break;
        case PAOTT_FAILURE_TIMEOUT:
            mqtt_mock_server_disable_auto_ack(test_context_311->mock_server);
        default:
            break;
    }

    struct aws_mqtt_connection_options connection_options = {
        .user_data = test_context_311,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(test_context_311->endpoint.address),
        .socket_options = &test_context_311->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(connection, &connection_options));

    struct aws_protocol_adapter_connection_event connection_success_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = false,
    };

    s_wait_for_connection_events_contains(&fixture, &connection_success_event, 1);

    int expected_error_code = s_test_type_to_expected_error_code(test_type, AWS_PAPT_MQTT311);

    struct request_response_protocol_adapter_subscription_event_record expected_outcome;
    s_request_response_protocol_adapter_subscription_event_record_init(
        &expected_outcome,
        allocator,
        AWS_PASET_UNSUBSCRIBE,
        aws_byte_cursor_from_c_str("hello/world"),
        expected_error_code,
        expected_error_code == AWS_ERROR_MQTT_TIMEOUT);

    struct aws_protocol_adapter_unsubscribe_options unsubscribe_options = {
        .topic_filter = aws_byte_cursor_from_buf(&expected_outcome.topic_filter),
        .ack_timeout_seconds = 2,
    };

    aws_mqtt_protocol_adapter_unsubscribe(fixture.protocol_adapter, &unsubscribe_options);

    s_wait_for_subscription_events_contains(&fixture, &expected_outcome, 1);

    s_request_response_protocol_adapter_subscription_event_record_cleanup(&expected_outcome);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(connection, aws_test311_on_disconnect_fn, test_context_311));

    struct aws_protocol_adapter_connection_event interruption_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };

    s_wait_for_connection_events_contains(&fixture, &interruption_event, 1);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_request_response_mqtt311_protocol_adapter_unsubscribe_success_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt311_protocol_adapter_unsubscribe_test(allocator, PAOTT_SUCCESS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_unsubscribe_success,
    s_request_response_mqtt311_protocol_adapter_unsubscribe_success_fn)

static int s_request_response_mqtt311_protocol_adapter_unsubscribe_failure_timeout_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt311_protocol_adapter_unsubscribe_test(allocator, PAOTT_FAILURE_TIMEOUT));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_unsubscribe_failure_timeout,
    s_request_response_mqtt311_protocol_adapter_unsubscribe_failure_timeout_fn)

static int s_do_request_response_mqtt311_protocol_adapter_publish_test(
    struct aws_allocator *allocator,
    enum protocol_adapter_operation_test_type test_type) {
    aws_mqtt_library_init(allocator);

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt311(&fixture, allocator));

    struct aws_mqtt_client_connection *connection =
        fixture.protocol_context.mqtt311_fixture.mqtt311_test_context.mqtt_connection;

    struct mqtt_connection_state_test *test_context_311 =
        &fixture.protocol_context.mqtt311_fixture.mqtt311_test_context;

    switch (test_type) {
        case PAOTT_SUCCESS:
            mqtt_mock_server_enable_auto_ack(test_context_311->mock_server);
            break;
        case PAOTT_FAILURE_REASON_CODE_RETRYABLE:
        case PAOTT_FAILURE_REASON_CODE_NOT_RETRYABLE:
            // 311 does not have puback reason codes or a way to fail
            AWS_FATAL_ASSERT(false);
            break;
        case PAOTT_FAILURE_ERROR_CODE:
            // there is no reasonable way to generate an async error-code-based failure; so skip this case
            AWS_FATAL_ASSERT(false);
            break;
        case PAOTT_FAILURE_TIMEOUT:
            mqtt_mock_server_disable_auto_ack(test_context_311->mock_server);
        default:
            break;
    }

    struct aws_mqtt_connection_options connection_options = {
        .user_data = test_context_311,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(test_context_311->endpoint.address),
        .socket_options = &test_context_311->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 16960, /* basically stop automatically sending PINGREQ */
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(connection, &connection_options));

    struct aws_protocol_adapter_connection_event connection_success_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = false,
    };

    s_wait_for_connection_events_contains(&fixture, &connection_success_event, 1);

    struct aws_protocol_adapter_publish_options publish_options = {
        .topic = aws_byte_cursor_from_c_str("hello/world"),
        .payload = aws_byte_cursor_from_c_str("SomePayload"),
        .ack_timeout_seconds = 2,
        .completion_callback_fn = s_rr_mqtt_protocol_adapter_test_on_publish_result,
        .user_data = &fixture,
    };

    aws_mqtt_protocol_adapter_publish(fixture.protocol_adapter, &publish_options);

    int expected_error_code = s_test_type_to_expected_error_code(test_type, AWS_PAPT_MQTT311);
    s_wait_for_publish_results_contains(&fixture, expected_error_code, 1);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(connection, aws_test311_on_disconnect_fn, test_context_311));

    struct aws_protocol_adapter_connection_event interruption_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };

    s_wait_for_connection_events_contains(&fixture, &interruption_event, 1);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_request_response_mqtt311_protocol_adapter_publish_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt311_protocol_adapter_publish_test(allocator, PAOTT_SUCCESS));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_publish_success,
    s_request_response_mqtt311_protocol_adapter_publish_success_fn)

static int s_request_response_mqtt311_protocol_adapter_publish_failure_timeout_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_request_response_mqtt311_protocol_adapter_publish_test(allocator, PAOTT_FAILURE_TIMEOUT));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_publish_failure_timeout,
    s_request_response_mqtt311_protocol_adapter_publish_failure_timeout_fn)

static int s_request_response_mqtt311_protocol_adapter_connection_events_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt311(&fixture, allocator));

    struct aws_mqtt_client_connection *connection =
        fixture.protocol_context.mqtt311_fixture.mqtt311_test_context.mqtt_connection;

    struct mqtt_connection_state_test *test_context_311 =
        &fixture.protocol_context.mqtt311_fixture.mqtt311_test_context;

    // always rejoin session and cause continual ping timeouts to generate interrupted events
    mqtt_mock_server_set_session_present(test_context_311->mock_server, true);
    mqtt_mock_server_set_max_ping_resp(test_context_311->mock_server, 0);

    struct aws_mqtt_connection_options connection_options = {
        .user_data = test_context_311,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(test_context_311->endpoint.address),
        .socket_options = &test_context_311->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 2,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(connection, &connection_options));

    struct aws_protocol_adapter_connection_event connection_success_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = true,
    };

    s_wait_for_connection_events_contains(&fixture, &connection_success_event, 1);

    struct aws_protocol_adapter_connection_event interruption_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };

    /* wait for ping timeout disconnect */
    s_wait_for_connection_events_contains(&fixture, &interruption_event, 1);

    /* wait for automatic reconnect */
    s_wait_for_connection_events_contains(&fixture, &connection_success_event, 2);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(connection, aws_test311_on_disconnect_fn, test_context_311));

    s_wait_for_connection_events_contains(&fixture, &interruption_event, 2);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_connection_events,
    s_request_response_mqtt311_protocol_adapter_connection_events_fn)

static int s_request_response_mqtt311_protocol_adapter_incoming_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt311(&fixture, allocator));

    struct aws_mqtt_client_connection *connection =
        fixture.protocol_context.mqtt311_fixture.mqtt311_test_context.mqtt_connection;

    struct mqtt_connection_state_test *test_context_311 =
        &fixture.protocol_context.mqtt311_fixture.mqtt311_test_context;

    /* reflect publishes */
    mqtt_mock_server_set_publish_reflection(test_context_311->mock_server, true);

    struct aws_mqtt_connection_options connection_options = {
        .user_data = test_context_311,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(test_context_311->endpoint.address),
        .socket_options = &test_context_311->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = 30 * 1000,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 20000,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(connection, &connection_options));

    struct aws_protocol_adapter_connection_event connection_success_event = {
        .event_type = AWS_PACET_CONNECTED,
    };

    s_wait_for_connection_events_contains(&fixture, &connection_success_event, 1);

    struct request_response_protocol_adapter_incoming_publish_event_record expected_publish;
    AWS_ZERO_STRUCT(expected_publish);

    s_request_response_protocol_adapter_incoming_publish_event_record_init(
        &expected_publish,
        allocator,
        aws_byte_cursor_from_c_str("hello/world"),
        aws_byte_cursor_from_c_str("SomePayload"));

    struct aws_protocol_adapter_publish_options publish_options = {
        .topic = aws_byte_cursor_from_buf(&expected_publish.topic),
        .payload = aws_byte_cursor_from_buf(&expected_publish.payload),
        .ack_timeout_seconds = 2,
        .completion_callback_fn = s_rr_mqtt_protocol_adapter_test_on_publish_result,
        .user_data = &fixture,
    };

    aws_mqtt_protocol_adapter_publish(fixture.protocol_adapter, &publish_options);

    s_wait_for_incoming_publish_events_contains(&fixture, &expected_publish, 1);

    s_request_response_protocol_adapter_incoming_publish_event_record_clean_up(&expected_publish);

    struct aws_protocol_adapter_connection_event interruption_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(connection, aws_test311_on_disconnect_fn, test_context_311));

    s_wait_for_connection_events_contains(&fixture, &interruption_event, 1);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_incoming_publish,
    s_request_response_mqtt311_protocol_adapter_incoming_publish_fn)

static int s_request_response_mqtt311_protocol_adapter_shutdown_while_pending_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_request_response_protocol_adapter_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_request_response_protocol_adapter_test_fixture_init_mqtt311(&fixture, allocator));

    struct aws_mqtt_client_connection *connection =
        fixture.protocol_context.mqtt311_fixture.mqtt311_test_context.mqtt_connection;

    struct mqtt_connection_state_test *test_context_311 =
        &fixture.protocol_context.mqtt311_fixture.mqtt311_test_context;

    /* reflect publishes */
    mqtt_mock_server_set_publish_reflection(test_context_311->mock_server, true);

    struct aws_mqtt_connection_options connection_options = {
        .user_data = test_context_311,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(test_context_311->endpoint.address),
        .socket_options = &test_context_311->socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = 30 * 1000,
        .protocol_operation_timeout_ms = 3000,
        .keep_alive_time_secs = 20000,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(connection, &connection_options));

    struct aws_protocol_adapter_connection_event connection_success_event = {
        .event_type = AWS_PACET_CONNECTED,
    };

    s_wait_for_connection_events_contains(&fixture, &connection_success_event, 1);

    // publish
    struct aws_protocol_adapter_publish_options publish_options = {
        .topic = aws_byte_cursor_from_c_str("hello/world"),
        .payload = aws_byte_cursor_from_c_str("SomePayload"),
        .ack_timeout_seconds = 5,
        .completion_callback_fn = s_rr_mqtt_protocol_adapter_test_on_publish_result,
        .user_data = &fixture,
    };

    aws_mqtt_protocol_adapter_publish(fixture.protocol_adapter, &publish_options);

    // subscribe
    struct aws_protocol_adapter_subscribe_options subscribe_options = {
        .topic_filter = aws_byte_cursor_from_c_str("hello/world"),
        .ack_timeout_seconds = 5,
    };

    aws_mqtt_protocol_adapter_subscribe(fixture.protocol_adapter, &subscribe_options);

    // unsubscribe
    struct aws_protocol_adapter_unsubscribe_options unsubscribe_options = {
        .topic_filter = aws_byte_cursor_from_c_str("hello/world"),
        .ack_timeout_seconds = 5,
    };

    aws_mqtt_protocol_adapter_unsubscribe(fixture.protocol_adapter, &unsubscribe_options);

    // tear down the adapter, leaving the in-progress operations with nothing to call back into
    s_aws_request_response_protocol_adapter_test_fixture_destroy_adapters(&fixture);

    ASSERT_SUCCESS(aws_mqtt_client_connection_disconnect(connection, aws_test311_on_disconnect_fn, test_context_311));

    /* have to dig into the 311 test fixture because we won't be getting an interrupted event */
    aws_test311_wait_for_disconnect_to_complete(test_context_311);

    s_aws_request_response_protocol_adapter_test_fixture_clean_up(&fixture);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    request_response_mqtt311_protocol_adapter_shutdown_while_pending,
    s_request_response_mqtt311_protocol_adapter_shutdown_while_pending_fn)
