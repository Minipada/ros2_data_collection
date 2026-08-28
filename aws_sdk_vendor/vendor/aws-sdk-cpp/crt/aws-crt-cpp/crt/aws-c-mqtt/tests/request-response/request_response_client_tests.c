/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/clock.h>
#include <aws/common/json.h>
#include <aws/common/uuid.h>
#include <aws/mqtt/private/client_impl_shared.h>
#include <aws/mqtt/private/request-response/protocol_adapter.h>
#include <aws/mqtt/private/request-response/request_response_subscription_set.h>
#include <aws/mqtt/request-response/request_response_client.h>

#include <aws/testing/aws_test_harness.h>

#include "../v3/mqtt311_testing_utils.h"
#include "../v3/mqtt_mock_server_handler.h"
#include "../v5/mqtt5_testing_utils.h"

enum rr_test_client_protocol {
    RRCP_MQTT311,
    RRCP_MQTT5,
};

struct aws_rr_client_test_fixture {
    struct aws_allocator *allocator;

    struct aws_mqtt_request_response_client *rr_client;

    enum rr_test_client_protocol test_protocol;
    union {
        struct aws_mqtt5_client_mock_test_fixture mqtt5_test_fixture;
        struct mqtt_connection_state_test mqtt311_test_fixture;
    } client_test_fixture;

    void *test_context;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    bool client_initialized;
    bool client_destroyed;

    struct aws_hash_table request_response_records;
    struct aws_hash_table streaming_records;
};

struct aws_rr_client_fixture_request_response_record {
    struct aws_allocator *allocator;

    struct aws_rr_client_test_fixture *fixture;

    struct aws_byte_cursor record_key_cursor;
    struct aws_byte_buf record_key;

    bool completed;
    int error_code;
    struct aws_byte_buf response;
    struct aws_byte_buf response_topic;
};

struct aws_rr_client_fixture_request_response_record *s_aws_rr_client_fixture_request_response_record_new(
    struct aws_allocator *allocator,
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor request_payload) {
    struct aws_rr_client_fixture_request_response_record *record =
        aws_mem_calloc(allocator, 1, sizeof(struct aws_rr_client_fixture_request_response_record));

    record->allocator = allocator;
    record->fixture = fixture;

    aws_byte_buf_init_copy_from_cursor(&record->record_key, allocator, request_payload);
    record->record_key_cursor = aws_byte_cursor_from_buf(&record->record_key);

    return record;
}

void s_aws_rr_client_fixture_request_response_record_delete(
    struct aws_rr_client_fixture_request_response_record *record) {
    aws_byte_buf_clean_up(&record->record_key);
    aws_byte_buf_clean_up(&record->response);
    aws_byte_buf_clean_up(&record->response_topic);

    aws_mem_release(record->allocator, record);
}

static void s_aws_rr_client_fixture_request_response_record_hash_destroy(void *element) {
    struct aws_rr_client_fixture_request_response_record *record = element;

    s_aws_rr_client_fixture_request_response_record_delete(record);
}

static void s_rrc_fixture_request_completion_callback(
    const struct aws_mqtt_rr_incoming_publish_event *publish_event,
    int error_code,
    void *user_data) {
    struct aws_rr_client_fixture_request_response_record *record = user_data;
    struct aws_rr_client_test_fixture *fixture = record->fixture;

    aws_mutex_lock(&fixture->lock);

    if (error_code == AWS_ERROR_SUCCESS) {
        AWS_FATAL_ASSERT(publish_event != NULL);

        aws_byte_buf_init_copy_from_cursor(&record->response, fixture->allocator, publish_event->payload);
        aws_byte_buf_init_copy_from_cursor(&record->response_topic, fixture->allocator, publish_event->topic);
    } else {
        AWS_FATAL_ASSERT(publish_event == NULL);
        record->error_code = error_code;
    }

    record->completed = true;

    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static struct aws_rr_client_fixture_request_response_record *s_rrc_fixture_add_request_record(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor record_key) {
    struct aws_rr_client_fixture_request_response_record *record =
        s_aws_rr_client_fixture_request_response_record_new(fixture->allocator, fixture, record_key);

    aws_hash_table_put(&fixture->request_response_records, &record->record_key_cursor, record, NULL);

    return record;
}

struct rrc_operation_completion_context {
    struct aws_byte_cursor key;
    struct aws_rr_client_test_fixture *fixture;
};

static bool s_is_request_complete(void *context) {
    struct rrc_operation_completion_context *completion_context = context;

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(&completion_context->fixture->request_response_records, &completion_context->key, &element);

    AWS_FATAL_ASSERT(element != NULL && element->value != NULL);

    struct aws_rr_client_fixture_request_response_record *record = element->value;

    return record->completed;
}

static void s_rrc_wait_on_request_completion(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor record_key) {
    struct rrc_operation_completion_context context = {
        .key = record_key,
        .fixture = fixture,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_is_request_complete, &context);
    aws_mutex_unlock(&fixture->lock);
}

static int s_rrc_verify_request_completion(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor record_key,
    int expected_error_code,
    struct aws_byte_cursor *expected_response_topic,
    struct aws_byte_cursor *expected_response) {
    aws_mutex_lock(&fixture->lock);

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(&fixture->request_response_records, &record_key, &element);

    AWS_FATAL_ASSERT(element != NULL && element->value != NULL);

    struct aws_rr_client_fixture_request_response_record *record = element->value;

    ASSERT_INT_EQUALS(expected_error_code, record->error_code);

    if (expected_response != NULL) {
        struct aws_byte_cursor actual_payload = aws_byte_cursor_from_buf(&record->response);
        ASSERT_TRUE(aws_byte_cursor_eq(expected_response, &actual_payload));

        struct aws_byte_cursor actual_response_topic = aws_byte_cursor_from_buf(&record->response_topic);
        ASSERT_TRUE(aws_byte_cursor_eq(expected_response_topic, &actual_response_topic));
    } else {
        ASSERT_INT_EQUALS(0, record->response.len);
        ASSERT_INT_EQUALS(0, record->response_topic.len);
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

struct aws_rr_client_fixture_user_properties {
    struct aws_byte_buf name;
    struct aws_byte_buf value;
};

struct aws_rr_client_fixture_publish_message {
    struct aws_byte_buf payload;
    struct aws_byte_buf topic;
    struct aws_byte_buf content_type;
    size_t user_property_count;
    struct aws_rr_client_fixture_user_properties *user_properties;
    /* A view for user_properties field, for convenient comparison with expected results. */
    struct aws_mqtt5_user_property *user_properties_view;
    uint32_t message_expiry_interval_seconds;
};

struct aws_rr_client_fixture_publish_message_view {
    struct aws_byte_cursor payload;
    struct aws_byte_cursor topic;
    struct aws_byte_cursor content_type;
    size_t user_property_count;
    const struct aws_mqtt5_user_property *user_properties;
    uint32_t message_expiry_interval_seconds;
};

struct aws_rr_client_fixture_streaming_record {
    struct aws_allocator *allocator;

    struct aws_rr_client_test_fixture *fixture;

    struct aws_byte_cursor record_key_cursor;
    struct aws_byte_buf record_key;

    struct aws_array_list publishes;
    struct aws_array_list subscription_events;

    bool terminated;
};

struct aws_rr_client_fixture_streaming_record_subscription_event {
    enum aws_rr_streaming_subscription_event_type status;
    int error_code;
};

struct aws_rr_client_fixture_streaming_record *s_aws_rr_client_fixture_streaming_record_new(
    struct aws_allocator *allocator,
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor record_key) {
    struct aws_rr_client_fixture_streaming_record *record =
        aws_mem_calloc(allocator, 1, sizeof(struct aws_rr_client_fixture_streaming_record));

    record->allocator = allocator;
    record->fixture = fixture;

    aws_byte_buf_init_copy_from_cursor(&record->record_key, allocator, record_key);
    record->record_key_cursor = aws_byte_cursor_from_buf(&record->record_key);

    aws_array_list_init_dynamic(
        &record->publishes, allocator, 10, sizeof(struct aws_rr_client_fixture_publish_message));
    aws_array_list_init_dynamic(
        &record->subscription_events,
        allocator,
        10,
        sizeof(struct aws_rr_client_fixture_streaming_record_subscription_event));

    return record;
}

void s_aws_rr_client_fixture_streaming_record_delete(struct aws_rr_client_fixture_streaming_record *record) {
    aws_byte_buf_clean_up(&record->record_key);

    size_t publish_count = aws_array_list_length(&record->publishes);
    for (size_t i = 0; i < publish_count; ++i) {
        struct aws_rr_client_fixture_publish_message publish_message;
        aws_array_list_get_at(&record->publishes, &publish_message, i);

        aws_byte_buf_clean_up(&publish_message.payload);
        aws_byte_buf_clean_up(&publish_message.topic);
        aws_byte_buf_clean_up(&publish_message.content_type);
        aws_mem_release(record->allocator, publish_message.user_properties_view);
        for (size_t j = 0; j < publish_message.user_property_count; ++j) {
            aws_byte_buf_clean_up(&publish_message.user_properties[j].name);
            aws_byte_buf_clean_up(&publish_message.user_properties[j].value);
        }
        aws_mem_release(record->allocator, publish_message.user_properties);
    }

    aws_array_list_clean_up(&record->publishes);
    aws_array_list_clean_up(&record->subscription_events);

    aws_mem_release(record->allocator, record);
}

static void s_aws_rr_client_fixture_streaming_record_hash_destroy(void *element) {
    struct aws_rr_client_fixture_streaming_record *record = element;

    s_aws_rr_client_fixture_streaming_record_delete(record);
}

static void s_rrc_fixture_streaming_operation_subscription_status_callback(
    enum aws_rr_streaming_subscription_event_type status,
    int error_code,
    void *user_data) {

    struct aws_rr_client_fixture_streaming_record *record = user_data;
    struct aws_rr_client_test_fixture *fixture = record->fixture;

    aws_mutex_lock(&fixture->lock);

    struct aws_rr_client_fixture_streaming_record_subscription_event event = {
        .status = status,
        .error_code = error_code,
    };
    aws_array_list_push_back(&record->subscription_events, &event);

    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_rrc_fixture_streaming_operation_incoming_publish_callback(
    const struct aws_mqtt_rr_incoming_publish_event *publish_event,
    void *user_data) {
    struct aws_rr_client_fixture_streaming_record *record = user_data;
    struct aws_rr_client_test_fixture *fixture = record->fixture;

    aws_mutex_lock(&fixture->lock);

    struct aws_rr_client_fixture_publish_message publish_message;
    AWS_ZERO_STRUCT(publish_message);

    aws_byte_buf_init_copy_from_cursor(&publish_message.payload, fixture->allocator, publish_event->payload);
    aws_byte_buf_init_copy_from_cursor(&publish_message.topic, fixture->allocator, publish_event->topic);
    if (publish_event->content_type) {
        aws_byte_buf_init_copy_from_cursor(
            &publish_message.content_type, fixture->allocator, *publish_event->content_type);
    }

    publish_message.user_property_count = publish_event->user_property_count;
    if (publish_event->user_property_count > 0) {
        publish_message.user_properties = aws_mem_calloc(
            fixture->allocator,
            publish_event->user_property_count,
            sizeof(struct aws_rr_client_fixture_user_properties));
        publish_message.user_properties_view = aws_mem_calloc(
            fixture->allocator, publish_event->user_property_count, sizeof(struct aws_mqtt5_user_property));
        for (size_t i = 0; i < publish_event->user_property_count; ++i) {
            aws_byte_buf_init_copy_from_cursor(
                &publish_message.user_properties[i].name, fixture->allocator, publish_event->user_properties[i].name);
            aws_byte_buf_init_copy_from_cursor(
                &publish_message.user_properties[i].value, fixture->allocator, publish_event->user_properties[i].value);
            publish_message.user_properties_view[i].name =
                aws_byte_cursor_from_buf(&publish_message.user_properties[i].name);
            publish_message.user_properties_view[i].value =
                aws_byte_cursor_from_buf(&publish_message.user_properties[i].value);
        }
    }

    if (publish_event->message_expiry_interval_seconds) {
        publish_message.message_expiry_interval_seconds = *publish_event->message_expiry_interval_seconds;
    }

    aws_array_list_push_back(&record->publishes, &publish_message);

    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static void s_rrc_fixture_streaming_operation_terminated_callback(void *user_data) {
    struct aws_rr_client_fixture_streaming_record *record = user_data;
    struct aws_rr_client_test_fixture *fixture = record->fixture;

    aws_mutex_lock(&fixture->lock);

    record->terminated = true;

    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static struct aws_rr_client_fixture_streaming_record *s_rrc_fixture_add_streaming_record(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor key) {
    struct aws_rr_client_fixture_streaming_record *record =
        s_aws_rr_client_fixture_streaming_record_new(fixture->allocator, fixture, key);

    aws_hash_table_put(&fixture->streaming_records, &record->record_key_cursor, record, NULL);

    return record;
}

static bool s_is_stream_terminated(void *context) {
    struct rrc_operation_completion_context *completion_context = context;

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(&completion_context->fixture->streaming_records, &completion_context->key, &element);

    AWS_FATAL_ASSERT(element != NULL && element->value != NULL);

    struct aws_rr_client_fixture_streaming_record *record = element->value;

    return record->terminated;
}

static void s_rrc_wait_on_streaming_termination(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor key) {
    struct rrc_operation_completion_context context = {
        .key = key,
        .fixture = fixture,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_is_stream_terminated, &context);
    aws_mutex_unlock(&fixture->lock);
}

struct rrc_streaming_event_wait_context {
    struct aws_byte_cursor operation_key;
    struct aws_rr_client_test_fixture *fixture;
    size_t event_count;
};

static bool s_streaming_operation_has_n_publishes(void *context) {
    struct rrc_streaming_event_wait_context *streaming_publish_context = context;

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(
        &streaming_publish_context->fixture->streaming_records, &streaming_publish_context->operation_key, &element);

    AWS_FATAL_ASSERT(element != NULL && element->value != NULL);

    struct aws_rr_client_fixture_streaming_record *record = element->value;

    return aws_array_list_length(&record->publishes) >= streaming_publish_context->event_count;
}

static void s_rrc_wait_for_n_streaming_publishes(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor key,
    size_t count) {
    struct rrc_streaming_event_wait_context context = {
        .operation_key = key,
        .fixture = fixture,
        .event_count = count,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_streaming_operation_has_n_publishes, &context);
    aws_mutex_unlock(&fixture->lock);
}

static int s_rrc_verify_streaming_publishes(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor key,
    size_t expected_publish_count,
    struct aws_rr_client_fixture_publish_message_view *expected_publishes) {

    aws_mutex_lock(&fixture->lock);

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(&fixture->streaming_records, &key, &element);

    AWS_FATAL_ASSERT(element != NULL && element->value != NULL);

    struct aws_rr_client_fixture_streaming_record *record = element->value;

    size_t actual_publish_count = aws_array_list_length(&record->publishes);
    ASSERT_INT_EQUALS(expected_publish_count, actual_publish_count);

    for (size_t i = 0; i < actual_publish_count; ++i) {
        struct aws_rr_client_fixture_publish_message actual_publish_message;
        aws_array_list_get_at(&record->publishes, &actual_publish_message, i);

        struct aws_rr_client_fixture_publish_message_view *expected_publish_message = &expected_publishes[i];

        ASSERT_BIN_ARRAYS_EQUALS(
            expected_publish_message->payload.ptr,
            expected_publish_message->payload.len,
            actual_publish_message.payload.buffer,
            actual_publish_message.payload.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_publish_message->topic.ptr,
            expected_publish_message->topic.len,
            actual_publish_message.topic.buffer,
            actual_publish_message.topic.len);
        if (fixture->test_protocol == RRCP_MQTT5) {
            ASSERT_BIN_ARRAYS_EQUALS(
                expected_publish_message->content_type.ptr,
                expected_publish_message->content_type.len,
                actual_publish_message.content_type.buffer,
                actual_publish_message.content_type.len);
            aws_mqtt5_test_verify_user_properties_raw(
                actual_publish_message.user_property_count,
                actual_publish_message.user_properties_view,
                expected_publish_message->user_property_count,
                expected_publish_message->user_properties);
            ASSERT_INT_EQUALS(
                expected_publish_message->message_expiry_interval_seconds,
                actual_publish_message.message_expiry_interval_seconds);
        } else {
            ASSERT_INT_EQUALS(0, actual_publish_message.content_type.len);
            ASSERT_PTR_EQUALS(NULL, actual_publish_message.content_type.buffer);
            ASSERT_INT_EQUALS(0, actual_publish_message.user_property_count);
            ASSERT_PTR_EQUALS(NULL, actual_publish_message.user_properties);
            ASSERT_INT_EQUALS(0, actual_publish_message.message_expiry_interval_seconds);
        }
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

static bool s_streaming_operation_has_n_subscription_events(void *context) {
    struct rrc_streaming_event_wait_context *streaming_publish_context = context;

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(
        &streaming_publish_context->fixture->streaming_records, &streaming_publish_context->operation_key, &element);

    AWS_FATAL_ASSERT(element != NULL && element->value != NULL);

    struct aws_rr_client_fixture_streaming_record *record = element->value;

    return aws_array_list_length(&record->subscription_events) >= streaming_publish_context->event_count;
}

static void s_rrc_wait_for_n_streaming_subscription_events(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor key,
    size_t count) {
    struct rrc_streaming_event_wait_context context = {
        .operation_key = key,
        .fixture = fixture,
        .event_count = count,
    };

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(
        &fixture->signal, &fixture->lock, s_streaming_operation_has_n_subscription_events, &context);
    aws_mutex_unlock(&fixture->lock);
}

static int s_rrc_verify_streaming_record_subscription_events(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor key,
    size_t expected_subscription_event_count,
    struct aws_rr_client_fixture_streaming_record_subscription_event *expected_subscription_events) {
    aws_mutex_lock(&fixture->lock);

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(&fixture->streaming_records, &key, &element);

    AWS_FATAL_ASSERT(element != NULL && element->value != NULL);

    struct aws_rr_client_fixture_streaming_record *record = element->value;

    size_t actual_subscription_event_count = aws_array_list_length(&record->subscription_events);
    ASSERT_INT_EQUALS(expected_subscription_event_count, actual_subscription_event_count);

    for (size_t i = 0; i < actual_subscription_event_count; ++i) {
        struct aws_rr_client_fixture_streaming_record_subscription_event actual_event;
        aws_array_list_get_at(&record->subscription_events, &actual_event, i);

        struct aws_rr_client_fixture_streaming_record_subscription_event *expected_event =
            &expected_subscription_events[i];

        ASSERT_INT_EQUALS(expected_event->status, actual_event.status);
        ASSERT_INT_EQUALS(expected_event->error_code, actual_event.error_code);
    }

    aws_mutex_unlock(&fixture->lock);

    return AWS_OP_SUCCESS;
}

static void s_aws_rr_client_test_fixture_on_initialized(void *user_data) {
    struct aws_rr_client_test_fixture *fixture = user_data;

    aws_mutex_lock(&fixture->lock);
    fixture->client_initialized = true;
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static bool s_rr_client_test_fixture_initialized(void *context) {
    struct aws_rr_client_test_fixture *fixture = context;

    return fixture->client_initialized;
}

static void s_aws_rr_client_test_fixture_wait_for_initialized(struct aws_rr_client_test_fixture *fixture) {
    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_rr_client_test_fixture_initialized, fixture);
    aws_mutex_unlock(&fixture->lock);
}

static void s_aws_rr_client_test_fixture_on_terminated(void *user_data) {
    struct aws_rr_client_test_fixture *fixture = user_data;

    aws_mutex_lock(&fixture->lock);
    fixture->client_destroyed = true;
    aws_mutex_unlock(&fixture->lock);
    aws_condition_variable_notify_all(&fixture->signal);
}

static int s_aws_rr_client_test_fixture_init_from_mqtt5(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_allocator *allocator,
    struct aws_mqtt_request_response_client_options *rr_client_options,
    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *client_test_fixture_options,
    void *test_context) {
    AWS_ZERO_STRUCT(*fixture);
    fixture->allocator = allocator;
    fixture->test_protocol = RRCP_MQTT5;

    aws_mutex_init(&fixture->lock);
    aws_condition_variable_init(&fixture->signal);
    fixture->test_context = test_context;

    aws_hash_table_init(
        &fixture->request_response_records,
        allocator,
        10,
        aws_hash_byte_cursor_ptr,
        aws_mqtt_byte_cursor_hash_equality,
        NULL,
        s_aws_rr_client_fixture_request_response_record_hash_destroy);

    aws_hash_table_init(
        &fixture->streaming_records,
        allocator,
        10,
        aws_hash_byte_cursor_ptr,
        aws_mqtt_byte_cursor_hash_equality,
        NULL,
        s_aws_rr_client_fixture_streaming_record_hash_destroy);

    if (aws_mqtt5_client_mock_test_fixture_init(
            &fixture->client_test_fixture.mqtt5_test_fixture, allocator, client_test_fixture_options)) {
        return AWS_OP_ERR;
    }

    struct aws_mqtt_request_response_client_options client_options = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = 5,
    };

    if (rr_client_options != NULL) {
        client_options = *rr_client_options;
    }

    client_options.initialized_callback = s_aws_rr_client_test_fixture_on_initialized;
    client_options.terminated_callback = s_aws_rr_client_test_fixture_on_terminated;
    client_options.user_data = fixture;

    fixture->rr_client = aws_mqtt_request_response_client_new_from_mqtt5_client(
        allocator, fixture->client_test_fixture.mqtt5_test_fixture.client, &client_options);
    AWS_FATAL_ASSERT(fixture->rr_client != NULL);

    aws_mqtt5_client_start(fixture->client_test_fixture.mqtt5_test_fixture.client);

    aws_wait_for_connected_lifecycle_event(&fixture->client_test_fixture.mqtt5_test_fixture);
    s_aws_rr_client_test_fixture_wait_for_initialized(fixture);

    return AWS_OP_SUCCESS;
}

static int s_aws_rr_client_test_fixture_init_from_mqtt311(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_allocator *allocator,
    struct aws_mqtt_request_response_client_options *rr_client_options,
    void *test_context) {
    AWS_ZERO_STRUCT(*fixture);
    fixture->allocator = allocator;
    fixture->test_protocol = RRCP_MQTT311;

    aws_mutex_init(&fixture->lock);
    aws_condition_variable_init(&fixture->signal);
    fixture->test_context = test_context;

    aws_hash_table_init(
        &fixture->request_response_records,
        allocator,
        10,
        aws_hash_byte_cursor_ptr,
        aws_mqtt_byte_cursor_hash_equality,
        NULL,
        s_aws_rr_client_fixture_request_response_record_hash_destroy);

    aws_hash_table_init(
        &fixture->streaming_records,
        allocator,
        10,
        aws_hash_byte_cursor_ptr,
        aws_mqtt_byte_cursor_hash_equality,
        NULL,
        s_aws_rr_client_fixture_streaming_record_hash_destroy);

    aws_test311_setup_mqtt_server_fn(allocator, &fixture->client_test_fixture.mqtt311_test_fixture);

    struct aws_mqtt_request_response_client_options client_options = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = 5,
    };

    if (rr_client_options != NULL) {
        client_options = *rr_client_options;
    }

    client_options.initialized_callback = s_aws_rr_client_test_fixture_on_initialized;
    client_options.terminated_callback = s_aws_rr_client_test_fixture_on_terminated;
    client_options.user_data = fixture;

    struct aws_mqtt_client_connection *mqtt_client = fixture->client_test_fixture.mqtt311_test_fixture.mqtt_connection;

    /* Server should reflect PUBLISH messages. */
    mqtt_mock_server_set_publish_reflection(fixture->client_test_fixture.mqtt311_test_fixture.mock_server, true);

    fixture->rr_client =
        aws_mqtt_request_response_client_new_from_mqtt311_client(allocator, mqtt_client, &client_options);
    AWS_FATAL_ASSERT(fixture->rr_client != NULL);

    struct aws_mqtt_connection_options connection_options = {
        .user_data = &fixture->client_test_fixture.mqtt311_test_fixture,
        .clean_session = false,
        .client_id = aws_byte_cursor_from_c_str("client1234"),
        .host_name = aws_byte_cursor_from_c_str(fixture->client_test_fixture.mqtt311_test_fixture.endpoint.address),
        .socket_options = &fixture->client_test_fixture.mqtt311_test_fixture.socket_options,
        .on_connection_complete = aws_test311_on_connection_complete_fn,
        .ping_timeout_ms = DEFAULT_TEST_PING_TIMEOUT_MS,
        .keep_alive_time_secs = 16960,
    };

    ASSERT_SUCCESS(aws_mqtt_client_connection_connect(mqtt_client, &connection_options));
    aws_test311_wait_for_connection_to_complete(&fixture->client_test_fixture.mqtt311_test_fixture);

    s_aws_rr_client_test_fixture_wait_for_initialized(fixture);

    return AWS_OP_SUCCESS;
}

static bool s_rr_client_test_fixture_terminated(void *context) {
    struct aws_rr_client_test_fixture *fixture = context;

    return fixture->client_destroyed;
}

static void s_aws_rr_client_test_fixture_clean_up(
    struct aws_rr_client_test_fixture *fixture,
    bool protocol_has_terminated) {
    aws_mqtt_request_response_client_release(fixture->rr_client);

    aws_mutex_lock(&fixture->lock);
    aws_condition_variable_wait_pred(&fixture->signal, &fixture->lock, s_rr_client_test_fixture_terminated, fixture);
    aws_mutex_unlock(&fixture->lock);

    if (!protocol_has_terminated) {
        if (fixture->test_protocol == RRCP_MQTT5) {
            aws_mqtt5_client_mock_test_fixture_clean_up(&fixture->client_test_fixture.mqtt5_test_fixture);
        } else {
            struct mqtt_connection_state_test *mqtt311_test_fixture =
                &fixture->client_test_fixture.mqtt311_test_fixture;
            aws_mqtt_client_connection_disconnect(
                mqtt311_test_fixture->mqtt_connection, aws_test311_on_disconnect_fn, mqtt311_test_fixture);
            aws_test311_clean_up_mqtt_server_fn(
                fixture->allocator, AWS_OP_SUCCESS, &fixture->client_test_fixture.mqtt311_test_fixture);
        }
    }

    aws_thread_join_all_managed();
    aws_mutex_clean_up(&fixture->lock);
    aws_condition_variable_clean_up(&fixture->signal);

    aws_hash_table_clean_up(&fixture->request_response_records);
    aws_hash_table_clean_up(&fixture->streaming_records);
}

static int s_rrc_mqtt5_create_destroy_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    aws_mqtt5_client_test_init_default_options(&client_test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options client_test_fixture_options = {
        .client_options = &client_test_options.client_options,
        .server_function_table = &client_test_options.server_function_table,
    };

    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(
        s_aws_rr_client_test_fixture_init_from_mqtt5(&fixture, allocator, NULL, &client_test_fixture_options, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_mqtt5_create_destroy, s_rrc_mqtt5_create_destroy_fn)

static int s_rrc_mqtt311_create_destroy_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_rr_client_test_fixture_init_from_mqtt311(&fixture, allocator, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_mqtt311_create_destroy, s_rrc_mqtt311_create_destroy_fn)

static char s_response_filter_wildcard[] = "response/filter/+";
static struct aws_byte_cursor s_response_filter_wildcard_cursor = {
    .ptr = (uint8_t *)s_response_filter_wildcard,
    .len = AWS_ARRAY_SIZE(s_response_filter_wildcard) - 1,
};

static int s_rrc_do_submit_request_operation_failure_test(
    struct aws_allocator *allocator,
    void (*request_mutator_fn)(struct aws_mqtt_request_operation_options *)) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    aws_mqtt5_client_test_init_default_options(&client_test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options client_test_fixture_options = {
        .client_options = &client_test_options.client_options,
        .server_function_table = &client_test_options.server_function_table,
    };

    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(
        s_aws_rr_client_test_fixture_init_from_mqtt5(&fixture, allocator, NULL, &client_test_fixture_options, NULL));

    struct aws_mqtt_request_operation_response_path response_paths[] = {
        {
            .topic = aws_byte_cursor_from_c_str("response/filter/accepted"),
            .correlation_token_json_path = aws_byte_cursor_from_c_str("client_token"),
        },
        {
            .topic = aws_byte_cursor_from_c_str("response/filter/rejected"),
            .correlation_token_json_path = aws_byte_cursor_from_c_str("client_token"),
        },
    };
    struct aws_mqtt_request_operation_options good_request = {
        .subscription_topic_filters = &s_response_filter_wildcard_cursor,
        .subscription_topic_filter_count = 1,
        .response_paths = response_paths,
        .response_path_count = AWS_ARRAY_SIZE(response_paths),
        .publish_topic = aws_byte_cursor_from_c_str("get/shadow"),
        .serialized_request = aws_byte_cursor_from_c_str("{}"),
        .correlation_token = aws_byte_cursor_from_c_str("MyRequest#1"),
    };
    ASSERT_SUCCESS(aws_mqtt_request_response_client_submit_request(fixture.rr_client, &good_request));

    struct aws_mqtt_request_operation_options bad_request = good_request;
    (*request_mutator_fn)(&bad_request);

    ASSERT_FAILS(aws_mqtt_request_response_client_submit_request(fixture.rr_client, &bad_request));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static void s_no_response_paths_mutator(struct aws_mqtt_request_operation_options *request_options) {
    request_options->response_path_count = 0;
    request_options->response_paths = NULL;
}

static int s_rrc_submit_request_operation_failure_no_response_paths_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_rrc_do_submit_request_operation_failure_test(allocator, s_no_response_paths_mutator);
}

AWS_TEST_CASE(
    rrc_submit_request_operation_failure_no_response_paths,
    s_rrc_submit_request_operation_failure_no_response_paths_fn)

static void s_invalid_response_topic_mutator(struct aws_mqtt_request_operation_options *request_options) {
    request_options->response_paths[0].topic = aws_byte_cursor_from_c_str("a/b/#");
}

static int s_rrc_submit_request_operation_failure_invalid_response_topic_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_rrc_do_submit_request_operation_failure_test(allocator, s_invalid_response_topic_mutator);
}

AWS_TEST_CASE(
    rrc_submit_request_operation_failure_invalid_response_topic,
    s_rrc_submit_request_operation_failure_invalid_response_topic_fn)

static void s_invalid_publish_topic_mutator(struct aws_mqtt_request_operation_options *request_options) {
    request_options->publish_topic = aws_byte_cursor_from_c_str("a/b/#");
}

static int s_rrc_submit_request_operation_failure_invalid_publish_topic_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_rrc_do_submit_request_operation_failure_test(allocator, s_invalid_publish_topic_mutator);
}

AWS_TEST_CASE(
    rrc_submit_request_operation_failure_invalid_publish_topic,
    s_rrc_submit_request_operation_failure_invalid_publish_topic_fn)

static char s_bad_filter[] = "a/#/c";
static struct aws_byte_cursor s_bad_filter_cursor = {
    .ptr = (uint8_t *)s_bad_filter,
    .len = AWS_ARRAY_SIZE(s_bad_filter) - 1,
};

static void s_invalid_subscription_topic_filter_mutator(struct aws_mqtt_request_operation_options *request_options) {
    request_options->subscription_topic_filters = &s_bad_filter_cursor;
}

static int s_rrc_submit_request_operation_failure_invalid_subscription_topic_filter_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_rrc_do_submit_request_operation_failure_test(allocator, s_invalid_subscription_topic_filter_mutator);
}

AWS_TEST_CASE(
    rrc_submit_request_operation_failure_invalid_subscription_topic_filter,
    s_rrc_submit_request_operation_failure_invalid_subscription_topic_filter_fn)

static void s_no_subscription_topic_filter_mutator(struct aws_mqtt_request_operation_options *request_options) {
    request_options->subscription_topic_filter_count = 0;
}

static int s_rrc_submit_request_operation_failure_no_subscription_topic_filters_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_rrc_do_submit_request_operation_failure_test(allocator, s_no_subscription_topic_filter_mutator);
}

AWS_TEST_CASE(
    rrc_submit_request_operation_failure_no_subscription_topic_filters,
    s_rrc_submit_request_operation_failure_no_subscription_topic_filters_fn)

static void s_empty_request_mutator(struct aws_mqtt_request_operation_options *request_options) {
    request_options->serialized_request = aws_byte_cursor_from_c_str("");
}

static int s_rrc_submit_request_operation_failure_empty_request_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_rrc_do_submit_request_operation_failure_test(allocator, s_empty_request_mutator);
}

AWS_TEST_CASE(
    rrc_submit_request_operation_failure_empty_request,
    s_rrc_submit_request_operation_failure_empty_request_fn)

static int s_rrc_submit_streaming_operation_failure_invalid_subscription_topic_filter_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    aws_mqtt5_client_test_init_default_options(&client_test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options client_test_fixture_options = {
        .client_options = &client_test_options.client_options,
        .server_function_table = &client_test_options.server_function_table,
    };

    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(
        s_aws_rr_client_test_fixture_init_from_mqtt5(&fixture, allocator, NULL, &client_test_fixture_options, NULL));

    struct aws_mqtt_streaming_operation_options good_options = {
        .topic_filter = aws_byte_cursor_from_c_str("a/b"),
    };

    struct aws_mqtt_rr_client_operation *good_operation =
        aws_mqtt_request_response_client_create_streaming_operation(fixture.rr_client, &good_options);
    ASSERT_NOT_NULL(good_operation);
    ASSERT_SUCCESS(aws_mqtt_rr_client_operation_activate(good_operation));

    aws_mqtt_rr_client_operation_release(good_operation);

    struct aws_mqtt_streaming_operation_options bad_options = good_options;
    bad_options.topic_filter = aws_byte_cursor_from_c_str("");

    struct aws_mqtt_rr_client_operation *bad_operation =
        aws_mqtt_request_response_client_create_streaming_operation(fixture.rr_client, &bad_options);
    ASSERT_NULL(bad_operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_submit_streaming_operation_failure_invalid_subscription_topic_filter,
    s_rrc_submit_streaming_operation_failure_invalid_subscription_topic_filter_fn)

static int s_do_rrc_single_request_operation_test_fn(
    struct aws_allocator *allocator,
    struct aws_mqtt_request_response_client_options *rr_client_options,
    struct aws_mqtt_request_operation_options *request_options,
    int expected_error_code,
    struct aws_byte_cursor *expected_response_topic,
    struct aws_byte_cursor *expected_payload,
    bool shutdown_after_submit) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    aws_mqtt5_client_test_init_default_options(&client_test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options client_test_fixture_options = {
        .client_options = &client_test_options.client_options,
        .server_function_table = &client_test_options.server_function_table,
    };

    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_rr_client_test_fixture_init_from_mqtt5(
        &fixture, allocator, rr_client_options, &client_test_fixture_options, NULL));

    struct aws_rr_client_fixture_request_response_record *record =
        s_rrc_fixture_add_request_record(&fixture, request_options->serialized_request);

    request_options->completion_callback = s_rrc_fixture_request_completion_callback;
    request_options->user_data = record;

    ASSERT_SUCCESS(aws_mqtt_request_response_client_submit_request(fixture.rr_client, request_options));

    if (shutdown_after_submit) {
        aws_mqtt_request_response_client_release(fixture.rr_client);
        fixture.rr_client = NULL;
    }

    s_rrc_wait_on_request_completion(&fixture, request_options->serialized_request);

    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, request_options->serialized_request, expected_error_code, expected_response_topic, expected_payload));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_rrc_submit_request_operation_failure_by_shutdown_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt_request_operation_response_path response_paths[] = {
        {
            .topic = aws_byte_cursor_from_c_str("response/filter/accepted"),
            .correlation_token_json_path = aws_byte_cursor_from_c_str("client_token"),
        },
    };

    struct aws_mqtt_request_operation_options request = {
        .subscription_topic_filters = &s_response_filter_wildcard_cursor,
        .subscription_topic_filter_count = 1,
        .response_paths = response_paths,
        .response_path_count = AWS_ARRAY_SIZE(response_paths),
        .publish_topic = aws_byte_cursor_from_c_str("get/shadow"),
        .serialized_request = aws_byte_cursor_from_c_str("request1"),
        .correlation_token = aws_byte_cursor_from_c_str("MyRequest#1"),
    };

    return s_do_rrc_single_request_operation_test_fn(
        allocator, NULL, &request, AWS_ERROR_MQTT_REQUEST_RESPONSE_CLIENT_SHUT_DOWN, NULL, NULL, true);
}

AWS_TEST_CASE(rrc_submit_request_operation_failure_by_shutdown, s_rrc_submit_request_operation_failure_by_shutdown_fn)

static int s_do_rrc_single_streaming_operation_test_fn(
    struct aws_allocator *allocator,
    struct aws_mqtt_request_response_client_options *rr_client_options,
    struct aws_mqtt_streaming_operation_options *streaming_options,
    size_t expected_subscription_event_count,
    struct aws_rr_client_fixture_streaming_record_subscription_event *expected_subscription_events,
    bool should_activate,
    bool should_activate_after_shutdown,
    bool release_after_client_shutdown) {
    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    aws_mqtt5_client_test_init_default_options(&client_test_options);

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options client_test_fixture_options = {
        .client_options = &client_test_options.client_options,
        .server_function_table = &client_test_options.server_function_table,
    };

    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_rr_client_test_fixture_init_from_mqtt5(
        &fixture, allocator, rr_client_options, &client_test_fixture_options, NULL));

    struct aws_byte_cursor streaming_id = aws_byte_cursor_from_c_str("streaming1");
    struct aws_rr_client_fixture_streaming_record *record = s_rrc_fixture_add_streaming_record(&fixture, streaming_id);

    streaming_options->incoming_publish_callback = s_rrc_fixture_streaming_operation_incoming_publish_callback;
    streaming_options->subscription_status_callback = s_rrc_fixture_streaming_operation_subscription_status_callback;
    streaming_options->terminated_callback = s_rrc_fixture_streaming_operation_terminated_callback;
    streaming_options->user_data = record;

    struct aws_mqtt_rr_client_operation *streaming_operation =
        aws_mqtt_request_response_client_create_streaming_operation(fixture.rr_client, streaming_options);
    ASSERT_NOT_NULL(streaming_operation);

    if (should_activate && !should_activate_after_shutdown) {
        ASSERT_SUCCESS(aws_mqtt_rr_client_operation_activate(streaming_operation));
    }

    aws_mqtt_request_response_client_release(fixture.rr_client);
    fixture.rr_client = NULL;

    /*
     * Extremely awkward sleep:
     *
     * We've submitted the operation and we've decref'd the client to zero.  When the operation submit task
     * is processed, if the release in the succeeding line has happened-before the client external destroy task
     * has run, then the operation's destroy will be scheduled in-thread and run ahead of the client external
     * destroy.  This doesn't break correctness, but it does prevent the client from emitting a HALTED event
     * on the subscription because the subscription/operation will be gone before the client external destroy
     * task runs.
     *
     * So we add a nice, fat sleep to guarantee that the client external destroy task runs before the operation
     * destroy task.
     */
    aws_thread_current_sleep(aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    if (should_activate && should_activate_after_shutdown) {
        ASSERT_SUCCESS(aws_mqtt_rr_client_operation_activate(streaming_operation));
    }

    /* Wait for the client finish termination.
     * This option is used for a regression test. In the previous implementation, releasing of the protocol client would
     * potentially release the event loop group if no other component acquire the event loop group. This would cause a
     * race condition where the rr_client or streaming operation is trying to use the event loop to process its tasks
     * while the event loop has already be released. We fixed it by acquire the event loop group for rr client. */
    if (release_after_client_shutdown) {
        aws_mqtt5_client_mock_test_fixture_clean_up(&fixture.client_test_fixture.mqtt5_test_fixture);
        // We sleep here to wait for event loop to be released by the client.
        aws_thread_current_sleep(aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));
    }

    aws_mqtt_rr_client_operation_release(streaming_operation);

    s_rrc_wait_on_streaming_termination(&fixture, streaming_id);

    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, streaming_id, expected_subscription_event_count, expected_subscription_events));

    s_aws_rr_client_test_fixture_clean_up(&fixture, release_after_client_shutdown);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_rrc_activate_streaming_operation_and_shutdown_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_HALTED,
            .error_code = AWS_ERROR_MQTT_REQUEST_RESPONSE_CLIENT_SHUT_DOWN,
        },
    };

    struct aws_mqtt_streaming_operation_options streaming_options = {
        .topic_filter = aws_byte_cursor_from_c_str("derp/filter"),
    };

    return s_do_rrc_single_streaming_operation_test_fn(
        allocator,
        NULL,
        &streaming_options,
        AWS_ARRAY_SIZE(expected_events),
        expected_events,
        /*should_activate=*/true,
        /*should_activate_after_shutdown=*/false,
        /*release_after_client_shutdown=*/false);
}

AWS_TEST_CASE(rrc_activate_streaming_operation_and_shutdown, s_rrc_activate_streaming_operation_and_shutdown_fn)

static int s_rrc_shutdown_and_activate_streaming_operation_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_HALTED,
            .error_code = AWS_ERROR_MQTT_REQUEST_RESPONSE_CLIENT_SHUT_DOWN,
        },
    };

    struct aws_mqtt_streaming_operation_options streaming_options = {
        .topic_filter = aws_byte_cursor_from_c_str("derp/filter"),
    };

    return s_do_rrc_single_streaming_operation_test_fn(
        allocator,
        NULL,
        &streaming_options,
        AWS_ARRAY_SIZE(expected_events),
        expected_events,
        /*should_activate=*/true,
        /*should_activate_after_shutdown=*/true,
        /*release_after_client_shutdown=*/false);
}

AWS_TEST_CASE(rrc_shutdown_and_activate_streaming_operation, s_rrc_shutdown_and_activate_streaming_operation_fn)

static int s_rrc_create_streaming_operation_and_shutdown_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt_streaming_operation_options streaming_options = {
        .topic_filter = aws_byte_cursor_from_c_str("derp/filter"),
    };

    return s_do_rrc_single_streaming_operation_test_fn(
        allocator,
        NULL,
        &streaming_options,
        0,
        NULL,
        /*should_activate=*/false,
        /*should_activate_after_shutdown=*/false,
        /*release_after_client_shutdown=*/false);
}

AWS_TEST_CASE(rrc_create_streaming_operation_and_shutdown, s_rrc_create_streaming_operation_and_shutdown_fn)

static int s_rrc_release_streaming_operation_after_client_shutdown_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt_streaming_operation_options streaming_options = {
        .topic_filter = aws_byte_cursor_from_c_str("derp/filter")};

    return s_do_rrc_single_streaming_operation_test_fn(
        allocator,
        NULL,
        &streaming_options,
        0,
        NULL,
        /*should_activate=*/false,
        /*should_activate_after_shutdown=*/false,
        /*release_after_client_shutdown=*/true);
}

AWS_TEST_CASE(
    rrc_release_streaming_operation_after_client_shutdown,
    s_rrc_release_streaming_operation_after_client_shutdown_fn)

static int s_rrc_submit_request_operation_failure_by_timeout_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt_request_operation_response_path response_paths[] = {
        {
            .topic = aws_byte_cursor_from_c_str("response/filter/accepted"),
            .correlation_token_json_path = aws_byte_cursor_from_c_str("client_token"),
        },
    };

    struct aws_mqtt_request_operation_options request = {
        .subscription_topic_filters = &s_response_filter_wildcard_cursor,
        .subscription_topic_filter_count = 1,
        .response_paths = response_paths,
        .response_path_count = AWS_ARRAY_SIZE(response_paths),
        .publish_topic = aws_byte_cursor_from_c_str("get/shadow"),
        .serialized_request = aws_byte_cursor_from_c_str("request1"),
        .correlation_token = aws_byte_cursor_from_c_str("MyRequest#1"),
    };

    struct aws_mqtt_request_response_client_options rr_client_options = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 1,
        .operation_timeout_seconds = 2,
    };

    return s_do_rrc_single_request_operation_test_fn(
        allocator, &rr_client_options, &request, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL, false);
}

AWS_TEST_CASE(rrc_submit_request_operation_failure_by_timeout, s_rrc_submit_request_operation_failure_by_timeout_fn)

static struct aws_mqtt_rr_client_operation *s_create_streaming_operation(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor record_key,
    struct aws_byte_cursor topic_filter) {
    struct aws_rr_client_fixture_streaming_record *record = s_rrc_fixture_add_streaming_record(fixture, record_key);

    struct aws_mqtt_streaming_operation_options streaming_options = {
        .topic_filter = topic_filter,
    };
    streaming_options.incoming_publish_callback = s_rrc_fixture_streaming_operation_incoming_publish_callback;
    streaming_options.subscription_status_callback = s_rrc_fixture_streaming_operation_subscription_status_callback;
    streaming_options.terminated_callback = s_rrc_fixture_streaming_operation_terminated_callback;
    streaming_options.user_data = record;

    struct aws_mqtt_rr_client_operation *streaming_operation =
        aws_mqtt_request_response_client_create_streaming_operation(fixture->rr_client, &streaming_options);
    aws_mqtt_rr_client_operation_activate(streaming_operation);

    return streaming_operation;
}

static int s_rrc_publish_5(
    struct aws_mqtt5_client *client,
    struct aws_byte_cursor topic,
    struct aws_byte_cursor payload,
    const struct aws_byte_cursor *content_type,
    size_t user_property_count,
    const struct aws_mqtt5_user_property *user_properties,
    const uint32_t *message_expiry_interval_seconds) {
    struct aws_mqtt5_packet_publish_view publish_options = {
        .topic = topic,
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .payload = payload,
        .content_type = content_type,
        .user_property_count = user_property_count,
        .user_properties = user_properties,
        .message_expiry_interval_seconds = message_expiry_interval_seconds};

    struct aws_mqtt5_publish_completion_options completion_options;
    AWS_ZERO_STRUCT(completion_options);

    return aws_mqtt5_client_publish(client, &publish_options, &completion_options);
}

static int s_rrc_publish_311(
    struct aws_mqtt_client_connection *connection,
    struct aws_byte_cursor topic,
    struct aws_byte_cursor payload) {
    uint16_t packet_id =
        aws_mqtt_client_connection_publish(connection, &topic, AWS_MQTT_QOS_AT_LEAST_ONCE, false, &payload, NULL, NULL);
    return packet_id > 0 ? AWS_OP_SUCCESS : AWS_OP_ERR;
}

static int s_rrc_protocol_client_publish(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor topic,
    struct aws_byte_cursor payload) {

    if (fixture->test_protocol == RRCP_MQTT311) {
        return s_rrc_publish_311(fixture->client_test_fixture.mqtt311_test_fixture.mqtt_connection, topic, payload);
    } else {
        const struct aws_byte_cursor *content_type = NULL;
        size_t user_property_count = 0;
        const struct aws_mqtt5_user_property *user_properties = NULL;
        const uint32_t *message_expiry_interval_seconds = NULL;
        return s_rrc_publish_5(
            fixture->client_test_fixture.mqtt5_test_fixture.client,
            topic,
            payload,
            content_type,
            user_property_count,
            user_properties,
            message_expiry_interval_seconds);
    }
}

static int s_rrc_protocol_client_publish_with_extra_fields(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor topic,
    struct aws_byte_cursor payload,
    const struct aws_byte_cursor *content_type,
    size_t user_property_count,
    const struct aws_mqtt5_user_property *user_properties,
    const uint32_t *message_expiry_interval_seconds) {

    if (fixture->test_protocol == RRCP_MQTT311) {
        return s_rrc_publish_311(fixture->client_test_fixture.mqtt311_test_fixture.mqtt_connection, topic, payload);
    } else {
        return s_rrc_publish_5(
            fixture->client_test_fixture.mqtt5_test_fixture.client,
            topic,
            payload,
            content_type,
            user_property_count,
            user_properties,
            message_expiry_interval_seconds);
    }
}

typedef void(modify_fixture_options_fn)(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options);

static int s_init_fixture_streaming_operation_success(
    struct aws_rr_client_test_fixture *fixture,
    struct mqtt5_client_test_options *client_test_options,
    struct aws_allocator *allocator,
    modify_fixture_options_fn *config_modifier,
    void *user_data,
    enum rr_test_client_protocol protocol) {

    if (protocol == RRCP_MQTT5) {
        aws_mqtt5_client_test_init_default_options(client_test_options);

        client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
            aws_mqtt5_server_send_suback_on_subscribe;
        client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
            aws_mqtt5_mock_server_handle_publish_puback_and_forward;

        struct aws_mqtt5_client_mqtt5_mock_test_fixture_options client_test_fixture_options = {
            .client_options = &client_test_options->client_options,
            .server_function_table = &client_test_options->server_function_table,
            .mock_server_user_data = user_data,
        };

        struct aws_mqtt_request_response_client_options rr_client_options = {
            .max_request_response_subscriptions = 2,
            .max_streaming_subscriptions = 1,
            .operation_timeout_seconds = 2,
        };

        if (config_modifier != NULL) {
            (*config_modifier)(&rr_client_options, client_test_options);
        }

        ASSERT_SUCCESS(s_aws_rr_client_test_fixture_init_from_mqtt5(
            fixture, allocator, &rr_client_options, &client_test_fixture_options, NULL));
    } else if (protocol == RRCP_MQTT311) {
        struct aws_mqtt_request_response_client_options rr_client_options = {
            .max_request_response_subscriptions = 2,
            .max_streaming_subscriptions = 1,
            .operation_timeout_seconds = 2,
        };
        ASSERT_SUCCESS(s_aws_rr_client_test_fixture_init_from_mqtt311(fixture, allocator, &rr_client_options, NULL));
    } else {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

/*
 * Minimal success test:
 *
 * Create a streaming operation, verify subscription established, inject several publishes to the operation's topic,
 * verify publishes received and routed through callbacks
 */
static int s_rrc_streaming_operation_success_single_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(
        s_init_fixture_streaming_operation_success(&fixture, &client_test_options, allocator, NULL, NULL, RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic = aws_byte_cursor_from_c_str("topic/+");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_streaming_operation_success_single, s_rrc_streaming_operation_success_single_fn)

/*
 * Test that all required PUBLISH packet fields are passed to stream operation if they're present in the packet.
 */
static int s_rrc_streaming_operation_success_capture_publish_packet_fn(
    struct aws_allocator *allocator,
    enum rr_test_client_protocol protocol) {

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(
        s_init_fixture_streaming_operation_success(&fixture, &client_test_options, allocator, NULL, NULL, protocol));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    /* two publishes on the mqtt client that get reflected into our subscription topic */
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor content_type1 = aws_byte_cursor_from_c_str("application/json");
    uint32_t message_expiry_interval_seconds1 = 10;

    char user_prop1_name[] = "Property1";
    char user_prop1_value[] = "Value1";
    char user_prop2_name[] = "Property2";
    char user_prop2_value[] = "Value2";
    const struct aws_mqtt5_user_property user_properties1[] = {
        {
            .name =
                {
                    .ptr = (uint8_t *)user_prop1_name,
                    .len = AWS_ARRAY_SIZE(user_prop1_name) - 1,
                },
            .value =
                {
                    .ptr = (uint8_t *)user_prop1_value,
                    .len = AWS_ARRAY_SIZE(user_prop1_value) - 1,
                },
        },
        {
            .name =
                {
                    .ptr = (uint8_t *)user_prop2_name,
                    .len = AWS_ARRAY_SIZE(user_prop2_name) - 1,
                },
            .value =
                {
                    .ptr = (uint8_t *)user_prop2_value,
                    .len = AWS_ARRAY_SIZE(user_prop2_value) - 1,
                },
        },
    };

    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    struct aws_byte_cursor content_type2 = aws_byte_cursor_from_c_str("");
    uint32_t message_expiry_interval_seconds2 = 0;
    const struct aws_mqtt5_user_property user_properties2[] = {{
        .name =
            {
                .ptr = (uint8_t *)user_prop1_name,
                .len = AWS_ARRAY_SIZE(user_prop1_name) - 1,
            },
        .value =
            {
                .ptr = (uint8_t *)user_prop1_value,
                .len = AWS_ARRAY_SIZE(user_prop1_value) - 1,
            },
    }};

    ASSERT_SUCCESS(s_rrc_protocol_client_publish_with_extra_fields(
        &fixture,
        topic_filter1,
        payload1,
        &content_type1,
        AWS_ARRAY_SIZE(user_properties1),
        user_properties1,
        &message_expiry_interval_seconds1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish_with_extra_fields(
        &fixture,
        topic_filter1,
        payload2,
        &content_type2,
        AWS_ARRAY_SIZE(user_properties2),
        user_properties2,
        &message_expiry_interval_seconds2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
            content_type1,
            AWS_ARRAY_SIZE(user_properties1),
            user_properties1,
            message_expiry_interval_seconds1,
        },
        {
            payload2,
            topic_filter1,
            content_type2,
            AWS_ARRAY_SIZE(user_properties2),
            user_properties2,
            message_expiry_interval_seconds2,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_rrc_streaming_operation_success_capture_mqtt5_publish_packet_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;
    return s_rrc_streaming_operation_success_capture_publish_packet_fn(allocator, RRCP_MQTT5);
}

AWS_TEST_CASE(
    rrc_streaming_operation_success_capture_mqtt5_publish_packet,
    s_rrc_streaming_operation_success_capture_mqtt5_publish_packet_fn)

static int s_rrc_streaming_operation_success_capture_mqtt311_publish_packet_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;
    return s_rrc_streaming_operation_success_capture_publish_packet_fn(allocator, RRCP_MQTT311);
}

AWS_TEST_CASE(
    rrc_streaming_operation_success_capture_mqtt311_publish_packet,
    s_rrc_streaming_operation_success_capture_mqtt311_publish_packet_fn)

/*
 * Variant of the minimal success test where we create two operations on the same topic filter, verify they both
 * get subscriptions established and publishes, then close one, send another publish and verify only the still-open
 * operation received it.
 */
static int s_rrc_streaming_operation_success_overlapping_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(
        s_init_fixture_streaming_operation_success(&fixture, &client_test_options, allocator, NULL, NULL, RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation1 =
        s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    struct aws_byte_cursor record_key2 = aws_byte_cursor_from_c_str("key2");
    struct aws_mqtt_rr_client_operation *operation2 =
        s_create_streaming_operation(&fixture, record_key2, topic_filter1);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);
    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key2, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key2, AWS_ARRAY_SIZE(expected_events), expected_events));

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    struct aws_byte_cursor payload3 = aws_byte_cursor_from_c_str("Payload3");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);
    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key2, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
        {
            payload3,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key1, 2, expected_publishes));
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key2, 2, expected_publishes));

    // close the first, wait for terminate
    aws_mqtt_rr_client_operation_release(operation1);
    s_rrc_wait_on_streaming_termination(&fixture, record_key1);

    // publish again
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload3));

    // verify second operation got the new publish
    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key2, 3);
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key2, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    // verify first operation did not
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key1, 2, expected_publishes));

    aws_mqtt_rr_client_operation_release(operation2);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_streaming_operation_success_overlapping, s_rrc_streaming_operation_success_overlapping_fn)

/*
 * Variant of the simple test where we start the protocol client offline.  In addition to the normal verifies, we also
 * verify nothing happens event wise until we start the client.
 */
static int s_rrc_streaming_operation_success_starting_offline_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(
        s_init_fixture_streaming_operation_success(&fixture, &client_test_options, allocator, NULL, NULL, RRCP_MQTT5));

    /* stop and start the underlying client */
    aws_mqtt5_client_stop(fixture.client_test_fixture.mqtt5_test_fixture.client, NULL, NULL);
    aws_wait_for_stopped_lifecycle_event(&fixture.client_test_fixture.mqtt5_test_fixture);

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    /* wait a while (longer than request timeout) to see if anything happens */
    aws_thread_current_sleep(aws_timestamp_convert(3, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    /* fails the test if any subscription events have been emitted */
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(&fixture, record_key1, 0, NULL));

    /* start the protocol client */
    aws_mqtt5_client_start(fixture.client_test_fixture.mqtt5_test_fixture.client);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_streaming_operation_success_starting_offline, s_rrc_streaming_operation_success_starting_offline_fn)

static void s_rrc_force_clean_session_config(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->client_options.session_behavior = AWS_MQTT5_CSBT_CLEAN;
}

/*
 * Verifies that a streaming operation recovers properly from a clean session resumption (by resubscribing), emitting
 * the proper subscription events, and receiving expected publishes once the subscription is re-established.
 */
static int s_rrc_streaming_operation_clean_session_reestablish_subscription_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture, &client_test_options, allocator, s_rrc_force_clean_session_config, NULL, RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
        {
            .status = ARRSSET_SUBSCRIPTION_LOST,
            .error_code = AWS_ERROR_SUCCESS,
        },
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(&fixture, record_key1, 1, expected_events));

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key1, 1, expected_publishes));

    /* stop and start the underlying client, this will force a resubscribe since it's a clean session */
    aws_mqtt5_client_stop(fixture.client_test_fixture.mqtt5_test_fixture.client, NULL, NULL);
    aws_wait_for_stopped_lifecycle_event(&fixture.client_test_fixture.mqtt5_test_fixture);

    aws_mqtt5_client_start(fixture.client_test_fixture.mqtt5_test_fixture.client);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 3);
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(&fixture, record_key1, 3, expected_events));

    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key1, 2, expected_publishes));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_streaming_operation_clean_session_reestablish_subscription,
    s_rrc_streaming_operation_clean_session_reestablish_subscription_fn)

static void s_rrc_force_resume_session_config(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_CONNECT] =
        aws_mqtt5_mock_server_handle_connect_honor_session_unconditional;
    client_test_options->client_options.session_behavior = AWS_MQTT5_CSBT_REJOIN_ALWAYS;
}

/*
 * Variant of the clean session test where instead we always resume a session.  Verify we don't get subscription
 * lost/established events afterwards and can still receive messages.
 */
static int s_rrc_streaming_operation_resume_session_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture, &client_test_options, allocator, s_rrc_force_resume_session_config, NULL, RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(&fixture, record_key1, 1, expected_events));

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key1, 1, expected_publishes));

    /* stop and start the underlying client */
    aws_mqtt5_client_stop(fixture.client_test_fixture.mqtt5_test_fixture.client, NULL, NULL);
    aws_wait_for_stopped_lifecycle_event(&fixture.client_test_fixture.mqtt5_test_fixture);

    aws_mqtt5_client_start(fixture.client_test_fixture.mqtt5_test_fixture.client);

    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key1, 2, expected_publishes));

    /* check events after the publish has completed, that shows nothing happened subscription-wise */
    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(&fixture, record_key1, 1, expected_events));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_streaming_operation_resume_session, s_rrc_streaming_operation_resume_session_fn)

struct rrc_subscribe_handler_context {
    struct aws_rr_client_test_fixture *fixture;

    size_t subscribes_received;
};

static enum aws_mqtt5_suback_reason_code s_rrc_success_suback_rcs[] = {
    AWS_MQTT5_SARC_GRANTED_QOS_1,
};

int s_handle_subscribe_with_initial_timeout(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct rrc_subscribe_handler_context *context = user_data;

    bool should_respond = false;
    aws_mutex_lock(&context->fixture->lock);
    should_respond = context->subscribes_received > 0;
    ++context->subscribes_received;
    aws_mutex_unlock(&context->fixture->lock);

    if (!should_respond) {
        return AWS_OP_SUCCESS;
    }

    struct aws_mqtt5_packet_subscribe_view *subscribe_packet = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_packet->packet_id,
        .reason_code_count = 1,
        .reason_codes = s_rrc_success_suback_rcs,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

static void s_rrc_initial_subscribe_timeout_config(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_handle_subscribe_with_initial_timeout;
}

/*
 * Variant of the basic success test where the first subscribe is ignored, causing it to timeout.  Verify the
 * client sends a second subscribe (which succeeds) after which everything is fine.
 */
static int s_rrc_streaming_operation_first_subscribe_times_out_resub_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;

    struct rrc_subscribe_handler_context subscribe_context = {
        .fixture = &fixture,
        .subscribes_received = 0,
    };
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture,
        &client_test_options,
        allocator,
        s_rrc_initial_subscribe_timeout_config,
        &subscribe_context,
        RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    // verify we ignored the first subscribe, triggering a second
    aws_mutex_lock(&fixture.lock);
    ASSERT_INT_EQUALS(2, subscribe_context.subscribes_received);
    aws_mutex_unlock(&fixture.lock);

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_streaming_operation_first_subscribe_times_out_resub_succeeds,
    s_rrc_streaming_operation_first_subscribe_times_out_resub_succeeds_fn)

static enum aws_mqtt5_suback_reason_code s_rrc_retryable_suback_rcs[] = {
    AWS_MQTT5_SARC_UNSPECIFIED_ERROR,
};

int s_handle_subscribe_with_initial_retryable_failure(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct rrc_subscribe_handler_context *context = user_data;

    bool should_succeed = false;
    aws_mutex_lock(&context->fixture->lock);
    should_succeed = context->subscribes_received > 0;
    ++context->subscribes_received;
    aws_mutex_unlock(&context->fixture->lock);

    struct aws_mqtt5_packet_subscribe_view *subscribe_packet = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_packet->packet_id,
        .reason_code_count = 1,
    };

    if (should_succeed) {
        suback_view.reason_codes = s_rrc_success_suback_rcs;
    } else {
        suback_view.reason_codes = s_rrc_retryable_suback_rcs;
    }

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

static void s_rrc_initial_subscribe_retryable_failure_config(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_handle_subscribe_with_initial_retryable_failure;
}

/*
 * Variant of the basic success test where the first subscribe triggers a retryable suback failure.  Verify the
 * client sends a second subscribe (which succeeds) after which everything is fine.
 */
static int s_rrc_streaming_operation_first_subscribe_retryable_failure_resub_succeeds_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;

    struct rrc_subscribe_handler_context subscribe_context = {
        .fixture = &fixture,
        .subscribes_received = 0,
    };
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture,
        &client_test_options,
        allocator,
        s_rrc_initial_subscribe_retryable_failure_config,
        &subscribe_context,
        RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    // verify we ignored the first subscribe, triggering a second
    aws_mutex_lock(&fixture.lock);
    ASSERT_INT_EQUALS(2, subscribe_context.subscribes_received);
    aws_mutex_unlock(&fixture.lock);

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_streaming_operation_first_subscribe_retryable_failure_resub_succeeds,
    s_rrc_streaming_operation_first_subscribe_retryable_failure_resub_succeeds_fn)

static enum aws_mqtt5_suback_reason_code s_rrc_unretryable_suback_rcs[] = {
    AWS_MQTT5_SARC_NOT_AUTHORIZED,
};

int s_handle_subscribe_with_terminal_failure(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct rrc_subscribe_handler_context *context = user_data;

    aws_mutex_lock(&context->fixture->lock);
    ++context->subscribes_received;
    aws_mutex_unlock(&context->fixture->lock);

    struct aws_mqtt5_packet_subscribe_view *subscribe_packet = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_packet->packet_id,
        .reason_code_count = 1,
        .reason_codes = s_rrc_unretryable_suback_rcs,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

static void s_rrc_subscribe_terminal_failure_config(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_handle_subscribe_with_terminal_failure;
}

/*
 * Failure variant where the subscribe triggers a non-retryable suback failure.  Verify the
 * operation gets halted.
 */
static int s_rrc_streaming_operation_subscribe_unretryable_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;

    struct rrc_subscribe_handler_context subscribe_context = {
        .fixture = &fixture,
        .subscribes_received = 0,
    };
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture,
        &client_test_options,
        allocator,
        s_rrc_subscribe_terminal_failure_config,
        &subscribe_context,
        RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    // wait an extra amount just for fun
    aws_thread_current_sleep(aws_timestamp_convert(2, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_HALTED,
            .error_code = AWS_ERROR_MQTT_REQUEST_RESPONSE_SUBSCRIBE_FAILURE,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    aws_mutex_lock(&fixture.lock);
    ASSERT_INT_EQUALS(1, subscribe_context.subscribes_received);
    aws_mutex_unlock(&fixture.lock);

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_streaming_operation_subscribe_unretryable_failure,
    s_rrc_streaming_operation_subscribe_unretryable_failure_fn)

static void s_rrc_unsubscribe_success_config(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success;
}

/*
 * Multi-operation variant where we exceed the streaming subscription budget, release everything and then verify
 * we can successfully establish a new streaming operation after everything cleans up.
 */
static int s_rrc_streaming_operation_failure_exceeds_subscription_budget_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture, &client_test_options, allocator, s_rrc_unsubscribe_success_config, NULL, RRCP_MQTT5));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1/+");
    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/1/abc");
    struct aws_mqtt_rr_client_operation *operation1 =
        s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    struct aws_byte_cursor record_key2 = aws_byte_cursor_from_c_str("key2");
    struct aws_byte_cursor topic_filter2 = aws_byte_cursor_from_c_str("/topic/2/+");
    struct aws_byte_cursor topic2 = aws_byte_cursor_from_c_str("/topic/2/def");
    struct aws_mqtt_rr_client_operation *operation2 =
        s_create_streaming_operation(&fixture, record_key2, topic_filter2);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);
    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key2, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_success_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_success_events), expected_success_events));

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_failure_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_HALTED,
            .error_code = AWS_ERROR_MQTT_REQUEST_RESPONSE_NO_SUBSCRIPTION_CAPACITY,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key2, AWS_ARRAY_SIZE(expected_success_events), expected_failure_events));

    // two publishes on the mqtt client that get reflected into our subscription topic1
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic1,
        },
        {
            payload2,
            topic1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(&fixture, record_key1, 2, expected_publishes));

    // close the first, wait for terminate
    aws_mqtt_rr_client_operation_release(operation1);
    s_rrc_wait_on_streaming_termination(&fixture, record_key1);

    // close the second, wait for terminate
    aws_mqtt_rr_client_operation_release(operation2);
    s_rrc_wait_on_streaming_termination(&fixture, record_key2);

    // let the unsubscribe resolve
    aws_thread_current_sleep(aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL));

    // make a third using topic filter 2
    struct aws_byte_cursor record_key3 = aws_byte_cursor_from_c_str("key3");
    struct aws_mqtt_rr_client_operation *operation3 = s_create_streaming_operation(&fixture, record_key3, topic2);

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key3, 1);
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key3, AWS_ARRAY_SIZE(expected_success_events), expected_success_events));

    // publish again
    struct aws_byte_cursor payload3 = aws_byte_cursor_from_c_str("payload3");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic2, payload3));

    // verify third operation got the new publish
    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key3, 1);

    struct aws_rr_client_fixture_publish_message_view third_expected_publishes[] = {
        {
            payload3,
            topic2,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key3, AWS_ARRAY_SIZE(third_expected_publishes), third_expected_publishes));

    aws_mqtt_rr_client_operation_release(operation3);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_streaming_operation_failure_exceeds_subscription_budget,
    s_rrc_streaming_operation_failure_exceeds_subscription_budget_fn)

static int s_submit_request_operation_from_prefix(
    struct aws_rr_client_test_fixture *fixture,
    struct aws_byte_cursor record_key,
    struct aws_byte_cursor prefix) {
    char accepted_path[128];
    char rejected_path[128];
    char subscription_topic_filter[128];
    char publish_topic[128];

    snprintf(accepted_path, AWS_ARRAY_SIZE(accepted_path), PRInSTR "/accepted", AWS_BYTE_CURSOR_PRI(prefix));
    snprintf(rejected_path, AWS_ARRAY_SIZE(rejected_path), PRInSTR "/rejected", AWS_BYTE_CURSOR_PRI(prefix));
    snprintf(
        subscription_topic_filter,
        AWS_ARRAY_SIZE(subscription_topic_filter),
        PRInSTR "/+",
        AWS_BYTE_CURSOR_PRI(prefix));
    snprintf(publish_topic, AWS_ARRAY_SIZE(publish_topic), PRInSTR "/get", AWS_BYTE_CURSOR_PRI(prefix));

    struct aws_byte_cursor subscription_topic_filter_cursor = aws_byte_cursor_from_c_str(subscription_topic_filter);

    char correlation_token[128];
    struct aws_byte_buf correlation_token_buf =
        aws_byte_buf_from_empty_array(correlation_token, AWS_ARRAY_SIZE(correlation_token));

    struct aws_uuid uuid;
    aws_uuid_init(&uuid);
    aws_uuid_to_str(&uuid, &correlation_token_buf);

    struct aws_mqtt_request_operation_response_path response_paths[] = {
        {
            .topic = aws_byte_cursor_from_c_str(accepted_path),
            .correlation_token_json_path = aws_byte_cursor_from_c_str("client_token"),
        },
        {
            .topic = aws_byte_cursor_from_c_str(rejected_path),
            .correlation_token_json_path = aws_byte_cursor_from_c_str("client_token"),
        },
    };

    struct aws_rr_client_fixture_request_response_record *record =
        s_rrc_fixture_add_request_record(fixture, record_key);

    struct aws_mqtt_request_operation_options request = {
        .subscription_topic_filters = &subscription_topic_filter_cursor,
        .subscription_topic_filter_count = 1,
        .response_paths = response_paths,
        .response_path_count = AWS_ARRAY_SIZE(response_paths),
        .publish_topic = aws_byte_cursor_from_c_str(publish_topic),
        .serialized_request = aws_byte_cursor_from_c_str("{}"),
        .correlation_token = aws_byte_cursor_from_buf(&correlation_token_buf),
        .completion_callback = s_rrc_fixture_request_completion_callback,
        .user_data = record,
    };

    return aws_mqtt_request_response_client_submit_request(fixture->rr_client, &request);
}

/*
 * Configure server to only respond to subscribes that match a streaming filter.  Submit a couple of
 * request-response operations ahead of a streaming operation.  Verify they both time out and that the streaming
 * operation successfully subscribes and receives publishes.
 */
static int s_rrc_streaming_operation_success_delayed_by_request_operations_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture, &client_test_options, allocator, s_rrc_unsubscribe_success_config, NULL, RRCP_MQTT5));

    struct aws_byte_cursor request_key1 = aws_byte_cursor_from_c_str("requestkey1");
    struct aws_byte_cursor request_key2 = aws_byte_cursor_from_c_str("requestkey2");

    ASSERT_SUCCESS(s_submit_request_operation_from_prefix(&fixture, request_key1, request_key1));
    ASSERT_SUCCESS(s_submit_request_operation_from_prefix(&fixture, request_key2, request_key2));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    s_rrc_wait_on_request_completion(&fixture, request_key1);
    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, request_key1, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));
    s_rrc_wait_on_request_completion(&fixture, request_key2);
    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, request_key2, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_streaming_operation_success_delayed_by_request_operations,
    s_rrc_streaming_operation_success_delayed_by_request_operations_fn)

/*
 * Variant of previous test where we sandwich the streaming operation by multiple request response operations and
 * verify all request-response operations fail with a timeout.
 */
static int s_rrc_streaming_operation_success_sandwiched_by_request_operations_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_streaming_operation_success(
        &fixture, &client_test_options, allocator, s_rrc_unsubscribe_success_config, NULL, RRCP_MQTT5));

    struct aws_byte_cursor request_key1 = aws_byte_cursor_from_c_str("requestkey1");
    struct aws_byte_cursor request_key2 = aws_byte_cursor_from_c_str("requestkey2");
    struct aws_byte_cursor request_key3 = aws_byte_cursor_from_c_str("requestkey3");
    struct aws_byte_cursor request_key4 = aws_byte_cursor_from_c_str("requestkey4");

    ASSERT_SUCCESS(s_submit_request_operation_from_prefix(&fixture, request_key1, request_key1));
    ASSERT_SUCCESS(s_submit_request_operation_from_prefix(&fixture, request_key2, request_key2));

    struct aws_byte_cursor record_key1 = aws_byte_cursor_from_c_str("key1");
    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/1");
    struct aws_mqtt_rr_client_operation *operation = s_create_streaming_operation(&fixture, record_key1, topic_filter1);

    ASSERT_SUCCESS(s_submit_request_operation_from_prefix(&fixture, request_key3, request_key3));
    ASSERT_SUCCESS(s_submit_request_operation_from_prefix(&fixture, request_key4, request_key4));

    s_rrc_wait_on_request_completion(&fixture, request_key1);
    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, request_key1, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));
    s_rrc_wait_on_request_completion(&fixture, request_key2);
    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, request_key2, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));

    s_rrc_wait_for_n_streaming_subscription_events(&fixture, record_key1, 1);

    struct aws_rr_client_fixture_streaming_record_subscription_event expected_events[] = {
        {
            .status = ARRSSET_SUBSCRIPTION_ESTABLISHED,
            .error_code = AWS_ERROR_SUCCESS,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_record_subscription_events(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_events), expected_events));

    // two publishes on the mqtt client that get reflected into our subscription topic
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");
    struct aws_byte_cursor payload2 = aws_byte_cursor_from_c_str("Payload2");
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload1));
    ASSERT_SUCCESS(s_rrc_protocol_client_publish(&fixture, topic_filter1, payload2));

    s_rrc_wait_for_n_streaming_publishes(&fixture, record_key1, 2);

    struct aws_rr_client_fixture_publish_message_view expected_publishes[] = {
        {
            payload1,
            topic_filter1,
        },
        {
            payload2,
            topic_filter1,
        },
    };
    ASSERT_SUCCESS(s_rrc_verify_streaming_publishes(
        &fixture, record_key1, AWS_ARRAY_SIZE(expected_publishes), expected_publishes));

    s_rrc_wait_on_request_completion(&fixture, request_key3);
    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, request_key3, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));
    s_rrc_wait_on_request_completion(&fixture, request_key4);
    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, request_key4, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));

    aws_mqtt_rr_client_operation_release(operation);

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_streaming_operation_success_sandwiched_by_request_operations,
    s_rrc_streaming_operation_success_sandwiched_by_request_operations_fn)

enum rrc_publish_handler_directive_type {
    RRC_PHDT_SUCCESS,
    RRC_PHDT_FAILURE_PUBACK_REASON_CODE,
    RRC_PHDT_FAILURE_BAD_PAYLOAD_FORMAT,
    RRC_PHDT_FAILURE_MISSING_CORRELATION_TOKEN,
    RRC_PHDT_FAILURE_BAD_CORRELATION_TOKEN_TYPE,
    RRC_PHDT_FAILURE_MISMATCHED_CORRELATION_TOKEN,
};

int aws_mqtt5_mock_server_handle_publish_json_request(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)user_data;

    struct aws_json_value *payload_json = NULL;
    struct aws_json_value *response_json = NULL;
    struct aws_allocator *allocator = connection->allocator;

    struct aws_mqtt5_packet_publish_view *publish_view = packet;

    /* unmarshal the payload as json */
    payload_json = aws_json_value_new_from_string(allocator, publish_view->payload);
    AWS_FATAL_ASSERT(payload_json != NULL);

    /* 'topic' field is where we should publish to */
    struct aws_json_value *topic_value =
        aws_json_value_get_from_object(payload_json, aws_byte_cursor_from_c_str("topic"));
    AWS_FATAL_ASSERT(topic_value != NULL && aws_json_value_is_string(topic_value));

    struct aws_byte_cursor topic;
    AWS_ZERO_STRUCT(topic);
    aws_json_value_get_string(topic_value, &topic);

    /* 'token' field is the correlation token we should reflect */
    struct aws_byte_cursor token;
    AWS_ZERO_STRUCT(token);

    struct aws_json_value *token_value =
        aws_json_value_get_from_object(payload_json, aws_byte_cursor_from_c_str("token"));
    if (token_value != NULL) {
        AWS_FATAL_ASSERT(aws_json_value_is_string(token_value));
        aws_json_value_get_string(token_value, &token);
    }

    /* 'reflection' field is an optional field we should reflect.  Used to ensure proper correlation on requests that
     * don't use correlation tokens */
    struct aws_byte_cursor reflection;
    AWS_ZERO_STRUCT(reflection);

    struct aws_json_value *reflection_value =
        aws_json_value_get_from_object(payload_json, aws_byte_cursor_from_c_str("reflection"));
    if (reflection_value != NULL) {
        AWS_FATAL_ASSERT(aws_json_value_is_string(reflection_value));
        aws_json_value_get_string(reflection_value, &reflection);
    }

    /* 'directive' field indicates how the response handler should behave */
    struct aws_json_value *directive_value =
        aws_json_value_get_from_object(payload_json, aws_byte_cursor_from_c_str("directive"));
    AWS_FATAL_ASSERT(directive_value != NULL && aws_json_value_is_number(directive_value));

    double raw_directive_value = 0;
    aws_json_value_get_number(directive_value, &raw_directive_value);
    enum rrc_publish_handler_directive_type directive = (int)raw_directive_value;

    /* send a PUBACK? */
    if (publish_view->qos == AWS_MQTT5_QOS_AT_LEAST_ONCE) {
        struct aws_mqtt5_packet_puback_view puback_view = {
            .packet_id = publish_view->packet_id,
            .reason_code = (directive == RRC_PHDT_FAILURE_PUBACK_REASON_CODE) ? AWS_MQTT5_PARC_NOT_AUTHORIZED
                                                                              : AWS_MQTT5_PARC_SUCCESS,
        };

        if (aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBACK, &puback_view)) {
            return AWS_OP_ERR;
        }
    }

    if (directive == RRC_PHDT_FAILURE_PUBACK_REASON_CODE) {
        goto done;
    }

    /* build the json response blob */
    char response_buffer[512];
    switch (directive) {
        case RRC_PHDT_FAILURE_BAD_PAYLOAD_FORMAT:
            snprintf(
                response_buffer,
                AWS_ARRAY_SIZE(response_buffer),
                "<token>" PRInSTR "</token>",
                AWS_BYTE_CURSOR_PRI(token));
            break;
        case RRC_PHDT_FAILURE_MISSING_CORRELATION_TOKEN:
            snprintf(
                response_buffer,
                AWS_ARRAY_SIZE(response_buffer),
                "{\"wrongfield\":\"" PRInSTR "\"}",
                AWS_BYTE_CURSOR_PRI(token));
            break;
        case RRC_PHDT_FAILURE_BAD_CORRELATION_TOKEN_TYPE:
            snprintf(response_buffer, AWS_ARRAY_SIZE(response_buffer), "{\"token\":5}");
            break;
        case RRC_PHDT_FAILURE_MISMATCHED_CORRELATION_TOKEN:
            snprintf(response_buffer, AWS_ARRAY_SIZE(response_buffer), "{\"token\":\"NotTheToken\"}");
            break;
        default: {
            int bytes_used = snprintf(response_buffer, AWS_ARRAY_SIZE(response_buffer), "{");
            if (token.len > 0) {
                bytes_used += snprintf(
                    response_buffer + bytes_used,
                    AWS_ARRAY_SIZE(response_buffer) - bytes_used,
                    "\"token\":\"" PRInSTR "\"",
                    AWS_BYTE_CURSOR_PRI(token));
            }
            if (reflection.len > 0) {
                if (token.len > 0) {
                    bytes_used +=
                        snprintf(response_buffer + bytes_used, AWS_ARRAY_SIZE(response_buffer) - bytes_used, ",");
                }
                bytes_used += snprintf(
                    response_buffer + bytes_used,
                    AWS_ARRAY_SIZE(response_buffer) - bytes_used,
                    "\"reflection\":\"" PRInSTR "\"",
                    AWS_BYTE_CURSOR_PRI(reflection));
            }
            snprintf(response_buffer + bytes_used, AWS_ARRAY_SIZE(response_buffer) - bytes_used, "}");
            break;
        }
    }

    /* build the response publish packet */
    struct aws_mqtt5_packet_publish_view response_publish_view = {
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .topic = topic,
        .payload = aws_byte_cursor_from_c_str(response_buffer),
    };

    aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_PUBLISH, &response_publish_view);

done:

    aws_json_value_destroy(payload_json);
    aws_json_value_destroy(response_json);

    return AWS_OP_SUCCESS;
}

static int s_init_fixture_request_operation_success(
    struct aws_rr_client_test_fixture *fixture,
    struct mqtt5_client_test_options *client_test_options,
    struct aws_allocator *allocator,
    modify_fixture_options_fn *config_modifier,
    void *user_data) {

    aws_mqtt5_client_test_init_default_options(client_test_options);

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        aws_mqtt5_server_send_suback_on_subscribe;
    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_PUBLISH] =
        aws_mqtt5_mock_server_handle_publish_json_request;
    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_UNSUBSCRIBE] =
        aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success;

    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options client_test_fixture_options = {
        .client_options = &client_test_options->client_options,
        .server_function_table = &client_test_options->server_function_table,
        .mock_server_user_data = user_data,
    };

    struct aws_mqtt_request_response_client_options rr_client_options = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = 2,
    };

    if (config_modifier != NULL) {
        (*config_modifier)(&rr_client_options, client_test_options);
    }

    ASSERT_SUCCESS(s_aws_rr_client_test_fixture_init_from_mqtt5(
        fixture, allocator, &rr_client_options, &client_test_fixture_options, NULL));

    return AWS_OP_SUCCESS;
}

static int s_rrc_test_submit_test_request(
    struct aws_rr_client_test_fixture *fixture,
    enum rrc_publish_handler_directive_type test_directive,
    const char *topic_prefix,
    struct aws_byte_cursor record_key,
    const char *response_topic,
    const char *token,
    const char *reflection,
    bool is_multi_subscribe) {

    char path1_buffer[128];
    snprintf(path1_buffer, AWS_ARRAY_SIZE(path1_buffer), "%s/accepted", topic_prefix);
    char path2_buffer[128];
    snprintf(path2_buffer, AWS_ARRAY_SIZE(path2_buffer), "%s/rejected", topic_prefix);

    struct aws_byte_cursor token_path = aws_byte_cursor_from_c_str("token");
    if (token == NULL) {
        AWS_ZERO_STRUCT(token_path);
    }

    struct aws_mqtt_request_operation_response_path response_paths[] = {
        {
            .topic = aws_byte_cursor_from_c_str(path1_buffer),
            .correlation_token_json_path = token_path,
        },
        {
            .topic = aws_byte_cursor_from_c_str(path2_buffer),
            .correlation_token_json_path = token_path,
        },
    };

    char subscription_buffer[128];
    snprintf(subscription_buffer, AWS_ARRAY_SIZE(subscription_buffer), "%s/+", topic_prefix);
    struct aws_byte_cursor subscription_buffer_cursor = aws_byte_cursor_from_c_str(subscription_buffer);

    struct aws_byte_cursor *subscriptions = &subscription_buffer_cursor;
    size_t subscription_count = 1;

    struct aws_byte_cursor multi_subs[] = {
        aws_byte_cursor_from_c_str(path1_buffer),
        aws_byte_cursor_from_c_str(path2_buffer),
    };
    if (is_multi_subscribe) {
        subscriptions = multi_subs;
        subscription_count = 2;
    }

    char publish_topic_buffer[128];
    snprintf(publish_topic_buffer, AWS_ARRAY_SIZE(publish_topic_buffer), "%s/publish", topic_prefix);

    char request_buffer[512];
    int used_bytes = snprintf(
        request_buffer,
        AWS_ARRAY_SIZE(request_buffer),
        "{\"topic\":\"%s\",\"directive\":%d",
        response_topic,
        (int)test_directive);

    if (token != NULL) {
        used_bytes += snprintf(
            request_buffer + used_bytes, AWS_ARRAY_SIZE(request_buffer) - used_bytes, ",\"token\":\"%s\"", token);
    }

    if (reflection != NULL) {
        used_bytes += snprintf(
            request_buffer + used_bytes,
            AWS_ARRAY_SIZE(request_buffer) - used_bytes,
            ",\"reflection\":\"%s\"",
            reflection);
    }

    snprintf(request_buffer + used_bytes, AWS_ARRAY_SIZE(request_buffer) - used_bytes, "}");

    struct aws_mqtt_request_operation_options request = {
        .subscription_topic_filters = subscriptions,
        .subscription_topic_filter_count = subscription_count,
        .response_paths = response_paths,
        .response_path_count = AWS_ARRAY_SIZE(response_paths),
        .publish_topic = aws_byte_cursor_from_c_str(publish_topic_buffer),
        .serialized_request = aws_byte_cursor_from_c_str(request_buffer),
        .correlation_token = aws_byte_cursor_from_c_str(token),
    };

    struct aws_rr_client_fixture_request_response_record *record =
        s_rrc_fixture_add_request_record(fixture, record_key);

    request.completion_callback = s_rrc_fixture_request_completion_callback;
    request.user_data = record;

    return aws_mqtt_request_response_client_submit_request(fixture->rr_client, &request);
}

static int s_rrc_request_response_success_response_path_accepted_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_SUCCESS, "test", record_key, "test/accepted", "token1", NULL, false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    struct aws_byte_cursor expected_response_topic = aws_byte_cursor_from_c_str("test/accepted");
    struct aws_byte_cursor expected_payload = aws_byte_cursor_from_c_str("{\"token\":\"token1\"}");
    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_SUCCESS, &expected_response_topic, &expected_payload));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_success_response_path_accepted,
    s_rrc_request_response_success_response_path_accepted_fn)

static int s_rrc_request_response_multi_sub_success_response_path_accepted_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_SUCCESS, "test", record_key, "test/accepted", "token1", NULL, true));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    struct aws_byte_cursor expected_response_topic = aws_byte_cursor_from_c_str("test/accepted");
    struct aws_byte_cursor expected_payload = aws_byte_cursor_from_c_str("{\"token\":\"token1\"}");
    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_SUCCESS, &expected_response_topic, &expected_payload));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_multi_sub_success_response_path_accepted,
    s_rrc_request_response_multi_sub_success_response_path_accepted_fn)

static int s_rrc_request_response_success_response_path_rejected_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_SUCCESS, "test", record_key, "test/rejected", "token5", NULL, false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    struct aws_byte_cursor expected_response_topic = aws_byte_cursor_from_c_str("test/rejected");
    struct aws_byte_cursor expected_payload = aws_byte_cursor_from_c_str("{\"token\":\"token5\"}");
    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_SUCCESS, &expected_response_topic, &expected_payload));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_success_response_path_rejected,
    s_rrc_request_response_success_response_path_rejected_fn)

static int s_rrc_request_response_multi_sub_success_response_path_rejected_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_SUCCESS, "test", record_key, "test/rejected", "token5", NULL, true));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    struct aws_byte_cursor expected_response_topic = aws_byte_cursor_from_c_str("test/rejected");
    struct aws_byte_cursor expected_payload = aws_byte_cursor_from_c_str("{\"token\":\"token5\"}");
    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_SUCCESS, &expected_response_topic, &expected_payload));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_multi_sub_success_response_path_rejected,
    s_rrc_request_response_multi_sub_success_response_path_rejected_fn)

static void s_fail_all_subscribes_config_modifier_fn(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_handle_subscribe_with_terminal_failure;
}

static int s_rrc_request_response_subscribe_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;

    struct rrc_subscribe_handler_context subscribe_context = {
        .fixture = &fixture,
        .subscribes_received = 0,
    };
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(
        &fixture, &client_test_options, allocator, s_fail_all_subscribes_config_modifier_fn, &subscribe_context));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_SUCCESS, "test", record_key, "test/accepted", "token1", NULL, false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_MQTT_REQUEST_RESPONSE_SUBSCRIBE_FAILURE, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_request_response_subscribe_failure, s_rrc_request_response_subscribe_failure_fn)

static enum aws_mqtt5_suback_reason_code s_rrc_usuback_success_rcs[] = {
    AWS_MQTT5_SARC_GRANTED_QOS_1,
};

int s_handle_second_subscribe_with_failure(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data) {
    (void)packet;

    struct rrc_subscribe_handler_context *context = user_data;

    size_t subscribes_received = 0;

    aws_mutex_lock(&context->fixture->lock);
    ++context->subscribes_received;
    subscribes_received = context->subscribes_received;
    aws_mutex_unlock(&context->fixture->lock);

    struct aws_mqtt5_packet_subscribe_view *subscribe_packet = packet;

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = subscribe_packet->packet_id,
        .reason_code_count = 1,
        .reason_codes = (subscribes_received == 1) ? s_rrc_usuback_success_rcs : s_rrc_unretryable_suback_rcs,
    };

    return aws_mqtt5_mock_server_send_packet(connection, AWS_MQTT5_PT_SUBACK, &suback_view);
}

static void s_fail_second_subscribe_config_modifier_fn(
    struct aws_mqtt_request_response_client_options *fixture_options,
    struct mqtt5_client_test_options *client_test_options) {
    (void)fixture_options;

    client_test_options->server_function_table.packet_handlers[AWS_MQTT5_PT_SUBSCRIBE] =
        s_handle_second_subscribe_with_failure;
}

static int s_rrc_request_response_multi_subscribe_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;

    struct rrc_subscribe_handler_context subscribe_context = {
        .fixture = &fixture,
        .subscribes_received = 0,
    };
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(
        &fixture, &client_test_options, allocator, s_fail_second_subscribe_config_modifier_fn, &subscribe_context));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_SUCCESS, "test", record_key, "test/accepted", "token1", NULL, true));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_MQTT_REQUEST_RESPONSE_SUBSCRIBE_FAILURE, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_request_response_multi_subscribe_failure, s_rrc_request_response_multi_subscribe_failure_fn)

static int s_rrc_request_response_failure_puback_reason_code_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_FAILURE_PUBACK_REASON_CODE, "test", record_key, "test/accepted", "token1", NULL, false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_MQTT_REQUEST_RESPONSE_PUBLISH_FAILURE, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_request_response_failure_puback_reason_code, s_rrc_request_response_failure_puback_reason_code_fn)

static int s_rrc_request_response_failure_invalid_payload_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_FAILURE_BAD_PAYLOAD_FORMAT, "test", record_key, "test/accepted", "token1", NULL, false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, record_key, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_request_response_failure_invalid_payload, s_rrc_request_response_failure_invalid_payload_fn)

static int s_rrc_request_response_failure_missing_correlation_token_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture,
        RRC_PHDT_FAILURE_MISSING_CORRELATION_TOKEN,
        "test",
        record_key,
        "test/accepted",
        "token1",
        NULL,
        false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, record_key, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_failure_missing_correlation_token,
    s_rrc_request_response_failure_missing_correlation_token_fn)

static int s_rrc_request_response_failure_invalid_correlation_token_type_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture,
        RRC_PHDT_FAILURE_BAD_CORRELATION_TOKEN_TYPE,
        "test",
        record_key,
        "test/accepted",
        "token1",
        NULL,
        false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, record_key, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_failure_invalid_correlation_token_type,
    s_rrc_request_response_failure_invalid_correlation_token_type_fn)

static int s_rrc_request_response_failure_non_matching_correlation_token_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture,
        RRC_PHDT_FAILURE_MISMATCHED_CORRELATION_TOKEN,
        "test",
        record_key,
        "test/accepted",
        "token1",
        NULL,
        false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    ASSERT_SUCCESS(
        s_rrc_verify_request_completion(&fixture, record_key, AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, NULL, NULL));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_failure_non_matching_correlation_token,
    s_rrc_request_response_failure_non_matching_correlation_token_fn)

static int s_rrc_request_response_success_empty_correlation_token_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str("testkey");
    ASSERT_SUCCESS(s_rrc_test_submit_test_request(
        &fixture, RRC_PHDT_SUCCESS, "test", record_key, "test/accepted", NULL, NULL, false));

    s_rrc_wait_on_request_completion(&fixture, record_key);

    struct aws_byte_cursor expected_response_topic = aws_byte_cursor_from_c_str("test/accepted");
    struct aws_byte_cursor expected_payload = aws_byte_cursor_from_c_str("{}");
    ASSERT_SUCCESS(s_rrc_verify_request_completion(
        &fixture, record_key, AWS_ERROR_SUCCESS, &expected_response_topic, &expected_payload));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_success_empty_correlation_token,
    s_rrc_request_response_success_empty_correlation_token_fn)

static int s_rrc_request_response_success_empty_correlation_token_sequence_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    for (size_t i = 0; i < 20; ++i) {
        char key_buffer[128];
        snprintf(key_buffer, AWS_ARRAY_SIZE(key_buffer), "testkey%zu", i);
        struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str(key_buffer);

        char prefix_buffer[128];
        snprintf(prefix_buffer, AWS_ARRAY_SIZE(prefix_buffer), "test%zu", i);

        char response_topic_buffer[128];
        snprintf(response_topic_buffer, AWS_ARRAY_SIZE(response_topic_buffer), "test%zu/accepted", i);

        ASSERT_SUCCESS(s_rrc_test_submit_test_request(
            &fixture, RRC_PHDT_SUCCESS, prefix_buffer, record_key, response_topic_buffer, NULL, NULL, false));
    }

    for (size_t i = 0; i < 20; ++i) {
        char key_buffer[128];
        snprintf(key_buffer, AWS_ARRAY_SIZE(key_buffer), "testkey%zu", i);
        struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str(key_buffer);

        char response_topic_buffer[128];
        snprintf(response_topic_buffer, AWS_ARRAY_SIZE(response_topic_buffer), "test%zu/accepted", i);

        s_rrc_wait_on_request_completion(&fixture, record_key);

        struct aws_byte_cursor expected_response_topic = aws_byte_cursor_from_c_str(response_topic_buffer);
        struct aws_byte_cursor expected_payload = aws_byte_cursor_from_c_str("{}");
        ASSERT_SUCCESS(s_rrc_verify_request_completion(
            &fixture, record_key, AWS_ERROR_SUCCESS, &expected_response_topic, &expected_payload));
    }

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrc_request_response_success_empty_correlation_token_sequence,
    s_rrc_request_response_success_empty_correlation_token_sequence_fn)

struct rrc_multi_test_operation {
    const char *prefix;
    const char *token;
    const char *reflection;
};

static int s_do_rrc_operation_sequence_test(
    struct aws_rr_client_test_fixture *fixture,
    size_t operation_count,
    struct rrc_multi_test_operation *operations) {
    for (size_t i = 0; i < operation_count; ++i) {

        struct rrc_multi_test_operation *operation = &operations[i];

        char key_buffer[128];
        snprintf(key_buffer, AWS_ARRAY_SIZE(key_buffer), "testkey%zu", i);
        struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str(key_buffer);

        char response_topic_buffer[128];
        snprintf(response_topic_buffer, AWS_ARRAY_SIZE(response_topic_buffer), "%s/accepted", operation->prefix);

        ASSERT_SUCCESS(s_rrc_test_submit_test_request(
            fixture,
            RRC_PHDT_SUCCESS,
            operation->prefix,
            record_key,
            response_topic_buffer,
            operation->token,
            operation->reflection,
            false));
    }

    for (size_t i = 0; i < operation_count; ++i) {

        struct rrc_multi_test_operation *operation = &operations[i];

        char key_buffer[128];
        snprintf(key_buffer, AWS_ARRAY_SIZE(key_buffer), "testkey%zu", i);
        struct aws_byte_cursor record_key = aws_byte_cursor_from_c_str(key_buffer);

        char response_topic_buffer[128];
        snprintf(response_topic_buffer, AWS_ARRAY_SIZE(response_topic_buffer), "%s/accepted", operation->prefix);

        s_rrc_wait_on_request_completion(fixture, record_key);

        struct aws_byte_cursor expected_response_topic = aws_byte_cursor_from_c_str(response_topic_buffer);
        struct aws_byte_cursor expected_payload;
        AWS_ZERO_STRUCT(expected_payload);
        char payload_buffer[256];
        int bytes_used = snprintf(payload_buffer, AWS_ARRAY_SIZE(payload_buffer), "{");
        if (operation->token != NULL) {
            bytes_used += snprintf(
                payload_buffer + bytes_used,
                AWS_ARRAY_SIZE(payload_buffer) - bytes_used,
                "\"token\":\"%s\"",
                operation->token);
            if (operation->reflection != NULL) {
                bytes_used += snprintf(payload_buffer + bytes_used, AWS_ARRAY_SIZE(payload_buffer) - bytes_used, ",");
            }
        }
        if (operation->reflection != NULL) {
            bytes_used += snprintf(
                payload_buffer + bytes_used,
                AWS_ARRAY_SIZE(payload_buffer) - bytes_used,
                "\"reflection\":\"%s\"",
                operation->reflection);
        }
        snprintf(payload_buffer + bytes_used, AWS_ARRAY_SIZE(payload_buffer) - bytes_used, "}");

        expected_payload = aws_byte_cursor_from_c_str(payload_buffer);
        ASSERT_SUCCESS(s_rrc_verify_request_completion(
            fixture, record_key, AWS_ERROR_SUCCESS, &expected_response_topic, &expected_payload));
    }

    return AWS_OP_SUCCESS;
}

static int s_rrc_request_response_multi_operation_sequence_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt5_client_test_options client_test_options;
    struct aws_rr_client_test_fixture fixture;
    ASSERT_SUCCESS(s_init_fixture_request_operation_success(&fixture, &client_test_options, allocator, NULL, NULL));

    struct rrc_multi_test_operation request_sequence[] = {
        {
            .prefix = "test",
            .token = "token1",
        },
        {
            .prefix = "test",
            .token = "token2",
        },
        {
            .prefix = "test2",
            .token = "token3",
        },
        {
            .prefix = "whatthe",
            .token = "hey",
            .reflection = "something",
        },
        {
            .prefix = "test",
            .token = "token4",
        },
        {
            .prefix = "test2",
            .token = "token5",
        },
        {
            .prefix = "provision",
            .reflection = "provision1",
        },
        {
            .prefix = "provision",
            .reflection = "provision2",
        },
        {
            .prefix = "create-keys-and-cert",
            .reflection = "create-keys1",
        },
        {
            .prefix = "test",
            .token = "token6",
        },
        {
            .prefix = "test2",
            .token = "token7",
        },
        {
            .prefix = "provision",
            .reflection = "provision3",
        },
    };

    ASSERT_SUCCESS(s_do_rrc_operation_sequence_test(&fixture, AWS_ARRAY_SIZE(request_sequence), request_sequence));

    s_aws_rr_client_test_fixture_clean_up(&fixture, false);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrc_request_response_multi_operation_sequence, s_rrc_request_response_multi_operation_sequence_fn)

struct aws_rr_client_fixture_matched_stream_subscription {
    struct aws_allocator *allocator;
    struct aws_byte_buf payload;
    struct aws_byte_buf topic;
    struct aws_byte_buf topic_filter;
};

struct aws_rr_client_fixture_matched_stream_subscription_view {
    struct aws_byte_cursor payload;
    struct aws_byte_cursor topic;
    struct aws_byte_cursor topic_filter;
};

struct aws_rr_client_fixture_matched_request_subscription {
    struct aws_allocator *allocator;
    struct aws_byte_buf payload;
    struct aws_byte_buf topic;
    struct aws_byte_buf topic_filter;
    struct aws_byte_buf token_path;
};

struct aws_rr_client_fixture_matched_request_subscription_view {
    struct aws_byte_cursor payload;
    struct aws_byte_cursor topic;
    struct aws_byte_cursor topic_filter;
    struct aws_byte_cursor token_path;
};

struct aws_rr_client_fixture_subscriptions_matches_record {
    struct aws_allocator *allocator;
    // table: topic_filter -> aws_rr_client_fixture_matched_stream_subscription
    struct aws_hash_table stream_matches;
    // table: topic_filter -> aws_rr_client_fixture_matched_request_subscription
    struct aws_hash_table request_matches;
    size_t stream_matches_count;
    size_t request_matches_count;
};

static void s_aws_rr_client_fixture_stream_subscription_destroy(void *value) {
    struct aws_rr_client_fixture_matched_stream_subscription *matched_subscription = value;
    aws_byte_buf_clean_up(&matched_subscription->payload);
    aws_byte_buf_clean_up(&matched_subscription->topic);
    aws_byte_buf_clean_up(&matched_subscription->topic_filter);
    aws_mem_release(matched_subscription->allocator, matched_subscription);
}

static void s_aws_rr_client_fixture_request_subscription_destroy(void *value) {
    struct aws_rr_client_fixture_matched_request_subscription *matched_subscription = value;
    aws_byte_buf_clean_up(&matched_subscription->payload);
    aws_byte_buf_clean_up(&matched_subscription->topic);
    aws_byte_buf_clean_up(&matched_subscription->topic_filter);
    aws_byte_buf_clean_up(&matched_subscription->token_path);
    aws_mem_release(matched_subscription->allocator, matched_subscription);
}

struct aws_rr_client_fixture_subscriptions_matches_record *s_aws_rr_client_fixture_subscriptions_matches_record_new(
    struct aws_allocator *allocator) {
    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        aws_mem_calloc(allocator, 1, sizeof(struct aws_rr_client_fixture_subscriptions_matches_record));

    record->allocator = allocator;
    aws_hash_table_init(
        &record->stream_matches,
        allocator,
        10,
        aws_hash_byte_cursor_ptr,
        aws_mqtt_byte_cursor_hash_equality,
        NULL,
        s_aws_rr_client_fixture_stream_subscription_destroy);
    record->stream_matches_count = 0;

    aws_hash_table_init(
        &record->request_matches,
        allocator,
        10,
        aws_hash_byte_cursor_ptr,
        aws_mqtt_byte_cursor_hash_equality,
        NULL,
        s_aws_rr_client_fixture_request_subscription_destroy);
    record->request_matches_count = 0;

    return record;
}

void s_aws_rr_client_fixture_subscriptions_macthes_record_delete(
    struct aws_rr_client_fixture_subscriptions_matches_record *record) {
    aws_hash_table_clean_up(&record->stream_matches);
    aws_hash_table_clean_up(&record->request_matches);
    aws_mem_release(record->allocator, record);
}

static int s_rrc_verify_subscriptions_publishes(
    struct aws_rr_client_fixture_subscriptions_matches_record *record,
    size_t expected_stream_matches_count,
    struct aws_rr_client_fixture_matched_stream_subscription_view *expected_stream_matched_subscriptions,
    size_t expected_request_matches_count,
    struct aws_rr_client_fixture_matched_request_subscription_view *expected_request_matched_subscriptions) {

    ASSERT_INT_EQUALS(expected_stream_matches_count, record->stream_matches_count);

    for (size_t i = 0; i < expected_stream_matches_count; ++i) {
        struct aws_rr_client_fixture_matched_stream_subscription_view *expected_matched_subscription =
            &expected_stream_matched_subscriptions[i];

        struct aws_hash_element *element = NULL;
        ASSERT_SUCCESS(
            aws_hash_table_find(&record->stream_matches, &expected_matched_subscription->topic_filter, &element));

        struct aws_rr_client_fixture_matched_stream_subscription *actual_matched_subscription = element->value;

        ASSERT_BIN_ARRAYS_EQUALS(
            expected_matched_subscription->payload.ptr,
            expected_matched_subscription->payload.len,
            actual_matched_subscription->payload.buffer,
            actual_matched_subscription->payload.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_matched_subscription->topic.ptr,
            expected_matched_subscription->topic.len,
            actual_matched_subscription->topic.buffer,
            actual_matched_subscription->topic.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_matched_subscription->topic_filter.ptr,
            expected_matched_subscription->topic_filter.len,
            actual_matched_subscription->topic_filter.buffer,
            actual_matched_subscription->topic_filter.len);
    }

    ASSERT_INT_EQUALS(expected_request_matches_count, record->request_matches_count);

    for (size_t i = 0; i < expected_request_matches_count; ++i) {
        struct aws_rr_client_fixture_matched_request_subscription_view *expected_matched_subscription =
            &expected_request_matched_subscriptions[i];

        struct aws_hash_element *element = NULL;
        ASSERT_SUCCESS(
            aws_hash_table_find(&record->request_matches, &expected_matched_subscription->topic_filter, &element));

        struct aws_rr_client_fixture_matched_request_subscription *actual_matched_subscription = element->value;

        ASSERT_BIN_ARRAYS_EQUALS(
            expected_matched_subscription->payload.ptr,
            expected_matched_subscription->payload.len,
            actual_matched_subscription->payload.buffer,
            actual_matched_subscription->payload.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_matched_subscription->topic.ptr,
            expected_matched_subscription->topic.len,
            actual_matched_subscription->topic.buffer,
            actual_matched_subscription->topic.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_matched_subscription->topic_filter.ptr,
            expected_matched_subscription->topic_filter.len,
            actual_matched_subscription->topic_filter.buffer,
            actual_matched_subscription->topic_filter.len);
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_matched_subscription->token_path.ptr,
            expected_matched_subscription->token_path.len,
            actual_matched_subscription->token_path.buffer,
            actual_matched_subscription->token_path.len);
    }

    return AWS_OP_SUCCESS;
}

static void s_rrs_fixture_on_stream_operation_subscription_match(
    const struct aws_linked_list *operations,
    const struct aws_byte_cursor *topic_filter,
    const struct aws_mqtt_rr_incoming_publish_event *publish_event,
    void *user_data) {

    (void)operations;

    struct aws_rr_client_fixture_subscriptions_matches_record *record = user_data;

    struct aws_rr_client_fixture_matched_stream_subscription *matched_subscription =
        aws_mem_calloc(record->allocator, 1, sizeof(struct aws_rr_client_fixture_matched_stream_subscription));
    matched_subscription->allocator = record->allocator;
    aws_byte_buf_init_copy_from_cursor(&matched_subscription->payload, record->allocator, publish_event->payload);
    aws_byte_buf_init_copy_from_cursor(&matched_subscription->topic, record->allocator, publish_event->topic);
    aws_byte_buf_init_copy_from_cursor(&matched_subscription->topic_filter, record->allocator, *topic_filter);

    aws_hash_table_put(&record->stream_matches, topic_filter, matched_subscription, NULL);
    ++record->stream_matches_count;
}

static void s_rrs_fixture_on_request_operation_subscription_match(
    struct aws_rr_response_path_entry *entry,
    const struct aws_mqtt_rr_incoming_publish_event *publish_event,
    void *user_data) {
    (void)entry;
    (void)publish_event;

    struct aws_rr_client_fixture_subscriptions_matches_record *record = user_data;

    struct aws_rr_client_fixture_matched_request_subscription *matched_subscription =
        aws_mem_calloc(record->allocator, 1, sizeof(struct aws_rr_client_fixture_matched_request_subscription));
    matched_subscription->allocator = record->allocator;
    aws_byte_buf_init_copy_from_cursor(&matched_subscription->payload, record->allocator, publish_event->payload);
    aws_byte_buf_init_copy_from_cursor(&matched_subscription->topic, record->allocator, publish_event->topic);
    aws_byte_buf_init_copy_from_cursor(&matched_subscription->topic_filter, record->allocator, publish_event->topic);
    aws_byte_buf_init_copy(&matched_subscription->token_path, record->allocator, &entry->correlation_token_json_path);

    aws_hash_table_put(&record->request_matches, &publish_event->topic, matched_subscription, NULL);
    ++record->request_matches_count;
}

static int s_rrs_init_cleanup_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;
    ASSERT_SUCCESS(aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator));
    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrs_init_cleanup, s_rrs_init_cleanup_fn)

static int s_rrs_stream_subscriptions_match_single_level_wildcards_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor topic_filter2 = aws_byte_cursor_from_c_str("topic/123/+");
    struct aws_byte_cursor topic_filter3 = aws_byte_cursor_from_c_str("topic/+/abc");

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter2);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter3);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    struct aws_rr_client_fixture_matched_stream_subscription_view matched_subscriptions[] = {
        {payload1, topic1, topic_filter1},
        {payload1, topic1, topic_filter2},
        {payload1, topic1, topic_filter3},
    };

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        AWS_ARRAY_SIZE(matched_subscriptions),
        matched_subscriptions,
        0, /* expected_request_matches_count */
        NULL));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrs_stream_subscriptions_match_single_level_wildcards,
    s_rrs_stream_subscriptions_match_single_level_wildcards_fn)

static int s_rrs_stream_subscriptions_match_multi_level_wildcards_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor topic_filter2 = aws_byte_cursor_from_c_str("topic/123/#");
    struct aws_byte_cursor topic_filter3 = aws_byte_cursor_from_c_str("topic/#");

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter2);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter3);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    struct aws_rr_client_fixture_matched_stream_subscription_view matched_subscriptions[] = {
        {payload1, topic1, topic_filter1},
        {payload1, topic1, topic_filter2},
        {payload1, topic1, topic_filter3},
    };

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        AWS_ARRAY_SIZE(matched_subscriptions),
        matched_subscriptions,
        0, /* expected_request_matches_count */
        NULL));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrs_stream_subscriptions_match_multi_level_wildcards,
    s_rrs_stream_subscriptions_match_multi_level_wildcards_fn)

/* Adding multiple identical stream subscriptions should add only one record. */
static int s_rrs_stream_subscriptions_add_duplicate_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/123/+");

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    struct aws_rr_client_fixture_matched_stream_subscription_view matched_subscriptions[] = {
        {payload1, topic1, topic_filter1},
    };

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        AWS_ARRAY_SIZE(matched_subscriptions),
        matched_subscriptions,
        0, /* expected_request_matches_count */
        NULL));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrs_stream_subscriptions_add_duplicate, s_rrs_stream_subscriptions_add_duplicate_fn)

static int s_rrs_stream_subscriptions_too_long_publish_topic_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/123/+");

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc/def");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        0, /* expected_stream_matches_count */
        NULL,
        0, /* expected_request_matches_count */
        NULL));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrs_stream_subscriptions_too_long_publish_topic, s_rrs_stream_subscriptions_too_long_publish_topic_fn)

static int s_rrs_stream_subscriptions_too_short_publish_topic_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic_filter1 = aws_byte_cursor_from_c_str("topic/123/+");

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic_filter1);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        0, /* expected_stream_matches_count */
        NULL,
        0, /* expected_request_matches_count */
        NULL));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrs_stream_subscriptions_too_short_publish_topic, s_rrs_stream_subscriptions_too_short_publish_topic_fn)

static int s_rrs_request_subscriptions_add_single_subscription_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor token_path1 = aws_byte_cursor_from_c_str("token1");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_request_subscription(&subscriptions, &topic1, &token_path1);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    struct aws_rr_client_fixture_matched_request_subscription_view matched_subscriptions[] = {
        {payload1, topic1, topic1, token_path1},
    };

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        0 /* expected_stream_matches_count */,
        NULL,
        AWS_ARRAY_SIZE(matched_subscriptions),
        matched_subscriptions));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrs_request_subscriptions_add_single_subscription, s_rrs_request_subscriptions_add_single_subscription_fn)

static int s_rrs_request_subscriptions_remove_subscription_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor token_path1 = aws_byte_cursor_from_c_str("token1");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_request_subscription(&subscriptions, &topic1, &token_path1);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    struct aws_rr_client_fixture_matched_request_subscription_view matched_subscriptions[] = {
        {payload1, topic1, topic1, token_path1},
    };

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        0 /* expected_stream_matches_count */,
        NULL,
        AWS_ARRAY_SIZE(matched_subscriptions),
        matched_subscriptions));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_remove_request_subscription(&subscriptions, &topic1);

    struct aws_rr_client_fixture_subscriptions_matches_record *record2 =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record2);

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record2, 0 /* expected_stream_matches_count */, NULL, 0 /* expected_request_matches_count */, NULL));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record2);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrs_request_subscriptions_remove_subscription, s_rrs_request_subscriptions_remove_subscription_fn)

/* After adding multiple identical request subscriptions, the same number of removals is required to actually remove
 * the subscription. */
static int s_rrs_request_subscriptions_add_duplicate_then_remove_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor token_path1 = aws_byte_cursor_from_c_str("token1");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_request_subscription(&subscriptions, &topic1, &token_path1);
    aws_mqtt_request_response_client_subscriptions_add_request_subscription(&subscriptions, &topic1, &token_path1);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    struct aws_rr_client_fixture_matched_request_subscription_view matched_subscriptions[] = {
        {payload1, topic1, topic1, token_path1},
    };

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        0 /* expected_stream_matches_count */,
        NULL,
        AWS_ARRAY_SIZE(matched_subscriptions),
        matched_subscriptions));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    // First remove, decrement subscription's internal counter to 1.
    aws_mqtt_request_response_client_subscriptions_remove_request_subscription(&subscriptions, &topic1);

    struct aws_rr_client_fixture_subscriptions_matches_record *record2 =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record2);

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record2,
        0 /* expected_stream_matches_count */,
        NULL,
        AWS_ARRAY_SIZE(matched_subscriptions),
        matched_subscriptions));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record2);

    // Second remove, decrement subscription's internal counter to 0 and remove it.
    aws_mqtt_request_response_client_subscriptions_remove_request_subscription(&subscriptions, &topic1);

    struct aws_rr_client_fixture_subscriptions_matches_record *record3 =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record3);

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record3, 0 /* expected_stream_matches_count */, NULL, 0 /* expected_request_matches_count */, NULL));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record3);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrs_request_subscriptions_add_duplicate_then_remove,
    s_rrs_request_subscriptions_add_duplicate_then_remove_fn)

static int s_rrs_request_subscriptions_remove_nonexistent_subscription_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    aws_mqtt_request_response_client_subscriptions_remove_request_subscription(&subscriptions, &topic1);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrs_request_subscriptions_remove_nonexistent_subscription,
    s_rrs_request_subscriptions_remove_nonexistent_subscription_fn)

static int s_rrs_stream_and_request_subscriptions_add_same_subscription_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_request_response_subscriptions subscriptions;

    aws_mqtt_request_response_client_subscriptions_init(&subscriptions, allocator);

    struct aws_byte_cursor topic1 = aws_byte_cursor_from_c_str("topic/123/abc");
    struct aws_byte_cursor topic2 = aws_byte_cursor_from_c_str("topic/123/+");
    struct aws_byte_cursor token_path1 = aws_byte_cursor_from_c_str("token1");
    struct aws_byte_cursor payload1 = aws_byte_cursor_from_c_str("Payload1");

    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic1);
    aws_mqtt_request_response_client_subscriptions_add_stream_subscription(&subscriptions, &topic2);
    aws_mqtt_request_response_client_subscriptions_add_request_subscription(&subscriptions, &topic1, &token_path1);

    struct aws_mqtt_rr_incoming_publish_event publish_event = {
        .topic = topic1,
        .payload = payload1,
    };

    struct aws_rr_client_fixture_subscriptions_matches_record *record =
        s_aws_rr_client_fixture_subscriptions_matches_record_new(allocator);

    aws_mqtt_request_response_client_subscriptions_handle_incoming_publish(
        &subscriptions,
        &publish_event,
        s_rrs_fixture_on_stream_operation_subscription_match,
        s_rrs_fixture_on_request_operation_subscription_match,
        record);

    struct aws_rr_client_fixture_matched_stream_subscription_view matched_stream_subscriptions[] = {
        {payload1, topic1, topic1},
        {payload1, topic1, topic2},
    };

    struct aws_rr_client_fixture_matched_request_subscription_view matched_request_subscriptions[] = {
        {payload1, topic1, topic1, token_path1},
    };

    ASSERT_SUCCESS(s_rrc_verify_subscriptions_publishes(
        record,
        AWS_ARRAY_SIZE(matched_stream_subscriptions),
        matched_stream_subscriptions,
        AWS_ARRAY_SIZE(matched_request_subscriptions),
        matched_request_subscriptions));

    s_aws_rr_client_fixture_subscriptions_macthes_record_delete(record);

    aws_mqtt_request_response_client_subscriptions_clean_up(&subscriptions);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrs_stream_and_request_subscriptions_add_same_subscription,
    s_rrs_stream_and_request_subscriptions_add_same_subscription_fn)
