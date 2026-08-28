/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/mqtt.h>

#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/host_resolver.h>
#include <aws/mqtt/private/v5/mqtt5_client_impl.h>
#include <aws/mqtt/private/v5/mqtt5_options_storage.h>
#include <aws/mqtt/private/v5/mqtt5_utils.h>
#include <aws/mqtt/v5/mqtt5_client.h>
#include <aws/mqtt/v5/mqtt5_types.h>

#include <aws/testing/aws_test_harness.h>

static uint8_t s_server_reference[] = "derp.com";
static struct aws_byte_cursor s_server_reference_cursor = {
    .ptr = s_server_reference,
    .len = AWS_ARRAY_SIZE(s_server_reference) - 1,
};

static uint8_t s_binary_data[] = "binary data";
static struct aws_byte_cursor s_binary_data_cursor = {
    .ptr = s_binary_data,
    .len = AWS_ARRAY_SIZE(s_binary_data) - 1,
};

static uint8_t s_too_long_for_uint16[UINT16_MAX + 1];
static struct aws_byte_cursor s_too_long_for_uint16_cursor = {
    .ptr = s_too_long_for_uint16,
    .len = AWS_ARRAY_SIZE(s_too_long_for_uint16),
};

// Mqtt5 Specific invalid codepoint in prohibited range U+007F - U+009F (value = U+008F)",
static uint8_t s_invalid_utf8[] = "\xC2\x8F";
static struct aws_byte_cursor s_invalid_utf8_string = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(s_invalid_utf8);
static uint8_t s_user_prop_name[] = "name";
static uint8_t s_user_prop_value[] = "value";

static const struct aws_mqtt5_user_property s_bad_user_properties_name[] = {
    {
        .name =
            {
                .ptr = s_too_long_for_uint16,
                .len = AWS_ARRAY_SIZE(s_too_long_for_uint16),
            },
        .value =
            {
                .ptr = (uint8_t *)s_user_prop_value,
                .len = AWS_ARRAY_SIZE(s_user_prop_value) - 1,
            },
    },
};

static const struct aws_mqtt5_user_property s_bad_user_properties_value[] = {
    {
        .name =
            {
                .ptr = s_user_prop_name,
                .len = AWS_ARRAY_SIZE(s_user_prop_name) - 1,
            },
        .value =
            {
                .ptr = s_too_long_for_uint16,
                .len = AWS_ARRAY_SIZE(s_too_long_for_uint16),
            },
    },
};

static const struct aws_mqtt5_user_property s_user_properties_with_invalid_name[] = {
    {.name =
         {
             .ptr = (uint8_t *)s_invalid_utf8,
             .len = AWS_ARRAY_SIZE(s_invalid_utf8) - 1,
         },
     .value = {
         .ptr = (uint8_t *)s_user_prop_value,
         .len = AWS_ARRAY_SIZE(s_user_prop_value) - 1,
     }}};

static const struct aws_mqtt5_user_property s_user_properties_with_invalid_value[] = {
    {.name =
         {
             .ptr = s_user_prop_name,
             .len = AWS_ARRAY_SIZE(s_user_prop_name) - 1,
         },
     .value = {
         .ptr = (uint8_t *)s_invalid_utf8,
         .len = AWS_ARRAY_SIZE(s_invalid_utf8) - 1,
     }}};

static struct aws_mqtt5_user_property s_bad_user_properties_too_many[AWS_MQTT5_CLIENT_MAXIMUM_USER_PROPERTIES + 1];

/*
 * rather than just checking the bad view, we do a side-by-side before-after view validation as well to give more
 * confidence that the failure is coming from what we're actually trying to check
 */
#define AWS_VALIDATION_FAILURE_PREFIX(packet_type, failure_reason, view_name, mutate_function)                         \
    static int s_mqtt5_operation_##packet_type##_validation_failure_##failure_reason##_fn(                             \
        struct aws_allocator *allocator, void *ctx) {                                                                  \
        (void)ctx;                                                                                                     \
                                                                                                                       \
        struct aws_mqtt5_packet_##packet_type##_view good_view = view_name;                                            \
        struct aws_mqtt5_packet_##packet_type##_view bad_view = view_name;                                             \
        (*mutate_function)(&bad_view);                                                                                 \
        ASSERT_SUCCESS(aws_mqtt5_packet_##packet_type##_view_validate(&good_view));                                    \
        ASSERT_FAILS(aws_mqtt5_packet_##packet_type##_view_validate(&bad_view));

#define AWS_VALIDATION_FAILURE_SUFFIX(packet_type, failure_reason)                                                     \
    return AWS_OP_SUCCESS;                                                                                             \
    }                                                                                                                  \
                                                                                                                       \
    AWS_TEST_CASE(                                                                                                     \
        mqtt5_operation_##packet_type##_validation_failure_##failure_reason,                                           \
        s_mqtt5_operation_##packet_type##_validation_failure_##failure_reason##_fn)

#define AWS_VALIDATION_FAILURE_TEST4(packet_type, failure_reason, view_name, mutate_function)                          \
    AWS_VALIDATION_FAILURE_PREFIX(packet_type, failure_reason, view_name, mutate_function)                             \
    struct aws_mqtt5_operation_##packet_type *operation =                                                              \
        aws_mqtt5_operation_##packet_type##_new(allocator, &good_view, NULL, NULL);                                    \
    ASSERT_NOT_NULL(operation);                                                                                        \
    aws_mqtt5_operation_release(&operation->base);                                                                     \
    ASSERT_NULL(aws_mqtt5_operation_##packet_type##_new(allocator, &bad_view, NULL, NULL));                            \
    AWS_VALIDATION_FAILURE_SUFFIX(packet_type, failure_reason)

#define AWS_VALIDATION_FAILURE_TEST3(packet_type, failure_reason, view_name, mutate_function)                          \
    AWS_VALIDATION_FAILURE_PREFIX(packet_type, failure_reason, view_name, mutate_function)                             \
    ASSERT_NULL(aws_mqtt5_operation_##packet_type##_new(allocator, NULL, &bad_view, NULL));                            \
    AWS_VALIDATION_FAILURE_SUFFIX(packet_type, failure_reason)

#define AWS_VALIDATION_FAILURE_TEST2(packet_type, failure_reason, view_name, mutate_function)                          \
    AWS_VALIDATION_FAILURE_PREFIX(packet_type, failure_reason, view_name, mutate_function)                             \
    ASSERT_NULL(aws_mqtt5_operation_##packet_type##_new(allocator, &bad_view));                                        \
    AWS_VALIDATION_FAILURE_SUFFIX(packet_type, failure_reason)

#define AWS_IOT_CORE_VALIDATION_FAILURE(packet_type, failure_reason, view_name, mutate_function)                       \
    static int s_mqtt5_operation_##packet_type##_validation_failure_##failure_reason##_fn(                             \
        struct aws_allocator *allocator, void *ctx) {                                                                  \
        (void)ctx;                                                                                                     \
                                                                                                                       \
        struct aws_mqtt5_packet_##packet_type##_view good_view = view_name;                                            \
        struct aws_mqtt5_packet_##packet_type##_view bad_view = view_name;                                             \
        (*mutate_function)(&bad_view);                                                                                 \
        ASSERT_SUCCESS(aws_mqtt5_packet_##packet_type##_view_validate(&good_view));                                    \
        ASSERT_SUCCESS(aws_mqtt5_packet_##packet_type##_view_validate(&bad_view));                                     \
                                                                                                                       \
        struct aws_mqtt5_client dummy_client;                                                                          \
        AWS_ZERO_STRUCT(dummy_client);                                                                                 \
        struct aws_mqtt5_client_options_storage client_options_storage;                                                \
        AWS_ZERO_STRUCT(client_options_storage);                                                                       \
        client_options_storage.extended_validation_and_flow_control_options = AWS_MQTT5_EVAFCO_AWS_IOT_CORE_DEFAULTS;  \
        dummy_client.config = &client_options_storage;                                                                 \
                                                                                                                       \
        ASSERT_NULL(aws_mqtt5_operation_##packet_type##_new(allocator, &dummy_client, &bad_view, NULL));               \
        return AWS_OP_SUCCESS;                                                                                         \
    }                                                                                                                  \
                                                                                                                       \
    AWS_TEST_CASE(                                                                                                     \
        mqtt5_operation_##packet_type##_validation_failure_##failure_reason,                                           \
        s_mqtt5_operation_##packet_type##_validation_failure_##failure_reason##_fn)

static struct aws_mqtt5_packet_disconnect_view s_good_disconnect_view;

static void s_make_server_reference_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->server_reference = &s_server_reference_cursor;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    server_reference,
    s_good_disconnect_view,
    s_make_server_reference_disconnect_view)

static void s_make_bad_reason_code_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->reason_code = -1;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    bad_reason_code,
    s_good_disconnect_view,
    s_make_bad_reason_code_disconnect_view)

static void s_make_reason_string_too_long_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->reason_string = &s_too_long_for_uint16_cursor;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    reason_string_too_long,
    s_good_disconnect_view,
    s_make_reason_string_too_long_disconnect_view)

static void s_make_reason_string_invalid_utf8_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->reason_string = &s_invalid_utf8_string;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    reason_string_invalid_utf8,
    s_good_disconnect_view,
    s_make_reason_string_invalid_utf8_disconnect_view)

static void s_make_user_properties_name_too_long_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_name);
    view->user_properties = s_bad_user_properties_name;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    user_properties_name_too_long,
    s_good_disconnect_view,
    s_make_user_properties_name_too_long_disconnect_view)

static void s_make_user_properties_name_invalid_utf8_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_name);
    view->user_properties = s_user_properties_with_invalid_name;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    user_properties_name_invalid_utf8,
    s_good_disconnect_view,
    s_make_user_properties_name_invalid_utf8_disconnect_view)

static void s_make_user_properties_value_too_long_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_value);
    view->user_properties = s_bad_user_properties_value;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    user_properties_value_too_long,
    s_good_disconnect_view,
    s_make_user_properties_value_too_long_disconnect_view)

static void s_make_user_properties_value_invalid_utf8_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_value);
    view->user_properties = s_user_properties_with_invalid_value;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    user_properties_value_invalid_utf8,
    s_good_disconnect_view,
    s_make_user_properties_value_invalid_utf8_disconnect_view)

static void s_make_user_properties_too_many_disconnect_view(struct aws_mqtt5_packet_disconnect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_too_many);
    view->user_properties = s_bad_user_properties_too_many;
}

AWS_VALIDATION_FAILURE_TEST4(
    disconnect,
    user_properties_too_many,
    s_good_disconnect_view,
    s_make_user_properties_too_many_disconnect_view)

static struct aws_mqtt5_packet_connect_view s_good_connect_view;

static void s_make_client_id_too_long_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->client_id.ptr = s_too_long_for_uint16;
    view->client_id.len = AWS_ARRAY_SIZE(s_too_long_for_uint16);
}

AWS_VALIDATION_FAILURE_TEST2(connect, client_id_too_long, s_good_connect_view, s_make_client_id_too_long_connect_view)

static void s_make_client_id_invalid_utf8_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->client_id = s_invalid_utf8_string;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    client_id_invalid_utf8,
    s_good_connect_view,
    s_make_client_id_invalid_utf8_connect_view)

static void s_make_username_too_long_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->username = &s_too_long_for_uint16_cursor;
}

AWS_VALIDATION_FAILURE_TEST2(connect, username_too_long, s_good_connect_view, s_make_username_too_long_connect_view)

static void s_make_username_invalid_utf8_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->username = &s_invalid_utf8_string;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    username_invalid_utf8,
    s_good_connect_view,
    s_make_username_invalid_utf8_connect_view)

static void s_make_password_too_long_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->password = &s_too_long_for_uint16_cursor;
}

AWS_VALIDATION_FAILURE_TEST2(connect, password_too_long, s_good_connect_view, s_make_password_too_long_connect_view)

static const uint16_t s_zero_receive_maximum = 0;
static void s_make_receive_maximum_zero_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->receive_maximum = &s_zero_receive_maximum;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    receive_maximum_zero,
    s_good_connect_view,
    s_make_receive_maximum_zero_connect_view)

static const uint32_t s_maximum_packet_size_zero = 0;
static void s_make_maximum_packet_size_zero_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->maximum_packet_size_bytes = &s_maximum_packet_size_zero;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    maximum_packet_size_zero,
    s_good_connect_view,
    s_make_maximum_packet_size_zero_connect_view)

static void s_make_auth_method_unsupported_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->authentication_method = &s_binary_data_cursor;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    auth_method_unsupported,
    s_good_connect_view,
    s_make_auth_method_unsupported_connect_view)

static void s_make_auth_data_unsupported_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->authentication_data = &s_binary_data_cursor;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    auth_data_unsupported,
    s_good_connect_view,
    s_make_auth_data_unsupported_connect_view)

static uint8_t s_bad_boolean = 2;
static void s_make_request_problem_information_invalid_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->request_problem_information = &s_bad_boolean;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    request_problem_information_invalid,
    s_good_connect_view,
    s_make_request_problem_information_invalid_connect_view)

static void s_make_request_response_information_invalid_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->request_response_information = &s_bad_boolean;
};

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    request_response_information_invalid,
    s_good_connect_view,
    s_make_request_response_information_invalid_connect_view)

static void s_make_user_properties_name_too_long_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_name);
    view->user_properties = s_bad_user_properties_name;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    user_properties_name_too_long,
    s_good_connect_view,
    s_make_user_properties_name_too_long_connect_view)

static void s_make_user_properties_name_invalid_utf8_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_name);
    view->user_properties = s_user_properties_with_invalid_name;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    user_properties_name_invalid_utf8,
    s_good_connect_view,
    s_make_user_properties_name_invalid_utf8_connect_view)

static void s_make_user_properties_value_too_long_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_value);
    view->user_properties = s_bad_user_properties_value;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    user_properties_value_too_long,
    s_good_connect_view,
    s_make_user_properties_value_too_long_connect_view)

static void s_make_user_properties_value_invalid_utf8_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_value);
    view->user_properties = s_user_properties_with_invalid_value;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    user_properties_value_invalid_utf8,
    s_good_connect_view,
    s_make_user_properties_value_invalid_utf8_connect_view)

static void s_make_user_properties_too_many_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_too_many);
    view->user_properties = s_bad_user_properties_too_many;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    user_properties_too_many,
    s_good_connect_view,
    s_make_user_properties_too_many_connect_view)

static uint8_t s_good_topic[] = "hello/world";

/* no-topic and no-topic-alias is invalid */
static struct aws_mqtt5_packet_publish_view s_good_will_publish_view = {
    .topic =
        {
            .ptr = s_good_topic,
            .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
        },
};

static struct aws_mqtt5_packet_connect_view s_good_will_connect_view = {
    .will = &s_good_will_publish_view,
};

static struct aws_mqtt5_packet_publish_view s_will_invalid_publish_view;

static void s_make_will_invalid_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->will = &s_will_invalid_publish_view;
}

AWS_VALIDATION_FAILURE_TEST2(connect, will_invalid, s_good_will_connect_view, s_make_will_invalid_connect_view)

static struct aws_mqtt5_packet_publish_view s_will_payload_too_long_publish_view = {
    .topic =
        {
            .ptr = s_user_prop_name,
            .len = AWS_ARRAY_SIZE(s_user_prop_name) - 1,
        },
    .payload = {
        .ptr = s_too_long_for_uint16,
        .len = AWS_ARRAY_SIZE(s_too_long_for_uint16),
    }};

static void s_make_will_payload_too_long_connect_view(struct aws_mqtt5_packet_connect_view *view) {
    view->will = &s_will_payload_too_long_publish_view;
}

AWS_VALIDATION_FAILURE_TEST2(
    connect,
    will_payload_too_long,
    s_good_will_connect_view,
    s_make_will_payload_too_long_connect_view)

static struct aws_mqtt5_subscription_view s_good_subscription[] = {
    {
        .topic_filter =
            {
                .ptr = s_good_topic,
                .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .no_local = false,
        .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE,
        .retain_as_published = false,
    },
};

static struct aws_mqtt5_packet_subscribe_view s_good_subscribe_view = {
    .subscriptions = s_good_subscription,
    .subscription_count = AWS_ARRAY_SIZE(s_good_subscription),
};

static void s_make_no_subscriptions_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->subscriptions = NULL;
    view->subscription_count = 0;
}

AWS_VALIDATION_FAILURE_TEST3(subscribe, no_subscriptions, s_good_subscribe_view, s_make_no_subscriptions_subscribe_view)

static struct aws_mqtt5_subscription_view
    s_too_many_subscriptions[AWS_MQTT5_CLIENT_MAXIMUM_SUBSCRIPTIONS_PER_SUBSCRIBE + 1];

static void s_make_too_many_subscriptions_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_too_many_subscriptions); ++i) {
        struct aws_mqtt5_subscription_view *subscription_view = &s_too_many_subscriptions[i];

        subscription_view->topic_filter = aws_byte_cursor_from_array(s_good_topic, AWS_ARRAY_SIZE(s_good_topic) - 1);
        subscription_view->qos = AWS_MQTT5_QOS_AT_MOST_ONCE;
        subscription_view->no_local = false;
        subscription_view->retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE;
        subscription_view->retain_as_published = false;
    }

    view->subscriptions = s_too_many_subscriptions;
    view->subscription_count = AWS_ARRAY_SIZE(s_too_many_subscriptions);
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    too_many_subscriptions,
    s_good_subscribe_view,
    s_make_too_many_subscriptions_subscribe_view)

static const uint32_t s_invalid_subscription_identifier = AWS_MQTT5_MAXIMUM_VARIABLE_LENGTH_INTEGER + 1;

static void s_make_invalid_subscription_identifier_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->subscription_identifier = &s_invalid_subscription_identifier;
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    invalid_subscription_identifier,
    s_good_subscribe_view,
    s_make_invalid_subscription_identifier_subscribe_view)

static uint8_t s_bad_topic_filter[] = "hello/#/world";

static struct aws_mqtt5_subscription_view s_invalid_topic_filter_subscription[] = {
    {
        .topic_filter =
            {
                .ptr = s_bad_topic_filter,
                .len = AWS_ARRAY_SIZE(s_bad_topic_filter) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .no_local = false,
        .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE,
        .retain_as_published = false,
    },
};

static struct aws_mqtt5_subscription_view s_invalid_utf8_topic_filter_subscription[] = {
    {
        .topic_filter =
            {
                .ptr = (uint8_t *)s_invalid_utf8,
                .len = AWS_ARRAY_SIZE(s_invalid_utf8) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .no_local = false,
        .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE,
        .retain_as_published = false,
    },
};

static void s_make_invalid_topic_filter_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->subscriptions = s_invalid_topic_filter_subscription;
    view->subscription_count = AWS_ARRAY_SIZE(s_invalid_topic_filter_subscription);
};

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    invalid_topic_filter,
    s_good_subscribe_view,
    s_make_invalid_topic_filter_subscribe_view)

static void s_make_invalid_utf8_topic_filter_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->subscriptions = s_invalid_utf8_topic_filter_subscription;
    view->subscription_count = AWS_ARRAY_SIZE(s_invalid_utf8_topic_filter_subscription);
};

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    invalid_utf8_topic_filter,
    s_good_subscribe_view,
    s_make_invalid_utf8_topic_filter_subscribe_view)

static struct aws_mqtt5_subscription_view s_invalid_qos_subscription[] = {
    {
        .topic_filter =
            {
                .ptr = s_good_topic,
                .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
            },
        .qos = 3,
        .no_local = false,
        .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE,
        .retain_as_published = false,
    },
};

static void s_make_invalid_qos_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->subscriptions = s_invalid_qos_subscription;
    view->subscription_count = AWS_ARRAY_SIZE(s_invalid_qos_subscription);
}

AWS_VALIDATION_FAILURE_TEST3(subscribe, invalid_qos, s_good_subscribe_view, s_make_invalid_qos_subscribe_view)

static struct aws_mqtt5_subscription_view s_invalid_retain_type_subscription[] = {
    {
        .topic_filter =
            {
                .ptr = s_good_topic,
                .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .no_local = false,
        .retain_handling_type = 5,
        .retain_as_published = false,
    },
};

static void s_make_invalid_retain_type_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->subscriptions = s_invalid_retain_type_subscription;
    view->subscription_count = AWS_ARRAY_SIZE(s_invalid_retain_type_subscription);
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    invalid_retain_type,
    s_good_subscribe_view,
    s_make_invalid_retain_type_subscribe_view)

static uint8_t s_shared_topic[] = "$share/sharename/topic/filter";

static struct aws_mqtt5_subscription_view s_invalid_no_local_subscription[] = {
    {
        .topic_filter =
            {
                .ptr = s_shared_topic,
                .len = AWS_ARRAY_SIZE(s_shared_topic) - 1,
            },
        .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
        .no_local = true,
        .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE,
        .retain_as_published = false,
    },
};

static void s_make_invalid_no_local_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->subscriptions = s_invalid_no_local_subscription;
    view->subscription_count = AWS_ARRAY_SIZE(s_invalid_no_local_subscription);
}

AWS_VALIDATION_FAILURE_TEST3(subscribe, invalid_no_local, s_good_subscribe_view, s_make_invalid_no_local_subscribe_view)

static void s_make_user_properties_name_too_long_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_name);
    view->user_properties = s_bad_user_properties_name;
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    user_properties_name_too_long,
    s_good_subscribe_view,
    s_make_user_properties_name_too_long_subscribe_view)

static void s_make_user_properties_name_invalid_utf8_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_name);
    view->user_properties = s_user_properties_with_invalid_name;
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    user_properties_name_invalid_utf8,
    s_good_subscribe_view,
    s_make_user_properties_name_invalid_utf8_subscribe_view)

static void s_make_user_properties_value_too_long_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_value);
    view->user_properties = s_bad_user_properties_value;
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    user_properties_value_too_long,
    s_good_subscribe_view,
    s_make_user_properties_value_too_long_subscribe_view)

static void s_make_user_properties_value_invalid_utf8_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_value);
    view->user_properties = s_user_properties_with_invalid_value;
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    user_properties_value_invalid_utf8,
    s_good_subscribe_view,
    s_make_user_properties_value_invalid_utf8_subscribe_view)

static void s_make_user_properties_too_many_subscribe_view(struct aws_mqtt5_packet_subscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_too_many);
    view->user_properties = s_bad_user_properties_too_many;
}

AWS_VALIDATION_FAILURE_TEST3(
    subscribe,
    user_properties_too_many,
    s_good_subscribe_view,
    s_make_user_properties_too_many_subscribe_view)

static struct aws_byte_cursor s_good_topic_filter[] = {
    {
        .ptr = s_good_topic,
        .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
    },
};

static struct aws_mqtt5_packet_unsubscribe_view s_good_unsubscribe_view = {
    .topic_filters = s_good_topic_filter,
    .topic_filter_count = AWS_ARRAY_SIZE(s_good_topic_filter),
};

static void s_make_no_topic_filters_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->topic_filters = NULL;
    view->topic_filter_count = 0;
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    no_topic_filters,
    s_good_unsubscribe_view,
    s_make_no_topic_filters_unsubscribe_view)

static struct aws_byte_cursor s_too_many_topic_filters[AWS_MQTT5_CLIENT_MAXIMUM_TOPIC_FILTERS_PER_UNSUBSCRIBE + 1];

static void s_make_too_many_topic_filters_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_too_many_topic_filters); ++i) {
        struct aws_byte_cursor *cursor = &s_too_many_topic_filters[i];

        cursor->ptr = s_good_topic;
        cursor->len = AWS_ARRAY_SIZE(s_good_topic) - 1;
    }

    view->topic_filters = s_too_many_topic_filters;
    view->topic_filter_count = AWS_ARRAY_SIZE(s_too_many_topic_filters);
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    too_many_topic_filters,
    s_good_unsubscribe_view,
    s_make_too_many_topic_filters_unsubscribe_view)

static struct aws_byte_cursor s_invalid_topic_filter[] = {
    {
        .ptr = s_bad_topic_filter,
        .len = AWS_ARRAY_SIZE(s_bad_topic_filter) - 1,
    },
};

static struct aws_byte_cursor s_invalid_utf8_topic_filter[] = {{
    .ptr = (uint8_t *)s_invalid_utf8,
    .len = AWS_ARRAY_SIZE(s_invalid_utf8) - 1,
}};

static void s_make_invalid_topic_filter_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->topic_filters = s_invalid_topic_filter;
    view->topic_filter_count = AWS_ARRAY_SIZE(s_invalid_topic_filter);
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    invalid_topic_filter,
    s_good_unsubscribe_view,
    s_make_invalid_topic_filter_unsubscribe_view)

static void s_make_invalid_utf8_topic_filter_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->topic_filters = s_invalid_utf8_topic_filter;
    view->topic_filter_count = AWS_ARRAY_SIZE(s_invalid_utf8_topic_filter);
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    invalid_utf8_topic_filter,
    s_good_unsubscribe_view,
    s_make_invalid_utf8_topic_filter_unsubscribe_view)

static void s_make_user_properties_name_too_long_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_name);
    view->user_properties = s_bad_user_properties_name;
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    user_properties_name_too_long,
    s_good_unsubscribe_view,
    s_make_user_properties_name_too_long_unsubscribe_view)

static void s_make_user_properties_name_invalid_utf8_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_name);
    view->user_properties = s_user_properties_with_invalid_name;
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    user_properties_name_invalid_utf8,
    s_good_unsubscribe_view,
    s_make_user_properties_name_invalid_utf8_unsubscribe_view)

static void s_make_user_properties_value_too_long_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_value);
    view->user_properties = s_bad_user_properties_value;
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    user_properties_value_too_long,
    s_good_unsubscribe_view,
    s_make_user_properties_value_too_long_unsubscribe_view)

static void s_make_user_properties_value_invalid_utf8_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_value);
    view->user_properties = s_user_properties_with_invalid_value;
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    user_properties_value_invalid_utf8,
    s_good_unsubscribe_view,
    s_make_user_properties_value_invalid_utf8_unsubscribe_view)

static void s_make_user_properties_too_many_unsubscribe_view(struct aws_mqtt5_packet_unsubscribe_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_too_many);
    view->user_properties = s_bad_user_properties_too_many;
}

AWS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    user_properties_too_many,
    s_good_unsubscribe_view,
    s_make_user_properties_too_many_unsubscribe_view)

static struct aws_mqtt5_packet_publish_view s_good_publish_view = {
    .topic =
        {
            .ptr = s_good_topic,
            .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
        },
};

static uint8_t s_invalid_topic[] = "hello/#";
static void s_make_invalid_topic_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->topic.ptr = s_invalid_topic;
    view->topic.len = AWS_ARRAY_SIZE(s_invalid_topic) - 1;
}

AWS_VALIDATION_FAILURE_TEST3(publish, invalid_topic, s_good_publish_view, s_make_invalid_topic_publish_view)

static void s_make_invalid_utf8_topic_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->topic = s_invalid_utf8_string;
}

AWS_VALIDATION_FAILURE_TEST3(publish, invalid_utf8_topic, s_good_publish_view, s_make_invalid_utf8_topic_publish_view)

static void s_make_no_topic_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->topic.ptr = NULL;
    view->topic.len = 0;
}

AWS_VALIDATION_FAILURE_TEST3(publish, no_topic, s_good_publish_view, s_make_no_topic_publish_view)

static enum aws_mqtt5_payload_format_indicator s_invalid_payload_format = 3;
static enum aws_mqtt5_payload_format_indicator s_valid_payload_format = AWS_MQTT5_PFI_UTF8;

static void s_make_invalid_payload_format_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->payload_format = &s_invalid_payload_format;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    invalid_payload_format,
    s_good_publish_view,
    s_make_invalid_payload_format_publish_view)

static void s_make_invalid_utf8_payload_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->payload_format = &s_valid_payload_format;
    view->payload = s_invalid_utf8_string;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    invalid_utf8_payload,
    s_good_publish_view,
    s_make_invalid_utf8_payload_publish_view)

static void s_make_response_topic_too_long_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->response_topic = &s_too_long_for_uint16_cursor;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    response_topic_too_long,
    s_good_publish_view,
    s_make_response_topic_too_long_publish_view)

static struct aws_byte_cursor s_invalid_topic_cursor = {
    .ptr = s_invalid_topic,
    .len = AWS_ARRAY_SIZE(s_invalid_topic) - 1,
};

static void s_make_response_topic_invalid_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->response_topic = &s_invalid_topic_cursor;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    invalid_response_topic,
    s_good_publish_view,
    s_make_response_topic_invalid_publish_view)

static void s_make_response_topic_invalid_utf8_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->response_topic = &s_invalid_utf8_string;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    invalid_utf8_response_topic,
    s_good_publish_view,
    s_make_response_topic_invalid_utf8_publish_view)

static void s_make_correlation_data_too_long_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->correlation_data = &s_too_long_for_uint16_cursor;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    correlation_data_too_long,
    s_good_publish_view,
    s_make_correlation_data_too_long_publish_view)

static void s_make_content_type_invalid_utf8_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->content_type = &s_invalid_utf8_string;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    invalid_utf8_content_type,
    s_good_publish_view,
    s_make_content_type_invalid_utf8_publish_view)

static const uint32_t s_subscription_identifiers[] = {1, 2};

static void s_make_subscription_identifier_exists_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->subscription_identifiers = s_subscription_identifiers;
    view->subscription_identifier_count = AWS_ARRAY_SIZE(s_subscription_identifiers);
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    subscription_identifier_exists,
    s_good_publish_view,
    s_make_subscription_identifier_exists_publish_view)

static void s_make_content_type_too_long_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->content_type = &s_too_long_for_uint16_cursor;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    content_type_too_long,
    s_good_publish_view,
    s_make_content_type_too_long_publish_view)

static const uint16_t s_topic_alias_zero = 0;
static void s_make_topic_alias_zero_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->topic_alias = &s_topic_alias_zero;
    view->topic.ptr = NULL;
    view->topic.len = 0;
}

AWS_VALIDATION_FAILURE_TEST3(publish, topic_alias_zero, s_good_publish_view, s_make_topic_alias_zero_publish_view)

static void s_make_user_properties_name_too_long_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_name);
    view->user_properties = s_bad_user_properties_name;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    user_properties_name_too_long,
    s_good_publish_view,
    s_make_user_properties_name_too_long_publish_view)

static void s_make_user_properties_name_invalid_utf8_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_name);
    view->user_properties = s_user_properties_with_invalid_name;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    user_properties_name_invalid_utf8,
    s_good_publish_view,
    s_make_user_properties_name_invalid_utf8_publish_view)

static void s_make_user_properties_value_too_long_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_value);
    view->user_properties = s_bad_user_properties_value;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    user_properties_value_too_long,
    s_good_publish_view,
    s_make_user_properties_value_too_long_publish_view)

static void s_make_user_properties_value_invalid_utf8_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_user_properties_with_invalid_value);
    view->user_properties = s_user_properties_with_invalid_value;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    user_properties_value_invalid_utf8,
    s_good_publish_view,
    s_make_user_properties_value_invalid_utf8_publish_view)

static void s_make_user_properties_too_many_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->user_property_count = AWS_ARRAY_SIZE(s_bad_user_properties_too_many);
    view->user_properties = s_bad_user_properties_too_many;
}

AWS_VALIDATION_FAILURE_TEST3(
    publish,
    user_properties_too_many,
    s_good_publish_view,
    s_make_user_properties_too_many_publish_view)

static void s_make_qos0_duplicate_true_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->qos = AWS_MQTT5_QOS_AT_MOST_ONCE;
    view->duplicate = true;
}

AWS_VALIDATION_FAILURE_TEST3(publish, qos0_duplicate_true, s_good_publish_view, s_make_qos0_duplicate_true_publish_view)

static void s_make_qos0_with_packet_id_publish_view(struct aws_mqtt5_packet_publish_view *view) {
    view->qos = AWS_MQTT5_QOS_AT_MOST_ONCE;
    view->packet_id = 1;
}

AWS_VALIDATION_FAILURE_TEST3(publish, qos0_with_packet_id, s_good_publish_view, s_make_qos0_with_packet_id_publish_view)

#define AWS_CLIENT_CREATION_VALIDATION_FAILURE(failure_reason, base_options, mutate_function)                          \
    static int s_mqtt5_client_options_validation_failure_##failure_reason##_fn(                                        \
        struct aws_allocator *allocator, void *ctx) {                                                                  \
        (void)ctx;                                                                                                     \
        aws_mqtt_library_init(allocator);                                                                              \
        struct aws_event_loop_group *elg = NULL;                                                                       \
        struct aws_host_resolver *hr = NULL;                                                                           \
        struct aws_client_bootstrap *bootstrap = NULL;                                                                 \
        elg = aws_event_loop_group_new_default(allocator, 1, NULL);                                                    \
                                                                                                                       \
        struct aws_host_resolver_default_options hr_options = {                                                        \
            .el_group = elg,                                                                                           \
            .max_entries = 1,                                                                                          \
        };                                                                                                             \
        hr = aws_host_resolver_new_default(allocator, &hr_options);                                                    \
                                                                                                                       \
        struct aws_client_bootstrap_options bootstrap_options = {                                                      \
            .event_loop_group = elg,                                                                                   \
            .host_resolver = hr,                                                                                       \
        };                                                                                                             \
        bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);                                           \
                                                                                                                       \
        struct aws_mqtt5_client_options good_options = base_options;                                                   \
        good_options.bootstrap = bootstrap;                                                                            \
                                                                                                                       \
        ASSERT_SUCCESS(aws_mqtt5_client_options_validate(&good_options));                                              \
        struct aws_mqtt5_client *client = aws_mqtt5_client_new(allocator, &good_options);                              \
        ASSERT_NOT_NULL(client);                                                                                       \
        aws_mqtt5_client_release(client);                                                                              \
                                                                                                                       \
        struct aws_mqtt5_client_options bad_options = good_options;                                                    \
        (*mutate_function)(&bad_options);                                                                              \
        ASSERT_FAILS(aws_mqtt5_client_options_validate(&bad_options));                                                 \
        ASSERT_NULL(aws_mqtt5_client_new(allocator, &bad_options));                                                    \
                                                                                                                       \
        aws_client_bootstrap_release(bootstrap);                                                                       \
        aws_host_resolver_release(hr);                                                                                 \
        aws_event_loop_group_release(elg);                                                                             \
                                                                                                                       \
        aws_mqtt_library_clean_up();                                                                                   \
        return AWS_OP_SUCCESS;                                                                                         \
    }                                                                                                                  \
                                                                                                                       \
    AWS_TEST_CASE(                                                                                                     \
        mqtt5_client_options_validation_failure_##failure_reason,                                                      \
        s_mqtt5_client_options_validation_failure_##failure_reason##_fn)

static struct aws_socket_options s_good_socket_options = {
    .type = AWS_SOCKET_STREAM,
    .domain = AWS_SOCKET_IPV4,
    .connect_timeout_ms = 10000,
};

static struct aws_mqtt5_packet_connect_view s_good_connect = {
    .keep_alive_interval_seconds = 30,
};

void s_lifecycle_event_handler(const struct aws_mqtt5_client_lifecycle_event *event) {
    (void)event;
}

void s_publish_received(const struct aws_mqtt5_packet_publish_view *publish, void *user_data) {
    (void)publish;
    (void)user_data;
}

static struct aws_mqtt5_client_options s_good_client_options = {
    .host_name =
        {
            .ptr = s_server_reference,
            .len = AWS_ARRAY_SIZE(s_server_reference) - 1,
        },
    .port = 1883,
    .socket_options = &s_good_socket_options,
    .connect_options = &s_good_connect,
    .ping_timeout_ms = 5000,
    .lifecycle_event_handler = &s_lifecycle_event_handler,
    .publish_received_handler = &s_publish_received,
};

static void s_make_no_host_client_options(struct aws_mqtt5_client_options *options) {
    options->host_name.ptr = NULL;
    options->host_name.len = 0;
}

AWS_CLIENT_CREATION_VALIDATION_FAILURE(no_host, s_good_client_options, s_make_no_host_client_options)

static void s_make_no_bootstrap_client_options(struct aws_mqtt5_client_options *options) {
    options->bootstrap = NULL;
}

AWS_CLIENT_CREATION_VALIDATION_FAILURE(no_bootstrap, s_good_client_options, s_make_no_bootstrap_client_options)

static void s_make_no_publish_received_client_options(struct aws_mqtt5_client_options *options) {
    options->publish_received_handler = NULL;
}

AWS_CLIENT_CREATION_VALIDATION_FAILURE(
    no_publish_received,
    s_good_client_options,
    s_make_no_publish_received_client_options)

static struct aws_socket_options s_bad_socket_options = {
    .type = AWS_SOCKET_DGRAM,
};

static void s_make_invalid_socket_options_client_options(struct aws_mqtt5_client_options *options) {
    options->socket_options = &s_bad_socket_options;
};

AWS_CLIENT_CREATION_VALIDATION_FAILURE(
    invalid_socket_options,
    s_good_client_options,
    s_make_invalid_socket_options_client_options)

static struct aws_mqtt5_packet_connect_view s_client_id_too_long_connect_view = {
    .client_id =
        {
            .ptr = s_too_long_for_uint16,
            .len = AWS_ARRAY_SIZE(s_too_long_for_uint16),
        },
};

static void s_make_invalid_connect_client_options(struct aws_mqtt5_client_options *options) {
    options->connect_options = &s_client_id_too_long_connect_view;
}

AWS_CLIENT_CREATION_VALIDATION_FAILURE(invalid_connect, s_good_client_options, s_make_invalid_connect_client_options)

static void s_make_invalid_port_client_options(struct aws_mqtt5_client_options *options) {
    options->port = 0xFFFFFFFF;
}

AWS_CLIENT_CREATION_VALIDATION_FAILURE(invalid_port, s_good_client_options, s_make_invalid_port_client_options)

#define AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST_PREFIX(packet_type, failure_reason, init_success_settings_fn)  \
    static int s_mqtt5_operation_##packet_type##_connection_settings_validation_failure_##failure_reason##_fn(         \
        struct aws_allocator *allocator, void *ctx) {                                                                  \
        (void)ctx;                                                                                                     \
                                                                                                                       \
        struct aws_mqtt5_client dummy_client;                                                                          \
        AWS_ZERO_STRUCT(dummy_client);                                                                                 \
                                                                                                                       \
        dummy_client.current_state = AWS_MCS_CONNECTED;                                                                \
        (*init_success_settings_fn)(&dummy_client);

#define AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST_SUFFIX(packet_type, failure_reason, init_failure_settings_fn)  \
    ASSERT_NOT_NULL(operation);                                                                                        \
                                                                                                                       \
    ASSERT_SUCCESS(aws_mqtt5_operation_validate_vs_connection_settings(&operation->base, &dummy_client));              \
                                                                                                                       \
    (*init_failure_settings_fn)(&dummy_client);                                                                        \
                                                                                                                       \
    ASSERT_FAILS(aws_mqtt5_operation_validate_vs_connection_settings(&operation->base, &dummy_client));                \
                                                                                                                       \
    aws_mqtt5_operation_release(&operation->base);                                                                     \
                                                                                                                       \
    return AWS_OP_SUCCESS;                                                                                             \
    }                                                                                                                  \
                                                                                                                       \
    AWS_TEST_CASE(                                                                                                     \
        mqtt5_operation_##packet_type##_connection_settings_validation_failure_##failure_reason,                       \
        s_mqtt5_operation_##packet_type##_connection_settings_validation_failure_##failure_reason##_fn)

#define AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST3(                                                              \
    packet_type, failure_reason, view_name, init_success_settings_fn, init_failure_settings_fn)                        \
    AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST_PREFIX(packet_type, failure_reason, init_success_settings_fn)      \
    struct aws_mqtt5_operation_##packet_type *operation =                                                              \
        aws_mqtt5_operation_##packet_type##_new(allocator, NULL, &view_name, NULL);                                    \
    AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST_SUFFIX(packet_type, failure_reason, init_failure_settings_fn)

static struct aws_mqtt5_packet_subscribe_view s_exceeds_maximum_packet_size_subscribe_view = {
    .subscriptions = s_good_subscription,
    .subscription_count = AWS_ARRAY_SIZE(s_good_subscription),
};

static void s_packet_size_init_settings_success_fn(struct aws_mqtt5_client *dummy_client) {
    dummy_client->negotiated_settings.maximum_packet_size_to_server = 100;
}

static void s_packet_size_init_settings_failure_fn(struct aws_mqtt5_client *dummy_client) {
    dummy_client->negotiated_settings.maximum_packet_size_to_server = 10;
}

AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST3(
    subscribe,
    exceeds_maximum_packet_size,
    s_exceeds_maximum_packet_size_subscribe_view,
    s_packet_size_init_settings_success_fn,
    s_packet_size_init_settings_failure_fn)

static struct aws_mqtt5_packet_unsubscribe_view s_exceeds_maximum_packet_size_unsubscribe_view = {
    .topic_filters = s_good_topic_filter,
    .topic_filter_count = AWS_ARRAY_SIZE(s_good_topic_filter),
};

AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST3(
    unsubscribe,
    exceeds_maximum_packet_size,
    s_exceeds_maximum_packet_size_unsubscribe_view,
    s_packet_size_init_settings_success_fn,
    s_packet_size_init_settings_failure_fn)

static struct aws_mqtt5_packet_publish_view s_exceeds_maximum_packet_size_publish_view = {
    .topic =
        {
            .ptr = s_good_topic,
            .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
        },
    .payload =
        {
            .ptr = s_binary_data,
            .len = AWS_ARRAY_SIZE(s_binary_data),
        },
};

AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST3(
    publish,
    exceeds_maximum_packet_size,
    s_exceeds_maximum_packet_size_publish_view,
    s_packet_size_init_settings_success_fn,
    s_packet_size_init_settings_failure_fn)

static struct aws_mqtt5_packet_publish_view s_exceeds_maximum_qos_publish_view = {
    .topic =
        {
            .ptr = s_good_topic,
            .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
        },
    .qos = AWS_MQTT5_QOS_EXACTLY_ONCE,
};

static void s_maximum_qos_init_settings_success_fn(struct aws_mqtt5_client *dummy_client) {
    dummy_client->negotiated_settings.maximum_packet_size_to_server = 100;
    dummy_client->negotiated_settings.maximum_qos = AWS_MQTT5_QOS_EXACTLY_ONCE;
}

static void s_maximum_qos_init_settings_failure_fn(struct aws_mqtt5_client *dummy_client) {
    dummy_client->negotiated_settings.maximum_qos = AWS_MQTT5_QOS_AT_LEAST_ONCE;
}

AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST3(
    publish,
    exceeds_maximum_qos,
    s_exceeds_maximum_qos_publish_view,
    s_maximum_qos_init_settings_success_fn,
    s_maximum_qos_init_settings_failure_fn)

static struct aws_mqtt5_packet_publish_view s_invalid_retain_publish_view = {
    .topic =
        {
            .ptr = s_good_topic,
            .len = AWS_ARRAY_SIZE(s_good_topic) - 1,
        },
    .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
    .retain = true,
};

static void s_invalid_retain_init_settings_success_fn(struct aws_mqtt5_client *dummy_client) {
    dummy_client->negotiated_settings.maximum_packet_size_to_server = 100;
    dummy_client->negotiated_settings.retain_available = true;
}

static void s_invalid_retain_init_settings_failure_fn(struct aws_mqtt5_client *dummy_client) {
    dummy_client->negotiated_settings.retain_available = false;
}

AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST3(
    publish,
    invalid_retain,
    s_invalid_retain_publish_view,
    s_invalid_retain_init_settings_success_fn,
    s_invalid_retain_init_settings_failure_fn)

#define AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST4(                                                              \
    packet_type, failure_reason, view_name, init_success_settings_fn, init_failure_settings_fn)                        \
    AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST_PREFIX(packet_type, failure_reason, init_success_settings_fn)      \
    struct aws_mqtt5_operation_##packet_type *operation =                                                              \
        aws_mqtt5_operation_##packet_type##_new(allocator, &view_name, NULL, NULL);                                    \
    AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST_SUFFIX(packet_type, failure_reason, init_failure_settings_fn)

static uint8_t s_disconnect_reason_string[] = "We're leaving this planet for somewhere else";
static struct aws_byte_cursor s_disconnect_reason_string_cursor = {
    .ptr = s_disconnect_reason_string,
    .len = AWS_ARRAY_SIZE(s_disconnect_reason_string) - 1,
};

static struct aws_mqtt5_packet_disconnect_view s_exceeds_maximum_packet_size_disconnect_view = {
    .reason_string = &s_disconnect_reason_string_cursor,
};

AWS_CONNECTION_SETTINGS_VALIDATION_FAILURE_TEST4(
    disconnect,
    exceeds_maximum_packet_size,
    s_exceeds_maximum_packet_size_disconnect_view,
    s_packet_size_init_settings_success_fn,
    s_packet_size_init_settings_failure_fn)

static const uint32_t s_positive_session_expiry = 1;
static const uint32_t s_session_expiry = 5;

static struct aws_mqtt5_packet_disconnect_view s_promote_zero_session_expiry_disconnect_view = {
    .session_expiry_interval_seconds = &s_session_expiry,
};

static int mqtt5_operation_disconnect_connection_settings_validation_failure_promote_zero_session_expiry_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_event_loop_group *elg = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct aws_host_resolver_default_options hr_options = {
        .el_group = elg,
        .max_entries = 1,
    };
    struct aws_host_resolver *hr = aws_host_resolver_new_default(allocator, &hr_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = elg,
        .host_resolver = hr,
    };
    struct aws_client_bootstrap *bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    struct aws_mqtt5_client_options client_options = s_good_client_options;
    client_options.host_name = s_server_reference_cursor;
    client_options.bootstrap = bootstrap;

    struct aws_mqtt5_packet_connect_view connect_options = s_good_connect;
    connect_options.session_expiry_interval_seconds = &s_positive_session_expiry;
    client_options.connect_options = &connect_options;

    struct aws_mqtt5_client_options_storage *client_options_storage =
        aws_mqtt5_client_options_storage_new(allocator, &client_options);
    ASSERT_NOT_NULL(client_options_storage);

    struct aws_mqtt5_client dummy_client;
    AWS_ZERO_STRUCT(dummy_client);

    dummy_client.current_state = AWS_MCS_CONNECTED;
    dummy_client.negotiated_settings.maximum_packet_size_to_server = 100;
    dummy_client.config = client_options_storage;

    struct aws_mqtt5_operation_disconnect *operation =
        aws_mqtt5_operation_disconnect_new(allocator, &s_promote_zero_session_expiry_disconnect_view, NULL, NULL);

    ASSERT_SUCCESS(aws_mqtt5_operation_validate_vs_connection_settings(&operation->base, &dummy_client));

    ((struct aws_mqtt5_client_options_storage *)dummy_client.config)
        ->connect->storage_view.session_expiry_interval_seconds = NULL;

    ASSERT_FAILS(aws_mqtt5_operation_validate_vs_connection_settings(&operation->base, &dummy_client));

    aws_mqtt5_operation_release(&operation->base);
    aws_mqtt5_client_options_storage_destroy(client_options_storage);

    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(hr);
    aws_event_loop_group_release(elg);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_operation_disconnect_connection_settings_validation_failure_promote_zero_session_expiry,
    mqtt5_operation_disconnect_connection_settings_validation_failure_promote_zero_session_expiry_fn)
