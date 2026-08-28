/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/v5/mqtt5_topic_alias.h>

#include <aws/common/byte_buf.h>
#include <aws/common/string.h>

#include <aws/testing/aws_test_harness.h>

AWS_STATIC_STRING_FROM_LITERAL(s_topic1, "hello/world");
AWS_STATIC_STRING_FROM_LITERAL(s_topic2, "this/is/a/longer/topic");

static int s_mqtt5_inbound_topic_alias_register_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_inbound_topic_alias_resolver resolver;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_init(&resolver, allocator));

    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_reset(&resolver, 10));

    ASSERT_FAILS(
        aws_mqtt5_inbound_topic_alias_resolver_register_alias(&resolver, 0, aws_byte_cursor_from_string(s_topic1)));
    ASSERT_FAILS(
        aws_mqtt5_inbound_topic_alias_resolver_register_alias(&resolver, 11, aws_byte_cursor_from_string(s_topic1)));

    aws_mqtt5_inbound_topic_alias_resolver_clean_up(&resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_inbound_topic_alias_register_failure, s_mqtt5_inbound_topic_alias_register_failure_fn)

static int s_mqtt5_inbound_topic_alias_resolve_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_inbound_topic_alias_resolver resolver;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_init(&resolver, allocator));

    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_reset(&resolver, 10));

    ASSERT_SUCCESS(
        aws_mqtt5_inbound_topic_alias_resolver_register_alias(&resolver, 1, aws_byte_cursor_from_string(s_topic1)));
    ASSERT_SUCCESS(
        aws_mqtt5_inbound_topic_alias_resolver_register_alias(&resolver, 10, aws_byte_cursor_from_string(s_topic2)));

    struct aws_byte_cursor topic1;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 1, &topic1));
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, topic1.ptr, topic1.len);

    struct aws_byte_cursor topic2;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 10, &topic2));
    ASSERT_BIN_ARRAYS_EQUALS(s_topic2->bytes, s_topic2->len, topic2.ptr, topic2.len);

    /* overwrite an existing alias to verify memory is cleaned up */
    ASSERT_SUCCESS(
        aws_mqtt5_inbound_topic_alias_resolver_register_alias(&resolver, 10, aws_byte_cursor_from_string(s_topic1)));

    struct aws_byte_cursor topic3;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 10, &topic3));
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, topic3.ptr, topic3.len);

    aws_mqtt5_inbound_topic_alias_resolver_clean_up(&resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_inbound_topic_alias_resolve_success, s_mqtt5_inbound_topic_alias_resolve_success_fn)

static int s_mqtt5_inbound_topic_alias_resolve_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_inbound_topic_alias_resolver resolver;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_init(&resolver, allocator));

    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_reset(&resolver, 10));

    struct aws_byte_cursor topic;
    ASSERT_FAILS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 0, &topic));
    ASSERT_FAILS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 11, &topic));
    ASSERT_FAILS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 10, &topic));

    aws_mqtt5_inbound_topic_alias_resolver_clean_up(&resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_inbound_topic_alias_resolve_failure, s_mqtt5_inbound_topic_alias_resolve_failure_fn)

static int s_mqtt5_inbound_topic_alias_reset_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_inbound_topic_alias_resolver resolver;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_init(&resolver, allocator));

    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_reset(&resolver, 10));

    ASSERT_SUCCESS(
        aws_mqtt5_inbound_topic_alias_resolver_register_alias(&resolver, 1, aws_byte_cursor_from_string(s_topic1)));

    struct aws_byte_cursor topic;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 1, &topic));
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, topic.ptr, topic.len);

    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_reset(&resolver, 10));

    ASSERT_FAILS(aws_mqtt5_inbound_topic_alias_resolver_resolve_alias(&resolver, 1, &topic));

    aws_mqtt5_inbound_topic_alias_resolver_clean_up(&resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_inbound_topic_alias_reset, s_mqtt5_inbound_topic_alias_reset_fn)

static int s_mqtt5_outbound_topic_alias_disabled_resolve_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_DISABLED);
    ASSERT_NOT_NULL(resolver);

    struct aws_mqtt5_packet_publish_view publish_view = {.topic = aws_byte_cursor_from_string(s_topic1)};

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(0, outbound_alias_id);
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, outbound_topic.ptr, outbound_topic.len);

    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, 0));

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_outbound_topic_alias_disabled_resolve_success,
    s_mqtt5_outbound_topic_alias_disabled_resolve_success_fn)

static int s_mqtt5_outbound_topic_alias_disabled_resolve_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_DISABLED);
    ASSERT_NOT_NULL(resolver);

    struct aws_mqtt5_packet_publish_view publish_view = {
        .topic =
            {
                .ptr = NULL,
                .len = 0,
            },
    };

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    ASSERT_FAILS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_outbound_topic_alias_disabled_resolve_failure,
    s_mqtt5_outbound_topic_alias_disabled_resolve_failure_fn)

static int s_mqtt5_outbound_topic_alias_manual_resolve_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_MANUAL);
    ASSERT_NOT_NULL(resolver);

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, 5);

    struct aws_mqtt5_packet_publish_view publish_view1 = {
        .topic = aws_byte_cursor_from_string(s_topic1),
    };

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    /* no alias case */
    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view1, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(0, outbound_alias_id);
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, outbound_topic.ptr, outbound_topic.len);

    uint16_t alias = 1;
    struct aws_mqtt5_packet_publish_view publish_view2 = {
        .topic = aws_byte_cursor_from_string(s_topic1),
        .topic_alias = &alias,
    };

    outbound_alias_id = 0;
    AWS_ZERO_STRUCT(outbound_topic);

    /* new valid alias assignment case */
    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view2, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(1, outbound_alias_id);
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, outbound_topic.ptr, outbound_topic.len);

    struct aws_mqtt5_packet_publish_view publish_view3 = {
        .topic = aws_byte_cursor_from_string(s_topic1),
        .topic_alias = &alias,
    };

    outbound_alias_id = 0;
    AWS_ZERO_STRUCT(outbound_topic);

    /* reuse valid alias assignment case */
    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view3, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(1, outbound_alias_id);
    ASSERT_INT_EQUALS(0, outbound_topic.len);

    /* switch topics but keep the alias, we should resolve to a full binding with the new topic */
    struct aws_mqtt5_packet_publish_view publish_view4 = {
        .topic = aws_byte_cursor_from_string(s_topic2),
        .topic_alias = &alias,
    };

    outbound_alias_id = 0;
    AWS_ZERO_STRUCT(outbound_topic);

    /* reuse valid alias assignment case */
    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view4, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(1, outbound_alias_id);
    ASSERT_BIN_ARRAYS_EQUALS(s_topic2->bytes, s_topic2->len, outbound_topic.ptr, outbound_topic.len);

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_manual_resolve_success, s_mqtt5_outbound_topic_alias_manual_resolve_success_fn)

static int s_mqtt5_outbound_topic_alias_manual_resolve_failure_zero_alias_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_MANUAL);
    ASSERT_NOT_NULL(resolver);

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, 5);

    uint16_t alias = 0;
    struct aws_mqtt5_packet_publish_view publish_view = {
        .topic = aws_byte_cursor_from_string(s_topic1),
        .topic_alias = &alias,
    };

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    ASSERT_FAILS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_outbound_topic_alias_manual_resolve_failure_zero_alias,
    s_mqtt5_outbound_topic_alias_manual_resolve_failure_zero_alias_fn)

static int s_mqtt5_outbound_topic_alias_manual_resolve_failure_too_big_alias_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_MANUAL);
    ASSERT_NOT_NULL(resolver);

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, 5);

    uint16_t alias = 6;
    struct aws_mqtt5_packet_publish_view publish_view = {
        .topic = aws_byte_cursor_from_string(s_topic1),
        .topic_alias = &alias,
    };

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    ASSERT_FAILS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_outbound_topic_alias_manual_resolve_failure_too_big_alias,
    s_mqtt5_outbound_topic_alias_manual_resolve_failure_too_big_alias_fn)

static int s_mqtt5_outbound_topic_alias_manual_reset_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_MANUAL);
    ASSERT_NOT_NULL(resolver);

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, 5);

    uint16_t alias = 2;
    struct aws_mqtt5_packet_publish_view publish_view = {
        .topic = aws_byte_cursor_from_string(s_topic1),
        .topic_alias = &alias,
    };

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    /* First, successfully bind an alias */
    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(2, outbound_alias_id);
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, outbound_topic.ptr, outbound_topic.len);

    /* Successfully use the alias */
    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(2, outbound_alias_id);
    ASSERT_INT_EQUALS(0, outbound_topic.len);

    /* Reset */
    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, 5);

    /* Fail to reuse the alias */
    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(2, outbound_alias_id);
    ASSERT_BIN_ARRAYS_EQUALS(s_topic1->bytes, s_topic1->len, outbound_topic.ptr, outbound_topic.len);

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_manual_reset, s_mqtt5_outbound_topic_alias_manual_reset_fn)

static int s_mqtt5_outbound_topic_alias_lru_zero_size_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_LRU);
    ASSERT_NOT_NULL(resolver);

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, 0);

    struct aws_mqtt5_packet_publish_view publish_view = {
        .topic = aws_byte_cursor_from_string(s_topic1),
    };

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(0, outbound_alias_id);
    ASSERT_BIN_ARRAYS_EQUALS(publish_view.topic.ptr, publish_view.topic.len, outbound_topic.ptr, outbound_topic.len);

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_lru_zero_size, s_mqtt5_outbound_topic_alias_lru_zero_size_fn)

#define LRU_SEQUENCE_TEST_CACHE_SIZE 2

struct lru_test_operation {
    struct aws_byte_cursor topic;
    size_t expected_alias_id;
    bool expected_reuse;
};

#define DEFINE_LRU_TEST_OPERATION(topic_type, alias_index, reused)                                                     \
    {                                                                                                                  \
        .topic = aws_byte_cursor_from_string(s_topic_##topic_type),                                                    \
        .expected_alias_id = alias_index,                                                                              \
        .expected_reuse = reused,                                                                                      \
    }

static int s_perform_lru_test_operation(
    struct aws_mqtt5_outbound_topic_alias_resolver *resolver,
    struct lru_test_operation *operation) {

    struct aws_mqtt5_packet_publish_view publish_view = {
        .topic = operation->topic,
    };

    uint16_t outbound_alias_id = 0;
    struct aws_byte_cursor outbound_topic;
    AWS_ZERO_STRUCT(outbound_topic);

    ASSERT_SUCCESS(aws_mqtt5_outbound_topic_alias_resolver_resolve_outbound_publish(
        resolver, &publish_view, &outbound_alias_id, &outbound_topic));

    ASSERT_INT_EQUALS(operation->expected_alias_id, outbound_alias_id);
    if (operation->expected_reuse) {
        ASSERT_INT_EQUALS(0, outbound_topic.len);
    } else {
        ASSERT_BIN_ARRAYS_EQUALS(operation->topic.ptr, operation->topic.len, outbound_topic.ptr, outbound_topic.len);
    }

    return AWS_OP_SUCCESS;
}

static int s_check_lru_sequence(
    struct aws_mqtt5_outbound_topic_alias_resolver *resolver,
    struct lru_test_operation *operations,
    size_t operation_count) {
    for (size_t i = 0; i < operation_count; ++i) {
        struct lru_test_operation *operation = &operations[i];
        ASSERT_SUCCESS(s_perform_lru_test_operation(resolver, operation));
    }

    return AWS_OP_SUCCESS;
}

static int s_perform_lru_sequence_test(
    struct aws_allocator *allocator,
    struct lru_test_operation *operations,
    size_t operation_count) {
    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_LRU);
    ASSERT_NOT_NULL(resolver);

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, LRU_SEQUENCE_TEST_CACHE_SIZE);

    ASSERT_SUCCESS(s_check_lru_sequence(resolver, operations, operation_count));

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_STATIC_STRING_FROM_LITERAL(s_topic_a, "topic/a");
AWS_STATIC_STRING_FROM_LITERAL(s_topic_b, "b/topic");
AWS_STATIC_STRING_FROM_LITERAL(s_topic_c, "topic/c");

static int s_mqtt5_outbound_topic_alias_lru_a_ar_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct lru_test_operation test_operations[] = {
        DEFINE_LRU_TEST_OPERATION(a, 1, false),
        DEFINE_LRU_TEST_OPERATION(a, 1, true),
    };

    ASSERT_SUCCESS(s_perform_lru_sequence_test(allocator, test_operations, AWS_ARRAY_SIZE(test_operations)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_lru_a_ar, s_mqtt5_outbound_topic_alias_lru_a_ar_fn)

static int s_mqtt5_outbound_topic_alias_lru_b_a_br_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct lru_test_operation test_operations[] = {
        DEFINE_LRU_TEST_OPERATION(b, 1, false),
        DEFINE_LRU_TEST_OPERATION(a, 2, false),
        DEFINE_LRU_TEST_OPERATION(b, 1, true),
    };

    ASSERT_SUCCESS(s_perform_lru_sequence_test(allocator, test_operations, AWS_ARRAY_SIZE(test_operations)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_lru_b_a_br, s_mqtt5_outbound_topic_alias_lru_b_a_br_fn)

static int s_mqtt5_outbound_topic_alias_lru_a_b_ar_br_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct lru_test_operation test_operations[] = {
        DEFINE_LRU_TEST_OPERATION(a, 1, false),
        DEFINE_LRU_TEST_OPERATION(b, 2, false),
        DEFINE_LRU_TEST_OPERATION(a, 1, true),
        DEFINE_LRU_TEST_OPERATION(b, 2, true),
    };

    ASSERT_SUCCESS(s_perform_lru_sequence_test(allocator, test_operations, AWS_ARRAY_SIZE(test_operations)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_lru_a_b_ar_br, s_mqtt5_outbound_topic_alias_lru_a_b_ar_br_fn)

static int s_mqtt5_outbound_topic_alias_lru_a_b_c_br_cr_br_cr_a_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct lru_test_operation test_operations[] = {
        DEFINE_LRU_TEST_OPERATION(a, 1, false),
        DEFINE_LRU_TEST_OPERATION(b, 2, false),
        DEFINE_LRU_TEST_OPERATION(c, 1, false),
        DEFINE_LRU_TEST_OPERATION(b, 2, true),
        DEFINE_LRU_TEST_OPERATION(c, 1, true),
        DEFINE_LRU_TEST_OPERATION(b, 2, true),
        DEFINE_LRU_TEST_OPERATION(c, 1, true),
        DEFINE_LRU_TEST_OPERATION(a, 2, false),
    };

    ASSERT_SUCCESS(s_perform_lru_sequence_test(allocator, test_operations, AWS_ARRAY_SIZE(test_operations)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt5_outbound_topic_alias_lru_a_b_c_br_cr_br_cr_a,
    s_mqtt5_outbound_topic_alias_lru_a_b_c_br_cr_br_cr_a_fn)

static int s_mqtt5_outbound_topic_alias_lru_a_b_c_a_cr_b_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct lru_test_operation test_operations[] = {
        DEFINE_LRU_TEST_OPERATION(a, 1, false),
        DEFINE_LRU_TEST_OPERATION(b, 2, false),
        DEFINE_LRU_TEST_OPERATION(c, 1, false),
        DEFINE_LRU_TEST_OPERATION(a, 2, false),
        DEFINE_LRU_TEST_OPERATION(c, 1, true),
        DEFINE_LRU_TEST_OPERATION(b, 2, false),
    };

    ASSERT_SUCCESS(s_perform_lru_sequence_test(allocator, test_operations, AWS_ARRAY_SIZE(test_operations)));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_lru_a_b_c_a_cr_b, s_mqtt5_outbound_topic_alias_lru_a_b_c_a_cr_b_fn)

static int s_mqtt5_outbound_topic_alias_lru_a_b_reset_a_b_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_outbound_topic_alias_resolver *resolver =
        aws_mqtt5_outbound_topic_alias_resolver_new(allocator, AWS_MQTT5_COTABT_LRU);
    ASSERT_NOT_NULL(resolver);

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, LRU_SEQUENCE_TEST_CACHE_SIZE);

    struct lru_test_operation test_operations[] = {
        DEFINE_LRU_TEST_OPERATION(a, 1, false),
        DEFINE_LRU_TEST_OPERATION(b, 2, false),
    };

    ASSERT_SUCCESS(s_check_lru_sequence(resolver, test_operations, AWS_ARRAY_SIZE(test_operations)));

    aws_mqtt5_outbound_topic_alias_resolver_reset(resolver, LRU_SEQUENCE_TEST_CACHE_SIZE);

    ASSERT_SUCCESS(s_check_lru_sequence(resolver, test_operations, AWS_ARRAY_SIZE(test_operations)));

    aws_mqtt5_outbound_topic_alias_resolver_destroy(resolver);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_outbound_topic_alias_lru_a_b_reset_a_b, s_mqtt5_outbound_topic_alias_lru_a_b_reset_a_b_fn)
