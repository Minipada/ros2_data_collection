/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "aws/mqtt/private/mqtt_subscription_set.h"
#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/client_impl_shared.h>

#include <aws/testing/aws_test_harness.h>

struct subscription_test_context_callback_record {
    struct aws_allocator *allocator;

    struct aws_byte_cursor topic;
    struct aws_byte_buf topic_buffer;

    size_t callback_count;
};

static struct subscription_test_context_callback_record *s_subscription_test_context_callback_record_new(
    struct aws_allocator *allocator,
    struct aws_byte_cursor topic) {
    struct subscription_test_context_callback_record *record =
        aws_mem_calloc(allocator, 1, sizeof(struct subscription_test_context_callback_record));
    record->allocator = allocator;
    record->callback_count = 1;

    aws_byte_buf_init_copy_from_cursor(&record->topic_buffer, allocator, topic);
    record->topic = aws_byte_cursor_from_buf(&record->topic_buffer);

    return record;
}

static void s_subscription_test_context_callback_record_destroy(
    struct subscription_test_context_callback_record *record) {
    if (record == NULL) {
        return;
    }

    aws_byte_buf_clean_up(&record->topic_buffer);

    aws_mem_release(record->allocator, record);
}

static void s_destroy_callback_record(void *element) {
    struct subscription_test_context_callback_record *record = element;

    s_subscription_test_context_callback_record_destroy(record);
}

struct aws_mqtt_subscription_set_test_context {
    struct aws_allocator *allocator;

    struct aws_hash_table callbacks;
};

static void s_aws_mqtt_subscription_set_test_context_init(
    struct aws_mqtt_subscription_set_test_context *context,
    struct aws_allocator *allocator) {
    context->allocator = allocator;

    aws_hash_table_init(
        &context->callbacks,
        allocator,
        10,
        aws_hash_byte_cursor_ptr,
        aws_mqtt_byte_cursor_hash_equality,
        NULL,
        s_destroy_callback_record);
}

static void s_aws_mqtt_subscription_set_test_context_clean_up(struct aws_mqtt_subscription_set_test_context *context) {
    aws_hash_table_clean_up(&context->callbacks);
}

static void s_aws_mqtt_subscription_set_test_context_record_callback(
    struct aws_mqtt_subscription_set_test_context *context,
    struct aws_byte_cursor topic) {

    struct aws_hash_element *element = NULL;
    aws_hash_table_find(&context->callbacks, &topic, &element);

    if (element == NULL) {
        struct subscription_test_context_callback_record *record =
            s_subscription_test_context_callback_record_new(context->allocator, topic);
        aws_hash_table_put(&context->callbacks, &record->topic, record, NULL);
    } else {
        struct subscription_test_context_callback_record *record = element->value;
        ++record->callback_count;
    }
}

static int s_aws_mqtt_subscription_set_test_context_validate_callbacks(
    struct aws_mqtt_subscription_set_test_context *context,
    struct subscription_test_context_callback_record *expected_records,
    size_t expected_record_count) {
    ASSERT_INT_EQUALS(expected_record_count, aws_hash_table_get_entry_count(&context->callbacks));

    for (size_t i = 0; i < expected_record_count; ++i) {
        struct subscription_test_context_callback_record *expected_record = expected_records + i;

        struct aws_hash_element *element = NULL;
        aws_hash_table_find(&context->callbacks, &expected_record->topic, &element);

        ASSERT_TRUE(element != NULL);
        ASSERT_TRUE(element->value != NULL);

        struct subscription_test_context_callback_record *actual_record = element->value;

        ASSERT_INT_EQUALS(expected_record->callback_count, actual_record->callback_count);
    }

    return AWS_OP_SUCCESS;
}

static void s_subscription_set_test_on_publish_received(
    struct aws_mqtt_client_connection *connection,
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *userdata) {
    (void)connection;
    (void)payload;
    (void)qos;
    (void)dup;
    (void)retain;

    struct aws_mqtt_subscription_set_test_context *context = userdata;

    s_aws_mqtt_subscription_set_test_context_record_callback(context, *topic);
}

enum subscription_set_operation_type {
    SSOT_ADD,
    SSOT_REMOVE,
    SSOT_PUBLISH,
};

struct subscription_set_operation {
    enum subscription_set_operation_type type;

    const char *topic_filter;

    const char *topic;
};

static void s_subscription_set_perform_operations(
    struct aws_mqtt_subscription_set_test_context *context,
    struct aws_mqtt_subscription_set *subscription_set,
    struct subscription_set_operation *operations,
    size_t operation_count) {
    for (size_t i = 0; i < operation_count; ++i) {
        struct subscription_set_operation *operation = operations + i;

        switch (operation->type) {
            case SSOT_ADD: {
                struct aws_mqtt_subscription_set_subscription_options subscription_options = {
                    .topic_filter = aws_byte_cursor_from_c_str(operation->topic_filter),
                    .callback_user_data = context,
                    .on_publish_received = s_subscription_set_test_on_publish_received,
                };
                aws_mqtt_subscription_set_add_subscription(subscription_set, &subscription_options);
                break;
            }

            case SSOT_REMOVE:
                aws_mqtt_subscription_set_remove_subscription(
                    subscription_set, aws_byte_cursor_from_c_str(operation->topic_filter));
                break;

            case SSOT_PUBLISH: {
                struct aws_mqtt_subscription_set_publish_received_options publish_options = {
                    .topic = aws_byte_cursor_from_c_str(operation->topic),
                };
                aws_mqtt_subscription_set_on_publish_received(subscription_set, &publish_options);
                break;
            }
        }
    }
}

static int s_mqtt_subscription_set_add_empty_not_subbed_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("/")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("abc")));

    aws_mqtt_subscription_set_destroy(subscription_set);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_add_empty_not_subbed, s_mqtt_subscription_set_add_empty_not_subbed_fn)

static int s_mqtt_subscription_set_add_single_path_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {
            .type = SSOT_ADD,
            .topic_filter = "a/b/c",
        },
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));

    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("/")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("abc")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/")));

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_add_single_path, s_mqtt_subscription_set_add_single_path_fn)

static int s_mqtt_subscription_set_add_overlapped_branching_paths_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "a/+/c"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c"},
        {.type = SSOT_ADD, .topic_filter = "+/b/c"},
        {.type = SSOT_ADD, .topic_filter = "+/+/+"},
        {.type = SSOT_ADD, .topic_filter = "/"},
        {.type = SSOT_ADD, .topic_filter = " "},
        {.type = SSOT_ADD, .topic_filter = "a"},
        {.type = SSOT_ADD, .topic_filter = "#"},
        {.type = SSOT_ADD, .topic_filter = "a/#"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c/d/e/f/g"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c/"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/+/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/+")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("/")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str(" ")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/#")));
    ASSERT_TRUE(
        aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e/f/g")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/")));

    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/+")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/+/b")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/c")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d")));
    ASSERT_FALSE(
        aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e/f")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/b/b")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/+/+")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str(" /")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("/ ")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("b")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/")));
    ASSERT_FALSE(
        aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e/f/g/")));

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt_subscription_set_add_overlapped_branching_paths,
    s_mqtt_subscription_set_add_overlapped_branching_paths_fn)

static int s_mqtt_subscription_set_remove_overlapping_path_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "a/b/c/d"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c"},
        {.type = SSOT_ADD, .topic_filter = "a/b"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c/d/e"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("a/b/c"));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d"));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("a/b"));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e"));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c/d/e")));

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_remove_overlapping_path, s_mqtt_subscription_set_remove_overlapping_path_fn)

static int s_mqtt_subscription_set_remove_branching_path_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "+/+/#"},
        {.type = SSOT_ADD, .topic_filter = "#"},
        {.type = SSOT_ADD, .topic_filter = "+/b/c"},
        {.type = SSOT_ADD, .topic_filter = "+/+/c"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/#")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/c")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("+/b/c"));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/#")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/c")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("#"));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/c")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("+/+/#"));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/b/c")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/c")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("+/+/c"));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/b/c")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("+/+/c")));

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_remove_branching_path, s_mqtt_subscription_set_remove_branching_path_fn)

static int s_mqtt_subscription_set_remove_invalid_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "a/b/c"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("+/b/c"));
    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("a"));
    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("a/b"));
    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("#"));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("a/b/c"));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_remove_invalid, s_mqtt_subscription_set_remove_invalid_fn)

static int s_mqtt_subscription_set_remove_empty_segments_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    // Add '///' Subbed, Remove '///'  NotSubbed
    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "///"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("///")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("////")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("//")));

    aws_mqtt_subscription_set_remove_subscription(subscription_set, aws_byte_cursor_from_c_str("///"));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(subscription_set, aws_byte_cursor_from_c_str("///")));

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_remove_empty_segments, s_mqtt_subscription_set_remove_empty_segments_fn)

static int s_mqtt_subscription_set_add_remove_repeated_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    const char *topic_filters[] = {"+/+/#", "#", "+/+/c/d", "+/b/c", "+/+/c"};
    size_t filter_count = AWS_ARRAY_SIZE(topic_filters);

    for (size_t i = 0; i < filter_count; ++i) {
        for (size_t j = 0; j < filter_count; ++j) {
            /*
             * this does not cover all permutations (n!) but lacking a permutation generator, this gets "reasonable"
             * coverage (n * n)
             */

            /* Add the topic filters in a shifting sequence */
            for (size_t add_index = 0; add_index < filter_count; ++add_index) {
                size_t final_index = (add_index + j) % filter_count;

                struct aws_mqtt_subscription_set_subscription_options subscription_options = {
                    .topic_filter = aws_byte_cursor_from_c_str(topic_filters[final_index]),
                };
                aws_mqtt_subscription_set_add_subscription(subscription_set, &subscription_options);
            }

            /* One-by-one, remove the topic filters in an independent shifting sequence */
            for (size_t remove_index = 0; remove_index < filter_count; ++remove_index) {
                size_t final_remove_index = (remove_index + i) % filter_count;

                aws_mqtt_subscription_set_remove_subscription(
                    subscription_set, aws_byte_cursor_from_c_str(topic_filters[final_remove_index]));

                for (size_t validate_index = 0; validate_index < filter_count; ++validate_index) {
                    size_t final_validate_index = (validate_index + i) % filter_count;
                    if (validate_index <= remove_index) {
                        ASSERT_FALSE(aws_mqtt_subscription_set_is_in_topic_tree(
                            subscription_set, aws_byte_cursor_from_c_str(topic_filters[final_validate_index])));
                    } else {
                        ASSERT_TRUE(aws_mqtt_subscription_set_is_in_topic_tree(
                            subscription_set, aws_byte_cursor_from_c_str(topic_filters[final_validate_index])));
                    }
                }
            }
        }
    }

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_add_remove_repeated, s_mqtt_subscription_set_add_remove_repeated_fn)

static int s_mqtt_subscription_set_publish_single_path_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "a/b/c/d"},
        {.type = SSOT_ADD, .topic_filter = "a"},
        {.type = SSOT_ADD, .topic_filter = "a/b"},
        {.type = SSOT_PUBLISH, .topic = "a/b/c"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));

    struct subscription_test_context_callback_record expected_callback_records[] = {
        {.topic = aws_byte_cursor_from_c_str("a"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("a/b/c/d"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("a/b"), .callback_count = 1},
    };
    ASSERT_SUCCESS(s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, expected_callback_records, 0));

    for (size_t i = 0; i < AWS_ARRAY_SIZE(expected_callback_records); ++i) {
        struct aws_mqtt_subscription_set_publish_received_options publish_options = {
            .topic = expected_callback_records[i].topic,
        };
        aws_mqtt_subscription_set_on_publish_received(subscription_set, &publish_options);

        ASSERT_SUCCESS(
            s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, expected_callback_records, i + 1));
    }

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_publish_single_path, s_mqtt_subscription_set_publish_single_path_fn)

static int s_mqtt_subscription_set_publish_multi_path_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "a/b/c/d"},
        {.type = SSOT_ADD, .topic_filter = "a/b/d"},
        {.type = SSOT_ADD, .topic_filter = "b"},
        {.type = SSOT_ADD, .topic_filter = "c/d"},
        {.type = SSOT_ADD, .topic_filter = "a/c"},
        {.type = SSOT_PUBLISH, .topic = "a"},
        {.type = SSOT_PUBLISH, .topic = "a/b"},
        {.type = SSOT_PUBLISH, .topic = "a/b/c"},
        {.type = SSOT_PUBLISH, .topic = "a/b/c/d/"},
        {.type = SSOT_PUBLISH, .topic = "b/c/d/"},
        {.type = SSOT_PUBLISH, .topic = "c"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));

    struct subscription_test_context_callback_record expected_callback_records[] = {
        {.topic = aws_byte_cursor_from_c_str("a/b/d"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("c/d"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("a/b/c/d"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("b"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("a/c"), .callback_count = 1},
    };

    ASSERT_SUCCESS(s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, NULL, 0));

    for (size_t i = 0; i < AWS_ARRAY_SIZE(expected_callback_records); ++i) {
        struct aws_mqtt_subscription_set_publish_received_options publish_options = {
            .topic = expected_callback_records[i].topic,
        };
        aws_mqtt_subscription_set_on_publish_received(subscription_set, &publish_options);

        ASSERT_SUCCESS(
            s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, expected_callback_records, i + 1));
    }

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_publish_multi_path, s_mqtt_subscription_set_publish_multi_path_fn)

static int s_mqtt_subscription_set_publish_single_level_wildcards_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "a/b/c"},
        {.type = SSOT_ADD, .topic_filter = "+/b/c"},
        {.type = SSOT_ADD, .topic_filter = "a/+/c"},
        {.type = SSOT_ADD, .topic_filter = "a/b/+"},
        {.type = SSOT_ADD, .topic_filter = "+/+/+"},
        {.type = SSOT_ADD, .topic_filter = "a/+/+"},
        {.type = SSOT_PUBLISH, .topic = "a"},
        {.type = SSOT_PUBLISH, .topic = "a/b"},
        {.type = SSOT_PUBLISH, .topic = "a/b/c/d"},
        {.type = SSOT_PUBLISH, .topic = "a/b/c/d/"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));

    struct subscription_test_context_callback_record expected_callback_records[] = {
        {.topic = aws_byte_cursor_from_c_str("a/b/d"), .callback_count = 3},
        {.topic = aws_byte_cursor_from_c_str("b/c/d"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("a/c/d"), .callback_count = 2},
        {.topic = aws_byte_cursor_from_c_str("c/b/c"), .callback_count = 2},
        {.topic = aws_byte_cursor_from_c_str("a/b/c"), .callback_count = 6},
    };

    ASSERT_SUCCESS(s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, NULL, 0));

    for (size_t i = 0; i < AWS_ARRAY_SIZE(expected_callback_records); ++i) {
        struct aws_mqtt_subscription_set_publish_received_options publish_options = {
            .topic = expected_callback_records[i].topic,
        };
        aws_mqtt_subscription_set_on_publish_received(subscription_set, &publish_options);

        ASSERT_SUCCESS(
            s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, expected_callback_records, i + 1));
    }

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt_subscription_set_publish_single_level_wildcards,
    s_mqtt_subscription_set_publish_single_level_wildcards_fn)

static int s_mqtt_subscription_set_publish_multi_level_wildcards_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "#"},
        {.type = SSOT_ADD, .topic_filter = "a/#"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c/#"},
        {.type = SSOT_ADD, .topic_filter = "/#"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));

    struct subscription_test_context_callback_record expected_callback_records[] = {
        {.topic = aws_byte_cursor_from_c_str("b"), .callback_count = 1},
        {.topic = aws_byte_cursor_from_c_str("/"), .callback_count = 2},
        {.topic = aws_byte_cursor_from_c_str("a"), .callback_count = 2},
        {.topic = aws_byte_cursor_from_c_str("a/b"), .callback_count = 2},
        {.topic = aws_byte_cursor_from_c_str("a/b/c"), .callback_count = 3},
        {.topic = aws_byte_cursor_from_c_str("a/b/c/d"), .callback_count = 3},
        {.topic = aws_byte_cursor_from_c_str("/x/y/z"), .callback_count = 2},
    };

    ASSERT_SUCCESS(s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, NULL, 0));

    for (size_t i = 0; i < AWS_ARRAY_SIZE(expected_callback_records); ++i) {
        struct aws_mqtt_subscription_set_publish_received_options publish_options = {
            .topic = expected_callback_records[i].topic,
        };
        aws_mqtt_subscription_set_on_publish_received(subscription_set, &publish_options);

        ASSERT_SUCCESS(
            s_aws_mqtt_subscription_set_test_context_validate_callbacks(&context, expected_callback_records, i + 1));
    }

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    mqtt_subscription_set_publish_multi_level_wildcards,
    s_mqtt_subscription_set_publish_multi_level_wildcards_fn)

static int s_mqtt_subscription_set_get_subscriptions_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_subscription_set_test_context context;
    s_aws_mqtt_subscription_set_test_context_init(&context, allocator);

    struct aws_mqtt_subscription_set *subscription_set = aws_mqtt_subscription_set_new(allocator);

    struct subscription_set_operation operations[] = {
        {.type = SSOT_ADD, .topic_filter = "#"},
        {.type = SSOT_ADD, .topic_filter = "a/#"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c/#"},
        {.type = SSOT_ADD, .topic_filter = "/#"},
        {.type = SSOT_ADD, .topic_filter = "/"},
        {.type = SSOT_ADD, .topic_filter = "a/b/c"},
        {.type = SSOT_ADD, .topic_filter = "a/#"},
        {.type = SSOT_REMOVE, .topic_filter = "/#"},
        {.type = SSOT_REMOVE, .topic_filter = "/"},
    };

    s_subscription_set_perform_operations(&context, subscription_set, operations, AWS_ARRAY_SIZE(operations));

    ASSERT_TRUE(aws_mqtt_subscription_set_is_subscribed(subscription_set, aws_byte_cursor_from_c_str("#")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_subscribed(subscription_set, aws_byte_cursor_from_c_str("a/#")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_subscribed(subscription_set, aws_byte_cursor_from_c_str("a/b/c/#")));
    ASSERT_TRUE(aws_mqtt_subscription_set_is_subscribed(subscription_set, aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_subscribed(subscription_set, aws_byte_cursor_from_c_str("/#")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_subscribed(subscription_set, aws_byte_cursor_from_c_str("/")));
    ASSERT_FALSE(aws_mqtt_subscription_set_is_subscribed(subscription_set, aws_byte_cursor_from_c_str("a")));

    struct aws_array_list subscriptions;
    aws_mqtt_subscription_set_get_subscriptions(subscription_set, &subscriptions);

    size_t subscription_count = aws_array_list_length(&subscriptions);
    ASSERT_INT_EQUALS(4, subscription_count);
    for (size_t i = 0; i < subscription_count; ++i) {
        struct aws_mqtt_subscription_set_subscription_options subscription;
        aws_array_list_get_at(&subscriptions, &subscription, i);

        ASSERT_TRUE(aws_mqtt_subscription_set_is_subscribed(subscription_set, subscription.topic_filter));
    }

    aws_array_list_clean_up(&subscriptions);

    aws_mqtt_subscription_set_destroy(subscription_set);

    s_aws_mqtt_subscription_set_test_context_clean_up(&context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_subscription_set_get_subscriptions, s_mqtt_subscription_set_get_subscriptions_fn)