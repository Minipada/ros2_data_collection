/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/private/topic_tree.h>

#include <aws/testing/aws_test_harness.h>

#include <aws/common/string.h>

static struct aws_byte_cursor s_empty_cursor = {
    .ptr = NULL,
    .len = 0,
};

static int times_called = 0;
static void on_publish(
    const struct aws_byte_cursor *topic,
    const struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain,
    void *user_data) {

    (void)topic;
    (void)payload;
    (void)dup;
    (void)qos;
    (void)retain;
    (void)user_data;

    times_called++;
}

static void s_string_clean_up(void *userdata) {
    struct aws_string *string = userdata;
    aws_string_destroy(string);
}

/* Subscribes to multiple topics and returns the number that matched with the pub_topic */
static int s_check_multi_topic_match(
    struct aws_allocator *allocator,
    const char **sub_filters,
    size_t sub_filters_len,
    const char *pub_topic) {

    times_called = 0;

    struct aws_mqtt_topic_tree tree;
    aws_mqtt_topic_tree_init(&tree, allocator);

    for (size_t i = 0; i < sub_filters_len; ++i) {
        struct aws_string *topic_filter = aws_string_new_from_c_str(allocator, sub_filters[i]);
        aws_mqtt_topic_tree_insert(
            &tree, topic_filter, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_string_clean_up, topic_filter);
    }

    struct aws_byte_cursor filter_cursor = aws_byte_cursor_from_array(pub_topic, strlen(pub_topic));
    struct aws_mqtt_packet_publish publish;
    aws_mqtt_packet_publish_init(&publish, false, AWS_MQTT_QOS_AT_MOST_ONCE, false, filter_cursor, 1, s_empty_cursor);

    aws_mqtt_topic_tree_publish(&tree, &publish);

    aws_mqtt_topic_tree_clean_up(&tree);

    return times_called;
}
static bool s_check_topic_match(struct aws_allocator *allocator, const char *sub_filter, const char *pub_topic) {
    int matches = s_check_multi_topic_match(allocator, &sub_filter, 1, pub_topic);
    return matches == 1;
}

AWS_TEST_CASE(mqtt_topic_tree_match, s_mqtt_topic_tree_match_fn)
static int s_mqtt_topic_tree_match_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    /* Check single-level filters */
    ASSERT_TRUE(s_check_topic_match(allocator, "a", "a"));
    ASSERT_FALSE(s_check_topic_match(allocator, "b", "B"));

    /* Check multi-level filters */
    ASSERT_TRUE(s_check_topic_match(allocator, "a/b", "a/b"));
    ASSERT_FALSE(s_check_topic_match(allocator, "a/b", "a/B"));
    ASSERT_FALSE(s_check_topic_match(allocator, "a/b", "a/b/c"));
    ASSERT_FALSE(s_check_topic_match(allocator, "a/b/c", "a/b"));
    ASSERT_TRUE(s_check_topic_match(allocator, "a/b/c", "a/b/c"));
    ASSERT_FALSE(s_check_topic_match(allocator, "a/b/c", "a/B/c"));

    /* Check single-level wildcard filters */
    ASSERT_TRUE(s_check_topic_match(allocator, "sport/tennis/+", "sport/tennis/player1"));
    ASSERT_TRUE(s_check_topic_match(allocator, "sport/tennis/+", "sport/tennis/player2"));
    ASSERT_FALSE(s_check_topic_match(allocator, "sport/tennis/+", "sport/tennis/player1/ranking"));

    ASSERT_TRUE(s_check_topic_match(allocator, "sport/+", "sport/"));
    ASSERT_FALSE(s_check_topic_match(allocator, "sport/+", "sport"));

    ASSERT_TRUE(s_check_topic_match(allocator, "+/+", "/finance"));
    ASSERT_TRUE(s_check_topic_match(allocator, "/+", "/finance"));
    ASSERT_FALSE(s_check_topic_match(allocator, "+", "/finance"));

    ASSERT_TRUE(s_check_topic_match(allocator, "///", "///"));
    ASSERT_FALSE(s_check_topic_match(allocator, "///", "//"));

    const char *sub_topics[] = {"a/b/c", "a/+/c", "a/#"};
    ASSERT_INT_EQUALS(s_check_multi_topic_match(allocator, sub_topics, AWS_ARRAY_SIZE(sub_topics), "a/b/c"), 3);
    ASSERT_INT_EQUALS(s_check_multi_topic_match(allocator, sub_topics, AWS_ARRAY_SIZE(sub_topics), "a/Z/c"), 2);
    ASSERT_INT_EQUALS(s_check_multi_topic_match(allocator, sub_topics, AWS_ARRAY_SIZE(sub_topics), "a/b/Z"), 1);
    ASSERT_INT_EQUALS(s_check_multi_topic_match(allocator, sub_topics, AWS_ARRAY_SIZE(sub_topics), "Z/b/c"), 0);

    return AWS_OP_SUCCESS;
}

static struct aws_byte_cursor s_topic_a_a = {
    .ptr = (uint8_t *)"a/a",
    .len = 3,
};
static struct aws_byte_cursor s_topic_a_a_a = {
    .ptr = (uint8_t *)"a/a/a",
    .len = 5,
};
static struct aws_byte_cursor s_topic_a_a_b = {
    .ptr = (uint8_t *)"a/a/b",
    .len = 5,
};

AWS_TEST_CASE(mqtt_topic_tree_unsubscribe, s_mqtt_topic_tree_unsubscribe_fn)
static int s_mqtt_topic_tree_unsubscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt_topic_tree tree;
    ASSERT_SUCCESS(aws_mqtt_topic_tree_init(&tree, allocator));

    struct aws_string *topic_a_a = aws_string_new_from_array(allocator, s_topic_a_a.ptr, s_topic_a_a.len);
    struct aws_string *topic_a_a_a = aws_string_new_from_array(allocator, s_topic_a_a_a.ptr, s_topic_a_a_a.len);
    struct aws_string *topic_a_a_b = aws_string_new_from_array(allocator, s_topic_a_a_b.ptr, s_topic_a_a_b.len);

    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, transaction_buf, aws_mqtt_topic_tree_action_size * 3);
    struct aws_array_list transaction;
    aws_array_list_init_static(&transaction, transaction_buf, 3, aws_mqtt_topic_tree_action_size);

    ASSERT_SUCCESS(aws_mqtt_topic_tree_insert(
        &tree, topic_a_a_a, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_string_clean_up, topic_a_a_a));
    /* At this moment, the topic_a_a was not inserted. Though the remove returns a success, the topic_a_a_a should still
     * remained in the tree.
     * The test is inspired by the ticket: https://github.com/awslabs/aws-crt-nodejs/issues/405. There was a crash when
     * we unsubscribe from an unsubscribed parent topic.
     * We fixed the issue in https://github.com/awslabs/aws-c-mqtt/pull/297 */
    ASSERT_SUCCESS(aws_mqtt_topic_tree_remove(&tree, &s_topic_a_a));
    ASSERT_SUCCESS(aws_mqtt_topic_tree_remove(&tree, &s_topic_a_a_a));
    /* Re-create, it was nuked by remove. */
    topic_a_a_a = aws_string_new_from_array(allocator, s_topic_a_a_a.ptr, s_topic_a_a_a.len);
    /* Ensure that the intermediate 'a' node was removed as well. */
    ASSERT_UINT_EQUALS(0, aws_hash_table_get_entry_count(&tree.root->subtopics));

    /* Put it back so we can test removal of a partial tree. */
    /* Bonus points: test transactions here */
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree, &transaction, topic_a_a_a, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_string_clean_up, topic_a_a_a));
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree, &transaction, topic_a_a, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_string_clean_up, topic_a_a));
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree, &transaction, topic_a_a_b, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_string_clean_up, topic_a_a_b));
    aws_mqtt_topic_tree_transaction_commit(&tree, &transaction);

    /* Should remove the last /a, but not the first 2. */
    void *userdata = (void *)0xBADCAFE;
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_remove(&tree, &transaction, &s_topic_a_a_a, &userdata));
    /* Ensure userdata was set back to the right user_data correctly. */
    ASSERT_PTR_EQUALS(topic_a_a_a, userdata);
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_remove(&tree, &transaction, &s_topic_a_a, NULL));
    aws_mqtt_topic_tree_transaction_commit(&tree, &transaction);

    struct aws_mqtt_packet_publish publish;
    aws_mqtt_packet_publish_init(&publish, false, AWS_MQTT_QOS_AT_MOST_ONCE, false, s_topic_a_a_a, 1, s_topic_a_a_a);

    times_called = 0;
    aws_mqtt_topic_tree_publish(&tree, &publish);
    ASSERT_INT_EQUALS(times_called, 0);

    publish.topic_name = s_topic_a_a_b;

    times_called = 0;
    aws_mqtt_topic_tree_publish(&tree, &publish);
    ASSERT_INT_EQUALS(times_called, 1);

    const struct aws_byte_cursor not_in_tree = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("not/in/tree");
    aws_mqtt_topic_tree_remove(&tree, &not_in_tree);

    aws_mqtt_topic_tree_clean_up(&tree);
    return AWS_OP_SUCCESS;
}

struct s_duplicate_test_ud {
    struct aws_string *string;
    bool cleaned;
};

static void s_userdata_cleanup(void *userdata) {
    struct s_duplicate_test_ud *ud = userdata;
    aws_string_destroy(ud->string);
    ud->cleaned = true;
}

AWS_TEST_CASE(mqtt_topic_tree_duplicate_transactions, s_mqtt_topic_tree_duplicate_transactions_fn)
static int s_mqtt_topic_tree_duplicate_transactions_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt_topic_tree tree;
    ASSERT_SUCCESS(aws_mqtt_topic_tree_init(&tree, allocator));

    struct aws_string *topic_a_a = aws_string_new_from_array(allocator, s_topic_a_a.ptr, s_topic_a_a.len);
    struct aws_string *topic_a_a_a = aws_string_new_from_array(allocator, s_topic_a_a_a.ptr, s_topic_a_a_a.len);
    struct aws_string *topic_a_a_a_copy = aws_string_new_from_array(allocator, s_topic_a_a_a.ptr, s_topic_a_a_a.len);
    struct aws_string *topic_a_a_b = aws_string_new_from_array(allocator, s_topic_a_a_b.ptr, s_topic_a_a_b.len);

    size_t number_topics = 4;

    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, transaction_buf, aws_mqtt_topic_tree_action_size * number_topics);
    struct aws_array_list transaction;
    aws_array_list_init_static(&transaction, transaction_buf, number_topics, aws_mqtt_topic_tree_action_size);

    /* Ensure that the intermediate 'a' node was removed as well. */
    ASSERT_UINT_EQUALS(0, aws_hash_table_get_entry_count(&tree.root->subtopics));

    struct s_duplicate_test_ud ud_a_a = {.string = topic_a_a, .cleaned = false};
    struct s_duplicate_test_ud ud_a_a_a = {.string = topic_a_a_a, .cleaned = false};
    struct s_duplicate_test_ud ud_a_a_a_copy = {.string = topic_a_a_a_copy, .cleaned = false};
    struct s_duplicate_test_ud ud_a_a_b = {.string = topic_a_a_b, .cleaned = false};

    /* insert duplicate strings, and the old userdata will be cleaned up */
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree, &transaction, topic_a_a_a, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_userdata_cleanup, &ud_a_a_a));
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree,
        &transaction,
        topic_a_a_a_copy,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        &on_publish,
        s_userdata_cleanup,
        &ud_a_a_a_copy));
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree, &transaction, topic_a_a, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_userdata_cleanup, &ud_a_a));
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree, &transaction, topic_a_a_b, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_userdata_cleanup, &ud_a_a_b));
    aws_mqtt_topic_tree_transaction_commit(&tree, &transaction);

    /* The copy replaced the original node, and the old string has been cleaned up, but the new string will live with
     * the topic tree. */
    ASSERT_TRUE(ud_a_a_a.cleaned);
    ASSERT_FALSE(ud_a_a_a_copy.cleaned);
    ASSERT_FALSE(ud_a_a.cleaned);
    ASSERT_FALSE(ud_a_a_b.cleaned);

    /* the result will be the same as we just intert three nodes */
    ASSERT_UINT_EQUALS(1, aws_hash_table_get_entry_count(&tree.root->subtopics));

    struct aws_mqtt_packet_publish publish;
    aws_mqtt_packet_publish_init(&publish, false, AWS_MQTT_QOS_AT_MOST_ONCE, false, s_topic_a_a, 1, s_topic_a_a);

    times_called = 0;
    aws_mqtt_topic_tree_publish(&tree, &publish);
    ASSERT_INT_EQUALS(times_called, 1);

    aws_mqtt_topic_tree_clean_up(&tree);
    ASSERT_TRUE(ud_a_a_a_copy.cleaned);
    ASSERT_TRUE(ud_a_a.cleaned);
    ASSERT_TRUE(ud_a_a_b.cleaned);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_topic_tree_transactions, s_mqtt_topic_tree_transactions_fn)
static int s_mqtt_topic_tree_transactions_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt_topic_tree tree;
    ASSERT_SUCCESS(aws_mqtt_topic_tree_init(&tree, allocator));

    struct aws_string *topic_a_a = aws_string_new_from_array(allocator, s_topic_a_a.ptr, s_topic_a_a.len);

    AWS_VARIABLE_LENGTH_ARRAY(uint8_t, transaction_buf, aws_mqtt_topic_tree_action_size * 3);
    struct aws_array_list transaction;
    aws_array_list_init_static(&transaction, transaction_buf, 3, aws_mqtt_topic_tree_action_size);

    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_insert(
        &tree, &transaction, topic_a_a, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_string_clean_up, topic_a_a));
    /* The userdata will not be cleaned up by roll back, since the transaction has not been commit yet. */
    aws_mqtt_topic_tree_transaction_roll_back(&tree, &transaction);

    /* Ensure that the intermediate 'a' node was removed as well. */
    ASSERT_UINT_EQUALS(0, aws_hash_table_get_entry_count(&tree.root->subtopics));
    /* Insert(commit), remove, roll back the removal */
    ASSERT_SUCCESS(aws_mqtt_topic_tree_insert(
        &tree, topic_a_a, AWS_MQTT_QOS_AT_MOST_ONCE, &on_publish, s_string_clean_up, topic_a_a));
    ASSERT_SUCCESS(aws_mqtt_topic_tree_transaction_remove(&tree, &transaction, &s_topic_a_a, NULL));
    aws_mqtt_topic_tree_transaction_roll_back(&tree, &transaction);

    struct aws_mqtt_packet_publish publish;
    aws_mqtt_packet_publish_init(&publish, false, AWS_MQTT_QOS_AT_MOST_ONCE, false, s_topic_a_a, 1, s_topic_a_a);

    times_called = 0;
    aws_mqtt_topic_tree_publish(&tree, &publish);
    ASSERT_INT_EQUALS(times_called, 1);

    aws_mqtt_topic_tree_clean_up(&tree);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_topic_validation, s_mqtt_topic_validation_fn)
static int s_mqtt_topic_validation_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;

#define ASSERT_TOPIC_VALIDITY(expected, topic)                                                                         \
    do {                                                                                                               \
        struct aws_byte_cursor topic_cursor;                                                                           \
        topic_cursor.ptr = (uint8_t *)(topic);                                                                         \
        topic_cursor.len = strlen(topic);                                                                              \
        ASSERT_##expected(aws_mqtt_is_valid_topic(&topic_cursor));                                                     \
    } while (false)

    ASSERT_TOPIC_VALIDITY(TRUE, "/");
    ASSERT_TOPIC_VALIDITY(TRUE, "a/");
    ASSERT_TOPIC_VALIDITY(TRUE, "/b");
    ASSERT_TOPIC_VALIDITY(TRUE, "a/b/c");

    ASSERT_TOPIC_VALIDITY(FALSE, "#");
    ASSERT_TOPIC_VALIDITY(FALSE, "sport/tennis/#");
    ASSERT_TOPIC_VALIDITY(FALSE, "sport/tennis#");
    ASSERT_TOPIC_VALIDITY(FALSE, "sport/tennis/#/ranking");
    ASSERT_TOPIC_VALIDITY(FALSE, "");
    ASSERT_TOPIC_VALIDITY(FALSE, "+");
    ASSERT_TOPIC_VALIDITY(FALSE, "+/tennis/#");
    ASSERT_TOPIC_VALIDITY(FALSE, "sport/+/player1");
    ASSERT_TOPIC_VALIDITY(FALSE, "sport+");

    ASSERT_TOPIC_VALIDITY(FALSE, "\x41/\xED\xBF\xBF/\x41");

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_topic_filter_validation, s_mqtt_topic_filter_validation_fn)
static int s_mqtt_topic_filter_validation_fn(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;

#define ASSERT_TOPIC_FILTER_VALIDITY(expected, topic_filter)                                                           \
    do {                                                                                                               \
        struct aws_byte_cursor topic_filter_cursor;                                                                    \
        topic_filter_cursor.ptr = (uint8_t *)(topic_filter);                                                           \
        topic_filter_cursor.len = strlen(topic_filter);                                                                \
        ASSERT_##expected(aws_mqtt_is_valid_topic_filter(&topic_filter_cursor));                                       \
    } while (false)

    ASSERT_TOPIC_FILTER_VALIDITY(TRUE, "#");
    ASSERT_TOPIC_FILTER_VALIDITY(TRUE, "sport/tennis/#");
    ASSERT_TOPIC_FILTER_VALIDITY(FALSE, "sport/tennis#");
    ASSERT_TOPIC_FILTER_VALIDITY(FALSE, "sport/tennis/#/ranking");
    ASSERT_TOPIC_FILTER_VALIDITY(FALSE, "");

    ASSERT_TOPIC_FILTER_VALIDITY(TRUE, "+/");
    ASSERT_TOPIC_FILTER_VALIDITY(TRUE, "+");
    ASSERT_TOPIC_FILTER_VALIDITY(TRUE, "+/tennis/#");
    ASSERT_TOPIC_FILTER_VALIDITY(TRUE, "sport/+/player1");
    ASSERT_TOPIC_FILTER_VALIDITY(FALSE, "sport+");

    ASSERT_TOPIC_FILTER_VALIDITY(FALSE, "\x41/\xED\xA0\x80/\x41");

    return AWS_OP_SUCCESS;
}
