/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/encoding.h>
#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/v5/mqtt5_utils.h>

#include <aws/testing/aws_test_harness.h>

static int s_mqtt5_topic_skip_rules_prefix_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_byte_cursor skip_cursor;
    struct aws_byte_cursor expected_cursor;

    /* nothing should be skipped */
    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("dont/skip/anything"));
    expected_cursor = aws_byte_cursor_from_c_str("dont/skip/anything");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str(""));
    expected_cursor = aws_byte_cursor_from_c_str("");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("/"));
    expected_cursor = aws_byte_cursor_from_c_str("/");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$aws"));
    expected_cursor = aws_byte_cursor_from_c_str("$aws");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$aws/rules"));
    expected_cursor = aws_byte_cursor_from_c_str("$aws/rules");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$aws/rules/"));
    expected_cursor = aws_byte_cursor_from_c_str("$aws/rules/");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$aws/rules/rulename"));
    expected_cursor = aws_byte_cursor_from_c_str("$aws/rules/rulename");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$share"));
    expected_cursor = aws_byte_cursor_from_c_str("$share");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$share/"));
    expected_cursor = aws_byte_cursor_from_c_str("$share/");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$share/share-name"));
    expected_cursor = aws_byte_cursor_from_c_str("$share/share-name");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    /* prefix should be skipped */
    skip_cursor =
        aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(aws_byte_cursor_from_c_str("$aws/rules/rulename/"));
    expected_cursor = aws_byte_cursor_from_c_str("");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(
        aws_byte_cursor_from_c_str("$aws/rules/some-rule/segment1/segment2"));
    expected_cursor = aws_byte_cursor_from_c_str("segment1/segment2");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(
        aws_byte_cursor_from_c_str("$share/share-name/segment1/segment2"));
    expected_cursor = aws_byte_cursor_from_c_str("segment1/segment2");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    skip_cursor = aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(
        aws_byte_cursor_from_c_str("$share/share-name/$aws/rules/some-rule/segment1/segment2"));
    expected_cursor = aws_byte_cursor_from_c_str("segment1/segment2");
    ASSERT_TRUE(aws_byte_cursor_eq(&skip_cursor, &expected_cursor));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_topic_skip_rules_prefix, s_mqtt5_topic_skip_rules_prefix_fn)

static int s_mqtt5_topic_get_segment_count_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    ASSERT_INT_EQUALS(1, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("")));
    ASSERT_INT_EQUALS(1, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("hello")));
    ASSERT_INT_EQUALS(2, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("hello/")));
    ASSERT_INT_EQUALS(2, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("hello/world")));
    ASSERT_INT_EQUALS(3, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("a/b/c")));
    ASSERT_INT_EQUALS(3, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("//")));
    ASSERT_INT_EQUALS(4, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("$SYS/bad/no/")));
    ASSERT_INT_EQUALS(1, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("$aws")));
    ASSERT_INT_EQUALS(8, aws_mqtt5_topic_get_segment_count(aws_byte_cursor_from_c_str("//a//b/c//")));

    ASSERT_INT_EQUALS(
        2,
        aws_mqtt5_topic_get_segment_count(aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(
            aws_byte_cursor_from_c_str("$aws/rules/some-rule/segment1/segment2"))));
    ASSERT_INT_EQUALS(
        1,
        aws_mqtt5_topic_get_segment_count(aws_mqtt5_topic_skip_aws_iot_core_uncounted_prefix(
            aws_byte_cursor_from_c_str("$aws/rules/some-rule/segment1"))));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_topic_get_segment_count, s_mqtt5_topic_get_segment_count_fn)

static int s_mqtt5_shared_subscription_validation_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("oof")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$sha")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share//")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share//test")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m+/")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m#/")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m")));
    ASSERT_FALSE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m/")));

    ASSERT_TRUE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m/#")));
    ASSERT_TRUE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m/great")));
    ASSERT_TRUE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m/test/+")));
    ASSERT_TRUE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m/+/test")));
    ASSERT_TRUE(aws_mqtt_is_topic_filter_shared_subscription(aws_byte_cursor_from_c_str("$share/m/test/#")));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_shared_subscription_validation, s_mqtt5_shared_subscription_validation_fn)
