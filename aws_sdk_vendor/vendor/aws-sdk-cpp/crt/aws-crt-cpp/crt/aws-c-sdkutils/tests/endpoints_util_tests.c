/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/byte_buf.h>
#include <aws/common/string.h>
#include <aws/sdkutils/private/endpoints_util.h>
#include <aws/testing/aws_test_harness.h>

AWS_TEST_CASE(endpoints_uri_normalize_path, s_test_uri_normalize_path)
static int s_test_uri_normalize_path(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_buf buf1;
    ASSERT_SUCCESS(aws_byte_buf_init_from_normalized_uri_path(allocator, aws_byte_cursor_from_c_str("/"), &buf1));
    ASSERT_TRUE(aws_byte_buf_eq_c_str(&buf1, "/"));
    aws_byte_buf_clean_up(&buf1);

    struct aws_byte_buf buf2;
    ASSERT_SUCCESS(aws_byte_buf_init_from_normalized_uri_path(allocator, aws_byte_cursor_from_c_str("aaa"), &buf2));
    ASSERT_TRUE(aws_byte_buf_eq_c_str(&buf2, "/aaa/"));
    aws_byte_buf_clean_up(&buf2);

    struct aws_byte_buf buf3;
    ASSERT_SUCCESS(aws_byte_buf_init_from_normalized_uri_path(allocator, aws_byte_cursor_from_c_str("aaa/"), &buf3));
    ASSERT_TRUE(aws_byte_buf_eq_c_str(&buf3, "/aaa/"));
    aws_byte_buf_clean_up(&buf3);

    struct aws_byte_buf buf4;
    ASSERT_SUCCESS(aws_byte_buf_init_from_normalized_uri_path(allocator, aws_byte_cursor_from_c_str("/aaa"), &buf4));
    ASSERT_TRUE(aws_byte_buf_eq_c_str(&buf4, "/aaa/"));
    aws_byte_buf_clean_up(&buf4);

    struct aws_byte_buf buf5;
    ASSERT_SUCCESS(aws_byte_buf_init_from_normalized_uri_path(allocator, aws_byte_cursor_from_c_str(""), &buf5));
    ASSERT_TRUE(aws_byte_buf_eq_c_str(&buf5, "/"));
    aws_byte_buf_clean_up(&buf5);

    return AWS_OP_SUCCESS;
}

int s_resolve_cb(struct aws_byte_cursor template, void *user_data, struct aws_owning_cursor *out_resolved) {
    (void)template;
    (void)user_data;
    *out_resolved = aws_endpoints_non_owning_cursor_create(aws_byte_cursor_from_c_str("test"));
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    endpoints_byte_buf_init_from_resolved_templated_string,
    s_test_byte_buf_init_from_resolved_templated_string)
static int s_test_byte_buf_init_from_resolved_templated_string(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_buf buf;

    ASSERT_SUCCESS(aws_byte_buf_init_from_resolved_templated_string(
        allocator, &buf, aws_byte_cursor_from_c_str("{e} a {b}{c} a {d}"), s_resolve_cb, NULL, false));
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(aws_byte_cursor_from_buf(&buf), "test a testtest a test");
    aws_byte_buf_clean_up(&buf);

    ASSERT_SUCCESS(aws_byte_buf_init_from_resolved_templated_string(
        allocator,
        &buf,
        aws_byte_cursor_from_c_str("{ \"a\": \"{b} {d} \", \"c\": \" {e} \"}"),
        s_resolve_cb,
        NULL,
        true));
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(aws_byte_cursor_from_buf(&buf), "{ \"a\": \"test test \", \"c\": \" test \"}");
    aws_byte_buf_clean_up(&buf);

    ASSERT_SUCCESS(aws_byte_buf_init_from_resolved_templated_string(
        allocator, &buf, aws_byte_cursor_from_c_str("a \" {b} \" a"), s_resolve_cb, NULL, false));
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(aws_byte_cursor_from_buf(&buf), "a \" test \" a");
    aws_byte_buf_clean_up(&buf);

    ASSERT_SUCCESS(aws_byte_buf_init_from_resolved_templated_string(
        allocator, &buf, aws_byte_cursor_from_c_str("{ \"a\": \"a \\\" {b} \\\" a\" }"), s_resolve_cb, NULL, true));
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(aws_byte_cursor_from_buf(&buf), "{ \"a\": \"a \\\" test \\\" a\" }");
    aws_byte_buf_clean_up(&buf);

    return AWS_OP_SUCCESS;
}
