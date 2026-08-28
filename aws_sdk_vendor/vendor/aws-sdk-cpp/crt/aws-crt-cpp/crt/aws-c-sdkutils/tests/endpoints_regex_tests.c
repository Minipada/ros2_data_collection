/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/sdkutils/private/endpoints_regex.h>
#include <aws/testing/aws_test_harness.h>

AWS_TEST_CASE(endpoints_regex_aws_region_matches, s_test_aws_region_matches)
static int s_test_aws_region_matches(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_endpoints_regex *regex =
        aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^(us|eu|ap|sa|ca|me|af|il)\\-\\w+\\-\\d+$"));

    ASSERT_NOT_NULL(regex);

    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("us-west-2")));
    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("eu-west-3")));
    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("ap-east-1")));
    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("sa-east-1")));
    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("ca-central-1")));
    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("me-central-1")));
    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("af-south-1")));
    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("il-central-1")));

    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_REGEX_NO_MATCH,
        aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("us-west")));
    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_REGEX_NO_MATCH,
        aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("uk-west-2")));
    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_REGEX_NO_MATCH,
        aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("us-w1st-2")));

    aws_endpoints_regex_destroy(regex);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(endpoints_regex_iso_region_matches, s_test_iso_region_matches)
static int s_test_iso_region_matches(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_endpoints_regex *regex =
        aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^eu\\-isoe\\-\\w+\\-\\d+$"));

    ASSERT_NOT_NULL(regex);

    ASSERT_SUCCESS(aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("eu-isoe-west-1")));

    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_REGEX_NO_MATCH,
        aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("us-west-2")));
    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_REGEX_NO_MATCH,
        aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("uk-isob-east-1")));
    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_REGEX_NO_MATCH,
        aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("us-i1sob-east-1")));

    ASSERT_ERROR(
        AWS_ERROR_SDKUTILS_ENDPOINTS_REGEX_NO_MATCH,
        aws_endpoints_regex_match(regex, aws_byte_cursor_from_c_str("us-isob-e1ast-1")));

    aws_endpoints_regex_destroy(regex);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(endpoints_regex_misc_validation, s_test_misc_regex_validation)
static int s_test_misc_regex_validation(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_endpoints_regex *regex =
        aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^us\\-(^ba)\\-\\w+\\-\\d+$"));

    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_SDKUTILS_ENDPOINTS_UNSUPPORTED_REGEX, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str(""));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, aws_last_error());

    regex = aws_endpoints_regex_new(
        allocator, aws_byte_cursor_from_c_str("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("aaaaa"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_SDKUTILS_ENDPOINTS_UNSUPPORTED_REGEX, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaa(aa$"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaaaa($"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaa()aa$"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaa*aa$"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_SDKUTILS_ENDPOINTS_UNSUPPORTED_REGEX, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaa+aa$"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_SDKUTILS_ENDPOINTS_UNSUPPORTED_REGEX, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaa(a|ab)aa$"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_SDKUTILS_ENDPOINTS_UNSUPPORTED_REGEX, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaa(a||b)aa$"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_INVALID_ARGUMENT, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaa*+aa$"));
    ASSERT_NULL(regex);
    ASSERT_INT_EQUALS(AWS_ERROR_SDKUTILS_ENDPOINTS_UNSUPPORTED_REGEX, aws_last_error());

    regex = aws_endpoints_regex_new(allocator, aws_byte_cursor_from_c_str("^aaaaa$"));
    ASSERT_NOT_NULL(regex);

    ASSERT_ERROR(
        AWS_ERROR_INVALID_ARGUMENT,
        aws_endpoints_regex_match(
            regex, aws_byte_cursor_from_c_str("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")));

    aws_endpoints_regex_destroy(regex);

    return AWS_OP_SUCCESS;
}
