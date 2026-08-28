/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/compression/compression.h>
#include <aws/testing/aws_test_harness.h>

AWS_TEST_CASE(library_init, s_test_library_init)
static int s_test_library_init(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_compression_library_init(allocator);

    /* Ensure that errors were registered */
    const char *err_name = aws_error_name(AWS_ERROR_COMPRESSION_UNKNOWN_SYMBOL);
    const char *expected = "AWS_ERROR_COMPRESSION_UNKNOWN_SYMBOL";
    ASSERT_BIN_ARRAYS_EQUALS(expected, strlen(expected), err_name, strlen(err_name));

    aws_compression_library_clean_up();
    return AWS_OP_SUCCESS;
}
