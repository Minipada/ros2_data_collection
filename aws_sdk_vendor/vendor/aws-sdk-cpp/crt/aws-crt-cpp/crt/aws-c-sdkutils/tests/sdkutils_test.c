/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/sdkutils/sdkutils.h>

static int s_sdkutils_library_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_sdkutils_library_init(allocator);
    aws_sdkutils_library_clean_up();
    return 0;
}

AWS_TEST_CASE(sdkutils_library_test, s_sdkutils_library_test)
