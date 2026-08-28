
/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/UUID.h>

#include <aws/testing/aws_test_harness.h>

#include <iostream>
#include <utility>

static int s_UUIDToString(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::UUID Uuid;
        Aws::Crt::String uuidStr = Uuid.ToString();
        ASSERT_TRUE(uuidStr.length() != 0);
        ASSERT_TRUE(Uuid == Aws::Crt::UUID(uuidStr));
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(UUIDToString, s_UUIDToString)
