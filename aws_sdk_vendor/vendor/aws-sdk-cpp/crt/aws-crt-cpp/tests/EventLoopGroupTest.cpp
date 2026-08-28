/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/testing/aws_test_harness.h>

#include <utility>

static int s_TestEventLoopResourceSafety(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;

    {
        Aws::Crt::ApiHandle handle;

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);
        ASSERT_NOT_NULL(eventLoopGroup.GetUnderlyingHandle());

        Aws::Crt::Io::EventLoopGroup eventLoopGroupPostMove(std::move(eventLoopGroup));
        ASSERT_TRUE(eventLoopGroupPostMove);
        ASSERT_NOT_NULL(eventLoopGroupPostMove.GetUnderlyingHandle());

        // NOLINTNEXTLINE
        ASSERT_FALSE(eventLoopGroup);
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(EventLoopResourceSafety, s_TestEventLoopResourceSafety)
