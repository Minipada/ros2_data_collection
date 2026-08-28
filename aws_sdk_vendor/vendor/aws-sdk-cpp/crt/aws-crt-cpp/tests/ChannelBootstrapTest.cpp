/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/testing/aws_test_harness.h>

#include <future>
#include <utility>

static int s_TestClientBootstrapResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);
        ASSERT_NOT_NULL(eventLoopGroup.GetUnderlyingHandle());

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);
        ASSERT_NOT_NULL(defaultHostResolver.GetUnderlyingHandle());

        std::promise<void> bootstrapShutdownPromise;
        std::future<void> bootstrapShutdownFuture = bootstrapShutdownPromise.get_future();
        {
            Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
            ASSERT_TRUE(clientBootstrap);
            ASSERT_NOT_NULL(clientBootstrap.GetUnderlyingHandle());
            clientBootstrap.EnableBlockingShutdown();
            clientBootstrap.SetShutdownCompleteCallback([&]() { bootstrapShutdownPromise.set_value(); });
        }

        ASSERT_TRUE(std::future_status::ready == bootstrapShutdownFuture.wait_for(std::chrono::seconds(10)));
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(ClientBootstrapResourceSafety, s_TestClientBootstrapResourceSafety)
