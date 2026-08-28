/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/Types.h>
#include <aws/crt/io/HostResolver.h>
#include <aws/testing/aws_test_harness.h>

#include <condition_variable>
#include <mutex>

static int s_TestDefaultResolution(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);
        ASSERT_NOT_NULL(eventLoopGroup.GetUnderlyingHandle());

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 5, allocator);
        ASSERT_TRUE(defaultHostResolver);
        ASSERT_NOT_NULL(defaultHostResolver.GetUnderlyingHandle());

        std::condition_variable semaphore;
        std::mutex semaphoreLock;
        size_t addressCount = 0;
        int error = 0;

        auto onHostResolved = [&](Aws::Crt::Io::HostResolver &,
                                  const Aws::Crt::Vector<Aws::Crt::Io::HostAddress> &addresses,
                                  int errorCode)
        {
            {
                std::lock_guard<std::mutex> lock(semaphoreLock);
                addressCount = addresses.size();
                error = errorCode;
                // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
                semaphore.notify_one();
            }
        };

        ASSERT_TRUE(defaultHostResolver.ResolveHost("localhost", onHostResolved));
        {
            std::unique_lock<std::mutex> lock(semaphoreLock);
            semaphore.wait(lock, [&]() { return addressCount || error; });
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(DefaultResolution, s_TestDefaultResolution)
