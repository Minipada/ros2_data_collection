/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/auth/aws_imds_client.h>
#include <aws/crt/Api.h>
#include <aws/crt/ImdsClient.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/testing/aws_test_harness.h>
#include <condition_variable>
#include <mutex>

using namespace Aws::Crt;
using namespace Aws::Crt::Auth;
using namespace Aws::Crt::Imds;

static int s_TestCreatingImdsClient(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Io::EventLoopGroup eventLoopGroup(1, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        ImdsClientConfig config;
        config.Bootstrap = &clientBootstrap;
        ImdsClient client(config);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(TestCreatingImdsClient, s_TestCreatingImdsClient)

static int s_TestImdsClientGetInstanceInfo(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Io::EventLoopGroup eventLoopGroup(1, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        ImdsClientConfig config;
        config.Bootstrap = &clientBootstrap;
        ImdsClient client(config);

        std::condition_variable signal;
        std::mutex lock;
        int error = 0;
        InstanceInfo info;

        auto callback = [&](const InstanceInfo &instanceInfo, int errorCode, void *)
        {
            std::unique_lock<std::mutex> ulock(lock);
            info = instanceInfo;
            error = errorCode;
            signal.notify_one();
        };

        client.GetInstanceInfo(callback, nullptr);

        {
            std::unique_lock<std::mutex> ulock(lock);
            signal.wait(ulock);
        }

        if (error == 0)
        {
            ASSERT_FALSE(0 == info.instanceId.size());
            ASSERT_NOT_NULL(info.instanceId.data());
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(TestImdsClientGetInstanceInfo, s_TestImdsClientGetInstanceInfo)

static int s_TestImdsClientGetCredentials(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Io::EventLoopGroup eventLoopGroup(1, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        ImdsClientConfig config;
        config.Bootstrap = &clientBootstrap;
        ImdsClient client(config);

        std::condition_variable signal;
        std::mutex lock;
        int error = 0;
        InstanceInfo info;

        std::string role;
        auto roleCallback = [&](const StringView &resource, int errorCode, void *)
        {
            std::unique_lock<std::mutex> ulock(lock);
            role = std::string(resource.data(), resource.size());
            error = errorCode;
            signal.notify_one();
        };

        client.GetAttachedIamRole(roleCallback, nullptr);

        {
            std::unique_lock<std::mutex> ulock(lock);
            signal.wait(ulock);
        }

        std::shared_ptr<Auth::Credentials> creds(nullptr);
        auto callback = [&](const Auth::Credentials &credentials, int errorCode, void *)
        {
            std::unique_lock<std::mutex> ulock(lock);
            creds = Aws::Crt::MakeShared<Credentials>(allocator, credentials.GetUnderlyingHandle());
            error = errorCode;
            signal.notify_one();
        };

        client.GetCredentials(StringView(role.data(), role.size()), callback, nullptr);

        {
            std::unique_lock<std::mutex> ulock(lock);
            signal.wait(ulock);
        }

        if (error == 0 && creds)
        {
            ASSERT_FALSE(0 == creds->GetAccessKeyId().len);
            ASSERT_NOT_NULL(creds->GetAccessKeyId().ptr);

            ASSERT_FALSE(0 == creds->GetSecretAccessKey().len);
            ASSERT_NOT_NULL(creds->GetSecretAccessKey().ptr);

            ASSERT_FALSE(0 == creds->GetSessionToken().len);
            ASSERT_NOT_NULL(creds->GetSessionToken().ptr);
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(TestImdsClientGetCredentials, s_TestImdsClientGetCredentials)
