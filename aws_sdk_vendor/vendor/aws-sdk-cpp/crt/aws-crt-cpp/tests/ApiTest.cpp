/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/Config.h>
#include <aws/crt/Types.h>
#include <aws/testing/aws_test_harness.h>

static int s_TestApiMultiCreateDestroy(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
    }

    {
        Aws::Crt::ApiHandle apiHandle(allocator);
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(ApiMultiCreateDestroy, s_TestApiMultiCreateDestroy)

static int s_TestApiMultiDefaultCreateDestroy(struct aws_allocator *allocator, void *)
{
    (void)allocator;

    {
        Aws::Crt::ApiHandle apiHandle;
    }

    {
        Aws::Crt::ApiHandle apiHandle;
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(ApiMultiDefaultCreateDestroy, s_TestApiMultiDefaultCreateDestroy)

static int s_TestApiStaticDefaultCreateDestroy(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::ApiHandle::GetOrCreateStaticDefaultClientBootstrap();
    }

    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::ApiHandle::GetOrCreateStaticDefaultClientBootstrap();
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(ApiStaticDefaultCreateDestroy, s_TestApiStaticDefaultCreateDestroy)

static int s_TestApiVersionReporting(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::ApiHandle::Version version = apiHandle.GetCrtVersion();
        ASSERT_UINT_EQUALS(version.major, AWS_CRT_CPP_VERSION_MAJOR);
        ASSERT_UINT_EQUALS(version.minor, AWS_CRT_CPP_VERSION_MINOR);
        ASSERT_UINT_EQUALS(version.patch, AWS_CRT_CPP_VERSION_PATCH);
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(ApiStaticVersionReporting, s_TestApiVersionReporting)
