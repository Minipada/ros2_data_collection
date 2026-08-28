/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>

#include <aws/testing/aws_test_harness.h>
#include <utility>
#if !BYO_CRYPTO
static int s_TestTLSContextResourceSafety(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
        tlsCtxOptions.SetTlsCipherPreference(AWS_IO_TLS_CIPHER_PREF_SYSTEM_DEFAULT);

        Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
        ASSERT_TRUE(tlsContext);

        auto tlsContextPostMove = std::move(tlsContext);
        ASSERT_TRUE(tlsContextPostMove);

        // NOLINTNEXTLINE
        ASSERT_FALSE(tlsContext);

        auto tlsConnectionOptions = tlsContextPostMove.NewConnectionOptions();
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(TLSContextResourceSafety, s_TestTLSContextResourceSafety)

static int s_TestTLSContextUninitializedNewConnectionOptions(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        // Intentionally create an uninitialized TlsContext
        Aws::Crt::Io::TlsContext tlsContext;

        Aws::Crt::Io::TlsConnectionOptions options = tlsContext.NewConnectionOptions();

        // Options should be uninitialized, but creating them should not result in a crash.
        ASSERT_TRUE(!options);
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(TLSContextUninitializedNewConnectionOptions, s_TestTLSContextUninitializedNewConnectionOptions)
#endif // !BYO_CRYPTO
