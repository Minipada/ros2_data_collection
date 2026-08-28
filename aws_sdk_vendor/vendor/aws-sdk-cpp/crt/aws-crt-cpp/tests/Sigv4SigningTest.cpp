/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>

#include <aws/auth/signable.h>
#include <aws/auth/signing.h>
#include <aws/auth/signing_result.h>
#include <aws/common/date_time.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/crt/auth/Sigv4Signing.h>
#include <aws/crt/http/HttpRequestResponse.h>

#include <aws/auth/credentials.h>
#include <aws/http/request_response.h>

#include <aws/testing/aws_test_harness.h>

#include <condition_variable>
#include <mutex>

using namespace Aws::Crt;
using namespace Aws::Crt::Auth;
using namespace Aws::Crt::Http;

class SignWaiter
{
  public:
    SignWaiter() : m_lock(), m_signal(), m_done(false) {}

    void OnSigningComplete(const std::shared_ptr<Aws::Crt::Http::HttpRequest> &, int)
    {
        std::unique_lock<std::mutex> lock(m_lock);
        m_done = true;
        m_signal.notify_one();
    }

    void Wait()
    {
        {
            std::unique_lock<std::mutex> lock(m_lock);
            m_signal.wait(lock, [this]() { return m_done == true; });
        }
    }

  private:
    std::mutex m_lock;
    std::condition_variable m_signal;
    bool m_done;
};

static std::shared_ptr<HttpRequest> s_MakeDummyRequest(Allocator *allocator)
{
    auto request = MakeShared<HttpRequest>(allocator);

    request->SetMethod(aws_byte_cursor_from_c_str("GET"));
    request->SetPath(aws_byte_cursor_from_c_str("http://www.test.com/mctest"));

    HttpHeader header;
    header.name = aws_byte_cursor_from_c_str("Host");
    header.value = aws_byte_cursor_from_c_str("www.test.com");

    request->AddHeader(header);

    auto bodyStream = Aws::Crt::MakeShared<std::stringstream>(allocator, "Something");
    request->SetBody(bodyStream);

    return request;
}

static std::shared_ptr<Credentials> s_MakeDummyCredentials(Allocator *allocator)
{
    return Aws::Crt::MakeShared<Credentials>(
        allocator,
        aws_byte_cursor_from_c_str("access"),
        aws_byte_cursor_from_c_str("secret"),
        aws_byte_cursor_from_c_str("token"),
        UINT64_MAX);
}

static std::shared_ptr<CredentialsProvider> s_MakeAsyncStaticProvider(
    Allocator *allocator,
    const Aws::Crt::Io::ClientBootstrap &bootstrap)
{
    struct aws_credentials_provider_imds_options imds_options;
    AWS_ZERO_STRUCT(imds_options);
    imds_options.bootstrap = bootstrap.GetUnderlyingHandle();

    struct aws_credentials_provider *provider1 = aws_credentials_provider_new_imds(allocator, &imds_options);

    aws_credentials_provider_static_options static_options;
    AWS_ZERO_STRUCT(static_options);
    static_options.access_key_id = aws_byte_cursor_from_c_str("access");
    static_options.secret_access_key = aws_byte_cursor_from_c_str("secret");
    static_options.session_token = aws_byte_cursor_from_c_str("token");

    struct aws_credentials_provider *provider2 = aws_credentials_provider_new_static(allocator, &static_options);

    struct aws_credentials_provider *providers[2] = {provider1, provider2};

    aws_credentials_provider_chain_options options;
    AWS_ZERO_STRUCT(options);
    options.providers = providers;
    options.provider_count = 2;

    struct aws_credentials_provider *provider_chain = aws_credentials_provider_new_chain(allocator, &options);
    aws_credentials_provider_release(provider1);
    aws_credentials_provider_release(provider2);
    if (provider_chain == NULL)
    {
        return nullptr;
    }

    return Aws::Crt::MakeShared<CredentialsProvider>(allocator, provider_chain, allocator);
}

static int s_Sigv4SigningTestCreateDestroy(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;

    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Io::TlsContextOptions tlsOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient(allocator);
        Aws::Crt::Io::TlsContext tlsContext(tlsOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        CredentialsProviderChainDefaultConfig config;
        config.Bootstrap = &clientBootstrap;
        config.TlsContext = &tlsContext;

        auto provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(config);

        auto signer = Aws::Crt::MakeShared<Sigv4HttpRequestSigner>(allocator, allocator);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(Sigv4SigningTestCreateDestroy, s_Sigv4SigningTestCreateDestroy)

static int s_Sigv4SigningTestSimple(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Io::TlsContextOptions tlsOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient(allocator);
        Aws::Crt::Io::TlsContext tlsContext(tlsOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        CredentialsProviderChainDefaultConfig config;
        config.Bootstrap = &clientBootstrap;
        config.TlsContext = &tlsContext;

        auto provider = s_MakeAsyncStaticProvider(allocator, clientBootstrap);

        auto signer = Aws::Crt::MakeShared<Sigv4HttpRequestSigner>(allocator, allocator);

        auto request = s_MakeDummyRequest(allocator);

        AwsSigningConfig signingConfig(allocator);
        signingConfig.SetSigningTimepoint(Aws::Crt::DateTime());
        signingConfig.SetRegion("test");
        signingConfig.SetService("service");
        signingConfig.SetCredentialsProvider(provider);

        SignWaiter waiter;

        ASSERT_TRUE(signer->SignRequest(
            request,
            signingConfig,
            [&](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &request, int errorCode)
            { waiter.OnSigningComplete(request, errorCode); }));
        waiter.Wait();
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(Sigv4SigningTestSimple, s_Sigv4SigningTestSimple)

static int s_Sigv4SigningTestCredentials(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Io::TlsContextOptions tlsOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient(allocator);
        Aws::Crt::Io::TlsContext tlsContext(tlsOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        CredentialsProviderChainDefaultConfig config;
        config.Bootstrap = &clientBootstrap;
        config.TlsContext = &tlsContext;

        auto signer = Aws::Crt::MakeShared<Sigv4HttpRequestSigner>(allocator, allocator);

        auto request = s_MakeDummyRequest(allocator);

        AwsSigningConfig signingConfig(allocator);
        signingConfig.SetSigningTimepoint(Aws::Crt::DateTime());
        signingConfig.SetRegion("test");
        signingConfig.SetService("service");
        signingConfig.SetCredentials(s_MakeDummyCredentials(allocator));

        SignWaiter waiter;

        ASSERT_TRUE(signer->SignRequest(
            request,
            signingConfig,
            [&](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &request, int errorCode)
            { waiter.OnSigningComplete(request, errorCode); }));
        waiter.Wait();
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(Sigv4SigningTestCredentials, s_Sigv4SigningTestCredentials)

static int s_Sigv4SigningTestUnsignedPayload(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Io::TlsContextOptions tlsOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient(allocator);
        Aws::Crt::Io::TlsContext tlsContext(tlsOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        CredentialsProviderChainDefaultConfig config;
        config.Bootstrap = &clientBootstrap;
        config.TlsContext = &tlsContext;

        auto signer = Aws::Crt::MakeShared<Sigv4HttpRequestSigner>(allocator, allocator);

        auto request = s_MakeDummyRequest(allocator);

        AwsSigningConfig signingConfig(allocator);
        signingConfig.SetSigningTimepoint(Aws::Crt::DateTime());
        signingConfig.SetRegion("test");
        signingConfig.SetService("service");
        signingConfig.SetCredentials(s_MakeDummyCredentials(allocator));
        signingConfig.SetSignedBodyValue(Aws::Crt::Auth::SignedBodyValue::UnsignedPayloadStr());
        signingConfig.SetSignedBodyHeader(Aws::Crt::Auth::SignedBodyHeaderType::XAmzContentSha256);

        SignWaiter waiter;

        ASSERT_TRUE(signer->SignRequest(
            request,
            signingConfig,
            [&](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &request, int errorCode)
            { waiter.OnSigningComplete(request, errorCode); }));
        waiter.Wait();
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(Sigv4SigningTestUnsignedPayload, s_Sigv4SigningTestUnsignedPayload)

/* Sigv4a Test */
AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id, "AKIAIOSFODNN7EXAMPLE");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key, "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY");
AWS_STATIC_STRING_FROM_LITERAL(s_test_ecc_pub_x, "18b7d04643359f6ec270dcbab8dce6d169d66ddc9778c75cfb08dfdb701637ab");
AWS_STATIC_STRING_FROM_LITERAL(s_test_ecc_pub_y, "fa36b35e4fe67e3112261d2e17a956ef85b06e44712d2850bcd3c2161e9993f2");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_canonical_request,
    "PUT\n"
    "/examplebucket/chunkObject.txt\n"
    "\n"
    "content-encoding:aws-chunked\n"
    "content-length:66824\n"
    "host:s3.amazonaws.com\n"
    "x-amz-content-sha256:STREAMING-AWS4-ECDSA-P256-SHA256-PAYLOAD\n"
    "x-amz-date:20130524T000000Z\n"
    "x-amz-decoded-content-length:66560\n"
    "x-amz-region-set:us-east-1\n"
    "x-amz-storage-class:REDUCED_REDUNDANCY\n"
    "\n"
    "content-encoding;content-length;host;x-amz-content-sha256;x-amz-date;x-amz-decoded-content-length;x-amz-region-"
    "set;x-amz-storage-class\n"
    "STREAMING-AWS4-ECDSA-P256-SHA256-PAYLOAD");

static std::shared_ptr<HttpRequest> s_MakeDummyRequestSigv4a(Allocator *allocator)
{
    auto request = MakeShared<HttpRequest>(allocator);

    HttpHeader headerHost;
    headerHost.name = aws_byte_cursor_from_c_str("host");
    headerHost.value = aws_byte_cursor_from_c_str("s3.amazonaws.com");
    request->AddHeader(headerHost);

    HttpHeader storageClass;
    storageClass.name = aws_byte_cursor_from_c_str("x-amz-storage-class");
    storageClass.value = aws_byte_cursor_from_c_str("REDUCED_REDUNDANCY");
    request->AddHeader(storageClass);

    HttpHeader headerContentEncoding;
    headerContentEncoding.name = aws_byte_cursor_from_c_str("Content-Encoding");
    headerContentEncoding.value = aws_byte_cursor_from_c_str("aws-chunked");
    request->AddHeader(headerContentEncoding);

    HttpHeader headerDecodedLength;
    headerDecodedLength.name = aws_byte_cursor_from_c_str("x-amz-decoded-content-length");
    headerDecodedLength.value = aws_byte_cursor_from_c_str("66560");
    request->AddHeader(headerDecodedLength);

    HttpHeader headerContentLength;
    headerContentLength.name = aws_byte_cursor_from_c_str("Content-Length");
    headerContentLength.value = aws_byte_cursor_from_c_str("66824");
    request->AddHeader(headerContentLength);

    request->SetMethod(aws_byte_cursor_from_c_str("PUT"));
    request->SetPath(aws_byte_cursor_from_c_str("/examplebucket/chunkObject.txt"));

    return request;
}

static std::shared_ptr<Credentials> s_MakeDummyCredentialsSigv4a(Allocator *allocator)
{
    struct aws_byte_cursor empty;
    AWS_ZERO_STRUCT(empty);

    return Aws::Crt::MakeShared<Credentials>(
        allocator,
        aws_byte_cursor_from_string(s_access_key_id),
        aws_byte_cursor_from_string(s_secret_access_key),
        empty,
        UINT64_MAX);
}

static int s_Sigv4aSigningTestCredentials(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Io::TlsContextOptions tlsOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient(allocator);
        Aws::Crt::Io::TlsContext tlsContext(tlsOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        CredentialsProviderChainDefaultConfig config;
        config.Bootstrap = &clientBootstrap;
        config.TlsContext = &tlsContext;

        auto signer = Aws::Crt::MakeShared<Sigv4HttpRequestSigner>(allocator, allocator);

        auto request = s_MakeDummyRequestSigv4a(allocator);
        AwsSigningConfig signingConfig(allocator);
        signingConfig.SetSigningAlgorithm(SigningAlgorithm::SigV4A);
        signingConfig.SetSignatureType(SignatureType::HttpRequestViaHeaders);
        signingConfig.SetRegion("us-east-1");
        signingConfig.SetService("s3");
        signingConfig.SetSigningTimepoint(Aws::Crt::DateTime("Fri, 24 May 2013 00:00:00 GMT", DateFormat::RFC822));
        signingConfig.SetUseDoubleUriEncode(false);
        signingConfig.SetShouldNormalizeUriPath(true);
        signingConfig.SetSignedBodyValue("STREAMING-AWS4-ECDSA-P256-SHA256-PAYLOAD");
        signingConfig.SetSignedBodyHeader(SignedBodyHeaderType::XAmzContentSha256);
        signingConfig.SetCredentials(s_MakeDummyCredentialsSigv4a(allocator));

        SignWaiter waiter;

        ASSERT_TRUE(signer->SignRequest(
            request,
            signingConfig,
            [&](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &request, int errorCode)
            { waiter.OnSigningComplete(request, errorCode); }));

        waiter.Wait();

        /* get signature from signed request */
        struct aws_byte_cursor signatureCursor;
        AWS_ZERO_STRUCT(signatureCursor);
        size_t headerCount = request->GetHeaderCount();
        for (size_t i = 0; i < headerCount; ++i)
        {
            Optional<HttpHeader> optionalHeader = request->GetHeader(i);
            if (optionalHeader.has_value())
            {
                HttpHeader header = optionalHeader.value();
                if (aws_byte_cursor_eq_c_str_ignore_case(&header.name, "Authorization"))
                {
                    struct aws_byte_cursor headerValueCursor = header.value;
                    struct aws_byte_cursor searchValue = aws_byte_cursor_from_c_str("Signature=");
                    aws_byte_cursor_find_exact(&headerValueCursor, &searchValue, &signatureCursor);
                    aws_byte_cursor_advance(&signatureCursor, searchValue.len);
                    break;
                }
            }
        }

        auto requestClean = s_MakeDummyRequestSigv4a(allocator);
        ScopedResource<struct aws_signable> signable = ScopedResource<struct aws_signable>(
            aws_signable_new_http_request(allocator, requestClean->GetUnderlyingMessage()), aws_signable_destroy);
        auto awsSigningConfig = static_cast<const AwsSigningConfig *>(&signingConfig);

        ASSERT_SUCCESS(aws_verify_sigv4a_signing(
            allocator,
            signable.get(),
            (aws_signing_config_base *)awsSigningConfig->GetUnderlyingHandle(),
            aws_byte_cursor_from_string(s_expected_canonical_request),
            signatureCursor,
            aws_byte_cursor_from_string(s_test_ecc_pub_x),
            aws_byte_cursor_from_string(s_test_ecc_pub_y)));
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(Sigv4aSigningTestCredentials, s_Sigv4aSigningTestCredentials)
