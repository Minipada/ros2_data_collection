/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/crypto/Hash.h>
#include <aws/crt/http/HttpConnection.h>
#include <aws/crt/http/HttpRequestResponse.h>
#include <aws/crt/io/Uri.h>

#include <aws/testing/aws_test_harness.h>

#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>

using namespace Aws::Crt;

#if !BYO_CRYPTO

static int s_VerifyFilesAreTheSame(Allocator *allocator, const char *fileName1, const char *fileName2)
{
    std::ifstream file1(fileName1, std::ios_base::binary);
    std::ifstream file2(fileName2, std::ios_base::binary);

    ASSERT_TRUE(file1);
    ASSERT_TRUE(file2);

    auto file1Hash = Crypto::Hash::CreateSHA256(allocator);
    uint8_t buffer[1024];
    AWS_ZERO_ARRAY(buffer);

    while (file1.read((char *)buffer, sizeof(buffer)))
    {
        auto read = file1.gcount();
        ByteCursor toHash = ByteCursorFromArray(buffer, (size_t)read);
        ASSERT_TRUE(file1Hash.Update(toHash));
    }

    auto file2Hash = Crypto::Hash::CreateSHA256(allocator);

    while (file2.read((char *)buffer, sizeof(buffer)))
    {
        auto read = file2.gcount();
        ByteCursor toHash = ByteCursorFromArray(buffer, (size_t)read);
        ASSERT_TRUE(file2Hash.Update(toHash));
    }

    uint8_t file1Digest[Crypto::SHA256_DIGEST_SIZE];
    AWS_ZERO_ARRAY(file1Digest);
    uint8_t file2Digest[Crypto::SHA256_DIGEST_SIZE];
    AWS_ZERO_ARRAY(file2Digest);

    ByteBuf file1DigestBuf = ByteBufFromEmptyArray(file1Digest, sizeof(file1Digest));
    ByteBuf file2DigestBuf = ByteBufFromEmptyArray(file2Digest, sizeof(file2Digest));

    ASSERT_TRUE(file1Hash.Digest(file1DigestBuf));
    ASSERT_TRUE(file2Hash.Digest(file2DigestBuf));

    ASSERT_BIN_ARRAYS_EQUALS(file2DigestBuf.buffer, file2DigestBuf.len, file1DigestBuf.buffer, file1DigestBuf.len);
    return AWS_OP_SUCCESS;
}

static int s_TestHttpDownloadNoBackPressure(struct aws_allocator *allocator, ByteCursor urlCursor, bool h2Required)
{
    int result = AWS_OP_ERR;

    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
        Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
        ASSERT_TRUE(tlsContext);

        Aws::Crt::Io::TlsConnectionOptions tlsConnectionOptions = tlsContext.NewConnectionOptions();

        Io::Uri uri(urlCursor, allocator);

        auto hostName = uri.GetHostName();
        tlsConnectionOptions.SetServerName(hostName);
        if (h2Required)
        {
            tlsConnectionOptions.SetAlpnList("h2");
        }

        Aws::Crt::Io::SocketOptions socketOptions;
        socketOptions.SetConnectTimeoutMs(5000);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        std::shared_ptr<Http::HttpClientConnection> connection(nullptr);
        bool errorOccured = true;
        bool connectionShutdown = false;

        std::condition_variable semaphore;
        std::mutex semaphoreLock;

        auto onConnectionSetup = [&](const std::shared_ptr<Http::HttpClientConnection> &newConnection, int errorCode)
        {
            std::lock_guard<std::mutex> lockGuard(semaphoreLock);

            if (!errorCode)
            {
                connection = newConnection;
                errorOccured = false;
            }
            else
            {
                connectionShutdown = true;
            }

            semaphore.notify_one();
        };

        auto onConnectionShutdown = [&](Http::HttpClientConnection &, int errorCode)
        {
            std::lock_guard<std::mutex> lockGuard(semaphoreLock);

            connectionShutdown = true;
            if (errorCode)
            {
                errorOccured = true;
            }

            semaphore.notify_one();
        };

        Http::HttpClientConnectionOptions httpClientConnectionOptions;
        httpClientConnectionOptions.Bootstrap = &clientBootstrap;
        httpClientConnectionOptions.OnConnectionSetupCallback = onConnectionSetup;
        httpClientConnectionOptions.OnConnectionShutdownCallback = onConnectionShutdown;
        httpClientConnectionOptions.SocketOptions = socketOptions;
        httpClientConnectionOptions.TlsOptions = tlsConnectionOptions;
        httpClientConnectionOptions.HostName = String((const char *)hostName.ptr, hostName.len);
        httpClientConnectionOptions.Port = 443;

        std::unique_lock<std::mutex> semaphoreULock(semaphoreLock);
        ASSERT_TRUE(Http::HttpClientConnection::CreateConnection(httpClientConnectionOptions, allocator));
        semaphore.wait(semaphoreULock, [&]() { return connection || connectionShutdown; });

        ASSERT_FALSE(errorOccured);
        ASSERT_FALSE(connectionShutdown);
        ASSERT_TRUE(connection);
        String fileName = h2Required ? "http_download_test_file_h2.txt" : "http_download_test_file_h1_1.txt";
        Http::HttpVersion excepted = h2Required ? Http::HttpVersion::Http2 : Http::HttpVersion::Http1_1;
        ASSERT_TRUE(connection->GetVersion() == excepted);

        int responseCode = 0;
        std::ofstream downloadedFile(fileName.c_str(), std::ios_base::binary);
        ASSERT_TRUE(downloadedFile);

        Http::HttpRequest request;
        Http::HttpRequestOptions requestOptions;
        requestOptions.request = &request;

        bool streamCompleted = false;
        requestOptions.onStreamComplete = [&](Http::HttpStream &, int errorCode)
        {
            std::lock_guard<std::mutex> lockGuard(semaphoreLock);

            streamCompleted = true;
            if (errorCode)
            {
                errorOccured = true;
            }

            semaphore.notify_one();
        };
        requestOptions.onIncomingHeadersBlockDone = nullptr;
        requestOptions.onIncomingHeaders =
            [&](Http::HttpStream &stream, enum aws_http_header_block, const Http::HttpHeader *, std::size_t)
        { responseCode = stream.GetResponseStatusCode(); };
        requestOptions.onIncomingBody = [&](Http::HttpStream &, const ByteCursor &data)
        { downloadedFile.write((const char *)data.ptr, data.len); };

        request.SetMethod(ByteCursorFromCString("GET"));
        request.SetPath(uri.GetPathAndQuery());

        Http::HttpHeader hostHeader;
        hostHeader.name = ByteCursorFromCString("host");
        hostHeader.value = uri.GetHostName();
        request.AddHeader(hostHeader);

        auto stream = connection->NewClientStream(requestOptions);
        ASSERT_TRUE(stream->Activate());

        semaphore.wait(semaphoreULock, [&]() { return streamCompleted; });
        ASSERT_INT_EQUALS(200, responseCode);

        connection->Close();
        semaphore.wait(semaphoreULock, [&]() { return connectionShutdown; });

        downloadedFile.flush();
        downloadedFile.close();
        result = s_VerifyFilesAreTheSame(allocator, fileName.c_str(), "http_test_doc.txt");
    }

    return result;
}

static int s_TestHttpDownloadNoBackPressureHTTP1_1(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    ByteCursor cursor = ByteCursorFromCString("https://aws-crt-test-stuff.s3.amazonaws.com/http_test_doc.txt");
    return s_TestHttpDownloadNoBackPressure(allocator, cursor, false /*h2Required*/);
}

AWS_TEST_CASE(HttpDownloadNoBackPressureHTTP1_1, s_TestHttpDownloadNoBackPressureHTTP1_1)

static int s_TestHttpDownloadNoBackPressureHTTP2(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    ByteCursor cursor = ByteCursorFromCString("https://d1cz66xoahf9cl.cloudfront.net/http_test_doc.txt");
    return s_TestHttpDownloadNoBackPressure(allocator, cursor, true /*h2Required*/);
}

AWS_TEST_CASE(HttpDownloadNoBackPressureHTTP2, s_TestHttpDownloadNoBackPressureHTTP2)

static int s_TestHttpStreamUnActivated(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
        Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
        ASSERT_TRUE(tlsContext);

        Aws::Crt::Io::TlsConnectionOptions tlsConnectionOptions = tlsContext.NewConnectionOptions();

        ByteCursor cursor = ByteCursorFromCString("https://aws-crt-test-stuff.s3.amazonaws.com/http_test_doc.txt");
        Io::Uri uri(cursor, allocator);

        auto hostName = uri.GetHostName();
        tlsConnectionOptions.SetServerName(hostName);

        Aws::Crt::Io::SocketOptions socketOptions;
        socketOptions.SetConnectTimeoutMs(1000);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        std::shared_ptr<Http::HttpClientConnection> connection(nullptr);
        bool errorOccured = true;
        bool connectionShutdown = false;

        std::condition_variable semaphore;
        std::mutex semaphoreLock;

        auto onConnectionSetup = [&](const std::shared_ptr<Http::HttpClientConnection> &newConnection, int errorCode)
        {
            std::lock_guard<std::mutex> lockGuard(semaphoreLock);

            if (!errorCode)
            {
                connection = newConnection;
                errorOccured = false;
            }
            else
            {
                connectionShutdown = true;
            }

            semaphore.notify_one();
        };

        auto onConnectionShutdown = [&](Http::HttpClientConnection &, int errorCode)
        {
            std::lock_guard<std::mutex> lockGuard(semaphoreLock);

            connectionShutdown = true;
            if (errorCode)
            {
                errorOccured = true;
            }

            semaphore.notify_one();
        };

        Http::HttpClientConnectionOptions httpClientConnectionOptions;
        httpClientConnectionOptions.Bootstrap = &clientBootstrap;
        httpClientConnectionOptions.OnConnectionSetupCallback = onConnectionSetup;
        httpClientConnectionOptions.OnConnectionShutdownCallback = onConnectionShutdown;
        httpClientConnectionOptions.SocketOptions = socketOptions;
        httpClientConnectionOptions.TlsOptions = tlsConnectionOptions;
        httpClientConnectionOptions.HostName = String((const char *)hostName.ptr, hostName.len);
        httpClientConnectionOptions.Port = 443;

        std::unique_lock<std::mutex> semaphoreULock(semaphoreLock);
        ASSERT_TRUE(Http::HttpClientConnection::CreateConnection(httpClientConnectionOptions, allocator));
        semaphore.wait(semaphoreULock, [&]() { return connection || connectionShutdown; });

        ASSERT_FALSE(errorOccured);
        ASSERT_FALSE(connectionShutdown);
        ASSERT_TRUE(connection);

        Http::HttpRequest request;
        Http::HttpRequestOptions requestOptions;
        requestOptions.request = &request;

        requestOptions.onStreamComplete = [&](Http::HttpStream &, int)
        {
            // do nothing.
        };
        requestOptions.onIncomingHeadersBlockDone = nullptr;
        requestOptions.onIncomingHeaders =
            [&](Http::HttpStream &, enum aws_http_header_block, const Http::HttpHeader *, std::size_t)
        {
            // do nothing
        };
        requestOptions.onIncomingBody = [&](Http::HttpStream &, const ByteCursor &)
        {
            // do nothing
        };

        request.SetMethod(ByteCursorFromCString("GET"));
        request.SetPath(uri.GetPathAndQuery());

        Http::HttpHeader hostHeader;
        hostHeader.name = ByteCursorFromCString("host");
        hostHeader.value = uri.GetHostName();
        request.AddHeader(hostHeader);

        // don't activate it and let it go out of scope.
        auto stream = connection->NewClientStream(requestOptions);
        stream = nullptr;
        connection->Close();
        semaphore.wait(semaphoreULock, [&]() { return connectionShutdown; });
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(HttpStreamUnActivated, s_TestHttpStreamUnActivated)

static int s_TestHttpCreateConnectionInvalidTlsConnectionOptions(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::TlsConnectionOptions invalidTlsConnectionOptions;
        ASSERT_FALSE(invalidTlsConnectionOptions);

        ByteCursor cursor = ByteCursorFromCString("https://aws-crt-test-stuff.s3.amazonaws.com/http_test_doc.txt");
        Io::Uri uri(cursor, allocator);

        auto hostName = uri.GetHostName();

        Aws::Crt::Io::SocketOptions socketOptions;

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(clientBootstrap);
        clientBootstrap.EnableBlockingShutdown();

        Http::HttpClientConnectionOptions httpClientConnectionOptions;
        httpClientConnectionOptions.Bootstrap = &clientBootstrap;
        httpClientConnectionOptions.OnConnectionSetupCallback = [](const std::shared_ptr<Http::HttpClientConnection> &,
                                                                   int) {};
        httpClientConnectionOptions.OnConnectionShutdownCallback = [](Http::HttpClientConnection &, int) {};
        httpClientConnectionOptions.SocketOptions = socketOptions;
        httpClientConnectionOptions.TlsOptions = invalidTlsConnectionOptions;
        httpClientConnectionOptions.HostName = String((const char *)hostName.ptr, hostName.len);
        httpClientConnectionOptions.Port = 443;

        ASSERT_FALSE(Http::HttpClientConnection::CreateConnection(httpClientConnectionOptions, allocator));
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(HttpCreateConnectionInvalidTlsConnectionOptions, s_TestHttpCreateConnectionInvalidTlsConnectionOptions)

#endif // !BYO_CRYPTO
