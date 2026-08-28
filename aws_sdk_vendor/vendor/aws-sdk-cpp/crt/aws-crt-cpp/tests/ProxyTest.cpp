/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/environment.h>

#include <aws/crt/Api.h>
#include <aws/crt/UUID.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/crt/http/HttpConnectionManager.h>
#include <aws/crt/http/HttpProxyStrategy.h>
#include <aws/crt/http/HttpRequestResponse.h>
#include <aws/crt/io/Uri.h>
#include <aws/iot/MqttClient.h>

#include <aws/testing/aws_test_harness.h>

#include <condition_variable>
#include <mutex>

using namespace Aws;
using namespace Aws::Crt;
using namespace Aws::Crt::Auth;
using namespace Aws::Crt::Http;
using namespace Aws::Crt::Io;

#if !BYO_CRYPTO

struct ProxyIntegrationTestState
{
    ProxyIntegrationTestState(struct aws_allocator *allocator)
        : m_allocator(allocator), m_streamComplete(false), m_streamStatusCode(0), m_credentialsFetched(false),
          m_mqttConnectComplete(false), m_mqttDisconnectComplete(false), m_mqttErrorCode(0), m_BasicUsername(NULL),
          m_BasicPassword(NULL)
    {
    }

    ~ProxyIntegrationTestState()
    {
        aws_string_destroy(m_BasicUsername);
        aws_string_destroy(m_BasicPassword);
    }

    struct aws_allocator *m_allocator;

    std::condition_variable m_signal;
    std::mutex m_lock;

    bool m_streamComplete;
    int32_t m_streamStatusCode;
    Aws::Crt::StringStream m_responseBuffer;
    bool m_credentialsFetched;

    bool m_mqttConnectComplete;
    bool m_mqttDisconnectComplete;
    int m_mqttErrorCode;

    struct aws_string *m_BasicUsername;
    struct aws_string *m_BasicPassword;

    HttpClientConnectionProxyOptions m_proxyOptions;
    HttpClientConnectionOptions m_connectionOptions;
    std::shared_ptr<TlsContext> m_tlsContext;
    std::shared_ptr<TlsContext> m_proxyTlsContext;
    std::shared_ptr<EventLoopGroup> m_eventLoopGroup;
    std::shared_ptr<DefaultHostResolver> m_hostResolver;
    std::shared_ptr<ClientBootstrap> m_clientBootstrap;
    std::shared_ptr<HttpClientConnectionManager> m_connectionManager;
    std::shared_ptr<HttpClientConnection> m_connection;
    std::shared_ptr<HttpRequest> m_request;
    std::shared_ptr<HttpClientStream> m_stream;
    std::shared_ptr<ICredentialsProvider> m_x509Provider;
    std::shared_ptr<TlsContext> m_x509TlsContext;
    std::shared_ptr<Credentials> m_credentials;
    std::shared_ptr<Iot::MqttClient> m_mqttClient;
    std::shared_ptr<Mqtt::MqttConnection> m_mqttConnection;
};

AWS_STATIC_STRING_FROM_LITERAL(s_https_endpoint, "https://s3.amazonaws.com");
AWS_STATIC_STRING_FROM_LITERAL(s_http_endpoint, "http://www.example.com");

static void s_InitializeProxyTestSupport(ProxyIntegrationTestState &testState)
{

    struct aws_allocator *allocator = testState.m_allocator;

    testState.m_eventLoopGroup = Aws::Crt::MakeShared<EventLoopGroup>(allocator, static_cast<uint16_t>(1), allocator);
    testState.m_hostResolver =
        Aws::Crt::MakeShared<DefaultHostResolver>(allocator, *testState.m_eventLoopGroup, 8, 30, allocator);
    testState.m_clientBootstrap = Aws::Crt::MakeShared<ClientBootstrap>(
        allocator, *testState.m_eventLoopGroup, *testState.m_hostResolver, allocator);
}

static void s_InitializeProxiedConnectionOptions(ProxyIntegrationTestState &testState, struct aws_byte_cursor url)
{
    struct aws_allocator *allocator = testState.m_allocator;

    Io::Uri uri(url, allocator);
    auto hostName = uri.GetHostName();
    auto scheme = uri.GetScheme();
    bool useTls = true;
    if (aws_byte_cursor_eq_c_str_ignore_case(&scheme, "http"))
    {
        useTls = false;
    }

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(10000);

    testState.m_connectionOptions.Bootstrap = testState.m_clientBootstrap.get();
    testState.m_connectionOptions.SocketOptions = socketOptions;
    testState.m_connectionOptions.HostName = String((const char *)hostName.ptr, hostName.len);
    testState.m_connectionOptions.Port = useTls ? 443 : 80;
    testState.m_connectionOptions.ProxyOptions = testState.m_proxyOptions;

    if (useTls)
    {
        Aws::Crt::Io::TlsConnectionOptions tlsConnectionOptions;
        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
        testState.m_tlsContext =
            Aws::Crt::MakeShared<TlsContext>(allocator, tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        tlsConnectionOptions = testState.m_tlsContext->NewConnectionOptions();
        tlsConnectionOptions.SetServerName(hostName);
        testState.m_connectionOptions.TlsOptions = tlsConnectionOptions;
    }
}

static void s_InitializeProxiedConnectionManager(ProxyIntegrationTestState &testState, struct aws_byte_cursor url)
{
    struct aws_allocator *allocator = testState.m_allocator;

    s_InitializeProxyTestSupport(testState);

    s_InitializeProxiedConnectionOptions(testState, url);

    Http::HttpClientConnectionManagerOptions connectionManagerOptions;
    connectionManagerOptions.ConnectionOptions = testState.m_connectionOptions;

    testState.m_connectionManager =
        Http::HttpClientConnectionManager::NewClientConnectionManager(connectionManagerOptions, allocator);
}

static void s_InitializeProxiedRawConnection(ProxyIntegrationTestState &testState, struct aws_byte_cursor url)
{
    struct aws_allocator *allocator = testState.m_allocator;

    s_InitializeProxyTestSupport(testState);

    s_InitializeProxiedConnectionOptions(testState, url);

    int acquisitionErrorCode = 0;
    std::shared_ptr<Http::HttpClientConnection> connection;

    testState.m_connectionOptions.OnConnectionSetupCallback =
        [&](std::shared_ptr<Http::HttpClientConnection> newConnection, int errorCode)
    {
        {
            std::lock_guard<std::mutex> lockGuard(testState.m_lock);

            acquisitionErrorCode = errorCode;
            if (!errorCode)
            {
                connection = newConnection;
            }
        }
        testState.m_signal.notify_one();
    };

    testState.m_connectionOptions.OnConnectionShutdownCallback = [&](HttpClientConnection & /*newConnection*/,
                                                                     int /*errorCode*/) {};

    HttpClientConnection::CreateConnection(testState.m_connectionOptions, allocator);

    std::unique_lock<std::mutex> uniqueLock(testState.m_lock);
    testState.m_signal.wait(uniqueLock, [&]() { return connection != nullptr || acquisitionErrorCode != 0; });

    testState.m_connection = connection;
}

static void s_AcquireProxyTestHttpConnection(ProxyIntegrationTestState &testState)
{

    int acquisitionErrorCode = 0;
    std::shared_ptr<Http::HttpClientConnection> connection;

    auto onConnectionAvailable = [&](std::shared_ptr<Http::HttpClientConnection> newConnection, int errorCode)
    {
        {
            std::lock_guard<std::mutex> lockGuard(testState.m_lock);

            acquisitionErrorCode = errorCode;
            if (!errorCode)
            {
                connection = newConnection;
            }
        }
        testState.m_signal.notify_one();
    };

    testState.m_connectionManager->AcquireConnection(onConnectionAvailable);
    std::unique_lock<std::mutex> uniqueLock(testState.m_lock);
    testState.m_signal.wait(uniqueLock, [&]() { return connection != nullptr || acquisitionErrorCode != 0; });

    testState.m_connection = connection;
}

AWS_STATIC_STRING_FROM_LITERAL(s_httpProxyHostEnvVariable, "AWS_TEST_HTTP_PROXY_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_httpProxyPortEnvVariable, "AWS_TEST_HTTP_PROXY_PORT");
AWS_STATIC_STRING_FROM_LITERAL(s_httpsProxyHostEnvVariable, "AWS_TEST_HTTPS_PROXY_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_httpsProxyPortEnvVariable, "AWS_TEST_HTTPS_PROXY_PORT");
AWS_STATIC_STRING_FROM_LITERAL(s_httpProxyBasicHostEnvVariable, "AWS_TEST_HTTP_PROXY_BASIC_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_httpProxyBasicPortEnvVariable, "AWS_TEST_HTTP_PROXY_BASIC_PORT");

enum HttpProxyTestHostType
{
    Http,
    Https,
    HttpBasic,
};

static const struct aws_string *s_GetProxyHostVariable(enum HttpProxyTestHostType proxyHostType)
{
    switch (proxyHostType)
    {
        case HttpProxyTestHostType::Http:
            return s_httpProxyHostEnvVariable;

        case HttpProxyTestHostType::Https:
            return s_httpsProxyHostEnvVariable;

        case HttpProxyTestHostType::HttpBasic:
            return s_httpProxyBasicHostEnvVariable;

        default:
            return NULL;
    }
}

static const struct aws_string *s_GetProxyPortVariable(enum HttpProxyTestHostType proxyHostType)
{
    switch (proxyHostType)
    {
        case HttpProxyTestHostType::Http:
            return s_httpProxyPortEnvVariable;

        case HttpProxyTestHostType::Https:
            return s_httpsProxyPortEnvVariable;

        case HttpProxyTestHostType::HttpBasic:
            return s_httpProxyBasicPortEnvVariable;

        default:
            return NULL;
    }
}

static int s_InitializeProxyEnvironmentalOptions(
    ProxyIntegrationTestState &testState,
    enum HttpProxyTestHostType proxyHostType)
{
    struct aws_allocator *allocator = testState.m_allocator;

    struct aws_string *proxy_host_name = NULL;
    struct aws_string *proxy_port = NULL;

    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_GetProxyHostVariable(proxyHostType), &proxy_host_name));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_GetProxyPortVariable(proxyHostType), &proxy_port));

    testState.m_proxyOptions.HostName = Aws::Crt::String(aws_string_c_str(proxy_host_name));
    testState.m_proxyOptions.Port = static_cast<uint32_t>(atoi(aws_string_c_str(proxy_port)));

    aws_string_destroy(proxy_host_name);
    aws_string_destroy(proxy_port);

    return AWS_OP_SUCCESS;
}

static int s_TestConnectionManagerTunnelingProxyHttp(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Http);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxiedConnectionManager(testState, aws_byte_cursor_from_string(s_https_endpoint));

        s_AcquireProxyTestHttpConnection(testState);
        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(ConnectionManagerTunnelingProxyHttp, s_TestConnectionManagerTunnelingProxyHttp)

static void s_InitializeTlsToProxy(ProxyIntegrationTestState &testState)
{
    struct aws_allocator *allocator = testState.m_allocator;

    Aws::Crt::Io::TlsConnectionOptions tlsConnectionOptions;
    Aws::Crt::Io::TlsContextOptions proxyTlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    proxyTlsCtxOptions.SetVerifyPeer(false);

    testState.m_proxyTlsContext =
        Aws::Crt::MakeShared<TlsContext>(allocator, proxyTlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

    tlsConnectionOptions = testState.m_proxyTlsContext->NewConnectionOptions();
    ByteCursor proxyName = ByteCursorFromString(testState.m_proxyOptions.HostName);
    tlsConnectionOptions.SetServerName(proxyName);

    testState.m_proxyOptions.TlsOptions = tlsConnectionOptions;
}

static int s_TestConnectionManagerTunnelingProxyHttps(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Https);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeTlsToProxy(testState);

        s_InitializeProxiedConnectionManager(testState, aws_byte_cursor_from_string(s_https_endpoint));

        s_AcquireProxyTestHttpConnection(testState);
        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(ConnectionManagerTunnelingProxyHttps, s_TestConnectionManagerTunnelingProxyHttps)

static int s_TestConnectionManagerTunnelingProxyHttpsInvalidTlsOptions(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Https);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeTlsToProxy(testState);

        s_InitializeProxyTestSupport(testState);

        s_InitializeProxiedConnectionOptions(testState, aws_byte_cursor_from_string(s_https_endpoint));

        Http::HttpClientConnectionManagerOptions connectionManagerOptions;
        connectionManagerOptions.ConnectionOptions = testState.m_connectionOptions;

        /* Reset TLS Options, making them invalid. */
        auto &proxyOpts = connectionManagerOptions.ConnectionOptions.ProxyOptions;
        ASSERT_TRUE(proxyOpts);

        proxyOpts->TlsOptions = Aws::Crt::Io::TlsConnectionOptions();
        ASSERT_FALSE(*proxyOpts->TlsOptions);

        std::shared_ptr<Aws::Crt::Http::HttpClientConnectionManager> connManager =
            Http::HttpClientConnectionManager::NewClientConnectionManager(connectionManagerOptions, allocator);

        ASSERT_TRUE(connManager == nullptr);
        ASSERT_TRUE(aws_last_error() == AWS_ERROR_INVALID_ARGUMENT);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    ConnectionManagerTunnelingProxyHttpsInvalidTlsOptions,
    s_TestConnectionManagerTunnelingProxyHttpsInvalidTlsOptions)

static void s_MakeForwardingTestRequest(ProxyIntegrationTestState &testState)
{
    struct aws_allocator *allocator = testState.m_allocator;

    testState.m_request = Aws::Crt::MakeShared<HttpRequest>(allocator, allocator);
    testState.m_request->SetMethod(ByteCursorFromCString("GET"));
    testState.m_request->SetPath(ByteCursorFromCString("/"));

    HttpRequestOptions requestOptions;
    requestOptions.request = testState.m_request.get();

    requestOptions.onIncomingBody = [&testState](Http::HttpStream &, const ByteCursor &data)
    {
        std::lock_guard<std::mutex> lock(testState.m_lock);

        Aws::Crt::String dataString((const char *)data.ptr, data.len);
        testState.m_responseBuffer << dataString;
    };

    requestOptions.onIncomingHeaders =
        [&testState](Http::HttpStream &, enum aws_http_header_block, const Http::HttpHeader *, std::size_t)
    {
        std::lock_guard<std::mutex> lock(testState.m_lock);
        if (testState.m_streamStatusCode == 0)
        {
            testState.m_streamStatusCode = testState.m_stream->GetResponseStatusCode();
        }
    };

    requestOptions.onStreamComplete = [&testState](Http::HttpStream & /*stream*/, int /*errorCode*/)
    {
        {
            std::lock_guard<std::mutex> lock(testState.m_lock);
            testState.m_streamComplete = true;
        }
        testState.m_signal.notify_one();
    };

    testState.m_stream = testState.m_connection->NewClientStream(requestOptions);
    testState.m_stream->Activate();
}

static void s_WaitOnTestStream(ProxyIntegrationTestState &testState)
{
    std::unique_lock<std::mutex> uniqueLock(testState.m_lock);
    testState.m_signal.wait(
        uniqueLock, [&testState]() { return testState.m_streamComplete || testState.m_stream == nullptr; });
}

static int s_TestConnectionManagerForwardingProxy(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Http);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Forwarding;

        s_InitializeProxiedConnectionManager(testState, aws_byte_cursor_from_string(s_http_endpoint));

        s_AcquireProxyTestHttpConnection(testState);
        ASSERT_TRUE(testState.m_connection != nullptr);

        s_MakeForwardingTestRequest(testState);

        s_WaitOnTestStream(testState);

        ASSERT_TRUE(testState.m_streamStatusCode == 200);
        Aws::Crt::String response = testState.m_responseBuffer.str();

        ASSERT_TRUE(response.find("example") != Aws::Crt::String::npos);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(ConnectionManagerForwardingProxy, s_TestConnectionManagerForwardingProxy)

AWS_STATIC_STRING_FROM_LITERAL(s_BasicAuthUsernameEnvVariable, "AWS_TEST_BASIC_AUTH_USERNAME");
AWS_STATIC_STRING_FROM_LITERAL(s_BasicAuthPasswordEnvVariable, "AWS_TEST_BASIC_AUTH_PASSWORD");

static int s_InitializeBasicAuthParameters(ProxyIntegrationTestState &testState)
{
    struct aws_allocator *allocator = testState.m_allocator;

    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_BasicAuthUsernameEnvVariable, &testState.m_BasicUsername));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_BasicAuthPasswordEnvVariable, &testState.m_BasicPassword));

    return AWS_OP_SUCCESS;
}

static int s_InitializeDeprecatedBasicAuth(ProxyIntegrationTestState &testState)
{
    testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Legacy;
    testState.m_proxyOptions.AuthType = AwsHttpProxyAuthenticationType::Basic;

    ASSERT_SUCCESS(s_InitializeBasicAuthParameters(testState));

    testState.m_proxyOptions.BasicAuthUsername = aws_string_c_str(testState.m_BasicUsername);
    testState.m_proxyOptions.BasicAuthPassword = aws_string_c_str(testState.m_BasicPassword);

    return AWS_OP_SUCCESS;
}

static int s_TestConnectionManagerTunnelingProxyBasicAuthDeprecated(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeDeprecatedBasicAuth(testState));
        s_InitializeProxiedConnectionManager(testState, aws_byte_cursor_from_string(s_https_endpoint));

        s_AcquireProxyTestHttpConnection(testState);
        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    ConnectionManagerTunnelingProxyBasicAuthDeprecated,
    s_TestConnectionManagerTunnelingProxyBasicAuthDeprecated)

static int s_InitializeBasicAuth(ProxyIntegrationTestState &testState)
{
    struct aws_allocator *allocator = testState.m_allocator;

    HttpProxyStrategyBasicAuthConfig basicAuthConfig;
    basicAuthConfig.ConnectionType = AwsHttpProxyConnectionType::Tunneling;

    ASSERT_SUCCESS(s_InitializeBasicAuthParameters(testState));

    basicAuthConfig.Username = aws_string_c_str(testState.m_BasicUsername);
    basicAuthConfig.Password = aws_string_c_str(testState.m_BasicPassword);

    testState.m_proxyOptions.ProxyStrategy =
        HttpProxyStrategy::CreateBasicHttpProxyStrategy(basicAuthConfig, allocator);

    return AWS_OP_SUCCESS;
}

static int s_TestConnectionManagerTunnelingProxyBasicAuth(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        ASSERT_SUCCESS(s_InitializeBasicAuth(testState));

        s_InitializeProxiedConnectionManager(testState, aws_byte_cursor_from_string(s_https_endpoint));

        s_AcquireProxyTestHttpConnection(testState);
        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(ConnectionManagerTunnelingProxyBasicAuth, s_TestConnectionManagerTunnelingProxyBasicAuth)

static int s_TestDirectConnectionTunnelingProxyHttp(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Http);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxiedRawConnection(testState, aws_byte_cursor_from_string(s_https_endpoint));

        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(DirectConnectionTunnelingProxyHttp, s_TestDirectConnectionTunnelingProxyHttp)

static int s_TestDirectConnectionTunnelingProxyHttps(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Https);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeTlsToProxy(testState);

        s_InitializeProxiedRawConnection(testState, aws_byte_cursor_from_string(s_https_endpoint));

        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(DirectConnectionTunnelingProxyHttps, s_TestDirectConnectionTunnelingProxyHttps)

static int s_TestDirectConnectionTunnelingProxyHttpsInvalidTlsOptions(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Https);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeTlsToProxy(testState);

        s_InitializeProxyTestSupport(testState);

        s_InitializeProxiedConnectionOptions(testState, aws_byte_cursor_from_string(s_https_endpoint));

        /* Reset TLS Options, making them invalid. */
        testState.m_connectionOptions.TlsOptions = Aws::Crt::Io::TlsConnectionOptions();
        ASSERT_FALSE(*testState.m_connectionOptions.TlsOptions);

        testState.m_connectionOptions.OnConnectionSetupCallback =
            [](const std::shared_ptr<Http::HttpClientConnection> &, int) {};
        testState.m_connectionOptions.OnConnectionShutdownCallback = [](Http::HttpClientConnection &, int) {};

        ASSERT_FALSE(HttpClientConnection::CreateConnection(testState.m_connectionOptions, allocator));
        ASSERT_TRUE(aws_last_error() == AWS_ERROR_INVALID_ARGUMENT);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    DirectConnectionTunnelingProxyHttpsInvalidTlsOptions,
    s_TestDirectConnectionTunnelingProxyHttpsInvalidTlsOptions)

static int s_TestDirectConnectionForwardingProxy(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Http);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Forwarding;

        s_InitializeProxiedRawConnection(testState, aws_byte_cursor_from_string(s_http_endpoint));
        ASSERT_TRUE(testState.m_connection != nullptr);

        s_MakeForwardingTestRequest(testState);

        s_WaitOnTestStream(testState);

        ASSERT_TRUE(testState.m_streamStatusCode == 200);
        Aws::Crt::String response = testState.m_responseBuffer.str();

        ASSERT_TRUE(response.find("example") != Aws::Crt::String::npos);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(DirectConnectionForwardingProxy, s_TestDirectConnectionForwardingProxy)

static int s_TestDirectConnectionTunnelingProxyBasicAuthDeprecated(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeDeprecatedBasicAuth(testState));

        s_InitializeProxiedRawConnection(testState, aws_byte_cursor_from_string(s_https_endpoint));
        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    DirectConnectionTunnelingProxyBasicAuthDeprecated,
    s_TestDirectConnectionTunnelingProxyBasicAuthDeprecated)

static int s_TestDirectConnectionTunnelingProxyBasicAuth(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        ASSERT_SUCCESS(s_InitializeBasicAuth(testState));

        s_InitializeProxiedRawConnection(testState, aws_byte_cursor_from_string(s_https_endpoint));
        ASSERT_TRUE(testState.m_connection != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(DirectConnectionTunnelingProxyBasicAuth, s_TestDirectConnectionTunnelingProxyBasicAuth)

AWS_STATIC_STRING_FROM_LITERAL(s_x509EndpointVariable, "AWS_TEST_MQTT311_IOT_CORE_X509_ENDPOINT");
AWS_STATIC_STRING_FROM_LITERAL(s_x509RoleAliasVariable, "AWS_TEST_MQTT311_IOT_CORE_X509_ROLE_ALIAS");
AWS_STATIC_STRING_FROM_LITERAL(s_x509ThingNameVariable, "AWS_TEST_MQTT311_IOT_CORE_X509_THING_NAME");
AWS_STATIC_STRING_FROM_LITERAL(s_CertificatePathVariable, "AWS_TEST_MQTT311_IOT_CORE_X509_CERT");
AWS_STATIC_STRING_FROM_LITERAL(s_KeyPathVariable, "AWS_TEST_MQTT311_IOT_CORE_X509_KEY");
AWS_STATIC_STRING_FROM_LITERAL(s_RootCAPathVariable, "AWS_TEST_MQTT311_IOT_CORE_X509_CA");

static int s_InitializeX509Provider(ProxyIntegrationTestState &testState)
{

    struct aws_allocator *allocator = testState.m_allocator;

    struct aws_string *x509Endpoint = NULL;
    struct aws_string *x509RoleAlias = NULL;
    struct aws_string *x509ThingName = NULL;
    struct aws_string *x509CertificatePath = NULL;
    struct aws_string *x509KeyPath = NULL;
    struct aws_string *x509RootCAPath = NULL;

    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_x509EndpointVariable, &x509Endpoint));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_x509RoleAliasVariable, &x509RoleAlias));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_x509ThingNameVariable, &x509ThingName));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_CertificatePathVariable, &x509CertificatePath));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_KeyPathVariable, &x509KeyPath));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_RootCAPathVariable, &x509RootCAPath));

    CredentialsProviderX509Config providerConfig;
    providerConfig.Bootstrap = testState.m_clientBootstrap.get();
    providerConfig.Endpoint = aws_string_c_str(x509Endpoint);
    providerConfig.RoleAlias = aws_string_c_str(x509RoleAlias);
    providerConfig.ThingName = aws_string_c_str(x509ThingName);
    providerConfig.ProxyOptions = testState.m_proxyOptions;

    Aws::Crt::Io::TlsContextOptions x509TlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        (const char *)x509CertificatePath->bytes, (const char *)x509KeyPath->bytes, allocator);
    x509TlsCtxOptions.OverrideDefaultTrustStore(NULL, (const char *)x509RootCAPath->bytes);

    testState.m_x509TlsContext =
        Aws::Crt::MakeShared<TlsContext>(allocator, x509TlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

    Aws::Crt::Io::TlsConnectionOptions tlsConnectionOptions = testState.m_x509TlsContext->NewConnectionOptions();
    providerConfig.TlsOptions = tlsConnectionOptions;

    testState.m_x509Provider = CredentialsProvider::CreateCredentialsProviderX509(providerConfig, allocator);
    ASSERT_NOT_NULL(testState.m_x509Provider.get());

    aws_string_destroy(x509Endpoint);
    aws_string_destroy(x509RoleAlias);
    aws_string_destroy(x509ThingName);
    aws_string_destroy(x509CertificatePath);
    aws_string_destroy(x509KeyPath);
    aws_string_destroy(x509RootCAPath);

    return AWS_OP_SUCCESS;
}

static int s_X509GetCredentials(ProxyIntegrationTestState &testState)
{

    auto credentialsResolved = [&testState](std::shared_ptr<Credentials> credentials, int /*errorCode*/)
    {
        {
            std::lock_guard<std::mutex> lock(testState.m_lock);
            testState.m_credentials = credentials;
            testState.m_credentialsFetched = true;
        }

        testState.m_signal.notify_one();
    };

    ASSERT_TRUE(testState.m_x509Provider->GetCredentials(credentialsResolved));

    return AWS_OP_SUCCESS;
}

static void s_WaitOnCredentials(ProxyIntegrationTestState &testState)
{
    std::unique_lock<std::mutex> uniqueLock(testState.m_lock);
    testState.m_signal.wait(uniqueLock, [&testState]() { return testState.m_credentialsFetched; });
}

static int s_TestX509ProxyHttpGetCredentials(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Http);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);
        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        s_X509GetCredentials(testState);
        s_WaitOnCredentials(testState);

        ASSERT_TRUE(testState.m_credentials != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(X509ProxyHttpGetCredentials, s_TestX509ProxyHttpGetCredentials)

static int s_TestX509ProxyHttpsGetCredentials(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Https);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);
        s_InitializeTlsToProxy(testState);

        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        s_X509GetCredentials(testState);
        s_WaitOnCredentials(testState);

        ASSERT_TRUE(testState.m_credentials != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(X509ProxyHttpsGetCredentials, s_TestX509ProxyHttpsGetCredentials)

static int s_TestX509ProxyBasicAuthDeprecatedGetCredentials(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeDeprecatedBasicAuth(testState));

        s_InitializeProxyTestSupport(testState);
        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        s_X509GetCredentials(testState);
        s_WaitOnCredentials(testState);

        ASSERT_TRUE(testState.m_credentials != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(X509ProxyBasicAuthDeprecatedGetCredentials, s_TestX509ProxyBasicAuthDeprecatedGetCredentials)

static int s_TestX509ProxyBasicAuthGetCredentials(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeBasicAuth(testState));
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);
        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        s_X509GetCredentials(testState);
        s_WaitOnCredentials(testState);

        ASSERT_TRUE(testState.m_credentials != nullptr);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(X509ProxyBasicAuthGetCredentials, s_TestX509ProxyBasicAuthGetCredentials)

AWS_STATIC_STRING_FROM_LITERAL(s_AwsIotSigningRegionVariable, "AWS_TEST_MQTT311_IOT_CORE_REGION");
AWS_STATIC_STRING_FROM_LITERAL(s_AwsIotMqttEndpointVariable, "AWS_TEST_MQTT311_IOT_CORE_HOST");

static int s_BuildMqttConnection(ProxyIntegrationTestState &testState)
{
    struct aws_allocator *allocator = testState.m_allocator;

    testState.m_mqttClient =
        Aws::Crt::MakeShared<Aws::Iot::MqttClient>(allocator, *testState.m_clientBootstrap, allocator);

    struct aws_string *awsIotSigningRegion = NULL;
    struct aws_string *awsIotEndpoint = NULL;
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_AwsIotSigningRegionVariable, &awsIotSigningRegion));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_AwsIotMqttEndpointVariable, &awsIotEndpoint));

    Iot::WebsocketConfig config(Aws::Crt::String(aws_string_c_str(awsIotSigningRegion)), testState.m_x509Provider);
    config.ProxyOptions = testState.m_proxyOptions;

    Iot::MqttClientConnectionConfigBuilder builder = Aws::Iot::MqttClientConnectionConfigBuilder(config);
    builder.WithEndpoint(Aws::Crt::String(aws_string_c_str(awsIotEndpoint)));

    testState.m_mqttConnection = testState.m_mqttClient->NewConnection(builder.Build());
    ASSERT_NOT_NULL(testState.m_mqttConnection.get());

    aws_string_destroy(awsIotSigningRegion);
    aws_string_destroy(awsIotEndpoint);

    return AWS_OP_SUCCESS;
}

static int s_ConnectToIotCore(ProxyIntegrationTestState &testState)
{

    testState.m_mqttConnection->OnConnectionCompleted =
        [&testState](Mqtt::MqttConnection &, int errorCode, Mqtt::ReturnCode, bool)
    {
        std::lock_guard<std::mutex> lock(testState.m_lock);

        testState.m_mqttConnectComplete = true;
        testState.m_mqttErrorCode = errorCode;
        testState.m_signal.notify_one();
    };

    testState.m_mqttConnection->OnDisconnect = [&testState](Mqtt::MqttConnection &)
    {
        std::lock_guard<std::mutex> lock(testState.m_lock);

        testState.m_mqttDisconnectComplete = true;
        testState.m_signal.notify_one();
    };

    Aws::Crt::UUID uuid;

    Aws::Crt::StringStream clientId;
    clientId << "IntegrationTest-" << uuid.ToString();

    ASSERT_TRUE(testState.m_mqttConnection->Connect(clientId.str().c_str(), true));

    return AWS_OP_SUCCESS;
}

static void s_WaitForIotCoreConnection(ProxyIntegrationTestState &testState)
{
    std::unique_lock<std::mutex> uniqueLock(testState.m_lock);
    testState.m_signal.wait(uniqueLock, [&testState]() { return testState.m_mqttConnectComplete; });
}

static void s_WaitForIotCoreDisconnect(ProxyIntegrationTestState &testState)
{
    std::unique_lock<std::mutex> uniqueLock(testState.m_lock);
    testState.m_signal.wait(uniqueLock, [&testState]() { return testState.m_mqttDisconnectComplete; });
}

static int s_TestMqttOverWebsocketsViaHttpProxy(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Http);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);
        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        ASSERT_SUCCESS(s_BuildMqttConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MqttOverWebsocketsViaHttpProxy, s_TestMqttOverWebsocketsViaHttpProxy)

static int s_TestMqttOverWebsocketsViaHttpsProxy(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Https);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);
        s_InitializeTlsToProxy(testState);

        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        ASSERT_SUCCESS(s_BuildMqttConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MqttOverWebsocketsViaHttpsProxy, s_TestMqttOverWebsocketsViaHttpsProxy)

static int s_TestMqttOverWebsocketsViaHttpProxyBasicAuthDeprecated(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeDeprecatedBasicAuth(testState));

        s_InitializeProxyTestSupport(testState);
        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        ASSERT_SUCCESS(s_BuildMqttConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    MqttOverWebsocketsViaHttpProxyBasicAuthDeprecated,
    s_TestMqttOverWebsocketsViaHttpProxyBasicAuthDeprecated)

static int s_TestMqttOverWebsocketsViaHttpProxyBasicAuth(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeBasicAuth(testState));
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);
        ASSERT_SUCCESS(s_InitializeX509Provider(testState));

        ASSERT_SUCCESS(s_BuildMqttConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MqttOverWebsocketsViaHttpProxyBasicAuth, s_TestMqttOverWebsocketsViaHttpProxyBasicAuth)

static int s_BuildMqttAlpnConnection(ProxyIntegrationTestState &testState)
{
    struct aws_allocator *allocator = testState.m_allocator;

    testState.m_mqttClient =
        Aws::Crt::MakeShared<Aws::Iot::MqttClient>(allocator, *testState.m_clientBootstrap, allocator);

    struct aws_string *awsIotEndpoint = NULL;
    struct aws_string *certificatePath = NULL;
    struct aws_string *keyPath = NULL;
    struct aws_string *rootCAPath = NULL;

    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_AwsIotMqttEndpointVariable, &awsIotEndpoint));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_CertificatePathVariable, &certificatePath));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_KeyPathVariable, &keyPath));
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_RootCAPathVariable, &rootCAPath));

    Iot::MqttClientConnectionConfigBuilder builder =
        Aws::Iot::MqttClientConnectionConfigBuilder(aws_string_c_str(certificatePath), aws_string_c_str(keyPath));
    builder.WithCertificateAuthority(aws_string_c_str(rootCAPath));
    builder.WithEndpoint(Aws::Crt::String(aws_string_c_str(awsIotEndpoint)));
    builder.WithHttpProxyOptions(testState.m_proxyOptions);

    testState.m_mqttConnection = testState.m_mqttClient->NewConnection(builder.Build());
    ASSERT_NOT_NULL(testState.m_mqttConnection.get());

    aws_string_destroy(awsIotEndpoint);
    aws_string_destroy(certificatePath);
    aws_string_destroy(keyPath);
    aws_string_destroy(rootCAPath);

    return AWS_OP_SUCCESS;
}

static int s_TestMqttViaHttpProxyAlpn(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Http);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);

        ASSERT_SUCCESS(s_BuildMqttAlpnConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MqttViaHttpProxyAlpn, s_TestMqttViaHttpProxyAlpn)

static int s_TestMqttViaHttpsProxyAlpn(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::Https);
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);
        s_InitializeTlsToProxy(testState);

        ASSERT_SUCCESS(s_BuildMqttAlpnConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MqttViaHttpsProxyAlpn, s_TestMqttViaHttpsProxyAlpn)

static int s_TestMqttViaHttpProxyAlpnBasicAuthDeprecated(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeDeprecatedBasicAuth(testState));

        s_InitializeProxyTestSupport(testState);

        ASSERT_SUCCESS(s_BuildMqttAlpnConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MqttViaHttpProxyAlpnBasicAuthDeprecated, s_TestMqttViaHttpProxyAlpnBasicAuthDeprecated)

static int s_TestMqttViaHttpProxyAlpnBasicAuth(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        ProxyIntegrationTestState testState(allocator);
        s_InitializeProxyEnvironmentalOptions(testState, HttpProxyTestHostType::HttpBasic);
        ASSERT_SUCCESS(s_InitializeBasicAuth(testState));
        testState.m_proxyOptions.ProxyConnectionType = AwsHttpProxyConnectionType::Tunneling;

        s_InitializeProxyTestSupport(testState);

        ASSERT_SUCCESS(s_BuildMqttAlpnConnection(testState));
        ASSERT_SUCCESS(s_ConnectToIotCore(testState));

        s_WaitForIotCoreConnection(testState);
        testState.m_mqttConnection->Disconnect();
        s_WaitForIotCoreDisconnect(testState);
    }

    /* now let everything tear down and make sure we don't leak or deadlock.*/
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MqttViaHttpProxyAlpnBasicAuth, s_TestMqttViaHttpProxyAlpnBasicAuth)

#endif // !BYO_CRYPTO
