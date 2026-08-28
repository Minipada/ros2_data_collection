/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>

#include <aws/common/common.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>
#include <aws/crt/UUID.h>
#include <aws/iot/MqttClient.h>
#include <aws/iot/MqttCommon.h>

#include <aws/testing/aws_test_harness.h>
#include <utility>

#if !BYO_CRYPTO

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_direct_hostname, "AWS_TEST_MQTT311_DIRECT_MQTT_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_direct_port, "AWS_TEST_MQTT311_DIRECT_MQTT_PORT");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_direct_basicauth_hostname,
    "AWS_TEST_MQTT311_DIRECT_MQTT_BASIC_AUTH_HOST");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_direct_basicauth_port,
    "AWS_TEST_MQTT311_DIRECT_MQTT_BASIC_AUTH_PORT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_basicauth_username, "AWS_TEST_MQTT311_BASIC_AUTH_USERNAME");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_basicauth_password, "AWS_TEST_MQTT311_BASIC_AUTH_PASSWORD");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_direct_tls_hostname, "AWS_TEST_MQTT311_DIRECT_MQTT_TLS_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_direct_tls_port, "AWS_TEST_MQTT311_DIRECT_MQTT_TLS_PORT");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_hostname, "AWS_TEST_MQTT311_IOT_CORE_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_cert, "AWS_TEST_MQTT311_IOT_CORE_RSA_CERT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_key, "AWS_TEST_MQTT311_IOT_CORE_RSA_KEY");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_proxy_hostname, "AWS_TEST_MQTT311_PROXY_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_proxy_port, "AWS_TEST_MQTT311_PROXY_PORT");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_ws_hostname, "AWS_TEST_MQTT311_WS_MQTT_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_ws_port, "AWS_TEST_MQTT311_WS_MQTT_PORT");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_ws_basicauth_hostname,
    "AWS_TEST_MQTT311_WS_MQTT_BASIC_AUTH_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_ws_basicauth_port, "AWS_TEST_MQTT311_WS_MQTT_BASIC_AUTH_PORT");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_ws_tls_hostname, "AWS_TEST_MQTT311_WS_MQTT_TLS_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_ws_tls_port, "AWS_TEST_MQTT311_WS_MQTT_TLS_PORT");

static int s_GetEnvVariable(Aws::Crt::Allocator *allocator, const aws_string *variableName, aws_string **output)
{
    int error = aws_get_environment_value(allocator, variableName, output);
    if (error == AWS_OP_SUCCESS && output)
    {
        if (aws_string_is_valid(*output))
        {
            return AWS_OP_SUCCESS;
        }
    }
    return AWS_OP_ERR;
}

static int s_TestMqttClientResourceSafety(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();

        Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
        ASSERT_TRUE(tlsContext);

        Aws::Crt::Io::SocketOptions socketOptions;
        socketOptions.SetConnectTimeoutMs(3000);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(allocator);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
        ASSERT_TRUE(mqttClient);

        Aws::Crt::Mqtt::MqttClient mqttClientMoved = std::move(mqttClient);
        ASSERT_TRUE(mqttClientMoved);

        auto mqttConnection = mqttClientMoved.NewConnection("www.example.com", 443, socketOptions, tlsContext);

        mqttConnection->SetOnMessageHandler(
            [](Aws::Crt::Mqtt::MqttConnection &, const Aws::Crt::String &, const Aws::Crt::ByteBuf &) {});
        mqttConnection->Disconnect();
        ASSERT_TRUE(mqttConnection);

        // NOLINTNEXTLINE
        ASSERT_FALSE(mqttClient);
    }

    return AWS_ERROR_SUCCESS;
}
AWS_TEST_CASE(MqttClientResourceSafety, s_TestMqttClientResourceSafety)

static int s_TestMqttClientNewConnectionUninitializedTlsContext(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::SocketOptions socketOptions;
        socketOptions.SetConnectTimeoutMs(3000);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(allocator);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
        ASSERT_TRUE(mqttClient);

        // Intentionally use a TlsContext that hasn't been initialized.
        Aws::Crt::Io::TlsContext tlsContext;

        // Passing the uninitialized TlsContext should result in a null connection, not one in an undefined state
        auto mqttConnection = mqttClient.NewConnection("www.example.com", 443, socketOptions, tlsContext);

        ASSERT_TRUE(mqttConnection == nullptr);
        ASSERT_TRUE(aws_last_error() == AWS_ERROR_INVALID_ARGUMENT);
    }

    return AWS_ERROR_SUCCESS;
}
AWS_TEST_CASE(MqttClientNewConnectionUninitializedTlsContext, s_TestMqttClientNewConnectionUninitializedTlsContext)

static int s_ConnectAndDisconnect(std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection)
{
    std::promise<bool> connectionCompletedPromise;
    std::promise<void> connectionClosedPromise;
    auto onConnectionCompleted =
        [&](Aws::Crt::Mqtt::MqttConnection &, int errorCode, Aws::Crt::Mqtt::ReturnCode returnCode, bool)
    {
        (void)returnCode;
        if (errorCode)
        {
            connectionCompletedPromise.set_value(false);
        }
        else
        {
            connectionCompletedPromise.set_value(true);
        }
    };
    auto onDisconnect = [&](Aws::Crt::Mqtt::MqttConnection &) { connectionClosedPromise.set_value(); };
    connection->OnConnectionCompleted = std::move(onConnectionCompleted);
    connection->OnDisconnect = std::move(onDisconnect);

    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();

    if (!connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/))
    {
        printf("Failed to connect");
        return AWS_OP_ERR;
    }
    if (connectionCompletedPromise.get_future().get() == false)
    {
        printf("Connection failed");
        return AWS_OP_ERR;
    }
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }
    return AWS_OP_SUCCESS;
}

/*
 * [ConnDC-UC1] Happy path. Direct connection with minimal configuration
 */
static int s_TestMqtt311DirectConnectionMinimal(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_port, &port);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);
    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, false);
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311DirectConnectionMinimal, s_TestMqtt311DirectConnectionMinimal)

/*
 * [ConnDC-UC2] Direct connection with basic authentication
 */
static int s_TestMqtt311DirectConnectionWithBasicAuth(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_basicauth_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_basicauth_port, &port);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_basicauth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_basicauth_password, &password);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        aws_string_destroy(username);
        aws_string_destroy(password);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);
    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, false);
    connection->SetLogin(aws_string_c_str(username), aws_string_c_str(password));
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    aws_string_destroy(username);
    aws_string_destroy(password);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311DirectConnectionWithBasicAuth, s_TestMqtt311DirectConnectionWithBasicAuth)

/*
 * [ConnDC-UC3] Direct connection with TLS
 */
static int s_TestMqtt311DirectConnectionWithTLS(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_tls_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_tls_port, &port);

    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    ASSERT_TRUE(tlsCtxOptions);
    tlsCtxOptions.SetVerifyPeer(false);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, tlsContext, false);
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311DirectConnectionWithTLS, s_TestMqtt311DirectConnectionWithTLS)

/*
 * [ConnDC-UC4] Direct connection with mutual TLS
 */
static int s_TestMqtt311DirectConnectionWithMutualTLS(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *cert_path = NULL;
    struct aws_string *key_path = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_cert, &cert_path);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_key, &key_path);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(cert_path);
        aws_string_destroy(key_path);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        aws_string_c_str(cert_path), aws_string_c_str(key_path), allocator);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection =
        client.NewConnection(aws_string_c_str(endpoint), 8883, socketOptions, tlsContext, false);
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(cert_path);
    aws_string_destroy(key_path);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311DirectConnectionWithMutualTLS, s_TestMqtt311DirectConnectionWithMutualTLS)

/*
 * [ConnDC-UC5] Direct connection with HttpProxy options
 */
static int s_TestMqtt311DirectConnectionWithHttpProxy(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;
    struct aws_string *proxy_endpoint = NULL;
    struct aws_string *proxy_port = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_tls_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_direct_tls_port, &port);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_proxy_hostname, &proxy_endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_proxy_port, &proxy_port);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        aws_string_destroy(proxy_endpoint);
        aws_string_destroy(proxy_port);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    ASSERT_TRUE(tlsCtxOptions);
    tlsCtxOptions.SetVerifyPeer(false);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Http::HttpClientConnectionProxyOptions proxyOptions;
    proxyOptions.HostName = aws_string_c_str(proxy_endpoint);
    proxyOptions.Port = (uint32_t)std::stoi(aws_string_c_str(proxy_port));
    proxyOptions.ProxyConnectionType = Aws::Crt::Http::AwsHttpProxyConnectionType::Tunneling;

    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, tlsContext, false);
    connection->SetHttpProxyOptions(proxyOptions);
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    aws_string_destroy(proxy_endpoint);
    aws_string_destroy(proxy_port);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311DirectConnectionWithHttpProxy, s_TestMqtt311DirectConnectionWithHttpProxy)

//////////////////////////////////////////////////////////
// Websocket Connect Test Cases [ConnWS-UC]
//////////////////////////////////////////////////////////

/*
 * [ConnWS-UC1] Happy path. Websocket connection with minimal configuration.
 */
static int s_TestMqtt311WSConnectionMinimal(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_port, &port);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);
    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, true);
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311WSConnectionMinimal, s_TestMqtt311WSConnectionMinimal)

/*
 * [ConnWS-UC2] websocket connection with basic authentication
 */
static int s_TestMqtt311WSConnectionWithBasicAuth(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_basicauth_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_basicauth_port, &port);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_basicauth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_basicauth_password, &password);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        aws_string_destroy(username);
        aws_string_destroy(password);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);
    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, true);
    connection->SetLogin(aws_string_c_str(username), aws_string_c_str(password));
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    aws_string_destroy(username);
    aws_string_destroy(password);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311WSConnectionWithBasicAuth, s_TestMqtt311WSConnectionWithBasicAuth)

/*
 * [ConnWS-UC3] websocket connection with TLS
 */
static int s_TestMqtt311WSConnectionWithTLS(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_tls_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_tls_port, &port);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    ASSERT_TRUE(tlsCtxOptions);
    tlsCtxOptions.SetVerifyPeer(false);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, tlsContext, true);
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);
    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311WSConnectionWithTLS, s_TestMqtt311WSConnectionWithTLS)

/*
 * ConnWS-UC5] Websocket connection with HttpProxy options
 *
 */
static int s_TestMqtt311WSConnectionWithHttpProxy(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *port = NULL;
    struct aws_string *proxy_endpoint = NULL;
    struct aws_string *proxy_port = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_tls_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_ws_tls_port, &port);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_proxy_hostname, &proxy_endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_proxy_port, &proxy_port);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(port);
        aws_string_destroy(proxy_endpoint);
        aws_string_destroy(proxy_port);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    ASSERT_TRUE(tlsCtxOptions);
    tlsCtxOptions.SetVerifyPeer(false);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Http::HttpClientConnectionProxyOptions proxyOptions;
    proxyOptions.HostName = aws_string_c_str(proxy_endpoint);
    proxyOptions.Port = (uint32_t)std::stoi(aws_string_c_str(proxy_port));
    proxyOptions.ProxyConnectionType = Aws::Crt::Http::AwsHttpProxyConnectionType::Tunneling;

    Aws::Crt::Mqtt::MqttClient client;
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection = client.NewConnection(
        aws_string_c_str(endpoint), (uint32_t)std::stoi(aws_string_c_str(port)), socketOptions, tlsContext, true);
    connection->SetHttpProxyOptions(proxyOptions);
    int connectResult = s_ConnectAndDisconnect(connection);
    ASSERT_SUCCESS(connectResult);

    aws_string_destroy(endpoint);
    aws_string_destroy(port);
    aws_string_destroy(proxy_endpoint);
    aws_string_destroy(proxy_port);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt311WSConnectionWithHttpProxy, s_TestMqtt311WSConnectionWithHttpProxy)

#endif // !BYO_CRYPTO
