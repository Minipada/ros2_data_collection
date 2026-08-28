/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/common/environment.h>
#include <aws/crt/Api.h>
#include <aws/crt/UUID.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/crt/http/HttpProxyStrategy.h>
#include <aws/crt/mqtt/Mqtt5Packets.h>
#include <aws/iot/Mqtt5Client.h>
#include <aws/iot/MqttCommon.h>
#include <aws/testing/aws_test_harness.h>

#include <atomic>
#include <utility>

using namespace Aws::Crt;
using namespace Aws::Crt::Mqtt5;

//////////////////////////////////////////////////////////
// Creation Test Cases [New-UC] (runs regardless of byo-crypto)
//////////////////////////////////////////////////////////

static void s_setupConnectionLifeCycle(
    Mqtt5::Mqtt5ClientOptions &mqtt5Options,
    std::promise<bool> &connectionPromise,
    std::promise<void> &stoppedPromise,
    const char *clientName = "Client")
{
    mqtt5Options.WithClientConnectionSuccessCallback(
        [&connectionPromise, clientName](const OnConnectionSuccessEventData &)
        {
            printf("[MQTT5]%s Connection Success.\n", clientName);
            connectionPromise.set_value(true);
        });

    mqtt5Options.WithClientConnectionFailureCallback(
        [&connectionPromise, clientName](const OnConnectionFailureEventData &eventData)
        {
            printf(
                "[MQTT5]%s Connection failed with error : %s\n", clientName, aws_error_debug_str(eventData.errorCode));
            connectionPromise.set_value(false);
        });

    mqtt5Options.WithClientStoppedCallback(
        [&stoppedPromise, clientName](const OnStoppedEventData &)
        {
            printf("[MQTT5]%s Stopped\n", clientName);
            stoppedPromise.set_value();
        });
}

/*
 * [New-UC1] Happy path. Minimal creation and cleanup
 */
static int s_TestMqtt5NewClientMin(Aws::Crt::Allocator *allocator, void *)
{
    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    // Hardcoded the host name and port for creation test
    mqtt5Options.WithHostName("localhost").WithPort(1883);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NewClientMinimal, s_TestMqtt5NewClientMin)

/*
 * [New-UC2] Maximum creation and cleanup
 */
static int s_TestMqtt5NewClientFull(Aws::Crt::Allocator *allocator, void *)
{
    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    // Hardcoded the host name and port for creation test
    mqtt5Options.WithHostName("localhost").WithPort(1883);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    // Setup will
    const Aws::Crt::String TEST_TOPIC = "test/MQTT5_Binding_CPP/s_TestMqtt5NewClientFull";
    ByteBuf will_payload = Aws::Crt::ByteBufFromCString("Will Test");
    std::shared_ptr<Mqtt5::PublishPacket> will = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(will_payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithClientId("s_TestMqtt5DirectConnectionFull" + Aws::Crt::UUID().ToString())
        .WithKeepAliveIntervalSec(1000)
        .WithMaximumPacketSizeBytes(1000L)
        .WithReceiveMaximum(1000)
        .WithRequestProblemInformation(true)
        .WithRequestResponseInformation(true)
        .WithSessionExpiryIntervalSec(1000L)
        .WithWill(will)
        .WithWillDelayIntervalSec(1000);

    Aws::Crt::Mqtt5::UserProperty userProperty("PropertyName", "PropertyValue");
    packetConnect->WithUserProperty(std::move(userProperty));

    Aws::Crt::Mqtt5::ReconnectOptions reconnectOptions = {
        Mqtt5::JitterMode::AWS_EXPONENTIAL_BACKOFF_JITTER_FULL, 1000, 1000, 1000};

    mqtt5Options.WithConnectOptions(packetConnect);
    mqtt5Options.WithBootstrap(&clientBootstrap);
    mqtt5Options.WithSocketOptions(socketOptions);
    mqtt5Options.WithSessionBehavior(Mqtt5::ClientSessionBehaviorType::AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS);
    mqtt5Options.WithClientExtendedValidationAndFlowControl(
        Mqtt5::ClientExtendedValidationAndFlowControl::AWS_MQTT5_EVAFCO_NONE);
    mqtt5Options.WithOfflineQueueBehavior(
        Mqtt5::ClientOperationQueueBehaviorType::AWS_MQTT5_COQBT_FAIL_QOS0_PUBLISH_ON_DISCONNECT);
    mqtt5Options.WithReconnectOptions(reconnectOptions);
    mqtt5Options.WithPingTimeoutMs(10000);
    mqtt5Options.WithConnackTimeoutMs(10000);
    mqtt5Options.WithAckTimeoutSec(60000);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NewClientFull, s_TestMqtt5NewClientFull)

//////////////////////////////////////////////////////////
// Tests that run only without byo-crypto
//////////////////////////////////////////////////////////

#if !BYO_CRYPTO

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_direct_hostName, "AWS_TEST_MQTT5_DIRECT_MQTT_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_direct_port, "AWS_TEST_MQTT5_DIRECT_MQTT_PORT");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_direct_basic_hostName,
    "AWS_TEST_MQTT5_DIRECT_MQTT_BASIC_AUTH_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_direct_basic_port, "AWS_TEST_MQTT5_DIRECT_MQTT_BASIC_AUTH_PORT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_direct_tls_hostName, "AWS_TEST_MQTT5_DIRECT_MQTT_TLS_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_direct_tls_port, "AWS_TEST_MQTT5_DIRECT_MQTT_TLS_PORT");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_websocket_hostName, "AWS_TEST_MQTT5_WS_MQTT_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_websocket_port, "AWS_TEST_MQTT5_WS_MQTT_PORT");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_websocket_basic_auth_hostName,
    "AWS_TEST_MQTT5_WS_MQTT_BASIC_AUTH_HOST");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_websocket_basic_auth_port,
    "AWS_TEST_MQTT5_WS_MQTT_BASIC_AUTH_PORT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_websocket_tls_hostName, "AWS_TEST_MQTT5_WS_MQTT_TLS_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_websocket_tls_port, "AWS_TEST_MQTT5_WS_MQTT_TLS_PORT");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_basic_username, "AWS_TEST_MQTT5_BASIC_AUTH_USERNAME");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_basic_password, "AWS_TEST_MQTT5_BASIC_AUTH_PASSWORD");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_proxy_hostname, "AWS_TEST_MQTT5_PROXY_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_proxy_port, "AWS_TEST_MQTT5_PROXY_PORT");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_certificate, "AWS_TEST_MQTT5_CERTIFICATE_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_private_key, "AWS_TEST_MQTT5_KEY_FILE");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_hostname, "AWS_TEST_MQTT5_IOT_CORE_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_certificate, "AWS_TEST_MQTT5_IOT_CORE_RSA_CERT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_key, "AWS_TEST_MQTT5_IOT_CORE_RSA_KEY");

enum Mqtt5TestType
{
    MQTT5CONNECT_DIRECT,
    MQTT5CONNECT_DIRECT_BASIC_AUTH,
    MQTT5CONNECT_DIRECT_TLS,
    MQTT5CONNECT_WS,
    MQTT5CONNECT_WS_BASIC_AUTH,
    MQTT5CONNECT_WS_TLS,
    MQTT5CONNECT_IOT_CORE
};

struct Mqtt5TestEnvVars
{
    Mqtt5TestEnvVars(struct aws_allocator *allocator, Mqtt5TestType type)
        : m_error(AWS_OP_SUCCESS), m_allocator(allocator)
    {
        switch (type)
        {
            case MQTT5CONNECT_DIRECT:
            {
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_direct_hostName, &m_hostname);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_direct_port, &m_port);
                if (m_error != AWS_OP_SUCCESS)
                {
                    return;
                }
                if (m_hostname == NULL || m_port == NULL)
                {
                    m_error = AWS_OP_ERR;
                    return;
                }
                m_hostname_string = aws_string_c_str(m_hostname);
                m_port_value = static_cast<uint32_t>(atoi(aws_string_c_str(m_port)));
                break;
            }
            case MQTT5CONNECT_DIRECT_BASIC_AUTH:
            {
                m_error |=
                    aws_get_environment_value(allocator, s_mqtt5_test_envName_direct_basic_hostName, &m_hostname);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_direct_basic_port, &m_port);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_basic_username, &m_username);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_basic_password, &m_password);

                if (m_error != AWS_OP_SUCCESS)
                {
                    return;
                }
                if (m_hostname == NULL || m_port == NULL || m_username == NULL || m_password == NULL)
                {
                    m_error = AWS_OP_ERR;
                    return;
                }
                m_hostname_string = aws_string_c_str(m_hostname);
                m_port_value = static_cast<uint32_t>(atoi(aws_string_c_str(m_port)));
                m_username_string = aws_string_c_str(m_username);
                m_password_cursor = ByteCursorFromArray(m_password->bytes, m_password->len);
                break;
            }
            case MQTT5CONNECT_DIRECT_TLS:
            {
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_direct_tls_hostName, &m_hostname);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_direct_tls_port, &m_port);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_certificate, &m_certificate_path);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_private_key, &m_private_key_path);

                if (m_error != AWS_OP_SUCCESS)
                {
                    return;
                }
                if (m_hostname == NULL || m_port == NULL || m_certificate_path == NULL || m_private_key_path == NULL)
                {
                    m_error = AWS_OP_ERR;
                    return;
                }
                m_hostname_string = aws_string_c_str(m_hostname);
                m_port_value = static_cast<uint32_t>(atoi(aws_string_c_str(m_port)));
                m_certificate_path_string = aws_string_c_str(m_certificate_path);
                m_private_key_path_string = aws_string_c_str(m_private_key_path);
                break;
            }
            case MQTT5CONNECT_WS:
            {
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_websocket_hostName, &m_hostname);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_websocket_port, &m_port);

                if (m_error != AWS_OP_SUCCESS)
                {
                    return;
                }
                if (m_hostname == NULL || m_port == NULL)
                {
                    m_error = AWS_OP_ERR;
                    return;
                }
                {
                    m_hostname_string = aws_string_c_str(m_hostname);
                    m_port_value = static_cast<uint32_t>(atoi(aws_string_c_str(m_port)));
                }
                break;
            }
            case MQTT5CONNECT_WS_BASIC_AUTH:
            {
                m_error |= aws_get_environment_value(
                    allocator, s_mqtt5_test_envName_websocket_basic_auth_hostName, &m_hostname);
                m_error |=
                    aws_get_environment_value(allocator, s_mqtt5_test_envName_websocket_basic_auth_port, &m_port);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_basic_username, &m_username);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_basic_password, &m_password);

                if (m_error != AWS_OP_SUCCESS)
                {
                    return;
                }
                if (m_hostname == NULL || m_port == NULL || m_username == NULL || m_password == NULL)
                {
                    m_error = AWS_OP_ERR;
                    return;
                }
                {
                    m_hostname_string = aws_string_c_str(m_hostname);
                    m_port_value = static_cast<uint32_t>(atoi(aws_string_c_str(m_port)));
                    m_username_string = aws_string_c_str(m_username);
                    m_password_cursor = ByteCursorFromArray(m_password->bytes, m_password->len);
                }
                break;
            }
            case MQTT5CONNECT_WS_TLS:
            {
                m_error |=
                    aws_get_environment_value(allocator, s_mqtt5_test_envName_websocket_tls_hostName, &m_hostname);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_websocket_tls_port, &m_port);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_certificate, &m_certificate_path);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_private_key, &m_private_key_path);

                if (m_error != AWS_OP_SUCCESS)
                {
                    return;
                }
                if (m_hostname == NULL || m_port == NULL || m_certificate_path == NULL || m_private_key_path == NULL)
                {
                    m_error = AWS_OP_ERR;
                    return;
                }
                {
                    m_hostname_string = aws_string_c_str(m_hostname);
                    m_port_value = static_cast<uint32_t>(atoi(aws_string_c_str(m_port)));
                    m_certificate_path_string = aws_string_c_str(m_certificate_path);
                    m_private_key_path_string = aws_string_c_str(m_private_key_path);
                }
                break;
            }
            case MQTT5CONNECT_IOT_CORE:
            {
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_iot_hostname, &m_hostname);
                m_error |=
                    aws_get_environment_value(allocator, s_mqtt5_test_envName_iot_certificate, &m_certificate_path);
                m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_iot_key, &m_private_key_path);

                if (m_error != AWS_OP_SUCCESS)
                {
                    return;
                }
                if (m_hostname == NULL || m_certificate_path == NULL || m_private_key_path == NULL)
                {
                    m_error = AWS_OP_ERR;
                    return;
                }
                {
                    m_hostname_string = aws_string_c_str(m_hostname);
                    m_certificate_path_string = aws_string_c_str(m_certificate_path);
                    m_private_key_path_string = aws_string_c_str(m_private_key_path);
                }
                break;
            }

            default:
                m_error = AWS_OP_ERR;
                break;
        }

        // Setup Http Proxy
        m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_proxy_hostname, &m_httpproxy_hostname);
        m_error |= aws_get_environment_value(allocator, s_mqtt5_test_envName_proxy_port, &m_httpproxy_port);

        if (m_error == AWS_OP_SUCCESS && m_httpproxy_hostname != NULL && m_httpproxy_port != NULL)
        {
            m_httpproxy_hostname_string = aws_string_c_str(m_httpproxy_hostname);
            m_httpproxy_port_value = static_cast<uint32_t>(atoi(aws_string_c_str(m_httpproxy_port)));
        }
    }

    operator bool() const { return m_error == AWS_OP_SUCCESS; }

    ~Mqtt5TestEnvVars()
    {
        if (m_hostname != NULL)
        {
            aws_string_destroy(m_hostname);
            m_hostname = NULL;
        }

        if (m_port != NULL)
        {
            aws_string_destroy(m_port);
            m_port = NULL;
        }

        if (m_username != NULL)
        {
            aws_string_destroy(m_username);
            m_username = NULL;
        }

        if (m_password != NULL)
        {
            aws_string_destroy(m_password);
            m_password = NULL;
        }

        if (m_certificate_path != NULL)
        {
            aws_string_destroy(m_certificate_path);
            m_certificate_path = NULL;
        }

        if (m_private_key_path != NULL)
        {
            aws_string_destroy(m_private_key_path);
            m_private_key_path = NULL;
        }

        if (m_httpproxy_hostname != NULL)
        {
            aws_string_destroy(m_httpproxy_hostname);
            m_httpproxy_hostname = NULL;
        }

        if (m_httpproxy_port != NULL)
        {
            aws_string_destroy(m_httpproxy_port);
            m_httpproxy_port = NULL;
        }
    }

    int m_error;
    struct aws_allocator *m_allocator;

    struct aws_string *m_hostname = NULL;
    struct aws_string *m_port = NULL;
    struct aws_string *m_username = NULL;
    struct aws_string *m_password = NULL;
    struct aws_string *m_certificate_path = NULL;
    struct aws_string *m_private_key_path = NULL;
    struct aws_string *m_httpproxy_hostname = NULL;
    struct aws_string *m_httpproxy_port = NULL;

    Aws::Crt::String m_hostname_string;
    uint32_t m_port_value;
    Aws::Crt::String m_username_string;
    Aws::Crt::ByteCursor m_password_cursor;
    Aws::Crt::String m_certificate_path_string;
    Aws::Crt::String m_private_key_path_string;
    Aws::Crt::String m_httpproxy_hostname_string;
    uint32_t m_httpproxy_port_value;
};

//////////////////////////////////////////////////////////
// Test Helper
//////////////////////////////////////////////////////////
static void s_setupConnectionLifeCycle(
    Aws::Iot::Mqtt5ClientBuilder *mqtt5Builder,
    std::promise<bool> &connectionPromise,
    std::promise<void> &stoppedPromise,
    const char *clientName = "Client")
{
    mqtt5Builder->WithClientConnectionSuccessCallback(
        [&connectionPromise, clientName](const OnConnectionSuccessEventData &)
        {
            printf("[MQTT5]%s Connection Success.", clientName);
            connectionPromise.set_value(true);
        });

    mqtt5Builder->WithClientConnectionFailureCallback(
        [&connectionPromise, clientName](const OnConnectionFailureEventData &eventData)
        {
            printf("[MQTT5]%s Connection failed with error : %s", clientName, aws_error_debug_str(eventData.errorCode));
            connectionPromise.set_value(false);
        });

    mqtt5Builder->WithClientStoppedCallback(
        [&stoppedPromise, clientName](const OnStoppedEventData &)
        {
            printf("[MQTT5]%s Stopped", clientName);
            stoppedPromise.set_value();
        });
}

//////////////////////////////////////////////////////////
// Direct Connect Test Cases [ConnDC-UC]
//////////////////////////////////////////////////////////

/*
 * [ConnDC-UC1] Happy path. Direct connection with minimal configuration
 */
static int s_TestMqtt5DirectConnectionMinimal(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5DirectConnectionMinimal, s_TestMqtt5DirectConnectionMinimal)

/*
 * [ConnDC-UC2] Direct connection with basic authentication
 */
static int s_TestMqtt5DirectConnectionWithBasicAuth(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT_BASIC_AUTH);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithUserName(mqtt5TestVars.m_username_string);
    packetConnect->WithPassword(mqtt5TestVars.m_password_cursor);
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5DirectConnectionWithBasicAuth, s_TestMqtt5DirectConnectionWithBasicAuth)

/*
 * [ConnDC-UC3] Direct connection with TLS
 */
static int s_TestMqtt5DirectConnectionWithTLS(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT_TLS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();

    ASSERT_TRUE(tlsCtxOptions);
    tlsCtxOptions.SetVerifyPeer(false);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();

    ASSERT_TRUE(tlsConnection);
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    connectionPromise.get_future().get();
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5DirectConnectionWithTLS, s_TestMqtt5DirectConnectionWithTLS)

/*
 * [ConnDC-UC4] Direct connection with mutual TLS
 */
static int s_TestMqtt5DirectConnectionWithMutualTLS(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        mqtt5TestVars.m_certificate_path_string.c_str(), mqtt5TestVars.m_private_key_path_string.c_str(), allocator);

    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();
    ASSERT_TRUE(tlsConnection);
    ASSERT_TRUE(tlsConnection.SetAlpnList("x-amzn-mqtt-ca"));
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5DirectConnectionWithMutualTLS, s_TestMqtt5DirectConnectionWithMutualTLS)

/*
 * [ConnDC-UC5] Direct connection with HttpProxy options
 */
static int s_TestMqtt5DirectConnectionWithHttpProxy(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT_TLS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    // HTTP PROXY
    if (mqtt5TestVars.m_httpproxy_hostname->len == 0)
    {
        printf("HTTP PROXY Environment Variables are not set for the test, skip the test");
        return AWS_OP_SUCCESS;
    }
    Aws::Crt::Http::HttpClientConnectionProxyOptions proxyOptions;
    proxyOptions.HostName = mqtt5TestVars.m_httpproxy_hostname_string;
    proxyOptions.Port = mqtt5TestVars.m_httpproxy_port_value;
    proxyOptions.ProxyConnectionType = Aws::Crt::Http::AwsHttpProxyConnectionType::Tunneling;
    mqtt5Options.WithHttpProxyOptions(proxyOptions);

    // TLS
    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    tlsCtxOptions.SetVerifyPeer(false);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();
    ASSERT_TRUE(tlsConnection);
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5DirectConnectionWithHttpProxy, s_TestMqtt5DirectConnectionWithHttpProxy)

/*
 * [ConnDC-UC6] Direct connection with all options set
 */
static int s_TestMqtt5DirectConnectionFull(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    // Setup will
    const Aws::Crt::String TEST_TOPIC =
        "test/MQTT5_Binding_CPP/s_TestMqtt5DirectConnectionFull" + Aws::Crt::UUID().ToString();
    ByteBuf will_payload = Aws::Crt::ByteBufFromCString("Will Test");
    std::shared_ptr<Mqtt5::PublishPacket> will = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(will_payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithClientId("s_TestMqtt5DirectConnectionFull" + Aws::Crt::UUID().ToString())
        .WithKeepAliveIntervalSec(1000)
        .WithMaximumPacketSizeBytes(1000L)
        .WithReceiveMaximum(1000)
        .WithRequestProblemInformation(true)
        .WithRequestResponseInformation(true)
        .WithSessionExpiryIntervalSec(1000L)
        .WithWill(will)
        .WithWillDelayIntervalSec(1000);

    Aws::Crt::Mqtt5::UserProperty userProperty("PropertyName", "PropertyValue");
    packetConnect->WithUserProperty(std::move(userProperty));

    Aws::Crt::Mqtt5::ReconnectOptions reconnectOptions = {
        Mqtt5::JitterMode::AWS_EXPONENTIAL_BACKOFF_JITTER_FULL, 1000, 1000, 1000};

    mqtt5Options.WithConnectOptions(packetConnect);
    mqtt5Options.WithBootstrap(&clientBootstrap);
    mqtt5Options.WithSocketOptions(socketOptions);
    mqtt5Options.WithSessionBehavior(Mqtt5::ClientSessionBehaviorType::AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS);
    mqtt5Options.WithClientExtendedValidationAndFlowControl(
        Mqtt5::ClientExtendedValidationAndFlowControl::AWS_MQTT5_EVAFCO_NONE);
    mqtt5Options.WithOfflineQueueBehavior(
        Mqtt5::ClientOperationQueueBehaviorType::AWS_MQTT5_COQBT_FAIL_QOS0_PUBLISH_ON_DISCONNECT);
    mqtt5Options.WithReconnectOptions(reconnectOptions);
    mqtt5Options.WithPingTimeoutMs(10000);
    mqtt5Options.WithConnackTimeoutMs(10000);
    mqtt5Options.WithAckTimeoutSec(60000);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5DirectConnectionFull, s_TestMqtt5DirectConnectionFull)

//////////////////////////////////////////////////////////
// Websocket Connect Test Cases [ConnWS-UC]
//////////////////////////////////////////////////////////

/*
 * [ConnWS-UC1] Happy path. Websocket connection with minimal configuration.
 */
static int s_TestMqtt5WSConnectionMinimal(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WSConnectionMinimal, s_TestMqtt5WSConnectionMinimal)

/*
 * [ConnWS-UC2] websocket connection with basic authentication
 */
static int s_TestMqtt5WSConnectionWithBasicAuth(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS_BASIC_AUTH);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithUserName(mqtt5TestVars.m_username_string);
    packetConnect->WithPassword(mqtt5TestVars.m_password_cursor);
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WSConnectionWithBasicAuth, s_TestMqtt5WSConnectionWithBasicAuth)

/*
 * [ConnWS-UC3] websocket connection with TLS
 */
static int s_TestMqtt5WSConnectionWithTLS(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS_TLS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();

    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();

    ASSERT_TRUE(tlsConnection);
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    // setup websocket config
    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    connectionPromise.get_future().get();
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WSConnectionWithTLS, s_TestMqtt5WSConnectionWithTLS)

/*
 * [ConnDC-UC4] Websocket connection with mutual TLS
 */
static int s_TestMqtt5WSConnectionWithMutualTLS(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        mqtt5TestVars.m_certificate_path_string.c_str(), mqtt5TestVars.m_private_key_path_string.c_str(), allocator);

    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();

    ASSERT_TRUE(tlsConnection);
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    // setup websocket config
    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    connectionPromise.get_future().get();
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WSConnectionWithMutualTLS, s_TestMqtt5WSConnectionWithMutualTLS)

/*
 * ConnWS-UC5] Websocket connection with HttpProxy options
 */
static int s_TestMqtt5WSConnectionWithHttpProxy(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS_TLS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443);

    // HTTP PROXY
    if (mqtt5TestVars.m_httpproxy_hostname->len == 0)
    {
        printf("HTTP PROXY Environment Variables are not set for the test, skip the test");
        return AWS_OP_SUCCESS;
    }
    Aws::Crt::Http::HttpClientConnectionProxyOptions proxyOptions;
    proxyOptions.HostName = mqtt5TestVars.m_httpproxy_hostname_string;
    proxyOptions.Port = mqtt5TestVars.m_httpproxy_port_value;
    proxyOptions.AuthType = Aws::Crt::Http::AwsHttpProxyAuthenticationType::None;
    proxyOptions.ProxyConnectionType = Aws::Crt::Http::AwsHttpProxyConnectionType::Tunneling;
    mqtt5Options.WithHttpProxyOptions(proxyOptions);

    // TLS
    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    tlsCtxOptions.SetVerifyPeer(false);
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();
    ASSERT_TRUE(tlsConnection);
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    // setup websocket config
    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);
    config.ProxyOptions = proxyOptions;

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
        });

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WSConnectionWithHttpProxy, s_TestMqtt5WSConnectionWithHttpProxy)

/*
 * [ConnDC-UC6] Direct connection with all options set
 */
static int s_TestMqtt5WSConnectionFull(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    // Setup will
    const Aws::Crt::String TEST_TOPIC =
        "test/MQTT5_Binding_CPP/s_TestMqtt5WSConnectionFull" + Aws::Crt::UUID().ToString();
    ByteBuf will_payload = Aws::Crt::ByteBufFromCString("Will Test");
    std::shared_ptr<Mqtt5::PublishPacket> will = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(will_payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithClientId("s_TestMqtt5WSConnectionFull" + Aws::Crt::UUID().ToString())
        .WithKeepAliveIntervalSec(1000)
        .WithMaximumPacketSizeBytes(1000L)
        .WithReceiveMaximum(1000)
        .WithRequestProblemInformation(true)
        .WithRequestResponseInformation(true)
        .WithSessionExpiryIntervalSec(1000L)
        .WithWill(will)
        .WithWillDelayIntervalSec(1000);

    Aws::Crt::Mqtt5::UserProperty userProperty("PropertyName", "PropertyValue");
    packetConnect->WithUserProperty(std::move(userProperty));

    Aws::Crt::Mqtt5::ReconnectOptions reconnectOptions = {
        Mqtt5::JitterMode::AWS_EXPONENTIAL_BACKOFF_JITTER_FULL, 1000, 1000, 1000};

    mqtt5Options.WithConnectOptions(packetConnect);
    mqtt5Options.WithBootstrap(&clientBootstrap);
    mqtt5Options.WithSocketOptions(socketOptions);
    mqtt5Options.WithSessionBehavior(Mqtt5::ClientSessionBehaviorType::AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS);
    mqtt5Options.WithClientExtendedValidationAndFlowControl(
        Mqtt5::ClientExtendedValidationAndFlowControl::AWS_MQTT5_EVAFCO_NONE);
    mqtt5Options.WithOfflineQueueBehavior(
        Mqtt5::ClientOperationQueueBehaviorType::AWS_MQTT5_COQBT_FAIL_QOS0_PUBLISH_ON_DISCONNECT);
    mqtt5Options.WithReconnectOptions(reconnectOptions);
    mqtt5Options.WithPingTimeoutMs(10000);
    mqtt5Options.WithConnackTimeoutMs(10000);
    mqtt5Options.WithAckTimeoutSec(60000);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    // setup websocket config
    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WSConnectionFull, s_TestMqtt5WSConnectionFull)

////////////////////////////////////////////////////////////
// Negative Connect Tests with Incorrect Data [ConnNegativeID-UC]
////////////////////////////////////////////////////////////

/*
 * [ConnNegativeID-UC1] Client connect with invalid host name
 */
static int s_TestMqtt5DirectInvalidHostname(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName("invalid");
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_FALSE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5InvalidHostname, s_TestMqtt5DirectInvalidHostname)

/*
 * [ConnNegativeID-UC2] Client connect with invalid port for direct connection
 */
static int s_TestMqtt5DirectInvalidPort(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443); // 443 is for mutual TLS, not for direct connect

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_FALSE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5InvalidPort, s_TestMqtt5DirectInvalidPort)

/*
 * [ConnNegativeID-UC3] Client connect with invalid port for websocket connection
 */
static int s_TestMqtt5WSInvalidPort(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_FALSE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WSInvalidPort, s_TestMqtt5WSInvalidPort)

/*
 * [ConnNegativeID-UC4] Client connect with socket timeout
 */
static int s_TestMqtt5SocketTimeout(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName("www.example.com");
    mqtt5Options.WithPort(81);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(1000);
    mqtt5Options.WithSocketOptions(socketOptions);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    // Override connection failed callback
    mqtt5Options.WithClientConnectionFailureCallback(
        [&connectionPromise](const OnConnectionFailureEventData &eventData)
        {
            printf("[MQTT5]Client Connection failed with error : %s", aws_error_debug_str(eventData.errorCode));
            ASSERT_TRUE(eventData.errorCode == AWS_IO_SOCKET_TIMEOUT);
            connectionPromise.set_value(false);
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_FALSE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5SocketTimeout, s_TestMqtt5SocketTimeout)

/*
 * [ConnNegativeID-UC5] Client connect with incorrect basic authentication credentials
 */
static int s_TestMqtt5IncorrectBasicAuth(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT_BASIC_AUTH);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithUserName("WRONG_USERNAME");
    packetConnect->WithPassword(ByteCursorFromCString("WRONG_PASSWORD"));
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_FALSE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5IncorrectBasicAuth, s_TestMqtt5IncorrectBasicAuth)

// [ConnNegativeID-UC6] Client Websocket Handshake Failure test
static int s_TestMqtt5IncorrectWSConnect(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        { onComplete(req, AWS_ERROR_UNSUPPORTED_OPERATION); });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_FALSE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5IncorrectWSConnect, s_TestMqtt5IncorrectWSConnect)

/*
 * [ConnNegativeID-UC7] Double Client ID Failure test
 */
static int s_TestMqtt5DoubleClientIDFailure(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithClientId("TestMqtt5DoubleClientIDFailure" + Aws::Crt::UUID().ToString());
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connection1Promise;
    std::promise<void> stopped1Promise;
    std::promise<bool> connection2Promise;
    std::promise<void> stopped2Promise;
    std::promise<void> disconnectionPromise;

    // SETUP CLIENT 1 CALLBACKS
    s_setupConnectionLifeCycle(mqtt5Options, connection1Promise, stopped1Promise, "Client1");
    mqtt5Options.WithClientDisconnectionCallback(
        [&disconnectionPromise](const OnDisconnectionEventData &eventData)
        {
            if (eventData.errorCode != 0)
            {
                printf("[MQTT5]Client1 disconnected with error : %s", aws_error_debug_str(eventData.errorCode));
                disconnectionPromise.set_value();
            }
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client1 = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client1);

    s_setupConnectionLifeCycle(mqtt5Options, connection2Promise, stopped2Promise, "Client2");

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client2 = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client2);

    ASSERT_TRUE(mqtt5Client1->Start());
    // Client 1 is connected.
    ASSERT_TRUE(connection1Promise.get_future().get());
    ASSERT_TRUE(mqtt5Client2->Start());

    // Make sure the client2 is connected.
    ASSERT_TRUE(connection2Promise.get_future().get());

    // Client 1 should get diconnected.
    disconnectionPromise.get_future().get();
    // reset the promise so it would not get confused when we stop the client;
    disconnectionPromise = std::promise<void>();

    ASSERT_TRUE(mqtt5Client2->Stop());
    stopped2Promise.get_future().get();
    ASSERT_TRUE(mqtt5Client1->Stop());
    stopped1Promise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5DoubleClientIDFailure, s_TestMqtt5DoubleClientIDFailure)

//////////////////////////////////////////////////////////
// Negative Data Input Tests [NewNegativePK-UC] - Ignored, not applied to C++
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
// Negotiated Settings Tests [Negotiated-UC]
//////////////////////////////////////////////////////////

/*
 * [Negotiated-UC1] Happy path, minimal success test
 */
static int s_TestMqtt5NegotiatedSettingsHappy(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    const uint32_t SESSION_EXPIRY_INTERVAL_SEC = 600;

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithSessionExpiryIntervalSec(SESSION_EXPIRY_INTERVAL_SEC);
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    // Override the ConnectionSuccessCallback to validate the negotiatedSettings
    mqtt5Options.WithClientConnectionSuccessCallback(
        [&](const OnConnectionSuccessEventData &eventData)
        {
            printf("[MQTT5]Client Connection Success.");
            ASSERT_TRUE(eventData.negotiatedSettings->getSessionExpiryIntervalSec() == SESSION_EXPIRY_INTERVAL_SEC);
            connectionPromise.set_value(true);
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NegotiatedSettingsHappy, s_TestMqtt5NegotiatedSettingsHappy)

/*
 * [Negotiated-UC2] maximum success test
 */
static int s_TestMqtt5NegotiatedSettingsFull(Aws::Crt::Allocator *allocator, void *)
{
    const String CLIENT_ID = "s_TestMqtt5NegotiatedSettingsFull" + Aws::Crt::UUID().ToString();
    const uint32_t SESSION_EXPIRY_INTERVAL_SEC = 600;
    const uint16_t RECEIVE_MAX = 12;
    const uint16_t KEEP_ALIVE_INTERVAL = 1000;

    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator, allocator);
    packetConnect->WithSessionExpiryIntervalSec(SESSION_EXPIRY_INTERVAL_SEC)
        .WithClientId(CLIENT_ID)
        .WithReceiveMaximum(RECEIVE_MAX)
        .WithMaximumPacketSizeBytes(UINT32_MAX)
        .WithKeepAliveIntervalSec(KEEP_ALIVE_INTERVAL);
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    // Override the ConnectionSuccessCallback to validate the negotiatedSettings
    mqtt5Options.WithClientConnectionSuccessCallback(
        [&](const OnConnectionSuccessEventData &eventData)
        {
            printf("[MQTT5]Client Connection Success.");
            std::shared_ptr<NegotiatedSettings> settings = eventData.negotiatedSettings;
            ASSERT_TRUE(settings->getSessionExpiryIntervalSec() == SESSION_EXPIRY_INTERVAL_SEC);
            ASSERT_TRUE(settings->getClientId() == CLIENT_ID);
            ASSERT_TRUE(settings->getServerKeepAliveSec() == KEEP_ALIVE_INTERVAL);
            connectionPromise.set_value(true);
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NegotiatedSettingsFull, s_TestMqtt5NegotiatedSettingsFull)

/*
 * [Negotiated-UC3] Server Settings limit test
 */
static int s_TestMqtt5NegotiatedSettingsLimit(Aws::Crt::Allocator *allocator, void *)
{
    const uint32_t SESSION_EXPIRY_INTERVAL_SEC = UINT32_MAX;
    const uint16_t RECEIVE_MAX = UINT16_MAX;
    const uint16_t KEEP_ALIVE_INTERVAL = UINT16_MAX;
    const uint32_t PACKET_MAX = UINT32_MAX;

    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithSessionExpiryIntervalSec(SESSION_EXPIRY_INTERVAL_SEC)
        .WithReceiveMaximum(RECEIVE_MAX)
        .WithMaximumPacketSizeBytes(PACKET_MAX)
        .WithKeepAliveIntervalSec(KEEP_ALIVE_INTERVAL);
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    mqtt5Options.WithClientConnectionSuccessCallback(
        [&](const OnConnectionSuccessEventData &eventData)
        {
            std::shared_ptr<NegotiatedSettings> settings = eventData.negotiatedSettings;
            uint16_t receivedmax = settings->getReceiveMaximumFromServer();
            uint32_t max_package = settings->getMaximumPacketSizeToServer();
            ASSERT_FALSE(receivedmax == RECEIVE_MAX);
            ASSERT_FALSE(max_package == PACKET_MAX);
            ASSERT_FALSE(settings->getRejoinedSession());

            connectionPromise.set_value(true);
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NegotiatedSettingsLimit, s_TestMqtt5NegotiatedSettingsLimit)

/*
 * [Negotiated-UC4] Rejoin Always Session Behavior
 */
static int s_TestMqtt5NegotiatedSettingsRejoinAlways(Aws::Crt::Allocator *allocator, void *)
{
    static const uint32_t SESSION_EXPIRY_INTERVAL_SEC = 3600;

    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithSessionExpiryIntervalSec(SESSION_EXPIRY_INTERVAL_SEC);
    packetConnect->WithClientId(Aws::Crt::UUID().ToString());
    mqtt5Options.WithConnectOptions(packetConnect);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    mqtt5Options.WithClientConnectionSuccessCallback(
        [&connectionPromise](const OnConnectionSuccessEventData &eventData)
        {
            std::shared_ptr<NegotiatedSettings> settings = eventData.negotiatedSettings;
            ASSERT_FALSE(settings->getRejoinedSession());
            connectionPromise.set_value(true);
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    mqtt5Options.WithSessionBehavior(Aws::Crt::Mqtt5::ClientSessionBehaviorType::AWS_MQTT5_CSBT_REJOIN_ALWAYS);

    std::promise<bool> sessionConnectedPromise;
    std::promise<void> sessionStoppedPromise;
    s_setupConnectionLifeCycle(mqtt5Options, sessionConnectedPromise, sessionStoppedPromise);

    mqtt5Options.WithClientConnectionSuccessCallback(
        [&sessionConnectedPromise](const OnConnectionSuccessEventData &eventData)
        {
            std::shared_ptr<NegotiatedSettings> settings = eventData.negotiatedSettings;
            ASSERT_TRUE(settings->getRejoinedSession());
            sessionConnectedPromise.set_value(true);
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> sessionMqtt5Client =
        Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(sessionMqtt5Client);

    ASSERT_TRUE(sessionMqtt5Client->Start());
    ASSERT_TRUE(sessionConnectedPromise.get_future().get());
    ASSERT_TRUE(sessionMqtt5Client->Stop());
    sessionStoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NegotiatedSettingsRejoinAlways, s_TestMqtt5NegotiatedSettingsRejoinAlways)

//////////////////////////////////////////////////////////
// Operation Tests [Op-UC]
//////////////////////////////////////////////////////////

/*
 * [Op-UC1] Sub-Unsub happy path
 */
static int s_TestMqtt5SubUnsub(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    const String TEST_TOPIC = "test/MQTT5_Binding_CPP" + Aws::Crt::UUID().ToString();

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    int receivedCount = 0;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    mqtt5Options.WithPublishReceivedCallback(
        [&receivedCount, TEST_TOPIC](const PublishReceivedEventData &eventData)
        {
            String topic = eventData.publishPacket->getTopic();
            if (topic == TEST_TOPIC)
            {
                receivedCount++;
            }
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    /* Subscribe to test topic */
    Mqtt5::Subscription subscription(TEST_TOPIC, Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    subscription.WithNoLocal(false);
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe =
        Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator, allocator);
    subscribe->WithSubscription(std::move(subscription));
    ASSERT_TRUE(mqtt5Client->Subscribe(subscribe));

    /* Publish message 1 to test topic */
    ByteBuf payload = Aws::Crt::ByteBufFromCString("Hello World");
    std::shared_ptr<Mqtt5::PublishPacket> publish = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    ASSERT_TRUE(mqtt5Client->Publish(publish));

    // Sleep and wait for message recieved
    aws_thread_current_sleep(2000 * 1000 * 1000);

    /* Unsubscribe to test topic */
    Vector<String> topics;
    topics.push_back(TEST_TOPIC);
    std::shared_ptr<Mqtt5::UnsubscribePacket> unsub =
        Aws::Crt::MakeShared<Mqtt5::UnsubscribePacket>(allocator, allocator);
    unsub->WithTopicFilters(topics);
    ASSERT_TRUE(mqtt5Client->Unsubscribe(unsub));

    /* Publish message2 to test topic */
    ASSERT_TRUE(mqtt5Client->Publish(publish));

    // Sleep and wait for message recieved
    aws_thread_current_sleep(2000 * 1000 * 1000);

    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    ASSERT_TRUE(receivedCount == 1);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5SubUnsub, s_TestMqtt5SubUnsub)

/*
 * [Op-UC2] Will test
 */
static int s_TestMqtt5WillTest(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    bool receivedWill = false;
    const String TEST_TOPIC = "test/MQTT5_Binding_CPP" + Aws::Crt::UUID().ToString();

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> subscriberConnectionPromise;
    std::promise<bool> publisherConnectionPromise;
    std::promise<void> subscriberStoppedPromise;
    std::promise<void> publisherStoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, subscriberConnectionPromise, subscriberStoppedPromise, "Suberscriber");

    mqtt5Options.WithPublishReceivedCallback(
        [&receivedWill, TEST_TOPIC](const PublishReceivedEventData &eventData)
        {
            String topic = eventData.publishPacket->getTopic();
            if (topic == TEST_TOPIC)
            {
                receivedWill = true;
            }
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> subscriber = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(subscriber);

    /* Set will for client option */
    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    ByteBuf will_payload = Aws::Crt::ByteBufFromCString("Will Test");
    std::shared_ptr<Mqtt5::PublishPacket> will = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(will_payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    packetConnect->WithWill(will);
    mqtt5Options.WithConnectOptions(packetConnect);

    s_setupConnectionLifeCycle(mqtt5Options, publisherConnectionPromise, publisherStoppedPromise, "Publisher");

    std::shared_ptr<Mqtt5::Mqtt5Client> publisher = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(publisher);

    ASSERT_TRUE(publisher->Start());
    publisherConnectionPromise.get_future().get();

    ASSERT_TRUE(subscriber->Start());
    subscriberConnectionPromise.get_future().get();

    /* Subscribe to test topic */
    Mqtt5::Subscription subscription(TEST_TOPIC, Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe =
        Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator, allocator);

    subscribe->WithSubscription(std::move(subscription));

    std::promise<void> subscribed;
    ASSERT_TRUE(subscriber->Subscribe(
        subscribe, [&subscribed](int, std::shared_ptr<Mqtt5::SubAckPacket>) { subscribed.set_value(); }));

    subscribed.get_future().get();

    std::shared_ptr<Mqtt5::DisconnectPacket> disconnect =
        Aws::Crt::MakeShared<Mqtt5::DisconnectPacket>(allocator, allocator);
    disconnect->WithReasonCode(AWS_MQTT5_DRC_DISCONNECT_WITH_WILL_MESSAGE);
    ASSERT_TRUE(publisher->Stop(disconnect));
    publisherStoppedPromise.get_future().get();

    aws_thread_current_sleep(10000ULL * 1000 * 1000);
    ASSERT_TRUE(receivedWill);

    ASSERT_TRUE(subscriber->Stop());
    subscriberStoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5WillTest, s_TestMqtt5WillTest)

/*
 *  Shared Subscription test
 */
static int s_TestMqtt5SharedSubscriptionTest(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    String currentUUID = Aws::Crt::UUID().ToString();

    const String TEST_TOPIC = "test/MQTT5_Binding_CPP_" + currentUUID;
    const String sharedTopicFilter = "$share/crttest/test/MQTT5_Binding_CPP_" + currentUUID;

    static const int MESSAGE_COUNT = 10;
    std::mutex receivedMessagesLock;
    std::condition_variable receivedMessagesSignal;
    std::vector<int> receivedMessages;

    Aws::Iot::Mqtt5ClientBuilder *subscribe_builder =
        Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
            mqtt5TestVars.m_hostname_string,
            mqtt5TestVars.m_certificate_path_string.c_str(),
            mqtt5TestVars.m_private_key_path_string.c_str(),
            allocator);
    ASSERT_TRUE(subscribe_builder);

    auto on_message_callback1 = [&](const PublishReceivedEventData &eventData)
    {
        String topic = eventData.publishPacket->getTopic();
        if (topic == TEST_TOPIC)
        {
            ByteCursor payload = eventData.publishPacket->getPayload();
            String message_string = String((const char *)payload.ptr, payload.len);
            int message_int = atoi(message_string.c_str());

            {
                std::lock_guard<std::mutex> guard(receivedMessagesLock);
                receivedMessages.push_back(message_int);

                if (receivedMessages.size() == MESSAGE_COUNT)
                {
                    receivedMessagesSignal.notify_all();
                }
            }
        }
    };
    auto on_message_callback2 = on_message_callback1;

    subscribe_builder->WithPublishReceivedCallback(on_message_callback1);

    Aws::Iot::Mqtt5ClientBuilder *subscribe_builder2 =
        Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
            mqtt5TestVars.m_hostname_string,
            mqtt5TestVars.m_certificate_path_string.c_str(),
            mqtt5TestVars.m_private_key_path_string.c_str(),
            allocator);
    ASSERT_TRUE(subscribe_builder2);
    subscribe_builder2->WithPublishReceivedCallback(on_message_callback2);

    Aws::Iot::Mqtt5ClientBuilder *publish_builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(publish_builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    std::promise<bool> connectionPromise2;
    std::promise<void> stoppedPromise2;

    std::promise<bool> connectionPromise3;
    std::promise<void> stoppedPromise3;

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);

    /* first subscriber */
    /* set a different client id so we are not disconnected from the server */
    packetConnect->WithClientId("s_TestMqtt5SharedSubscriptionTest" + Aws::Crt::UUID().ToString());
    subscribe_builder->WithConnectOptions(packetConnect);
    s_setupConnectionLifeCycle(subscribe_builder, connectionPromise, stoppedPromise, "Subscriber 1");
    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = subscribe_builder->Build();
    ASSERT_TRUE(mqtt5Client);

    /* second subscriber */
    /* set a different client id so we are not disconnected from the server */
    packetConnect->WithClientId("s_TestMqtt5SharedSubscriptionTest" + Aws::Crt::UUID().ToString());
    subscribe_builder2->WithConnectOptions(packetConnect);
    s_setupConnectionLifeCycle(subscribe_builder2, connectionPromise2, stoppedPromise2, "Subscriber 2");
    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client2 = subscribe_builder2->Build();
    ASSERT_TRUE(mqtt5Client2);

    /* publisher */
    /* set a different client id so we are not disconnected from the server */
    packetConnect->WithClientId("s_TestMqtt5SharedSubscriptionTest" + Aws::Crt::UUID().ToString());
    publish_builder->WithConnectOptions(packetConnect);
    s_setupConnectionLifeCycle(publish_builder, connectionPromise3, stoppedPromise3, "Publisher");
    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Publisher = publish_builder->Build();
    ASSERT_TRUE(mqtt5Publisher);

    /* Connect all clients */
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(mqtt5Client2->Start());
    ASSERT_TRUE(mqtt5Publisher->Start());

    /* Wait for all clents to connect */
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(connectionPromise2.get_future().get());
    ASSERT_TRUE(connectionPromise3.get_future().get());

    /* Subscribe to test topic */
    Mqtt5::Subscription subscription(sharedTopicFilter, Mqtt5::QOS::AWS_MQTT5_QOS_AT_MOST_ONCE, allocator);
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe =
        Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator, allocator);
    subscribe->WithSubscription(std::move(subscription));

    /* Subscribe to test topic */
    Mqtt5::Subscription subscription2(sharedTopicFilter, Mqtt5::QOS::AWS_MQTT5_QOS_AT_MOST_ONCE, allocator);
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe2 =
        Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator, allocator);
    subscribe2->WithSubscription(std::move(subscription2));

    std::promise<void> suback;
    auto onSubAck = [&](int, std::shared_ptr<SubAckPacket>) { suback.set_value(); };

    /* subscribe first client */
    ASSERT_TRUE(mqtt5Client->Subscribe(subscribe, onSubAck));
    suback.get_future().wait();

    suback = std::promise<void>();

    /* subscribe second client */
    ASSERT_TRUE(mqtt5Client2->Subscribe(subscribe2, onSubAck));
    suback.get_future().wait();

    /* Publish message 10 to test topic */
    for (int i = 0; i < MESSAGE_COUNT; i++)
    {
        std::string payload = std::to_string(i);
        std::shared_ptr<Mqtt5::PublishPacket> publish = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
            allocator,
            TEST_TOPIC,
            ByteCursorFromCString(payload.c_str()),
            Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE,
            allocator);
        ASSERT_TRUE(mqtt5Publisher->Publish(publish));
    }

    /* Wait for all packets to be received on both clients */
    std::unique_lock<std::mutex> receivedLock(receivedMessagesLock);
    receivedMessagesSignal.wait(receivedLock, [&]() { return receivedMessages.size() == MESSAGE_COUNT; });

    std::sort(receivedMessages.begin(), receivedMessages.end());
    /* make sure all messages are received  with no duplicates*/
    for (int i = 0; i < MESSAGE_COUNT; i++)
    {
        ASSERT_INT_EQUALS(i, receivedMessages[i]);
    }
    /* Stop all clients */
    ASSERT_TRUE(mqtt5Client->Stop());
    ASSERT_TRUE(mqtt5Client2->Stop());
    ASSERT_TRUE(mqtt5Publisher->Stop());

    /* Wait for all clents to disconnect */
    stoppedPromise.get_future().get();
    stoppedPromise2.get_future().get();
    stoppedPromise3.get_future().get();

    delete subscribe_builder;
    delete subscribe_builder2;
    delete publish_builder;

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(Mqtt5SharedSubscriptionTest, s_TestMqtt5SharedSubscriptionTest)

//////////////////////////////////////////////////////////
// Error Operation Tests [ErrorOp-UC]
//////////////////////////////////////////////////////////

/*
 * [ErrorOp-UC1] Null Publish Test
 */
static int s_TestMqtt5NullPublish(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    const String TEST_TOPIC = "test/s_TestMqtt5NullPublish" + Aws::Crt::UUID().ToString();

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    // Invalid publish packet with empty topic
    ByteBuf payload = Aws::Crt::ByteBufFromCString("Mqtt5 Null Publish Test");
    std::shared_ptr<Mqtt5::PublishPacket> publish = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, "", ByteCursorFromByteBuf(payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);

    /* Publish message 1 to test topic */
    ASSERT_FALSE(mqtt5Client->Publish(publish));

    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NullPublish, s_TestMqtt5NullPublish)

/*
 * [ErrorOp-UC2] Null Subscribe Test
 */
static int s_TestMqtt5NullSubscribe(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    const String TEST_TOPIC = "test/s_TestMqtt5NullSubscribe" + Aws::Crt::UUID().ToString();

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    /* Subscribe to empty subscribe packet*/
    Vector<Mqtt5::Subscription> subscriptionList;
    subscriptionList.clear();
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe =
        Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator, allocator);
    subscribe->WithSubscriptions(subscriptionList);
    ASSERT_FALSE(mqtt5Client->Subscribe(subscribe));

    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NullSubscribe, s_TestMqtt5NullSubscribe)

/*
 * [ErrorOp-UC3] Null unsubscribe test
 */
static int s_TestMqtt5NullUnsubscribe(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    const String TEST_TOPIC = "test/s_TestMqtt5NullUnsubscribe" + Aws::Crt::UUID().ToString();

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    /* Subscribe to empty subscribe packet*/
    Vector<String> unsubList;
    unsubList.clear();
    std::shared_ptr<Mqtt5::UnsubscribePacket> unsubscribe =
        Aws::Crt::MakeShared<Mqtt5::UnsubscribePacket>(allocator, allocator);
    unsubscribe->WithTopicFilters(unsubList);
    ASSERT_FALSE(mqtt5Client->Unsubscribe(unsubscribe));

    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5NullUnsubscribe, s_TestMqtt5NullUnsubscribe)

/*
 * Reuse unsubscribe packet test.
 * The scenario in this test once caused memory leak, so the test ensures the issue is fixed for good.
 */
static int s_TestMqtt5ReuseUnsubscribePacket(Aws::Crt::Allocator *allocator, void *)
{
    ApiHandle apiHandle(allocator);

    const String TEST_TOPIC = "test/s_TestMqtt5NullUnsubscribe" + Aws::Crt::UUID().ToString();

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName("www.example.com").WithPort(1111);
    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    Vector<String> unsubList{TEST_TOPIC};
    std::shared_ptr<Mqtt5::UnsubscribePacket> unsubscribe =
        Aws::Crt::MakeShared<Mqtt5::UnsubscribePacket>(allocator, allocator);
    unsubscribe->WithTopicFilters(unsubList);
    ASSERT_TRUE(mqtt5Client->Unsubscribe(unsubscribe));
    /* Unsubscribe once again using the same UnsubscribePacket. */
    ASSERT_TRUE(mqtt5Client->Unsubscribe(unsubscribe));

    ASSERT_TRUE(mqtt5Client->Stop());

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5ReuseUnsubscribePacket, s_TestMqtt5ReuseUnsubscribePacket)

//////////////////////////////////////////////////////////
// QoS1 Test Cases [QoS1-UC]
//////////////////////////////////////////////////////////

/*
 * [QoS1-UC1] Happy path. No drop in connection, no retry, no reconnect.
 */
static int s_TestMqtt5QoS1SubPub(Aws::Crt::Allocator *allocator, void *)
{

    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    const int MESSAGE_NUMBER = 10;
    const String TEST_TOPIC = "test/s_TestMqtt5QoS1SubPub" + Aws::Crt::UUID().ToString();
    std::vector<int> receivedMessages;
    for (int i = 0; i < MESSAGE_NUMBER; i++)
    {
        receivedMessages.push_back(0);
    }

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> subscriberConnectionPromise;
    std::promise<bool> publisherConnectionPromise;
    std::promise<void> subscriberStoppedPromise;
    std::promise<void> publisherStoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, subscriberConnectionPromise, subscriberStoppedPromise, "Subscriber");

    mqtt5Options.WithPublishReceivedCallback(
        [&](const PublishReceivedEventData &eventData)
        {
            String topic = eventData.publishPacket->getTopic();
            if (topic == TEST_TOPIC)
            {
                ByteCursor payload = eventData.publishPacket->getPayload();
                String message_string = String((const char *)payload.ptr, payload.len);
                int message_int = atoi(message_string.c_str());
                ASSERT_TRUE(message_int < MESSAGE_NUMBER);
                ++receivedMessages[message_int];
            }
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> subscriber = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(subscriber);

    s_setupConnectionLifeCycle(mqtt5Options, publisherConnectionPromise, publisherStoppedPromise, "Publisher");

    std::shared_ptr<Mqtt5::Mqtt5Client> publisher = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(publisher);

    ASSERT_TRUE(publisher->Start());
    ASSERT_TRUE(publisherConnectionPromise.get_future().get());

    ASSERT_TRUE(subscriber->Start());
    ASSERT_TRUE(subscriberConnectionPromise.get_future().get());

    /* Subscribe to test topic */
    Mqtt5::Subscription subscription(TEST_TOPIC, Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe = Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator);
    subscribe->WithSubscription(std::move(subscription));

    std::promise<void> subscribed;
    ASSERT_TRUE(subscriber->Subscribe(
        subscribe, [&subscribed](int, std::shared_ptr<Mqtt5::SubAckPacket>) { subscribed.set_value(); }));

    subscribed.get_future().get();

    /* Publish 10 messages to test topic */
    for (int i = 0; i < MESSAGE_NUMBER; i++)
    {
        std::string payload = std::to_string(i);
        std::shared_ptr<Mqtt5::PublishPacket> publish = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
            allocator,
            TEST_TOPIC,
            ByteCursorFromCString(payload.c_str()),
            Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE,
            allocator);
        ASSERT_TRUE(publisher->Publish(publish));
    }

    /* Wait 10s for the messages*/
    aws_thread_current_sleep(10000ULL * 1000 * 1000);

    for (int i = 0; i < MESSAGE_NUMBER; i++)
    {
        ASSERT_TRUE(receivedMessages[i] > 0);
    }

    ASSERT_TRUE(subscriber->Stop());
    subscriberStoppedPromise.get_future().get();
    ASSERT_TRUE(publisher->Stop());
    publisherStoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5QoS1SubPub, s_TestMqtt5QoS1SubPub)

///*
// *  TODO:
// * [QoS1-UC2] Happy path. No drop in connection, no retry, no reconnect.
// * [QoS1-UC3] Similar to QoS1-UC2 setup, but Subscriber will reconnect with clean session
// * [QoS1-UC4] Similar to QoS1-UC2 setup, but the Subscriber stays disconnected longer than the retain message timeout.
// */

/*
 * [Retain-UC1] Set-And-Clear Test
 */
static int s_TestMqtt5RetainSetAndClear(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);
    const Aws::Crt::String TEST_TOPIC = "test/s_TestMqtt5RetainSetAndClear" + Aws::Crt::UUID().ToString();
    const Aws::Crt::String RETAIN_MESSAGE = "This is a retian message";

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connection1Promise;
    std::promise<void> stopped1Promise;
    std::promise<bool> connection2Promise;
    std::promise<void> stopped2Promise;
    std::promise<void> client2RetianMessageReceived;
    std::promise<bool> connection3Promise;
    std::promise<void> stopped3Promise;

    // SETUP CLIENT 1 CALLBACKS
    s_setupConnectionLifeCycle(mqtt5Options, connection1Promise, stopped1Promise, "Client1");

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client1 = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client1);

    s_setupConnectionLifeCycle(mqtt5Options, connection2Promise, stopped2Promise, "Client2");

    mqtt5Options.WithPublishReceivedCallback(
        [&client2RetianMessageReceived, TEST_TOPIC](const PublishReceivedEventData &eventData)
        {
            String topic = eventData.publishPacket->getTopic();
            if (topic == TEST_TOPIC)
            {
                client2RetianMessageReceived.set_value();
            }
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client2 = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client2);

    s_setupConnectionLifeCycle(mqtt5Options, connection3Promise, stopped3Promise, "Client3");

    mqtt5Options.WithPublishReceivedCallback(
        [TEST_TOPIC](const PublishReceivedEventData &eventData)
        {
            String topic = eventData.publishPacket->getTopic();
            if (topic == TEST_TOPIC)
            {
                // Client3 should not receive any retian message
                ASSERT_FALSE(false);
            }
            return 0;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client3 = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client3);

    // 1. client1 start and publish a retian message
    ASSERT_TRUE(mqtt5Client1->Start());
    ASSERT_TRUE(connection1Promise.get_future().get());
    std::shared_ptr<Mqtt5::PublishPacket> setRetainPacket =
        Aws::Crt::MakeShared<Mqtt5::PublishPacket>(allocator, allocator);
    setRetainPacket->WithTopic(TEST_TOPIC).WithPayload(ByteCursorFromString(RETAIN_MESSAGE)).WithRetain(true);
    ASSERT_TRUE(mqtt5Client1->Publish(setRetainPacket));

    // 2. connect to client 2
    ASSERT_TRUE(mqtt5Client2->Start());
    ASSERT_TRUE(connection2Promise.get_future().get());
    // 3. client2 subscribe to retain topic
    Mqtt5::Subscription subscription(TEST_TOPIC, Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe =
        Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator, allocator);
    subscribe->WithSubscription(std::move(subscription));
    ASSERT_TRUE(mqtt5Client2->Subscribe(subscribe));

    client2RetianMessageReceived.get_future().get();

    // Stop client2
    ASSERT_TRUE(mqtt5Client2->Stop());
    stopped2Promise.get_future().get();

    // 4. client1 reset retian message
    std::shared_ptr<Mqtt5::PublishPacket> clearRetainPacket =
        Aws::Crt::MakeShared<Mqtt5::PublishPacket>(allocator, allocator);
    clearRetainPacket->WithTopic(TEST_TOPIC).WithRetain(true);
    ASSERT_TRUE(mqtt5Client1->Publish(clearRetainPacket));

    // 5. client3 start and subscribe to retain topic
    ASSERT_TRUE(mqtt5Client3->Start());
    ASSERT_TRUE(connection3Promise.get_future().get());
    ASSERT_TRUE(mqtt5Client3->Subscribe(subscribe));

    // Wait for client 3
    aws_thread_current_sleep(2000 * 1000 * 1000);

    ASSERT_TRUE(mqtt5Client3->Stop());
    stopped3Promise.get_future().get();
    ASSERT_TRUE(mqtt5Client1->Stop());
    stopped1Promise.get_future().get();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5RetainSetAndClear, s_TestMqtt5RetainSetAndClear)

//////////////////////////////////////////////////////////
// Interruption tests [IT-UC]
//////////////////////////////////////////////////////////

/*
 * [IT-UC1] Interrupt subscription
 */
static int s_TestMqtt5InterruptSub(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    const Aws::Crt::String TEST_TOPIC = "test/s_TestMqtt5InterruptSub" + Aws::Crt::UUID().ToString();
    /* Subscribe to test topic */
    Mqtt5::Subscription subscription(TEST_TOPIC, Mqtt5::QOS::AWS_MQTT5_QOS_AT_MOST_ONCE, allocator);
    std::shared_ptr<Mqtt5::SubscribePacket> subscribe =
        Aws::Crt::MakeShared<Mqtt5::SubscribePacket>(allocator, allocator);
    subscribe->WithSubscription(std::move(subscription));
    ASSERT_TRUE(mqtt5Client->Subscribe(subscribe));

    /* Stop immediately */
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5InterruptSub, s_TestMqtt5InterruptSub)

/*
 * [IT-UC2] Interrupt Unsubscription
 */
static int s_TestMqtt5InterruptUnsub(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(builder);
    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    const Aws::Crt::String TEST_TOPIC = "test/s_TestMqtt5InterruptUnsub" + Aws::Crt::UUID().ToString();

    /* Unsub from topic*/
    Vector<String> topics;
    topics.push_back(TEST_TOPIC);
    std::shared_ptr<Mqtt5::UnsubscribePacket> unsub =
        Aws::Crt::MakeShared<Mqtt5::UnsubscribePacket>(allocator, allocator);
    unsub->WithTopicFilters(topics);
    ASSERT_TRUE(mqtt5Client->Unsubscribe(unsub));

    /* Stop immediately */
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5InterruptUnsub, s_TestMqtt5InterruptUnsub)

/*
 * [IT-UC3] Interrupt Publish
 */
static int s_TestMqtt5InterruptPublishQoS1(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    const Aws::Crt::String TEST_TOPIC = "test/s_TestMqtt5InterruptPublish" + Aws::Crt::UUID().ToString();

    /* Publish QOS1 to test topic */
    ByteBuf payload = Aws::Crt::ByteBufFromCString("Hello World");
    std::shared_ptr<Mqtt5::PublishPacket> publish = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    ASSERT_TRUE(mqtt5Client->Publish(publish));

    /* Stop immediately */
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5InterruptPublishQoS1, s_TestMqtt5InterruptPublishQoS1)

//////////////////////////////////////////////////////////
// Misc Tests
//////////////////////////////////////////////////////////

/*
 * [Misc] test_operation_statistics_uc1
 */
static int s_TestMqtt5OperationStatisticsSimple(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    const String TEST_TOPIC = "test/MQTT5_Binding_CPP" + Aws::Crt::UUID().ToString();

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string).WithPort(mqtt5TestVars.m_port_value);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    /* Make sure the operations are empty */
    Mqtt5::Mqtt5ClientOperationStatistics statistics = mqtt5Client->GetOperationStatistics();
    ASSERT_INT_EQUALS(0, statistics.incompleteOperationCount);
    ASSERT_INT_EQUALS(0, statistics.incompleteOperationSize);
    ASSERT_INT_EQUALS(0, statistics.unackedOperationCount);
    ASSERT_INT_EQUALS(0, statistics.unackedOperationSize);

    /* Publish message 1 to test topic */
    ByteBuf payload = Aws::Crt::ByteBufFromCString("Hello World");
    std::shared_ptr<Mqtt5::PublishPacket> publish = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);
    ASSERT_TRUE(mqtt5Client->Publish(publish));

    // Sleep and wait for message received
    aws_thread_current_sleep(2000 * 1000 * 1000);

    /* Make sure the operations are empty */
    statistics = mqtt5Client->GetOperationStatistics();
    ASSERT_INT_EQUALS(0, statistics.incompleteOperationCount);
    ASSERT_INT_EQUALS(0, statistics.incompleteOperationSize);
    ASSERT_INT_EQUALS(0, statistics.unackedOperationCount);
    ASSERT_INT_EQUALS(0, statistics.unackedOperationSize);

    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5OperationStatisticsSimple, s_TestMqtt5OperationStatisticsSimple)

/* Mqtt5-to-Mqtt3 Adapter Test */

/* Test Helper Functions */
/* Test connect and disconnect with MqttConnection Interface */
static int s_ConnectAndDisconnectThroughMqtt3(std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connection)
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

    // Mqtt5 Test client policy only allows client id start with "test-"
    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = "test-" + Uuid.ToString();

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

/* Test connect and disconnect with Mqtt5 Interface */
static int s_AdapterConenctAndDisconnectThroughMqtt5(
    Mqtt5::Mqtt5ClientOptions &mqtt5Options,
    Aws::Crt::Allocator *allocator,
    const char *clientName = "Client")
{

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;
    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise, clientName);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    std::shared_ptr<Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection && *mqttConnection);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    return AWS_OP_SUCCESS;
}

/*
 * [Mqtt5to3Adapter-UC1] Happy path. Minimal creation and cleanup
 */
static int s_TestMqtt5to3AdapterNewConnectionMin(Aws::Crt::Allocator *allocator, void *)
{
    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    // Hardcoded the host name and port for creation test
    mqtt5Options.WithHostName("localhost").WithPort(1883);
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    std::shared_ptr<Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterNewConnectionMin, s_TestMqtt5to3AdapterNewConnectionMin)

/*
 * [Mqtt5to3Adapter-UC2] Maximum creation and cleanup
 */
static int s_TestMqtt5to3AdapterNewClientFull(Aws::Crt::Allocator *allocator, void *)
{
    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    // Hardcoded the host name and port for creation test
    mqtt5Options.WithHostName("localhost").WithPort(1883);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    // Setup will
    const Aws::Crt::String TEST_TOPIC = "test/MQTT5_Binding_CPP/s_TestMqtt5NewClientFull";
    ByteBuf will_payload = Aws::Crt::ByteBufFromCString("Will Test");
    std::shared_ptr<Mqtt5::PublishPacket> will = Aws::Crt::MakeShared<Mqtt5::PublishPacket>(
        allocator, TEST_TOPIC, ByteCursorFromByteBuf(will_payload), Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE, allocator);

    std::shared_ptr<Aws::Crt::Mqtt5::ConnectPacket> packetConnect =
        Aws::Crt::MakeShared<Aws::Crt::Mqtt5::ConnectPacket>(allocator);
    packetConnect->WithClientId("s_TestMqtt5DirectConnectionFull" + Aws::Crt::UUID().ToString())
        .WithKeepAliveIntervalSec(1000)
        .WithMaximumPacketSizeBytes(1000L)
        .WithReceiveMaximum(1000)
        .WithRequestProblemInformation(true)
        .WithRequestResponseInformation(true)
        .WithSessionExpiryIntervalSec(1000L)
        .WithWill(will)
        .WithWillDelayIntervalSec(1000);

    Aws::Crt::Mqtt5::UserProperty userProperty("PropertyName", "PropertyValue");
    packetConnect->WithUserProperty(std::move(userProperty));

    Aws::Crt::Mqtt5::ReconnectOptions reconnectOptions = {
        Mqtt5::JitterMode::AWS_EXPONENTIAL_BACKOFF_JITTER_FULL, 1000, 1000, 1000};

    mqtt5Options.WithConnectOptions(packetConnect);
    mqtt5Options.WithBootstrap(&clientBootstrap);
    mqtt5Options.WithSocketOptions(socketOptions);
    mqtt5Options.WithSessionBehavior(Mqtt5::ClientSessionBehaviorType::AWS_MQTT5_CSBT_REJOIN_POST_SUCCESS);
    mqtt5Options.WithClientExtendedValidationAndFlowControl(
        Mqtt5::ClientExtendedValidationAndFlowControl::AWS_MQTT5_EVAFCO_NONE);
    mqtt5Options.WithOfflineQueueBehavior(
        Mqtt5::ClientOperationQueueBehaviorType::AWS_MQTT5_COQBT_FAIL_QOS0_PUBLISH_ON_DISCONNECT);
    mqtt5Options.WithReconnectOptions(reconnectOptions);
    mqtt5Options.WithPingTimeoutMs(10000);
    mqtt5Options.WithConnackTimeoutMs(10000);
    mqtt5Options.WithAckTimeoutSec(60000);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    std::shared_ptr<Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection && *mqttConnection);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterNewClientFull, s_TestMqtt5to3AdapterNewClientFull)

/*
 * [Mqtt5to3Adapter-UC3] Happy path. Minimal direct connection through Mqtt3 Interface
 */
static int s_TestMqtt5to3AdapterDirectConnectionMinimalThroughMqtt3(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);

    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    std::shared_ptr<Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);
    int connectResult = s_ConnectAndDisconnectThroughMqtt3(mqttConnection);
    ASSERT_SUCCESS(connectResult);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    Mqtt5to3AdapterDirectConnectionMinimalThroughMqtt3,
    s_TestMqtt5to3AdapterDirectConnectionMinimalThroughMqtt3)

/*
 * [Mqtt5to3Adapter-UC4] Websocket creation and connection through Mqtt3 Interface
 */
static int s_TestMqtt5to3AdapterWSConnectionMinimalThroughMqtt3(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    int Mqtt5WebSocket = 0;
    int Mqtt3WebSocket = 0;

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config, &Mqtt5WebSocket](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
            Mqtt5WebSocket = 1;
        });

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    std::shared_ptr<Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);

    mqttConnection->WebsocketInterceptor = [config, &Mqtt3WebSocket](
                                               std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
                                               const Aws::Crt::Mqtt::OnWebSocketHandshakeInterceptComplete &onComplete)
    {
        auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
        { onComplete(req1, errorCode); };

        auto signerConfig = config.CreateSigningConfigCb();

        config.Signer->SignRequest(req, *signerConfig, signingComplete);
        Mqtt3WebSocket = 1;
    };

    int connectResult = s_ConnectAndDisconnectThroughMqtt3(mqttConnection);
    ASSERT_SUCCESS(connectResult);
    ASSERT_TRUE(Mqtt3WebSocket == 1);
    ASSERT_TRUE(Mqtt5WebSocket == 0);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterWSConnectionMinimalThroughMqtt3, s_TestMqtt5to3AdapterWSConnectionMinimalThroughMqtt3)

/*
 * [Mqtt5to3Adapter-UC5] IoT MutalTLS creation and cleanup with Mqtt5ClientBuilder through Mqtt3 Interface
 */
static int s_TestMqtt5to3AdapterWithIoTConnectionThroughMqtt3(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(builder);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();
    ASSERT_TRUE(mqtt5Client);

    // Created a Mqtt311 Connection from mqtt5Client. The options are setup by the builder.
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);
    delete builder;
    int connectResult = s_ConnectAndDisconnectThroughMqtt3(mqttConnection);
    ASSERT_SUCCESS(connectResult);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterWithIoTConnectionThroughMqtt3, s_TestMqtt5to3AdapterWithIoTConnectionThroughMqtt3)

/*
 * [Mqtt5to3Adapter-UC6] MutalTLS connection through Mqtt3 Interface
 */
static int s_TestMqtt5to3AdapterDirectConnectionWithMutualTLSThroughMqtt3(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        mqtt5TestVars.m_certificate_path_string.c_str(), mqtt5TestVars.m_private_key_path_string.c_str(), allocator);

    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();
    ASSERT_TRUE(tlsConnection);
    ASSERT_TRUE(tlsConnection.SetAlpnList("x-amzn-mqtt-ca"));
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    // Created a Mqtt311 Connection from mqtt5Client. The options are setup by the builder.
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);
    int connectResult = s_ConnectAndDisconnectThroughMqtt3(mqttConnection);
    ASSERT_SUCCESS(connectResult);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    Mqtt5to3AdapterDirectConnectionWithMutualTLSThroughMqtt3,
    s_TestMqtt5to3AdapterDirectConnectionWithMutualTLSThroughMqtt3)

/*
 * [Mqtt5to3Adapter-UC7] Happy path. Minimal direct connection through Mqtt5 Interface
 */
static int s_TestMqtt5to3AdapterDirectConnectionMinimalThroughMqtt5(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_DIRECT);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);
    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);
    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);

    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    return s_AdapterConenctAndDisconnectThroughMqtt5(mqtt5Options, allocator);
}
AWS_TEST_CASE(
    Mqtt5to3AdapterDirectConnectionMinimalThroughMqtt5,
    s_TestMqtt5to3AdapterDirectConnectionMinimalThroughMqtt5)

/*
 * [Mqtt5to3Adapter-UC8] Websocket creation and connection through Mqtt5 Interface
 */
static int s_TestMqtt5to3AdapterWSConnectionMinimalThroughMqtt5(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_WS);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(mqtt5TestVars.m_port_value);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);

    ASSERT_TRUE(provider);

    Aws::Iot::WebsocketConfig config("us-east-1", provider);

    int Mqtt5WebSocket = 0;
    int Mqtt3WebSocket = 0;

    mqtt5Options.WithWebsocketHandshakeTransformCallback(
        [config, &Mqtt5WebSocket](
            std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
            const Aws::Crt::Mqtt5::OnWebSocketHandshakeInterceptComplete &onComplete)
        {
            auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
            { onComplete(req1, errorCode); };

            auto signerConfig = config.CreateSigningConfigCb();

            config.Signer->SignRequest(req, *signerConfig, signingComplete);
            Mqtt5WebSocket = 1;
        });

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;
    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);

    std::shared_ptr<Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);

    mqttConnection->WebsocketInterceptor = [config, &Mqtt3WebSocket](
                                               std::shared_ptr<Aws::Crt::Http::HttpRequest> req,
                                               const Aws::Crt::Mqtt::OnWebSocketHandshakeInterceptComplete &onComplete)
    {
        auto signingComplete = [onComplete](const std::shared_ptr<Aws::Crt::Http::HttpRequest> &req1, int errorCode)
        { onComplete(req1, errorCode); };

        auto signerConfig = config.CreateSigningConfigCb();

        config.Signer->SignRequest(req, *signerConfig, signingComplete);
        Mqtt3WebSocket = 1;
    };

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    ASSERT_TRUE(Mqtt3WebSocket == 0);
    ASSERT_TRUE(Mqtt5WebSocket == 1);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterWSConnectionMinimalThroughMqtt5, s_TestMqtt5to3AdapterWSConnectionMinimalThroughMqtt5)

/*
 * [Mqtt5to3Adapter-UC9] IoT MutalTLS creation and cleanup with Mqtt5ClientBuilder through Mqtt5 Interface
 */
static int s_TestMqtt5to3AdapterWithIoTConnectionThroughMqtt5(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();
    ASSERT_TRUE(mqtt5Client);
    // Created a Mqtt311 Connection from mqtt5Client. The options are setup by the builder.
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    /* Stop immediately */
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterWithIoTConnectionThroughMqtt5, s_TestMqtt5to3AdapterWithIoTConnectionThroughMqtt5)

/*
 * [Mqtt5to3Adapter-UC10] MutalTLS connection through Mqtt5 Interface
 */
static int s_TestMqtt5to3AdapterDirectConnectionWithMutualTLSThroughMqtt5(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        mqtt5TestVars.m_certificate_path_string.c_str(), mqtt5TestVars.m_private_key_path_string.c_str(), allocator);

    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();
    ASSERT_TRUE(tlsConnection);
    ASSERT_TRUE(tlsConnection.SetAlpnList("x-amzn-mqtt-ca"));
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    return s_AdapterConenctAndDisconnectThroughMqtt5(mqtt5Options, allocator);
}
AWS_TEST_CASE(
    Mqtt5to3AdapterDirectConnectionWithMutualTLSThroughMqtt5,
    s_TestMqtt5to3AdapterDirectConnectionWithMutualTLSThroughMqtt5)

/*
 * [Mqtt5to3Adapter-UC11] Test sub/unsub/publish operations through adapter
 */
static int s_TestMqtt5to3AdapterOperations(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(builder);

    bool subscribed = false;
    bool published = false;
    bool unsubscribed = false;
    uint8_t received = 0;
    std::mutex mutex;
    std::condition_variable cv;
    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    String testUUID = Aws::Crt::UUID().ToString();
    String testTopic = "test/MQTT5to3Adapter_" + testUUID;
    ByteBuf testPayload = Aws::Crt::ByteBufFromCString("PUBLISH ME!");

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();
    ASSERT_TRUE(mqtt5Client);
    // Created a Mqtt311 Connection from mqtt5Client. The options are setup by the builder.
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    auto onMessage = [&](Mqtt::MqttConnection &, const String &topic, const ByteBuf &payload, bool, Mqtt::QOS, bool)
    {
        printf("GOT MESSAGE topic=%s payload=" PRInSTR "\n", topic.c_str(), AWS_BYTE_BUF_PRI(payload));
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++received;
        }
        cv.notify_one();
    };
    auto onSubAck = [&](Mqtt::MqttConnection &, uint16_t packetId, const Aws::Crt::String &topic, Mqtt::QOS qos, int)
    {
        printf("SUBACK id=%d topic=%s qos=%d\n", packetId, topic.c_str(), qos);
        {
            std::lock_guard<std::mutex> lock(mutex);
            subscribed = true;
        }
        cv.notify_one();
    };
    auto onPubAck = [&](Mqtt::MqttConnection &, uint16_t packetId, int)
    {
        printf("PUBLISHED id=%d\n", packetId);
        {
            std::lock_guard<std::mutex> lock(mutex);
            published = true;
        }
        cv.notify_one();
    };
    auto onUnsubAck = [&](Mqtt::MqttConnection &, uint16_t packetId, int)
    {
        printf("UNSUBACK id=%d\n", packetId);
        {
            std::lock_guard<std::mutex> lock(mutex);
            unsubscribed = true;
        }
        cv.notify_one();
    };

    mqttConnection->Subscribe(
        testTopic.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, std::move(onMessage), std::move(onSubAck));

    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return subscribed; });
    }

    mqttConnection->Publish(testTopic.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, testPayload, onPubAck);

    // wait for publish
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return published; });
    }

    // Sleep and wait for message recieved
    aws_thread_current_sleep(2000 * 1000 * 1000);

    mqttConnection->Unsubscribe(testTopic.c_str(), onUnsubAck);
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return subscribed; });
    }
    published = false;
    mqttConnection->Publish(testTopic.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, testPayload, onPubAck);

    // wait for publish
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return published; });
    }

    /* Stop immediately */
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    delete builder;

    ASSERT_TRUE(received == 1);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterOperations, s_TestMqtt5to3AdapterOperations)

/*
 * [Mqtt5to3Adapter-UC11] Test s_TestMqtt5to3AdapterNullPubAck
 * The unit test would have memory leak if the callback data for incomplete publish was not released.
 */
static int s_TestMqtt5to3AdapterNullPubAck(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        mqtt5TestVars.m_hostname_string,
        mqtt5TestVars.m_certificate_path_string.c_str(),
        mqtt5TestVars.m_private_key_path_string.c_str(),
        allocator);
    ASSERT_TRUE(builder);

    String testUUID = Aws::Crt::UUID().ToString();
    String testTopic = "test/MQTT5to3Adapter_" + testUUID;
    ByteBuf testPayload = Aws::Crt::ByteBufFromCString("PUBLISH ME!");

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();
    ASSERT_TRUE(mqtt5Client);
    // Created a Mqtt311 Connection from mqtt5Client. The options are setup by the builder.
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqttConnection =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection);

    // Publish an offline message to create an incomplete publish operation
    mqttConnection->Publish(testTopic.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, testPayload, NULL);

    delete builder;

    // If the incomplete operation callback was not called, there would be a memory leak as the callback data was not
    // released
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(Mqtt5to3AdapterNullPubAck, s_TestMqtt5to3AdapterNullPubAck)

/*
 * [Mqtt5to3Adapter-UC11] Test one mqtt5 client with multiple adapters
 */
static int s_TestMqtt5to3AdapterMultipleAdapters(Aws::Crt::Allocator *allocator, void *)
{
    Mqtt5TestEnvVars mqtt5TestVars(allocator, MQTT5CONNECT_IOT_CORE);
    if (!mqtt5TestVars)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }
    String randomID = Aws::Crt::UUID().ToString();
    String testTopic1 = "test/topic1_" + randomID;
    String testTopic2 = "test/topic2_" + randomID;

    ApiHandle apiHandle(allocator);

    Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
    mqtt5Options.WithHostName(mqtt5TestVars.m_hostname_string);
    mqtt5Options.WithPort(443);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        mqtt5TestVars.m_certificate_path_string.c_str(), mqtt5TestVars.m_private_key_path_string.c_str(), allocator);

    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);
    Aws::Crt::Io::TlsConnectionOptions tlsConnection = tlsContext.NewConnectionOptions();
    ASSERT_TRUE(tlsConnection);
    ASSERT_TRUE(tlsConnection.SetAlpnList("x-amzn-mqtt-ca"));
    mqtt5Options.WithTlsConnectionOptions(tlsConnection);

    /* Setup Mqtt5 Client lifecycle evetns */
    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;
    s_setupConnectionLifeCycle(mqtt5Options, connectionPromise, stoppedPromise);

    std::shared_ptr<Mqtt5::Mqtt5Client> mqtt5Client = Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
    ASSERT_TRUE(mqtt5Client);
    // Created a Mqtt311 Connection from mqtt5Client. The options are setup by the builder.
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqttConnection1 =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection1);

    // Created a Mqtt311 Connection from mqtt5Client. The options are setup by the builder.
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqttConnection2 =
        Mqtt::MqttConnection::NewConnectionFromMqtt5Client(mqtt5Client);
    ASSERT_TRUE(mqttConnection2);

    size_t received1 = 0;
    size_t received2 = 0;
    std::mutex mutex;
    std::condition_variable cv;
    bool subscribed1 = false;
    bool subscribed2 = false;
    bool published = false;
    ByteBuf testPayload = Aws::Crt::ByteBufFromCString("PUBLISH ME!");

    auto onMessage1 = [&](Mqtt::MqttConnection &, const String &topic, const ByteBuf &payload, bool, Mqtt::QOS, bool)
    {
        printf("GOT MESSAGE topic=%s payload=" PRInSTR "\n", topic.c_str(), AWS_BYTE_BUF_PRI(payload));
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++received1;
        }
        cv.notify_one();
    };
    auto onSubAck1 = [&](Mqtt::MqttConnection &, uint16_t packetId, const Aws::Crt::String &topic, Mqtt::QOS qos, int)
    {
        printf("SUBACK id=%d topic=%s qos=%d\n", packetId, topic.c_str(), qos);
        {
            std::lock_guard<std::mutex> lock(mutex);
            subscribed1 = true;
        }
        cv.notify_one();
    };

    auto onMessage2 = [&](Mqtt::MqttConnection &, const String &topic, const ByteBuf &payload, bool, Mqtt::QOS, bool)
    {
        printf("GOT MESSAGE topic=%s payload=" PRInSTR "\n", topic.c_str(), AWS_BYTE_BUF_PRI(payload));
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++received2;
        }
        cv.notify_one();
    };
    auto onSubAck2 = [&](Mqtt::MqttConnection &, uint16_t packetId, const Aws::Crt::String &topic, Mqtt::QOS qos, int)
    {
        printf("SUBACK id=%d topic=%s qos=%d\n", packetId, topic.c_str(), qos);
        {
            std::lock_guard<std::mutex> lock(mutex);
            subscribed2 = true;
        }
        cv.notify_one();
    };

    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise.get_future().get());

    mqttConnection1->Subscribe(testTopic1.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, onMessage1, onSubAck1);
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return subscribed1; });
    }

    mqttConnection2->Subscribe(testTopic2.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, onMessage2, onSubAck2);
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return subscribed2; });
    }

    auto onPubAck = [&](Mqtt::MqttConnection &, uint16_t packetId, int)
    {
        printf("PUBLISHED id=%d\n", packetId);
        {
            std::lock_guard<std::mutex> lock(mutex);
            published = true;
        }
        cv.notify_one();
    };

    // Publish to testTopic1
    mqttConnection1->Publish(testTopic1.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, testPayload, onPubAck);
    // wait for publish
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return published; });
    }

    published = false;
    // Publish to testTopic2
    mqttConnection1->Publish(testTopic2.c_str(), Mqtt::QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, testPayload, onPubAck);
    // wait for publish
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return published; });
    }

    // Sleep and wait for message recieved
    aws_thread_current_sleep(2000 * 1000 * 1000);
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise.get_future().get();

    ASSERT_TRUE(received1 == 1);
    ASSERT_TRUE(received2 == 1);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(Mqtt5to3AdapterMultipleAdapters, s_TestMqtt5to3AdapterMultipleAdapters)

#endif // !BYO_CRYPTO∏
