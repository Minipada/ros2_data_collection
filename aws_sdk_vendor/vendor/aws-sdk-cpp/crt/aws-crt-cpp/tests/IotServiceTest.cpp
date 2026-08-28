/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/crt/Api.h>

#include <aws/testing/aws_test_harness.h>

#include <aws/common/clock.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>
#include <aws/crt/UUID.h>

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <thread>
#include <utility>

#include <aws/io/logging.h>

#if !BYO_CRYPTO

namespace
{

    struct IotServiceTestEnvVars
    {
        Aws::Crt::String inputHost;
        Aws::Crt::String inputCertificate;
        Aws::Crt::String inputPrivateKey;
        Aws::Crt::String inputRootCa;
    };

} // namespace

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_core_host, "AWS_TEST_MQTT311_IOT_CORE_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_core_cert, "AWS_TEST_MQTT311_IOT_CORE_RSA_CERT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_core_key, "AWS_TEST_MQTT311_IOT_CORE_RSA_KEY");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_core_ca, "AWS_TEST_MQTT311_ROOT_CA");

static Aws::Crt::Optional<Aws::Crt::String> s_GetEnvVariable(
    Aws::Crt::Allocator *allocator,
    const aws_string *variableName)
{
    Aws::Crt::Optional<Aws::Crt::String> value;

    aws_string *output = nullptr;
    int error = aws_get_environment_value(allocator, variableName, &output);
    if (error == AWS_OP_SUCCESS)
    {
        if (aws_string_is_valid(output))
        {
            value = Aws::Crt::String(aws_string_c_str(output), output->len);
        }
        else
        {
            error = AWS_OP_ERR;
        }
    }
    aws_string_destroy(output);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment variable %s is not set or missing\n", aws_string_c_str(variableName));
    }
    return value;
}

static int s_GetEnvVariables(Aws::Crt::Allocator *allocator, IotServiceTestEnvVars &envVars)
{
    Aws::Crt::Optional<Aws::Crt::String> inputHost = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_core_host);
    Aws::Crt::Optional<Aws::Crt::String> inputCertificate =
        s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_core_cert);
    Aws::Crt::Optional<Aws::Crt::String> inputPrivateKey =
        s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_core_key);
    Aws::Crt::Optional<Aws::Crt::String> inputRootCa = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_core_ca);

    if (!inputHost || !inputCertificate || !inputPrivateKey || !inputRootCa)
    {
        return AWS_OP_ERR;
    }

    envVars.inputHost = std::move(inputHost.value());
    envVars.inputCertificate = std::move(inputCertificate.value());
    envVars.inputPrivateKey = std::move(inputPrivateKey.value());
    envVars.inputRootCa = std::move(inputRootCa.value());

    return AWS_OP_SUCCESS;
}

static int s_ValidateCredentialFiles(const IotServiceTestEnvVars &envVars)
{
    const char *credentialFiles[] = {
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str(), envVars.inputRootCa.c_str()};
    for (size_t fileIdx = 0; fileIdx < AWS_ARRAY_SIZE(credentialFiles); ++fileIdx)
    {
        std::ifstream file;
        file.open(credentialFiles[fileIdx]);
        if (!file.is_open())
        {
            printf("Required credential file %s is missing or unreadable\n", credentialFiles[fileIdx]);
            return AWS_OP_ERR;
        }
    }

    return AWS_OP_SUCCESS;
}

static int s_TestIotPublishSubscribe(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    int tries = 0;
    while (tries++ < 10)
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
            envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
        tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
        Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
        ASSERT_TRUE(tlsContext);

        Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
        ASSERT_TRUE(eventLoopGroup);

        Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
        ASSERT_TRUE(defaultHostResolver);

        Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
        ASSERT_TRUE(allocator);
        clientBootstrap.EnableBlockingShutdown();

        Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
        ASSERT_TRUE(mqttClient);

        Aws::Crt::Io::SocketOptions socketOptions;
        socketOptions.SetConnectTimeoutMs(3000);

        auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

        std::mutex mutex;
        std::condition_variable cv;
        bool connected = false;
        bool subscribed = false;
        bool published = false;
        bool received = false;
        bool closed = false;
        auto onConnectionCompleted = [&](MqttConnection &, int errorCode, ReturnCode returnCode, bool sessionPresent)
        {
            printf(
                "%s errorCode=%d returnCode=%d sessionPresent=%d\n",
                (errorCode == 0) ? "CONNECTED" : "COMPLETED",
                errorCode,
                (int)returnCode,
                (int)sessionPresent);
            {
                std::lock_guard<std::mutex> lock(mutex);
                connected = true;
            }
            cv.notify_one();
        };
        auto onDisconnect = [&](MqttConnection &)
        {
            printf("DISCONNECTED\n");
            {
                std::lock_guard<std::mutex> lock(mutex);
                connected = false;
            }
            cv.notify_one();
        };
        auto onTest = [&](MqttConnection &, const Aws::Crt::String &topic, const Aws::Crt::ByteBuf &payload)
        {
            printf("GOT MESSAGE topic=%s payload=" PRInSTR "\n", topic.c_str(), AWS_BYTE_BUF_PRI(payload));
            {
                std::lock_guard<std::mutex> lock(mutex);
                received = true;
            }
            cv.notify_one();
        };
        auto onSubAck = [&](MqttConnection &, uint16_t packetId, const Aws::Crt::String &topic, QOS qos, int)
        {
            printf("SUBACK id=%d topic=%s qos=%d\n", packetId, topic.c_str(), qos);
            {
                std::lock_guard<std::mutex> lock(mutex);
                subscribed = true;
            }
            cv.notify_one();
        };
        auto onPubAck = [&](MqttConnection &, uint16_t packetId, int)
        {
            printf("PUBLISHED id=%d\n", packetId);
            {
                std::lock_guard<std::mutex> lock(mutex);
                published = true;
            }
            cv.notify_one();
        };
        auto onConnectionClosed = [&](MqttConnection &, OnConnectionClosedData *data)
        {
            (void)data;
            printf("CLOSED\n");
            {
                std::lock_guard<std::mutex> lock(mutex);
                closed = true;
                // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
                cv.notify_one();
            }
        };

        mqttConnection->OnConnectionCompleted = onConnectionCompleted;
        mqttConnection->OnDisconnect = onDisconnect;
        mqttConnection->OnConnectionClosed = onConnectionClosed;
        Aws::Crt::UUID Uuid;
        Aws::Crt::String uuidStr = Uuid.ToString();
        mqttConnection->Connect(uuidStr.c_str(), true);

        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return connected; });
        }

        mqttConnection->Subscribe("/publish/me/senpai", QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, onTest, onSubAck);

        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return subscribed; });
        }

        Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromCString("notice me pls");
        mqttConnection->Publish("/publish/me/senpai", QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPubAck);

        // wait for publish
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return published; });
        }

        // wait for the message received callback
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return received; });
        }

        mqttConnection->Disconnect();
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return !connected; });
        }

        // Make sure the closed callback fired
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return closed; });
        }
        ASSERT_TRUE(mqttConnection);
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotPublishSubscribe, s_TestIotPublishSubscribe)

static int s_TestIotConnectionSuccessTest(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
    tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
    ASSERT_TRUE(mqttClient);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);
    ASSERT_TRUE(mqttConnection);
    ASSERT_TRUE(*mqttConnection);

    std::mutex mutex;
    std::condition_variable cv;
    bool connection_success = false;
    bool closed = false;

    auto onConnectionSuccess = [&](MqttConnection &, OnConnectionSuccessData *data)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            connection_success = true;
        }
        printf("CONNECTION SUCCESS: returnCode=%i sessionPresent=%i\n", data->returnCode, data->sessionPresent);
        cv.notify_one();
    };

    auto onConnectionClosed = [&](MqttConnection &, OnConnectionClosedData *data)
    {
        (void)data;
        printf("CLOSED");
        {
            std::lock_guard<std::mutex> lock(mutex);
            closed = true;
            // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
            cv.notify_one();
        }
    };

    mqttConnection->OnConnectionSuccess = onConnectionSuccess;
    mqttConnection->OnConnectionClosed = onConnectionClosed;

    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();
    mqttConnection->Connect(uuidStr.c_str(), true);

    // Make sure the connection success callback fired
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return connection_success; });
    }

    mqttConnection->Disconnect();

    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return closed; });
    }
    ASSERT_TRUE(mqttConnection);

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotConnectionSuccessTest, s_TestIotConnectionSuccessTest)

static int s_TestIotConnectionFailureTest(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
    tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
    ASSERT_TRUE(mqttClient);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    // Intentially use a bad port so we fail to connect
    auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 123, socketOptions, tlsContext);

    std::mutex mutex;
    std::condition_variable cv;
    bool connection_failure = false;
    auto onConnectionFailure = [&](MqttConnection &, OnConnectionFailureData *data)
    {
        printf("CONNECTION FAILURE: error=%i\n", data->error);
        {
            std::lock_guard<std::mutex> lock(mutex);
            connection_failure = true;
            // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
            cv.notify_one();
        }
    };
    mqttConnection->OnConnectionFailure = onConnectionFailure;
    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();
    mqttConnection->Connect(uuidStr.c_str(), true);

    // Make sure the connection failure callback fired
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return connection_failure; });
    }
    ASSERT_TRUE(mqttConnection);

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotConnectionFailureTest, s_TestIotConnectionFailureTest)

static int s_TestIotWillTest(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
            envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
        tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
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

        Aws::Crt::UUID Uuid;
        Aws::Crt::String uuidStr = Uuid.ToString();

        Aws::Crt::String topicStr = "will/topic/";
        topicStr += uuidStr;
        Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromCString("notice me pls");

        auto willConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);
        willConnection->SetWill(topicStr.c_str(), QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload);
        std::mutex willMutex;
        std::condition_variable willCv;
        bool willConnected = false;
        auto willOnConnectionCompleted =
            [&](MqttConnection &, int errorCode, ReturnCode returnCode, bool sessionPresent)
        {
            (void)errorCode;
            (void)returnCode;
            (void)sessionPresent;
            {
                std::lock_guard<std::mutex> lock(willMutex);
                willConnected = true;
            }
            willCv.notify_one();
        };
        auto willOnDisconnect = [&](MqttConnection &)
        {
            {
                std::lock_guard<std::mutex> lock(willMutex);
                willConnected = false;
            }
            willCv.notify_one();
        };
        willConnection->OnConnectionCompleted = willOnConnectionCompleted;
        willConnection->OnDisconnect = willOnDisconnect;
        willConnection->Connect((Aws::Crt::String("test-01-") + uuidStr).c_str(), true);
        {
            std::unique_lock<std::mutex> lock(willMutex);
            willCv.wait(lock, [&]() { return willConnected; });
        }

        auto subscriberConnection =
            mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);
        std::mutex subscriberMutex;
        std::condition_variable subscriberCv;
        bool subscriberConnected = false;
        bool subscriberSubscribed = false;
        bool subscriberReceived = false;
        auto subscriberOnConnectionCompleted =
            [&](MqttConnection &, int errorCode, ReturnCode returnCode, bool sessionPresent)
        {
            (void)errorCode;
            (void)returnCode;
            (void)sessionPresent;
            {
                std::lock_guard<std::mutex> lock(subscriberMutex);
                subscriberConnected = true;
            }
            subscriberCv.notify_one();
        };
        auto subscriberOnDisconnect = [&](MqttConnection &)
        {
            {
                std::lock_guard<std::mutex> lock(subscriberMutex);
                subscriberConnected = false;
                // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
                subscriberCv.notify_one();
            }
        };
        auto subscriberOnSubAck = [&](MqttConnection &, uint16_t packetId, const Aws::Crt::String &topic, QOS qos, int)
        {
            (void)packetId;
            (void)topic;
            (void)qos;
            {
                std::lock_guard<std::mutex> lock(subscriberMutex);
                subscriberSubscribed = true;
            }
            subscriberCv.notify_one();
        };
        auto subscriberOnTest = [&](MqttConnection &, const Aws::Crt::String &topic, const Aws::Crt::ByteBuf &payload)
        {
            (void)topic;
            (void)payload;
            {
                std::lock_guard<std::mutex> lock(subscriberMutex);
                subscriberReceived = true;
            }
            subscriberCv.notify_one();
        };
        subscriberConnection->OnConnectionCompleted = subscriberOnConnectionCompleted;
        subscriberConnection->OnDisconnect = subscriberOnDisconnect;
        subscriberConnection->Connect((Aws::Crt::String("test-02-") + uuidStr).c_str(), true);
        {
            std::unique_lock<std::mutex> lock(subscriberMutex);
            subscriberCv.wait(lock, [&]() { return subscriberConnected; });
        }
        subscriberConnection->Subscribe(
            topicStr.c_str(), QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, subscriberOnTest, subscriberOnSubAck);
        {
            std::unique_lock<std::mutex> lock(subscriberMutex);
            subscriberCv.wait(lock, [&]() { return subscriberSubscribed; });
        }

        // there appears to be a race condition broker-side in IoT Core such that the interrupter is occasionally
        // rejected rather than the existing connection.  We try and work around this in two ways:
        //  (1) Wait a couple seconds to let things settle in terms of eventual consistency.
        //  (2) Have the interrupter make connection attempts in a loop until a successful interruption occurs.
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Disconnect the client by interrupting it with another client with the same ID
        // which will cause the will to be sent
        auto interruptConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);
        std::mutex interruptMutex;
        std::condition_variable interruptCv;
        bool interruptConnected = false;
        bool interruptConnectionAttemptComplete = false;
        auto interruptOnConnectionCompleted =
            [&](MqttConnection &, int errorCode, ReturnCode returnCode, bool sessionPresent)
        {
            (void)errorCode;
            (void)returnCode;
            (void)sessionPresent;
            {
                std::lock_guard<std::mutex> lock(interruptMutex);
                interruptConnectionAttemptComplete = true;
                if (errorCode == AWS_ERROR_SUCCESS && returnCode == AWS_MQTT_CONNECT_ACCEPTED)
                {
                    interruptConnected = true;
                }
            }
            interruptCv.notify_one();
        };
        auto interruptOnDisconnect = [&](MqttConnection &)
        {
            {
                std::lock_guard<std::mutex> lock(interruptMutex);
                interruptConnected = false;
            }
            interruptCv.notify_one();
        };
        interruptConnection->OnConnectionCompleted = interruptOnConnectionCompleted;
        interruptConnection->OnDisconnect = interruptOnDisconnect;

        bool continueConnecting = true;
        while (continueConnecting)
        {
            interruptConnection->Connect((Aws::Crt::String("test-01-") + uuidStr).c_str(), true);
            {
                std::unique_lock<std::mutex> lock(interruptMutex);
                interruptCv.wait(lock, [&]() { return interruptConnectionAttemptComplete; });
                continueConnecting = !interruptConnected;
            }
        }

        // wait for message received callback - meaning the will was sent
        {
            std::unique_lock<std::mutex> lock(subscriberMutex);
            subscriberCv.wait(lock, [&]() { return subscriberReceived; });
        }

        // Disconnect everything
        willConnection->Disconnect();
        {
            std::unique_lock<std::mutex> lock(willMutex);
            willCv.wait(lock, [&]() { return !willConnected; });
        }
        interruptConnection->Disconnect();
        {
            std::unique_lock<std::mutex> lock(interruptMutex);
            interruptCv.wait(lock, [&]() { return !interruptConnected; });
        }
        subscriberConnection->Disconnect();
        {
            std::unique_lock<std::mutex> lock(subscriberMutex);
            subscriberCv.wait(lock, [&]() { return !subscriberConnected; });
        }
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotWillTest, s_TestIotWillTest)

static int s_TestIotStatisticsPublishWaitStatisticsDisconnect(Aws::Crt::Allocator *allocator, void *ctx)
{
    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
            envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
        tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
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

        auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

        std::mutex mutex;
        std::condition_variable cv;
        bool connected = false;
        bool published = false;
        auto onConnectionCompleted = [&](MqttConnection &, int errorCode, ReturnCode returnCode, bool sessionPresent)
        {
            printf(
                "%s errorCode=%d returnCode=%d sessionPresent=%d\n",
                (errorCode == 0) ? "CONNECTED" : "COMPLETED",
                errorCode,
                (int)returnCode,
                (int)sessionPresent);
            {
                std::lock_guard<std::mutex> lock(mutex);
                connected = true;
            }
            cv.notify_one();
        };
        auto onDisconnect = [&](MqttConnection &)
        {
            printf("DISCONNECTED\n");
            {
                std::lock_guard<std::mutex> lock(mutex);
                connected = false;
                // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
                cv.notify_one();
            }
        };
        auto onPubAck = [&](MqttConnection &, uint16_t packetId, int)
        {
            printf("PUBLISHED id=%d\n", packetId);
            {
                std::lock_guard<std::mutex> lock(mutex);
                published = true;
            }
            cv.notify_one();
        };

        mqttConnection->OnConnectionCompleted = onConnectionCompleted;
        mqttConnection->OnDisconnect = onDisconnect;
        Aws::Crt::UUID Uuid;
        Aws::Crt::String uuidStr = Uuid.ToString();
        mqttConnection->Connect(uuidStr.c_str(), true);

        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return connected; });
        }

        // Check operation statistics
        Aws::Crt::Mqtt::MqttConnectionOperationStatistics statistics = mqttConnection->GetOperationStatistics();
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationCount);
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationSize);
        // We skip the unacked beecause it is heavily socket-timing based and we (currently) do not have good control
        // over that.
        // TODO: Find a way to reliably test the unacked statistics

        Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromCString("notice me pls");
        mqttConnection->Publish("/publish/me/senpai", QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPubAck);

        // wait for publish
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return published; });
        }

        // Check operation statistics
        statistics = mqttConnection->GetOperationStatistics();
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationCount);
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationSize);
        // We skip the unacked beecause it is heavily socket-timing based and we (currently) do not have good control
        // over that.
        // TODO: Find a way to reliably test the unacked statistics

        mqttConnection->Disconnect();
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return !connected; });
        }
        ASSERT_TRUE(mqttConnection);
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IoTStatisticsPublishWaitStatisticsDisconnect, s_TestIotStatisticsPublishWaitStatisticsDisconnect)

static int s_TestIotStatisticsPublishStatisticsWaitDisconnect(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
            envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
        tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
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

        auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

        std::mutex mutex;
        std::condition_variable cv;
        bool connected = false;
        bool published = false;
        auto onConnectionCompleted = [&](MqttConnection &, int errorCode, ReturnCode returnCode, bool sessionPresent)
        {
            printf(
                "%s errorCode=%d returnCode=%d sessionPresent=%d\n",
                (errorCode == 0) ? "CONNECTED" : "COMPLETED",
                errorCode,
                (int)returnCode,
                (int)sessionPresent);
            {
                std::lock_guard<std::mutex> lock(mutex);
                connected = true;
            }
            cv.notify_one();
        };
        auto onDisconnect = [&](MqttConnection &)
        {
            printf("DISCONNECTED\n");
            {
                std::lock_guard<std::mutex> lock(mutex);
                connected = false;
                // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
                cv.notify_one();
            }
        };
        auto onPubAck = [&](MqttConnection &, uint16_t packetId, int)
        {
            printf("PUBLISHED id=%d\n", packetId);
            {
                std::lock_guard<std::mutex> lock(mutex);
                published = true;
            }
            cv.notify_one();
        };

        mqttConnection->OnConnectionCompleted = onConnectionCompleted;
        mqttConnection->OnDisconnect = onDisconnect;
        Aws::Crt::UUID Uuid;
        Aws::Crt::String uuidStr = Uuid.ToString();
        mqttConnection->Connect(uuidStr.c_str(), true);
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return connected; });
        }

        // Check operation statistics
        Aws::Crt::Mqtt::MqttConnectionOperationStatistics statistics = mqttConnection->GetOperationStatistics();
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationCount);
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationSize);

        Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromCString("notice me pls");
        mqttConnection->Publish("/publish/me/senpai", QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPubAck);

        // Check operation statistics
        // Per packet: (The size of the topic (18), the size of the payload, 2 for the header and 2 for the packet ID)
        uint64_t expected_size = payload.len + 22;
        statistics = mqttConnection->GetOperationStatistics();
        ASSERT_INT_EQUALS(1, statistics.incompleteOperationCount);
        ASSERT_INT_EQUALS(expected_size, statistics.incompleteOperationSize);
        // We skip the unacked beecause it is heavily socket-timing based and we (currently) do not have good control
        // over that.
        // TODO: Find a way to reliably test the unacked statistics

        // wait for publish
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return published; });
        }

        // Check operation statistics
        statistics = mqttConnection->GetOperationStatistics();
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationCount);
        ASSERT_INT_EQUALS(0, statistics.incompleteOperationSize);
        // We skip the unacked beecause it is heavily socket-timing based and we (currently) do not have good control
        // over that.
        // TODO: Find a way to reliably test the unacked statistics

        mqttConnection->Disconnect();
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return !connected; });
        }
        ASSERT_TRUE(mqttConnection);
    }

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IoTStatisticsPublishStatisticsWaitDisconnect, s_TestIotStatisticsPublishStatisticsWaitDisconnect)

static int s_TestIotConnectionDestruction(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
    tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
    ASSERT_TRUE(mqttClient);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

    std::mutex mutex;
    std::condition_variable cv;
    bool connection_success = false;

    auto onConnectionSuccess = [&](MqttConnection &, OnConnectionSuccessData *data)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            connection_success = true;
        }
        printf("CONNECTION SUCCESS: returnCode=%i sessionPresent=%i\n", data->returnCode, data->sessionPresent);
        cv.notify_one();
    };

    mqttConnection->OnConnectionSuccess = onConnectionSuccess;

    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();
    mqttConnection->Connect(uuidStr.c_str(), true);

    // Make sure the connection success callback fired
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return connection_success; });
    }

    mqttConnection->Disconnect();

    // Intentionally don't wait for the disconnect callback completion.

    ASSERT_TRUE(mqttConnection);

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotConnectionDestruction, s_TestIotConnectionDestruction)

static int s_TestIotConnectionDestructionWithExecutingCallback(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
    tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
    ASSERT_TRUE(mqttClient);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

    std::mutex mutex;
    std::condition_variable cv;
    bool connectionSuccess = false;
    bool disconnectingStarted = false;

    auto onConnectionSuccess = [&](MqttConnection &, OnConnectionSuccessData *data)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            connectionSuccess = true;
        }
        printf("CONNECTION SUCCESS: returnCode=%i sessionPresent=%i\n", data->returnCode, data->sessionPresent);
        cv.notify_one();
    };

    mqttConnection->OnConnectionSuccess = onConnectionSuccess;

    mqttConnection->OnDisconnect = [&](MqttConnection &)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            disconnectingStarted = true;
            // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
            cv.notify_one();
        }
        printf("Disconnecting...\n");
        // Add some delay to the disconnection callback, so the destruction process will definitely start while
        // the callback is still executing.
        aws_thread_current_sleep(aws_timestamp_convert(2, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, nullptr));
    };

    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();
    mqttConnection->Connect(uuidStr.c_str(), true);

    // Make sure the connection success callback fired
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return connectionSuccess; });
    }

    mqttConnection->Disconnect();

    // Make sure the onDisconnect callback fired.
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return disconnectingStarted; });
    }

    // Intentionally don't wait for the dicsonnect callback completion.

    ASSERT_TRUE(mqttConnection);

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotConnectionDestructionWithExecutingCallback, s_TestIotConnectionDestructionWithExecutingCallback)

static int s_TestIotConnectionDestructionWithinConnectionCallback(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
    tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
    ASSERT_TRUE(mqttClient);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

    std::mutex mutex;
    std::condition_variable cv;
    bool connection_success = false;

    auto onConnectionSuccess = [&](MqttConnection &, OnConnectionSuccessData *data)
    {
        // Destroy mqtt connection object.
        mqttConnection.reset();

        {
            std::lock_guard<std::mutex> lock(mutex);
            connection_success = true;
            // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
            cv.notify_one();
        }
        printf("CONNECTION SUCCESS: returnCode=%i sessionPresent=%i\n", data->returnCode, data->sessionPresent);
    };

    mqttConnection->OnConnectionSuccess = onConnectionSuccess;

    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();
    mqttConnection->Connect(uuidStr.c_str(), true);

    // Make sure the connection success callback fired
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return connection_success; });
    }

    ASSERT_FALSE(mqttConnection);

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotConnectionDestructionWithinConnectionCallback, s_TestIotConnectionDestructionWithinConnectionCallback)

static int s_TestIotConnectionDestructionWithinDisconnectCallback(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
    tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
    ASSERT_TRUE(mqttClient);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

    std::mutex mutex;
    std::condition_variable cv;
    bool connectionSuccess = false;
    bool disconnected = false;

    auto onConnectionSuccess = [&](MqttConnection &, OnConnectionSuccessData *data)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            connectionSuccess = true;
        }
        printf("CONNECTION SUCCESS: returnCode=%i sessionPresent=%i\n", data->returnCode, data->sessionPresent);
        cv.notify_one();
    };

    mqttConnection->OnConnectionSuccess = onConnectionSuccess;

    mqttConnection->OnDisconnect = [&](MqttConnection &)
    {
        // Destroy mqtt connection object.
        mqttConnection.reset();
        {
            std::lock_guard<std::mutex> lock(mutex);
            disconnected = true;
            // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
            cv.notify_one();
        }
    };

    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();
    mqttConnection->Connect(uuidStr.c_str(), true);

    // Make sure the connection success callback fired
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return connectionSuccess; });
    }

    mqttConnection->Disconnect();

    // Make sure the onDisconnect callback fired.
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return disconnected; });
    }

    ASSERT_FALSE(mqttConnection);

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotConnectionDestructionWithinDisconnectCallback, s_TestIotConnectionDestructionWithinDisconnectCallback)

static int s_TestIotConnectionDestructionWithPublish(Aws::Crt::Allocator *allocator, void *ctx)
{
    (void)ctx;

    using namespace Aws::Crt::Mqtt;

    IotServiceTestEnvVars envVars;
    if (s_GetEnvVariables(allocator, envVars) != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    if (s_ValidateCredentialFiles(envVars) != AWS_OP_SUCCESS)
    {
        printf("Credential files are not set for the test, skip the test");
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        envVars.inputCertificate.c_str(), envVars.inputPrivateKey.c_str());
    tlsCtxOptions.OverrideDefaultTrustStore(nullptr, envVars.inputRootCa.c_str());
    Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);
    ASSERT_TRUE(tlsContext);

    Aws::Crt::Io::EventLoopGroup eventLoopGroup(0, allocator);
    ASSERT_TRUE(eventLoopGroup);

    Aws::Crt::Io::DefaultHostResolver defaultHostResolver(eventLoopGroup, 8, 30, allocator);
    ASSERT_TRUE(defaultHostResolver);

    Aws::Crt::Io::ClientBootstrap clientBootstrap(eventLoopGroup, defaultHostResolver, allocator);
    ASSERT_TRUE(allocator);
    clientBootstrap.EnableBlockingShutdown();

    Aws::Crt::Mqtt::MqttClient mqttClient(clientBootstrap, allocator);
    ASSERT_TRUE(mqttClient);

    Aws::Crt::Io::SocketOptions socketOptions;
    socketOptions.SetConnectTimeoutMs(3000);

    auto mqttConnection = mqttClient.NewConnection(envVars.inputHost.c_str(), 8883, socketOptions, tlsContext);

    std::mutex mutex;
    std::condition_variable cv;
    bool connected = false;
    bool published = false;
    auto onConnectionSuccess = [&](MqttConnection &, OnConnectionSuccessData * /*data*/)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            connected = true;
        }
        cv.notify_one();
    };
    mqttConnection->OnConnectionSuccess = onConnectionSuccess;

    Aws::Crt::UUID Uuid;
    Aws::Crt::String uuidStr = Uuid.ToString();
    mqttConnection->Connect(uuidStr.c_str(), true);

    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return connected; });
    }

    // Publish data.
    Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromCString("notice me pls");
    auto onPubAck = [&](MqttConnection &connection, uint16_t packetId, int)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            published = true;
            // This notify_one call has to be under mutex, to prevent a possible use-after-free case.
            cv.notify_one();
        }
        // Add some time for the main thread to destroy the connection.
        aws_thread_current_sleep(aws_timestamp_convert(2, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, nullptr));

        // Try to access the connection object.
        printf("On published: packet id is %d, connection last error is %d\n", packetId, connection.LastError());
    };
    mqttConnection->Publish("/publish/me/senpai", QOS::AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPubAck);

    // wait for publish
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]() { return published; });
    }

    mqttConnection.reset();

    ASSERT_FALSE(mqttConnection);

    return AWS_ERROR_SUCCESS;
}

AWS_TEST_CASE(IotConnectionDestructionWithPublish, s_TestIotConnectionDestructionWithPublish)

#endif // !BYO_CRYPTO
