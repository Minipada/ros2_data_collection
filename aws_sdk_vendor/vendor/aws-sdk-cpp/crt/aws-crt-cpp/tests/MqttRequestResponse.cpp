/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/environment.h>
#include <aws/crt/Api.h>
#include <aws/crt/JsonObject.h>
#include <aws/crt/Types.h>
#include <aws/crt/UUID.h>
#include <aws/crt/mqtt/Mqtt5Packets.h>
#include <aws/iot/Mqtt5Client.h>
#include <aws/iot/MqttCommon.h>
#include <aws/iot/MqttRequestResponseClient.h>
#include <aws/testing/aws_test_harness.h>

#include <atomic>
#include <utility>

enum ProtocolType
{
    Mqtt5,
    Mqtt311
};

AWS_STATIC_STRING_FROM_LITERAL(s_rrEnvVariableHost, "AWS_TEST_MQTT5_IOT_CORE_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_rrEnvVariableCertificatePath, "AWS_TEST_MQTT5_IOT_CORE_RSA_CERT");
AWS_STATIC_STRING_FROM_LITERAL(s_rrEnvVariablePrivateKeyPath, "AWS_TEST_MQTT5_IOT_CORE_RSA_KEY");

struct TestState;

struct ResponseTracker
{
    ResponseTracker() : state(nullptr), topic(), payload(), errorCode(AWS_ERROR_SUCCESS), complete(false) {}

    TestState *state;
    Aws::Crt::String topic;
    Aws::Crt::String payload;
    int errorCode;
    bool complete;
};

struct TestPublishEvent
{
    Aws::Crt::String topic;
    Aws::Crt::String payload;
    Aws::Crt::Optional<Aws::Crt::String> contentType;
    Aws::Crt::Optional<Aws::Crt::Vector<Aws::Crt::Mqtt5::UserProperty>> userProperties;
    Aws::Crt::Optional<uint32_t> messageExpiryIntervalSeconds;
};

struct TestState
{
    TestState(Aws::Crt::Allocator *allocator) : allocator(allocator) {}

    Aws::Crt::Allocator *allocator;

    std::mutex lock;
    std::condition_variable signal;

    bool connected = false;

    Aws::Crt::Vector<std::shared_ptr<ResponseTracker>> responseTrackers;

    Aws::Crt::Vector<Aws::Iot::RequestResponse::SubscriptionStatusEvent> subscriptionStatusEvents;
    Aws::Crt::Vector<TestPublishEvent> incomingPublishEvents;
};

static void s_waitForConnected(struct TestState *state)
{
    {
        std::unique_lock<std::mutex> lock(state->lock);
        state->signal.wait(lock, [state] { return state->connected; });
    }
}

static void s_updateConnected(struct TestState *state, bool connected)
{
    {
        std::lock_guard<std::mutex> lock(state->lock);
        state->connected = connected;
    }
    state->signal.notify_one();
}

static std::shared_ptr<ResponseTracker> s_addResponseTracker(TestState *state)
{
    std::shared_ptr<ResponseTracker> tracker = Aws::Crt::MakeShared<ResponseTracker>(state->allocator);
    tracker->state = state;
    {
        std::unique_lock<std::mutex> lock(state->lock);
        state->responseTrackers.push_back(tracker);
    }

    return tracker;
}

static void s_waitForResponse(ResponseTracker *responseTracker)
{
    TestState *state = responseTracker->state;
    {
        std::unique_lock<std::mutex> lock(state->lock);
        state->signal.wait(lock, [responseTracker] { return responseTracker->complete; });
    }
}

static void s_completeResponseWithSuccess(
    ResponseTracker *responseTracker,
    Aws::Crt::ByteCursor topic,
    Aws::Crt::ByteCursor payload)
{
    TestState *state = responseTracker->state;
    {
        std::unique_lock<std::mutex> lock(state->lock);
        responseTracker->topic = Aws::Crt::String((const char *)topic.ptr, topic.len);
        responseTracker->payload.assign((const uint8_t *)payload.ptr, (const uint8_t *)payload.ptr + payload.len);
        responseTracker->complete = true;
    }
    state->signal.notify_one();
}

static void s_completeResponseWithError(ResponseTracker *responseTracker, int errorCode)
{
    TestState *state = responseTracker->state;
    {
        std::unique_lock<std::mutex> lock(state->lock);
        responseTracker->errorCode = errorCode;
        responseTracker->complete = true;
    }
    state->signal.notify_one();
}

static void s_onRequestComplete(Aws::Iot::RequestResponse::UnmodeledResult &&result, ResponseTracker *responseTracker)
{
    if (result.IsSuccess())
    {
        const auto &response = result.GetResponse();
        s_completeResponseWithSuccess(responseTracker, response.GetTopic(), response.GetPayload());
    }
    else
    {
        s_completeResponseWithError(responseTracker, result.GetError());
    }
}

static void s_onSubscriptionStatusEvent(Aws::Iot::RequestResponse::SubscriptionStatusEvent &&event, TestState *state)
{
    {
        std::unique_lock<std::mutex> lock(state->lock);
        state->subscriptionStatusEvents.push_back(event);
    }
    state->signal.notify_one();
}

static void s_waitForSubscriptionStatusEvent(
    TestState *state,
    Aws::Iot::RequestResponse::SubscriptionStatusEventType type,
    int errorCode)
{
    {
        std::unique_lock<std::mutex> lock(state->lock);
        state->signal.wait(
            lock,
            [=]()
            {
                return std::any_of(
                    state->subscriptionStatusEvents.cbegin(),
                    state->subscriptionStatusEvents.cend(),
                    [=](const Aws::Iot::RequestResponse::SubscriptionStatusEvent &event)
                    { return event.GetType() == type && event.GetErrorCode() == errorCode; });
            });
    }
}

static void s_onIncomingPublishEvent(Aws::Iot::RequestResponse::IncomingPublishEvent &&event, TestState *state)
{
    {
        std::unique_lock<std::mutex> lock(state->lock);

        auto topicCursor = event.GetTopic();
        Aws::Crt::String topicAsString((const char *)topicCursor.ptr, topicCursor.len);

        auto payloadCursor = event.GetPayload();
        Aws::Crt::String payloadAsString((const char *)payloadCursor.ptr, payloadCursor.len);

        auto contentTypeCursor = event.GetContentType();
        Aws::Crt::Optional<Aws::Crt::String> contentTypeAsString;
        if (contentTypeCursor)
        {
            contentTypeAsString = Aws::Crt::String((const char *)contentTypeCursor->ptr, contentTypeCursor->len);
        }

        const auto &userPropertiesView = event.GetUserProperties();
        Aws::Crt::Vector<Aws::Crt::Mqtt5::UserProperty> userProperties;
        if (userPropertiesView)
        {
            std::transform(
                std::begin(*userPropertiesView),
                std::end(*userPropertiesView),
                std::back_inserter(userProperties),
                [](const Aws::Iot::RequestResponse::UserPropertyView &userPropertyView)
                {
                    return Aws::Crt::Mqtt5::UserProperty(
                        Aws::Crt::String((const char *)userPropertyView.m_name.ptr, userPropertyView.m_name.len),
                        Aws::Crt::String((const char *)userPropertyView.m_value.ptr, userPropertyView.m_value.len));
                });
        }

        auto messageExpiryIntervalSeconds = event.GetMessageExpiryIntervalSeconds();

        state->incomingPublishEvents.push_back(
            {std::move(topicAsString),
             std::move(payloadAsString),
             std::move(contentTypeAsString),
             std::move(userProperties),
             std::move(messageExpiryIntervalSeconds)});
    }
    state->signal.notify_one();
}

static void s_waitForIncomingPublishWithPredicate(
    TestState *state,
    const std::function<bool(const TestPublishEvent &)> &predicate)
{
    {
        std::unique_lock<std::mutex> lock(state->lock);
        state->signal.wait(
            lock,
            [=]()
            {
                return std::any_of(
                    state->incomingPublishEvents.cbegin(),
                    state->incomingPublishEvents.cend(),
                    [=](const TestPublishEvent &publishEvent) { return predicate(publishEvent); });
            });
    }
}

struct TestContext
{
    std::shared_ptr<Aws::Iot::RequestResponse::IMqttRequestResponseClient> client;
    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> protocolClient5;
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> protocolClient311;
};

static void s_startProtocolClient(TestContext &context)
{
    if (context.protocolClient5)
    {
        context.protocolClient5->Start();
    }
    else
    {
        auto uuid = Aws::Crt::UUID().ToString();
        context.protocolClient311->Connect(uuid.c_str(), true, 30, 15000, 5000);
    }
}

static TestContext s_CreateClient(
    Aws::Crt::Allocator *allocator,
    ProtocolType protocol,
    struct TestState *state,
    Aws::Iot::RequestResponse::RequestResponseClientOptions *options = NULL)
{
    TestContext context;
    Aws::Iot::RequestResponse::RequestResponseClientOptions finalOptions;

    struct aws_string *host = NULL;
    struct aws_string *certificatePath = NULL;
    struct aws_string *privateKeyPath = NULL;

    if (aws_get_environment_value(allocator, s_rrEnvVariableHost, &host) || !host)
    {
        goto done;
    }

    if (aws_get_environment_value(allocator, s_rrEnvVariableCertificatePath, &certificatePath) || !certificatePath)
    {
        goto done;
    }

    if (aws_get_environment_value(allocator, s_rrEnvVariablePrivateKeyPath, &privateKeyPath) || !privateKeyPath)
    {
        goto done;
    }

    if (options != nullptr)
    {
        finalOptions = *options;
    }
    else
    {
        finalOptions.WithMaxRequestResponseSubscriptions(4);
        finalOptions.WithMaxStreamingSubscriptions(2);
        finalOptions.WithOperationTimeoutInSeconds(30);
    }

    if (protocol == ProtocolType::Mqtt5)
    {
        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
            aws_string_c_str(certificatePath), aws_string_c_str(privateKeyPath), allocator);
        Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        Aws::Crt::Mqtt5::Mqtt5ClientOptions mqtt5Options(allocator);
        mqtt5Options.WithHostName(Aws::Crt::String(aws_string_c_str(host)));
        mqtt5Options.WithPort(8883);
        mqtt5Options.WithTlsConnectionOptions(tlsContext.NewConnectionOptions());

        mqtt5Options.WithClientConnectionSuccessCallback(
            [state](const Aws::Crt::Mqtt5::OnConnectionSuccessEventData &event)
            {
                (void)event;
                s_updateConnected(state, true);
            });
        mqtt5Options.WithClientDisconnectionCallback(
            [state](const Aws::Crt::Mqtt5::OnDisconnectionEventData &event)
            {
                (void)event;
                s_updateConnected(state, false);
            });

        context.protocolClient5 = Aws::Crt::Mqtt5::Mqtt5Client::NewMqtt5Client(mqtt5Options, allocator);
        context.client = Aws::Iot::RequestResponse::NewClientFrom5(*context.protocolClient5, finalOptions, allocator);
    }
    else
    {
        Aws::Crt::Io::TlsContextOptions tlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
            aws_string_c_str(certificatePath), aws_string_c_str(privateKeyPath), allocator);
        Aws::Crt::Io::TlsContext tlsContext(tlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

        Aws::Crt::Io::SocketOptions socketOptions;
        socketOptions.SetConnectTimeoutMs(10000);

        Aws::Crt::Mqtt::MqttClient client;
        context.protocolClient311 =
            client.NewConnection(aws_string_c_str(host), 8883, socketOptions, tlsContext, false);

        context.protocolClient311->OnConnectionSuccess =
            [state](Aws::Crt::Mqtt::MqttConnection &connection, Aws::Crt::Mqtt::OnConnectionSuccessData *callbackData)
        {
            (void)connection;
            (void)callbackData;
            s_updateConnected(state, true);
        };
        context.protocolClient311->OnDisconnect = [state](Aws::Crt::Mqtt::MqttConnection &connection)
        {
            (void)connection;
            s_updateConnected(state, false);
        };

        context.client =
            Aws::Iot::RequestResponse::NewClientFrom311(*context.protocolClient311, finalOptions, allocator);
    }

done:

    aws_string_destroy(host);
    aws_string_destroy(certificatePath);
    aws_string_destroy(privateKeyPath);

    return context;
}

void s_publishToProtocolClient(
    TestContext &context,
    Aws::Crt::String topic,
    Aws::Crt::String payload,
    const Aws::Crt::Optional<Aws::Crt::String> &contentType,
    const Aws::Crt::Optional<Aws::Crt::Vector<Aws::Crt::Mqtt5::UserProperty>> &userProperties,
    const Aws::Crt::Optional<uint32_t> &messageExpiryIntervalSeconds,
    Aws::Crt::Allocator *allocator)
{
    if (context.protocolClient5)
    {
        auto packet = Aws::Crt::MakeShared<Aws::Crt::Mqtt5::PublishPacket>(
            allocator,
            topic,
            Aws::Crt::ByteCursorFromString(payload),
            Aws::Crt::Mqtt5::QOS::AWS_MQTT5_QOS_AT_MOST_ONCE);
        if (contentType)
        {
            Aws::Crt::ByteCursor contentTypeCursor{contentType->length(), (uint8_t *)contentType->c_str()};
            packet->WithContentType(contentTypeCursor);
        }
        if (userProperties)
        {
            packet->WithUserProperties(*userProperties);
        }
        if (messageExpiryIntervalSeconds)
        {
            packet->WithMessageExpiryIntervalSec(*messageExpiryIntervalSeconds);
        }
        context.protocolClient5->Publish(packet);
    }
    else
    {
        Aws::Crt::ByteCursor payloadCursor = Aws::Crt::ByteCursorFromString(payload);
        Aws::Crt::ByteBuf payloadBuffer;
        aws_byte_buf_init_copy_from_cursor(&payloadBuffer, allocator, payloadCursor);

        context.protocolClient311->Publish(
            topic.c_str(),
            AWS_MQTT_QOS_AT_MOST_ONCE,
            false,
            payloadBuffer,
            [](Aws::Crt::Mqtt::MqttConnection &connection, uint16_t packetId, int errorCode)
            {
                (void)connection;
                (void)packetId;
                (void)errorCode;
            });
        aws_byte_buf_clean_up(&payloadBuffer);
    }
}

static int s_MqttRequestResponse_CreateDestroy5(Aws::Crt::Allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle handle;
        struct TestState state(allocator);

        auto context = s_CreateClient(allocator, ProtocolType::Mqtt5, &state);
        if (!context.client)
        {
            return AWS_OP_SKIP;
        }
    }

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(MqttRequestResponse_CreateDestroy5, s_MqttRequestResponse_CreateDestroy5)

static int s_MqttRequestResponse_CreateDestroy311(Aws::Crt::Allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle handle;
        struct TestState state(allocator);

        auto context = s_CreateClient(allocator, ProtocolType::Mqtt311, &state);
        if (!context.client)
        {
            return AWS_OP_SKIP;
        }
    }

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(MqttRequestResponse_CreateDestroy311, s_MqttRequestResponse_CreateDestroy311)

static int s_SubmitGetNamedShadowRejectedRequest(TestContext &context, TestState *state, bool useCorrelationToken)
{
    std::shared_ptr<ResponseTracker> tracker = s_addResponseTracker(state);
    ResponseTracker *rawResponseTracker = tracker.get();

    Aws::Crt::JsonObject jsonObject;
    auto uuid = Aws::Crt::UUID().ToString();
    jsonObject.WithString("clientToken", uuid);
    Aws::Crt::String payloadWithCorrelationToken = jsonObject.View().WriteCompact(true);

    auto shadowName = Aws::Crt::UUID().ToString();

    char subscriptionTopicFilter[256];
    snprintf(
        subscriptionTopicFilter,
        AWS_ARRAY_SIZE(subscriptionTopicFilter),
        "$aws/things/NoSuchThing/shadow/name/%s/get/+",
        shadowName.c_str());

    char acceptedTopic[256];
    snprintf(
        acceptedTopic,
        AWS_ARRAY_SIZE(acceptedTopic),
        "$aws/things/NoSuchThing/shadow/name/%s/get/accepted",
        shadowName.c_str());

    char rejectedTopic[256];
    snprintf(
        rejectedTopic,
        AWS_ARRAY_SIZE(rejectedTopic),
        "$aws/things/NoSuchThing/shadow/name/%s/get/rejected",
        shadowName.c_str());

    char publishTopic[256];
    snprintf(
        publishTopic, AWS_ARRAY_SIZE(publishTopic), "$aws/things/NoSuchThing/shadow/name/%s/get", shadowName.c_str());

    struct aws_mqtt_request_operation_options requestOptions;
    AWS_ZERO_STRUCT(requestOptions);

    struct aws_byte_cursor subscription_topic_filters[1] = {
        aws_byte_cursor_from_c_str(subscriptionTopicFilter),
    };
    requestOptions.subscription_topic_filters = subscription_topic_filters;
    requestOptions.subscription_topic_filter_count = 1;

    struct aws_mqtt_request_operation_response_path responsePaths[2];
    AWS_ZERO_STRUCT(responsePaths[0]);
    AWS_ZERO_STRUCT(responsePaths[1]);
    responsePaths[0].topic = aws_byte_cursor_from_c_str(acceptedTopic);
    responsePaths[1].topic = aws_byte_cursor_from_c_str(rejectedTopic);
    if (useCorrelationToken)
    {
        responsePaths[0].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        responsePaths[1].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        requestOptions.correlation_token = Aws::Crt::ByteCursorFromString(uuid);
        requestOptions.serialized_request = Aws::Crt::ByteCursorFromString(payloadWithCorrelationToken);
    }
    else
    {
        requestOptions.serialized_request = aws_byte_cursor_from_c_str("{}");
    }

    requestOptions.response_paths = responsePaths;
    requestOptions.response_path_count = 2;
    requestOptions.publish_topic = aws_byte_cursor_from_c_str(publishTopic);
    requestOptions.user_data = rawResponseTracker;

    int result = context.client->SubmitRequest(
        requestOptions,
        [rawResponseTracker](Aws::Iot::RequestResponse::UnmodeledResult &&result)
        { s_onRequestComplete(std::move(result), rawResponseTracker); });
    ASSERT_INT_EQUALS(AWS_OP_SUCCESS, result);

    s_waitForResponse(rawResponseTracker);

    {
        std::lock_guard<std::mutex> lock(state->lock);
        ASSERT_INT_EQUALS(AWS_ERROR_SUCCESS, tracker->errorCode);

        ASSERT_TRUE(
            tracker->topic == Aws::Crt::String((const char *)responsePaths[1].topic.ptr, responsePaths[1].topic.len));
        ASSERT_TRUE(tracker->payload.find("No shadow exists with name") != std::string::npos);
    }

    return AWS_OP_SUCCESS;
}

static int s_doGetNamedShadowSuccessRejectedTest(
    Aws::Crt::Allocator *allocator,
    ProtocolType protocol,
    bool useCorrelationToken)
{

    Aws::Crt::ApiHandle handle;
    struct TestState state(allocator);

    auto context = s_CreateClient(allocator, protocol, &state);
    if (!context.client)
    {
        return AWS_OP_SKIP;
    }

    s_startProtocolClient(context);
    s_waitForConnected(&state);

    return s_SubmitGetNamedShadowRejectedRequest(context, &state, useCorrelationToken);
}

static int s_MqttRequestResponse_GetNamedShadowSuccessRejected311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowSuccessRejectedTest(allocator, ProtocolType::Mqtt311, true);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowSuccessRejected311,
    s_MqttRequestResponse_GetNamedShadowSuccessRejected311)

static int s_MqttRequestResponse_GetNamedShadowSuccessRejected5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowSuccessRejectedTest(allocator, ProtocolType::Mqtt5, true);
}
AWS_TEST_CASE(MqttRequestResponse_GetNamedShadowSuccessRejected5, s_MqttRequestResponse_GetNamedShadowSuccessRejected5)

static int s_MqttRequestResponse_GetNamedShadowSuccessRejectedNoCorrelationToken311(
    Aws::Crt::Allocator *allocator,
    void *)
{
    return s_doGetNamedShadowSuccessRejectedTest(allocator, ProtocolType::Mqtt311, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowSuccessRejectedNoCorrelationToken311,
    s_MqttRequestResponse_GetNamedShadowSuccessRejectedNoCorrelationToken311)

static int s_MqttRequestResponse_GetNamedShadowSuccessRejectedNoCorrelationToken5(
    Aws::Crt::Allocator *allocator,
    void *)
{
    return s_doGetNamedShadowSuccessRejectedTest(allocator, ProtocolType::Mqtt5, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowSuccessRejectedNoCorrelationToken5,
    s_MqttRequestResponse_GetNamedShadowSuccessRejectedNoCorrelationToken5)

static int s_SubmitUpdateNamedShadowAcceptedRequest(TestContext &context, TestState *state, bool useCorrelationToken)
{
    std::shared_ptr<ResponseTracker> tracker = s_addResponseTracker(state);
    ResponseTracker *rawResponseTracker = tracker.get();

    auto shadowName = Aws::Crt::UUID().ToString();
    char acceptedTopic[128];
    snprintf(
        acceptedTopic,
        AWS_ARRAY_SIZE(acceptedTopic),
        "$aws/things/NoSuchThing/shadow/name/%s/update/accepted",
        shadowName.c_str());
    char rejectedTopic[128];
    snprintf(
        rejectedTopic,
        AWS_ARRAY_SIZE(rejectedTopic),
        "$aws/things/NoSuchThing/shadow/name/%s/update/rejected",
        shadowName.c_str());

    auto clientToken = Aws::Crt::UUID().ToString();
    auto stateToken = Aws::Crt::UUID().ToString();
    char desiredState[256];
    snprintf(desiredState, AWS_ARRAY_SIZE(desiredState), "{\"magic\":\"%s\"}", stateToken.c_str());

    char payload[512];

    struct aws_mqtt_request_operation_options requestOptions;
    AWS_ZERO_STRUCT(requestOptions);

    struct aws_byte_cursor subscription_topic_filters[2] = {
        aws_byte_cursor_from_c_str(acceptedTopic),
        aws_byte_cursor_from_c_str(rejectedTopic),
    };
    requestOptions.subscription_topic_filters = subscription_topic_filters;
    requestOptions.subscription_topic_filter_count = 2;

    struct aws_mqtt_request_operation_response_path responsePaths[2];
    AWS_ZERO_STRUCT(responsePaths[0]);
    AWS_ZERO_STRUCT(responsePaths[1]);
    responsePaths[0].topic = aws_byte_cursor_from_c_str(acceptedTopic);
    responsePaths[1].topic = aws_byte_cursor_from_c_str(rejectedTopic);
    if (useCorrelationToken)
    {
        responsePaths[0].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        responsePaths[1].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        requestOptions.correlation_token = Aws::Crt::ByteCursorFromString(clientToken);
        snprintf(
            payload,
            AWS_ARRAY_SIZE(payload),
            "{\"clientToken\":\"%s\",\"state\":{\"desired\":%s}}",
            clientToken.c_str(),
            desiredState);
    }
    else
    {
        snprintf(payload, AWS_ARRAY_SIZE(payload), "{\"state\":{\"desired\":%s}}", desiredState);
    }

    requestOptions.serialized_request = aws_byte_cursor_from_c_str(payload);

    requestOptions.response_paths = responsePaths;
    requestOptions.response_path_count = 2;

    char publishTopic[128];
    snprintf(
        publishTopic,
        AWS_ARRAY_SIZE(publishTopic),
        "$aws/things/NoSuchThing/shadow/name/%s/update",
        shadowName.c_str());

    requestOptions.publish_topic = aws_byte_cursor_from_c_str(publishTopic);
    requestOptions.user_data = rawResponseTracker;

    int result = context.client->SubmitRequest(
        requestOptions,
        [rawResponseTracker](Aws::Iot::RequestResponse::UnmodeledResult &&result)
        { s_onRequestComplete(std::move(result), rawResponseTracker); });
    ASSERT_INT_EQUALS(AWS_OP_SUCCESS, result);

    s_waitForResponse(rawResponseTracker);

    {
        std::lock_guard<std::mutex> lock(state->lock);
        ASSERT_INT_EQUALS(AWS_ERROR_SUCCESS, tracker->errorCode);

        ASSERT_TRUE(
            tracker->topic == Aws::Crt::String((const char *)responsePaths[0].topic.ptr, responsePaths[0].topic.len));
        ASSERT_TRUE(tracker->payload.length() > 0);
    }

    return AWS_OP_SUCCESS;
}

static int s_doUpdateNamedShadowSuccessAcceptedTest(
    Aws::Crt::Allocator *allocator,
    ProtocolType protocol,
    bool useCorrelationToken)
{

    Aws::Crt::ApiHandle handle;
    struct TestState state(allocator);

    auto context = s_CreateClient(allocator, protocol, &state);
    if (!context.client)
    {
        return AWS_OP_SKIP;
    }

    s_startProtocolClient(context);
    s_waitForConnected(&state);

    return s_SubmitUpdateNamedShadowAcceptedRequest(context, &state, useCorrelationToken);
}

static int s_MqttRequestResponse_UpdateNamedShadowSuccessAccepted311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doUpdateNamedShadowSuccessAcceptedTest(allocator, ProtocolType::Mqtt311, true);
}
AWS_TEST_CASE(
    MqttRequestResponse_UpdateNamedShadowSuccessAccepted311,
    s_MqttRequestResponse_UpdateNamedShadowSuccessAccepted311)

static int s_MqttRequestResponse_UpdateNamedShadowSuccessAccepted5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doUpdateNamedShadowSuccessAcceptedTest(allocator, ProtocolType::Mqtt5, true);
}
AWS_TEST_CASE(
    MqttRequestResponse_UpdateNamedShadowSuccessAccepted5,
    s_MqttRequestResponse_UpdateNamedShadowSuccessAccepted5)

static int s_MqttRequestResponse_UpdateNamedShadowSuccessAcceptedNoCorrelationToken311(
    Aws::Crt::Allocator *allocator,
    void *)
{
    return s_doUpdateNamedShadowSuccessAcceptedTest(allocator, ProtocolType::Mqtt311, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_UpdateNamedShadowSuccessAcceptedNoCorrelationToken311,
    s_MqttRequestResponse_UpdateNamedShadowSuccessAcceptedNoCorrelationToken311)

static int s_MqttRequestResponse_UpdateNamedShadowSuccessAcceptedNoCorrelationToken5(
    Aws::Crt::Allocator *allocator,
    void *)
{
    return s_doUpdateNamedShadowSuccessAcceptedTest(allocator, ProtocolType::Mqtt5, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_UpdateNamedShadowSuccessAcceptedNoCorrelationToken5,
    s_MqttRequestResponse_UpdateNamedShadowSuccessAcceptedNoCorrelationToken5)

static int s_SubmitGetNamedShadowTimeoutRequest(TestContext &context, TestState *state, bool useCorrelationToken)
{
    std::shared_ptr<ResponseTracker> tracker = s_addResponseTracker(state);
    ResponseTracker *rawResponseTracker = tracker.get();

    Aws::Crt::JsonObject jsonObject;
    auto uuid = Aws::Crt::UUID().ToString();
    jsonObject.WithString("clientToken", uuid);
    Aws::Crt::String payloadWithCorrelationToken = jsonObject.View().WriteCompact(true);

    auto shadowName = Aws::Crt::UUID().ToString();

    char subscriptionTopicFilter[256];
    snprintf(
        subscriptionTopicFilter,
        AWS_ARRAY_SIZE(subscriptionTopicFilter),
        "$aws/things/NoSuchThing/shadow/name/%s/get/+",
        shadowName.c_str());

    char acceptedTopic[256];
    snprintf(
        acceptedTopic,
        AWS_ARRAY_SIZE(acceptedTopic),
        "$aws/things/NoSuchThing/shadow/name/%s/get/accepted",
        shadowName.c_str());

    char rejectedTopic[256];
    snprintf(
        rejectedTopic,
        AWS_ARRAY_SIZE(rejectedTopic),
        "$aws/things/NoSuchThing/shadow/name/%s/get/rejected",
        shadowName.c_str());

    struct aws_mqtt_request_operation_options requestOptions;
    AWS_ZERO_STRUCT(requestOptions);

    struct aws_byte_cursor subscription_topic_filters[1] = {
        aws_byte_cursor_from_c_str(subscriptionTopicFilter),
    };
    requestOptions.subscription_topic_filters = subscription_topic_filters;
    requestOptions.subscription_topic_filter_count = 1;

    struct aws_mqtt_request_operation_response_path responsePaths[2];
    AWS_ZERO_STRUCT(responsePaths[0]);
    AWS_ZERO_STRUCT(responsePaths[1]);
    responsePaths[0].topic = aws_byte_cursor_from_c_str(acceptedTopic);
    responsePaths[1].topic = aws_byte_cursor_from_c_str(rejectedTopic);
    if (useCorrelationToken)
    {
        responsePaths[0].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        responsePaths[1].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        requestOptions.correlation_token = Aws::Crt::ByteCursorFromString(uuid);
        requestOptions.serialized_request = Aws::Crt::ByteCursorFromString(payloadWithCorrelationToken);
    }
    else
    {
        requestOptions.serialized_request = aws_byte_cursor_from_c_str("{}");
    }

    requestOptions.response_paths = responsePaths;
    requestOptions.response_path_count = 2;
    requestOptions.publish_topic = aws_byte_cursor_from_c_str("wrong/publish/topic");
    requestOptions.user_data = rawResponseTracker;

    int result = context.client->SubmitRequest(
        requestOptions,
        [rawResponseTracker](Aws::Iot::RequestResponse::UnmodeledResult &&result)
        { s_onRequestComplete(std::move(result), rawResponseTracker); });
    ASSERT_INT_EQUALS(AWS_OP_SUCCESS, result);

    s_waitForResponse(rawResponseTracker);

    {
        std::lock_guard<std::mutex> lock(state->lock);
        ASSERT_INT_EQUALS(AWS_ERROR_MQTT_REQUEST_RESPONSE_TIMEOUT, tracker->errorCode);
        ASSERT_TRUE(tracker->topic.empty());
        ASSERT_TRUE(tracker->payload.empty());
    }

    return AWS_OP_SUCCESS;
}

static int s_doGetNamedShadowTimeoutTest(
    Aws::Crt::Allocator *allocator,
    ProtocolType protocol,
    bool useCorrelationToken)
{
    Aws::Iot::RequestResponse::RequestResponseClientOptions clientOptions;
    clientOptions.WithMaxRequestResponseSubscriptions(4);
    clientOptions.WithMaxStreamingSubscriptions(2);
    clientOptions.WithOperationTimeoutInSeconds(3);

    Aws::Crt::ApiHandle handle;
    struct TestState state(allocator);

    auto context = s_CreateClient(allocator, protocol, &state, &clientOptions);
    if (!context.client)
    {
        return AWS_OP_SKIP;
    }

    s_startProtocolClient(context);
    s_waitForConnected(&state);

    return s_SubmitGetNamedShadowTimeoutRequest(context, &state, useCorrelationToken);
}

static int s_MqttRequestResponse_GetNamedShadowTimeout311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowTimeoutTest(allocator, ProtocolType::Mqtt311, true);
}
AWS_TEST_CASE(MqttRequestResponse_GetNamedShadowTimeout311, s_MqttRequestResponse_GetNamedShadowTimeout311)

static int s_MqttRequestResponse_GetNamedShadowTimeout5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowTimeoutTest(allocator, ProtocolType::Mqtt5, true);
}
AWS_TEST_CASE(MqttRequestResponse_GetNamedShadowTimeout5, s_MqttRequestResponse_GetNamedShadowTimeout5)

static int s_MqttRequestResponse_GetNamedShadowTimeoutNoCorrelationToken311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowTimeoutTest(allocator, ProtocolType::Mqtt311, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowTimeoutNoCorrelationToken311,
    s_MqttRequestResponse_GetNamedShadowTimeoutNoCorrelationToken311)

static int s_MqttRequestResponse_GetNamedShadowTimeoutNoCorrelationToken5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowTimeoutTest(allocator, ProtocolType::Mqtt5, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowTimeoutNoCorrelationToken5,
    s_MqttRequestResponse_GetNamedShadowTimeoutNoCorrelationToken5)

static int s_SubmitGetNamedShadowFailureOnCloseRequest(TestContext &context, TestState *state, bool useCorrelationToken)
{
    std::shared_ptr<ResponseTracker> tracker = s_addResponseTracker(state);
    ResponseTracker *rawResponseTracker = tracker.get();

    Aws::Crt::JsonObject jsonObject;
    auto uuid = Aws::Crt::UUID().ToString();
    jsonObject.WithString("clientToken", uuid);
    Aws::Crt::String payloadWithCorrelationToken = jsonObject.View().WriteCompact(true);

    struct aws_mqtt_request_operation_options requestOptions;
    AWS_ZERO_STRUCT(requestOptions);

    struct aws_byte_cursor subscription_topic_filters[1] = {
        aws_byte_cursor_from_c_str("$aws/things/NoSuchThing/shadow/name/Derp/get/+"),
    };
    requestOptions.subscription_topic_filters = subscription_topic_filters;
    requestOptions.subscription_topic_filter_count = 1;

    struct aws_mqtt_request_operation_response_path responsePaths[2];
    AWS_ZERO_STRUCT(responsePaths[0]);
    AWS_ZERO_STRUCT(responsePaths[1]);
    responsePaths[0].topic = aws_byte_cursor_from_c_str("$aws/things/NoSuchThing/shadow/name/Derp/get/accepted");
    responsePaths[1].topic = aws_byte_cursor_from_c_str("$aws/things/NoSuchThing/shadow/name/Derp/get/rejected");
    if (useCorrelationToken)
    {
        responsePaths[0].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        responsePaths[1].correlation_token_json_path = aws_byte_cursor_from_c_str("clientToken");
        requestOptions.correlation_token = Aws::Crt::ByteCursorFromString(uuid);
        requestOptions.serialized_request = Aws::Crt::ByteCursorFromString(payloadWithCorrelationToken);
    }
    else
    {
        requestOptions.serialized_request = aws_byte_cursor_from_c_str("{}");
    }

    requestOptions.response_paths = responsePaths;
    requestOptions.response_path_count = 2;
    requestOptions.publish_topic = aws_byte_cursor_from_c_str("wrong/publish/topic");
    requestOptions.user_data = rawResponseTracker;

    int result = context.client->SubmitRequest(
        requestOptions,
        [rawResponseTracker](Aws::Iot::RequestResponse::UnmodeledResult &&result)
        { s_onRequestComplete(std::move(result), rawResponseTracker); });
    ASSERT_INT_EQUALS(AWS_OP_SUCCESS, result);

    context.client = nullptr;

    s_waitForResponse(rawResponseTracker);

    {
        std::lock_guard<std::mutex> lock(state->lock);
        ASSERT_INT_EQUALS(AWS_ERROR_MQTT_REQUEST_RESPONSE_CLIENT_SHUT_DOWN, tracker->errorCode);
        ASSERT_TRUE(tracker->topic.empty());
        ASSERT_TRUE(tracker->payload.empty());
    }

    return AWS_OP_SUCCESS;
}

static int s_doGetNamedShadowFailureOnClosedTest(
    Aws::Crt::Allocator *allocator,
    ProtocolType protocol,
    bool useCorrelationToken)
{
    Aws::Crt::ApiHandle handle;
    struct TestState state(allocator);

    auto context = s_CreateClient(allocator, protocol, &state);
    if (!context.client)
    {
        return AWS_OP_SKIP;
    }

    s_startProtocolClient(context);
    s_waitForConnected(&state);

    return s_SubmitGetNamedShadowFailureOnCloseRequest(context, &state, useCorrelationToken);
}

static int s_MqttRequestResponse_GetNamedShadowFailureOnClose311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowFailureOnClosedTest(allocator, ProtocolType::Mqtt311, true);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowFailureOnClose311,
    s_MqttRequestResponse_GetNamedShadowFailureOnClose311)

static int s_MqttRequestResponse_GetNamedShadowFailureOnClose5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowFailureOnClosedTest(allocator, ProtocolType::Mqtt5, true);
}
AWS_TEST_CASE(MqttRequestResponse_GetNamedShadowFailureOnClose5, s_MqttRequestResponse_GetNamedShadowFailureOnClose5)

static int s_MqttRequestResponse_GetNamedShadowFailureOnCloseNoCorrelationToken311(
    Aws::Crt::Allocator *allocator,
    void *)
{
    return s_doGetNamedShadowFailureOnClosedTest(allocator, ProtocolType::Mqtt311, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowFailureOnCloseNoCorrelationToken311,
    s_MqttRequestResponse_GetNamedShadowFailureOnCloseNoCorrelationToken311)

static int s_MqttRequestResponse_GetNamedShadowFailureOnCloseNoCorrelationToken5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doGetNamedShadowFailureOnClosedTest(allocator, ProtocolType::Mqtt5, false);
}
AWS_TEST_CASE(
    MqttRequestResponse_GetNamedShadowFailureOnCloseNoCorrelationToken5,
    s_MqttRequestResponse_GetNamedShadowFailureOnCloseNoCorrelationToken5)

static std::shared_ptr<Aws::Iot::RequestResponse::IStreamingOperation> s_CreateValidStream(
    TestContext &context,
    TestState *state,
    const Aws::Crt::String &topicFilter)
{
    Aws::Iot::RequestResponse::StreamingOperationOptionsInternal config;

    config.subscriptionStatusEventHandler = [state](Aws::Iot::RequestResponse::SubscriptionStatusEvent &&event)
    { s_onSubscriptionStatusEvent(std::move(event), state); };
    config.incomingPublishEventHandler = [state](Aws::Iot::RequestResponse::IncomingPublishEvent &&event)
    { s_onIncomingPublishEvent(std::move(event), state); };

    config.subscriptionTopicFilter = aws_byte_cursor_from_c_str(topicFilter.c_str());

    return context.client->CreateStream(config);
}

static int s_doShadowUpdatedStreamOpenCloseSuccessTest(Aws::Crt::Allocator *allocator, ProtocolType protocol)
{
    Aws::Crt::ApiHandle handle;
    struct TestState state(allocator);

    auto context = s_CreateClient(allocator, protocol, &state);
    if (!context.client)
    {
        return AWS_OP_SKIP;
    }

    s_startProtocolClient(context);
    s_waitForConnected(&state);

    auto uuid = Aws::Crt::UUID().ToString();
    auto stream = s_CreateValidStream(context, &state, uuid);
    ASSERT_NOT_NULL(stream.get());

    stream->Open();

    s_waitForSubscriptionStatusEvent(
        &state, Aws::Iot::RequestResponse::SubscriptionStatusEventType::SubscriptionEstablished, AWS_ERROR_SUCCESS);

    stream = nullptr;

    return AWS_OP_SUCCESS;
}

static int s_MqttRequestResponse_ShadowUpdatedStreamOpenCloseSuccess5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doShadowUpdatedStreamOpenCloseSuccessTest(allocator, ProtocolType::Mqtt5);
}
AWS_TEST_CASE(
    MqttRequestResponse_ShadowUpdatedStreamOpenCloseSuccess5,
    s_MqttRequestResponse_ShadowUpdatedStreamOpenCloseSuccess5)

static int s_MqttRequestResponse_ShadowUpdatedStreamOpenCloseSuccess311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doShadowUpdatedStreamOpenCloseSuccessTest(allocator, ProtocolType::Mqtt311);
}
AWS_TEST_CASE(
    MqttRequestResponse_ShadowUpdatedStreamOpenCloseSuccess311,
    s_MqttRequestResponse_ShadowUpdatedStreamOpenCloseSuccess311)

static int s_doShadowUpdatedStreamOpenCloseClientTest(Aws::Crt::Allocator *allocator, ProtocolType protocol)
{
    Aws::Crt::ApiHandle handle;
    std::shared_ptr<Aws::Iot::RequestResponse::IStreamingOperation> stream = nullptr;

    struct TestState state(allocator);

    auto context = s_CreateClient(allocator, protocol, &state);
    if (!context.client)
    {
        return AWS_OP_SKIP;
    }

    s_startProtocolClient(context);
    s_waitForConnected(&state);

    auto uuid = Aws::Crt::UUID().ToString();
    stream = s_CreateValidStream(context, &state, uuid);
    ASSERT_NOT_NULL(stream.get());

    stream->Open();

    s_waitForSubscriptionStatusEvent(
        &state, Aws::Iot::RequestResponse::SubscriptionStatusEventType::SubscriptionEstablished, AWS_ERROR_SUCCESS);

    // Close all the clients.  We should get a subscription halted event.
    context.client = nullptr;
    context.protocolClient311 = nullptr;
    context.protocolClient5 = nullptr;

    s_waitForSubscriptionStatusEvent(
        &state,
        Aws::Iot::RequestResponse::SubscriptionStatusEventType::SubscriptionHalted,
        AWS_ERROR_MQTT_REQUEST_RESPONSE_CLIENT_SHUT_DOWN);

    stream = nullptr;

    return AWS_OP_SUCCESS;
}

static int s_MqttRequestResponse_ShadowUpdatedStreamClientClosed5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doShadowUpdatedStreamOpenCloseClientTest(allocator, ProtocolType::Mqtt5);
}
AWS_TEST_CASE(
    MqttRequestResponse_ShadowUpdatedStreamClientClosed5,
    s_MqttRequestResponse_ShadowUpdatedStreamClientClosed5)

static int s_MqttRequestResponse_ShadowUpdatedStreamClientClosed311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doShadowUpdatedStreamOpenCloseClientTest(allocator, ProtocolType::Mqtt311);
}
AWS_TEST_CASE(
    MqttRequestResponse_ShadowUpdatedStreamClientClosed311,
    s_MqttRequestResponse_ShadowUpdatedStreamClientClosed311)

static const char *s_publishPayload = "IncomingPublish";

static int s_doShadowUpdatedStreamIncomingPublishTest(Aws::Crt::Allocator *allocator, ProtocolType protocol)
{
    Aws::Crt::ApiHandle handle;
    std::shared_ptr<Aws::Iot::RequestResponse::IStreamingOperation> stream = nullptr;

    struct TestState state(allocator);

    auto context = s_CreateClient(allocator, protocol, &state);
    if (!context.client)
    {
        return AWS_OP_SKIP;
    }

    s_startProtocolClient(context);
    s_waitForConnected(&state);

    auto uuid = Aws::Crt::UUID().ToString();
    stream = s_CreateValidStream(context, &state, uuid);
    ASSERT_NOT_NULL(stream.get());

    stream->Open();

    s_waitForSubscriptionStatusEvent(
        &state, Aws::Iot::RequestResponse::SubscriptionStatusEventType::SubscriptionEstablished, AWS_ERROR_SUCCESS);

    Aws::Crt::String contentType("application/json");
    Aws::Crt::Vector<Aws::Crt::Mqtt5::UserProperty> userProperties{
        {"property_1", "value_1"},
        {"property_2", "value_2"},
    };
    uint32_t messageExpiryIntervalSeconds = 8;
    s_publishToProtocolClient(
        context, uuid, s_publishPayload, contentType, userProperties, messageExpiryIntervalSeconds, allocator);

    s_waitForIncomingPublishWithPredicate(
        &state,
        [&context, &uuid, &contentType, &userProperties](const TestPublishEvent &publishEvent)
        {
            if (publishEvent.topic != uuid || publishEvent.payload != Aws::Crt::String(s_publishPayload))
            {
                return false;
            }
            if (context.protocolClient5)
            {
                if (!publishEvent.contentType || *publishEvent.contentType != contentType)
                {
                    return false;
                }
                if (!publishEvent.userProperties || publishEvent.userProperties->size() != userProperties.size() ||
                    *publishEvent.userProperties != userProperties)
                {
                    return false;
                }
                /* We can't check for the exact value here as it'll be decremented by the server part. */
                if (!publishEvent.messageExpiryIntervalSeconds)
                {
                    return false;
                }
            }

            return true;
        });

    return AWS_OP_SUCCESS;
}

static int s_MqttRequestResponse_ShadowUpdatedStreamIncomingPublishSuccess5(Aws::Crt::Allocator *allocator, void *)
{
    return s_doShadowUpdatedStreamIncomingPublishTest(allocator, ProtocolType::Mqtt5);
}
AWS_TEST_CASE(
    MqttRequestResponse_ShadowUpdatedStreamIncomingPublishSuccess5,
    s_MqttRequestResponse_ShadowUpdatedStreamIncomingPublishSuccess5)

static int s_MqttRequestResponse_ShadowUpdatedStreamIncomingPublishSuccess311(Aws::Crt::Allocator *allocator, void *)
{
    return s_doShadowUpdatedStreamIncomingPublishTest(allocator, ProtocolType::Mqtt311);
}
AWS_TEST_CASE(
    MqttRequestResponse_ShadowUpdatedStreamIncomingPublishSuccess311,
    s_MqttRequestResponse_ShadowUpdatedStreamIncomingPublishSuccess311)
