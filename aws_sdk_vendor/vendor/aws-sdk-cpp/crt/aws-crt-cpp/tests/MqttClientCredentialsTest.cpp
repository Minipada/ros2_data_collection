/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>

#include <aws/common/common.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>
#include <aws/crt/UUID.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/crt/io/Pkcs11.h>
#include <aws/iot/MqttClient.h>
#include <aws/iot/MqttCommon.h>

#include <aws/testing/aws_test_harness.h>
#include <utility>

#if !BYO_CRYPTO

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_hostname, "AWS_TEST_MQTT311_IOT_CORE_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_region, "AWS_TEST_MQTT311_IOT_CORE_REGION");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_nosign_custom_auth_name,
    "AWS_TEST_MQTT311_IOT_CORE_NO_SIGNING_AUTHORIZER_NAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_nosign_custom_auth_username,
    "AWS_TEST_MQTT311_IOT_CORE_NO_SIGNING_AUTHORIZER_USERNAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_nosign_custom_auth_password,
    "AWS_TEST_MQTT311_IOT_CORE_NO_SIGNING_AUTHORIZER_PASSWORD");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_sign_custom_auth_name,
    "AWS_TEST_MQTT311_IOT_CORE_SIGNING_AUTHORIZER_NAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_sign_custom_auth_username,
    "AWS_TEST_MQTT311_IOT_CORE_SIGNING_AUTHORIZER_USERNAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_sign_custom_auth_password,
    "AWS_TEST_MQTT311_IOT_CORE_SIGNING_AUTHORIZER_PASSWORD");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_sign_custom_auth_tokenvalue,
    "AWS_TEST_MQTT311_IOT_CORE_SIGNING_AUTHORIZER_TOKEN");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_sign_custom_auth_tokenkey,
    "AWS_TEST_MQTT311_IOT_CORE_SIGNING_AUTHORIZER_TOKEN_KEY_NAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_sign_custom_auth_tokensignature,
    "AWS_TEST_MQTT311_IOT_CORE_SIGNING_AUTHORIZER_TOKEN_SIGNATURE");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_sign_custom_auth_tokensignature_unencoded,
    "AWS_TEST_MQTT311_IOT_CORE_SIGNING_AUTHORIZER_TOKEN_SIGNATURE_UNENCODED");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_pkcs11_lib, "AWS_TEST_PKCS11_LIB");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_pkcs11_token_label, "AWS_TEST_PKCS11_TOKEN_LABEL");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_pkcs11_pin, "AWS_TEST_PKCS11_PIN");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_pkcs11_private_key_label, "AWS_TEST_PKCS11_PKEY_LABEL");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_pkcs11_cert, "AWS_TEST_PKCS11_CERT_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_pkcs11_ca, "AWS_TEST_PKCS11_CA_FILE");
// C++ specific PKCS11 check: only runs PKCS11 if 'DUSE_OPENSSL=ON' is set in the builder
AWS_STATIC_STRING_FROM_LITERAL(s_test_envName_iot_pkcs11_use_openssl, "AWS_TEST_PKCS11_USE_OPENSSL_SET");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_pkcs12_key, "AWS_TEST_MQTT311_IOT_CORE_PKCS12_KEY");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_pkcs12_key_password,
    "AWS_TEST_MQTT311_IOT_CORE_PKCS12_KEY_PASSWORD");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_windows_cert, "AWS_TEST_MQTT311_IOT_CORE_WINDOWS_CERT_STORE");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_credential_access_key,
    "AWS_TEST_MQTT311_ROLE_CREDENTIAL_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_credential_secret_access_key,
    "AWS_TEST_MQTT311_ROLE_CREDENTIAL_SECRET_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_credential_session_token,
    "AWS_TEST_MQTT311_ROLE_CREDENTIAL_SESSION_TOKEN");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt_cred_access_key, "AWS_ACCESS_KEY_ID");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt_cred_secret_access_key, "AWS_SECRET_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt_cred_session_token, "AWS_SESSION_TOKEN");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_cognito_endpoint, "AWS_TEST_MQTT311_COGNITO_ENDPOINT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_cognito_identity, "AWS_TEST_MQTT311_COGNITO_IDENTITY");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt311_test_envName_iot_profile_credentials,
    "AWS_TEST_MQTT311_IOT_PROFILE_CREDENTIALS");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_iot_profile_config, "AWS_TEST_MQTT311_IOT_PROFILE_CONFIG");

// Needed to return "success" instead of skip in Codebuild so it doesn't count as a failure
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt311_test_envName_codebuild, "CODEBUILD_BUILD_ID");

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

/*
 * Custom Auth (no signing) connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWithNoSigningCustomAuth(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *empty_string = aws_string_new_from_c_str(allocator, "");

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_nosign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_nosign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_nosign_custom_auth_password, &password);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        aws_string_destroy(empty_string);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder::NewDefaultBuilder();
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    clientConfigBuilder.WithCustomAuthorizer(
        aws_string_c_str(username),
        aws_string_c_str(authname),
        aws_string_c_str(empty_string),
        aws_string_c_str(password));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(empty_string);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWithNoSigningCustomAuth, s_TestIoTMqtt311ConnectWithNoSigningCustomAuth)

/*
 * Custom Auth (signing) connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWithSigningCustomAuth(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *signature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokensignature, &signature);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        aws_string_destroy(signature);
        aws_string_destroy(tokenKeyName);
        aws_string_destroy(tokenValue);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder::NewDefaultBuilder();
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    clientConfigBuilder.WithCustomAuthorizer(
        aws_string_c_str(username),
        aws_string_c_str(authname),
        aws_string_c_str(signature),
        aws_string_c_str(password),
        aws_string_c_str(tokenKeyName),
        aws_string_c_str(tokenValue));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(signature);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWithSigningCustomAuth, s_TestIoTMqtt311ConnectWithSigningCustomAuth)

/*
 * Custom Auth (signing with unencoded signature) connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWithSigningCustomAuthUnencoded(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *unencodedSignature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(
        allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokensignature_unencoded, &unencodedSignature);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        aws_string_destroy(unencodedSignature);
        aws_string_destroy(tokenKeyName);
        aws_string_destroy(tokenValue);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder::NewDefaultBuilder();
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    clientConfigBuilder.WithCustomAuthorizer(
        aws_string_c_str(username),
        aws_string_c_str(authname),
        aws_string_c_str(unencodedSignature),
        aws_string_c_str(password),
        aws_string_c_str(tokenKeyName),
        aws_string_c_str(tokenValue));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(unencodedSignature);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWithSigningCustomAuthUnencoded, s_TestIoTMqtt311ConnectWithSigningCustomAuthUnencoded)

/*
 * Custom Auth (signing) connect for MQTT311 - Websockets
 */
static int s_TestIoTMqtt311ConnectWithSigningCustomAuthWebsockets(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *signature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;
    struct aws_string *signingRegion = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokensignature, &signature);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_region, &signingRegion);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        aws_string_destroy(signature);
        aws_string_destroy(tokenKeyName);
        aws_string_destroy(tokenValue);
        aws_string_destroy(signingRegion);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(signingRegion), provider);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(websocketConfig);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    clientConfigBuilder.WithCustomAuthorizer(
        aws_string_c_str(username),
        aws_string_c_str(authname),
        aws_string_c_str(signature),
        aws_string_c_str(password),
        aws_string_c_str(tokenKeyName),
        aws_string_c_str(tokenValue));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(signature);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);
    aws_string_destroy(signingRegion);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWithSigningCustomAuthWebsockets, s_TestIoTMqtt311ConnectWithSigningCustomAuthWebsockets)

/*
 * Custom Auth (signing) connect for MQTT311 - Websockets
 */
static int s_TestIoTMqtt311ConnectWithSigningCustomAuthWebsocketsUnencoded(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *unencodedSignature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;
    struct aws_string *signingRegion = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(
        allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokensignature_unencoded, &unencodedSignature);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_region, &signingRegion);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        aws_string_destroy(unencodedSignature);
        aws_string_destroy(tokenKeyName);
        aws_string_destroy(tokenValue);
        aws_string_destroy(signingRegion);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(signingRegion), provider);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(websocketConfig);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    clientConfigBuilder.WithCustomAuthorizer(
        aws_string_c_str(username),
        aws_string_c_str(authname),
        aws_string_c_str(unencodedSignature),
        aws_string_c_str(password),
        aws_string_c_str(tokenKeyName),
        aws_string_c_str(tokenValue));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(unencodedSignature);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);
    aws_string_destroy(signingRegion);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    IoTMqtt311ConnectWithSigningCustomAuthWebsocketsUnencoded,
    s_TestIoTMqtt311ConnectWithSigningCustomAuthWebsocketsUnencoded)

/*
 * PKCS11 connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWithPKCS11(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *pkcs11_lib = NULL;
    struct aws_string *pkcs11_cert = NULL;
    struct aws_string *pkcs11_userPin = NULL;
    struct aws_string *pkcs11_tokenLabel = NULL;
    struct aws_string *pkcs11_privateKeyLabel = NULL;
    struct aws_string *pkcs11_ca = NULL;
    struct aws_string *pkcs11_use_openssl = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs11_lib, &pkcs11_lib);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs11_cert, &pkcs11_cert);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs11_pin, &pkcs11_userPin);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs11_token_label, &pkcs11_tokenLabel);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs11_private_key_label, &pkcs11_privateKeyLabel);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs11_ca, &pkcs11_ca);
    s_GetEnvVariable(allocator, s_test_envName_iot_pkcs11_use_openssl, &pkcs11_use_openssl);
    if (error != AWS_OP_SUCCESS || pkcs11_use_openssl == NULL)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(pkcs11_lib);
        aws_string_destroy(pkcs11_cert);
        aws_string_destroy(pkcs11_userPin);
        aws_string_destroy(pkcs11_tokenLabel);
        aws_string_destroy(pkcs11_privateKeyLabel);
        aws_string_destroy(pkcs11_ca);
        aws_string_destroy(pkcs11_use_openssl);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Io::Pkcs11Lib> pkcs11Lib = Aws::Crt::Io::Pkcs11Lib::Create(
        aws_string_c_str(pkcs11_lib), Aws::Crt::Io::Pkcs11Lib::InitializeFinalizeBehavior::Strict, allocator);
    ASSERT_TRUE(pkcs11Lib);
    Aws::Crt::Io::TlsContextPkcs11Options pkcs11Options(pkcs11Lib);
    pkcs11Options.SetCertificateFilePath(aws_string_c_str(pkcs11_cert));
    pkcs11Options.SetUserPin(aws_string_c_str(pkcs11_userPin));
    pkcs11Options.SetTokenLabel(aws_string_c_str(pkcs11_tokenLabel));
    pkcs11Options.SetPrivateKeyObjectLabel(aws_string_c_str(pkcs11_privateKeyLabel));

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(pkcs11Options, allocator);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    clientConfigBuilder.WithPortOverride(8883);
    clientConfigBuilder.WithCertificateAuthority(aws_string_c_str(pkcs11_ca));

    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(pkcs11_lib);
    aws_string_destroy(pkcs11_cert);
    aws_string_destroy(pkcs11_userPin);
    aws_string_destroy(pkcs11_tokenLabel);
    aws_string_destroy(pkcs11_privateKeyLabel);
    aws_string_destroy(pkcs11_ca);
    aws_string_destroy(pkcs11_use_openssl);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWithPKCS11, s_TestIoTMqtt311ConnectWithPKCS11)

/*
 * PKCS12 connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWithPKCS12(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *pkcs12_key = NULL;
    struct aws_string *pkcs12_password = NULL;
    struct aws_string *codebuild_buildID = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs12_key, &pkcs12_key);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_pkcs12_key_password, &pkcs12_password);
    s_GetEnvVariable(allocator, s_mqtt311_test_envName_codebuild, &codebuild_buildID);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(pkcs12_key);
        aws_string_destroy(pkcs12_password);

        // If this is in Codebuild, return as a 'success' even though it is a skip
        if (codebuild_buildID && aws_string_is_valid(codebuild_buildID))
        {
            aws_string_destroy(codebuild_buildID);
            return AWS_OP_SUCCESS;
        }
        aws_string_destroy(codebuild_buildID);

        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Iot::Pkcs12Options testPkcs12Options;
    testPkcs12Options.pkcs12_file = aws_string_c_str(pkcs12_key);
    testPkcs12Options.pkcs12_password = aws_string_c_str(pkcs12_password);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(testPkcs12Options, allocator);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(pkcs12_key);
    aws_string_destroy(pkcs12_password);
    aws_string_destroy(codebuild_buildID);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWithPKCS12, s_TestIoTMqtt311ConnectWithPKCS12)

/*
 * Windows Cert connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWithWindowsCert(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *windows_cert = NULL;
    struct aws_string *codebuild_buildID = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_windows_cert, &windows_cert);
    s_GetEnvVariable(allocator, s_mqtt311_test_envName_codebuild, &codebuild_buildID);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(windows_cert);

        // If in Codebuild, return as a 'success' even though it is a skip
        if (codebuild_buildID && aws_string_is_valid(codebuild_buildID))
        {
            aws_string_destroy(codebuild_buildID);
            return AWS_OP_SUCCESS;
        }
        aws_string_destroy(codebuild_buildID);

        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(aws_string_c_str(windows_cert));
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(windows_cert);
    aws_string_destroy(codebuild_buildID);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWithWindowsCert, s_TestIoTMqtt311ConnectWithWindowsCert)

/*
 * AWS Default Credentials Provider connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWSDefault(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_region, &region);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig config(aws_string_c_str(region), provider);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(config);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWSDefault, s_TestIoTMqtt311ConnectWSDefault)

/*
 * AWS Static Credentials Provider connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWSStatic(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *accessKeyId = NULL;
    struct aws_string *secretAccessKey = NULL;
    struct aws_string *sessionToken = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_credential_access_key, &accessKeyId);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_credential_secret_access_key, &secretAccessKey);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_credential_session_token, &sessionToken);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(accessKeyId);
        aws_string_destroy(secretAccessKey);
        aws_string_destroy(sessionToken);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    Aws::Crt::Auth::CredentialsProviderStaticConfig providerConfig;
    providerConfig.AccessKeyId = aws_byte_cursor_from_c_str(aws_string_c_str(accessKeyId));
    providerConfig.SecretAccessKey = aws_byte_cursor_from_c_str(aws_string_c_str(secretAccessKey));
    providerConfig.SessionToken = aws_byte_cursor_from_c_str(aws_string_c_str(sessionToken));
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderStatic(providerConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig config(aws_string_c_str(region), provider);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(config);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(accessKeyId);
    aws_string_destroy(secretAccessKey);
    aws_string_destroy(sessionToken);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWSStatic, s_TestIoTMqtt311ConnectWSStatic)

/*
 * AWS Cognito Credentials Provider connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWSCognito(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *cognitoEndpoint = NULL;
    struct aws_string *cognitoIdentity = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_cognito_endpoint, &cognitoEndpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_cognito_identity, &cognitoIdentity);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(cognitoEndpoint);
        aws_string_destroy(cognitoIdentity);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    Aws::Crt::Io::TlsContextOptions cognitoTlsOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    Aws::Crt::Io::TlsContext cognitoTls = Aws::Crt::Io::TlsContext(cognitoTlsOptions, Aws::Crt::Io::TlsMode::CLIENT);
    Aws::Crt::Auth::CredentialsProviderCognitoConfig providerConfig;
    providerConfig.Endpoint = aws_string_c_str(cognitoEndpoint);
    providerConfig.Identity = aws_string_c_str(cognitoIdentity);
    providerConfig.TlsCtx = cognitoTls;
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderCognito(providerConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig config(aws_string_c_str(region), provider);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(config);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(cognitoEndpoint);
    aws_string_destroy(cognitoIdentity);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWSCognito, s_TestIoTMqtt311ConnectWSCognito)

/*
 * AWS Profile Provider connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWSProfile(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *profileCredentials = NULL;
    struct aws_string *profileConfig = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_profile_credentials, &profileCredentials);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_profile_config, &profileConfig);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(profileCredentials);
        aws_string_destroy(profileConfig);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    Aws::Crt::Auth::CredentialsProviderProfileConfig providerConfig;
    providerConfig.ConfigFileNameOverride = aws_byte_cursor_from_c_str(aws_string_c_str(profileConfig));
    providerConfig.CredentialsFileNameOverride = aws_byte_cursor_from_c_str(aws_string_c_str(profileCredentials));
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderProfile(providerConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig config(aws_string_c_str(region), provider);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(config);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(profileCredentials);
    aws_string_destroy(profileConfig);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWSProfile, s_TestIoTMqtt311ConnectWSProfile)

/*
 * AWS Environment Provider connect for MQTT311
 */
static int s_TestIoTMqtt311ConnectWSEnvironment(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;

    struct aws_string *accessKey = NULL;
    struct aws_string *secretAccessKey = NULL;
    struct aws_string *sessionToken = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt311_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt_cred_access_key, &accessKey);
    error |= s_GetEnvVariable(allocator, s_mqtt_cred_secret_access_key, &secretAccessKey);
    error |= s_GetEnvVariable(allocator, s_mqtt_cred_session_token, &sessionToken);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(accessKey);
        aws_string_destroy(secretAccessKey);
        aws_string_destroy(sessionToken);
        return AWS_OP_SKIP;
    }

    Aws::Crt::ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderEnvironment();
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig config(aws_string_c_str(region), provider);

    Aws::Iot::MqttClient client;
    auto clientConfigBuilder = Aws::Iot::MqttClientConnectionConfigBuilder(config);
    clientConfigBuilder.WithEndpoint(aws_string_c_str(endpoint));
    auto clientConfig = clientConfigBuilder.Build();
    ASSERT_TRUE(clientConfig);
    auto connection = client.NewConnection(clientConfig);
    ASSERT_TRUE(*connection);

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

    ASSERT_TRUE(connection->Connect(uuidStr.c_str(), true /*cleanSession*/, 5000 /*keepAliveTimeSecs*/));
    ASSERT_TRUE(connectionCompletedPromise.get_future().get());
    if (connection->Disconnect())
    {
        connectionClosedPromise.get_future().wait();
    }

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(accessKey);
    aws_string_destroy(secretAccessKey);
    aws_string_destroy(sessionToken);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt311ConnectWSEnvironment, s_TestIoTMqtt311ConnectWSEnvironment)

#endif // !BYO_CRYPTO
