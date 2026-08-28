/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/common/environment.h>
#include <aws/crt/Api.h>
#include <aws/crt/UUID.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/crt/http/HttpProxyStrategy.h>
#include <aws/crt/io/Pkcs11.h>
#include <aws/crt/mqtt/Mqtt5Packets.h>
#include <aws/iot/Mqtt5Client.h>
#include <aws/iot/MqttCommon.h>
#include <aws/testing/aws_test_harness.h>

#include <utility>
#if !BYO_CRYPTO

using namespace Aws::Crt;
using namespace Aws::Crt::Mqtt5;

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_hostname, "AWS_TEST_MQTT5_IOT_CORE_HOST");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_region, "AWS_TEST_MQTT5_IOT_CORE_REGION");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_rsa_cert, "AWS_TEST_MQTT311_IOT_CORE_RSA_CERT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_rsa_key, "AWS_TEST_MQTT311_IOT_CORE_RSA_KEY");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_nosign_custom_auth_name,
    "AWS_TEST_MQTT5_IOT_CORE_NO_SIGNING_AUTHORIZER_NAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_nosign_custom_auth_username,
    "AWS_TEST_MQTT5_IOT_CORE_NO_SIGNING_AUTHORIZER_USERNAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_nosign_custom_auth_password,
    "AWS_TEST_MQTT5_IOT_CORE_NO_SIGNING_AUTHORIZER_PASSWORD");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_sign_custom_auth_name,
    "AWS_TEST_MQTT5_IOT_CORE_SIGNING_AUTHORIZER_NAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_sign_custom_auth_username,
    "AWS_TEST_MQTT5_IOT_CORE_SIGNING_AUTHORIZER_USERNAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_sign_custom_auth_password,
    "AWS_TEST_MQTT5_IOT_CORE_SIGNING_AUTHORIZER_PASSWORD");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_sign_custom_auth_tokenvalue,
    "AWS_TEST_MQTT5_IOT_CORE_SIGNING_AUTHORIZER_TOKEN");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_sign_custom_auth_tokenkey,
    "AWS_TEST_MQTT5_IOT_CORE_SIGNING_AUTHORIZER_TOKEN_KEY_NAME");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_sign_custom_auth_tokensignature,
    "AWS_TEST_MQTT5_IOT_CORE_SIGNING_AUTHORIZER_TOKEN_SIGNATURE");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_sign_custom_auth_tokensignature_unencoded,
    "AWS_TEST_MQTT5_IOT_CORE_SIGNING_AUTHORIZER_TOKEN_SIGNATURE_UNENCODED");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_pkcs11_lib, "AWS_TEST_PKCS11_LIB");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_pkcs11_token_label, "AWS_TEST_PKCS11_TOKEN_LABEL");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_pkcs11_pin, "AWS_TEST_PKCS11_PIN");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_pkcs11_private_key_label, "AWS_TEST_PKCS11_PKEY_LABEL");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_pkcs11_cert, "AWS_TEST_PKCS11_CERT_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_pkcs11_ca, "AWS_TEST_PKCS11_CA_FILE");
// C++ specific PKCS11 check: only runs PKCS11 if 'DUSE_OPENSSL=ON' is set in the builder
AWS_STATIC_STRING_FROM_LITERAL(s_test_envName_iot_pkcs11_use_openssl, "AWS_TEST_PKCS11_USE_OPENSSL_SET");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_pkcs12_key, "AWS_TEST_MQTT5_IOT_CORE_PKCS12_KEY");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_pkcs12_key_password,
    "AWS_TEST_MQTT5_IOT_CORE_PKCS12_KEY_PASSWORD");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_windows_cert, "AWS_TEST_MQTT5_IOT_CORE_WINDOWS_CERT_STORE");

AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_credential_access_key,
    "AWS_TEST_MQTT5_ROLE_CREDENTIAL_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_credential_secret_access_key,
    "AWS_TEST_MQTT5_ROLE_CREDENTIAL_SECRET_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(
    s_mqtt5_test_envName_iot_credential_session_token,
    "AWS_TEST_MQTT5_ROLE_CREDENTIAL_SESSION_TOKEN");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt_cred_access_key, "AWS_ACCESS_KEY_ID");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt_cred_secret_access_key, "AWS_SECRET_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt_cred_session_token, "AWS_SESSION_TOKEN");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_cognito_endpoint, "AWS_TEST_MQTT5_COGNITO_ENDPOINT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_cognito_identity, "AWS_TEST_MQTT5_COGNITO_IDENTITY");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_profile_credentials, "AWS_TEST_MQTT5_IOT_PROFILE_CREDENTIALS");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_profile_config, "AWS_TEST_MQTT5_IOT_PROFILE_CONFIG");

AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_x509_endpoint, "AWS_TEST_MQTT5_IOT_CORE_X509_ENDPOINT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_x509_role_alias, "AWS_TEST_MQTT5_IOT_CORE_X509_ROLE_ALIAS");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_x509_thing_name, "AWS_TEST_MQTT5_IOT_CORE_X509_THING_NAME");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_x509_cert, "AWS_TEST_MQTT5_IOT_CORE_X509_CERT");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_x509_key, "AWS_TEST_MQTT5_IOT_CORE_X509_KEY");
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_iot_x509_ca, "AWS_TEST_MQTT5_IOT_CORE_X509_CA");

// Needed to return "success" instead of skip in Codebuild so it doesn't count as a failure
AWS_STATIC_STRING_FROM_LITERAL(s_mqtt5_test_envName_codebuild, "CODEBUILD_BUILD_ID");

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

// Test Helper
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

static int s_CheckClientAndStop(
    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client,
    std::promise<bool> *connectionPromise,
    std::promise<void> *stoppedPromise)
{
    ASSERT_TRUE(mqtt5Client);
    ASSERT_TRUE(mqtt5Client->Start());
    ASSERT_TRUE(connectionPromise->get_future().get());
    ASSERT_TRUE(mqtt5Client->Stop());
    stoppedPromise->get_future().get();
    return AWS_OP_SUCCESS;
}

/*
 * IoT Builder with mTLS key/cert connect
 */
static int s_TestIoTMqtt5ConnectWithmTLS(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *cert = NULL;
    struct aws_string *key = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_rsa_cert, &cert);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_rsa_key, &key);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(cert);
        aws_string_destroy(key);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        aws_string_c_str(endpoint), aws_string_c_str(cert), aws_string_c_str(key), allocator);

    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(cert);
    aws_string_destroy(key);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithmTLS, s_TestIoTMqtt5ConnectWithmTLS)

/*
 * IoT Builder with websocket connect
 */
static int s_TestIoTMqtt5ConnectWithWebsocket(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    // Create websocket configuration
    Aws::Crt::Auth::CredentialsProviderChainDefaultConfig defaultConfig;
    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderChainDefault(defaultConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region), provider);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithWebsocket(
        aws_string_c_str(endpoint), websocketConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(IoTMqtt5ConnectWithWebsocket, s_TestIoTMqtt5ConnectWithWebsocket)

/*
 * Custom Auth (signing) connect
 */
static int s_TestIoTMqtt5ConnectWithSigningCustomAuth(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *signature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokensignature, &signature);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);

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

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5CustomAuthConfig authConfig(allocator);
    authConfig.WithAuthorizerName(aws_string_c_str(authname));
    authConfig.WithUsername(aws_string_c_str(username));
    authConfig.WithPassword(ByteCursorFromString(aws_string_c_str(password)));
    authConfig.WithTokenKeyName(aws_string_c_str(tokenKeyName));
    authConfig.WithTokenValue(aws_string_c_str(tokenValue));
    authConfig.WithTokenSignature(aws_string_c_str(signature));

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithCustomAuthorizer(
        aws_string_c_str(endpoint), authConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(signature);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithSigningCustomAuth, s_TestIoTMqtt5ConnectWithSigningCustomAuth)

/*
 * Custom Auth (signing with unencoded signature) connect
 */
static int s_TestIoTMqtt5ConnectWithSigningCustomAuthUnencoded(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *unencodedSignature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(
        allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokensignature_unencoded, &unencodedSignature);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);

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

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5CustomAuthConfig authConfig(allocator);
    authConfig.WithAuthorizerName(aws_string_c_str(authname));
    authConfig.WithUsername(aws_string_c_str(username));
    authConfig.WithPassword(ByteCursorFromString(aws_string_c_str(password)));
    authConfig.WithTokenKeyName(aws_string_c_str(tokenKeyName));
    authConfig.WithTokenValue(aws_string_c_str(tokenValue));
    authConfig.WithTokenSignature(aws_string_c_str(unencodedSignature));

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithCustomAuthorizer(
        aws_string_c_str(endpoint), authConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(unencodedSignature);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithSigningCustomAuthUnencoded, s_TestIoTMqtt5ConnectWithSigningCustomAuthUnencoded)

/*
 * Custom Auth (no signing) connect
 */
static int s_TestIoTMqtt5ConnectWithNoSigningCustomAuth(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_nosign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_nosign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_nosign_custom_auth_password, &password);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5CustomAuthConfig authConfig(allocator);
    authConfig.WithAuthorizerName(aws_string_c_str(authname));
    authConfig.WithUsername(aws_string_c_str(username));
    authConfig.WithPassword(ByteCursorFromString(aws_string_c_str(password)));

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithCustomAuthorizer(
        aws_string_c_str(endpoint), authConfig, allocator);
    ASSERT_TRUE(builder);
    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    aws_string_destroy(endpoint);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithNoSigningCustomAuth, s_TestIoTMqtt5ConnectWithNoSigningCustomAuth)

/*
 * Custom Auth (no signing) connect - websockets
 */
static int s_TestIoTMqtt5ConnectWithNoSigningCustomAuthWebsockets(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_nosign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_nosign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_nosign_custom_auth_password, &password);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5CustomAuthConfig authConfig(allocator);
    authConfig.WithAuthorizerName(aws_string_c_str(authname));
    authConfig.WithUsername(aws_string_c_str(username));
    authConfig.WithPassword(ByteCursorFromString(aws_string_c_str(password)));

    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region));

    Aws::Iot::Mqtt5ClientBuilder *builder =
        Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithCustomAuthorizerWebsocket(
            aws_string_c_str(endpoint), authConfig, websocketConfig, allocator);
    ASSERT_TRUE(builder);
    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithNoSigningCustomAuthWebsockets, s_TestIoTMqtt5ConnectWithNoSigningCustomAuthWebsockets)

/*
 * Custom Auth (signing) connect - websockets
 */
static int s_TestIoTMqtt5ConnectWithSigningCustomAuthWebsockets(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *signature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokensignature, &signature);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        aws_string_destroy(signature);
        aws_string_destroy(tokenKeyName);
        aws_string_destroy(tokenValue);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5CustomAuthConfig authConfig(allocator);
    authConfig.WithAuthorizerName(aws_string_c_str(authname));
    authConfig.WithUsername(aws_string_c_str(username));
    authConfig.WithPassword(ByteCursorFromString(aws_string_c_str(password)));
    authConfig.WithTokenKeyName(aws_string_c_str(tokenKeyName));
    authConfig.WithTokenValue(aws_string_c_str(tokenValue));
    authConfig.WithTokenSignature(aws_string_c_str(signature));

    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region));

    Aws::Iot::Mqtt5ClientBuilder *builder =
        Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithCustomAuthorizerWebsocket(
            aws_string_c_str(endpoint), authConfig, websocketConfig, allocator);
    ASSERT_TRUE(builder);
    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);
    aws_string_destroy(signature);

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithSigningCustomAuthWebsockets, s_TestIoTMqtt5ConnectWithSigningCustomAuthWebsockets)

/*
 * Custom Auth (signing but unencoded signature) connect - websockets
 */
static int s_TestIoTMqtt5ConnectWithSigningCustomAuthWebsocketsUnencoded(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *authname = NULL;
    struct aws_string *username = NULL;
    struct aws_string *password = NULL;
    struct aws_string *unencodedSignature = NULL;
    struct aws_string *tokenKeyName = NULL;
    struct aws_string *tokenValue = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_name, &authname);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_username, &username);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_password, &password);
    error |= s_GetEnvVariable(
        allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokensignature_unencoded, &unencodedSignature);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenkey, &tokenKeyName);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_sign_custom_auth_tokenvalue, &tokenValue);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(authname);
        aws_string_destroy(username);
        aws_string_destroy(password);
        aws_string_destroy(unencodedSignature);
        aws_string_destroy(tokenKeyName);
        aws_string_destroy(tokenValue);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Mqtt5CustomAuthConfig authConfig(allocator);
    authConfig.WithAuthorizerName(aws_string_c_str(authname));
    authConfig.WithUsername(aws_string_c_str(username));
    authConfig.WithPassword(ByteCursorFromString(aws_string_c_str(password)));
    authConfig.WithTokenKeyName(aws_string_c_str(tokenKeyName));
    authConfig.WithTokenValue(aws_string_c_str(tokenValue));
    authConfig.WithTokenSignature(aws_string_c_str(unencodedSignature));

    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region));

    Aws::Iot::Mqtt5ClientBuilder *builder =
        Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithCustomAuthorizerWebsocket(
            aws_string_c_str(endpoint), authConfig, websocketConfig, allocator);
    ASSERT_TRUE(builder);
    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(authname);
    aws_string_destroy(username);
    aws_string_destroy(password);
    aws_string_destroy(tokenKeyName);
    aws_string_destroy(tokenValue);
    aws_string_destroy(unencodedSignature);

    delete builder;
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(
    IoTMqtt5ConnectWithSigningCustomAuthWebsocketsUnencoded,
    s_TestIoTMqtt5ConnectWithSigningCustomAuthWebsocketsUnencoded)

/*
 * PKCS11 connect for MQTT5
 */
static int s_TestIoTMqtt5ConnectWithPKCS11(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *pkcs11_lib = NULL;
    struct aws_string *pkcs11_cert = NULL;
    struct aws_string *pkcs11_userPin = NULL;
    struct aws_string *pkcs11_tokenLabel = NULL;
    struct aws_string *pkcs11_privateKeyLabel = NULL;
    struct aws_string *pkcs11_ca = NULL;
    struct aws_string *pkcs11_use_openssl = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs11_lib, &pkcs11_lib);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs11_cert, &pkcs11_cert);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs11_pin, &pkcs11_userPin);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs11_token_label, &pkcs11_tokenLabel);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs11_private_key_label, &pkcs11_privateKeyLabel);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs11_ca, &pkcs11_ca);
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

    ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Io::Pkcs11Lib> pkcs11Lib = Aws::Crt::Io::Pkcs11Lib::Create(
        aws_string_c_str(pkcs11_lib), Aws::Crt::Io::Pkcs11Lib::InitializeFinalizeBehavior::Strict, allocator);
    if (!pkcs11Lib)
    {
        fprintf(stderr, "Pkcs11Lib failed: %s\n", Aws::Crt::ErrorDebugString(Aws::Crt::LastError()));
        ASSERT_TRUE(false);
    }
    Aws::Crt::Io::TlsContextPkcs11Options pkcs11Options(pkcs11Lib);
    pkcs11Options.SetCertificateFilePath(aws_string_c_str(pkcs11_cert));
    pkcs11Options.SetUserPin(aws_string_c_str(pkcs11_userPin));
    pkcs11Options.SetTokenLabel(aws_string_c_str(pkcs11_tokenLabel));
    pkcs11Options.SetPrivateKeyObjectLabel(aws_string_c_str(pkcs11_privateKeyLabel));

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsPkcs11(
        aws_string_c_str(endpoint), pkcs11Options, allocator);

    builder->WithPort(8883);
    builder->WithCertificateAuthority(aws_string_c_str(pkcs11_ca));

    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
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
AWS_TEST_CASE(IoTMqtt5ConnectWithPKCS11, s_TestIoTMqtt5ConnectWithPKCS11)

/*
 * PKCS12 connect for MQTT5
 */
static int s_TestIoTMqtt5ConnectWithPKCS12(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *pkcs12_key = NULL;
    struct aws_string *pkcs12_password = NULL;
    struct aws_string *codebuild_buildID = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs12_key, &pkcs12_key);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_pkcs12_key_password, &pkcs12_password);
    s_GetEnvVariable(allocator, s_mqtt5_test_envName_codebuild, &codebuild_buildID);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(pkcs12_key);
        aws_string_destroy(pkcs12_password);

        // If in Codebuild, return as a 'success' even though it is a skip
        if (codebuild_buildID && aws_string_is_valid(codebuild_buildID))
        {
            aws_string_destroy(codebuild_buildID);
            return AWS_OP_SUCCESS;
        }
        aws_string_destroy(codebuild_buildID);

        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    Aws::Iot::Pkcs12Options testPkcs12Options;
    testPkcs12Options.pkcs12_file = aws_string_c_str(pkcs12_key);
    testPkcs12Options.pkcs12_password = aws_string_c_str(pkcs12_password);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsPkcs12(
        aws_string_c_str(endpoint), testPkcs12Options, allocator);

    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(pkcs12_key);
    aws_string_destroy(pkcs12_password);
    aws_string_destroy(codebuild_buildID);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithPKCS12, s_TestIoTMqtt5ConnectWithPKCS12)

/*
 * Windows Cert connect for MQTT311
 */
static int s_TestIoTMqtt5ConnectWithWindowsCert(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *windows_cert = NULL;
    struct aws_string *codebuild_buildID = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_windows_cert, &windows_cert);
    s_GetEnvVariable(allocator, s_mqtt5_test_envName_codebuild, &codebuild_buildID);
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

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithWindowsCertStorePath(
        aws_string_c_str(endpoint), aws_string_c_str(windows_cert), allocator);

    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(windows_cert);
    aws_string_destroy(codebuild_buildID);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWithWindowsCert, s_TestIoTMqtt5ConnectWithWindowsCert)

/*
 * AWS Static Credentials Provider connect for MQTT311
 */
static int s_TestIoTMqtt5ConnectWSStatic(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *accessKeyId = NULL;
    struct aws_string *secretAccessKey = NULL;
    struct aws_string *sessionToken = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_credential_access_key, &accessKeyId);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_credential_secret_access_key, &secretAccessKey);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_credential_session_token, &sessionToken);
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

    ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    Aws::Crt::Auth::CredentialsProviderStaticConfig providerConfig;
    providerConfig.AccessKeyId = aws_byte_cursor_from_c_str(aws_string_c_str(accessKeyId));
    providerConfig.SecretAccessKey = aws_byte_cursor_from_c_str(aws_string_c_str(secretAccessKey));
    providerConfig.SessionToken = aws_byte_cursor_from_c_str(aws_string_c_str(sessionToken));
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderStatic(providerConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region), provider);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithWebsocket(
        aws_string_c_str(endpoint), websocketConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(accessKeyId);
    aws_string_destroy(secretAccessKey);
    aws_string_destroy(sessionToken);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWSStatic, s_TestIoTMqtt5ConnectWSStatic)

/*
 * AWS Cognito Credentials Provider connect for MQTT311
 */
static int s_TestIoTMqtt5ConnectWSCognito(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *cognitoEndpoint = NULL;
    struct aws_string *cognitoIdentity = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_cognito_endpoint, &cognitoEndpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_cognito_identity, &cognitoIdentity);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(cognitoEndpoint);
        aws_string_destroy(cognitoIdentity);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    Aws::Crt::Io::TlsContextOptions cognitoTlsOptions = Aws::Crt::Io::TlsContextOptions::InitDefaultClient();
    Aws::Crt::Io::TlsContext cognitoTls = Aws::Crt::Io::TlsContext(cognitoTlsOptions, Aws::Crt::Io::TlsMode::CLIENT);
    Aws::Crt::Auth::CredentialsProviderCognitoConfig providerConfig;
    providerConfig.Endpoint = aws_string_c_str(cognitoEndpoint);
    providerConfig.Identity = aws_string_c_str(cognitoIdentity);
    providerConfig.TlsCtx = cognitoTls;
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderCognito(providerConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region), provider);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithWebsocket(
        aws_string_c_str(endpoint), websocketConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(cognitoEndpoint);
    aws_string_destroy(cognitoIdentity);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWSCognito, s_TestIoTMqtt5ConnectWSCognito)

/*
 * AWS Profile Provider connect for MQTT5
 */
static int s_TestIoTMqtt5ConnectWSProfile(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;
    struct aws_string *profileCredentials = NULL;
    struct aws_string *profileConfig = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_profile_credentials, &profileCredentials);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_profile_config, &profileConfig);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(profileCredentials);
        aws_string_destroy(profileConfig);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    Aws::Crt::Auth::CredentialsProviderProfileConfig providerConfig;
    providerConfig.ConfigFileNameOverride = aws_byte_cursor_from_c_str(aws_string_c_str(profileConfig));
    providerConfig.CredentialsFileNameOverride = aws_byte_cursor_from_c_str(aws_string_c_str(profileCredentials));
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderProfile(providerConfig);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region), provider);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithWebsocket(
        aws_string_c_str(endpoint), websocketConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(profileCredentials);
    aws_string_destroy(profileConfig);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWSProfile, s_TestIoTMqtt5ConnectWSProfile)

/*
 * AWS Environment Provider connect for MQTT5
 */
static int s_TestIoTMqtt5ConnectWSEnvironment(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;

    struct aws_string *accessKey = NULL;
    struct aws_string *secretAccessKey = NULL;
    struct aws_string *sessionToken = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
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

    ApiHandle apiHandle(allocator);

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider = nullptr;
    provider = Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderEnvironment();
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region), provider);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithWebsocket(
        aws_string_c_str(endpoint), websocketConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(accessKey);
    aws_string_destroy(secretAccessKey);
    aws_string_destroy(sessionToken);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWSEnvironment, s_TestIoTMqtt5ConnectWSEnvironment)

/*
 * AWS X509 connect for MQTT5
 */
static int s_TestIoTMqtt5ConnectWSX509(Aws::Crt::Allocator *allocator, void *)
{
    struct aws_string *endpoint = NULL;
    struct aws_string *region = NULL;

    struct aws_string *x509Endpoint = NULL;
    struct aws_string *x509RoleAlias = NULL;
    struct aws_string *x509ThingName = NULL;
    struct aws_string *x509CertificatePath = NULL;
    struct aws_string *x509KeyPath = NULL;
    struct aws_string *x509RootCAPath = NULL;

    int error = s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_hostname, &endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_region, &region);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_x509_endpoint, &x509Endpoint);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_x509_role_alias, &x509RoleAlias);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_x509_thing_name, &x509ThingName);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_x509_cert, &x509CertificatePath);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_x509_key, &x509KeyPath);
    error |= s_GetEnvVariable(allocator, s_mqtt5_test_envName_iot_x509_ca, &x509RootCAPath);
    if (error != AWS_OP_SUCCESS)
    {
        printf("Environment Variables are not set for the test, skip the test");
        aws_string_destroy(endpoint);
        aws_string_destroy(region);
        aws_string_destroy(x509Endpoint);
        aws_string_destroy(x509RoleAlias);
        aws_string_destroy(x509ThingName);
        aws_string_destroy(x509CertificatePath);
        aws_string_destroy(x509KeyPath);
        aws_string_destroy(x509RootCAPath);
        return AWS_OP_SKIP;
    }

    ApiHandle apiHandle(allocator);

    auto m_eventLoopGroup =
        Aws::Crt::MakeShared<Aws::Crt::Io::EventLoopGroup>(allocator, static_cast<uint16_t>(1), allocator);
    auto m_hostResolver =
        Aws::Crt::MakeShared<Aws::Crt::Io::DefaultHostResolver>(allocator, *m_eventLoopGroup, 8, 30, allocator);
    auto m_clientBootstrap =
        Aws::Crt::MakeShared<Aws::Crt::Io::ClientBootstrap>(allocator, *m_eventLoopGroup, *m_hostResolver, allocator);

    Aws::Crt::Auth::CredentialsProviderX509Config providerConfig;
    providerConfig.Bootstrap = m_clientBootstrap.get();
    providerConfig.Endpoint = aws_string_c_str(x509Endpoint);
    providerConfig.RoleAlias = aws_string_c_str(x509RoleAlias);
    providerConfig.ThingName = aws_string_c_str(x509ThingName);

    Aws::Crt::Io::TlsContextOptions x509TlsCtxOptions = Aws::Crt::Io::TlsContextOptions::InitClientWithMtls(
        (const char *)x509CertificatePath->bytes, (const char *)x509KeyPath->bytes, allocator);
    x509TlsCtxOptions.OverrideDefaultTrustStore(NULL, (const char *)x509RootCAPath->bytes);

    auto m_x509TlsContext = Aws::Crt::MakeShared<Aws::Crt::Io::TlsContext>(
        allocator, x509TlsCtxOptions, Aws::Crt::Io::TlsMode::CLIENT, allocator);

    Aws::Crt::Io::TlsConnectionOptions tlsConnectionOptions = m_x509TlsContext->NewConnectionOptions();
    providerConfig.TlsOptions = tlsConnectionOptions;

    std::shared_ptr<Aws::Crt::Auth::ICredentialsProvider> provider =
        Aws::Crt::Auth::CredentialsProvider::CreateCredentialsProviderX509(providerConfig, allocator);
    ASSERT_TRUE(provider);
    Aws::Iot::WebsocketConfig websocketConfig(aws_string_c_str(region), provider);

    Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithWebsocket(
        aws_string_c_str(endpoint), websocketConfig, allocator);
    ASSERT_TRUE(builder);

    std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;

    s_setupConnectionLifeCycle(builder, connectionPromise, stoppedPromise);

    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> mqtt5Client = builder->Build();

    ASSERT_SUCCESS(s_CheckClientAndStop(mqtt5Client, &connectionPromise, &stoppedPromise));

    delete builder;
    aws_string_destroy(endpoint);
    aws_string_destroy(region);
    aws_string_destroy(x509Endpoint);
    aws_string_destroy(x509RoleAlias);
    aws_string_destroy(x509ThingName);
    aws_string_destroy(x509CertificatePath);
    aws_string_destroy(x509KeyPath);
    aws_string_destroy(x509RootCAPath);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(IoTMqtt5ConnectWSX509, s_TestIoTMqtt5ConnectWSX509)

#endif // !BYO_CRYPTO
