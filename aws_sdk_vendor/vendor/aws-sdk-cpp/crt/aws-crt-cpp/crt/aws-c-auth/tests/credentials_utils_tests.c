/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/auth/private/credentials_utils.h>
#include <aws/common/environment.h>
#include <aws/sdkutils/aws_profile.h>
#include <aws/testing/aws_test_harness.h>

static int s_credentials_utils_construct_endpoint_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_string *service_name = aws_string_new_from_c_str(allocator, "sts");
    struct aws_string *endpoint;
    struct aws_string *region;

    region = aws_string_new_from_c_str(allocator, "us-east-2");
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, NULL, NULL));
    ASSERT_STR_EQUALS("sts.us-east-2.amazonaws.com", aws_string_c_str(endpoint));
    aws_string_destroy(endpoint);
    aws_string_destroy(region);

    region = aws_string_new_from_c_str(allocator, "cn-northwest-1");
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, NULL, NULL));
    ASSERT_STR_EQUALS("sts.cn-northwest-1.amazonaws.com.cn", aws_string_c_str(endpoint));
    aws_string_destroy(endpoint);
    aws_string_destroy(region);

    region = aws_string_new_from_c_str(allocator, "us-iso-east-1");
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, NULL, NULL));
    ASSERT_STR_EQUALS("sts.us-iso-east-1.c2s.ic.gov", aws_string_c_str(endpoint));
    aws_string_destroy(endpoint);
    aws_string_destroy(region);

    region = aws_string_new_from_c_str(allocator, "us-isob-east-1");
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, NULL, NULL));
    ASSERT_STR_EQUALS("sts.us-isob-east-1.sc2s.sgov.gov", aws_string_c_str(endpoint));
    aws_string_destroy(endpoint);
    aws_string_destroy(region);

    region = aws_string_new_from_c_str(allocator, "eu-isoe-west-1");
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, NULL, NULL));
    ASSERT_STR_EQUALS("sts.eu-isoe-west-1.cloud.adc-e.uk", aws_string_c_str(endpoint));
    aws_string_destroy(endpoint);
    aws_string_destroy(region);

    region = aws_string_new_from_c_str(allocator, "us-isof-south-1");
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, NULL, NULL));
    ASSERT_STR_EQUALS("sts.us-isof-south-1.csp.hci.ic.gov", aws_string_c_str(endpoint));
    aws_string_destroy(endpoint);
    aws_string_destroy(region);

    aws_string_destroy(service_name);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_utils_construct_endpoint_test, s_credentials_utils_construct_endpoint_test);

static int s_credentials_utils_endpoint_override_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_string *region = aws_string_new_from_c_str(allocator, "us-east-2");
    struct aws_string *service_name = aws_string_new_from_c_str(allocator, "sts");
    struct aws_string *service_name_env = aws_string_new_from_c_str(allocator, "STS");
    struct aws_string *endpoint = NULL;

    /* test service-specific endpoint override */
    struct aws_string *endpoint_override_env = aws_string_new_from_c_str(allocator, "AWS_ENDPOINT_URL_STS");
    struct aws_string *endpoint_override = aws_string_new_from_c_str(allocator, "test.endpoint.override.com");
    aws_set_environment_value(endpoint_override_env, endpoint_override);
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name_env, service_name, NULL, NULL));
    ASSERT_STR_EQUALS(aws_string_c_str(endpoint_override), aws_string_c_str(endpoint));
    aws_unset_environment_value(endpoint_override_env);
    aws_string_destroy(endpoint_override);
    aws_string_destroy(endpoint_override_env);
    aws_string_destroy(endpoint);

    /* test global endpoint override */
    struct aws_string *endpoint_override_env_global = aws_string_new_from_c_str(allocator, "AWS_ENDPOINT_URL");
    endpoint_override = aws_string_new_from_c_str(allocator, "global.endpoint.override.com");
    aws_set_environment_value(endpoint_override_env_global, endpoint_override);
    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name_env, service_name, NULL, NULL));
    ASSERT_STR_EQUALS(aws_string_c_str(endpoint_override), aws_string_c_str(endpoint));
    aws_unset_environment_value(endpoint_override_env_global);
    aws_string_destroy(endpoint_override);
    aws_string_destroy(endpoint_override_env_global);
    aws_string_destroy(endpoint);

    /* test service-specific config endpoint override */
    struct aws_byte_cursor service_override_config_contents =
        aws_byte_cursor_from_c_str("[profile test-profile]\n"
                                   "test = 123\n"
                                   "services = test-services\n"
                                   "[services test-services]\n"
                                   "sts =\n"
                                   "    endpoint_url = test.sts.endpoint.com\n"
                                   "[sso-session session]\n");
    struct aws_byte_buf config_file;
    AWS_ZERO_STRUCT(config_file);
    aws_byte_buf_init_copy_from_cursor(&config_file, allocator, service_override_config_contents);
    struct aws_profile_collection *profile_collection =
        aws_profile_collection_new_from_buffer(allocator, &config_file, AWS_PST_CONFIG);
    struct aws_string *profile_name = aws_string_new_from_c_str(allocator, "test-profile");
    const struct aws_profile *profile = aws_profile_collection_get_profile(profile_collection, profile_name);

    struct aws_string *expected_endpoint = aws_string_new_from_c_str(allocator, "test.sts.endpoint.com");

    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, profile_collection, profile));
    ASSERT_STR_EQUALS(aws_string_c_str(expected_endpoint), aws_string_c_str(endpoint));
    aws_string_destroy(expected_endpoint);
    aws_byte_buf_clean_up(&config_file);
    aws_profile_collection_release(profile_collection);
    aws_string_destroy(endpoint);
    aws_string_destroy(profile_name);
    /* test global config endpoint override */
    struct aws_byte_cursor global_override_config_contents =
        aws_byte_cursor_from_c_str("[profile test-profile]\n"
                                   "test = 123\n"
                                   "endpoint_url = global.sts.endpoint.com\n");
    AWS_ZERO_STRUCT(config_file);
    aws_byte_buf_init_copy_from_cursor(&config_file, allocator, global_override_config_contents);
    profile_collection = aws_profile_collection_new_from_buffer(allocator, &config_file, AWS_PST_CONFIG);
    profile_name = aws_string_new_from_c_str(allocator, "test-profile");
    profile = aws_profile_collection_get_profile(profile_collection, profile_name);
    expected_endpoint = aws_string_new_from_c_str(allocator, "global.sts.endpoint.com");

    ASSERT_SUCCESS(aws_credentials_provider_construct_endpoint(
        allocator, &endpoint, region, service_name, service_name, service_name, profile_collection, profile));
    ASSERT_STR_EQUALS(aws_string_c_str(expected_endpoint), aws_string_c_str(endpoint));
    aws_string_destroy(expected_endpoint);
    aws_byte_buf_clean_up(&config_file);
    aws_profile_collection_release(profile_collection);
    aws_string_destroy(endpoint);
    aws_string_destroy(profile_name);

    aws_string_destroy(region);
    aws_string_destroy(service_name);
    aws_string_destroy(service_name_env);
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(credentials_utils_endpoint_override_test, s_credentials_utils_endpoint_override_test);

static int s_credentials_utils_parse_account_id_from_arn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(
        aws_parse_account_id_from_arn(aws_byte_cursor_from_c_str("::::123456789012::")), "123456789012");
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(
        aws_parse_account_id_from_arn(aws_byte_cursor_from_c_str("arn:a:b:c:123456789012::")), "123456789012");
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(aws_parse_account_id_from_arn(aws_byte_cursor_from_c_str("::::::")), "");
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(aws_parse_account_id_from_arn(aws_byte_cursor_from_c_str("invalid-arn")), "");
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(credentials_utils_parse_account_id_from_arn, s_credentials_utils_parse_account_id_from_arn);
