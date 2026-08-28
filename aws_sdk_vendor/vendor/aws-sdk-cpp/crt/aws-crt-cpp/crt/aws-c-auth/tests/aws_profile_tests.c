/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/private/aws_profile.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>
#include <aws/io/file_utils.h>

AWS_STATIC_STRING_FROM_LITERAL(s_config_override_path, "/tmp/.aws/config");

#ifdef _WIN32
AWS_STATIC_STRING_FROM_LITERAL(s_config_override_path_result, "\\tmp\\.aws\\config");
#else
AWS_STATIC_STRING_FROM_LITERAL(s_config_override_path_result, "/tmp/.aws/config");
#endif /* _WIN32 */

static int s_config_file_path_override_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor override_cursor = aws_byte_cursor_from_string(s_config_override_path);
    struct aws_string *path = aws_get_config_file_path(allocator, &override_cursor);
    ASSERT_TRUE(aws_string_compare(path, s_config_override_path_result) == 0);

    aws_string_destroy(path);

    return 0;
}

AWS_TEST_CASE(config_file_path_override_test, s_config_file_path_override_test);

AWS_STATIC_STRING_FROM_LITERAL(s_config_env_var, "AWS_CONFIG_FILE");

static int s_config_file_path_environment_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_set_environment_value(s_config_env_var, s_config_override_path);

    struct aws_string *path = aws_get_config_file_path(allocator, NULL);
    ASSERT_TRUE(aws_string_compare(path, s_config_override_path_result) == 0);

    aws_string_destroy(path);

    return 0;
}

AWS_TEST_CASE(config_file_path_environment_test, s_config_file_path_environment_test);

AWS_STATIC_STRING_FROM_LITERAL(s_credentials_override_path, "/tmp/.aws/credentials");

#ifdef _WIN32
AWS_STATIC_STRING_FROM_LITERAL(s_credentials_override_path_result, "\\tmp\\.aws\\credentials");
#else
AWS_STATIC_STRING_FROM_LITERAL(s_credentials_override_path_result, "/tmp/.aws/credentials");
#endif /* _WIN32 */

static int s_credentials_file_path_override_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor override_cursor = aws_byte_cursor_from_string(s_credentials_override_path);
    struct aws_string *path = aws_get_credentials_file_path(allocator, &override_cursor);
    ASSERT_TRUE(aws_string_compare(path, s_credentials_override_path_result) == 0);

    aws_string_destroy(path);

    return 0;
}

AWS_TEST_CASE(credentials_file_path_override_test, s_credentials_file_path_override_test);

AWS_STATIC_STRING_FROM_LITERAL(s_credentials_env_var, "AWS_SHARED_CREDENTIALS_FILE");

static int s_credentials_file_path_environment_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_set_environment_value(s_credentials_env_var, s_credentials_override_path);

    struct aws_string *path = aws_get_credentials_file_path(allocator, NULL);
    ASSERT_TRUE(aws_string_compare(path, s_credentials_override_path_result) == 0);

    aws_string_destroy(path);

    return 0;
}

AWS_TEST_CASE(credentials_file_path_environment_test, s_credentials_file_path_environment_test);

AWS_STATIC_STRING_FROM_LITERAL(s_profile_env_var, "AWS_PROFILE");
AWS_STATIC_STRING_FROM_LITERAL(s_profile_override, "NotTheDefault");

static int s_profile_override_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor override_cursor = aws_byte_cursor_from_string(s_profile_override);
    struct aws_string *profile_name = aws_get_profile_name(allocator, &override_cursor);
    ASSERT_TRUE(aws_string_compare(profile_name, s_profile_override) == 0);

    aws_string_destroy(profile_name);

    return 0;
}

AWS_TEST_CASE(profile_override_test, s_profile_override_test);

static int s_profile_environment_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_set_environment_value(s_profile_env_var, s_profile_override);

    struct aws_string *profile_name = aws_get_profile_name(allocator, NULL);
    ASSERT_TRUE(aws_string_compare(profile_name, s_profile_override) == 0);

    aws_string_destroy(profile_name);

    return 0;
}

AWS_TEST_CASE(profile_environment_test, s_profile_environment_test);
