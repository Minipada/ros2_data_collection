/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include "shared_credentials_test_definitions.h"
#include <aws/auth/private/sso_token_utils.h>

static int s_parse_token_location_url_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    struct aws_string *start_url = aws_string_new_from_c_str(allocator, "https://d-92671207e4.awsapps.com/start");
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, start_url);

    struct aws_byte_cursor token_cursor = aws_byte_cursor_from_string(token_path);
    struct aws_byte_cursor expected_token_cursor =
        aws_byte_cursor_from_c_str("13f9d35043871d073ab260e020f0ffde092cb14b.json");
    struct aws_byte_cursor find_cursor;
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&token_cursor, &expected_token_cursor, &find_cursor));

    aws_string_destroy(start_url);
    aws_string_destroy(token_path);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(parse_token_location_url_test, s_parse_token_location_url_test);

static int s_parse_token_location_session_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    struct aws_string *session = aws_string_new_from_c_str(allocator, "admin");
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, session);
    struct aws_byte_cursor token_cursor = aws_byte_cursor_from_string(token_path);
    struct aws_byte_cursor expected_token_cursor =
        aws_byte_cursor_from_c_str("d033e22ae348aeb5660fc2140aec35850c4da997.json");
    struct aws_byte_cursor find_cursor;
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&token_cursor, &expected_token_cursor, &find_cursor));

    aws_string_destroy(session);
    aws_string_destroy(token_path);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(parse_token_location_session_test, s_parse_token_location_session_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_valid_token_json,
    "{\"accessToken\": \"string\",\"expiresAt\": \"2019-11-14T04:05:45Z\",\"refreshToken\": \"string\",\"clientId\": "
    "\"123321\",\"clientSecret\": \"ABCDE123\",\"registrationExpiresAt\": "
    "\"2022-03-06T19:53:17Z\",\"region\": \"us-west-2\",\"startUrl\": \"https://d-abc123.awsapps.com/start\"}");
static int s_parse_sso_token_valid(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    struct aws_string *file_path = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(file_path, s_valid_token_json));
    struct aws_sso_token *sso_token = aws_sso_token_new_from_file(allocator, file_path);
    ASSERT_TRUE(aws_string_eq_c_str(sso_token->access_token, "string"));
    ASSERT_INT_EQUALS((uint64_t)aws_date_time_as_epoch_secs(&sso_token->expiration), 1573704345);
    aws_string_destroy(file_path);
    aws_sso_token_destroy(sso_token);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(parse_sso_token_valid, s_parse_sso_token_valid);

AWS_STATIC_STRING_FROM_LITERAL(s_invalid_token_json, "invalid json");
static int s_parse_sso_token_invalid(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    struct aws_string *file_path = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(file_path, s_invalid_token_json));
    ASSERT_NULL(aws_sso_token_new_from_file(allocator, file_path));
    ASSERT_INT_EQUALS(AWS_AUTH_SSO_TOKEN_INVALID, aws_last_error());
    aws_string_destroy(file_path);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(parse_sso_token_invalid, s_parse_sso_token_invalid);

AWS_STATIC_STRING_FROM_LITERAL(
    s_missing_access_token_json,
    "{\"expiresAt\": \"2019-11-14T04:05:45Z\",\"refreshToken\": \"string\",\"clientId\": "
    "\"123321\",\"clientSecret\": \"ABCDE123\",\"registrationExpiresAt\": "
    "\"2022-03-06T19:53:17Z\",\"region\": \"us-west-2\",\"startUrl\": \"https://d-abc123.awsapps.com/start\"}");
static int s_parse_sso_token_invalid_missing_access_token(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    struct aws_string *file_path = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(file_path, s_missing_access_token_json));
    ASSERT_NULL(aws_sso_token_new_from_file(allocator, file_path));
    ASSERT_INT_EQUALS(AWS_AUTH_SSO_TOKEN_INVALID, aws_last_error());
    aws_string_destroy(file_path);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(parse_sso_token_invalid_missing_access_token, s_parse_sso_token_invalid_missing_access_token);

AWS_STATIC_STRING_FROM_LITERAL(
    s_missing_expires_at_json,
    "{\"accessToken\": \"string\",\"refreshToken\": \"string\",\"clientId\": "
    "\"123321\",\"clientSecret\": \"ABCDE123\",\"registrationExpiresAt\": "
    "\"2022-03-06T19:53:17Z\",\"region\": \"us-west-2\",\"startUrl\": \"https://d-abc123.awsapps.com/start\"}");
static int s_parse_sso_token_missing_expires_at(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    struct aws_string *file_path = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(file_path, s_missing_expires_at_json));
    ASSERT_NULL(aws_sso_token_new_from_file(allocator, file_path));
    ASSERT_INT_EQUALS(AWS_AUTH_SSO_TOKEN_INVALID, aws_last_error());
    aws_string_destroy(file_path);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(parse_sso_token_missing_expires_at, s_parse_sso_token_missing_expires_at);

AWS_STATIC_STRING_FROM_LITERAL(
    s_invalid_expires_at_json,
    "{\"accessToken\": \"string\",\"expiresAt\": \"1234567\",\"refreshToken\": \"string\",\"clientId\": "
    "\"123321\",\"clientSecret\": \"ABCDE123\",\"registrationExpiresAt\": "
    "\"2022-03-06T19:53:17Z\",\"region\": \"us-west-2\",\"startUrl\": \"https://d-abc123.awsapps.com/start\"}");
static int s_parse_sso_token_invalid_expires_at(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    struct aws_string *file_path = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(file_path, s_invalid_expires_at_json));
    ASSERT_NULL(aws_sso_token_new_from_file(allocator, file_path));
    ASSERT_INT_EQUALS(AWS_AUTH_SSO_TOKEN_INVALID, aws_last_error());
    aws_string_destroy(file_path);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(parse_sso_token_invalid_expires_at, s_parse_sso_token_invalid_expires_at);
