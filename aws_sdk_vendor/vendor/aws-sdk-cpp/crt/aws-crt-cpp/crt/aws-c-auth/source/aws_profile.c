/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/auth/private/aws_profile.h>

#include <aws/auth/credentials.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>

static const struct aws_string *s_profile_get_property_value(
    const struct aws_profile *profile,
    const struct aws_string *property_name) {

    const struct aws_profile_property *property = aws_profile_get_property(profile, property_name);
    if (property == NULL) {
        return NULL;
    }

    return aws_profile_property_get_value(property);
}

AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_profile_var, "aws_access_key_id");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_profile_var, "aws_secret_access_key");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_profile_var, "aws_session_token");
AWS_STATIC_STRING_FROM_LITERAL(s_account_id_profile_var, "aws_account_id");

struct aws_credentials *aws_credentials_new_from_profile(
    struct aws_allocator *allocator,
    const struct aws_profile *profile) {
    const struct aws_string *access_key = s_profile_get_property_value(profile, s_access_key_id_profile_var);
    const struct aws_string *secret_key = s_profile_get_property_value(profile, s_secret_access_key_profile_var);
    if (access_key == NULL || secret_key == NULL) {
        return NULL;
    }

    const struct aws_string *session_token = s_profile_get_property_value(profile, s_session_token_profile_var);
    const struct aws_string *account_id = s_profile_get_property_value(profile, s_account_id_profile_var);
    struct aws_credentials_options creds_option = {
        .access_key_id_cursor = aws_byte_cursor_from_string(access_key),
        .secret_access_key_cursor = aws_byte_cursor_from_string(secret_key),
        .session_token_cursor = aws_byte_cursor_from_string(session_token),
        .account_id_cursor = aws_byte_cursor_from_string(account_id),
        .expiration_timepoint_seconds = UINT64_MAX,
    };

    return aws_credentials_new_with_options(allocator, &creds_option);
}
