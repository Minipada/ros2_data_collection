/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/auth/credentials.h>

#include <aws/auth/private/credentials_utils.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>

AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_env_var, "AWS_ACCESS_KEY_ID");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_env_var, "AWS_SECRET_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_env_var, "AWS_SESSION_TOKEN");
AWS_STATIC_STRING_FROM_LITERAL(s_account_id_env_var, "AWS_ACCOUNT_ID");

static int s_credentials_provider_environment_get_credentials_async(
    struct aws_credentials_provider *provider,
    aws_on_get_credentials_callback_fn callback,
    void *user_data) {

    struct aws_allocator *allocator = provider->allocator;

    struct aws_string *access_key_id = NULL;
    struct aws_string *secret_access_key = NULL;
    struct aws_string *session_token = NULL;
    struct aws_string *account_id = NULL;
    struct aws_credentials *credentials = NULL;
    int error_code = AWS_ERROR_SUCCESS;

    aws_get_environment_value(allocator, s_access_key_id_env_var, &access_key_id);
    aws_get_environment_value(allocator, s_secret_access_key_env_var, &secret_access_key);
    aws_get_environment_value(allocator, s_session_token_env_var, &session_token);
    aws_get_environment_value(allocator, s_account_id_env_var, &account_id);

    if (access_key_id != NULL && access_key_id->len > 0 && secret_access_key != NULL && secret_access_key->len > 0) {
        struct aws_credentials_options creds_option = {
            .access_key_id_cursor = aws_byte_cursor_from_string(access_key_id),
            .secret_access_key_cursor = aws_byte_cursor_from_string(secret_access_key),
            .session_token_cursor = aws_byte_cursor_from_string(session_token),
            .account_id_cursor = aws_byte_cursor_from_string(account_id),
            .expiration_timepoint_seconds = UINT64_MAX,
        };
        credentials = aws_credentials_new_with_options(allocator, &creds_option);
        if (credentials == NULL) {
            error_code = aws_last_error();
        }
    } else {
        error_code = AWS_AUTH_CREDENTIALS_PROVIDER_INVALID_ENVIRONMENT;
    }

    if (error_code == AWS_ERROR_SUCCESS) {
        AWS_LOGF_INFO(
            AWS_LS_AUTH_CREDENTIALS_PROVIDER, "id=%p: Loaded credentials from environment variables", (void *)provider);
    } else {
        AWS_LOGF_INFO(
            AWS_LS_AUTH_CREDENTIALS_PROVIDER,
            "id=%p: Failed to load credentials from environment variables: %s",
            (void *)provider,
            aws_error_str(error_code));
    }

    callback(credentials, error_code, user_data);

    aws_credentials_release(credentials);
    aws_string_destroy(account_id);
    aws_string_destroy(session_token);
    aws_string_destroy(secret_access_key);
    aws_string_destroy(access_key_id);

    return AWS_OP_SUCCESS;
}

static void s_credentials_provider_environment_destroy(struct aws_credentials_provider *provider) {
    aws_credentials_provider_invoke_shutdown_callback(provider);

    aws_mem_release(provider->allocator, provider);
}

static struct aws_credentials_provider_vtable s_aws_credentials_provider_environment_vtable = {
    .get_credentials = s_credentials_provider_environment_get_credentials_async,
    .destroy = s_credentials_provider_environment_destroy,
};

struct aws_credentials_provider *aws_credentials_provider_new_environment(
    struct aws_allocator *allocator,
    const struct aws_credentials_provider_environment_options *options) {
    struct aws_credentials_provider *provider = aws_mem_acquire(allocator, sizeof(struct aws_credentials_provider));
    if (provider == NULL) {
        return NULL;
    }

    AWS_ZERO_STRUCT(*provider);

    aws_credentials_provider_init_base(provider, allocator, &s_aws_credentials_provider_environment_vtable, NULL);

    provider->shutdown_options = options->shutdown_options;

    return provider;
}
