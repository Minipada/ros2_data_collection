/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/common/clock.h>
#include <aws/common/condition_variable.h>
#include <aws/common/environment.h>
#include <aws/common/mutex.h>
#include <aws/common/string.h>
#include <aws/common/thread.h>
#include <aws/http/http.h>

#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/tls_channel_handler.h>
#include <credentials_provider_utils.h>

#include "shared_credentials_test_definitions.h"

#ifdef _MSC_VER
#    pragma warning(disable : 4996)
#endif

AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_test_value, "My Access Key");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_test_value, "SekritKey");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_test_value, "Some Session Token");
AWS_STATIC_STRING_FROM_LITERAL(s_account_id_test_value, "Some Account Value");

AWS_STATIC_STRING_FROM_LITERAL(s_account_id_env_var, "AWS_ACCOUNT_ID");

static int s_credentials_create_destroy_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials *credentials = aws_credentials_new_from_string(
        allocator, s_access_key_id_test_value, s_secret_access_key_test_value, s_session_token_test_value, UINT64_MAX);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(credentials), s_access_key_id_test_value);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(credentials), s_secret_access_key_test_value);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(credentials), s_session_token_test_value);

    aws_credentials_release(credentials);

    return 0;
}

AWS_TEST_CASE(credentials_create_destroy_test, s_credentials_create_destroy_test);

static int s_anonymous_credentials_create_destroy_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials *credentials = aws_credentials_new_anonymous(allocator);

    ASSERT_NOT_NULL(credentials);
    ASSERT_TRUE(aws_credentials_is_anonymous(credentials));
    ASSERT_NULL(aws_credentials_get_access_key_id(credentials).ptr);
    ASSERT_NULL(aws_credentials_get_secret_access_key(credentials).ptr);
    ASSERT_NULL(aws_credentials_get_session_token(credentials).ptr);

    aws_credentials_release(credentials);
    return 0;
}

AWS_TEST_CASE(anonymous_credentials_create_destroy_test, s_anonymous_credentials_create_destroy_test);

struct aws_credentials_shutdown_checker {
    struct aws_mutex lock;
    struct aws_condition_variable signal;
    bool is_shutdown_complete;
};

static struct aws_credentials_shutdown_checker s_shutdown_checker;

static void s_aws_credentials_shutdown_checker_init(void) {
    AWS_ZERO_STRUCT(s_shutdown_checker);
    aws_mutex_init(&s_shutdown_checker.lock);
    aws_condition_variable_init(&s_shutdown_checker.signal);
}

static void s_aws_credentials_shutdown_checker_clean_up(void) {
    aws_mutex_clean_up(&s_shutdown_checker.lock);
    aws_condition_variable_clean_up(&s_shutdown_checker.signal);
}

static void s_on_shutdown_complete(void *user_data) {
    (void)user_data;

    aws_mutex_lock(&s_shutdown_checker.lock);
    s_shutdown_checker.is_shutdown_complete = true;
    aws_mutex_unlock(&s_shutdown_checker.lock);

    aws_condition_variable_notify_one(&s_shutdown_checker.signal);
}

static bool s_has_tester_received_shutdown_callback(void *user_data) {
    (void)user_data;

    return s_shutdown_checker.is_shutdown_complete;
}

static void s_aws_wait_for_provider_shutdown_callback(void) {
    aws_mutex_lock(&s_shutdown_checker.lock);
    aws_condition_variable_wait_pred(
        &s_shutdown_checker.signal, &s_shutdown_checker.lock, s_has_tester_received_shutdown_callback, NULL);
    aws_mutex_unlock(&s_shutdown_checker.lock);
}

/*
 * Helper function that takes a provider, expected results from a credentials query,
 * and uses the provider testing utils to query the results
 */
static int s_do_basic_provider_test(
    struct aws_credentials_provider *provider,
    int expected_calls,
    const struct aws_string *expected_access_key_id,
    const struct aws_string *expected_secret_access_key,
    const struct aws_string *expected_session_token,
    const struct aws_string *expected_account_id) {

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, expected_calls);

    int get_async_result =
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results);
    ASSERT_TRUE(get_async_result == AWS_OP_SUCCESS);

    aws_wait_on_credentials_callback(&callback_results);

    ASSERT_TRUE(callback_results.count == expected_calls);

    if (callback_results.credentials != NULL && !aws_credentials_is_anonymous(callback_results.credentials)) {
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_access_key_id(callback_results.credentials), expected_access_key_id);

        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_secret_access_key(callback_results.credentials), expected_secret_access_key);

        if (expected_session_token != NULL) {
            ASSERT_CURSOR_VALUE_STRING_EQUALS(
                aws_credentials_get_session_token(callback_results.credentials), expected_session_token);
        } else {
            ASSERT_TRUE(aws_credentials_get_session_token(callback_results.credentials).len == 0);
        }

        if (expected_account_id != NULL) {
            ASSERT_CURSOR_VALUE_STRING_EQUALS(
                aws_credentials_get_account_id(callback_results.credentials), expected_account_id);
        } else {
            ASSERT_TRUE(aws_credentials_get_account_id(callback_results.credentials).len == 0);
        }
    } else {
        ASSERT_TRUE(expected_access_key_id == NULL);
        ASSERT_TRUE(expected_secret_access_key == NULL);
        ASSERT_TRUE(expected_session_token == NULL);
    }

    aws_get_credentials_test_callback_result_clean_up(&callback_results);

    return AWS_OP_SUCCESS;
}

static int s_static_credentials_provider_basic_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials_provider_static_options options = {
        .access_key_id = aws_byte_cursor_from_string(s_access_key_id_test_value),
        .secret_access_key = aws_byte_cursor_from_string(s_secret_access_key_test_value),
        .session_token = aws_byte_cursor_from_string(s_session_token_test_value),
        .account_id = aws_byte_cursor_from_string(s_account_id_test_value),
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    s_aws_credentials_shutdown_checker_init();

    struct aws_credentials_provider *provider = aws_credentials_provider_new_static(allocator, &options);

    ASSERT_TRUE(
        s_do_basic_provider_test(
            provider,
            1,
            s_access_key_id_test_value,
            s_secret_access_key_test_value,
            s_session_token_test_value,
            s_account_id_test_value) == AWS_OP_SUCCESS);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    return 0;
}

AWS_TEST_CASE(static_credentials_provider_basic_test, s_static_credentials_provider_basic_test);

static int s_anonymous_credentials_provider_basic_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials_provider_shutdown_options shutdown_options = {
        .shutdown_callback = s_on_shutdown_complete,
        .shutdown_user_data = NULL,
    };

    s_aws_credentials_shutdown_checker_init();

    struct aws_credentials_provider *provider = aws_credentials_provider_new_anonymous(allocator, &shutdown_options);

    ASSERT_TRUE(s_do_basic_provider_test(provider, 1, NULL, NULL, NULL, NULL) == AWS_OP_SUCCESS);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    /* Check that NULL works for the optional shutdown options */
    provider = aws_credentials_provider_new_anonymous(allocator, NULL);
    aws_credentials_provider_release(provider);

    return 0;
}

AWS_TEST_CASE(anonymous_credentials_provider_basic_test, s_anonymous_credentials_provider_basic_test);

static int s_environment_credentials_provider_basic_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_credentials_shutdown_checker_init();

    aws_set_environment_value(s_access_key_id_env_var, s_access_key_id_test_value);
    aws_set_environment_value(s_secret_access_key_env_var, s_secret_access_key_test_value);
    aws_set_environment_value(s_session_token_env_var, s_session_token_test_value);

    struct aws_credentials_provider_environment_options options = {
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_environment(allocator, &options);

    ASSERT_TRUE(
        s_do_basic_provider_test(
            provider,
            1,
            s_access_key_id_test_value,
            s_secret_access_key_test_value,
            s_session_token_test_value,
            NULL) == AWS_OP_SUCCESS);

    aws_set_environment_value(s_account_id_env_var, s_account_id_test_value);

    ASSERT_TRUE(
        s_do_basic_provider_test(
            provider,
            1,
            s_access_key_id_test_value,
            s_secret_access_key_test_value,
            s_session_token_test_value,
            s_account_id_test_value) == AWS_OP_SUCCESS);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    return 0;
}

AWS_TEST_CASE(environment_credentials_provider_basic_test, s_environment_credentials_provider_basic_test);

static int s_environment_credentials_provider_empty_env_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    s_aws_credentials_shutdown_checker_init();
    struct aws_string *empty = aws_string_new_from_c_str(allocator, "");

    aws_set_environment_value(s_access_key_id_env_var, empty);
    aws_set_environment_value(s_secret_access_key_env_var, empty);
    aws_set_environment_value(s_session_token_env_var, empty);

    struct aws_credentials_provider_environment_options options = {
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_environment(allocator, &options);
    /* Instead of getting an empty credentials, should just fail to fetch credentials */
    ASSERT_TRUE(s_do_basic_provider_test(provider, 1, NULL, NULL, NULL, NULL) == AWS_OP_SUCCESS);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();
    aws_string_destroy(empty);
    s_aws_credentials_shutdown_checker_clean_up();

    return 0;
}

AWS_TEST_CASE(environment_credentials_provider_empty_env_test, s_environment_credentials_provider_empty_env_test);

static int s_do_environment_credentials_provider_failure(struct aws_allocator *allocator) {
    s_aws_credentials_shutdown_checker_init();

    aws_unset_environment_value(s_access_key_id_env_var);
    aws_unset_environment_value(s_secret_access_key_env_var);
    aws_unset_environment_value(s_session_token_env_var);

    struct aws_credentials_provider_environment_options options = {
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_environment(allocator, &options);

    ASSERT_TRUE(s_do_basic_provider_test(provider, 1, NULL, NULL, NULL, NULL) == AWS_OP_SUCCESS);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    return 0;
}

/*
 * Set of related tests that all check and make sure that if you don't specify enough
 * of the credentials data in the environment, you get nothing when you query an
 * environment provider.
 */
static int s_environment_credentials_provider_negative_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    /* nothing in the environment */
    ASSERT_TRUE(s_do_environment_credentials_provider_failure(allocator) == 0);

    /* access key only shouldn't work */
    aws_set_environment_value(s_access_key_id_env_var, s_access_key_id_test_value);
    ASSERT_TRUE(s_do_environment_credentials_provider_failure(allocator) == 0);

    /* secret key only shouldn't work either */
    aws_unset_environment_value(s_access_key_id_env_var);
    aws_set_environment_value(s_secret_access_key_env_var, s_secret_access_key_test_value);
    ASSERT_TRUE(s_do_environment_credentials_provider_failure(allocator) == 0);

    return 0;
}

AWS_TEST_CASE(environment_credentials_provider_negative_test, s_environment_credentials_provider_negative_test);

#define TEST_CACHE_REFRESH_TIME_MS 10000

AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_1, "AccessKey1");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_1, "SecretKey1");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_1, "SessionToken1");
AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_2, "AccessKey2");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_2, "SecretKey2");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_2, "SessionToken2");

int s_wait_for_get_credentials(struct aws_get_credentials_test_callback_result *callback_results) {
    aws_wait_on_credentials_callback(callback_results);

    return 0;
}

int s_invoke_get_credentials(
    struct aws_credentials_provider *provider,
    struct aws_get_credentials_test_callback_result *callback_results,
    int call_count) {
    aws_get_credentials_test_callback_result_init(callback_results, call_count);

    for (int i = 0; i < call_count; ++i) {
        int get_async_result = aws_credentials_provider_get_credentials(
            provider, aws_test_get_credentials_async_callback, callback_results);
        ASSERT_TRUE(get_async_result == AWS_OP_SUCCESS);
    }

    return 0;
}

#define ASYNC_TEST_DELAY_NS 1000000

int s_wait_for_get_credentials_with_mock_async_provider(
    struct aws_get_credentials_test_callback_result *callback_results,
    struct aws_credentials_provider *mock_async_provider) {

    /* Mock provider already has credentials, but won't invoke any
     * get-credentials callbacks until this function is called.
     * The callbacks will fire on another thread. */
    aws_credentials_provider_mock_async_fire_callbacks(mock_async_provider);

    /* Wait for all queued get-credentials callbacks to fire */
    aws_wait_on_credentials_callback(callback_results);

    return 0;
}

static int s_verify_callback_status(
    struct aws_get_credentials_test_callback_result *results,
    int expected_call_count,
    const struct aws_string *expected_access_key_id,
    const struct aws_string *expected_secret_access_key,
    const struct aws_string *expected_session_token) {

    aws_mutex_lock(&results->sync);
    ASSERT_TRUE(results->count == expected_call_count);

    if (expected_access_key_id == NULL) {
        ASSERT_NULL(results->credentials);
    } else {
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_access_key_id(results->credentials), expected_access_key_id);
    }

    if (expected_secret_access_key == NULL) {
        ASSERT_NULL(results->credentials);
    } else {
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_secret_access_key(results->credentials), expected_secret_access_key);
    }

    if (expected_session_token != NULL) {
        ASSERT_CURSOR_VALUE_STRING_EQUALS(
            aws_credentials_get_session_token(results->credentials), expected_session_token);
    }

    aws_mutex_unlock(&results->sync);

    return 0;
}

static int s_cached_credentials_provider_elapsed_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    mock_aws_set_system_time(0);
    mock_aws_set_high_res_time(1);

    s_aws_credentials_shutdown_checker_init();

    struct aws_credentials *first_creds = aws_credentials_new_from_string(
        allocator, s_access_key_id_1, s_secret_access_key_1, s_session_token_1, UINT64_MAX);
    struct aws_credentials *second_creds = aws_credentials_new_from_string(
        allocator, s_access_key_id_2, s_secret_access_key_2, s_session_token_2, UINT64_MAX);

    struct get_credentials_mock_result mock_results[] = {
        {.error_code = 0, .credentials = first_creds},
        {.error_code = 0, .credentials = second_creds},
    };

    struct aws_credentials_provider_shutdown_options shutdown_options = {
        .shutdown_callback = NULL,
        .shutdown_user_data = NULL,
    };

    struct aws_credentials_provider *mock_provider =
        aws_credentials_provider_new_mock(allocator, mock_results, 2, &shutdown_options);

    struct aws_credentials_provider_cached_options options;
    AWS_ZERO_STRUCT(options);
    options.source = mock_provider;
    options.refresh_time_in_milliseconds = TEST_CACHE_REFRESH_TIME_MS;
    options.high_res_clock_fn = mock_aws_get_high_res_time;
    options.system_clock_fn = mock_aws_get_system_time;
    options.shutdown_options.shutdown_callback = s_on_shutdown_complete;
    options.shutdown_options.shutdown_user_data = NULL;

    struct aws_credentials_provider *cached_provider = aws_credentials_provider_new_cached(allocator, &options);
    aws_credentials_provider_release(mock_provider);

    struct aws_get_credentials_test_callback_result callback_results;
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Invoke a couple more times to verify the mock isn't getting called
     */
    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Advance time, but not enough to cause a cache expiration, verify everything's the same
     */
    uint64_t refresh_in_ns =
        aws_timestamp_convert(TEST_CACHE_REFRESH_TIME_MS, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_NANOS, NULL);
    uint64_t now = 0;
    mock_aws_get_high_res_time(&now);
    mock_aws_set_high_res_time(now + refresh_in_ns - 1);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Advance time enough to cause cache expiration, verify we get the second set of mocked credentials
     */
    mock_aws_set_high_res_time(now + refresh_in_ns);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_2, s_secret_access_key_2, s_session_token_2) ==
        0);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    aws_credentials_provider_release(cached_provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    aws_credentials_release(second_creds);
    aws_credentials_release(first_creds);

    return 0;
}

AWS_TEST_CASE(cached_credentials_provider_elapsed_test, s_cached_credentials_provider_elapsed_test);

#define TEST_CACHED_CREDENTIALS_EXPIRATION_TIMEPOINT 3600

static int s_cached_credentials_provider_expired_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    mock_aws_set_system_time(0);
    mock_aws_set_high_res_time(1);

    s_aws_credentials_shutdown_checker_init();

    struct aws_credentials *first_creds = aws_credentials_new_from_string(
        allocator,
        s_access_key_id_1,
        s_secret_access_key_1,
        s_session_token_1,
        TEST_CACHED_CREDENTIALS_EXPIRATION_TIMEPOINT);
    struct aws_credentials *second_creds = aws_credentials_new_from_string(
        allocator,
        s_access_key_id_2,
        s_secret_access_key_2,
        s_session_token_2,
        TEST_CACHED_CREDENTIALS_EXPIRATION_TIMEPOINT * 2);

    struct get_credentials_mock_result mock_results[] = {
        {.error_code = 0, .credentials = first_creds},
        {.error_code = 0, .credentials = second_creds},
    };

    struct aws_credentials_provider_shutdown_options shutdown_options;
    AWS_ZERO_STRUCT(shutdown_options);

    struct aws_credentials_provider *mock_provider =
        aws_credentials_provider_new_mock(allocator, mock_results, 2, &shutdown_options);

    struct aws_credentials_provider_cached_options options;
    AWS_ZERO_STRUCT(options);
    options.source = mock_provider;
    options.refresh_time_in_milliseconds = TEST_CACHE_REFRESH_TIME_MS;
    options.high_res_clock_fn = mock_aws_get_high_res_time;
    options.system_clock_fn = mock_aws_get_system_time;
    options.shutdown_options.shutdown_callback = s_on_shutdown_complete;
    options.shutdown_options.shutdown_user_data = NULL;

    struct aws_credentials_provider *cached_provider = aws_credentials_provider_new_cached(allocator, &options);
    aws_credentials_provider_release(mock_provider);

    struct aws_get_credentials_test_callback_result callback_results;
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Invoke a couple more times to verify the mock isn't getting called
     */
    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Advance time enough to cause a refresh from the caching provider's perspective, but not enough to expire the
     * actual credentials.  Nothing should change because the credential's expiration takes priority.
     */
    uint64_t provider_refresh_in_ns =
        aws_timestamp_convert(TEST_CACHE_REFRESH_TIME_MS, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_NANOS, NULL);

    uint64_t now = 0;
    mock_aws_get_high_res_time(&now);
    mock_aws_set_high_res_time(now + provider_refresh_in_ns);

    mock_aws_get_system_time(&now);
    mock_aws_set_system_time(now + provider_refresh_in_ns);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Advance time enough to trigger credentials expiration, verify we get the second set of mocked credentials
     */
    uint64_t credential_expiration_in_ns = aws_timestamp_convert(
        TEST_CACHED_CREDENTIALS_EXPIRATION_TIMEPOINT, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);

    mock_aws_get_high_res_time(&now);
    mock_aws_set_high_res_time(now + credential_expiration_in_ns);

    mock_aws_get_system_time(&now);
    mock_aws_set_system_time(now + credential_expiration_in_ns);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 1) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials(&callback_results) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 1, s_access_key_id_2, s_secret_access_key_2, s_session_token_2) ==
        0);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    aws_credentials_provider_release(cached_provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    aws_credentials_release(second_creds);
    aws_credentials_release(first_creds);

    return 0;
}

AWS_TEST_CASE(cached_credentials_provider_expired_test, s_cached_credentials_provider_expired_test);

static int s_cached_credentials_provider_queued_async_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    s_aws_credentials_shutdown_checker_init();

    mock_aws_set_system_time(0);
    mock_aws_set_high_res_time(1);

    struct aws_credentials *first_creds = aws_credentials_new_from_string(
        allocator, s_access_key_id_1, s_secret_access_key_1, s_session_token_1, UINT64_MAX);
    struct aws_credentials *second_creds = aws_credentials_new_from_string(
        allocator, s_access_key_id_2, s_secret_access_key_2, s_session_token_2, UINT64_MAX);

    struct aws_event_loop_group *event_loop_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct get_credentials_mock_result mock_results[] = {
        {.error_code = 0, .credentials = first_creds},
        {.error_code = 0, .credentials = second_creds},
    };

    struct aws_credentials_provider_shutdown_options shutdown_options;
    AWS_ZERO_STRUCT(shutdown_options);

    struct aws_credentials_provider *mock_provider =
        aws_credentials_provider_new_mock_async(allocator, mock_results, 2, event_loop_group, &shutdown_options);

    struct aws_credentials_provider_cached_options options;
    AWS_ZERO_STRUCT(options);
    options.source = mock_provider;
    options.refresh_time_in_milliseconds = TEST_CACHE_REFRESH_TIME_MS;
    options.high_res_clock_fn = mock_aws_get_high_res_time;
    options.system_clock_fn = mock_aws_get_system_time;
    options.shutdown_options.shutdown_callback = s_on_shutdown_complete;
    options.shutdown_options.shutdown_user_data = NULL;

    struct aws_credentials_provider *cached_provider = aws_credentials_provider_new_cached(allocator, &options);
    aws_credentials_provider_release(mock_provider);

    struct aws_get_credentials_test_callback_result callback_results;

    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 2) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials_with_mock_async_provider(&callback_results, mock_provider) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 2, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Invoke a couple more times to verify the mock isn't getting called
     */
    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 2) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials_with_mock_async_provider(&callback_results, mock_provider) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 2, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 2) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials_with_mock_async_provider(&callback_results, mock_provider) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 2, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Advance time, but not enough to cause a cache expiration, verify everything's the same
     */
    uint64_t refresh_in_ns =
        aws_timestamp_convert(TEST_CACHE_REFRESH_TIME_MS, AWS_TIMESTAMP_MILLIS, AWS_TIMESTAMP_NANOS, NULL);
    uint64_t now = 0;
    mock_aws_get_high_res_time(&now);
    mock_aws_set_high_res_time(now + refresh_in_ns - 1);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 2) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials_with_mock_async_provider(&callback_results, mock_provider) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 2, s_access_key_id_1, s_secret_access_key_1, s_session_token_1) ==
        0);

    /*
     * Advance time enough to cause cache expiration, verify we get the second set of mocked credentials
     */
    mock_aws_set_high_res_time(now + refresh_in_ns);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    ASSERT_TRUE(s_invoke_get_credentials(cached_provider, &callback_results, 2) == 0);
    ASSERT_TRUE(s_wait_for_get_credentials_with_mock_async_provider(&callback_results, mock_provider) == 0);
    ASSERT_TRUE(
        s_verify_callback_status(&callback_results, 2, s_access_key_id_2, s_secret_access_key_2, s_session_token_2) ==
        0);

    aws_credentials_provider_release(cached_provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    aws_get_credentials_test_callback_result_clean_up(&callback_results);

    aws_credentials_release(second_creds);
    aws_credentials_release(first_creds);

    aws_event_loop_group_release(event_loop_group);

    aws_auth_library_clean_up();

    return 0;
}

AWS_TEST_CASE(cached_credentials_provider_queued_async_test, s_cached_credentials_provider_queued_async_test);

static int s_profile_credentials_provider_new_destroy_defaults_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    s_aws_credentials_shutdown_checker_init();

    struct aws_credentials_provider_profile_options options;
    AWS_ZERO_STRUCT(options);
    options.shutdown_options.shutdown_callback = s_on_shutdown_complete;
    options.shutdown_options.shutdown_user_data = NULL;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);

    aws_credentials_provider_release(provider);

    if (provider) {
        s_aws_wait_for_provider_shutdown_callback();
    }

    s_aws_credentials_shutdown_checker_clean_up();

    return 0;
}

AWS_TEST_CASE(
    profile_credentials_provider_new_destroy_defaults_test,
    s_profile_credentials_provider_new_destroy_defaults_test);

AWS_STATIC_STRING_FROM_LITERAL(s_config_file_path, "~derp/.aws/config");
AWS_STATIC_STRING_FROM_LITERAL(s_credentials_file_path, "/Ithink/globalpaths/arebroken/.aws/credentials");
AWS_STATIC_STRING_FROM_LITERAL(s_profile_name, "notdefault");

static int s_profile_credentials_provider_new_destroy_overrides_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    s_aws_credentials_shutdown_checker_init();

    struct aws_credentials_provider_profile_options options;
    AWS_ZERO_STRUCT(options);
    options.config_file_name_override = aws_byte_cursor_from_string(s_config_file_path);
    options.credentials_file_name_override = aws_byte_cursor_from_string(s_credentials_file_path);
    options.profile_name_override = aws_byte_cursor_from_string(s_profile_name);
    options.shutdown_options.shutdown_callback = s_on_shutdown_complete;
    options.shutdown_options.shutdown_user_data = NULL;

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    return 0;
}

AWS_TEST_CASE(
    profile_credentials_provider_new_destroy_overrides_test,
    s_profile_credentials_provider_new_destroy_overrides_test);

typedef int(s_verify_credentials_callback_fn)(struct aws_get_credentials_test_callback_result *callback_results);

static int s_do_credentials_provider_profile_test(
    struct aws_allocator *allocator,
    const struct aws_string *config_file_path,
    const struct aws_string *config_contents,
    const struct aws_string *creds_file_path,
    const struct aws_string *credentials_contents,
    struct aws_credentials_provider_profile_options *options,
    s_verify_credentials_callback_fn verifier,
    bool reset_environment) {

    s_aws_credentials_shutdown_checker_init();

    int result = AWS_OP_ERR;

    if (reset_environment) {
        /* Zero out all of the environment variables, just in case the user has it set (other tests may re-set it) */
        aws_unset_environment_value(s_default_profile_env_variable_name);
        aws_unset_environment_value(s_default_config_path_env_variable_name);
        aws_unset_environment_value(s_default_credentials_path_env_variable_name);
    }

    if (aws_create_profile_file(config_file_path, config_contents) ||
        aws_create_profile_file(creds_file_path, credentials_contents)) {
        return AWS_OP_ERR;
    }

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, options);
    if (provider == NULL) {
        return AWS_OP_ERR;
    }

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);

    int get_async_result =
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results);

    if (get_async_result == AWS_OP_SUCCESS) {
        aws_wait_on_credentials_callback(&callback_results);

        result = verifier(&callback_results);
    }

    aws_get_credentials_test_callback_result_clean_up(&callback_results);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    return result;
}

AWS_STATIC_STRING_FROM_LITERAL(
    s_config_contents,
    "[profile default]\naws_access_key_id=fake_access_key\naws_secret_access_key=fake_secret_key\n");
AWS_STATIC_STRING_FROM_LITERAL(
    s_config_contents2,
    "[profile default]\naws_access_key_id=fake_access_key2\naws_secret_access_key=fake_secret_key2\n");
AWS_STATIC_STRING_FROM_LITERAL(
    s_account_id_config_contents,
    "[profile "
    "default]\naws_access_key_id=fake_access_key\naws_secret_access_key=fake_secret_key\naws_account_id=fake_account_"
    "id");

AWS_STATIC_STRING_FROM_LITERAL(
    s_credentials_contents,
    "[foo]\naws_access_key_id=foo_access\naws_secret_access_key=foo_secret\naws_session_token=foo_session\n");
AWS_STATIC_STRING_FROM_LITERAL(
    s_credentials_contents2,
    "[foo]\naws_access_key_id=foo_access2\naws_secret_access_key=foo_secret2\naws_session_token=foo_session2\n");

AWS_STATIC_STRING_FROM_LITERAL(s_fake_access, "fake_access_key");
AWS_STATIC_STRING_FROM_LITERAL(s_fake_secret, "fake_secret_key");
AWS_STATIC_STRING_FROM_LITERAL(s_fake_account_id, "fake_account_id");

int s_verify_default_credentials_callback(struct aws_get_credentials_test_callback_result *callback_results) {
    ASSERT_TRUE(callback_results->count == 1);
    ASSERT_TRUE(callback_results->credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(callback_results->credentials), s_fake_access);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(callback_results->credentials), s_fake_secret);
    ASSERT_TRUE(aws_credentials_get_session_token(callback_results->credentials).len == 0);

    return AWS_OP_SUCCESS;
}

static int s_profile_credentials_provider_default_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    ASSERT_SUCCESS(s_do_credentials_provider_profile_test(
        allocator,
        config_file_str,
        s_config_contents,
        creds_file_str,
        s_credentials_contents,
        &options,
        s_verify_default_credentials_callback,
        true));

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(profile_credentials_provider_default_test, s_profile_credentials_provider_default_test);

int s_verify_account_id_credentials_callback(struct aws_get_credentials_test_callback_result *callback_results) {
    ASSERT_TRUE(callback_results->count == 1);
    ASSERT_TRUE(callback_results->credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(callback_results->credentials), s_fake_access);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(callback_results->credentials), s_fake_secret);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_account_id(callback_results->credentials), s_fake_account_id);
    ASSERT_TRUE(aws_credentials_get_session_token(callback_results->credentials).len == 0);

    return AWS_OP_SUCCESS;
}

static int s_profile_credentials_provider_account_id_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)s_account_id_config_contents;

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    ASSERT_SUCCESS(s_do_credentials_provider_profile_test(
        allocator,
        config_file_str,
        s_account_id_config_contents,
        creds_file_str,
        s_credentials_contents,
        &options,
        s_verify_account_id_credentials_callback,
        true));

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(profile_credentials_provider_account_id_test, s_profile_credentials_provider_account_id_test);

static int s_profile_credentials_provider_cached_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    if (aws_create_profile_file(config_file_str, s_config_contents) ||
        aws_create_profile_file(creds_file_str, s_credentials_contents)) {
        return AWS_OP_ERR;
    }

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    /* Read the config files */
    struct aws_profile_collection *config_profiles = NULL;
    struct aws_profile_collection *credentials_profiles = NULL;
    struct aws_profile_collection *merged_profiles = NULL;
    struct aws_string *credentials_file_path = NULL;
    struct aws_string *config_file_path = NULL;
    struct aws_string *profile_name = NULL;

    credentials_file_path = aws_get_credentials_file_path(allocator, &options.credentials_file_name_override);
    ASSERT_NOT_NULL(credentials_file_path);
    config_file_path = aws_get_config_file_path(allocator, &options.config_file_name_override);
    ASSERT_NOT_NULL(config_file_path);
    profile_name = aws_get_profile_name(allocator, &options.profile_name_override);
    ASSERT_NOT_NULL(profile_name);
    config_profiles = aws_profile_collection_new_from_file(allocator, config_file_path, AWS_PST_CONFIG);
    ASSERT_NOT_NULL(config_profiles);
    credentials_profiles = aws_profile_collection_new_from_file(allocator, credentials_file_path, AWS_PST_CREDENTIALS);
    ASSERT_NOT_NULL(credentials_profiles);
    merged_profiles = aws_profile_collection_new_from_merge(allocator, config_profiles, credentials_profiles);
    ASSERT_NOT_NULL(merged_profiles);
    aws_profile_collection_release(config_profiles);
    aws_profile_collection_release(credentials_profiles);

    options.profile_collection_cached = merged_profiles;

    s_aws_credentials_shutdown_checker_init();

    /* Update profile and config file */
    if (aws_create_profile_file(config_file_str, s_config_contents2) ||
        aws_create_profile_file(creds_file_str, s_credentials_contents2)) {
        return AWS_OP_ERR;
    }

    struct aws_credentials_provider *provider = aws_credentials_provider_new_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);

    ASSERT_SUCCESS(
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));
    aws_wait_on_credentials_callback(&callback_results);
    ASSERT_SUCCESS(s_verify_default_credentials_callback(&callback_results));

    aws_get_credentials_test_callback_result_clean_up(&callback_results);

    /* Fetch the credentials again */
    aws_get_credentials_test_callback_result_init(&callback_results, 1);

    ASSERT_SUCCESS(
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));
    aws_wait_on_credentials_callback(&callback_results);

    /* assert that credentials are not changed */
    ASSERT_SUCCESS(s_verify_default_credentials_callback(&callback_results));

    aws_get_credentials_test_callback_result_clean_up(&callback_results);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();
    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);
    aws_profile_collection_release(merged_profiles);
    aws_string_destroy(credentials_file_path);
    aws_string_destroy(config_file_path);
    aws_string_destroy(profile_name);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(profile_credentials_provider_cached_test, s_profile_credentials_provider_cached_test);

AWS_STATIC_STRING_FROM_LITERAL(s_foo_profile, "foo");

AWS_STATIC_STRING_FROM_LITERAL(s_foo_access, "foo_access");
AWS_STATIC_STRING_FROM_LITERAL(s_foo_secret, "foo_secret");
AWS_STATIC_STRING_FROM_LITERAL(s_foo_session, "foo_session");

int s_verify_nondefault_credentials_callback(struct aws_get_credentials_test_callback_result *callback_results) {
    ASSERT_TRUE(callback_results->count == 1);
    ASSERT_TRUE(callback_results->credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(callback_results->credentials), s_foo_access);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(callback_results->credentials), s_foo_secret);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(callback_results->credentials), s_foo_session);

    return AWS_OP_SUCCESS;
}

static int s_profile_credentials_provider_nondefault_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    struct aws_credentials_provider_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .credentials_file_name_override = aws_byte_cursor_from_string(creds_file_str),
        .profile_name_override = aws_byte_cursor_from_string(s_foo_profile),
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    ASSERT_SUCCESS(s_do_credentials_provider_profile_test(
        allocator,
        config_file_str,
        s_config_contents,
        creds_file_str,
        s_credentials_contents,
        &options,
        s_verify_nondefault_credentials_callback,
        true));

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(profile_credentials_provider_nondefault_test, s_profile_credentials_provider_nondefault_test);

static int s_profile_credentials_provider_environment_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    /*
     * Force a profile switch via environment variable
     */
    aws_set_environment_value(s_default_profile_env_variable_name, s_foo_profile);

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_string *creds_file_str = aws_create_process_unique_file_name(allocator);

    /*
     * Redirect config and credentials files by environment
     */
    aws_set_environment_value(s_default_config_path_env_variable_name, config_file_str);
    aws_set_environment_value(s_default_credentials_path_env_variable_name, creds_file_str);

    struct aws_credentials_provider_profile_options options = {
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    ASSERT_SUCCESS(s_do_credentials_provider_profile_test(
        allocator,
        config_file_str,
        s_config_contents,
        creds_file_str,
        s_credentials_contents,
        &options,
        s_verify_nondefault_credentials_callback,
        false));

    aws_string_destroy(config_file_str);
    aws_string_destroy(creds_file_str);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(profile_credentials_provider_environment_test, s_profile_credentials_provider_environment_test);

AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_value1, "Access1");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_value1, "Secret1");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_value1, "Session1");

AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_value2, "Access2");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_value2, "Secret2");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_value2, "Session2");

static int s_do_provider_chain_test(
    struct aws_allocator *allocator,
    struct aws_credentials_provider *provider1,
    struct aws_credentials_provider *provider2,
    s_verify_credentials_callback_fn verifier) {

    s_aws_credentials_shutdown_checker_init();

    struct aws_credentials_provider *providers[2] = {provider1, provider2};

    struct aws_credentials_provider_chain_options options;
    AWS_ZERO_STRUCT(options);
    options.providers = providers;
    options.provider_count = 2;
    options.shutdown_options.shutdown_callback = s_on_shutdown_complete;
    options.shutdown_options.shutdown_user_data = NULL;

    struct aws_credentials_provider *provider_chain = aws_credentials_provider_new_chain(allocator, &options);
    aws_credentials_provider_release(provider1);
    aws_credentials_provider_release(provider2);
    if (provider_chain == NULL) {
        return 0;
    }

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);

    int get_async_result = aws_credentials_provider_get_credentials(
        provider_chain, aws_test_get_credentials_async_callback, &callback_results);

    int verification_result = AWS_OP_ERR;
    if (get_async_result == AWS_OP_SUCCESS) {
        aws_wait_on_credentials_callback(&callback_results);

        verification_result = verifier(&callback_results);
    }

    aws_get_credentials_test_callback_result_clean_up(&callback_results);

    aws_credentials_provider_release(provider_chain);

    s_aws_wait_for_provider_shutdown_callback();

    s_aws_credentials_shutdown_checker_clean_up();

    return verification_result;
}

int s_verify_first_credentials_callback(struct aws_get_credentials_test_callback_result *callback_results) {
    ASSERT_TRUE(callback_results->count == 1);
    ASSERT_TRUE(callback_results->credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_access_key_id(callback_results->credentials), s_access_key_id_value1);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_secret_access_key(callback_results->credentials), s_secret_access_key_value1);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(
        aws_credentials_get_session_token(callback_results->credentials), s_session_token_value1);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_first_in_chain_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials_provider_static_options options1 = {
        .access_key_id = aws_byte_cursor_from_string(s_access_key_id_value1),
        .secret_access_key = aws_byte_cursor_from_string(s_secret_access_key_value1),
        .session_token = aws_byte_cursor_from_string(s_session_token_value1),
    };

    struct aws_credentials_provider_static_options options2 = {
        .access_key_id = aws_byte_cursor_from_string(s_access_key_id_value2),
        .secret_access_key = aws_byte_cursor_from_string(s_secret_access_key_value2),
        .session_token = aws_byte_cursor_from_string(s_session_token_value2),
    };

    return s_do_provider_chain_test(
        allocator,
        aws_credentials_provider_new_static(allocator, &options1),
        aws_credentials_provider_new_static(allocator, &options2),
        s_verify_first_credentials_callback);
}

AWS_TEST_CASE(credentials_provider_first_in_chain_test, s_credentials_provider_first_in_chain_test);

AWS_STATIC_STRING_FROM_LITERAL(s_access2, "Access2");
AWS_STATIC_STRING_FROM_LITERAL(s_secret2, "Secret2");
AWS_STATIC_STRING_FROM_LITERAL(s_session2, "Session2");

int s_verify_second_credentials_callback(struct aws_get_credentials_test_callback_result *callback_results) {
    ASSERT_TRUE(callback_results->count == 1);
    ASSERT_TRUE(callback_results->credentials != NULL);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_access_key_id(callback_results->credentials), s_access2);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_secret_access_key(callback_results->credentials), s_secret2);
    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_session_token(callback_results->credentials), s_session2);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_second_in_chain_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials_provider_shutdown_options null_options;
    AWS_ZERO_STRUCT(null_options);

    struct aws_credentials_provider_static_options options = {
        .access_key_id = aws_byte_cursor_from_string(s_access_key_id_value2),
        .secret_access_key = aws_byte_cursor_from_string(s_secret_access_key_value2),
        .session_token = aws_byte_cursor_from_string(s_session_token_value2),
    };

    return s_do_provider_chain_test(
        allocator,
        aws_credentials_provider_new_null(allocator, &null_options),
        aws_credentials_provider_new_static(allocator, &options),
        s_verify_second_credentials_callback);
}

AWS_TEST_CASE(credentials_provider_second_in_chain_test, s_credentials_provider_second_in_chain_test);

int s_verify_null_credentials_callback(struct aws_get_credentials_test_callback_result *callback_results) {
    ASSERT_TRUE(callback_results->count == 1);
    ASSERT_TRUE(callback_results->credentials == NULL);

    return AWS_OP_SUCCESS;
}

static int s_credentials_provider_null_chain_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials_provider_shutdown_options null_options;
    AWS_ZERO_STRUCT(null_options);

    return s_do_provider_chain_test(
        allocator,
        aws_credentials_provider_new_null(allocator, &null_options),
        aws_credentials_provider_new_null(allocator, &null_options),
        s_verify_null_credentials_callback);
}

AWS_TEST_CASE(credentials_provider_null_chain_test, s_credentials_provider_null_chain_test);

static int s_credentials_provider_default_test(struct aws_allocator *allocator, bool manual_tls) {
    aws_auth_library_init(allocator);

    s_aws_credentials_shutdown_checker_init();
    /*
     * Do a basic environment provider test, but use the default provider chain
     */

    aws_set_environment_value(s_access_key_id_env_var, s_access_key_id_test_value);
    aws_set_environment_value(s_secret_access_key_env_var, s_secret_access_key_test_value);
    aws_set_environment_value(s_session_token_env_var, s_session_token_test_value);

    struct aws_event_loop_group *el_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = el_group,
        .max_entries = 4,
    };
    struct aws_host_resolver *resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .host_resolver = resolver,
        .on_shutdown_complete = NULL,
        .host_resolution_config = NULL,
        .user_data = NULL,
        .event_loop_group = el_group,
    };

    struct aws_client_bootstrap *bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);
    ASSERT_NOT_NULL(bootstrap);

    struct aws_tls_ctx *tls_ctx = NULL;
    if (manual_tls) {
        struct aws_tls_ctx_options tls_options;
        aws_tls_ctx_options_init_default_client(&tls_options, allocator);
        tls_ctx = aws_tls_client_ctx_new(allocator, &tls_options);
        ASSERT_NOT_NULL(tls_ctx);
        aws_tls_ctx_options_clean_up(&tls_options);
    }

    struct aws_credentials_provider_chain_default_options options = {
        .bootstrap = bootstrap,
        .tls_ctx = tls_ctx,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_chain_default(allocator, &options);

    /* release tls_ctx early to prove that provider acquired a reference */
    if (tls_ctx) {
        aws_tls_ctx_release(tls_ctx);
        tls_ctx = NULL;
    }

    ASSERT_TRUE(
        s_do_basic_provider_test(
            provider,
            1,
            s_access_key_id_test_value,
            s_secret_access_key_test_value,
            s_session_token_test_value,
            NULL) == AWS_OP_SUCCESS);

    /*
     * Verify that there's some caching before the environment by modifying the environment and requerying
     */
    aws_set_environment_value(s_access_key_id_env_var, s_access_key_id_1);
    aws_set_environment_value(s_secret_access_key_env_var, s_secret_access_key_1);
    aws_set_environment_value(s_session_token_env_var, s_session_token_1);

    ASSERT_TRUE(
        s_do_basic_provider_test(
            provider,
            1,
            s_access_key_id_test_value,
            s_secret_access_key_test_value,
            s_session_token_test_value,
            NULL) == AWS_OP_SUCCESS);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();
    s_aws_credentials_shutdown_checker_clean_up();
    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);
    aws_event_loop_group_release(el_group);

    aws_auth_library_clean_up();

    return 0;
}

static int s_credentials_provider_default_basic_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_credentials_provider_default_test(allocator, false /*manual_tls*/);
}

AWS_TEST_CASE(credentials_provider_default_basic_test, s_credentials_provider_default_basic_test);

static int s_credentials_provider_default_manual_tls_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_credentials_provider_default_test(allocator, true /*manual_tls*/);
}
AWS_TEST_CASE(credentials_provider_default_manual_tls_test, s_credentials_provider_default_manual_tls_test);

static int s_credentials_provider_default_chain_disable_environment_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);

    s_aws_credentials_shutdown_checker_init();
    /*
     * Set the environment variable values, but make sure they are not used when environment credentials provider is
     * disabled.
     */
    aws_set_environment_value(s_access_key_id_env_var, s_access_key_id_test_value);
    aws_set_environment_value(s_secret_access_key_env_var, s_secret_access_key_test_value);
    aws_set_environment_value(s_session_token_env_var, s_session_token_test_value);

    struct aws_event_loop_group *el_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = el_group,
        .max_entries = 4,
    };
    struct aws_host_resolver *resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .host_resolver = resolver,
        .on_shutdown_complete = NULL,
        .host_resolution_config = NULL,
        .user_data = NULL,
        .event_loop_group = el_group,
    };

    struct aws_client_bootstrap *bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);
    ASSERT_NOT_NULL(bootstrap);

    struct aws_byte_buf config_profile_collection_buf;
    AWS_ZERO_STRUCT(config_profile_collection_buf);
    /* Override profile with an empty buffer to prevent sourcing valid credentials from the profile */
    struct aws_profile_collection *config_profile_collection =
        aws_profile_collection_new_from_buffer(allocator, &config_profile_collection_buf, AWS_PST_CONFIG);
    ASSERT_NOT_NULL(config_profile_collection);

    struct aws_credentials_provider_chain_default_options options = {
        .bootstrap = bootstrap,
        .shutdown_options =
            {
                .shutdown_callback = s_on_shutdown_complete,
                .shutdown_user_data = NULL,
            },
        .skip_environment_credentials_provider = true,
        .profile_collection_cached = config_profile_collection,
    };

    struct aws_credentials_provider *provider = aws_credentials_provider_new_chain_default(allocator, &options);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);

    int get_async_result =
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results);
    ASSERT_TRUE(get_async_result == AWS_OP_SUCCESS);

    aws_wait_on_credentials_callback(&callback_results);
    /* Assert that no credentials were sourced from the environment */
    ASSERT_NULL(callback_results.credentials);
    ASSERT_TRUE(callback_results.last_error != AWS_OP_SUCCESS);
    aws_get_credentials_test_callback_result_clean_up(&callback_results);

    aws_credentials_provider_release(provider);

    s_aws_wait_for_provider_shutdown_callback();
    s_aws_credentials_shutdown_checker_clean_up();
    aws_client_bootstrap_release(bootstrap);
    aws_host_resolver_release(resolver);
    aws_event_loop_group_release(el_group);
    aws_profile_collection_release(config_profile_collection);
    aws_auth_library_clean_up();

    return 0;
}

AWS_TEST_CASE(
    credentials_provider_default_chain_disable_environment_test,
    s_credentials_provider_default_chain_disable_environment_test);
