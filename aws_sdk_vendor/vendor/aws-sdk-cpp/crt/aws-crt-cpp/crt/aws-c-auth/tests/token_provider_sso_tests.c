/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/private/sso_token_providers.h>
#include <aws/auth/private/sso_token_utils.h>
#include <aws/common/clock.h>
#include <aws/common/environment.h>
#include <aws/common/file.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/tls_channel_handler.h>
#include <aws/sdkutils/aws_profile.h>

#include <credentials_provider_utils.h>

#include "shared_credentials_test_definitions.h"

struct sso_session_profile_example {
    const char *name;
    struct aws_byte_cursor text;
};

static int s_sso_token_provider_profile_invalid_profile_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    const struct sso_session_profile_example invalid_profile_examples[] = {
        {
            .name = "No config",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
                "[profile default]\naws_access_key_id=fake_access_key\naws_secret_access_key=fake_secret_key\n"),
        },
        {
            .name = "No sso_start_url",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[profile "
                                                          "default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_region=us-east-1\n"),
        },
        {
            .name = "only sso_session",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[profile "
                                                          "default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_session=dev\n[sso-session "
                                                          "dev]\nsso_start_url=url\nsso_region=us-east-1"),
        },
    };

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_token_provider_sso_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
    };

    for (size_t i = 0; i < AWS_ARRAY_SIZE(invalid_profile_examples); ++i) {
        printf("invalid example [%zu]: %s\n", i, invalid_profile_examples[i].name);
        struct aws_string *config_contents = aws_string_new_from_cursor(allocator, &invalid_profile_examples[i].text);
        ASSERT_SUCCESS(aws_create_profile_file(config_file_str, config_contents));
        ASSERT_NULL(aws_token_provider_new_sso_profile(allocator, &options));
        aws_string_destroy(config_contents);
    }

    aws_string_destroy(config_file_str);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_profile_invalid_profile_test, s_sso_token_provider_profile_invalid_profile_test);

static int s_sso_token_provider_profile_valid_profile_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);
    static struct sso_session_profile_example s_valid_profile_examples[] = {
        {
            .name = "profile",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_region=us-east-"
                                                          "1\nsso_start_url=url"),
        },
        {
            .name = "with sso_session",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
                "[default]\naws_access_key_id=fake_access_key\naws_secret_"
                "access_key=fake_secret_key\nsso_region=us-east-1\nsso_start_url=url\nsso_"
                "session=dev\n[sso-session dev]\nsso_region=us-east-"
                "1\nsso_start_url=url2"),
        },
    };

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_token_provider_sso_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
    };

    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_valid_profile_examples); ++i) {
        printf("valid example [%zu]: %s\n", i, s_valid_profile_examples[i].name);
        struct aws_string *config_contents = aws_string_new_from_cursor(allocator, &s_valid_profile_examples[i].text);
        ASSERT_SUCCESS(aws_create_profile_file(config_file_str, config_contents));
        struct aws_credentials_provider *provider = aws_token_provider_new_sso_profile(allocator, &options);
        ASSERT_NOT_NULL(provider);
        aws_credentials_provider_release(provider);
        aws_string_destroy(config_contents);
    }

    aws_string_destroy(config_file_str);
    aws_auth_library_clean_up();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_profile_valid_profile_test, s_sso_token_provider_profile_valid_profile_test);

static struct aws_mock_token_provider_sso_tester {
    struct aws_tls_ctx *tls_ctx;
    struct aws_event_loop_group *el_group;
    struct aws_host_resolver *resolver;
    struct aws_client_bootstrap *bootstrap;

    struct aws_mutex lock;
    struct aws_condition_variable signal;
    struct aws_credentials *credentials;
    bool has_received_credentials_callback;
    bool has_received_shutdown_callback;
    int error_code;

} s_tester;

static int s_aws_mock_token_provider_sso_tester_init(struct aws_allocator *allocator) {
    aws_auth_library_init(allocator);

    AWS_ZERO_STRUCT(s_tester);

    struct aws_tls_ctx_options tls_ctx_options;
    aws_tls_ctx_options_init_default_client(&tls_ctx_options, allocator);
    s_tester.tls_ctx = aws_tls_client_ctx_new(allocator, &tls_ctx_options);
    ASSERT_NOT_NULL(s_tester.tls_ctx);

    s_tester.el_group = aws_event_loop_group_new_default(allocator, 0, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = s_tester.el_group,
        .max_entries = 8,
    };
    s_tester.resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = s_tester.el_group,
        .host_resolver = s_tester.resolver,
    };
    s_tester.bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    return AWS_OP_SUCCESS;
}

void s_aws_mock_token_provider_sso_tester_cleanup(void) {
    aws_tls_ctx_release(s_tester.tls_ctx);
    aws_client_bootstrap_release(s_tester.bootstrap);
    aws_host_resolver_release(s_tester.resolver);
    aws_event_loop_group_release(s_tester.el_group);

    aws_condition_variable_clean_up(&s_tester.signal);
    aws_mutex_clean_up(&s_tester.lock);
    aws_credentials_release(s_tester.credentials);
    aws_auth_library_clean_up();
}

static int s_sso_token_provider_sso_session_invalid_config_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);
    const struct sso_session_profile_example invalid_config_examples[] = {
        {
            .name = "no sso-session",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_region=us-east-"
                                                          "1\nsso_start_url=url"),
        },
        {
            .name = "sso_session with without sso_region",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[profile "
                                                          "default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_session=dev\n[sso-session "
                                                          "dev]\nsso_start_url=url"),
        },
        {
            .name = "sso_session with without sso_start_url",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[profile "
                                                          "default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_session=dev\n[sso-session "
                                                          "dev]\nsso_region=us-east-1"),
        },
        {
            .name = "sso_session with different profile region",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
                "[profile "
                "default]\naws_access_key_id=fake_access_key\naws_secret_"
                "access_key=fake_secret_key\nsso_session=dev\nsso_region=us-west-"
                "1\nsso_start_url=url\n[sso-session dev]\nsso_region=us-east-1\nsso_start_url=url"),
        },
        {
            .name = "sso_session with different profile start url",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
                "[profile "
                "default]\naws_access_key_id=fake_access_key\naws_secret_"
                "access_key=fake_secret_key\nsso_session=dev\nsso_region=us-east-"
                "1\nsso_start_url=url\n[sso-session dev]\nsso_region=us-east-1\nsso_start_url=url2"),
        },
        {
            .name = "different sso_session name",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
                "[default]\naws_access_key_id=fake_access_key\naws_secret_"
                "access_key=fake_secret_key\nsso_region=us-east-1\nsso_start_url=url\nsso_"
                "session=dev\n[sso-session dev2]\nsso_region=us-east-"
                "1\nsso_start_url=url"),
        },
    };

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_token_provider_sso_session_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .tls_ctx = s_tester.tls_ctx,
        .bootstrap = s_tester.bootstrap,
    };

    for (size_t i = 0; i < AWS_ARRAY_SIZE(invalid_config_examples); ++i) {
        printf("invalid example [%zu]: %s\n", i, invalid_config_examples[i].name);
        struct aws_string *config_contents = aws_string_new_from_cursor(allocator, &invalid_config_examples[i].text);
        ASSERT_SUCCESS(aws_create_profile_file(config_file_str, config_contents));
        ASSERT_NULL(aws_token_provider_new_sso_session(allocator, &options));
        aws_string_destroy(config_contents);
    }

    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_sso_session_invalid_config_test, s_sso_token_provider_sso_session_invalid_config_test);

static int s_sso_token_provider_sso_session_valid_config_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);

    static struct sso_session_profile_example s_valid_profile_examples[] = {
        {
            .name = "sso-session",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_"
                                                          "session=dev\n[sso-session dev]\nsso_region=us-east-"
                                                          "1\nsso_start_url=url"),
        },
        {
            .name = "with profile sso_region",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_region=us-east-1\nsso_"
                                                          "session=dev\n[sso-session dev]\nsso_region=us-east-"
                                                          "1\nsso_start_url=url"),
        },
        {
            .name = "with profile sso_start_url",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("[default]\naws_access_key_id=fake_access_key\naws_secret_"
                                                          "access_key=fake_secret_key\nsso_start_url=url\nsso_"
                                                          "session=dev\n[sso-session dev]\nsso_region=us-east-"
                                                          "1\nsso_start_url=url"),
        },
        {
            .name = "with profile sso_region and sso_start_url",
            .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
                "[default]\naws_access_key_id=fake_access_key\naws_secret_"
                "access_key=fake_secret_key\nsso_region=us-east-1\nsso_start_url=url\nsso_"
                "session=dev\n[sso-session dev]\nsso_region=us-east-"
                "1\nsso_start_url=url"),
        },
    };

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    struct aws_token_provider_sso_session_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .tls_ctx = s_tester.tls_ctx,
        .bootstrap = s_tester.bootstrap,
    };

    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_valid_profile_examples); ++i) {
        printf("valid example [%zu]: %s\n", i, s_valid_profile_examples[i].name);
        struct aws_string *config_contents = aws_string_new_from_cursor(allocator, &s_valid_profile_examples[i].text);
        ASSERT_SUCCESS(aws_create_profile_file(config_file_str, config_contents));
        struct aws_credentials_provider *provider = aws_token_provider_new_sso_session(allocator, &options);
        ASSERT_NOT_NULL(provider);
        aws_credentials_provider_release(provider);
        aws_string_destroy(config_contents);
    }

    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_sso_session_valid_config_test, s_sso_token_provider_sso_session_valid_config_test);

/* start_url should be same in `s_sso_profile_start_url` and `s_sso_profile_config_contents` */
AWS_STATIC_STRING_FROM_LITERAL(s_sso_profile_start_url, "https://d-123.awsapps.com/start");
AWS_STATIC_STRING_FROM_LITERAL(
    s_sso_profile_config_contents,
    "[default]\n"
    "sso_start_url = https://d-123.awsapps.com/start\n");
/* session name should be same in both `s_sso_session_name` and `s_sso_session_config_contents`*/
AWS_STATIC_STRING_FROM_LITERAL(s_sso_session_name, "session");
AWS_STATIC_STRING_FROM_LITERAL(
    s_sso_session_config_contents,
    "[default]\n"
    "sso_session = session\n"
    "[sso-session session]\n"
    "sso_start_url = https://d-123.awsapps.com/start\n"
    "sso_region = us-west-2\n");
AWS_STATIC_STRING_FROM_LITERAL(
    s_sso_token,
    "{\"accessToken\": \"ValidAccessToken\",\"expiresAt\": \"2015-03-12T05:35:19Z\"}");
AWS_STATIC_STRING_FROM_LITERAL(s_invalid_config, "invalid config");

AWS_STATIC_STRING_FROM_LITERAL(s_good_token, "ValidAccessToken");
static uint64_t s_token_expiration_s = 1426138519;
static int s_sso_token_provider_sso_session_basic_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);

    /* redirect $HOME */
    struct aws_string *tmp_home;
    ASSERT_SUCCESS(aws_create_random_home_directory(allocator, &tmp_home));

    /* create token file */
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, s_sso_session_name);
    ASSERT_NOT_NULL(token_path);
    ASSERT_SUCCESS(aws_create_directory_components(allocator, token_path));
    ASSERT_SUCCESS(aws_create_profile_file(token_path, s_sso_token));

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, s_sso_session_config_contents));
    mock_aws_set_system_time(0);
    struct aws_token_provider_sso_session_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .tls_ctx = s_tester.tls_ctx,
        .bootstrap = s_tester.bootstrap,
        .system_clock_fn = mock_aws_get_system_time,
    };

    struct aws_credentials_provider *provider = aws_token_provider_new_sso_session(allocator, &options);
    ASSERT_NOT_NULL(provider);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);
    ASSERT_SUCCESS(
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));
    aws_wait_on_credentials_callback(&callback_results);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_token(callback_results.credentials), s_good_token);
    ASSERT_INT_EQUALS(
        aws_credentials_get_expiration_timepoint_seconds(callback_results.credentials), s_token_expiration_s);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    aws_credentials_provider_release(provider);

    aws_directory_delete(tmp_home, true);
    aws_string_destroy(tmp_home);
    aws_string_destroy(token_path);
    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_sso_session_basic_success, s_sso_token_provider_sso_session_basic_success);

static int s_sso_token_provider_sso_session_config_file_cached(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);

    /* redirect $HOME */
    struct aws_string *tmp_home;
    ASSERT_SUCCESS(aws_create_random_home_directory(allocator, &tmp_home));

    /* create token file */
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, s_sso_session_name);
    ASSERT_NOT_NULL(token_path);
    ASSERT_SUCCESS(aws_create_directory_components(allocator, token_path));
    ASSERT_SUCCESS(aws_create_profile_file(token_path, s_sso_token));

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, s_invalid_config));

    struct aws_byte_buf profile_buffer = aws_byte_buf_from_c_str(aws_string_c_str(s_sso_session_config_contents));
    struct aws_profile_collection *config_collection =
        aws_profile_collection_new_from_buffer(allocator, &profile_buffer, AWS_PST_CONFIG);

    mock_aws_set_system_time(0);
    struct aws_token_provider_sso_session_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .config_file_cached = config_collection,
        .tls_ctx = s_tester.tls_ctx,
        .bootstrap = s_tester.bootstrap,
        .system_clock_fn = mock_aws_get_system_time,
    };

    struct aws_credentials_provider *provider = aws_token_provider_new_sso_session(allocator, &options);
    ASSERT_NOT_NULL(provider);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);
    ASSERT_SUCCESS(
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));
    aws_wait_on_credentials_callback(&callback_results);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_token(callback_results.credentials), s_good_token);
    ASSERT_INT_EQUALS(
        aws_credentials_get_expiration_timepoint_seconds(callback_results.credentials), s_token_expiration_s);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    aws_credentials_provider_release(provider);

    aws_directory_delete(tmp_home, true);
    aws_string_destroy(tmp_home);
    aws_string_destroy(token_path);
    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    aws_profile_collection_release(config_collection);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_sso_session_config_file_cached, s_sso_token_provider_sso_session_config_file_cached);

static int s_sso_token_provider_sso_session_expired_token(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);

    /* redirect $HOME */
    struct aws_string *tmp_home;
    ASSERT_SUCCESS(aws_create_random_home_directory(allocator, &tmp_home));

    /* create token file */
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, s_sso_session_name);
    ASSERT_NOT_NULL(token_path);
    ASSERT_SUCCESS(aws_create_directory_components(allocator, token_path));
    ASSERT_SUCCESS(aws_create_profile_file(token_path, s_sso_token));

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, s_sso_session_config_contents));
    uint64_t nano_expiration =
        aws_timestamp_convert(s_token_expiration_s + 1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);
    mock_aws_set_system_time(nano_expiration);
    struct aws_token_provider_sso_session_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .tls_ctx = s_tester.tls_ctx,
        .bootstrap = s_tester.bootstrap,
        .system_clock_fn = mock_aws_get_system_time,
    };

    struct aws_credentials_provider *provider = aws_token_provider_new_sso_session(allocator, &options);
    ASSERT_NOT_NULL(provider);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);
    ASSERT_ERROR(
        AWS_AUTH_SSO_TOKEN_EXPIRED,
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));
    aws_credentials_provider_release(provider);

    aws_directory_delete(tmp_home, true);
    aws_string_destroy(tmp_home);
    aws_string_destroy(token_path);
    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_sso_session_expired_token, s_sso_token_provider_sso_session_expired_token);

static int s_sso_token_provider_profile_basic_success(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);

    /* redirect $HOME */
    struct aws_string *tmp_home;
    ASSERT_SUCCESS(aws_create_random_home_directory(allocator, &tmp_home));

    /* create token file */
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, s_sso_profile_start_url);
    ASSERT_NOT_NULL(token_path);
    ASSERT_SUCCESS(aws_create_directory_components(allocator, token_path));
    ASSERT_SUCCESS(aws_create_profile_file(token_path, s_sso_token));

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, s_sso_profile_config_contents));

    mock_aws_set_system_time(0);
    struct aws_token_provider_sso_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .system_clock_fn = mock_aws_get_system_time,
    };

    struct aws_credentials_provider *provider = aws_token_provider_new_sso_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);
    ASSERT_SUCCESS(
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));
    aws_wait_on_credentials_callback(&callback_results);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_token(callback_results.credentials), s_good_token);
    ASSERT_INT_EQUALS(
        aws_credentials_get_expiration_timepoint_seconds(callback_results.credentials), s_token_expiration_s);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    aws_credentials_provider_release(provider);

    aws_directory_delete(tmp_home, true);
    aws_string_destroy(tmp_home);
    aws_string_destroy(token_path);
    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_profile_basic_success, s_sso_token_provider_profile_basic_success);
static int s_sso_token_provider_profile_cached_config_file(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);

    /* redirect $HOME */
    struct aws_string *tmp_home;
    ASSERT_SUCCESS(aws_create_random_home_directory(allocator, &tmp_home));

    /* create token file */
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, s_sso_profile_start_url);
    ASSERT_NOT_NULL(token_path);
    ASSERT_SUCCESS(aws_create_directory_components(allocator, token_path));
    ASSERT_SUCCESS(aws_create_profile_file(token_path, s_sso_token));

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, s_invalid_config));

    struct aws_byte_buf profile_buffer = aws_byte_buf_from_c_str(aws_string_c_str(s_sso_profile_config_contents));

    struct aws_profile_collection *config_collection =
        aws_profile_collection_new_from_buffer(allocator, &profile_buffer, AWS_PST_CONFIG);

    mock_aws_set_system_time(0);
    struct aws_token_provider_sso_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .config_file_cached = config_collection,
        .system_clock_fn = mock_aws_get_system_time,
    };

    struct aws_credentials_provider *provider = aws_token_provider_new_sso_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);
    ASSERT_SUCCESS(
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));
    aws_wait_on_credentials_callback(&callback_results);

    ASSERT_CURSOR_VALUE_STRING_EQUALS(aws_credentials_get_token(callback_results.credentials), s_good_token);
    ASSERT_INT_EQUALS(
        aws_credentials_get_expiration_timepoint_seconds(callback_results.credentials), s_token_expiration_s);

    aws_get_credentials_test_callback_result_clean_up(&callback_results);
    aws_credentials_provider_release(provider);

    aws_directory_delete(tmp_home, true);
    aws_string_destroy(tmp_home);
    aws_string_destroy(token_path);
    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    aws_profile_collection_release(config_collection);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_profile_cached_config_file, s_sso_token_provider_profile_cached_config_file);

static int s_sso_token_provider_profile_expired_token(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    s_aws_mock_token_provider_sso_tester_init(allocator);

    /* redirect $HOME */
    struct aws_string *tmp_home;
    ASSERT_SUCCESS(aws_create_random_home_directory(allocator, &tmp_home));

    /* create token file */
    struct aws_string *token_path = aws_construct_sso_token_path(allocator, s_sso_profile_start_url);
    ASSERT_NOT_NULL(token_path);
    ASSERT_SUCCESS(aws_create_directory_components(allocator, token_path));
    ASSERT_SUCCESS(aws_create_profile_file(token_path, s_sso_token));

    struct aws_string *config_file_str = aws_create_process_unique_file_name(allocator);
    ASSERT_SUCCESS(aws_create_profile_file(config_file_str, s_sso_profile_config_contents));

    uint64_t nano_expiration =
        aws_timestamp_convert(s_token_expiration_s + 100, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);
    mock_aws_set_system_time(nano_expiration);
    struct aws_token_provider_sso_profile_options options = {
        .config_file_name_override = aws_byte_cursor_from_string(config_file_str),
        .system_clock_fn = mock_aws_get_system_time,
    };

    struct aws_credentials_provider *provider = aws_token_provider_new_sso_profile(allocator, &options);
    ASSERT_NOT_NULL(provider);

    struct aws_get_credentials_test_callback_result callback_results;
    aws_get_credentials_test_callback_result_init(&callback_results, 1);
    ASSERT_ERROR(
        AWS_AUTH_SSO_TOKEN_EXPIRED,
        aws_credentials_provider_get_credentials(provider, aws_test_get_credentials_async_callback, &callback_results));

    aws_credentials_provider_release(provider);

    aws_directory_delete(tmp_home, true);
    aws_string_destroy(tmp_home);
    aws_string_destroy(token_path);
    aws_string_destroy(config_file_str);
    s_aws_mock_token_provider_sso_tester_cleanup();
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sso_token_provider_profile_expired_token, s_sso_token_provider_profile_expired_token);
