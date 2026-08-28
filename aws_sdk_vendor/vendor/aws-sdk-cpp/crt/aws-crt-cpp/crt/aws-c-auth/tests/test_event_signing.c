/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/aws_signing.h>
#include <aws/auth/signable.h>
#include <aws/auth/signing.h>
#include <aws/auth/signing_config.h>
#include <aws/auth/signing_result.h>
#include <aws/cal/ecc.h>
#include <aws/common/condition_variable.h>
#include <aws/common/encoding.h>
#include <aws/common/mutex.h>
#include <aws/common/string.h>
#include <aws/http/connection.h>
#include <aws/http/request_response.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/socket.h>
#include <aws/io/stream.h>

AWS_STATIC_STRING_FROM_LITERAL(s_event_access_key_id, "access");
AWS_STATIC_STRING_FROM_LITERAL(s_event_secret_access_key, "secret");
AWS_STATIC_STRING_FROM_LITERAL(s_event_test_region, "us-east-1");
AWS_STATIC_STRING_FROM_LITERAL(s_event_test_service, "demo");
AWS_STATIC_STRING_FROM_LITERAL(s_event_request_date, "Fri, 16 Jan 1981 06:30:00 GMT");
AWS_STATIC_STRING_FROM_LITERAL(s_event_test_date, "Fri, 16 Jan 1981 06:30:01 GMT");
AWS_STATIC_STRING_FROM_LITERAL(s_event_test_date_2, "Fri, 16 Jan 1981 06:30:02 GMT");
AWS_STATIC_STRING_FROM_LITERAL(s_event_test_date_3, "Fri, 16 Jan 1981 06:30:03 GMT");
AWS_STATIC_STRING_FROM_LITERAL(s_event_test_date_4, "Fri, 16 Jan 1981 06:30:04 GMT");

static struct aws_http_header s_host_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("host"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("demo.us-east-1.amazonaws.com"),
};

static struct aws_http_header s_content_encoding_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Content-Encoding"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("application/vnd.amazon.eventstream"),
};

AWS_STATIC_STRING_FROM_LITERAL(s_event_test_path, "/streaming");

static struct aws_http_message *s_build_event_request(struct aws_allocator *allocator) {
    struct aws_http_message *request = aws_http_message_new_request(allocator);

    aws_http_message_add_header(request, s_host_header);
    aws_http_message_add_header(request, s_content_encoding_header);
    aws_http_message_set_request_method(request, aws_http_method_post);
    aws_http_message_set_request_path(request, aws_byte_cursor_from_string(s_event_test_path));

    return request;
}

static int s_initialize_request_signing_config(
    struct aws_signing_config_aws *config,
    struct aws_credentials *credentials) {
    config->config_type = AWS_SIGNING_CONFIG_AWS;
    config->algorithm = AWS_SIGNING_ALGORITHM_V4;
    config->signature_type = AWS_ST_HTTP_REQUEST_HEADERS;
    config->region = aws_byte_cursor_from_string(s_event_test_region);
    config->service = aws_byte_cursor_from_string(s_event_test_service);

    struct aws_byte_cursor event_test_date_cursor = aws_byte_cursor_from_string(s_event_request_date);
    if (aws_date_time_init_from_str_cursor(&config->date, &event_test_date_cursor, AWS_DATE_FORMAT_RFC822)) {
        return AWS_OP_ERR;
    }

    config->flags.use_double_uri_encode = false;
    config->flags.should_normalize_uri_path = true;
    config->signed_body_value = g_aws_signed_body_value_streaming_aws4_hmac_sha256_events;
    config->signed_body_header = AWS_SBHT_X_AMZ_CONTENT_SHA256;
    config->credentials = credentials;

    return AWS_OP_SUCCESS;
}

static int s_initialize_event_signing_config(
    struct aws_signing_config_aws *config,
    struct aws_credentials *credentials) {
    config->config_type = AWS_SIGNING_CONFIG_AWS;
    config->algorithm = AWS_SIGNING_ALGORITHM_V4;
    config->signature_type = AWS_ST_HTTP_REQUEST_EVENT;
    config->region = aws_byte_cursor_from_string(s_event_test_region);
    config->service = aws_byte_cursor_from_string(s_event_test_service);

    config->flags.use_double_uri_encode = false;
    config->flags.should_normalize_uri_path = true;
    config->signed_body_header = AWS_SBHT_NONE;
    config->credentials = credentials;

    return AWS_OP_SUCCESS;
}

struct event_signing_tester {
    struct aws_credentials *credentials;
    struct aws_http_message *request;
    struct aws_signable *request_signable;
    struct aws_signing_config_aws request_signing_config;
    struct aws_signing_config_aws event_signing_config;
    struct aws_byte_buf chunk1;
    struct aws_byte_buf chunk2;
    struct aws_byte_buf chunk3;

    struct aws_input_stream *chunk1_stream;
    struct aws_input_stream *chunk2_stream;
    struct aws_input_stream *chunk3_stream;

    struct aws_mutex mutex;
    bool request_completed;

    struct aws_byte_buf request_authorization_header;
    struct aws_byte_buf last_signature;
};

#define EVENT_SIZE 1

static int s_event_signing_tester_init(struct aws_allocator *allocator, struct event_signing_tester *tester) {
    tester->credentials =
        aws_credentials_new_from_string(allocator, s_event_access_key_id, s_event_secret_access_key, NULL, UINT64_MAX);

    tester->request = s_build_event_request(allocator);
    tester->request_signable = aws_signable_new_http_request(allocator, tester->request);

    AWS_ZERO_STRUCT(tester->request_signing_config);
    ASSERT_SUCCESS(s_initialize_request_signing_config(&tester->request_signing_config, tester->credentials));
    ASSERT_SUCCESS(s_initialize_event_signing_config(&tester->event_signing_config, tester->credentials));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->request_authorization_header, allocator, 512));
    ASSERT_SUCCESS(aws_byte_buf_init(&tester->last_signature, allocator, 128));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->chunk1, allocator, EVENT_SIZE));
    ASSERT_TRUE(aws_byte_buf_write_u8_n(&tester->chunk1, 'A', EVENT_SIZE));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->chunk2, allocator, EVENT_SIZE));
    ASSERT_TRUE(aws_byte_buf_write_u8_n(&tester->chunk2, 'B', EVENT_SIZE));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->chunk3, allocator, EVENT_SIZE));
    ASSERT_TRUE(aws_byte_buf_write_u8_n(&tester->chunk3, 'C', EVENT_SIZE));

    struct aws_byte_cursor chunk1_cursor = aws_byte_cursor_from_buf(&tester->chunk1);
    tester->chunk1_stream = aws_input_stream_new_from_cursor(allocator, &chunk1_cursor);

    struct aws_byte_cursor chunk2_cursor = aws_byte_cursor_from_buf(&tester->chunk2);
    tester->chunk2_stream = aws_input_stream_new_from_cursor(allocator, &chunk2_cursor);

    struct aws_byte_cursor chunk3_cursor = aws_byte_cursor_from_buf(&tester->chunk3);
    tester->chunk3_stream = aws_input_stream_new_from_cursor(allocator, &chunk3_cursor);

    aws_mutex_init(&tester->mutex);
    tester->request_completed = false;

    return AWS_OP_SUCCESS;
}

static void s_event_signing_tester_cleanup(struct event_signing_tester *tester) {
    aws_signable_destroy(tester->request_signable);
    aws_http_message_release(tester->request);
    aws_credentials_release(tester->credentials);
    aws_byte_buf_clean_up(&tester->request_authorization_header);
    aws_byte_buf_clean_up(&tester->last_signature);
    aws_byte_buf_clean_up(&tester->chunk1);
    aws_byte_buf_clean_up(&tester->chunk2);
    aws_byte_buf_clean_up(&tester->chunk3);

    aws_input_stream_destroy(tester->chunk1_stream);
    aws_input_stream_destroy(tester->chunk2_stream);
    aws_input_stream_destroy(tester->chunk3_stream);

    aws_mutex_clean_up(&tester->mutex);
}

static void s_on_request_signing_complete(struct aws_signing_result *result, int error_code, void *userdata) {
    AWS_FATAL_ASSERT(error_code == 0);
    struct event_signing_tester *tester = userdata;

    struct aws_array_list *headers = NULL;
    aws_signing_result_get_property_list(result, g_aws_http_headers_property_list_name, &headers);

    struct aws_byte_cursor auth_header_name = aws_byte_cursor_from_string(g_aws_signing_authorization_header_name);
    struct aws_byte_cursor auth_header_value;
    AWS_ZERO_STRUCT(auth_header_value);
    for (size_t i = 0; i < aws_array_list_length(headers); ++i) {
        struct aws_signing_result_property pair;
        AWS_ZERO_STRUCT(pair);
        if (aws_array_list_get_at(headers, &pair, i)) {
            continue;
        }

        if (pair.name == NULL) {
            continue;
        }

        struct aws_byte_cursor pair_name_cursor = aws_byte_cursor_from_string(pair.name);
        if (aws_byte_cursor_eq_ignore_case(&pair_name_cursor, &auth_header_name)) {
            auth_header_value = aws_byte_cursor_from_string(pair.value);
            break;
        }
    }
    aws_byte_buf_append_dynamic(&tester->request_authorization_header, &auth_header_value);

    struct aws_string *signature = NULL;
    aws_signing_result_get_property(result, g_aws_signature_property_name, &signature);

    struct aws_byte_cursor signature_cursor = aws_byte_cursor_from_string(signature);
    aws_byte_buf_append_dynamic(&tester->last_signature, &signature_cursor);
}

static void s_on_event_signing_complete(struct aws_signing_result *result, int error_code, void *userdata) {
    (void)error_code;

    struct event_signing_tester *tester = userdata;

    tester->last_signature.len = 0;

    struct aws_string *signature = NULL;
    aws_signing_result_get_property(result, g_aws_signature_property_name, &signature);

    struct aws_byte_cursor signature_cursor = aws_byte_cursor_from_string(signature);
    aws_byte_buf_append_dynamic(&tester->last_signature, &signature_cursor);
}

AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_request_authorization_header,
    "AWS4-HMAC-SHA256 Credential=access/19810116/us-east-1/demo/aws4_request, "
    "SignedHeaders=content-encoding;host;x-amz-content-sha256;x-amz-date, "
    "Signature=e1d8e8c8815e60969f2a34765c9a15945ffc0badbaa4b7e3b163ea19131e949b");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_request_signature,
    "e1d8e8c8815e60969f2a34765c9a15945ffc0badbaa4b7e3b163ea19131e949b");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_first_chunk_signature,
    "7aabf85b765e6a4d0d500b6e968657b14726fa3e1eb7e839302728ffd77629a5");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_second_chunk_signature,
    "f72aa9642f571d24a6e1ae42f10f073ad9448d8a028b6bcd82da081335adda02");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_third_chunk_signature,
    "632af120435b57ec241d8bfbb12e496dfd5e2730a1a02ac0ab6eaa230ae02e9a");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_final_chunk_signature,
    "c6f679ddb3af68f5e82f0cf6761244cb2338cf11e7d01a24130aea1b7c17e53e");

static int s_sigv4_event_signing_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct event_signing_tester tester;
    AWS_ZERO_STRUCT(tester);
    ASSERT_SUCCESS(s_event_signing_tester_init(allocator, &tester));

    /* Sign the base request and check the signature and authorization header */
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        tester.request_signable,
        (void *)&tester.request_signing_config,
        s_on_request_signing_complete,
        &tester));

    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_request_authorization_header->bytes,
        s_expected_request_authorization_header->len,
        tester.request_authorization_header.buffer,
        tester.request_authorization_header.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_request_signature->bytes,
        s_expected_request_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);

    /* Make and sign the first chunk */
    struct aws_signable *first_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk1_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    struct aws_byte_cursor event_test_date_cursor = aws_byte_cursor_from_string(s_event_test_date);
    ASSERT_SUCCESS(aws_date_time_init_from_str_cursor(
        &tester.event_signing_config.date, &event_test_date_cursor, AWS_DATE_FORMAT_RFC822));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, first_chunk_signable, (void *)&tester.event_signing_config, s_on_event_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_first_chunk_signature->bytes,
        s_expected_first_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(first_chunk_signable);

    /* Make and sign the second chunk */
    struct aws_signable *second_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk2_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    event_test_date_cursor = aws_byte_cursor_from_string(s_event_test_date_2);
    ASSERT_SUCCESS(aws_date_time_init_from_str_cursor(
        &tester.event_signing_config.date, &event_test_date_cursor, AWS_DATE_FORMAT_RFC822));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, second_chunk_signable, (void *)&tester.event_signing_config, s_on_event_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_second_chunk_signature->bytes,
        s_expected_second_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(second_chunk_signable);

    /* Make and sign the third chunk */
    struct aws_signable *third_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk3_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    event_test_date_cursor = aws_byte_cursor_from_string(s_event_test_date_3);
    ASSERT_SUCCESS(aws_date_time_init_from_str_cursor(
        &tester.event_signing_config.date, &event_test_date_cursor, AWS_DATE_FORMAT_RFC822));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, third_chunk_signable, (void *)&tester.event_signing_config, s_on_event_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_third_chunk_signature->bytes,
        s_expected_third_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(third_chunk_signable);

    /* Make and sign the final, empty chunk */
    struct aws_signable *final_chunk_signable =
        aws_signable_new_chunk(allocator, NULL, aws_byte_cursor_from_buf(&tester.last_signature));
    event_test_date_cursor = aws_byte_cursor_from_string(s_event_test_date_4);
    ASSERT_SUCCESS(aws_date_time_init_from_str_cursor(
        &tester.event_signing_config.date, &event_test_date_cursor, AWS_DATE_FORMAT_RFC822));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, final_chunk_signable, (void *)&tester.event_signing_config, s_on_event_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_final_chunk_signature->bytes,
        s_expected_final_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(final_chunk_signable);

    s_event_signing_tester_cleanup(&tester);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sigv4_event_signing_test, s_sigv4_event_signing_test);
