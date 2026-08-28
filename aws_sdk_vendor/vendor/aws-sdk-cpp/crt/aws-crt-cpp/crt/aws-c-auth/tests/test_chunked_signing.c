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

/*
 * The chunked signing test is built using the complete chunked signing example in the s3 docs:
 * https://docs.aws.amazon.com/AmazonS3/latest/API/sigv4-streaming.html
 *
 */

AWS_STATIC_STRING_FROM_LITERAL(s_integration_chunked_access_key_id, "example");
AWS_STATIC_STRING_FROM_LITERAL(s_integration_chunked_secret_access_key, "example");

AWS_STATIC_STRING_FROM_LITERAL(s_chunked_access_key_id, "AKIAIOSFODNN7EXAMPLE");
AWS_STATIC_STRING_FROM_LITERAL(s_chunked_secret_access_key, "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY");
AWS_STATIC_STRING_FROM_LITERAL(s_chunked_test_region, "us-east-1");
AWS_STATIC_STRING_FROM_LITERAL(s_chunked_test_service, "s3");
AWS_STATIC_STRING_FROM_LITERAL(s_chunked_test_date, "Fri, 24 May 2013 00:00:00 GMT");

static struct aws_http_header s_host_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("host"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("s3.amazonaws.com"),
};

static struct aws_http_header s_integration_host_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("host"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("example_bucket"),
};

static struct aws_http_header s_storage_class_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("x-amz-storage-class"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("REDUCED_REDUNDANCY"),
};

static struct aws_http_header s_content_encoding_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Content-Encoding"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("aws-chunked"),
};

static struct aws_http_header s_decoded_length_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("x-amz-decoded-content-length"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("66560"),
};

static struct aws_http_header s_integration_decoded_length_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("x-amz-decoded-content-length"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("1"),
};

static struct aws_http_header s_content_length_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Content-Length"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("66824"),
};
static struct aws_http_header s_trailer_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("x-amz-trailer"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("first,second,third"),
};

static struct aws_http_header s_integration_content_length_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("Content-Length"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("535"),
};

static struct aws_http_header s_integration_trailer_header = {
    .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("x-amz-trailer"),
    .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("x-amz-checksum-crc32c"),
};

AWS_STATIC_STRING_FROM_LITERAL(s_chunked_test_path, "/examplebucket/chunkObject.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_integration_test_path, "/john_is_the_bees_knees_1_byte");

static struct aws_http_message *s_build_chunked_test_request(struct aws_allocator *allocator) {
    struct aws_http_message *request = aws_http_message_new_request(allocator);

    aws_http_message_add_header(request, s_host_header);
    aws_http_message_add_header(request, s_storage_class_header);
    aws_http_message_add_header(request, s_content_encoding_header);
    aws_http_message_add_header(request, s_decoded_length_header);
    aws_http_message_add_header(request, s_content_length_header);
    aws_http_message_set_request_method(request, aws_http_method_put);
    aws_http_message_set_request_path(request, aws_byte_cursor_from_string(s_chunked_test_path));

    return request;
}

static struct aws_http_message *s_build_trailing_headers_test_request(struct aws_allocator *allocator) {
    struct aws_http_message *request = aws_http_message_new_request(allocator);

    aws_http_message_add_header(request, s_host_header);
    aws_http_message_add_header(request, s_storage_class_header);
    aws_http_message_add_header(request, s_content_encoding_header);
    aws_http_message_add_header(request, s_decoded_length_header);
    aws_http_message_add_header(request, s_content_length_header);
    aws_http_message_add_header(request, s_trailer_header);
    aws_http_message_set_request_method(request, aws_http_method_put);
    aws_http_message_set_request_path(request, aws_byte_cursor_from_string(s_chunked_test_path));

    return request;
}
static struct aws_http_message *s_build_integration_test_request(struct aws_allocator *allocator) {
    struct aws_http_message *request = aws_http_message_new_request(allocator);

    aws_http_message_add_header(request, s_content_encoding_header);
    aws_http_message_add_header(request, s_integration_host_header);
    aws_http_message_add_header(request, s_integration_decoded_length_header);
    aws_http_message_add_header(request, s_integration_content_length_header);
    aws_http_message_add_header(request, s_integration_trailer_header);
    aws_http_message_set_request_method(request, aws_http_method_put);
    aws_http_message_set_request_path(request, aws_byte_cursor_from_string(s_integration_test_path));

    return request;
}

static int s_initialize_request_signing_config(
    struct aws_signing_config_aws *config,
    struct aws_credentials *credentials) {
    config->config_type = AWS_SIGNING_CONFIG_AWS;
    config->algorithm = AWS_SIGNING_ALGORITHM_V4;
    config->signature_type = AWS_ST_HTTP_REQUEST_HEADERS;
    config->region = aws_byte_cursor_from_string(s_chunked_test_region);
    config->service = aws_byte_cursor_from_string(s_chunked_test_service);

    struct aws_byte_cursor chunked_test_date_cursor = aws_byte_cursor_from_string(s_chunked_test_date);
    if (aws_date_time_init_from_str_cursor(&config->date, &chunked_test_date_cursor, AWS_DATE_FORMAT_RFC822)) {
        return AWS_OP_ERR;
    }

    config->flags.use_double_uri_encode = false;
    config->flags.should_normalize_uri_path = true;
    config->signed_body_value = g_aws_signed_body_value_streaming_aws4_hmac_sha256_payload;
    config->signed_body_header = AWS_SBHT_X_AMZ_CONTENT_SHA256;
    config->credentials = credentials;

    return AWS_OP_SUCCESS;
}

static int s_initialize_chunk_signing_config(
    struct aws_signing_config_aws *config,
    struct aws_credentials *credentials) {
    config->config_type = AWS_SIGNING_CONFIG_AWS;
    config->algorithm = AWS_SIGNING_ALGORITHM_V4;
    config->signature_type = AWS_ST_HTTP_REQUEST_CHUNK;
    config->region = aws_byte_cursor_from_string(s_chunked_test_region);
    config->service = aws_byte_cursor_from_string(s_chunked_test_service);

    struct aws_byte_cursor chunked_test_date_cursor = aws_byte_cursor_from_string(s_chunked_test_date);
    if (aws_date_time_init_from_str_cursor(&config->date, &chunked_test_date_cursor, AWS_DATE_FORMAT_RFC822)) {
        return AWS_OP_ERR;
    }

    config->flags.use_double_uri_encode = false;
    config->flags.should_normalize_uri_path = true;
    config->signed_body_header = AWS_SBHT_NONE;
    config->credentials = credentials;

    return AWS_OP_SUCCESS;
}

static int s_initialize_trailing_headers_signing_config(
    struct aws_signing_config_aws *config,
    struct aws_credentials *credentials) {
    config->config_type = AWS_SIGNING_CONFIG_AWS;
    config->algorithm = AWS_SIGNING_ALGORITHM_V4;
    config->signature_type = AWS_ST_HTTP_REQUEST_TRAILING_HEADERS;
    config->region = aws_byte_cursor_from_string(s_chunked_test_region);
    config->service = aws_byte_cursor_from_string(s_chunked_test_service);

    struct aws_byte_cursor chunked_test_date_cursor = aws_byte_cursor_from_string(s_chunked_test_date);
    if (aws_date_time_init_from_str_cursor(&config->date, &chunked_test_date_cursor, AWS_DATE_FORMAT_RFC822)) {
        return AWS_OP_ERR;
    }

    config->flags.use_double_uri_encode = false;
    config->flags.should_normalize_uri_path = true;
    config->signed_body_header = AWS_SBHT_NONE;
    config->credentials = credentials;

    return AWS_OP_SUCCESS;
}

struct chunked_signing_tester {
    struct aws_credentials *credentials;
    struct aws_ecc_key_pair *verification_key;
    struct aws_http_message *request;
    struct aws_signable *request_signable;
    struct aws_http_message *integration_request;
    struct aws_signable *integration_request_signable;
    struct aws_http_message *trailing_request;
    struct aws_signable *trailing_request_signable;
    struct aws_signing_config_aws request_signing_config;
    struct aws_signing_config_aws chunk_signing_config;
    struct aws_signing_config_aws trailing_headers_signing_config;
    struct aws_byte_buf chunk1;
    struct aws_byte_buf chunk2;
    struct aws_byte_buf integration_chunk;
    struct aws_input_stream *chunk1_stream;
    struct aws_input_stream *chunk2_stream;
    struct aws_input_stream *integration_chunk_stream;
    struct aws_http_headers *trailing_headers;
    struct aws_http_headers *integration_trailing_headers;
    struct aws_mutex mutex;
    bool request_completed;
    struct aws_condition_variable c_var;

    struct aws_byte_buf request_authorization_header;
    struct aws_byte_buf last_signature;
};

#define CHUNK1_SIZE 65536
#define CHUNK2_SIZE 1024

AWS_STATIC_STRING_FROM_LITERAL(
    s_chunked_test_ecc_pub_x,
    "18b7d04643359f6ec270dcbab8dce6d169d66ddc9778c75cfb08dfdb701637ab");
AWS_STATIC_STRING_FROM_LITERAL(
    s_chunked_test_ecc_pub_y,
    "fa36b35e4fe67e3112261d2e17a956ef85b06e44712d2850bcd3c2161e9993f2");

static struct aws_http_headers *s_trailing_headers_new(struct aws_allocator *allocator) {
    struct aws_http_headers *trailing_headers = aws_http_headers_new(allocator);
    const struct aws_http_header trailer1 = {
        .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("first"),
        .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("1st"),
    };
    const struct aws_http_header trailer2 = {
        .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("second"),
        .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("2nd"),
    };
    const struct aws_http_header trailer3 = {
        .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("third"),
        .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("3rd"),
    };
    aws_http_headers_add_header(trailing_headers, &trailer1);
    aws_http_headers_add_header(trailing_headers, &trailer2);
    aws_http_headers_add_header(trailing_headers, &trailer3);
    return trailing_headers;
}

static int s_chunked_signing_tester_init(struct aws_allocator *allocator, struct chunked_signing_tester *tester) {
    tester->credentials = aws_credentials_new_from_string(
        allocator, s_chunked_access_key_id, s_chunked_secret_access_key, NULL, UINT64_MAX);
    tester->verification_key = aws_ecc_key_new_from_hex_coordinates(
        allocator,
        AWS_CAL_ECDSA_P256,
        aws_byte_cursor_from_string(s_chunked_test_ecc_pub_x),
        aws_byte_cursor_from_string(s_chunked_test_ecc_pub_y));

    tester->request = s_build_chunked_test_request(allocator);
    tester->request_signable = aws_signable_new_http_request(allocator, tester->request);

    tester->trailing_request = s_build_trailing_headers_test_request(allocator);
    tester->trailing_request_signable = aws_signable_new_http_request(allocator, tester->trailing_request);

    tester->integration_request = s_build_integration_test_request(allocator);
    tester->integration_request_signable = aws_signable_new_http_request(allocator, tester->integration_request);

    AWS_ZERO_STRUCT(tester->request_signing_config);
    ASSERT_SUCCESS(s_initialize_request_signing_config(&tester->request_signing_config, tester->credentials));
    ASSERT_SUCCESS(s_initialize_chunk_signing_config(&tester->chunk_signing_config, tester->credentials));
    ASSERT_SUCCESS(
        s_initialize_trailing_headers_signing_config(&tester->trailing_headers_signing_config, tester->credentials));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->request_authorization_header, allocator, 512));
    ASSERT_SUCCESS(aws_byte_buf_init(&tester->last_signature, allocator, 128));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->chunk1, allocator, CHUNK1_SIZE));
    ASSERT_TRUE(aws_byte_buf_write_u8_n(&tester->chunk1, 'a', CHUNK1_SIZE));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->chunk2, allocator, CHUNK2_SIZE));
    ASSERT_TRUE(aws_byte_buf_write_u8_n(&tester->chunk2, 'a', CHUNK2_SIZE));

    ASSERT_SUCCESS(aws_byte_buf_init(&tester->integration_chunk, allocator, 1));
    ASSERT_TRUE(aws_byte_buf_write_u8_n(&tester->integration_chunk, 'a', 1));

    tester->trailing_headers = s_trailing_headers_new(allocator);
    tester->integration_trailing_headers = aws_http_headers_new(allocator);
    const struct aws_http_header checksum = {
        .name = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("x-amz-checksum-crc32c"),
        .value = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("wdBDMA=="),
    };
    aws_http_headers_add_header(tester->integration_trailing_headers, &checksum);

    struct aws_byte_cursor chunk1_cursor = aws_byte_cursor_from_buf(&tester->chunk1);
    tester->chunk1_stream = aws_input_stream_new_from_cursor(allocator, &chunk1_cursor);

    struct aws_byte_cursor chunk2_cursor = aws_byte_cursor_from_buf(&tester->chunk2);
    tester->chunk2_stream = aws_input_stream_new_from_cursor(allocator, &chunk2_cursor);

    struct aws_byte_cursor integration_chunk_cursor = aws_byte_cursor_from_buf(&tester->integration_chunk);
    tester->integration_chunk_stream = aws_input_stream_new_from_cursor(allocator, &integration_chunk_cursor);
    aws_mutex_init(&tester->mutex);
    tester->c_var = (struct aws_condition_variable)AWS_CONDITION_VARIABLE_INIT;
    tester->request_completed = false;

    return AWS_OP_SUCCESS;
}

static void s_chunked_signing_tester_cleanup(struct chunked_signing_tester *tester) {
    aws_signable_destroy(tester->request_signable);
    aws_http_message_release(tester->request);
    aws_signable_destroy(tester->integration_request_signable);
    aws_signable_destroy(tester->trailing_request_signable);
    aws_http_message_release(tester->trailing_request);
    aws_http_message_release(tester->integration_request);
    aws_credentials_release(tester->credentials);
    aws_ecc_key_pair_release(tester->verification_key);
    aws_byte_buf_clean_up(&tester->request_authorization_header);
    aws_byte_buf_clean_up(&tester->last_signature);
    aws_byte_buf_clean_up(&tester->integration_chunk);
    aws_byte_buf_clean_up(&tester->chunk1);
    aws_byte_buf_clean_up(&tester->chunk2);
    aws_http_headers_release(tester->integration_trailing_headers);
    aws_http_headers_release(tester->trailing_headers);
    aws_input_stream_destroy(tester->integration_chunk_stream);
    aws_input_stream_destroy(tester->chunk1_stream);
    aws_input_stream_destroy(tester->chunk2_stream);
    aws_mutex_clean_up(&tester->mutex);
}

static void s_on_request_signing_complete(struct aws_signing_result *result, int error_code, void *userdata) {
    (void)error_code;

    struct chunked_signing_tester *tester = userdata;

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

static void s_on_integration_request_signing_complete(
    struct aws_signing_result *result,
    int error_code,
    void *userdata) {
    (void)error_code;

    struct chunked_signing_tester *tester = userdata;

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

    aws_apply_signing_result_to_http_request(tester->integration_request, aws_default_allocator(), result);
}

static void s_on_chunk_signing_complete(struct aws_signing_result *result, int error_code, void *userdata) {
    (void)error_code;

    struct chunked_signing_tester *tester = userdata;

    tester->last_signature.len = 0;

    struct aws_string *signature = NULL;
    aws_signing_result_get_property(result, g_aws_signature_property_name, &signature);

    struct aws_byte_cursor signature_cursor = aws_byte_cursor_from_string(signature);
    aws_byte_buf_append_dynamic(&tester->last_signature, &signature_cursor);
}

/* There is an error in the s3 docs where they list the authorization header value: it is missing a space between
 * the ',' and 'SignedHeaders=' as well as a space between the ',' and 'Signature='
 */
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_request_authorization_header,
    "AWS4-HMAC-SHA256 Credential=AKIAIOSFODNN7EXAMPLE/20130524/us-east-1/s3/aws4_request, "
    "SignedHeaders=content-encoding;content-length;host;x-amz-content-sha256;x-amz-date;x-amz-decoded-content-length;x-"
    "amz-storage-class, Signature=4f232c4386841ef735655705268965c44a0e4690baa4adea153f7db9fa80a0a9");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_request_signature,
    "4f232c4386841ef735655705268965c44a0e4690baa4adea153f7db9fa80a0a9");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_first_chunk_signature,
    "ad80c730a21e5b8d04586a2213dd63b9a0e99e0e2307b0ade35a65485a288648");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_second_chunk_signature,
    "0055627c9e194cb4542bae2aa5492e3c1575bbb81b612b7d234b86a503ef5497");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_final_chunk_signature,
    "b6c6ea8a5354eaf15b3cb7646744f4275b71ea724fed81ceb9323e279d449df9");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_trailing_headers_signature,
    "df5735bd9f3295cd9386572292562fefc93ba94e80a0a1ddcbd652c4e0a75e6c");

static int s_sigv4_chunked_signing_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct chunked_signing_tester tester;
    AWS_ZERO_STRUCT(tester);
    ASSERT_SUCCESS(s_chunked_signing_tester_init(allocator, &tester));

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
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, first_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_first_chunk_signature->bytes,
        s_expected_first_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(first_chunk_signable);

    /* Make and sign the second chunk */
    struct aws_signable *second_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk2_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, second_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_second_chunk_signature->bytes,
        s_expected_second_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(second_chunk_signable);

    /* Make and sign the final, empty chunk */
    struct aws_signable *final_chunk_signable =
        aws_signable_new_chunk(allocator, NULL, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, final_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_final_chunk_signature->bytes,
        s_expected_final_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(final_chunk_signable);

    s_chunked_signing_tester_cleanup(&tester);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sigv4_chunked_signing_test, s_sigv4_chunked_signing_test);

static int s_sigv4_trailing_headers_signing_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct chunked_signing_tester tester;
    AWS_ZERO_STRUCT(tester);
    ASSERT_SUCCESS(s_chunked_signing_tester_init(allocator, &tester));

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
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, first_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_first_chunk_signature->bytes,
        s_expected_first_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(first_chunk_signable);

    /* Make and sign the second chunk */
    struct aws_signable *second_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk2_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, second_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_second_chunk_signature->bytes,
        s_expected_second_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(second_chunk_signable);

    /* Make and sign the final, empty chunk */
    struct aws_signable *final_chunk_signable =
        aws_signable_new_chunk(allocator, NULL, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, final_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_final_chunk_signature->bytes,
        s_expected_final_chunk_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(final_chunk_signable);

    /* Make and sign the trailing headers */
    struct aws_http_headers *trailing_headers = s_trailing_headers_new(allocator);
    struct aws_signable *trailing_headers_signable = aws_signable_new_trailing_headers(
        allocator, trailing_headers, aws_byte_cursor_from_buf(&tester.last_signature));
    /* test aws_signable_new_trailing_headers properly acquires trailing_headers */
    aws_http_headers_release(trailing_headers);
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        trailing_headers_signable,
        (void *)&tester.trailing_headers_signing_config,
        s_on_chunk_signing_complete,
        &tester));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_trailing_headers_signature->bytes,
        s_expected_trailing_headers_signature->len,
        tester.last_signature.buffer,
        tester.last_signature.len);
    aws_signable_destroy(trailing_headers_signable);

    s_chunked_signing_tester_cleanup(&tester);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sigv4_trailing_headers_signing_test, s_sigv4_trailing_headers_signing_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_chunked_expected_canonical_request_cursor,
    "PUT\n"
    "/examplebucket/chunkObject.txt\n"
    "\n"
    "content-encoding:aws-chunked\n"
    "content-length:66824\n"
    "host:s3.amazonaws.com\n"
    "x-amz-content-sha256:STREAMING-AWS4-ECDSA-P256-SHA256-PAYLOAD\n"
    "x-amz-date:20130524T000000Z\n"
    "x-amz-decoded-content-length:66560\n"
    "x-amz-region-set:us-east-1\n"
    "x-amz-storage-class:REDUCED_REDUNDANCY\n"
    "\n"
    "content-encoding;content-length;host;x-amz-content-sha256;x-amz-date;x-amz-decoded-content-length;x-amz-region-"
    "set;x-amz-storage-class\n"
    "STREAMING-AWS4-ECDSA-P256-SHA256-PAYLOAD");

AWS_STATIC_STRING_FROM_LITERAL(
    s_chunked_expected_trailing_headers_canonical_request_cursor,
    "PUT\n"
    "/examplebucket/chunkObject.txt\n"
    "\n"
    "content-encoding:aws-chunked\n"
    "content-length:66824\n"
    "host:s3.amazonaws.com\n"
    "x-amz-content-sha256:STREAMING-AWS4-ECDSA-P256-SHA256-PAYLOAD-TRAILER\n"
    "x-amz-date:20130524T000000Z\n"
    "x-amz-decoded-content-length:66560\n"
    "x-amz-region-set:us-east-1\n"
    "x-amz-storage-class:REDUCED_REDUNDANCY\n"
    "x-amz-trailer:first,second,third\n"
    "\n"
    "content-encoding;content-length;host;x-amz-content-sha256;x-amz-date;x-amz-decoded-content-length;x-amz-region-"
    "set;x-amz-storage-class;x-amz-trailer\n"
    "STREAMING-AWS4-ECDSA-P256-SHA256-PAYLOAD-TRAILER");

AWS_STATIC_STRING_FROM_LITERAL(
    s_chunk_sts_pre_signature,
    "AWS4-ECDSA-P256-SHA256-PAYLOAD\n"
    "20130524T000000Z\n"
    "20130524/s3/aws4_request\n");

AWS_STATIC_STRING_FROM_LITERAL(
    s_chunk1_sts_post_signature,
    "\ne3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855\n"
    "bf718b6f653bebc184e1479f1935b8da974d701b893afcf49e701f3e2f9f9c5a");

AWS_STATIC_STRING_FROM_LITERAL(
    s_chunk2_sts_post_signature,
    "\ne3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855\n"
    "2edc986847e209b4016e141a6dc8716d3207350f416969382d431539bf292e4a");

AWS_STATIC_STRING_FROM_LITERAL(
    s_chunk3_sts_post_signature,
    "\ne3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855\n"
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

AWS_STATIC_STRING_FROM_LITERAL(
    s_trailing_headers_expected_sts_pre_signature,
    "AWS4-ECDSA-P256-SHA256-TRAILER\n"
    "20130524T000000Z\n"
    "20130524/s3/aws4_request\n");
AWS_STATIC_STRING_FROM_LITERAL(
    s_trailing_headers_expected_sts_post_signature,
    "\n83d8f190334fb741bc8daf73c891689d320bd8017756bc730c540021ed48001f");

static int s_sigv4a_chunked_signing_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct chunked_signing_tester tester;
    AWS_ZERO_STRUCT(tester);
    ASSERT_SUCCESS(s_chunked_signing_tester_init(allocator, &tester));
    tester.request_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;
    tester.request_signing_config.signed_body_value = g_aws_signed_body_value_streaming_aws4_ecdsa_p256_sha256_payload;
    tester.chunk_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;

    /* Sign the base request */
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        tester.request_signable,
        (void *)&tester.request_signing_config,
        s_on_request_signing_complete,
        &tester));

    struct aws_byte_cursor signature_cursor =
        aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /*
     * Validate the request signature
     */
    ASSERT_SUCCESS(aws_verify_sigv4a_signing(
        allocator,
        tester.request_signable,
        (void *)&tester.request_signing_config,
        aws_byte_cursor_from_string(s_chunked_expected_canonical_request_cursor),
        signature_cursor,
        aws_byte_cursor_from_string(s_chunked_test_ecc_pub_x),
        aws_byte_cursor_from_string(s_chunked_test_ecc_pub_y)));

    /* Manually build the first chunk string-to-sign since it's based on a signature that varies per run */
    struct aws_byte_buf chunk_string_to_sign;
    ASSERT_SUCCESS(aws_byte_buf_init(&chunk_string_to_sign, allocator, 512));
    struct aws_byte_cursor chunk_sts_pre_signature = aws_byte_cursor_from_string(s_chunk_sts_pre_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_pre_signature));
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &signature_cursor));
    struct aws_byte_cursor chunk_sts_post_signature = aws_byte_cursor_from_string(s_chunk1_sts_post_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_post_signature));

    /* Make and sign the first chunk */
    struct aws_signable *first_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk1_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, first_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    struct aws_byte_cursor chunk_signature_cursor =
        aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /* Verify the first chunk's signature */
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        allocator, tester.verification_key, aws_byte_cursor_from_buf(&chunk_string_to_sign), chunk_signature_cursor));

    aws_signable_destroy(first_chunk_signable);

    /* Manually build the second chunk string-to-sign since it's based on a signature that varies per run */
    chunk_string_to_sign.len = 0;
    chunk_sts_pre_signature = aws_byte_cursor_from_string(s_chunk_sts_pre_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_pre_signature));
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_signature_cursor));
    chunk_sts_post_signature = aws_byte_cursor_from_string(s_chunk2_sts_post_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_post_signature));

    /* Make and sign the second chunk */
    struct aws_signable *second_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk2_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, second_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    chunk_signature_cursor = aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /* Verify the second chunk's signature */
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        allocator, tester.verification_key, aws_byte_cursor_from_buf(&chunk_string_to_sign), chunk_signature_cursor));

    aws_signable_destroy(second_chunk_signable);

    /* Manually build the final chunk string-to-sign since it's based on a signature that varies per run */
    chunk_string_to_sign.len = 0;
    chunk_sts_pre_signature = aws_byte_cursor_from_string(s_chunk_sts_pre_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_pre_signature));
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_signature_cursor));
    chunk_sts_post_signature = aws_byte_cursor_from_string(s_chunk3_sts_post_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_post_signature));

    /* Make and sign the final, empty chunk */
    struct aws_signable *final_chunk_signable =
        aws_signable_new_chunk(allocator, NULL, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, final_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    chunk_signature_cursor = aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /* Verify the final chunk's signature */
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        allocator, tester.verification_key, aws_byte_cursor_from_buf(&chunk_string_to_sign), chunk_signature_cursor));

    aws_signable_destroy(final_chunk_signable);

    aws_byte_buf_clean_up(&chunk_string_to_sign);

    s_chunked_signing_tester_cleanup(&tester);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sigv4a_chunked_signing_test, s_sigv4a_chunked_signing_test);

static int s_sigv4a_trailing_headers_signing_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct chunked_signing_tester tester;
    AWS_ZERO_STRUCT(tester);
    ASSERT_SUCCESS(s_chunked_signing_tester_init(allocator, &tester));
    tester.request_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;
    tester.request_signing_config.signed_body_value =
        g_aws_signed_body_value_streaming_aws4_ecdsa_p256_sha256_payload_trailer;
    tester.chunk_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;
    tester.trailing_headers_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;

    /* Sign the base request */
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        tester.trailing_request_signable,
        (void *)&tester.request_signing_config,
        s_on_request_signing_complete,
        &tester));

    struct aws_byte_cursor signature_cursor =
        aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /*
     * Validate the request signature
     */
    ASSERT_SUCCESS(aws_verify_sigv4a_signing(
        allocator,
        tester.trailing_request_signable,
        (void *)&tester.request_signing_config,
        aws_byte_cursor_from_string(s_chunked_expected_trailing_headers_canonical_request_cursor),
        signature_cursor,
        aws_byte_cursor_from_string(s_chunked_test_ecc_pub_x),
        aws_byte_cursor_from_string(s_chunked_test_ecc_pub_y)));

    /* Manually build the first chunk string-to-sign since it's based on a signature that varies per run */
    struct aws_byte_buf chunk_string_to_sign;
    ASSERT_SUCCESS(aws_byte_buf_init(&chunk_string_to_sign, allocator, 512));
    struct aws_byte_cursor chunk_sts_pre_signature = aws_byte_cursor_from_string(s_chunk_sts_pre_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_pre_signature));
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &signature_cursor));
    struct aws_byte_cursor chunk_sts_post_signature = aws_byte_cursor_from_string(s_chunk1_sts_post_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_post_signature));

    /* Make and sign the first chunk */
    struct aws_signable *first_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk1_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, first_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    struct aws_byte_cursor chunk_signature_cursor =
        aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /* Verify the first chunk's signature */
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        allocator, tester.verification_key, aws_byte_cursor_from_buf(&chunk_string_to_sign), chunk_signature_cursor));

    aws_signable_destroy(first_chunk_signable);

    /* Manually build the second chunk string-to-sign since it's based on a signature that varies per run */
    chunk_string_to_sign.len = 0;
    chunk_sts_pre_signature = aws_byte_cursor_from_string(s_chunk_sts_pre_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_pre_signature));
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_signature_cursor));
    chunk_sts_post_signature = aws_byte_cursor_from_string(s_chunk2_sts_post_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_post_signature));

    /* Make and sign the second chunk */
    struct aws_signable *second_chunk_signable =
        aws_signable_new_chunk(allocator, tester.chunk2_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, second_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    chunk_signature_cursor = aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /* Verify the second chunk's signature */
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        allocator, tester.verification_key, aws_byte_cursor_from_buf(&chunk_string_to_sign), chunk_signature_cursor));

    aws_signable_destroy(second_chunk_signable);

    /* Manually build the final chunk string-to-sign since it's based on a signature that varies per run */
    chunk_string_to_sign.len = 0;
    chunk_sts_pre_signature = aws_byte_cursor_from_string(s_chunk_sts_pre_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_pre_signature));
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_signature_cursor));
    chunk_sts_post_signature = aws_byte_cursor_from_string(s_chunk3_sts_post_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_post_signature));

    /* Make and sign the final, empty chunk */
    struct aws_signable *final_chunk_signable =
        aws_signable_new_chunk(allocator, NULL, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, final_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    chunk_signature_cursor = aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    /* Verify the final chunk's signature */
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        allocator, tester.verification_key, aws_byte_cursor_from_buf(&chunk_string_to_sign), chunk_signature_cursor));

    aws_signable_destroy(final_chunk_signable);

    chunk_string_to_sign.len = 0;
    chunk_sts_pre_signature = aws_byte_cursor_from_string(s_trailing_headers_expected_sts_pre_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_pre_signature));
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_signature_cursor));
    chunk_sts_post_signature = aws_byte_cursor_from_string(s_trailing_headers_expected_sts_post_signature);
    ASSERT_SUCCESS(aws_byte_buf_append(&chunk_string_to_sign, &chunk_sts_post_signature));

    struct aws_signable *trailing_headers_signable = aws_signable_new_trailing_headers(
        allocator, tester.trailing_headers, aws_byte_cursor_from_buf(&tester.last_signature));

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        trailing_headers_signable,
        (void *)&tester.trailing_headers_signing_config,
        s_on_chunk_signing_complete,
        &tester));

    struct aws_byte_cursor trailing_headers_signature_cursor =
        aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_buf(&tester.last_signature));

    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        allocator,
        tester.verification_key,
        aws_byte_cursor_from_buf(&chunk_string_to_sign),
        trailing_headers_signature_cursor));

    aws_signable_destroy(trailing_headers_signable);

    aws_byte_buf_clean_up(&chunk_string_to_sign);

    s_chunked_signing_tester_cleanup(&tester);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(sigv4a_trailing_headers_signing_test, s_sigv4a_trailing_headers_signing_test);

static int s_on_incoming_headers_fn(
    struct aws_http_stream *stream,
    enum aws_http_header_block header_block,
    const struct aws_http_header *header_array,
    size_t num_headers,
    void *user_data) {

    struct chunked_signing_tester *tester = user_data;
    (void)tester;
    (void)stream;

    /* Ignore informational headers */
    if (header_block == AWS_HTTP_HEADER_BLOCK_INFORMATIONAL) {
        return AWS_OP_SUCCESS;
    }

    int status = 0;
    aws_http_stream_get_incoming_response_status(stream, &status);
    fprintf(stdout, "Response Status: %d\n", status);

    for (size_t i = 0; i < num_headers; ++i) {
        fwrite(header_array[i].name.ptr, 1, header_array[i].name.len, stdout);
        fprintf(stdout, ": ");
        fwrite(header_array[i].value.ptr, 1, header_array[i].value.len, stdout);
        fprintf(stdout, "\n");
    }

    return AWS_OP_SUCCESS;
}

static int s_on_incoming_header_block_done_fn(
    struct aws_http_stream *stream,
    enum aws_http_header_block header_block,
    void *user_data) {
    (void)stream;
    (void)header_block;
    (void)user_data;

    return AWS_OP_SUCCESS;
}

static void s_on_stream_complete_fn(struct aws_http_stream *stream, int error_code, void *user_data) {
    (void)error_code;
    (void)user_data;
    aws_http_stream_release(stream);
}

static int s_on_incoming_body_fn(struct aws_http_stream *stream, const struct aws_byte_cursor *data, void *user_data) {

    (void)stream;
    (void)user_data;

    fwrite(data->ptr, 1, data->len, stdout);

    return AWS_OP_SUCCESS;
}

void s_log_headers(struct aws_http_message *request) {
    struct aws_http_header header;
    AWS_ZERO_STRUCT(header);
    for (size_t i = 0; i < aws_http_message_get_header_count(request); ++i) {
        aws_http_message_get_header(request, &header, i);
        fwrite(header.name.ptr, 1, header.name.len, stdout);
        fprintf(stdout, ": ");
        fwrite(header.value.ptr, 1, header.value.len, stdout);
        fprintf(stdout, "\n");
    }
}

void s_send_request(struct aws_http_connection *connection, int error_code, void *user_data) {
    (void)error_code;
    struct chunked_signing_tester *tester = user_data;
    struct aws_http_make_request_options opt = {
        .self_size = sizeof(opt),
        .user_data = tester,
        .request = tester->integration_request,
        .on_response_headers = s_on_incoming_headers_fn,
        .on_response_header_block_done = s_on_incoming_header_block_done_fn,
        .on_response_body = s_on_incoming_body_fn,
        .on_complete = s_on_stream_complete_fn,
    };

    struct aws_http_stream *stream = aws_http_connection_make_request(connection, &opt);
    aws_http_stream_activate(stream);
}

static void s_on_client_connection_shutdown(struct aws_http_connection *connection, int error_code, void *user_data) {
    (void)error_code;
    (void)connection;
    struct chunked_signing_tester *tester = user_data;

    aws_mutex_lock(&tester->mutex);
    tester->request_completed = true;
    aws_mutex_unlock(&tester->mutex);
    aws_condition_variable_notify_all(&tester->c_var);
}

static bool s_completion_predicate(void *arg) {
    struct chunked_signing_tester *tester = arg;
    return tester->request_completed;
}

static int s_sigv4a_trailing_header_integration_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_auth_library_init(allocator);

    struct chunked_signing_tester tester;
    AWS_ZERO_STRUCT(tester);
    ASSERT_SUCCESS(s_chunked_signing_tester_init(allocator, &tester));
    tester.request_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;
    tester.request_signing_config.signed_body_value =
        g_aws_signed_body_value_streaming_aws4_ecdsa_p256_sha256_payload_trailer;
    tester.chunk_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;
    tester.trailing_headers_signing_config.algorithm = AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC;

    AWS_ZERO_STRUCT(tester.request_signing_config.date);
    AWS_ZERO_STRUCT(tester.chunk_signing_config.date);
    AWS_ZERO_STRUCT(tester.trailing_headers_signing_config.date);
    aws_date_time_init_now(&tester.request_signing_config.date);
    tester.chunk_signing_config.date = tester.request_signing_config.date;
    tester.trailing_headers_signing_config.date = tester.request_signing_config.date;
    // aws_date_time_init_now(&tester.chunk_signing_config.date);
    // aws_date_time_init_now(&tester.trailing_headers_signing_config.date);

    struct aws_credentials *credentials = aws_credentials_new_from_string(
        allocator, s_integration_chunked_access_key_id, s_integration_chunked_secret_access_key, NULL, UINT64_MAX);
    tester.request_signing_config.credentials = credentials;
    tester.chunk_signing_config.credentials = credentials;
    tester.trailing_headers_signing_config.credentials = credentials;

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        tester.integration_request_signable,
        (void *)&tester.request_signing_config,
        s_on_integration_request_signing_complete,
        &tester));

    struct aws_signable *first_chunk_signable = aws_signable_new_chunk(
        allocator, tester.integration_chunk_stream, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, first_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    struct aws_byte_buf first_chunk_signature;
    AWS_ZERO_STRUCT(first_chunk_signature);
    aws_byte_buf_init_copy(&first_chunk_signature, aws_default_allocator(), &tester.last_signature);

    /* Make and sign the final, empty chunk */
    struct aws_signable *final_chunk_signable =
        aws_signable_new_chunk(allocator, NULL, aws_byte_cursor_from_buf(&tester.last_signature));
    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, final_chunk_signable, (void *)&tester.chunk_signing_config, s_on_chunk_signing_complete, &tester));

    struct aws_byte_buf final_chunk_signature;
    AWS_ZERO_STRUCT(final_chunk_signature);
    aws_byte_buf_init_copy(&final_chunk_signature, aws_default_allocator(), &tester.last_signature);

    struct aws_signable *trailing_headers_signable = aws_signable_new_trailing_headers(
        allocator, tester.integration_trailing_headers, aws_byte_cursor_from_buf(&tester.last_signature));

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        trailing_headers_signable,
        (void *)&tester.trailing_headers_signing_config,
        s_on_chunk_signing_complete,
        &tester));

    struct aws_byte_buf trailing_header_signature;
    AWS_ZERO_STRUCT(trailing_header_signature);
    aws_byte_buf_init_copy(&trailing_header_signature, aws_default_allocator(), &tester.last_signature);

    struct aws_byte_cursor pre_chunk = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("1;chunk-signature=");
    struct aws_byte_cursor first_chunk = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\r\na\r\n0;chunk-signature=");
    struct aws_byte_cursor trailer_chunk =
        AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\r\nx-amz-checksum-crc32c:wdBDMA==\r\nx-amz-trailer-signature:");
    struct aws_byte_cursor carriage_return = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\r\n\r\n");
    struct aws_byte_cursor first_chunk_signature_cursor = aws_byte_cursor_from_buf(&first_chunk_signature);
    struct aws_byte_cursor final_chunk_signature_cursor = aws_byte_cursor_from_buf(&final_chunk_signature);
    struct aws_byte_cursor trailing_header_signature_cursor = aws_byte_cursor_from_buf(&trailing_header_signature);

    AWS_ASSERT(aws_byte_cursor_is_valid(&pre_chunk));
    AWS_ASSERT(aws_byte_cursor_is_valid(&first_chunk));
    AWS_ASSERT(aws_byte_cursor_is_valid(&trailer_chunk));
    AWS_ASSERT(aws_byte_cursor_is_valid(&carriage_return));
    AWS_ASSERT(aws_byte_cursor_is_valid(&first_chunk_signature_cursor));
    AWS_ASSERT(aws_byte_cursor_is_valid(&final_chunk_signature_cursor));
    AWS_ASSERT(aws_byte_cursor_is_valid(&trailing_header_signature_cursor));

    size_t body_buf_len = pre_chunk.len + first_chunk.len + trailer_chunk.len + carriage_return.len +
                          first_chunk_signature_cursor.len + final_chunk_signature_cursor.len +
                          trailing_header_signature_cursor.len;
    struct aws_byte_buf body_buffer;
    aws_byte_buf_init(&body_buffer, aws_default_allocator(), body_buf_len);
    aws_byte_buf_append(&body_buffer, &pre_chunk);
    aws_byte_buf_append(&body_buffer, &first_chunk_signature_cursor);
    aws_byte_buf_append(&body_buffer, &first_chunk);
    aws_byte_buf_append(&body_buffer, &final_chunk_signature_cursor);
    aws_byte_buf_append(&body_buffer, &trailer_chunk);
    aws_byte_buf_append(&body_buffer, &trailing_header_signature_cursor);
    aws_byte_buf_append(&body_buffer, &carriage_return);
    struct aws_byte_cursor body_cursor = aws_byte_cursor_from_buf(&body_buffer);
    struct aws_input_stream *body_stream = aws_input_stream_new_from_cursor(aws_default_allocator(), &body_cursor);
    aws_http_message_set_body_stream(tester.integration_request, body_stream);

    struct aws_http_client_connection_options client_options = AWS_HTTP_CLIENT_CONNECTION_OPTIONS_INIT;
    struct aws_byte_cursor host = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("example_bucket");

    struct aws_socket_options socket_options = {
        .type = AWS_SOCKET_STREAM,
        .connect_timeout_ms = 3000,
        .keep_alive_timeout_sec = 0,
        .keepalive = false,
        .keep_alive_interval_sec = 0,
    };

    struct aws_event_loop_group *el_group = aws_event_loop_group_new_default(allocator, 1, NULL);

    struct aws_host_resolver_default_options resolver_options = {
        .el_group = el_group,
        .max_entries = 8,
    };

    struct aws_host_resolver *resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = el_group,
        .host_resolver = resolver,
    };
    struct aws_client_bootstrap *bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    client_options.on_setup = s_send_request;
    client_options.on_shutdown = s_on_client_connection_shutdown;
    client_options.user_data = &tester;
    client_options.allocator = aws_default_allocator();
    client_options.host_name = host;
    client_options.bootstrap = bootstrap;
    client_options.socket_options = &socket_options;
    client_options.port = 80;

    AWS_LOGF_INFO(AWS_LS_AUTH_SIGNING, "Headers");
    s_log_headers(tester.integration_request);
    AWS_LOGF_INFO(AWS_LS_AUTH_SIGNING, "Request Body\n" PRInSTR "\n", AWS_BYTE_BUF_PRI(body_buffer));

    aws_http_client_connect(&client_options);
    aws_mutex_lock(&tester.mutex);
    aws_condition_variable_wait_pred(&tester.c_var, &tester.mutex, s_completion_predicate, &tester);
    aws_mutex_unlock(&tester.mutex);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(sigv4a_trailing_header_integration_test, s_sigv4a_trailing_header_integration_test);
