/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/aws_signing.h>
#include <aws/auth/signable.h>
#include <aws/auth/signing.h>
#include <aws/cal/ecc.h>
#include <aws/cal/hash.h>
#include <aws/common/condition_variable.h>
#include <aws/common/encoding.h>
#include <aws/common/environment.h>
#include <aws/common/json.h>
#include <aws/common/string.h>
#include <aws/http/request_response.h>
#include <aws/io/file_utils.h>
#include <aws/io/stream.h>
#include <aws/io/uri.h>

#include <aws/auth/signing_config.h>
#include <ctype.h>

#include "credentials_provider_utils.h"
#include "test_signable.h"

#if defined(_MSC_VER)
#    pragma warning(disable : 4996) /* deprecation */
#endif

AWS_STATIC_STRING_FROM_LITERAL(s_header_canonical_request_filename, "header-canonical-request.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_header_string_to_sign_filename, "header-string-to-sign.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_header_signed_request_filename, "header-signed-request.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_header_signature_filename, "header-signature.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_query_canonical_request_filename, "query-canonical-request.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_query_string_to_sign_filename, "query-string-to-sign.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_query_signed_request_filename, "query-signed-request.txt");
AWS_STATIC_STRING_FROM_LITERAL(s_query_signature_filename, "query-signature.txt");

AWS_STATIC_STRING_FROM_LITERAL(s_public_key_filename, "public-key.json");

static const struct aws_string *s_get_canonical_request_filename(enum aws_signature_type signature_type) {
    switch (signature_type) {
        case AWS_ST_HTTP_REQUEST_HEADERS:
            return s_header_canonical_request_filename;
        case AWS_ST_HTTP_REQUEST_QUERY_PARAMS:
            return s_query_canonical_request_filename;

        default:
            return NULL;
    }
}

static const struct aws_string *s_get_string_to_sign_filename(enum aws_signature_type signature_type) {
    switch (signature_type) {
        case AWS_ST_HTTP_REQUEST_HEADERS:
            return s_header_string_to_sign_filename;
        case AWS_ST_HTTP_REQUEST_QUERY_PARAMS:
            return s_query_string_to_sign_filename;

        default:
            return NULL;
    }
}

static const struct aws_string *s_get_signed_request_filename(enum aws_signature_type signature_type) {
    switch (signature_type) {
        case AWS_ST_HTTP_REQUEST_HEADERS:
            return s_header_signed_request_filename;
        case AWS_ST_HTTP_REQUEST_QUERY_PARAMS:
            return s_query_signed_request_filename;

        default:
            return NULL;
    }
}

static const struct aws_string *s_get_signature_filename(enum aws_signature_type signature_type) {
    switch (signature_type) {
        case AWS_ST_HTTP_REQUEST_HEADERS:
            return s_header_signature_filename;
        case AWS_ST_HTTP_REQUEST_QUERY_PARAMS:
            return s_query_signature_filename;

        default:
            return NULL;
    }
}

struct v4_test_case_contents {
    struct aws_allocator *allocator;
    struct aws_byte_buf context;
    struct aws_byte_buf request;
    struct aws_byte_buf public_key;
    struct aws_byte_buf expected_canonical_request;
    struct aws_byte_buf expected_string_to_sign;
    struct aws_byte_buf sample_signed_request;
    struct aws_byte_buf sample_signature;
};

static void s_strip_windows_line_endings(struct aws_byte_buf *buffer) {
    size_t write_index = 0;
    for (size_t read_index = 0; read_index < buffer->len; ++read_index) {
        char current = buffer->buffer[read_index];
        if (current != '\r') {
            buffer->buffer[write_index++] = current;
        }
    }

    buffer->len = write_index;
}

static int s_load_test_case_file(
    struct aws_allocator *allocator,
    const char *parent_folder,
    const char *test_name,
    const char *filename,
    struct aws_byte_buf *buffer) {
    char path[1024];
    snprintf(path, AWS_ARRAY_SIZE(path), "./%s/%s/%s", parent_folder, test_name, filename);
    return aws_byte_buf_init_from_file(buffer, allocator, path);
}

static int s_v4_test_case_context_init_from_file_set(
    struct v4_test_case_contents *contents,
    struct aws_allocator *allocator,
    const char *parent_folder,
    const char *test_name,
    enum aws_signature_type signature_type) {

    AWS_ZERO_STRUCT(*contents);
    contents->allocator = allocator;

    /* required files */
    if (s_load_test_case_file(allocator, parent_folder, test_name, "request.txt", &contents->request) ||
        s_load_test_case_file(allocator, parent_folder, test_name, "context.json", &contents->context)) {
        return AWS_OP_ERR;
    }

    s_strip_windows_line_endings(&contents->request);

    s_load_test_case_file(
        allocator, parent_folder, test_name, (const char *)s_public_key_filename->bytes, &contents->public_key);

    s_load_test_case_file(
        allocator,
        parent_folder,
        test_name,
        aws_string_c_str(s_get_canonical_request_filename(signature_type)),
        &contents->expected_canonical_request);

    s_strip_windows_line_endings(&contents->expected_canonical_request);

    s_load_test_case_file(
        allocator,
        parent_folder,
        test_name,
        aws_string_c_str(s_get_string_to_sign_filename(signature_type)),
        &contents->expected_string_to_sign);

    s_strip_windows_line_endings(&contents->expected_string_to_sign);

    s_load_test_case_file(
        allocator,
        parent_folder,
        test_name,
        aws_string_c_str(s_get_signed_request_filename(signature_type)),
        &contents->sample_signed_request);

    s_strip_windows_line_endings(&contents->sample_signed_request);

    s_load_test_case_file(
        allocator,
        parent_folder,
        test_name,
        aws_string_c_str(s_get_signature_filename(signature_type)),
        &contents->sample_signature);

    s_strip_windows_line_endings(&contents->sample_signature);

    return AWS_OP_SUCCESS;
}

static void s_v4_test_case_contents_clean_up(struct v4_test_case_contents *contents) {
    if (contents->allocator) {
        aws_byte_buf_clean_up(&contents->request);
        aws_byte_buf_clean_up(&contents->context);
        aws_byte_buf_clean_up(&contents->public_key);
        aws_byte_buf_clean_up(&contents->expected_canonical_request);
        aws_byte_buf_clean_up(&contents->expected_string_to_sign);
        aws_byte_buf_clean_up(&contents->sample_signed_request);
        aws_byte_buf_clean_up(&contents->sample_signature);

        contents->allocator = NULL;
    }
}

struct v4_test_context {
    struct aws_allocator *allocator;
    enum aws_signing_algorithm algorithm;
    struct v4_test_case_contents test_case_data;

    struct aws_string *region_config;
    struct aws_string *service;
    struct aws_string *timestamp;
    struct aws_credentials *credentials;
    bool should_normalize;
    bool should_sign_body;
    uint64_t expiration_in_seconds;
    struct aws_input_stream *payload_stream;
    struct aws_ecc_key_pair *signing_key;
    struct aws_ecc_key_pair *verification_key;

    struct aws_signable *signable;
    struct aws_signing_config_aws *config;

    struct aws_mutex lock;
    struct aws_condition_variable signal;

    bool done;

    struct aws_signing_state_aws *signing_state;
    struct aws_http_message *request;

    bool should_generate_test_case;

    struct aws_string *canonical_signing_auth_value;
    bool omit_session_token;
};

static void s_v4_test_context_clean_up(struct v4_test_context *context) {
    s_v4_test_case_contents_clean_up(&context->test_case_data);

    aws_http_message_release(context->request);
    aws_input_stream_destroy(context->payload_stream);
    aws_ecc_key_pair_release(context->signing_key);
    aws_ecc_key_pair_release(context->verification_key);

    aws_string_destroy(context->region_config);
    aws_string_destroy(context->service);
    aws_string_destroy(context->timestamp);
    aws_credentials_release(context->credentials);

    aws_mutex_clean_up(&context->lock);
    aws_condition_variable_clean_up(&context->signal);

    aws_signing_state_destroy(context->signing_state);

    aws_mem_release(context->allocator, context->config);
    aws_signable_destroy(context->signable);

    aws_string_destroy(context->canonical_signing_auth_value);
}

AWS_STATIC_STRING_FROM_LITERAL(s_empty_empty_string, "\0");
AWS_STATIC_STRING_FROM_LITERAL(s_credentials_name, "credentials");
AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_name, "access_key_id");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_name, "secret_access_key");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_name, "token");
AWS_STATIC_STRING_FROM_LITERAL(s_region_name, "region");
AWS_STATIC_STRING_FROM_LITERAL(s_service_name, "service");
AWS_STATIC_STRING_FROM_LITERAL(s_timestamp_name, "timestamp");
AWS_STATIC_STRING_FROM_LITERAL(s_normalize_name, "normalize");
AWS_STATIC_STRING_FROM_LITERAL(s_body_name, "sign_body");
AWS_STATIC_STRING_FROM_LITERAL(s_expiration_name, "expiration_in_seconds");
AWS_STATIC_STRING_FROM_LITERAL(s_omit_token_name, "omit_session_token");

static int s_v4_test_context_parse_context_file(struct v4_test_context *context) {
    struct aws_byte_buf *document = &context->test_case_data.context;
    struct aws_json_value *document_root = NULL;
    int result = AWS_OP_ERR;

    struct aws_byte_cursor null_terminator_cursor = aws_byte_cursor_from_string(s_empty_empty_string);
    if (aws_byte_buf_append_dynamic(document, &null_terminator_cursor)) {
        goto done;
    }

    struct aws_byte_cursor document_buffer_cursor = aws_byte_cursor_from_buf(document);
    document_root = aws_json_value_new_from_string(aws_default_allocator(), document_buffer_cursor);
    if (document_root == NULL) {
        goto done;
    }

    struct aws_json_value *credentials_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_credentials_name));
    AWS_FATAL_ASSERT(credentials_node != NULL);

    /*
     * Pull out the three credentials components
     */
    struct aws_json_value *access_key_id =
        aws_json_value_get_from_object(credentials_node, aws_byte_cursor_from_string(s_access_key_id_name));
    struct aws_json_value *secret_access_key =
        aws_json_value_get_from_object(credentials_node, aws_byte_cursor_from_string(s_secret_access_key_name));
    struct aws_json_value *session_token =
        aws_json_value_get_from_object(credentials_node, aws_byte_cursor_from_string(s_session_token_name));

    struct aws_byte_cursor access_key_id_cursor;
    if (!aws_json_value_is_string(access_key_id) ||
        aws_json_value_get_string(access_key_id, &access_key_id_cursor) == AWS_OP_ERR) {
        goto done;
    }

    struct aws_byte_cursor secret_access_key_cursor;
    AWS_ZERO_STRUCT(secret_access_key_cursor);
    struct aws_byte_cursor session_token_cursor;
    AWS_ZERO_STRUCT(session_token_cursor);

    if (aws_json_value_is_string(session_token)) {
        aws_json_value_get_string(session_token, &session_token_cursor);
    }
    if (aws_json_value_is_string(secret_access_key)) {
        aws_json_value_get_string(secret_access_key, &secret_access_key_cursor);
    }

    if (context->signing_key == NULL) {
        context->credentials = aws_credentials_new(
            context->allocator, access_key_id_cursor, secret_access_key_cursor, session_token_cursor, UINT64_MAX);
        context->signing_key =
            aws_ecc_key_pair_new_ecdsa_p256_key_from_aws_credentials(context->allocator, context->credentials);
    } else {
        context->credentials = aws_credentials_new_ecc(
            context->allocator, access_key_id_cursor, context->signing_key, session_token_cursor, UINT64_MAX);
        context->signing_key = aws_credentials_get_ecc_key_pair(context->credentials);
        aws_ecc_key_pair_acquire(context->signing_key);
    }

    AWS_FATAL_ASSERT(context->credentials != NULL);

    struct aws_json_value *region_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_region_name));
    struct aws_byte_cursor region_node_cursor;
    if (region_node == NULL || !aws_json_value_is_string(region_node) ||
        aws_json_value_get_string(region_node, &region_node_cursor) == AWS_OP_ERR) {
        goto done;
    }

    context->region_config = aws_string_new_from_cursor(context->allocator, &region_node_cursor);
    if (context->region_config == NULL) {
        goto done;
    }

    struct aws_json_value *service_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_service_name));
    struct aws_byte_cursor service_node_cursor;
    if (service_node == NULL || !aws_json_value_is_string(service_node) ||
        aws_json_value_get_string(service_node, &service_node_cursor) == AWS_OP_ERR) {
        goto done;
    }

    context->service = aws_string_new_from_cursor(context->allocator, &service_node_cursor);
    if (context->service == NULL) {
        goto done;
    }

    struct aws_json_value *timestamp_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_timestamp_name));
    struct aws_byte_cursor timestamp_node_cursor;
    if (timestamp_node == NULL || !aws_json_value_is_string(timestamp_node) ||
        aws_json_value_get_string(timestamp_node, &timestamp_node_cursor) == AWS_OP_ERR) {
        goto done;
    }

    context->timestamp = aws_string_new_from_cursor(context->allocator, &timestamp_node_cursor);
    if (context->timestamp == NULL) {
        goto done;
    }

    struct aws_json_value *normalize_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_normalize_name));
    if (normalize_node == NULL || !aws_json_value_is_boolean(normalize_node)) {
        goto done;
    }

    aws_json_value_get_boolean(normalize_node, &context->should_normalize);

    struct aws_json_value *body_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_body_name));
    if (body_node == NULL || !aws_json_value_is_boolean(body_node)) {
        goto done;
    }

    aws_json_value_get_boolean(body_node, &context->should_sign_body);

    struct aws_json_value *expiration_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_expiration_name));
    if (expiration_node == NULL || !aws_json_value_is_number(expiration_node)) {
        goto done;
    }

    double expiration_in_seconds_double = 0;
    aws_json_value_get_number(expiration_node, &expiration_in_seconds_double);
    context->expiration_in_seconds = (uint64_t)expiration_in_seconds_double;

    struct aws_json_value *omit_token_node =
        aws_json_value_get_from_object(document_root, aws_byte_cursor_from_string(s_omit_token_name));
    if (omit_token_node != NULL && aws_json_value_is_boolean(omit_token_node)) {
        aws_json_value_get_boolean(omit_token_node, &context->omit_session_token);
    }

    result = AWS_OP_SUCCESS;

done:

    if (document_root != NULL) {
        aws_json_value_destroy(document_root);
    }

    return result;
}

static int s_parse_request(
    struct aws_allocator *allocator,
    struct aws_byte_cursor request_cursor,
    struct aws_http_message **out_request,
    struct aws_input_stream **out_body_stream) {
    int result = AWS_OP_ERR;

    *out_request = NULL;
    *out_body_stream = NULL;

    struct aws_array_list request_lines;
    AWS_ZERO_STRUCT(request_lines);
    if (aws_array_list_init_dynamic(&request_lines, allocator, 10, sizeof(struct aws_byte_cursor))) {
        return AWS_OP_ERR;
    }

    struct aws_input_stream *body_stream = NULL;
    struct aws_http_message *request = aws_http_message_new_request(allocator);
    if (request == NULL) {
        goto done;
    }

    struct aws_array_list header_set;
    AWS_ZERO_STRUCT(header_set);
    if (aws_array_list_init_dynamic(&header_set, allocator, 10, sizeof(struct aws_signable_property_list_pair))) {
        goto done;
    }

    if (aws_byte_cursor_split_on_char(&request_cursor, '\n', &request_lines)) {
        goto done;
    }

    size_t line_count = aws_array_list_length(&request_lines);
    if (line_count == 0) {
        goto done;
    }

    struct aws_byte_cursor first_line;
    AWS_ZERO_STRUCT(first_line);
    if (aws_array_list_get_at(&request_lines, &first_line, 0)) {
        goto done;
    }

    struct aws_byte_cursor method_cursor;
    AWS_ZERO_STRUCT(method_cursor);
    if (!aws_byte_cursor_next_split(&first_line, ' ', &method_cursor)) {
        goto done;
    }

    aws_http_message_set_request_method(request, method_cursor);

    aws_byte_cursor_advance(&first_line, method_cursor.len + 1);

    /* not safe in general, but all test cases end in " HTTP/1.1" */
    struct aws_byte_cursor uri_cursor = first_line;
    uri_cursor.len -= 9;

    aws_http_message_set_request_path(request, uri_cursor);

    /* headers */
    size_t line_index = 1;
    for (; line_index < line_count; ++line_index) {
        struct aws_byte_cursor current_line;
        AWS_ZERO_STRUCT(current_line);
        if (aws_array_list_get_at(&request_lines, &current_line, line_index)) {
            goto done;
        }

        if (current_line.len == 0) {
            /* empty line = end of headers */
            break;
        }

        if (isspace(*current_line.ptr)) {
            /* multi-line header, append the entire line to the most recent header's value */
            size_t current_header_count = aws_array_list_length(&header_set);
            AWS_FATAL_ASSERT(current_header_count > 0);

            struct aws_signable_property_list_pair *current_header;
            if (aws_array_list_get_at_ptr(&header_set, (void **)&current_header, current_header_count - 1)) {
                goto done;
            }

            current_header->value.len = (current_line.ptr + current_line.len) - current_header->value.ptr;
        } else {
            /* new header, parse it and add to the header set */
            struct aws_signable_property_list_pair current_header;
            AWS_ZERO_STRUCT(current_header);
            if (!aws_byte_cursor_next_split(&current_line, ':', &current_header.name)) {
                goto done;
            }

            aws_byte_cursor_advance(&current_line, current_header.name.len + 1);
            current_header.value = current_line;

            aws_array_list_push_back(&header_set, &current_header);
        }
    }

    size_t header_count = aws_array_list_length(&header_set);
    for (size_t i = 0; i < header_count; ++i) {
        struct aws_signable_property_list_pair property_header;
        aws_array_list_get_at(&header_set, &property_header, i);

        struct aws_http_header header = {
            .name = property_header.name,
            .value = property_header.value,
        };

        aws_http_message_add_header(request, header);
    }

    /* body */
    struct aws_byte_cursor body_cursor;
    AWS_ZERO_STRUCT(body_cursor);
    if (line_index + 1 < line_count) {
        if (aws_array_list_get_at(&request_lines, &body_cursor, line_index + 1)) {
            goto done;
        }

        /* body length is the end of the whole request (pointer) minus the start of the body pointer */
        body_cursor.len = (request_cursor.ptr + request_cursor.len - body_cursor.ptr);

        body_stream = aws_input_stream_new_from_cursor(allocator, &body_cursor);
        if (body_stream == NULL) {
            goto done;
        }

        aws_http_message_set_body_stream(request, body_stream);
    }

    result = AWS_OP_SUCCESS;

done:

    aws_array_list_clean_up(&request_lines);
    aws_array_list_clean_up(&header_set);

    if (result == AWS_OP_ERR) {
        aws_http_message_release(request);
        aws_input_stream_destroy(body_stream);
    } else {
        *out_request = request;
        *out_body_stream = body_stream;
    }

    return result;
}

static int s_v4_test_context_init_signing_config(
    struct v4_test_context *context,
    enum aws_signature_type signature_type) {

    context->signable = aws_signable_new_http_request(context->allocator, context->request);

    context->config = aws_mem_calloc(context->allocator, 1, sizeof(struct aws_signing_config_aws));
    if (context->config == NULL) {
        return AWS_OP_ERR;
    }

    context->config->config_type = AWS_SIGNING_CONFIG_AWS;
    context->config->algorithm = context->algorithm;
    context->config->signature_type = signature_type;
    context->config->region = aws_byte_cursor_from_string(context->region_config);
    context->config->service = aws_byte_cursor_from_string(context->service);
    context->config->flags.use_double_uri_encode = true;
    context->config->flags.should_normalize_uri_path = context->should_normalize;
    context->config->flags.omit_session_token = context->omit_session_token;

    /* ToDo: make the tests more fine-grained now that we have updated payload signing controls */
    if (context->should_sign_body) {
        context->config->signed_body_header = AWS_SBHT_X_AMZ_CONTENT_SHA256;
    } else {
        context->config->signed_body_value = g_aws_signed_body_value_empty_sha256;
    }

    context->config->credentials = context->credentials;
    context->config->expiration_in_seconds = context->expiration_in_seconds;

    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(context->timestamp);
    if (aws_date_time_init_from_str_cursor(&context->config->date, &date_cursor, AWS_DATE_FORMAT_ISO_8601)) {
        return AWS_OP_ERR;
    }

    context->signing_state = aws_signing_state_new(context->allocator, context->config, context->signable, NULL, NULL);
    if (context->signing_state == NULL) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

static int s_v4_test_context_parse_verification_key(struct v4_test_context *context) {
    AWS_FATAL_ASSERT(context->signing_key != NULL);

    if (context->test_case_data.public_key.len == 0) {
        context->verification_key = context->signing_key;
        aws_ecc_key_pair_acquire(context->signing_key);
        aws_ecc_key_pair_derive_public_key(context->verification_key);
        return AWS_OP_SUCCESS;
    }

    struct aws_byte_buf pub_x_buffer;
    AWS_ZERO_STRUCT(pub_x_buffer);
    struct aws_byte_buf pub_y_buffer;
    AWS_ZERO_STRUCT(pub_y_buffer);

    struct aws_byte_buf *document = &context->test_case_data.public_key;
    struct aws_json_value *document_root = NULL;
    int result = AWS_OP_ERR;

    struct aws_byte_cursor null_terminator_cursor = aws_byte_cursor_from_string(s_empty_empty_string);
    if (aws_byte_buf_append_dynamic(document, &null_terminator_cursor)) {
        goto done;
    }

    struct aws_byte_cursor document_cursor = aws_byte_cursor_from_buf(document);
    document_root = aws_json_value_new_from_string(aws_default_allocator(), document_cursor);
    if (document_root == NULL) {
        goto done;
    }

    /*
     * Pull out the three credentials components
     */
    struct aws_json_value *pub_x = aws_json_value_get_from_object(document_root, aws_byte_cursor_from_c_str("X"));
    struct aws_json_value *pub_y = aws_json_value_get_from_object(document_root, aws_byte_cursor_from_c_str("Y"));
    if (!aws_json_value_is_string(pub_x) || !aws_json_value_is_string(pub_y)) {
        goto done;
    }

    struct aws_byte_cursor pub_x_hex_cursor;
    struct aws_byte_cursor pub_y_hex_cursor;
    aws_json_value_get_string(pub_x, &pub_x_hex_cursor);
    aws_json_value_get_string(pub_y, &pub_y_hex_cursor);

    size_t pub_x_length = 0;
    size_t pub_y_length = 0;
    if (aws_hex_compute_decoded_len(pub_x_hex_cursor.len, &pub_x_length) ||
        aws_hex_compute_decoded_len(pub_y_hex_cursor.len, &pub_y_length)) {
        goto done;
    }

    if (aws_byte_buf_init(&pub_x_buffer, context->allocator, pub_x_length) ||
        aws_byte_buf_init(&pub_y_buffer, context->allocator, pub_y_length)) {
        goto done;
    }

    if (aws_hex_decode(&pub_x_hex_cursor, &pub_x_buffer) || aws_hex_decode(&pub_y_hex_cursor, &pub_y_buffer)) {
        goto done;
    }

    struct aws_byte_cursor pub_x_cursor = aws_byte_cursor_from_buf(&pub_x_buffer);
    struct aws_byte_cursor pub_y_cursor = aws_byte_cursor_from_buf(&pub_y_buffer);

    context->verification_key =
        aws_ecc_key_pair_new_from_public_key(context->allocator, AWS_CAL_ECDSA_P256, &pub_x_cursor, &pub_y_cursor);

    AWS_FATAL_ASSERT(context->verification_key != NULL);

    result = AWS_OP_SUCCESS;

done:

    if (document_root) {
        aws_json_value_destroy(document_root);
    }

    aws_byte_buf_clean_up(&pub_x_buffer);
    aws_byte_buf_clean_up(&pub_y_buffer);

    return result;
}

AWS_STATIC_STRING_FROM_LITERAL(s_generate_test_env_var_name, "GENERATE_TEST_CASES");

static int s_v4_test_context_init(
    struct v4_test_context *context,
    struct aws_allocator *allocator,
    const char *parent_folder,
    const char *test_name,
    enum aws_signing_algorithm algorithm,
    enum aws_signature_type signature_type) {

    AWS_ZERO_STRUCT(*context);
    context->allocator = allocator;
    context->algorithm = algorithm;

    struct aws_string *should_generate = NULL;
    ASSERT_SUCCESS(aws_get_environment_value(allocator, s_generate_test_env_var_name, &should_generate));

    context->should_generate_test_case = should_generate != NULL;
    aws_string_destroy(should_generate);

    if (s_v4_test_case_context_init_from_file_set(
            &context->test_case_data, allocator, parent_folder, test_name, signature_type)) {
        return AWS_OP_ERR;
    }

    if (s_v4_test_context_parse_context_file(context)) {
        return AWS_OP_ERR;
    }

    if (s_parse_request(
            context->allocator,
            aws_byte_cursor_from_buf(&context->test_case_data.request),
            &context->request,
            &context->payload_stream)) {
        return AWS_OP_ERR;
    }

    if (s_v4_test_context_init_signing_config(context, signature_type)) {
        return AWS_OP_ERR;
    }

    if (s_v4_test_context_parse_verification_key(context)) {
        return AWS_OP_ERR;
    }

    if (aws_mutex_init(&context->lock)) {
        return AWS_OP_ERR;
    }

    if (aws_condition_variable_init(&context->signal)) {
        return AWS_OP_ERR;
    }

    context->done = false;

    return AWS_OP_SUCCESS;
}

bool s_is_signing_complete_predicate(void *userdata) {
    struct v4_test_context *context = userdata;
    return context->done;
}

void s_wait_on_signing_complete(struct v4_test_context *context) {
    aws_mutex_lock(&context->lock);
    if (!context->done) {
        aws_condition_variable_wait_pred(&context->signal, &context->lock, s_is_signing_complete_predicate, context);
    }
    aws_mutex_unlock(&context->lock);
}

static void s_on_signing_complete(struct aws_signing_result *result, int error_code, void *userdata) {

    AWS_FATAL_ASSERT(error_code == AWS_ERROR_SUCCESS);

    struct v4_test_context *context = userdata;

    aws_apply_signing_result_to_http_request(context->request, context->allocator, result);

    struct aws_string *auth_value = NULL;
    aws_signing_result_get_property(result, g_aws_signature_property_name, &auth_value);
    struct aws_byte_cursor auth_value_cursor =
        aws_trim_padded_sigv4a_signature(aws_byte_cursor_from_string(auth_value));
    context->canonical_signing_auth_value = aws_string_new_from_cursor(context->allocator, &auth_value_cursor);

    /* Mark results complete */
    aws_mutex_lock(&context->lock);
    context->done = true;
    aws_condition_variable_notify_one(&context->signal);
    aws_mutex_unlock(&context->lock);
}

static int s_write_test_file(
    const char *parent_folder,
    const char *test_name,
    const struct aws_string *filename,
    const struct aws_byte_buf *contents) {
    char path[1024];
    snprintf(path, AWS_ARRAY_SIZE(path), "./%s/%s/%s", parent_folder, test_name, aws_string_c_str(filename));

    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        return AWS_OP_ERR;
    }

    struct aws_byte_cursor cursor = aws_byte_cursor_from_array(contents->buffer, contents->len);
    fprintf(fp, PRInSTR, AWS_BYTE_CURSOR_PRI(cursor));

    fclose(fp);

    return AWS_OP_SUCCESS;
}

AWS_STATIC_STRING_FROM_LITERAL(s_public_key_prefix_json, "{\n");
AWS_STATIC_STRING_FROM_LITERAL(s_public_key_x_json, "  \"X\":\"");
AWS_STATIC_STRING_FROM_LITERAL(s_public_key_y_json, "\",\n  \"Y\":\"");
AWS_STATIC_STRING_FROM_LITERAL(s_public_key_suffix_json, "\"\n}\n");

int s_write_public_key_file(struct v4_test_context *context, const char *parent_folder, const char *test_name) {
    AWS_FATAL_ASSERT(context->verification_key != NULL);
    int result = AWS_OP_ERR;
    struct aws_byte_buf key_buffer;
    AWS_ZERO_STRUCT(key_buffer);

    if (aws_byte_buf_init(&key_buffer, context->allocator, 256)) {
        goto done;
    }

    struct aws_byte_cursor pub_x_cursor;
    AWS_ZERO_STRUCT(pub_x_cursor);
    struct aws_byte_cursor pub_y_cursor;
    AWS_ZERO_STRUCT(pub_y_cursor);

    aws_ecc_key_pair_derive_public_key(context->verification_key);
    aws_ecc_key_pair_get_public_key(context->verification_key, &pub_x_cursor, &pub_y_cursor);

    struct aws_byte_cursor prefix_cursor = aws_byte_cursor_from_string(s_public_key_prefix_json);
    aws_byte_buf_append_dynamic(&key_buffer, &prefix_cursor);

    struct aws_byte_cursor x_cursor = aws_byte_cursor_from_string(s_public_key_x_json);
    aws_byte_buf_append_dynamic(&key_buffer, &x_cursor);

    aws_hex_encode_append_dynamic(&pub_x_cursor, &key_buffer);

    struct aws_byte_cursor y_cursor = aws_byte_cursor_from_string(s_public_key_y_json);
    aws_byte_buf_append_dynamic(&key_buffer, &y_cursor);

    aws_hex_encode_append_dynamic(&pub_y_cursor, &key_buffer);

    struct aws_byte_cursor suffix_cursor = aws_byte_cursor_from_string(s_public_key_suffix_json);
    aws_byte_buf_append_dynamic(&key_buffer, &suffix_cursor);

    result = s_write_test_file(parent_folder, test_name, s_public_key_filename, &key_buffer);

done:

    aws_byte_buf_clean_up(&key_buffer);

    return result;
}

static int s_generate_test_case(
    struct v4_test_context *test_context,
    const char *parent_folder,
    const char *test_name) {
    {
        struct aws_signing_state_aws *signing_state = test_context->signing_state;

        /* Generate public key file */
        if (test_context->algorithm == AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC &&
            test_context->config->signature_type == AWS_ST_HTTP_REQUEST_HEADERS) {
            ASSERT_SUCCESS(s_write_public_key_file(test_context, parent_folder, test_name));
        }

        /* 1a - generate canonical request */
        ASSERT_TRUE(aws_signing_build_canonical_request(signing_state) == AWS_OP_SUCCESS);
        ASSERT_SUCCESS(s_write_test_file(
            parent_folder,
            test_name,
            s_get_canonical_request_filename(test_context->config->signature_type),
            &signing_state->canonical_request));

        /* 1b- generate string to sign */
        ASSERT_TRUE(aws_signing_build_string_to_sign(signing_state) == AWS_OP_SUCCESS);
        ASSERT_SUCCESS(s_write_test_file(
            parent_folder,
            test_name,
            s_get_string_to_sign_filename(test_context->config->signature_type),
            &signing_state->string_to_sign));
    }

    return AWS_OP_SUCCESS;
}

static int s_check_piecewise_test_case(struct v4_test_context *test_context) {
    struct aws_signing_state_aws *signing_state = test_context->signing_state;

    /* 1a -  validate canonical request */
    ASSERT_TRUE(aws_signing_build_canonical_request(signing_state) == AWS_OP_SUCCESS);
    ASSERT_BIN_ARRAYS_EQUALS(
        test_context->test_case_data.expected_canonical_request.buffer,
        test_context->test_case_data.expected_canonical_request.len,
        signing_state->canonical_request.buffer,
        signing_state->canonical_request.len);

    /* 1b- validate string to sign */
    ASSERT_TRUE(aws_signing_build_string_to_sign(signing_state) == AWS_OP_SUCCESS);
    ASSERT_BIN_ARRAYS_EQUALS(
        test_context->test_case_data.expected_string_to_sign.buffer,
        test_context->test_case_data.expected_string_to_sign.len,
        signing_state->string_to_sign.buffer,
        signing_state->string_to_sign.len);

    /* authorization values checked in the end-to-end tests */

    return AWS_OP_SUCCESS;
}

static int s_do_sigv4_test_piecewise(
    struct aws_allocator *allocator,
    const char *parent_folder,
    const char *test_name,
    enum aws_signing_algorithm algorithm,
    enum aws_signature_type signature_type) {

    struct v4_test_context test_context;
    AWS_ZERO_STRUCT(test_context);

    ASSERT_SUCCESS(
        s_v4_test_context_init(&test_context, allocator, parent_folder, test_name, algorithm, signature_type));

    if (test_context.should_generate_test_case) {
        ASSERT_SUCCESS(s_generate_test_case(&test_context, parent_folder, test_name));
    } else {
        ASSERT_SUCCESS(s_check_piecewise_test_case(&test_context));
    }

    s_v4_test_context_clean_up(&test_context);

    return AWS_OP_SUCCESS;
}

static int s_write_signed_request_to_file(
    struct v4_test_context *test_context,
    const char *parent_folder,
    const char *test_name,
    const struct aws_string *filename) {
    char path[1024];
    snprintf(path, AWS_ARRAY_SIZE(path), "./%s/%s/%s", parent_folder, test_name, aws_string_c_str(filename));

    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        return AWS_OP_ERR;
    }

    struct aws_byte_cursor method_cursor;
    ASSERT_SUCCESS(aws_http_message_get_request_method(test_context->request, &method_cursor));

    struct aws_byte_cursor path_cursor;
    ASSERT_SUCCESS(aws_http_message_get_request_path(test_context->request, &path_cursor));

    fprintf(
        fp, PRInSTR " " PRInSTR " HTTP/1.1\n", AWS_BYTE_CURSOR_PRI(method_cursor), AWS_BYTE_CURSOR_PRI(path_cursor));
    size_t header_count = aws_http_message_get_header_count(test_context->request);
    for (size_t i = 0; i < header_count; ++i) {
        struct aws_http_header header;
        AWS_ZERO_STRUCT(header);
        ASSERT_SUCCESS(aws_http_message_get_header(test_context->request, &header, i));
        fprintf(fp, PRInSTR ":" PRInSTR "\n", AWS_BYTE_CURSOR_PRI(header.name), AWS_BYTE_CURSOR_PRI(header.value));
    }

    fprintf(fp, "\n");

    if (test_context->payload_stream) {
        int64_t stream_length = 0;
        ASSERT_SUCCESS(aws_input_stream_get_length(test_context->payload_stream, &stream_length));

        struct aws_byte_buf stream_buf;
        ASSERT_SUCCESS(aws_byte_buf_init(&stream_buf, test_context->allocator, (size_t)stream_length));

        ASSERT_SUCCESS(aws_input_stream_seek(test_context->payload_stream, 0, AWS_SSB_BEGIN));
        ASSERT_SUCCESS(aws_input_stream_read(test_context->payload_stream, &stream_buf));
        ASSERT_TRUE(stream_buf.len == (size_t)stream_length);

        fprintf(fp, PRInSTR, AWS_BYTE_BUF_PRI(stream_buf));

        aws_byte_buf_clean_up(&stream_buf);
    }

    fclose(fp);

    return AWS_OP_SUCCESS;
}

static int s_check_header_value(struct aws_http_message *request, struct aws_http_header *expected_header) {
    size_t header_count = aws_http_message_get_header_count(request);
    for (size_t i = 0; i < header_count; ++i) {
        struct aws_http_header header;
        AWS_ZERO_STRUCT(header);

        ASSERT_SUCCESS(aws_http_message_get_header(request, &header, i));

        if (aws_byte_cursor_eq_ignore_case(&header.name, &expected_header->name)) {
            if (aws_byte_cursor_eq(&header.value, &expected_header->value)) {
                aws_http_message_erase_header(request, i);
                return AWS_OP_SUCCESS;
            }
        }
    }

    ASSERT_TRUE(false);
}

static int s_check_query_authorization(
    struct v4_test_context *test_context,
    struct aws_byte_cursor signed_path,
    struct aws_byte_cursor expected_path) {

    struct aws_uri signed_uri;
    ASSERT_SUCCESS(aws_uri_init_parse(&signed_uri, test_context->allocator, &signed_path));

    struct aws_uri expected_uri;
    ASSERT_SUCCESS(aws_uri_init_parse(&expected_uri, test_context->allocator, &expected_path));

    ASSERT_BIN_ARRAYS_EQUALS(signed_uri.path.ptr, signed_uri.path.len, expected_uri.path.ptr, expected_uri.path.len);

    struct aws_array_list signed_params;
    ASSERT_SUCCESS(
        aws_array_list_init_dynamic(&signed_params, test_context->allocator, 10, sizeof(struct aws_uri_param)));
    ASSERT_SUCCESS(aws_uri_query_string_params(&signed_uri, &signed_params));

    struct aws_array_list expected_params;
    ASSERT_SUCCESS(
        aws_array_list_init_dynamic(&expected_params, test_context->allocator, 10, sizeof(struct aws_uri_param)));
    ASSERT_SUCCESS(aws_uri_query_string_params(&expected_uri, &expected_params));

    ASSERT_TRUE(aws_array_list_length(&signed_params) == aws_array_list_length(&expected_params));

    struct aws_byte_cursor signature_cursor = aws_byte_cursor_from_string(g_aws_signing_authorization_query_param_name);

    size_t signed_param_count = aws_array_list_length(&signed_params);
    for (size_t i = 0; i < signed_param_count; ++i) {
        struct aws_uri_param signed_param;
        aws_array_list_get_at(&signed_params, &signed_param, i);

        if (aws_byte_cursor_eq_ignore_case(&signed_param.key, &signature_cursor) &&
            test_context->algorithm == AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC) {
            ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
                test_context->allocator,
                test_context->verification_key,
                aws_byte_cursor_from_buf(&test_context->test_case_data.expected_string_to_sign),
                signed_param.value));
        } else {
            bool found = false;
            for (size_t j = 0; j < signed_param_count; ++j) {
                struct aws_uri_param expected_param;
                aws_array_list_get_at(&expected_params, &expected_param, j);
                if (aws_byte_cursor_eq_ignore_case(&signed_param.key, &expected_param.key)) {
                    ASSERT_TRUE(aws_byte_cursor_eq_ignore_case(&signed_param.value, &expected_param.value));
                    found = true;
                    break;
                }
            }

            ASSERT_TRUE(found);
        }
    }

    aws_uri_clean_up(&signed_uri);
    aws_uri_clean_up(&expected_uri);
    aws_array_list_clean_up(&signed_params);
    aws_array_list_clean_up(&expected_params);

    return AWS_OP_SUCCESS;
}

static int s_get_authorization_pair(
    const struct aws_byte_cursor *authorization_value,
    const struct aws_byte_cursor name,
    struct aws_byte_cursor value_end,
    struct aws_byte_cursor *value_out) {
    struct aws_byte_cursor value_start_cursor;
    AWS_ZERO_STRUCT(value_start_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(authorization_value, &name, &value_start_cursor));
    aws_byte_cursor_advance(&value_start_cursor, name.len);

    struct aws_byte_cursor value_end_cursor;
    AWS_ZERO_STRUCT(value_end_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&value_start_cursor, &value_end, &value_end_cursor));

    *value_out = value_start_cursor;
    value_out->len = value_end_cursor.ptr - value_start_cursor.ptr;

    return AWS_OP_SUCCESS;
}

static int s_compare_authorization_pair(
    const struct aws_byte_cursor *signed_value,
    const struct aws_byte_cursor *expected_value,
    const struct aws_byte_cursor name) {
    struct aws_byte_cursor signed_pair_value;
    AWS_ZERO_STRUCT(signed_pair_value);
    ASSERT_SUCCESS(s_get_authorization_pair(signed_value, name, aws_byte_cursor_from_c_str(", "), &signed_pair_value));

    struct aws_byte_cursor expected_pair_value;
    AWS_ZERO_STRUCT(expected_pair_value);
    ASSERT_SUCCESS(
        s_get_authorization_pair(expected_value, name, aws_byte_cursor_from_c_str(", "), &expected_pair_value));

    ASSERT_BIN_ARRAYS_EQUALS(
        signed_pair_value.ptr, signed_pair_value.len, expected_pair_value.ptr, expected_pair_value.len);

    return AWS_OP_SUCCESS;
}

static int s_check_header_authorization(
    struct v4_test_context *test_context,
    struct aws_http_header *header,
    struct aws_http_message *expected_request) {
    struct aws_byte_cursor signed_authorization_value = header->value;

    struct aws_byte_cursor expected_authorization_value;
    AWS_ZERO_STRUCT(expected_authorization_value);

    size_t expected_header_count = aws_http_message_get_header_count(expected_request);
    for (size_t i = 0; i < expected_header_count; ++i) {
        struct aws_http_header expected_header;
        AWS_ZERO_STRUCT(expected_header);

        if (aws_http_message_get_header(expected_request, &expected_header, i)) {
            continue;
        }

        if (aws_byte_cursor_eq_c_str_ignore_case(&expected_header.name, "Authorization")) {
            expected_authorization_value = expected_header.value;
            break;
        }
    }

    ASSERT_TRUE(expected_authorization_value.len > 0);

    struct aws_byte_cursor space_cursor = aws_byte_cursor_from_c_str(" ");

    struct aws_byte_cursor signed_space_cursor;
    AWS_ZERO_STRUCT(signed_space_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&signed_authorization_value, &space_cursor, &signed_space_cursor));
    struct aws_byte_cursor signed_algorithm_cursor = {
        .ptr = signed_authorization_value.ptr,
        .len = signed_space_cursor.ptr - signed_authorization_value.ptr,
    };

    struct aws_byte_cursor expected_space_cursor;
    AWS_ZERO_STRUCT(expected_space_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&expected_authorization_value, &space_cursor, &expected_space_cursor));
    struct aws_byte_cursor expected_algorithm_cursor = {
        .ptr = expected_authorization_value.ptr,
        .len = expected_space_cursor.ptr - expected_authorization_value.ptr,
    };

    ASSERT_BIN_ARRAYS_EQUALS(
        signed_algorithm_cursor.ptr,
        signed_algorithm_cursor.len,
        expected_algorithm_cursor.ptr,
        expected_algorithm_cursor.len);

    ASSERT_SUCCESS(s_compare_authorization_pair(
        &signed_authorization_value, &expected_authorization_value, aws_byte_cursor_from_c_str("Credential=")));
    ASSERT_SUCCESS(s_compare_authorization_pair(
        &signed_authorization_value, &expected_authorization_value, aws_byte_cursor_from_c_str("SignedHeaders=")));

    struct aws_byte_buf *string_to_sign = &test_context->test_case_data.expected_string_to_sign;
    struct aws_byte_cursor signature_key_cursor = aws_byte_cursor_from_c_str("Signature=");

    struct aws_byte_cursor signed_signature_value;
    AWS_ZERO_STRUCT(signed_signature_value);
    ASSERT_SUCCESS(
        aws_byte_cursor_find_exact(&signed_authorization_value, &signature_key_cursor, &signed_signature_value));
    aws_byte_cursor_advance(&signed_signature_value, signature_key_cursor.len);
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        test_context->allocator,
        test_context->verification_key,
        aws_byte_cursor_from_buf(string_to_sign),
        signed_signature_value));

    struct aws_byte_cursor expected_signature_value;
    AWS_ZERO_STRUCT(expected_signature_value);
    ASSERT_SUCCESS(
        aws_byte_cursor_find_exact(&expected_authorization_value, &signature_key_cursor, &expected_signature_value));
    aws_byte_cursor_advance(&expected_signature_value, signature_key_cursor.len);
    ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
        test_context->allocator,
        test_context->verification_key,
        aws_byte_cursor_from_buf(string_to_sign),
        expected_signature_value));

    return AWS_OP_SUCCESS;
}

static int s_check_signed_request(struct v4_test_context *test_context, struct aws_byte_buf *expected_request_buffer) {

    struct aws_http_message *expected_request = NULL;
    struct aws_input_stream *body_stream = NULL;

    ASSERT_SUCCESS(s_parse_request(
        test_context->allocator, aws_byte_cursor_from_buf(expected_request_buffer), &expected_request, &body_stream));
    ASSERT_NOT_NULL(expected_request);

    /* method */
    struct aws_byte_cursor signed_method;
    AWS_ZERO_STRUCT(signed_method);
    aws_http_message_get_request_method(test_context->request, &signed_method);

    struct aws_byte_cursor expected_method;
    AWS_ZERO_STRUCT(expected_method);
    aws_http_message_get_request_method(expected_request, &expected_method);

    ASSERT_BIN_ARRAYS_EQUALS(expected_method.ptr, expected_method.len, signed_method.ptr, signed_method.len);

    /* path + query string */
    struct aws_byte_cursor signed_path;
    AWS_ZERO_STRUCT(signed_path);
    aws_http_message_get_request_path(test_context->request, &signed_path);

    struct aws_byte_cursor expected_path;
    AWS_ZERO_STRUCT(expected_path);
    aws_http_message_get_request_path(expected_request, &expected_path);

    if (test_context->config->signature_type == AWS_ST_HTTP_REQUEST_QUERY_PARAMS) {
        ASSERT_SUCCESS(s_check_query_authorization(test_context, signed_path, expected_path));
    } else {
        ASSERT_BIN_ARRAYS_EQUALS(expected_path.ptr, expected_path.len, signed_path.ptr, signed_path.len);
    }

    /* headers */
    size_t signed_header_count = aws_http_message_get_header_count(test_context->request);
    size_t expected_header_count = aws_http_message_get_header_count(expected_request);
    ASSERT_TRUE(signed_header_count == expected_header_count);

    for (size_t i = 0; i < signed_header_count; ++i) {
        struct aws_http_header header;
        AWS_ZERO_STRUCT(header);

        if (aws_http_message_get_header(test_context->request, &header, i)) {
            continue;
        }

        if (test_context->config->signature_type == AWS_ST_HTTP_REQUEST_HEADERS &&
            aws_byte_cursor_eq_c_str_ignore_case(&header.name, "Authorization") &&
            test_context->algorithm == AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC) {
            ASSERT_SUCCESS(s_check_header_authorization(test_context, &header, expected_request));
        } else {
            ASSERT_SUCCESS(s_check_header_value(expected_request, &header));
        }
    }

    aws_http_message_release(expected_request);
    aws_input_stream_destroy(body_stream);

    return AWS_OP_SUCCESS;
}

static int s_do_sigv4_test_end_to_end(
    struct aws_allocator *allocator,
    const char *parent_folder,
    const char *test_name,
    enum aws_signing_algorithm algorithm,
    enum aws_signature_type signature_type) {

    struct v4_test_context test_context;
    AWS_ZERO_STRUCT(test_context);

    ASSERT_SUCCESS(
        s_v4_test_context_init(&test_context, allocator, parent_folder, test_name, algorithm, signature_type));

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, test_context.signable, (void *)test_context.config, s_on_signing_complete, &test_context));

    s_wait_on_signing_complete(&test_context);

    if (test_context.should_generate_test_case) {
        ASSERT_SUCCESS(s_write_signed_request_to_file(
            &test_context, parent_folder, test_name, s_get_signed_request_filename(signature_type)));
    } else {
        ASSERT_SUCCESS(s_check_signed_request(&test_context, &test_context.test_case_data.sample_signed_request));
    }

    s_v4_test_context_clean_up(&test_context);

    return AWS_OP_SUCCESS;
}

static int s_write_signature_to_file(
    struct v4_test_context *test_context,
    const char *parent_folder,
    const char *test_name,
    const struct aws_string *filename) {
    char path[1024];
    snprintf(path, AWS_ARRAY_SIZE(path), "./%s/%s/%s", parent_folder, test_name, aws_string_c_str(filename));

    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        return AWS_OP_ERR;
    }

    struct aws_byte_cursor signature_cursor = aws_byte_cursor_from_string(test_context->canonical_signing_auth_value);
    fprintf(fp, PRInSTR, AWS_BYTE_CURSOR_PRI(signature_cursor));

    fclose(fp);

    return AWS_OP_SUCCESS;
}

static int s_do_sigv4_test_canonical_only(
    struct aws_allocator *allocator,
    const char *parent_folder,
    const char *test_name,
    enum aws_signing_algorithm algorithm,
    enum aws_signature_type signature_type) {

    struct v4_test_context test_context;
    AWS_ZERO_STRUCT(test_context);

    ASSERT_SUCCESS(
        s_v4_test_context_init(&test_context, allocator, parent_folder, test_name, algorithm, signature_type));

    /* replace the http request signable with a canonical request signable */
    aws_signable_destroy(test_context.signable);
    test_context.signable = aws_signable_new_canonical_request(
        allocator, aws_byte_cursor_from_buf(&test_context.test_case_data.expected_canonical_request));
    if (signature_type == AWS_ST_HTTP_REQUEST_QUERY_PARAMS) {
        test_context.config->signature_type = AWS_ST_CANONICAL_REQUEST_QUERY_PARAMS;
    } else {
        test_context.config->signature_type = AWS_ST_CANONICAL_REQUEST_HEADERS;
    }

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator, test_context.signable, (void *)test_context.config, s_on_signing_complete, &test_context));

    s_wait_on_signing_complete(&test_context);

    if (test_context.should_generate_test_case) {
        ASSERT_SUCCESS(s_write_signature_to_file(
            &test_context, parent_folder, test_name, s_get_signature_filename(signature_type)));
    } else {
        if (algorithm == AWS_SIGNING_ALGORITHM_V4) {
            ASSERT_BIN_ARRAYS_EQUALS(
                test_context.test_case_data.sample_signature.buffer,
                test_context.test_case_data.sample_signature.len,
                test_context.canonical_signing_auth_value->bytes,
                test_context.canonical_signing_auth_value->len);
        } else {
            ASSERT_SUCCESS(aws_validate_v4a_authorization_value(
                test_context.allocator,
                test_context.verification_key,
                aws_byte_cursor_from_buf(&test_context.test_case_data.expected_string_to_sign),
                aws_byte_cursor_from_string(test_context.canonical_signing_auth_value)));
        }
    }

    s_v4_test_context_clean_up(&test_context);

    return AWS_OP_SUCCESS;
}

static int s_do_sigv4_test_signing(
    struct aws_allocator *allocator,
    const char *parent_folder,
    const char *test_name,
    enum aws_signing_algorithm algorithm,
    enum aws_signature_type signature_type) {

    ASSERT_SUCCESS(s_do_sigv4_test_piecewise(allocator, parent_folder, test_name, algorithm, signature_type));

    ASSERT_SUCCESS(s_do_sigv4_test_end_to_end(allocator, parent_folder, test_name, algorithm, signature_type));

    ASSERT_SUCCESS(s_do_sigv4_test_canonical_only(allocator, parent_folder, test_name, algorithm, signature_type));

    return AWS_OP_SUCCESS;
}

static int s_do_sigv4a_test_case(struct aws_allocator *allocator, const char *test_name, const char *parent_folder) {

    /* Set up everything */
    aws_auth_library_init(allocator);

    ASSERT_SUCCESS(s_do_sigv4_test_signing(
        allocator, parent_folder, test_name, AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC, AWS_ST_HTTP_REQUEST_HEADERS));
    ASSERT_SUCCESS(s_do_sigv4_test_signing(
        allocator, parent_folder, test_name, AWS_SIGNING_ALGORITHM_V4_ASYMMETRIC, AWS_ST_HTTP_REQUEST_QUERY_PARAMS));

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

#define DECLARE_SIGV4A_TEST_SUITE_CASE(test_name, test_name_string)                                                    \
    static int s_sigv4a_##test_name##_test(struct aws_allocator *allocator, void *ctx) {                               \
        (void)ctx;                                                                                                     \
        return s_do_sigv4a_test_case(allocator, test_name_string, "./v4a");                                            \
    }                                                                                                                  \
    AWS_TEST_CASE(sigv4a_##test_name##_test, s_sigv4a_##test_name##_test);

DECLARE_SIGV4A_TEST_SUITE_CASE(get_header_key_duplicate, "get-header-key-duplicate");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_header_value_multiline, "get-header-value-multiline");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_header_value_order, "get-header-value-order");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_header_value_trim, "get-header-value-trim");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_unreserved, "get-unreserved");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_utf8, "get-utf8");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla, "get-vanilla");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla_empty_query_key, "get-vanilla-empty-query-key");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla_query, "get-vanilla-query");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla_query_order_key_case, "get-vanilla-query-order-key-case");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla_query_order_encoded, "get-vanilla-query-order-encoded");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla_unreserved, "get-vanilla-query-unreserved");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla_utf8_query, "get-vanilla-utf8-query");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_header_key_case, "post-header-key-case");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_header_key_sort, "post-header-key-sort");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_header_value_case, "post-header-value-case");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_vanilla, "post-vanilla");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_vanilla_empty_query_value, "post-vanilla-empty-query-value");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_vanilla_query, "post-vanilla-query");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_x_www_form_urlencoded, "post-x-www-form-urlencoded");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_x_www_form_urlencoded_parameters, "post-x-www-form-urlencoded-parameters");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_vanilla_with_session_token, "get-vanilla-with-session-token");

DECLARE_SIGV4A_TEST_SUITE_CASE(post_sts_header_after, "post-sts-header-after");
DECLARE_SIGV4A_TEST_SUITE_CASE(post_sts_header_before, "post-sts-header-before");

DECLARE_SIGV4A_TEST_SUITE_CASE(get_relative_normalized, "get-relative-normalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_relative_unnormalized, "get-relative-unnormalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_relative_relative_normalized, "get-relative-relative-normalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_relative_relative_unnormalized, "get-relative-relative-unnormalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slash_normalized, "get-slash-normalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slash_unnormalized, "get-slash-unnormalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slash_dot_slash_normalized, "get-slash-dot-slash-normalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slash_dot_slash_unnormalized, "get-slash-dot-slash-unnormalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slash_pointless_dot_normalized, "get-slash-pointless-dot-normalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slash_pointless_dot_unnormalized, "get-slash-pointless-dot-unnormalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slashes_normalized, "get-slashes-normalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_slashes_unnormalized, "get-slashes-unnormalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_space_normalized, "get-space-normalized");
DECLARE_SIGV4A_TEST_SUITE_CASE(get_space_unnormalized, "get-space-unnormalized");

static int s_do_sigv4_test_case(struct aws_allocator *allocator, const char *test_name, const char *parent_folder) {

    /* Set up everything */
    aws_auth_library_init(allocator);

    ASSERT_SUCCESS(s_do_sigv4_test_signing(
        allocator, parent_folder, test_name, AWS_SIGNING_ALGORITHM_V4, AWS_ST_HTTP_REQUEST_HEADERS));
    ASSERT_SUCCESS(s_do_sigv4_test_signing(
        allocator, parent_folder, test_name, AWS_SIGNING_ALGORITHM_V4, AWS_ST_HTTP_REQUEST_QUERY_PARAMS));

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

#define DECLARE_SIGV4_TEST_SUITE_CASE(test_name, test_name_string)                                                     \
    static int s_sigv4_##test_name##_test(struct aws_allocator *allocator, void *ctx) {                                \
        (void)ctx;                                                                                                     \
        return s_do_sigv4_test_case(allocator, test_name_string, "./v4");                                              \
    }                                                                                                                  \
    AWS_TEST_CASE(sigv4_##test_name##_test, s_sigv4_##test_name##_test);

DECLARE_SIGV4_TEST_SUITE_CASE(get_header_key_duplicate, "get-header-key-duplicate");
DECLARE_SIGV4_TEST_SUITE_CASE(get_header_value_multiline, "get-header-value-multiline");
DECLARE_SIGV4_TEST_SUITE_CASE(get_header_value_order, "get-header-value-order");
DECLARE_SIGV4_TEST_SUITE_CASE(get_header_value_trim, "get-header-value-trim");
DECLARE_SIGV4_TEST_SUITE_CASE(get_unreserved, "get-unreserved");
DECLARE_SIGV4_TEST_SUITE_CASE(get_utf8, "get-utf8");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla, "get-vanilla");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla_empty_query_key, "get-vanilla-empty-query-key");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla_query, "get-vanilla-query");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla_query_order_key_case, "get-vanilla-query-order-key-case");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla_query_order_encoded, "get-vanilla-query-order-encoded");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla_unreserved, "get-vanilla-query-unreserved");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla_utf8_query, "get-vanilla-utf8-query");
DECLARE_SIGV4_TEST_SUITE_CASE(post_header_key_case, "post-header-key-case");
DECLARE_SIGV4_TEST_SUITE_CASE(post_header_key_sort, "post-header-key-sort");
DECLARE_SIGV4_TEST_SUITE_CASE(post_header_value_case, "post-header-value-case");
DECLARE_SIGV4_TEST_SUITE_CASE(post_vanilla, "post-vanilla");
DECLARE_SIGV4_TEST_SUITE_CASE(post_vanilla_empty_query_value, "post-vanilla-empty-query-value");
DECLARE_SIGV4_TEST_SUITE_CASE(post_vanilla_query, "post-vanilla-query");
DECLARE_SIGV4_TEST_SUITE_CASE(post_x_www_form_urlencoded, "post-x-www-form-urlencoded");
DECLARE_SIGV4_TEST_SUITE_CASE(post_x_www_form_urlencoded_parameters, "post-x-www-form-urlencoded-parameters");
DECLARE_SIGV4_TEST_SUITE_CASE(get_vanilla_with_session_token, "get-vanilla-with-session-token");

DECLARE_SIGV4_TEST_SUITE_CASE(post_sts_header_after, "post-sts-header-after");
DECLARE_SIGV4_TEST_SUITE_CASE(post_sts_header_before, "post-sts-header-before");

DECLARE_SIGV4_TEST_SUITE_CASE(get_relative_normalized, "get-relative-normalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_relative_unnormalized, "get-relative-unnormalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_relative_relative_normalized, "get-relative-relative-normalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_relative_relative_unnormalized, "get-relative-relative-unnormalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slash_normalized, "get-slash-normalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slash_unnormalized, "get-slash-unnormalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slash_dot_slash_normalized, "get-slash-dot-slash-normalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slash_dot_slash_unnormalized, "get-slash-dot-slash-unnormalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slash_pointless_dot_normalized, "get-slash-pointless-dot-normalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slash_pointless_dot_unnormalized, "get-slash-pointless-dot-unnormalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slashes_normalized, "get-slashes-normalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_slashes_unnormalized, "get-slashes-unnormalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_space_normalized, "get-space-normalized");
DECLARE_SIGV4_TEST_SUITE_CASE(get_space_unnormalized, "get-space-unnormalized");

AWS_STATIC_STRING_FROM_LITERAL(s_test_suite_service, "service");
AWS_STATIC_STRING_FROM_LITERAL(s_test_suite_region, "us-east-1");
AWS_STATIC_STRING_FROM_LITERAL(s_test_suite_access_key_id, "AKIDEXAMPLE");
AWS_STATIC_STRING_FROM_LITERAL(s_test_suite_secret_access_key, "wJalrXUtnFEMI/K7MDENG+bPxRfiCYEXAMPLEKEY");
AWS_STATIC_STRING_FROM_LITERAL(s_test_suite_date, "2015-08-30T12:36:00Z");

static int s_do_header_skip_test(
    struct aws_allocator *allocator,
    aws_should_sign_header_fn *should_sign,
    const struct aws_string *request_contents,
    const struct aws_string *expected_canonical_request) {

    aws_auth_library_init(allocator);

    struct aws_credentials *credentials = aws_credentials_new_from_string(
        allocator, s_test_suite_access_key_id, s_test_suite_secret_access_key, NULL, UINT64_MAX);
    ASSERT_NOT_NULL(credentials);

    struct aws_signing_config_aws config;
    AWS_ZERO_STRUCT(config);
    config.credentials = credentials;
    config.algorithm = AWS_SIGNING_ALGORITHM_V4;
    config.signature_type = AWS_ST_HTTP_REQUEST_HEADERS;
    config.region = aws_byte_cursor_from_string(s_test_suite_region);
    config.service = aws_byte_cursor_from_string(s_test_suite_service);
    config.should_sign_header = should_sign;

    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_test_suite_date);
    ASSERT_SUCCESS(aws_date_time_init_from_str_cursor(&config.date, &date_cursor, AWS_DATE_FORMAT_ISO_8601));

    struct aws_http_message *message = NULL;
    struct aws_input_stream *body_stream = NULL;
    ASSERT_SUCCESS(s_parse_request(allocator, aws_byte_cursor_from_string(request_contents), &message, &body_stream));
    struct aws_signable *signable = aws_signable_new_http_request(allocator, message);

    /* release reference to message early, to ensure signable keeps it alive */
    aws_http_message_release(message);

    struct aws_signing_state_aws *signing_state = aws_signing_state_new(allocator, &config, signable, NULL, NULL);
    ASSERT_NOT_NULL(signing_state);

    ASSERT_SUCCESS(aws_signing_build_canonical_request(signing_state));

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_canonical_request->bytes,
        expected_canonical_request->len,
        signing_state->canonical_request.buffer,
        signing_state->canonical_request.len);

    aws_input_stream_destroy(body_stream);
    aws_signing_state_destroy(signing_state);
    aws_credentials_release(credentials);
    aws_signable_destroy(signable);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_xray_header_request,
    "GET / HTTP/1.1\n"
    "Host:example.amazonaws.com\n"
    "x-amzn-trace-id:fsdbofdshfdsjkjhfs\n\n");

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_xray_header_expected_canonical_request,
    "GET\n"
    "/\n"
    "\n"
    "host:example.amazonaws.com\n"
    "x-amz-date:20150830T123600Z\n"
    "\n"
    "host;x-amz-date\n"
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

static int s_sigv4_skip_xray_header_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_header_skip_test(
        allocator, NULL, s_skip_xray_header_request, s_skip_xray_header_expected_canonical_request);
}
AWS_TEST_CASE(sigv4_skip_xray_header_test, s_sigv4_skip_xray_header_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_user_agent_header_request,
    "GET / HTTP/1.1\n"
    "User-agent:c sdk v1.0\n"
    "Host:example.amazonaws.com\n\n");

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_user_agent_header_expected_canonical_request,
    "GET\n"
    "/\n"
    "\n"
    "host:example.amazonaws.com\n"
    "x-amz-date:20150830T123600Z\n"
    "\n"
    "host;x-amz-date\n"
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

static int s_sigv4_skip_user_agent_header_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_header_skip_test(
        allocator, NULL, s_skip_user_agent_header_request, s_skip_user_agent_header_expected_canonical_request);
}
AWS_TEST_CASE(sigv4_skip_user_agent_header_test, s_sigv4_skip_user_agent_header_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_transfer_encoding_header_request,
    "GET / HTTP/1.1\n"
    "Transfer-Encoding: chunked\n"
    "Host:example.amazonaws.com\n\n");

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_transfer_encoding_header_expected_canonical_request,
    "GET\n"
    "/\n"
    "\n"
    "host:example.amazonaws.com\n"
    "x-amz-date:20150830T123600Z\n"
    "\n"
    "host;x-amz-date\n"
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

static int s_sigv4_skip_transfer_encoding_header_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_header_skip_test(
        allocator,
        NULL,
        s_skip_transfer_encoding_header_request,
        s_skip_transfer_encoding_header_expected_canonical_request);
}
AWS_TEST_CASE(sigv4_skip_transfer_encoding_header_test, s_sigv4_skip_transfer_encoding_header_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_custom_header_request,
    "GET / HTTP/1.1\n"
    "MyHeader:Blahblah\n"
    "Host:example.amazonaws.com\n"
    "AnotherHeader:Oof\n\n");

AWS_STATIC_STRING_FROM_LITERAL(
    s_skip_custom_header_expected_canonical_request,
    "GET\n"
    "/\n"
    "\n"
    "host:example.amazonaws.com\n"
    "x-amz-date:20150830T123600Z\n"
    "\n"
    "host;x-amz-date\n"
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

static bool s_should_sign_header(const struct aws_byte_cursor *name, void *userdata) {
    (void)userdata;

    struct aws_byte_cursor my_header_cursor = aws_byte_cursor_from_c_str("myheader");
    struct aws_byte_cursor another_header_cursor = aws_byte_cursor_from_c_str("anOtherHeader");

    if (aws_byte_cursor_eq_ignore_case(name, &my_header_cursor) ||
        aws_byte_cursor_eq_ignore_case(name, &another_header_cursor)) {
        return false;
    }

    return true;
}

static int s_sigv4_skip_custom_header_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_header_skip_test(
        allocator, s_should_sign_header, s_skip_custom_header_request, s_skip_custom_header_expected_canonical_request);
}
AWS_TEST_CASE(sigv4_skip_custom_header_test, s_sigv4_skip_custom_header_test);

static int s_do_forbidden_header_param_test(
    struct aws_allocator *allocator,
    const struct aws_string *request_contents,
    enum aws_auth_errors expected_error) {

    aws_auth_library_init(allocator);

    struct aws_credentials *credentials = aws_credentials_new_from_string(
        allocator, s_test_suite_access_key_id, s_test_suite_secret_access_key, NULL, UINT64_MAX);
    ASSERT_NOT_NULL(credentials);

    struct aws_signing_config_aws config;
    AWS_ZERO_STRUCT(config);
    config.credentials = credentials;
    config.algorithm = AWS_SIGNING_ALGORITHM_V4;
    config.signature_type = AWS_ST_HTTP_REQUEST_HEADERS;
    config.region = aws_byte_cursor_from_string(s_test_suite_region);
    config.service = aws_byte_cursor_from_string(s_test_suite_service);

    struct aws_byte_cursor date_cursor = aws_byte_cursor_from_string(s_test_suite_date);
    ASSERT_SUCCESS(aws_date_time_init_from_str_cursor(&config.date, &date_cursor, AWS_DATE_FORMAT_ISO_8601));

    struct aws_http_message *message = NULL;
    struct aws_input_stream *body_stream = NULL;
    ASSERT_SUCCESS(s_parse_request(allocator, aws_byte_cursor_from_string(request_contents), &message, &body_stream));
    struct aws_signable *signable = aws_signable_new_http_request(allocator, message);

    struct aws_signing_state_aws *signing_state = aws_signing_state_new(allocator, &config, signable, NULL, NULL);
    ASSERT_NOT_NULL(signing_state);

    ASSERT_FAILS(aws_signing_build_canonical_request(signing_state));
    ASSERT_TRUE(aws_last_error() == expected_error);

    aws_input_stream_destroy(body_stream);
    aws_http_message_release(message);

    aws_signing_state_destroy(signing_state);
    aws_credentials_release(credentials);
    aws_signable_destroy(signable);

    aws_auth_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_STATIC_STRING_FROM_LITERAL(
    s_amz_date_header_request,
    "GET / HTTP/1.1\n"
    "Host:example.amazonaws.com\n"
    "X-Amz-Date:20150830T123600Z");

static int s_sigv4_fail_date_header_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_amz_date_header_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_HEADER);
}
AWS_TEST_CASE(sigv4_fail_date_header_test, s_sigv4_fail_date_header_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_amz_content_sha256_header_request,
    "GET / HTTP/1.1\n"
    "Host:example.amazonaws.com\n"
    "x-amz-content-sha256:lieslieslies");

static int s_sigv4_fail_content_header_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_amz_content_sha256_header_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_HEADER);
}
AWS_TEST_CASE(sigv4_fail_content_header_test, s_sigv4_fail_content_header_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_authorization_header_request,
    "GET / HTTP/1.1\n"
    "Host:example.amazonaws.com\n"
    "Authorization:lieslieslies");

static int s_sigv4_fail_authorization_header_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_authorization_header_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_HEADER);
}
AWS_TEST_CASE(sigv4_fail_authorization_header_test, s_sigv4_fail_authorization_header_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_amz_signature_param_request,
    "GET /?X-Amz-Signature=Something HTTP/1.1\n"
    "Host:example.amazonaws.com\n");

static int s_sigv4_fail_signature_param_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_amz_signature_param_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_QUERY_PARAM);
}
AWS_TEST_CASE(sigv4_fail_signature_param_test, s_sigv4_fail_signature_param_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_amz_date_param_request,
    "GET /?X-Amz-Date=Tomorrow HTTP/1.1\n"
    "Host:example.amazonaws.com\n");

static int s_sigv4_fail_date_param_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_amz_date_param_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_QUERY_PARAM);
}
AWS_TEST_CASE(sigv4_fail_date_param_test, s_sigv4_fail_date_param_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_amz_credential_param_request,
    "GET /?X-Amz-Credential=TopSekrit HTTP/1.1\n"
    "Host:example.amazonaws.com\n");

static int s_sigv4_fail_credential_param_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_amz_credential_param_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_QUERY_PARAM);
}
AWS_TEST_CASE(sigv4_fail_credential_param_test, s_sigv4_fail_credential_param_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_amz_algorithm_param_request,
    "GET /?X-Amz-Algorithm=BubbleSort HTTP/1.1\n"
    "Host:example.amazonaws.com\n");

static int s_sigv4_fail_algorithm_param_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_amz_algorithm_param_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_QUERY_PARAM);
}
AWS_TEST_CASE(sigv4_fail_algorithm_param_test, s_sigv4_fail_algorithm_param_test);

AWS_STATIC_STRING_FROM_LITERAL(
    s_amz_signed_headers_param_request,
    "GET /?X-Amz-SignedHeaders=User-Agent HTTP/1.1\n"
    "Host:example.amazonaws.com\n");

static int s_sigv4_fail_signed_headers_param_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    return s_do_forbidden_header_param_test(
        allocator, s_amz_signed_headers_param_request, AWS_AUTH_SIGNING_ILLEGAL_REQUEST_QUERY_PARAM);
}
AWS_TEST_CASE(sigv4_fail_signed_headers_param_test, s_sigv4_fail_signed_headers_param_test);

struct null_credentials_state {
    struct aws_signing_result *result;
    int error_code;
};

static void s_null_credentials_on_signing_complete(struct aws_signing_result *result, int error_code, void *userdata) {

    struct null_credentials_state *state = userdata;
    state->result = result;
    state->error_code = error_code;
}

static int s_signer_null_credentials_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct get_credentials_mock_result results = {
        .credentials = NULL,
        .error_code = AWS_AUTH_SIGNING_NO_CREDENTIALS,
    };

    struct aws_http_message *request = aws_http_message_new_request(allocator);
    struct aws_signable *signable = aws_signable_new_http_request(allocator, request);

    struct aws_signing_config_aws config = {
        .config_type = AWS_SIGNING_CONFIG_AWS,
        .algorithm = AWS_SIGNING_ALGORITHM_V4,
        .signature_type = AWS_ST_HTTP_REQUEST_HEADERS,
        .region = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("us-east-1"),
        .service = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("elasticdohickeyservice"),
    };
    config.credentials_provider = aws_credentials_provider_new_mock(allocator, &results, 1, NULL);
    aws_date_time_init_now(&config.date);

    struct null_credentials_state state;
    AWS_ZERO_STRUCT(state);

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        signable,
        (struct aws_signing_config_base *)&config,
        s_null_credentials_on_signing_complete,
        &state));

    ASSERT_PTR_EQUALS(NULL, state.result);
    ASSERT_INT_EQUALS(AWS_AUTH_SIGNING_NO_CREDENTIALS, state.error_code);

    aws_credentials_provider_release(config.credentials_provider);
    aws_signable_destroy(signable);
    aws_http_message_release(request);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(signer_null_credentials_test, s_signer_null_credentials_test);

static void s_anonymous_credentials_on_signing_complete(
    struct aws_signing_result *result,
    int error_code,
    void *userdata) {

    struct null_credentials_state *state = userdata;
    state->result = result;
    state->error_code = error_code;
    AWS_FATAL_ASSERT(result->properties.p_impl != NULL);
    AWS_FATAL_ASSERT(result->property_lists.p_impl != NULL);
}

static int s_signer_anonymous_credentials_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_credentials *credentials = aws_credentials_new_anonymous(allocator);
    ASSERT_NOT_NULL(credentials);

    struct aws_http_message *request = aws_http_message_new_request(allocator);
    struct aws_signable *signable = aws_signable_new_http_request(allocator, request);

    struct aws_signing_config_aws config = {
        .config_type = AWS_SIGNING_CONFIG_AWS,
        .algorithm = AWS_SIGNING_ALGORITHM_V4,
        .signature_type = AWS_ST_HTTP_REQUEST_HEADERS,
        .region = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("us-east-1"),
        .service = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("elasticdohickeyservice"),
        .credentials = credentials,
    };

    struct null_credentials_state state;
    AWS_ZERO_STRUCT(state);

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        signable,
        (struct aws_signing_config_base *)&config,
        s_anonymous_credentials_on_signing_complete,
        &state));

    ASSERT_INT_EQUALS(AWS_ERROR_SUCCESS, state.error_code);
    ASSERT_NOT_NULL(state.result);

    aws_credentials_release(credentials);
    aws_signable_destroy(signable);
    aws_http_message_release(request);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(signer_anonymous_credentials_test, s_signer_anonymous_credentials_test);

static int s_signer_anonymous_credentials_provider_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct get_credentials_mock_result mock_result = {
        .credentials = aws_credentials_new_anonymous(allocator),
        .error_code = AWS_ERROR_SUCCESS,
    };

    struct aws_http_message *request = aws_http_message_new_request(allocator);
    struct aws_signable *signable = aws_signable_new_http_request(allocator, request);

    struct aws_signing_config_aws config = {
        .config_type = AWS_SIGNING_CONFIG_AWS,
        .algorithm = AWS_SIGNING_ALGORITHM_V4,
        .signature_type = AWS_ST_HTTP_REQUEST_HEADERS,
        .region = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("us-east-1"),
        .service = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("elasticdohickeyservice"),
    };

    config.credentials_provider = aws_credentials_provider_new_mock(allocator, &mock_result, 1, NULL);

    struct null_credentials_state state;
    AWS_ZERO_STRUCT(state);

    ASSERT_SUCCESS(aws_sign_request_aws(
        allocator,
        signable,
        (struct aws_signing_config_base *)&config,
        s_anonymous_credentials_on_signing_complete,
        &state));

    ASSERT_INT_EQUALS(AWS_ERROR_SUCCESS, state.error_code);
    ASSERT_NOT_NULL(state.result);

    aws_credentials_release(mock_result.credentials);
    aws_credentials_provider_release(config.credentials_provider);
    aws_signable_destroy(signable);
    aws_http_message_release(request);

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(signer_anonymous_credentials_provider_test, s_signer_anonymous_credentials_provider_test);
