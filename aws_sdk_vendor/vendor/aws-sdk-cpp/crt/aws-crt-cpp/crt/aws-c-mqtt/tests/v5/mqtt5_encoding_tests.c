/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt5_testing_utils.h"
#include <aws/common/string.h>
#include <aws/io/stream.h>
#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/v5/mqtt5_decoder.h>
#include <aws/mqtt/private/v5/mqtt5_encoder.h>
#include <aws/mqtt/private/v5/mqtt5_utils.h>
#include <aws/mqtt/v5/mqtt5_types.h>

#include <aws/testing/aws_test_harness.h>

static int s_mqtt5_vli_size_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    size_t encode_size = 0;

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(0, &encode_size));
    ASSERT_INT_EQUALS(1, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(1, &encode_size));
    ASSERT_INT_EQUALS(1, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(127, &encode_size));
    ASSERT_INT_EQUALS(1, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(128, &encode_size));
    ASSERT_INT_EQUALS(2, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(256, &encode_size));
    ASSERT_INT_EQUALS(2, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(16383, &encode_size));
    ASSERT_INT_EQUALS(2, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(16384, &encode_size));
    ASSERT_INT_EQUALS(3, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(16385, &encode_size));
    ASSERT_INT_EQUALS(3, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(2097151, &encode_size));
    ASSERT_INT_EQUALS(3, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(2097152, &encode_size));
    ASSERT_INT_EQUALS(4, encode_size);

    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(AWS_MQTT5_MAXIMUM_VARIABLE_LENGTH_INTEGER, &encode_size));
    ASSERT_INT_EQUALS(4, encode_size);

    ASSERT_FAILS(
        aws_mqtt5_get_variable_length_encode_size(AWS_MQTT5_MAXIMUM_VARIABLE_LENGTH_INTEGER + 1, &encode_size));
    ASSERT_FAILS(aws_mqtt5_get_variable_length_encode_size(0xFFFFFFFF, &encode_size));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_vli_size, s_mqtt5_vli_size_fn)

static int s_do_success_round_trip_vli_test(uint32_t value, struct aws_allocator *allocator) {
    struct aws_byte_buf buffer;
    aws_byte_buf_init(&buffer, allocator, 4);

    ASSERT_SUCCESS(aws_mqtt5_encode_variable_length_integer(&buffer, value));

    size_t encoded_size = 0;
    ASSERT_SUCCESS(aws_mqtt5_get_variable_length_encode_size(value, &encoded_size));
    ASSERT_INT_EQUALS(encoded_size, buffer.len);

    uint32_t decoded_value = 0;
    for (size_t i = 1; i < encoded_size; ++i) {
        struct aws_byte_cursor partial_cursor = aws_byte_cursor_from_buf(&buffer);
        partial_cursor.len = i;

        enum aws_mqtt5_decode_result_type result = aws_mqtt5_decode_vli(&partial_cursor, &decoded_value);
        ASSERT_INT_EQUALS(AWS_MQTT5_DRT_MORE_DATA, result);
    }

    struct aws_byte_cursor full_cursor = aws_byte_cursor_from_buf(&buffer);

    enum aws_mqtt5_decode_result_type result = aws_mqtt5_decode_vli(&full_cursor, &decoded_value);
    ASSERT_INT_EQUALS(AWS_MQTT5_DRT_SUCCESS, result);
    ASSERT_INT_EQUALS(decoded_value, value);

    aws_byte_buf_clean_up(&buffer);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_vli_success_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(0, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(1, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(47, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(127, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(128, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(129, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(511, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(8000, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(16383, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(16384, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(16385, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(100000, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(4200000, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(34200000, allocator));
    ASSERT_SUCCESS(s_do_success_round_trip_vli_test(AWS_MQTT5_MAXIMUM_VARIABLE_LENGTH_INTEGER, allocator));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_vli_success_round_trip, s_mqtt5_vli_success_round_trip_fn)

static int s_mqtt5_vli_encode_failures_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_buf buffer;
    aws_byte_buf_init(&buffer, allocator, 4);

    ASSERT_FAILS(aws_mqtt5_encode_variable_length_integer(&buffer, AWS_MQTT5_MAXIMUM_VARIABLE_LENGTH_INTEGER + 1));
    ASSERT_FAILS(aws_mqtt5_encode_variable_length_integer(&buffer, 0x80000000));
    ASSERT_FAILS(aws_mqtt5_encode_variable_length_integer(&buffer, 0xFFFFFFFF));

    aws_byte_buf_clean_up(&buffer);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_vli_encode_failures, s_mqtt5_vli_encode_failures_fn)

static uint8_t bad_buffers0[] = {0x80, 0x80, 0x80, 0x80};
static uint8_t bad_buffers1[] = {0x81, 0x81, 0x81, 0xFF};

static int s_mqtt5_vli_decode_failures_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    uint32_t value = 0;

    struct aws_byte_cursor cursor = aws_byte_cursor_from_array(&bad_buffers0[0], AWS_ARRAY_SIZE(bad_buffers0));
    ASSERT_INT_EQUALS(AWS_MQTT5_DRT_ERROR, aws_mqtt5_decode_vli(&cursor, &value));

    cursor = aws_byte_cursor_from_array(&bad_buffers1[0], AWS_ARRAY_SIZE(bad_buffers1));
    ASSERT_INT_EQUALS(AWS_MQTT5_DRT_ERROR, aws_mqtt5_decode_vli(&cursor, &value));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_vli_decode_failures, s_mqtt5_vli_decode_failures_fn)

static char s_user_prop1_name[] = "Property1";
static char s_user_prop1_value[] = "Value1";
static char s_user_prop2_name[] = "Property2";
static char s_user_prop2_value[] = "Value2";
static const struct aws_mqtt5_user_property s_user_properties[] = {
    {
        .name =
            {
                .ptr = (uint8_t *)s_user_prop1_name,
                .len = AWS_ARRAY_SIZE(s_user_prop1_name) - 1,
            },
        .value =
            {
                .ptr = (uint8_t *)s_user_prop1_value,
                .len = AWS_ARRAY_SIZE(s_user_prop1_value) - 1,
            },
    },
    {
        .name =
            {
                .ptr = (uint8_t *)s_user_prop2_name,
                .len = AWS_ARRAY_SIZE(s_user_prop2_name) - 1,
            },
        .value =
            {
                .ptr = (uint8_t *)s_user_prop2_value,
                .len = AWS_ARRAY_SIZE(s_user_prop2_value) - 1,
            },
    },
};

static const char *s_reason_string = "This is why I'm disconnecting";
static const char *s_server_reference = "connect-here-instead.com";
static const char *s_will_payload = "{\"content\":\"This is the payload of a will message\"}";
static const char *s_will_topic = "zomg/where-did/my/connection-go";
static const char *s_will_response_topic = "TheWillResponseTopic";
static const char *s_will_correlation_data = "ThisAndThat";
static const char *s_will_content_type = "Json";
static const char *s_client_id = "DeviceNumber47";
static const char *s_username = "MyUser";
static const char *s_password = "SuprSekritDontRead";

struct aws_mqtt5_encode_decode_tester {
    struct aws_mqtt5_encoder_function_table encoder_function_table;
    struct aws_mqtt5_decoder_function_table decoder_function_table;

    size_t view_count;

    size_t expected_view_count;
    void *expected_views;
};

static void s_aws_mqtt5_encode_decoder_tester_init_single_view(
    struct aws_mqtt5_encode_decode_tester *tester,
    void *expected_view) {
    tester->view_count = 0;
    tester->expected_view_count = 1;
    tester->expected_views = expected_view;

    aws_mqtt5_encode_init_testing_function_table(&tester->encoder_function_table);
    aws_mqtt5_decode_init_testing_function_table(&tester->decoder_function_table);
}

int s_check_packet_encoding_reserved_flags(struct aws_byte_buf *encoding, enum aws_mqtt5_packet_type packet_type) {
    if (packet_type == AWS_MQTT5_PT_PUBLISH) {
        return AWS_OP_SUCCESS;
    }

    uint8_t expected_reserved_flags_value = 0;
    switch (packet_type) {
        case AWS_MQTT5_PT_PUBREL:
        case AWS_MQTT5_PT_SUBSCRIBE:
        case AWS_MQTT5_PT_UNSUBSCRIBE:
            expected_reserved_flags_value = 0x02;
            break;

        default:
            break;
    }

    uint8_t first_byte = encoding->buffer[0];

    uint8_t encoded_packet_type = (first_byte >> 4) & 0x0F;
    ASSERT_INT_EQUALS(packet_type, encoded_packet_type);

    uint8_t reserved_flags = first_byte & 0x0F;
    ASSERT_INT_EQUALS(expected_reserved_flags_value, reserved_flags);

    return AWS_OP_SUCCESS;
}

typedef int(aws_mqtt5_check_encoding_fn)(struct aws_byte_buf *encoding);

struct aws_mqtt5_packet_round_trip_test_context {
    struct aws_allocator *allocator;
    enum aws_mqtt5_packet_type packet_type;
    void *packet_view;
    aws_mqtt5_on_packet_received_fn *decoder_callback;
    aws_mqtt5_check_encoding_fn *encoding_checker;
};

static int s_aws_mqtt5_encode_decode_round_trip_test(
    struct aws_mqtt5_packet_round_trip_test_context *context,
    size_t encode_fragment_size,
    size_t decode_fragment_size) {
    struct aws_byte_buf whole_dest;
    aws_byte_buf_init(&whole_dest, context->allocator, 4096);

    struct aws_mqtt5_encode_decode_tester tester;
    s_aws_mqtt5_encode_decoder_tester_init_single_view(&tester, context->packet_view);

    struct aws_mqtt5_encoder_options encoder_options = {
        .encoders = &tester.encoder_function_table,
    };

    struct aws_mqtt5_encoder encoder;
    ASSERT_SUCCESS(aws_mqtt5_encoder_init(&encoder, context->allocator, &encoder_options));
    ASSERT_SUCCESS(aws_mqtt5_encoder_append_packet_encoding(&encoder, context->packet_type, context->packet_view));

    enum aws_mqtt5_encoding_result result = AWS_MQTT5_ER_OUT_OF_ROOM;
    while (result == AWS_MQTT5_ER_OUT_OF_ROOM) {
        struct aws_byte_buf fragment_dest;
        aws_byte_buf_init(&fragment_dest, context->allocator, encode_fragment_size);

        result = aws_mqtt5_encoder_encode_to_buffer(&encoder, &fragment_dest);
        ASSERT_TRUE(result != AWS_MQTT5_ER_ERROR);

        struct aws_byte_cursor fragment_cursor = aws_byte_cursor_from_buf(&fragment_dest);
        ASSERT_SUCCESS(aws_byte_buf_append_dynamic(&whole_dest, &fragment_cursor));

        aws_byte_buf_clean_up(&fragment_dest);
    }

    /*
     * For packet types that support encoded size calculations (outgoing operations), verify that the encoded size
     * calculation matches the length we encoded
     */
    size_t expected_packet_size = 0;
    if (!aws_mqtt5_packet_view_get_encoded_size(context->packet_type, context->packet_view, &expected_packet_size)) {
        ASSERT_INT_EQUALS(whole_dest.len, expected_packet_size);
    }

    ASSERT_SUCCESS(s_check_packet_encoding_reserved_flags(&whole_dest, context->packet_type));

    ASSERT_INT_EQUALS(AWS_MQTT5_ER_FINISHED, result);

    struct aws_mqtt5_decoder_options decoder_options = {
        .on_packet_received = context->decoder_callback,
        .callback_user_data = &tester,
        .decoder_table = &tester.decoder_function_table,
    };

    struct aws_mqtt5_decoder decoder;
    ASSERT_SUCCESS(aws_mqtt5_decoder_init(&decoder, context->allocator, &decoder_options));

    struct aws_mqtt5_inbound_topic_alias_resolver inbound_alias_resolver;
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_init(&inbound_alias_resolver, context->allocator));
    ASSERT_SUCCESS(aws_mqtt5_inbound_topic_alias_resolver_reset(&inbound_alias_resolver, 65535));
    aws_mqtt5_decoder_set_inbound_topic_alias_resolver(&decoder, &inbound_alias_resolver);

    struct aws_byte_cursor whole_cursor = aws_byte_cursor_from_buf(&whole_dest);
    while (whole_cursor.len > 0) {
        size_t advance = aws_min_size(whole_cursor.len, decode_fragment_size);
        struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&whole_cursor, advance);

        ASSERT_SUCCESS(aws_mqtt5_decoder_on_data_received(&decoder, fragment_cursor));
    }

    ASSERT_INT_EQUALS(1, tester.view_count);

    /* Now do a check where we partially decode the buffer, reset, and then fully decode. */
    aws_mqtt5_decoder_reset(&decoder);
    tester.view_count = 0;

    whole_cursor = aws_byte_cursor_from_buf(&whole_dest);
    if (decode_fragment_size >= whole_cursor.len) {
        whole_cursor.len--;
    } else {
        whole_cursor.len -= decode_fragment_size;
    }

    while (whole_cursor.len > 0) {
        size_t advance = aws_min_size(whole_cursor.len, decode_fragment_size);
        struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&whole_cursor, advance);

        ASSERT_SUCCESS(aws_mqtt5_decoder_on_data_received(&decoder, fragment_cursor));
    }

    /* Nothing should have been received */
    ASSERT_INT_EQUALS(0, tester.view_count);

    /* Reset and decode the whole packet, everything should be fine */
    aws_mqtt5_decoder_reset(&decoder);
    whole_cursor = aws_byte_cursor_from_buf(&whole_dest);
    while (whole_cursor.len > 0) {
        size_t advance = aws_min_size(whole_cursor.len, decode_fragment_size);
        struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&whole_cursor, advance);

        ASSERT_SUCCESS(aws_mqtt5_decoder_on_data_received(&decoder, fragment_cursor));
    }

    ASSERT_INT_EQUALS(1, tester.view_count);

    aws_byte_buf_clean_up(&whole_dest);
    aws_mqtt5_inbound_topic_alias_resolver_clean_up(&inbound_alias_resolver);
    aws_mqtt5_encoder_clean_up(&encoder);
    aws_mqtt5_decoder_clean_up(&decoder);

    return AWS_OP_SUCCESS;
}

static size_t s_encode_fragment_sizes[] = {4, 5, 7, 65536};
static size_t s_decode_fragment_sizes[] = {1, 2, 3, 11, 65536};

static int s_aws_mqtt5_encode_decode_round_trip_matrix_test(struct aws_mqtt5_packet_round_trip_test_context *context) {
    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_encode_fragment_sizes); ++i) {
        size_t encode_fragment_size = s_encode_fragment_sizes[i];
        for (size_t j = 0; j < AWS_ARRAY_SIZE(s_decode_fragment_sizes); ++j) {
            size_t decode_fragment_size = s_decode_fragment_sizes[j];

            ASSERT_SUCCESS(
                s_aws_mqtt5_encode_decode_round_trip_test(context, encode_fragment_size, decode_fragment_size));
        }
    }

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_on_disconnect_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_DISCONNECT, (uint32_t)type);

    struct aws_mqtt5_packet_disconnect_view *disconnect_view = packet_view;
    struct aws_mqtt5_packet_disconnect_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS((uint32_t)expected_view->reason_code, (uint32_t)disconnect_view->reason_code);
    ASSERT_INT_EQUALS(
        *expected_view->session_expiry_interval_seconds, *disconnect_view->session_expiry_interval_seconds);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->reason_string->ptr,
        expected_view->reason_string->len,
        disconnect_view->reason_string->ptr,
        disconnect_view->reason_string->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->server_reference->ptr,
        expected_view->server_reference->len,
        disconnect_view->server_reference->ptr,
        disconnect_view->server_reference->len);
    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        disconnect_view->user_property_count,
        disconnect_view->user_properties,
        expected_view->user_property_count,
        expected_view->user_properties));

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_disconnect_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint32_t session_expiry_interval_seconds = 333;
    struct aws_byte_cursor reason_string_cursor = aws_byte_cursor_from_c_str(s_reason_string);
    struct aws_byte_cursor server_reference_cursor = aws_byte_cursor_from_c_str(s_server_reference);

    struct aws_mqtt5_packet_disconnect_view disconnect_view = {
        .reason_code = AWS_MQTT5_DRC_DISCONNECT_WITH_WILL_MESSAGE,
        .session_expiry_interval_seconds = &session_expiry_interval_seconds,
        .reason_string = &reason_string_cursor,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .server_reference = &server_reference_cursor,
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_DISCONNECT,
        .packet_view = &disconnect_view,
        .decoder_callback = s_aws_mqtt5_on_disconnect_received_fn,
    };
    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_disconnect_round_trip, s_mqtt5_packet_disconnect_round_trip_fn)

static int s_aws_mqtt5_on_pingreq_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_PINGREQ, (uint32_t)type);
    ASSERT_NULL(packet_view);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_pingreq_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_PINGREQ,
        .packet_view = NULL,
        .decoder_callback = s_aws_mqtt5_on_pingreq_received_fn,
    };
    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_pingreq_round_trip, s_mqtt5_packet_pingreq_round_trip_fn)

static int s_aws_mqtt5_on_pingresp_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_PINGRESP, (uint32_t)type);
    ASSERT_NULL(packet_view);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_pingresp_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_PINGRESP,
        .packet_view = NULL,
        .decoder_callback = s_aws_mqtt5_on_pingresp_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_pingresp_round_trip, s_mqtt5_packet_pingresp_round_trip_fn)

static int s_aws_mqtt5_on_connect_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_CONNECT, (uint32_t)type);

    struct aws_mqtt5_packet_connect_view *connect_view = packet_view;
    struct aws_mqtt5_packet_connect_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS(expected_view->keep_alive_interval_seconds, connect_view->keep_alive_interval_seconds);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->client_id.ptr,
        expected_view->client_id.len,
        connect_view->client_id.ptr,
        connect_view->client_id.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->username->ptr,
        expected_view->username->len,
        connect_view->username->ptr,
        connect_view->username->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->password->ptr,
        expected_view->password->len,
        connect_view->password->ptr,
        connect_view->password->len);
    ASSERT_TRUE(expected_view->clean_start == connect_view->clean_start);
    ASSERT_INT_EQUALS(*expected_view->session_expiry_interval_seconds, *connect_view->session_expiry_interval_seconds);

    ASSERT_INT_EQUALS(*expected_view->request_response_information, *connect_view->request_response_information);
    ASSERT_INT_EQUALS(*expected_view->request_problem_information, *connect_view->request_problem_information);
    ASSERT_INT_EQUALS(*expected_view->receive_maximum, *connect_view->receive_maximum);
    ASSERT_INT_EQUALS(*expected_view->topic_alias_maximum, *connect_view->topic_alias_maximum);
    ASSERT_INT_EQUALS(*expected_view->maximum_packet_size_bytes, *connect_view->maximum_packet_size_bytes);

    ASSERT_INT_EQUALS(*expected_view->will_delay_interval_seconds, *connect_view->will_delay_interval_seconds);

    const struct aws_mqtt5_packet_publish_view *expected_will_view = expected_view->will;
    const struct aws_mqtt5_packet_publish_view *will_view = connect_view->will;

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_will_view->payload.ptr,
        expected_will_view->payload.len,
        will_view->payload.ptr,
        will_view->payload.len);
    ASSERT_INT_EQUALS(expected_will_view->qos, will_view->qos);
    ASSERT_INT_EQUALS(expected_will_view->retain, will_view->retain);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_will_view->topic.ptr, expected_will_view->topic.len, will_view->topic.ptr, will_view->topic.len);
    ASSERT_INT_EQUALS(*expected_will_view->payload_format, *will_view->payload_format);
    ASSERT_INT_EQUALS(
        *expected_will_view->message_expiry_interval_seconds, *will_view->message_expiry_interval_seconds);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_will_view->response_topic->ptr,
        expected_will_view->response_topic->len,
        will_view->response_topic->ptr,
        will_view->response_topic->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_will_view->correlation_data->ptr,
        expected_will_view->correlation_data->len,
        will_view->correlation_data->ptr,
        will_view->correlation_data->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_will_view->content_type->ptr,
        expected_will_view->content_type->len,
        will_view->content_type->ptr,
        will_view->content_type->len);

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_will_view->user_property_count,
        expected_will_view->user_properties,
        will_view->user_property_count,
        will_view->user_properties));

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        connect_view->user_property_count,
        connect_view->user_properties));

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->authentication_method->ptr,
        expected_view->authentication_method->len,
        connect_view->authentication_method->ptr,
        connect_view->authentication_method->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->authentication_data->ptr,
        expected_view->authentication_data->len,
        connect_view->authentication_data->ptr,
        connect_view->authentication_data->len);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_encode_connect_to_buffer(
    struct aws_allocator *allocator,
    struct aws_mqtt5_packet_connect_view *connect_view,
    struct aws_byte_buf *buffer) {
    AWS_ZERO_STRUCT(*buffer);
    aws_byte_buf_init(buffer, allocator, 4096);

    struct aws_mqtt5_encoder_options encoder_options;
    AWS_ZERO_STRUCT(encoder_options);

    struct aws_mqtt5_encoder encoder;
    ASSERT_SUCCESS(aws_mqtt5_encoder_init(&encoder, allocator, &encoder_options));
    ASSERT_SUCCESS(aws_mqtt5_encoder_append_packet_encoding(&encoder, AWS_MQTT5_PT_CONNECT, connect_view));

    enum aws_mqtt5_encoding_result result = aws_mqtt5_encoder_encode_to_buffer(&encoder, buffer);
    ASSERT_INT_EQUALS(AWS_MQTT5_ER_FINISHED, result);

    aws_mqtt5_encoder_clean_up(&encoder);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_connect_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor will_payload_cursor = aws_byte_cursor_from_c_str(s_will_payload);
    enum aws_mqtt5_payload_format_indicator payload_format = AWS_MQTT5_PFI_UTF8;
    uint32_t message_expiry_interval_seconds = 65537;
    struct aws_byte_cursor will_response_topic = aws_byte_cursor_from_c_str(s_will_response_topic);
    struct aws_byte_cursor will_correlation_data = aws_byte_cursor_from_c_str(s_will_correlation_data);
    struct aws_byte_cursor will_content_type = aws_byte_cursor_from_c_str(s_will_content_type);
    struct aws_byte_cursor username = aws_byte_cursor_from_c_str(s_username);
    struct aws_byte_cursor password = aws_byte_cursor_from_c_str(s_password);
    uint32_t session_expiry_interval_seconds = 3600;
    uint8_t request_response_information = 1;
    uint8_t request_problem_information = 1;
    uint16_t receive_maximum = 50;
    uint16_t topic_alias_maximum = 16;
    uint32_t maximum_packet_size_bytes = 1ULL << 24;
    uint32_t will_delay_interval_seconds = 30;
    struct aws_byte_cursor authentication_method = aws_byte_cursor_from_c_str("AuthMethod");
    struct aws_byte_cursor authentication_data = aws_byte_cursor_from_c_str("SuperSecret");

    struct aws_mqtt5_packet_publish_view will_view = {
        .payload = will_payload_cursor,
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .retain = true,
        .topic = aws_byte_cursor_from_c_str(s_will_topic),
        .payload_format = &payload_format,
        .message_expiry_interval_seconds = &message_expiry_interval_seconds,
        .response_topic = &will_response_topic,
        .correlation_data = &will_correlation_data,
        .content_type = &will_content_type,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds = 1200,
        .client_id = aws_byte_cursor_from_c_str(s_client_id),
        .username = &username,
        .password = &password,
        .clean_start = true,
        .session_expiry_interval_seconds = &session_expiry_interval_seconds,
        .request_response_information = &request_response_information,
        .request_problem_information = &request_problem_information,
        .receive_maximum = &receive_maximum,
        .topic_alias_maximum = &topic_alias_maximum,
        .maximum_packet_size_bytes = &maximum_packet_size_bytes,
        .will_delay_interval_seconds = &will_delay_interval_seconds,
        .will = &will_view,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .authentication_method = &authentication_method,
        .authentication_data = &authentication_data,
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_CONNECT,
        .packet_view = &connect_view,
        .decoder_callback = s_aws_mqtt5_on_connect_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    struct aws_byte_buf encoding_buffer;
    ASSERT_SUCCESS(s_mqtt5_packet_encode_connect_to_buffer(allocator, &connect_view, &encoding_buffer));

    /*
     * For this packet: 1 byte packet type + flags, 2 bytes vli remaining length + 7 bytes protocol prefix
     * (0x00, 0x04, "MQTT", 0x05), then we're at the CONNECT flags which we want to check
     */
    size_t connect_flags_byte_index = 10;
    uint8_t connect_flags = encoding_buffer.buffer[connect_flags_byte_index];

    /*
     * Verify Will flag (0x04), Will QoS (0x08), Will Retain (0x20), Username (0x80),
     * clean start (0x02), password (0x40) flags are set
     */
    ASSERT_INT_EQUALS(connect_flags, 0xEE);

    aws_byte_buf_clean_up(&encoding_buffer);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_connect_round_trip, s_mqtt5_packet_connect_round_trip_fn)

static int s_aws_mqtt5_on_connack_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_CONNACK, (uint32_t)type);

    struct aws_mqtt5_packet_connack_view *connack_view = packet_view;
    struct aws_mqtt5_packet_connack_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS(expected_view->session_present, connack_view->session_present);
    ASSERT_INT_EQUALS(expected_view->reason_code, connack_view->reason_code);
    ASSERT_INT_EQUALS(*expected_view->session_expiry_interval, *connack_view->session_expiry_interval);
    ASSERT_INT_EQUALS(*expected_view->receive_maximum, *connack_view->receive_maximum);
    ASSERT_INT_EQUALS(*expected_view->maximum_qos, *connack_view->maximum_qos);
    ASSERT_INT_EQUALS(*expected_view->retain_available, *connack_view->retain_available);
    ASSERT_INT_EQUALS(*expected_view->maximum_packet_size, *connack_view->maximum_packet_size);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->assigned_client_identifier->ptr,
        expected_view->assigned_client_identifier->len,
        connack_view->assigned_client_identifier->ptr,
        connack_view->assigned_client_identifier->len);
    ASSERT_INT_EQUALS(*expected_view->topic_alias_maximum, *connack_view->topic_alias_maximum);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->reason_string->ptr,
        expected_view->reason_string->len,
        connack_view->reason_string->ptr,
        connack_view->reason_string->len);

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        connack_view->user_property_count,
        connack_view->user_properties));

    ASSERT_INT_EQUALS(
        *expected_view->wildcard_subscriptions_available, *connack_view->wildcard_subscriptions_available);
    ASSERT_INT_EQUALS(
        *expected_view->subscription_identifiers_available, *connack_view->subscription_identifiers_available);
    ASSERT_INT_EQUALS(*expected_view->shared_subscriptions_available, *connack_view->shared_subscriptions_available);
    ASSERT_INT_EQUALS(*expected_view->server_keep_alive, *connack_view->server_keep_alive);

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->response_information->ptr,
        expected_view->response_information->len,
        connack_view->response_information->ptr,
        connack_view->response_information->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->server_reference->ptr,
        expected_view->server_reference->len,
        connack_view->server_reference->ptr,
        connack_view->server_reference->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->authentication_method->ptr,
        expected_view->authentication_method->len,
        connack_view->authentication_method->ptr,
        connack_view->authentication_method->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->authentication_data->ptr,
        expected_view->authentication_data->len,
        connack_view->authentication_data->ptr,
        connack_view->authentication_data->len);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_connack_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint32_t session_expiry_interval = 3600;
    uint16_t receive_maximum = 20;
    enum aws_mqtt5_qos maximum_qos = AWS_MQTT5_QOS_AT_LEAST_ONCE;
    bool retain_available = true;
    uint32_t maximum_packet_size = 1U << 18;
    struct aws_byte_cursor assigned_client_identifier = aws_byte_cursor_from_c_str("server-assigned-client-id-01");
    uint16_t topic_alias_maximum = 10;
    struct aws_byte_cursor reason_string = aws_byte_cursor_from_c_str("You've been banned from this server");
    bool wildcard_subscriptions_available = true;
    bool subscription_identifiers_available = true;
    bool shared_subscriptions_available = true;
    uint16_t server_keep_alive = 1200;
    struct aws_byte_cursor response_information = aws_byte_cursor_from_c_str("some/topic-prefix");
    struct aws_byte_cursor server_reference = aws_byte_cursor_from_c_str("www.somewhere-else.com:1883");
    struct aws_byte_cursor authentication_method = aws_byte_cursor_from_c_str("GSSAPI");
    struct aws_byte_cursor authentication_data = aws_byte_cursor_from_c_str("TOP-SECRET");

    struct aws_mqtt5_packet_connack_view connack_view = {
        .session_present = true,
        .reason_code = AWS_MQTT5_CRC_BANNED,
        .session_expiry_interval = &session_expiry_interval,
        .receive_maximum = &receive_maximum,
        .maximum_qos = &maximum_qos,
        .retain_available = &retain_available,
        .maximum_packet_size = &maximum_packet_size,
        .assigned_client_identifier = &assigned_client_identifier,
        .topic_alias_maximum = &topic_alias_maximum,
        .reason_string = &reason_string,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .wildcard_subscriptions_available = &wildcard_subscriptions_available,
        .subscription_identifiers_available = &subscription_identifiers_available,
        .shared_subscriptions_available = &shared_subscriptions_available,
        .server_keep_alive = &server_keep_alive,
        .response_information = &response_information,
        .server_reference = &server_reference,
        .authentication_method = &authentication_method,
        .authentication_data = &authentication_data,
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_CONNACK,
        .packet_view = &connack_view,
        .decoder_callback = s_aws_mqtt5_on_connack_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_connack_round_trip, s_mqtt5_packet_connack_round_trip_fn)

static char s_subscription_topic_1[] = "test_topic_1";
static char s_subscription_topic_2[] = "test_topic_2";
static char s_subscription_topic_3[] = "test_topic_3";

static const struct aws_mqtt5_subscription_view s_subscriptions[] = {
    {.topic_filter =
         {
             .ptr = (uint8_t *)s_subscription_topic_1,
             .len = AWS_ARRAY_SIZE(s_subscription_topic_1) - 1,
         },
     .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
     .no_local = true,
     .retain_as_published = true,
     .retain_handling_type = AWS_MQTT5_RHT_DONT_SEND},
    {.topic_filter =
         {
             .ptr = (uint8_t *)s_subscription_topic_2,
             .len = AWS_ARRAY_SIZE(s_subscription_topic_2) - 1,
         },
     .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
     .no_local = false,
     .retain_as_published = true,
     .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE},
    {.topic_filter =
         {
             .ptr = (uint8_t *)s_subscription_topic_3,
             .len = AWS_ARRAY_SIZE(s_subscription_topic_3) - 1,
         },
     .qos = AWS_MQTT5_QOS_AT_MOST_ONCE,
     .no_local = false,
     .retain_as_published = true,
     .retain_handling_type = AWS_MQTT5_RHT_SEND_ON_SUBSCRIBE_IF_NEW},
};

int aws_mqtt5_test_verify_subscriptions_raw(
    size_t subscription_count,
    const struct aws_mqtt5_subscription_view *subscriptions,
    size_t expected_count,
    const struct aws_mqtt5_subscription_view *expected_subscriptions) {

    ASSERT_UINT_EQUALS(expected_count, subscription_count);

    for (size_t i = 0; i < expected_count; ++i) {
        const struct aws_mqtt5_subscription_view *expected_subscription = &expected_subscriptions[i];
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_subscription->topic_filter.ptr,
            expected_subscription->topic_filter.len,
            subscriptions[i].topic_filter.ptr,
            subscriptions[i].topic_filter.len);
        ASSERT_INT_EQUALS(expected_subscription->qos, subscriptions[i].qos);
        ASSERT_INT_EQUALS(expected_subscription->no_local, subscriptions[i].no_local);
        ASSERT_INT_EQUALS(expected_subscription->retain_as_published, subscriptions[i].retain_as_published);
        ASSERT_INT_EQUALS(expected_subscription->retain_handling_type, subscriptions[i].retain_handling_type);
    }

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_on_subscribe_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    (void)type;

    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    struct aws_mqtt5_packet_subscribe_view *subscribe_view = packet_view;
    struct aws_mqtt5_packet_subscribe_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS(expected_view->packet_id, subscribe_view->packet_id);
    ASSERT_SUCCESS(aws_mqtt5_test_verify_subscriptions_raw(
        expected_view->subscription_count,
        expected_view->subscriptions,
        subscribe_view->subscription_count,
        subscribe_view->subscriptions));

    ASSERT_INT_EQUALS(*expected_view->subscription_identifier, *subscribe_view->subscription_identifier);

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        subscribe_view->user_property_count,
        subscribe_view->user_properties));

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_subscribe_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;
    uint32_t subscription_identifier = 1;

    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .packet_id = packet_id,
        .subscription_count = AWS_ARRAY_SIZE(s_subscriptions),
        .subscriptions = &s_subscriptions[0],
        .subscription_identifier = &subscription_identifier,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_SUBSCRIBE,
        .packet_view = &subscribe_view,
        .decoder_callback = s_aws_mqtt5_on_subscribe_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_subscribe_round_trip, s_mqtt5_packet_subscribe_round_trip_fn)

static int s_aws_mqtt5_on_suback_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_SUBACK, (uint32_t)type);

    struct aws_mqtt5_packet_suback_view *suback_view = packet_view;
    struct aws_mqtt5_packet_suback_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS(expected_view->packet_id, suback_view->packet_id);

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->reason_string->ptr,
        expected_view->reason_string->len,
        suback_view->reason_string->ptr,
        suback_view->reason_string->len);

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        suback_view->user_property_count,
        suback_view->user_properties));

    ASSERT_INT_EQUALS(expected_view->reason_code_count, suback_view->reason_code_count);
    for (size_t i = 0; i < suback_view->reason_code_count; ++i) {
        ASSERT_INT_EQUALS(expected_view->reason_codes[i], suback_view->reason_codes[i]);
    }
    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_suback_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;
    struct aws_byte_cursor reason_string = aws_byte_cursor_from_c_str("Some random reason string");
    enum aws_mqtt5_suback_reason_code reason_code_1 = AWS_MQTT5_SARC_GRANTED_QOS_0;
    enum aws_mqtt5_suback_reason_code reason_code_2 = AWS_MQTT5_SARC_GRANTED_QOS_1;
    enum aws_mqtt5_suback_reason_code reason_code_3 = AWS_MQTT5_SARC_GRANTED_QOS_2;
    const enum aws_mqtt5_suback_reason_code reason_codes[3] = {
        reason_code_1,
        reason_code_2,
        reason_code_3,
    };

    struct aws_mqtt5_packet_suback_view suback_view = {
        .packet_id = packet_id,
        .reason_string = &reason_string,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .reason_code_count = 3,
        .reason_codes = &reason_codes[0],
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_SUBACK,
        .packet_view = &suback_view,
        .decoder_callback = s_aws_mqtt5_on_suback_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_suback_round_trip, s_mqtt5_packet_suback_round_trip_fn)

static char s_unsubscribe_topic_1[] = "test_topic_1";
static char s_unsubscribe_topic_2[] = "test_topic_2";
static char s_unsubscribe_topic_3[] = "test_topic_3";

static const struct aws_byte_cursor s_unsubscribe_topics[] = {
    {
        .ptr = (uint8_t *)s_unsubscribe_topic_1,
        .len = AWS_ARRAY_SIZE(s_unsubscribe_topic_1) - 1,
    },
    {
        .ptr = (uint8_t *)s_unsubscribe_topic_2,
        .len = AWS_ARRAY_SIZE(s_unsubscribe_topic_2) - 1,
    },
    {
        .ptr = (uint8_t *)s_unsubscribe_topic_3,
        .len = AWS_ARRAY_SIZE(s_unsubscribe_topic_3) - 1,
    },
};

static int s_aws_mqtt5_on_unsubscribe_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    (void)type;

    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    struct aws_mqtt5_packet_unsubscribe_view *unsubscribe_view = packet_view;
    struct aws_mqtt5_packet_unsubscribe_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS(expected_view->packet_id, unsubscribe_view->packet_id);

    ASSERT_INT_EQUALS(expected_view->topic_filter_count, unsubscribe_view->topic_filter_count);

    for (size_t i = 0; i < unsubscribe_view->topic_filter_count; ++i) {
        const struct aws_byte_cursor unsubscribe_topic = unsubscribe_view->topic_filters[i];
        const struct aws_byte_cursor expected_topic = expected_view->topic_filters[i];
        ASSERT_BIN_ARRAYS_EQUALS(expected_topic.ptr, expected_topic.len, unsubscribe_topic.ptr, unsubscribe_topic.len);
    }

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        unsubscribe_view->user_property_count,
        unsubscribe_view->user_properties));

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_unsubscribe_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;

    struct aws_mqtt5_packet_unsubscribe_view unsubscribe_view = {
        .packet_id = packet_id,
        .topic_filter_count = AWS_ARRAY_SIZE(s_unsubscribe_topics),
        .topic_filters = &s_unsubscribe_topics[0],
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_UNSUBSCRIBE,
        .packet_view = &unsubscribe_view,
        .decoder_callback = s_aws_mqtt5_on_unsubscribe_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_unsubscribe_round_trip, s_mqtt5_packet_unsubscribe_round_trip_fn)

static int s_aws_mqtt5_on_unsuback_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_UNSUBACK, (uint32_t)type);

    struct aws_mqtt5_packet_unsuback_view *unsuback_view = packet_view;
    struct aws_mqtt5_packet_unsuback_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS(expected_view->packet_id, unsuback_view->packet_id);

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->reason_string->ptr,
        expected_view->reason_string->len,
        unsuback_view->reason_string->ptr,
        unsuback_view->reason_string->len);

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        unsuback_view->user_property_count,
        unsuback_view->user_properties));

    ASSERT_INT_EQUALS(expected_view->reason_code_count, unsuback_view->reason_code_count);
    for (size_t i = 0; i < unsuback_view->reason_code_count; ++i) {
        ASSERT_INT_EQUALS(expected_view->reason_codes[i], unsuback_view->reason_codes[i]);
    }
    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_unsuback_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;
    struct aws_byte_cursor reason_string = aws_byte_cursor_from_c_str("Some random reason string");
    enum aws_mqtt5_unsuback_reason_code reason_code_1 = AWS_MQTT5_UARC_SUCCESS;
    enum aws_mqtt5_unsuback_reason_code reason_code_2 = AWS_MQTT5_UARC_NO_SUBSCRIPTION_EXISTED;
    enum aws_mqtt5_unsuback_reason_code reason_code_3 = AWS_MQTT5_UARC_UNSPECIFIED_ERROR;
    const enum aws_mqtt5_unsuback_reason_code reason_codes[3] = {
        reason_code_1,
        reason_code_2,
        reason_code_3,
    };

    struct aws_mqtt5_packet_unsuback_view unsuback_view = {
        .packet_id = packet_id,
        .reason_string = &reason_string,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .reason_code_count = AWS_ARRAY_SIZE(reason_codes),
        .reason_codes = &reason_codes[0],
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_UNSUBACK,
        .packet_view = &unsuback_view,
        .decoder_callback = s_aws_mqtt5_on_unsuback_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_unsuback_round_trip, s_mqtt5_packet_unsuback_round_trip_fn)

static char s_publish_topic[] = "test_topic";
static char s_publish_payload[] = "test payload contents";
static char s_publish_response_topic[] = "test response topic";
static char s_publish_correlation_data[] = "test correlation data";
static char s_publish_content_type[] = "test content type";

static int s_aws_mqtt5_on_publish_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    (void)type;

    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    struct aws_mqtt5_packet_publish_view *publish_view = packet_view;
    struct aws_mqtt5_packet_publish_view *expected_view = tester->expected_views;

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->payload.ptr, expected_view->payload.len, publish_view->payload.ptr, publish_view->payload.len);
    ASSERT_INT_EQUALS(expected_view->packet_id, publish_view->packet_id);
    ASSERT_INT_EQUALS(expected_view->qos, publish_view->qos);
    ASSERT_INT_EQUALS(expected_view->duplicate, publish_view->duplicate);
    ASSERT_INT_EQUALS(expected_view->retain, publish_view->retain);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->topic.ptr, expected_view->topic.len, publish_view->topic.ptr, publish_view->topic.len);
    ASSERT_INT_EQUALS(*expected_view->payload_format, *publish_view->payload_format);
    ASSERT_INT_EQUALS(*expected_view->message_expiry_interval_seconds, *publish_view->message_expiry_interval_seconds);
    ASSERT_INT_EQUALS(*expected_view->topic_alias, *publish_view->topic_alias);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->response_topic->ptr,
        expected_view->response_topic->len,
        publish_view->response_topic->ptr,
        publish_view->response_topic->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->correlation_data->ptr,
        expected_view->correlation_data->len,
        publish_view->correlation_data->ptr,
        publish_view->correlation_data->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->content_type->ptr,
        expected_view->content_type->len,
        publish_view->content_type->ptr,
        publish_view->content_type->len);
    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        publish_view->user_property_count,
        publish_view->user_properties));

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_publish_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;
    enum aws_mqtt5_payload_format_indicator payload_format = AWS_MQTT5_PFI_UTF8;
    uint32_t message_expiry_interval_seconds = 100;
    uint16_t topic_alias = 1;
    struct aws_byte_cursor response_topic = {
        .ptr = (uint8_t *)s_publish_response_topic,
        .len = AWS_ARRAY_SIZE(s_publish_response_topic) - 1,
    };
    struct aws_byte_cursor correlation_data = {
        .ptr = (uint8_t *)s_publish_correlation_data,
        .len = AWS_ARRAY_SIZE(s_publish_correlation_data) - 1,
    };
    struct aws_byte_cursor content_type = {
        .ptr = (uint8_t *)s_publish_content_type,
        .len = AWS_ARRAY_SIZE(s_publish_content_type) - 1,
    };

    uint32_t subscription_identifiers[2] = {2, 3};

    struct aws_mqtt5_packet_publish_view publish_view = {
        .payload =
            {
                .ptr = (uint8_t *)s_publish_payload,
                .len = AWS_ARRAY_SIZE(s_publish_payload) - 1,
            },
        .packet_id = packet_id,
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .duplicate = false,
        .retain = false,
        .topic =
            {
                .ptr = (uint8_t *)s_publish_topic,
                .len = AWS_ARRAY_SIZE(s_publish_topic) - 1,
            },
        .payload_format = &payload_format,
        .message_expiry_interval_seconds = &message_expiry_interval_seconds,
        .topic_alias = &topic_alias,
        .response_topic = &response_topic,
        .correlation_data = &correlation_data,
        .subscription_identifier_count = AWS_ARRAY_SIZE(subscription_identifiers),
        .subscription_identifiers = subscription_identifiers,
        .content_type = &content_type,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_PUBLISH,
        .packet_view = &publish_view,
        .decoder_callback = s_aws_mqtt5_on_publish_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_publish_round_trip, s_mqtt5_packet_publish_round_trip_fn)

static char s_puback_reason_string[] = "test reason string";

static int s_aws_mqtt5_on_puback_received_fn(enum aws_mqtt5_packet_type type, void *packet_view, void *user_data) {
    (void)type;

    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    struct aws_mqtt5_packet_puback_view *puback_view = packet_view;
    struct aws_mqtt5_packet_puback_view *expected_view = tester->expected_views;
    ASSERT_INT_EQUALS(expected_view->packet_id, puback_view->packet_id);
    ASSERT_INT_EQUALS(expected_view->reason_code, puback_view->reason_code);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->reason_string->ptr,
        expected_view->reason_string->len,
        puback_view->reason_string->ptr,
        puback_view->reason_string->len);
    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        puback_view->user_property_count,
        puback_view->user_properties));

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_puback_round_trip_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;
    enum aws_mqtt5_puback_reason_code reason_code = AWS_MQTT5_PARC_NO_MATCHING_SUBSCRIBERS;
    struct aws_byte_cursor reason_string = {
        .ptr = (uint8_t *)s_puback_reason_string,
        .len = AWS_ARRAY_SIZE(s_puback_reason_string) - 1,
    };

    struct aws_mqtt5_packet_puback_view puback_view = {
        .packet_id = packet_id,
        .reason_code = reason_code,
        .reason_string = &reason_string,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_PUBACK,
        .packet_view = &puback_view,
        .decoder_callback = s_aws_mqtt5_on_puback_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_puback_round_trip, s_mqtt5_packet_puback_round_trip_fn)

static int s_aws_mqtt5_on_no_will_connect_received_fn(
    enum aws_mqtt5_packet_type type,
    void *packet_view,
    void *user_data) {
    struct aws_mqtt5_encode_decode_tester *tester = user_data;

    ++tester->view_count;

    ASSERT_INT_EQUALS((uint32_t)AWS_MQTT5_PT_CONNECT, (uint32_t)type);

    struct aws_mqtt5_packet_connect_view *connect_view = packet_view;
    struct aws_mqtt5_packet_connect_view *expected_view = tester->expected_views;

    ASSERT_INT_EQUALS(expected_view->keep_alive_interval_seconds, connect_view->keep_alive_interval_seconds);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->client_id.ptr,
        expected_view->client_id.len,
        connect_view->client_id.ptr,
        connect_view->client_id.len);

    if (expected_view->username != NULL) {
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_view->username->ptr,
            expected_view->username->len,
            connect_view->username->ptr,
            connect_view->username->len);
    } else {
        ASSERT_NULL(connect_view->username);
    }

    if (expected_view->password != NULL) {
        ASSERT_BIN_ARRAYS_EQUALS(
            expected_view->password->ptr,
            expected_view->password->len,
            connect_view->password->ptr,
            connect_view->password->len);
    } else {
        ASSERT_NULL(connect_view->password);
    }

    ASSERT_TRUE(expected_view->clean_start == connect_view->clean_start);
    ASSERT_INT_EQUALS(*expected_view->session_expiry_interval_seconds, *connect_view->session_expiry_interval_seconds);

    ASSERT_INT_EQUALS(*expected_view->request_response_information, *connect_view->request_response_information);
    ASSERT_INT_EQUALS(*expected_view->request_problem_information, *connect_view->request_problem_information);
    ASSERT_INT_EQUALS(*expected_view->receive_maximum, *connect_view->receive_maximum);
    ASSERT_INT_EQUALS(*expected_view->topic_alias_maximum, *connect_view->topic_alias_maximum);
    ASSERT_INT_EQUALS(*expected_view->maximum_packet_size_bytes, *connect_view->maximum_packet_size_bytes);

    ASSERT_NULL(connect_view->will_delay_interval_seconds);

    ASSERT_NULL(connect_view->will);

    ASSERT_SUCCESS(aws_mqtt5_test_verify_user_properties_raw(
        expected_view->user_property_count,
        expected_view->user_properties,
        connect_view->user_property_count,
        connect_view->user_properties));

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->authentication_method->ptr,
        expected_view->authentication_method->len,
        connect_view->authentication_method->ptr,
        connect_view->authentication_method->len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_view->authentication_data->ptr,
        expected_view->authentication_data->len,
        connect_view->authentication_data->ptr,
        connect_view->authentication_data->len);

    return AWS_OP_SUCCESS;
}

static int s_mqtt5_packet_encode_connect_no_will_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor username = aws_byte_cursor_from_c_str(s_username);
    struct aws_byte_cursor password = aws_byte_cursor_from_c_str(s_password);
    uint32_t session_expiry_interval_seconds = 3600;
    uint8_t request_response_information = 1;
    uint8_t request_problem_information = 1;
    uint16_t receive_maximum = 50;
    uint16_t topic_alias_maximum = 16;
    uint32_t maximum_packet_size_bytes = 1ULL << 24;
    uint32_t will_delay_interval_seconds = 30;
    struct aws_byte_cursor authentication_method = aws_byte_cursor_from_c_str("AuthMethod");
    struct aws_byte_cursor authentication_data = aws_byte_cursor_from_c_str("SuperSecret");

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds = 1200,
        .client_id = aws_byte_cursor_from_c_str(s_client_id),
        .username = &username,
        .password = &password,
        .clean_start = true,
        .session_expiry_interval_seconds = &session_expiry_interval_seconds,
        .request_response_information = &request_response_information,
        .request_problem_information = &request_problem_information,
        .receive_maximum = &receive_maximum,
        .topic_alias_maximum = &topic_alias_maximum,
        .maximum_packet_size_bytes = &maximum_packet_size_bytes,
        .will_delay_interval_seconds = &will_delay_interval_seconds,
        .will = NULL,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .authentication_method = &authentication_method,
        .authentication_data = &authentication_data,
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_CONNECT,
        .packet_view = &connect_view,
        .decoder_callback = s_aws_mqtt5_on_no_will_connect_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    struct aws_byte_buf encoding_buffer;
    ASSERT_SUCCESS(s_mqtt5_packet_encode_connect_to_buffer(allocator, &connect_view, &encoding_buffer));

    /*
     * For this packet: 1 byte packet type + flags, 2 bytes vli remaining length + 7 bytes protocol prefix
     * (0x00, 0x04, "MQTT", 0x05), then we're at the CONNECT flags which we want to check
     */
    size_t connect_flags_byte_index = 10;
    uint8_t connect_flags = encoding_buffer.buffer[connect_flags_byte_index];

    /*
     * Verify Will flag, Will QoS, Will Retain are all zero,
     * while clean start (0x02), password (0x40) and username (0x80) flags are set
     */
    ASSERT_INT_EQUALS(connect_flags, 0xC2);

    aws_byte_buf_clean_up(&encoding_buffer);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_encode_connect_no_will, s_mqtt5_packet_encode_connect_no_will_fn)

static int s_mqtt5_packet_encode_connect_no_username_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor password = aws_byte_cursor_from_c_str(s_password);
    uint32_t session_expiry_interval_seconds = 3600;
    uint8_t request_response_information = 1;
    uint8_t request_problem_information = 1;
    uint16_t receive_maximum = 50;
    uint16_t topic_alias_maximum = 16;
    uint32_t maximum_packet_size_bytes = 1ULL << 24;
    uint32_t will_delay_interval_seconds = 30;
    struct aws_byte_cursor authentication_method = aws_byte_cursor_from_c_str("AuthMethod");
    struct aws_byte_cursor authentication_data = aws_byte_cursor_from_c_str("SuperSecret");

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds = 1200,
        .client_id = aws_byte_cursor_from_c_str(s_client_id),
        .username = NULL,
        .password = &password,
        .clean_start = true,
        .session_expiry_interval_seconds = &session_expiry_interval_seconds,
        .request_response_information = &request_response_information,
        .request_problem_information = &request_problem_information,
        .receive_maximum = &receive_maximum,
        .topic_alias_maximum = &topic_alias_maximum,
        .maximum_packet_size_bytes = &maximum_packet_size_bytes,
        .will_delay_interval_seconds = &will_delay_interval_seconds,
        .will = NULL,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .authentication_method = &authentication_method,
        .authentication_data = &authentication_data,
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_CONNECT,
        .packet_view = &connect_view,
        .decoder_callback = s_aws_mqtt5_on_no_will_connect_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    struct aws_byte_buf encoding_buffer;
    ASSERT_SUCCESS(s_mqtt5_packet_encode_connect_to_buffer(allocator, &connect_view, &encoding_buffer));

    /*
     * For this packet: 1 byte packet type + flags, 2 bytes vli remaining length + 7 bytes protocol prefix
     * (0x00, 0x04, "MQTT", 0x05), then we're at the CONNECT flags which we want to check
     */
    size_t connect_flags_byte_index = 10;
    uint8_t connect_flags = encoding_buffer.buffer[connect_flags_byte_index];

    /*
     * Verify Will flag, Will QoS, Will Retain, Username are all zero,
     * while clean start (0x02), password (0x40) flags are set
     */
    ASSERT_INT_EQUALS(connect_flags, 0x42);

    aws_byte_buf_clean_up(&encoding_buffer);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_encode_connect_no_username, s_mqtt5_packet_encode_connect_no_username_fn)

static int s_mqtt5_packet_encode_connect_no_password_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor username = aws_byte_cursor_from_c_str(s_username);
    uint32_t session_expiry_interval_seconds = 3600;
    uint8_t request_response_information = 1;
    uint8_t request_problem_information = 1;
    uint16_t receive_maximum = 50;
    uint16_t topic_alias_maximum = 16;
    uint32_t maximum_packet_size_bytes = 1ULL << 24;
    uint32_t will_delay_interval_seconds = 30;
    struct aws_byte_cursor authentication_method = aws_byte_cursor_from_c_str("AuthMethod");
    struct aws_byte_cursor authentication_data = aws_byte_cursor_from_c_str("SuperSecret");

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds = 1200,
        .client_id = aws_byte_cursor_from_c_str(s_client_id),
        .username = &username,
        .password = NULL,
        .clean_start = true,
        .session_expiry_interval_seconds = &session_expiry_interval_seconds,
        .request_response_information = &request_response_information,
        .request_problem_information = &request_problem_information,
        .receive_maximum = &receive_maximum,
        .topic_alias_maximum = &topic_alias_maximum,
        .maximum_packet_size_bytes = &maximum_packet_size_bytes,
        .will_delay_interval_seconds = &will_delay_interval_seconds,
        .will = NULL,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .authentication_method = &authentication_method,
        .authentication_data = &authentication_data,
    };

    struct aws_mqtt5_packet_round_trip_test_context context = {
        .allocator = allocator,
        .packet_type = AWS_MQTT5_PT_CONNECT,
        .packet_view = &connect_view,
        .decoder_callback = s_aws_mqtt5_on_no_will_connect_received_fn,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_encode_decode_round_trip_matrix_test(&context));

    struct aws_byte_buf encoding_buffer;
    ASSERT_SUCCESS(s_mqtt5_packet_encode_connect_to_buffer(allocator, &connect_view, &encoding_buffer));

    /*
     * For this packet: 1 byte packet type + flags, 1 byte vli remaining length + 7 bytes protocol prefix
     * (0x00, 0x04, "MQTT", 0x05), then we're at the CONNECT flags which we want to check
     */
    size_t connect_flags_byte_index = 9;
    uint8_t connect_flags = encoding_buffer.buffer[connect_flags_byte_index];

    /*
     * Verify Will flag, Will QoS, Will Retain, Password are all zero,
     * while clean start (0x02), username (0x80) flags are set
     */
    ASSERT_INT_EQUALS(connect_flags, 0x82);

    aws_byte_buf_clean_up(&encoding_buffer);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_encode_connect_no_password, s_mqtt5_packet_encode_connect_no_password_fn)

static int s_mqtt5_packet_encode_connect_will_property_order_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_cursor will_payload_cursor = aws_byte_cursor_from_c_str(s_will_payload);
    enum aws_mqtt5_payload_format_indicator payload_format = AWS_MQTT5_PFI_UTF8;
    uint32_t message_expiry_interval_seconds = 65537;
    struct aws_byte_cursor will_response_topic = aws_byte_cursor_from_c_str(s_will_response_topic);
    struct aws_byte_cursor will_correlation_data = aws_byte_cursor_from_c_str(s_will_correlation_data);
    struct aws_byte_cursor will_content_type = aws_byte_cursor_from_c_str(s_will_content_type);
    struct aws_byte_cursor username = aws_byte_cursor_from_c_str(s_username);
    struct aws_byte_cursor password = aws_byte_cursor_from_c_str(s_password);
    uint32_t session_expiry_interval_seconds = 3600;
    uint8_t request_response_information = 1;
    uint8_t request_problem_information = 1;
    uint16_t receive_maximum = 50;
    uint16_t topic_alias_maximum = 16;
    uint32_t maximum_packet_size_bytes = 1ULL << 24;
    uint32_t will_delay_interval_seconds = 30;
    struct aws_byte_cursor authentication_method = aws_byte_cursor_from_c_str("AuthMethod");
    struct aws_byte_cursor authentication_data = aws_byte_cursor_from_c_str("SuperSecret");

    struct aws_mqtt5_packet_publish_view will_view = {
        .payload = will_payload_cursor,
        .qos = AWS_MQTT5_QOS_AT_LEAST_ONCE,
        .retain = true,
        .topic = aws_byte_cursor_from_c_str(s_will_topic),
        .payload_format = &payload_format,
        .message_expiry_interval_seconds = &message_expiry_interval_seconds,
        .response_topic = &will_response_topic,
        .correlation_data = &will_correlation_data,
        .content_type = &will_content_type,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    struct aws_mqtt5_packet_connect_view connect_view = {
        .keep_alive_interval_seconds = 1200,
        .client_id = aws_byte_cursor_from_c_str(s_client_id),
        .username = &username,
        .password = &password,
        .clean_start = true,
        .session_expiry_interval_seconds = &session_expiry_interval_seconds,
        .request_response_information = &request_response_information,
        .request_problem_information = &request_problem_information,
        .receive_maximum = &receive_maximum,
        .topic_alias_maximum = &topic_alias_maximum,
        .maximum_packet_size_bytes = &maximum_packet_size_bytes,
        .will_delay_interval_seconds = &will_delay_interval_seconds,
        .will = &will_view,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .authentication_method = &authentication_method,
        .authentication_data = &authentication_data,
    };

    struct aws_byte_buf encoding_buffer;
    ASSERT_SUCCESS(s_mqtt5_packet_encode_connect_to_buffer(allocator, &connect_view, &encoding_buffer));

    struct aws_byte_cursor encoding_cursor = aws_byte_cursor_from_buf(&encoding_buffer);

    struct aws_byte_cursor client_id_cursor;
    AWS_ZERO_STRUCT(client_id_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&encoding_cursor, &connect_view.client_id, &client_id_cursor));

    struct aws_byte_cursor will_topic_cursor;
    AWS_ZERO_STRUCT(will_topic_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&encoding_cursor, &will_view.topic, &will_topic_cursor));

    struct aws_byte_cursor will_message_payload_cursor;
    AWS_ZERO_STRUCT(will_message_payload_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&encoding_cursor, &will_view.payload, &will_message_payload_cursor));

    struct aws_byte_cursor username_cursor;
    AWS_ZERO_STRUCT(username_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&encoding_cursor, connect_view.username, &username_cursor));

    struct aws_byte_cursor password_cursor;
    AWS_ZERO_STRUCT(password_cursor);
    ASSERT_SUCCESS(aws_byte_cursor_find_exact(&encoding_cursor, connect_view.password, &password_cursor));

    ASSERT_NOT_NULL(client_id_cursor.ptr);
    ASSERT_NOT_NULL(will_topic_cursor.ptr);
    ASSERT_NOT_NULL(will_message_payload_cursor.ptr);
    ASSERT_NOT_NULL(username_cursor.ptr);
    ASSERT_NOT_NULL(password_cursor.ptr);

    ASSERT_TRUE(client_id_cursor.ptr < will_topic_cursor.ptr);
    ASSERT_TRUE(will_topic_cursor.ptr < will_message_payload_cursor.ptr);
    ASSERT_TRUE(will_message_payload_cursor.ptr < username_cursor.ptr);
    ASSERT_TRUE(username_cursor.ptr < password_cursor.ptr);

    aws_byte_buf_clean_up(&encoding_buffer);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_packet_encode_connect_will_property_order, s_mqtt5_packet_encode_connect_will_property_order_fn)

static int s_aws_mqtt5_decoder_decode_subscribe_first_byte_check(struct aws_mqtt5_decoder *decoder) {
    uint8_t first_byte = decoder->packet_first_byte;
    if ((first_byte & 0x0F) != 2) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}
static int s_aws_mqtt5_decoder_decode_unsubscribe_first_byte_check(struct aws_mqtt5_decoder *decoder) {
    uint8_t first_byte = decoder->packet_first_byte;
    if ((first_byte & 0x0F) != 2) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}
static int s_aws_mqtt5_decoder_decode_disconnect_first_byte_check(struct aws_mqtt5_decoder *decoder) {
    uint8_t first_byte = decoder->packet_first_byte;
    if ((first_byte & 0x0F) != 0) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt5_first_byte_check(
    struct aws_allocator *allocator,
    enum aws_mqtt5_packet_type packet_type,
    void *packet_view) {
    struct aws_byte_buf whole_dest;
    aws_byte_buf_init(&whole_dest, allocator, 4096);

    struct aws_mqtt5_encode_decode_tester tester;
    aws_mqtt5_encode_init_testing_function_table(&tester.encoder_function_table);
    aws_mqtt5_decode_init_testing_function_table(&tester.decoder_function_table);

    tester.decoder_function_table.decoders_by_packet_type[AWS_MQTT5_PT_SUBSCRIBE] =
        &s_aws_mqtt5_decoder_decode_subscribe_first_byte_check;
    tester.decoder_function_table.decoders_by_packet_type[AWS_MQTT5_PT_UNSUBSCRIBE] =
        &s_aws_mqtt5_decoder_decode_unsubscribe_first_byte_check;
    tester.decoder_function_table.decoders_by_packet_type[AWS_MQTT5_PT_DISCONNECT] =
        &s_aws_mqtt5_decoder_decode_disconnect_first_byte_check;

    struct aws_mqtt5_encoder_options encoder_options = {
        .encoders = &tester.encoder_function_table,
    };

    struct aws_mqtt5_encoder encoder;

    ASSERT_SUCCESS(aws_mqtt5_encoder_init(&encoder, allocator, &encoder_options));
    ASSERT_SUCCESS(aws_mqtt5_encoder_append_packet_encoding(&encoder, packet_type, packet_view));

    enum aws_mqtt5_encoding_result result = AWS_MQTT5_ER_ERROR;
    result = aws_mqtt5_encoder_encode_to_buffer(&encoder, &whole_dest);
    ASSERT_INT_EQUALS(AWS_MQTT5_ER_FINISHED, result);

    struct aws_mqtt5_decoder_options decoder_options = {
        .callback_user_data = &tester,
        .decoder_table = &tester.decoder_function_table,
    };

    struct aws_mqtt5_decoder decoder;
    ASSERT_SUCCESS(aws_mqtt5_decoder_init(&decoder, allocator, &decoder_options));

    struct aws_byte_cursor whole_cursor = aws_byte_cursor_from_buf(&whole_dest);
    ASSERT_SUCCESS(aws_mqtt5_decoder_on_data_received(&decoder, whole_cursor));

    aws_byte_buf_clean_up(&whole_dest);
    aws_mqtt5_encoder_clean_up(&encoder);
    aws_mqtt5_decoder_clean_up(&decoder);

    return AWS_OP_SUCCESS;
}

static int mqtt5_first_byte_reserved_header_check_subscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;
    uint32_t subscription_identifier = 1;

    struct aws_mqtt5_packet_subscribe_view subscribe_view = {
        .packet_id = packet_id,
        .subscription_count = AWS_ARRAY_SIZE(s_subscriptions),
        .subscriptions = &s_subscriptions[0],
        .subscription_identifier = &subscription_identifier,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    ASSERT_SUCCESS(s_aws_mqtt5_first_byte_check(allocator, AWS_MQTT5_PT_SUBSCRIBE, &subscribe_view));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_first_byte_reserved_header_check_subscribe, mqtt5_first_byte_reserved_header_check_subscribe_fn)

static int mqtt5_first_byte_reserved_header_check_unsubscribe_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt5_packet_id_t packet_id = 47;

    struct aws_mqtt5_packet_unsubscribe_view unsubscribe_view = {
        .packet_id = packet_id,
        .topic_filter_count = AWS_ARRAY_SIZE(s_unsubscribe_topics),
        .topic_filters = &s_unsubscribe_topics[0],
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
    };

    ASSERT_SUCCESS(s_aws_mqtt5_first_byte_check(allocator, AWS_MQTT5_PT_UNSUBSCRIBE, &unsubscribe_view));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_first_byte_reserved_header_check_unsubscribe, mqtt5_first_byte_reserved_header_check_unsubscribe_fn)

static int mqtt5_first_byte_reserved_header_check_disconnect_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint32_t session_expiry_interval_seconds = 333;
    struct aws_byte_cursor reason_string_cursor = aws_byte_cursor_from_c_str(s_reason_string);
    struct aws_byte_cursor server_reference_cursor = aws_byte_cursor_from_c_str(s_server_reference);

    struct aws_mqtt5_packet_disconnect_view disconnect_view = {
        .reason_code = AWS_MQTT5_DRC_DISCONNECT_WITH_WILL_MESSAGE,
        .session_expiry_interval_seconds = &session_expiry_interval_seconds,
        .reason_string = &reason_string_cursor,
        .user_property_count = AWS_ARRAY_SIZE(s_user_properties),
        .user_properties = &s_user_properties[0],
        .server_reference = &server_reference_cursor,
    };

    ASSERT_SUCCESS(s_aws_mqtt5_first_byte_check(allocator, AWS_MQTT5_PT_DISCONNECT, &disconnect_view));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt5_first_byte_reserved_header_check_disconnect, mqtt5_first_byte_reserved_header_check_disconnect_fn)
