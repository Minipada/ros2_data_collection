/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/mqtt/private/fixed_header.h>
#include <aws/mqtt/private/mqtt311_decoder.h>
#include <aws/mqtt/private/packets.h>

struct mqtt_311_decoding_test_context {
    struct aws_allocator *allocator;
    struct aws_mqtt311_decoder decoder;

    void *expected_packet;
    size_t packet_count[16];
};

static int s_compare_fixed_header(
    struct aws_mqtt_fixed_header *expected_header,
    struct aws_mqtt_fixed_header *actual_header) {
    ASSERT_INT_EQUALS(expected_header->packet_type, actual_header->packet_type);
    ASSERT_INT_EQUALS(expected_header->remaining_length, actual_header->remaining_length);
    ASSERT_INT_EQUALS(expected_header->flags, actual_header->flags);

    return AWS_OP_SUCCESS;
}

static int s_decoding_test_handle_publish(struct aws_byte_cursor message_cursor, void *user_data) {

    struct mqtt_311_decoding_test_context *context = user_data;
    (void)context;

    struct aws_mqtt_packet_publish publish;
    if (aws_mqtt_packet_publish_decode(&message_cursor, &publish)) {
        return AWS_OP_ERR;
    }

    struct aws_mqtt_packet_publish *expected_publish = context->expected_packet;

    ASSERT_SUCCESS(s_compare_fixed_header(&expected_publish->fixed_header, &publish.fixed_header));

    ASSERT_INT_EQUALS(expected_publish->packet_identifier, publish.packet_identifier);

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_publish->topic_name.ptr,
        expected_publish->topic_name.len,
        publish.topic_name.ptr,
        publish.topic_name.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        expected_publish->payload.ptr, expected_publish->payload.len, publish.payload.ptr, publish.payload.len);

    ++context->packet_count[AWS_MQTT_PACKET_PUBLISH];

    return AWS_OP_SUCCESS;
}

static int s_decoding_test_handle_suback(struct aws_byte_cursor message_cursor, void *user_data) {

    struct mqtt_311_decoding_test_context *context = user_data;
    (void)context;

    struct aws_mqtt_packet_suback suback;
    if (aws_mqtt_packet_suback_init(&suback, context->allocator, 0 /* fake packet_id */)) {
        return AWS_OP_ERR;
    }

    int result = AWS_OP_ERR;
    if (aws_mqtt_packet_suback_decode(&message_cursor, &suback)) {
        goto done;
    }

    struct aws_mqtt_packet_suback *expected_suback = context->expected_packet;

    ASSERT_INT_EQUALS(expected_suback->packet_identifier, suback.packet_identifier);

    size_t expected_ack_count = aws_array_list_length(&expected_suback->return_codes);
    size_t actual_ack_count = aws_array_list_length(&suback.return_codes);
    ASSERT_INT_EQUALS(expected_ack_count, actual_ack_count);

    for (size_t i = 0; i < expected_ack_count; ++i) {
        uint8_t expected_return_code = 0;
        aws_array_list_get_at(&expected_suback->return_codes, &expected_return_code, i);

        uint8_t actual_return_code = 0;
        aws_array_list_get_at(&suback.return_codes, &actual_return_code, i);

        ASSERT_INT_EQUALS(expected_return_code, actual_return_code);
    }

    ++context->packet_count[AWS_MQTT_PACKET_SUBACK];

    result = AWS_OP_SUCCESS;

done:

    aws_mqtt_packet_suback_clean_up(&suback);

    return result;
}

static int s_decoding_test_handle_unsuback(struct aws_byte_cursor message_cursor, void *user_data) {

    struct mqtt_311_decoding_test_context *context = user_data;
    (void)context;

    struct aws_mqtt_packet_ack unsuback;
    if (aws_mqtt_packet_unsuback_init(&unsuback, 0 /* fake packet_id */)) {
        return AWS_OP_ERR;
    }

    if (aws_mqtt_packet_ack_decode(&message_cursor, &unsuback)) {
        return AWS_OP_ERR;
    }

    struct aws_mqtt_packet_ack *expected_unsuback = context->expected_packet;

    ASSERT_INT_EQUALS(expected_unsuback->packet_identifier, unsuback.packet_identifier);

    ++context->packet_count[AWS_MQTT_PACKET_UNSUBACK];

    return AWS_OP_SUCCESS;
}

static int s_decoding_test_handle_puback(struct aws_byte_cursor message_cursor, void *user_data) {

    struct mqtt_311_decoding_test_context *context = user_data;
    (void)context;

    struct aws_mqtt_packet_ack puback;
    if (aws_mqtt_packet_puback_init(&puback, 0 /* fake packet_id */)) {
        return AWS_OP_ERR;
    }

    if (aws_mqtt_packet_ack_decode(&message_cursor, &puback)) {
        return AWS_OP_ERR;
    }

    struct aws_mqtt_packet_ack *expected_puback = context->expected_packet;

    ASSERT_INT_EQUALS(expected_puback->packet_identifier, puback.packet_identifier);

    ++context->packet_count[AWS_MQTT_PACKET_PUBACK];

    return AWS_OP_SUCCESS;
}

static int s_decoding_test_handle_pingresp(struct aws_byte_cursor message_cursor, void *user_data) {

    struct mqtt_311_decoding_test_context *context = user_data;
    (void)context;

    struct aws_mqtt_packet_connection pingresp;
    if (aws_mqtt_packet_pingresp_init(&pingresp)) {
        return AWS_OP_ERR;
    }

    if (aws_mqtt_packet_connection_decode(&message_cursor, &pingresp)) {
        return AWS_OP_ERR;
    }

    ++context->packet_count[AWS_MQTT_PACKET_PINGRESP];

    return AWS_OP_SUCCESS;
}

static int s_decoding_test_handle_connack(struct aws_byte_cursor message_cursor, void *user_data) {

    struct mqtt_311_decoding_test_context *context = user_data;
    (void)context;

    struct aws_mqtt_packet_connack connack;
    if (aws_mqtt_packet_connack_init(&connack, false, 0)) {
        return AWS_OP_ERR;
    }

    if (aws_mqtt_packet_connack_decode(&message_cursor, &connack)) {
        return AWS_OP_ERR;
    }

    struct aws_mqtt_packet_connack *expected_connack = context->expected_packet;

    ASSERT_INT_EQUALS(expected_connack->session_present, connack.session_present);
    ASSERT_INT_EQUALS(expected_connack->connect_return_code, connack.connect_return_code);

    ++context->packet_count[AWS_MQTT_PACKET_CONNACK];

    return AWS_OP_SUCCESS;
}

static struct aws_mqtt_client_connection_packet_handlers s_decoding_test_packet_handlers = {
    .handlers_by_packet_type = {
        [AWS_MQTT_PACKET_PUBLISH] = &s_decoding_test_handle_publish,
        [AWS_MQTT_PACKET_SUBACK] = &s_decoding_test_handle_suback,
        [AWS_MQTT_PACKET_UNSUBACK] = &s_decoding_test_handle_unsuback,
        [AWS_MQTT_PACKET_PUBACK] = &s_decoding_test_handle_puback,
        [AWS_MQTT_PACKET_PINGRESP] = &s_decoding_test_handle_pingresp,
        [AWS_MQTT_PACKET_CONNACK] = &s_decoding_test_handle_connack,
    }};

static void s_init_decoding_test_context(
    struct mqtt_311_decoding_test_context *context,
    struct aws_allocator *allocator) {
    AWS_ZERO_STRUCT(*context);

    context->allocator = allocator;

    struct aws_mqtt311_decoder_options config = {
        .packet_handlers = &s_decoding_test_packet_handlers,
        .handler_user_data = context,
    };

    aws_mqtt311_decoder_init(&context->decoder, allocator, &config);
}

static void s_clean_up_decoding_test_context(struct mqtt_311_decoding_test_context *context) {

    aws_mqtt311_decoder_clean_up(&context->decoder);
}

#define TEST_ADJACENT_PACKET_COUNT 4

static int s_mqtt_frame_and_decode_publish_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    /*
     * For completeness, run the test with payload sizes that lead to a remaining length VLI encoding of 1, 2, 3, and
     * 4 bytes.
     */
    size_t payload_sizes[] = {35, 1234, 1 << 16, 1 << 21};

    for (size_t i = 0; i < AWS_ARRAY_SIZE(payload_sizes); ++i) {
        struct mqtt_311_decoding_test_context test_context;
        s_init_decoding_test_context(&test_context, allocator);

        struct aws_mqtt311_decoder *decoder = &test_context.decoder;

        size_t publish_payload_size = payload_sizes[i];

        /* Intentionally don't initialize so we have lots of garbage */
        uint8_t *raw_payload = aws_mem_acquire(allocator, publish_payload_size);

        struct aws_mqtt_packet_publish publish_packet;
        ASSERT_SUCCESS(aws_mqtt_packet_publish_init(
            &publish_packet,
            true,
            AWS_MQTT_QOS_AT_LEAST_ONCE,
            false,
            aws_byte_cursor_from_c_str("Hello/World"),
            12,
            aws_byte_cursor_from_array(raw_payload, publish_payload_size)));

        test_context.expected_packet = &publish_packet;

        struct aws_byte_buf encoded_buffer;
        aws_byte_buf_init(&encoded_buffer, allocator, (publish_payload_size + 100) * TEST_ADJACENT_PACKET_COUNT);

        for (size_t j = 0; j < TEST_ADJACENT_PACKET_COUNT; ++j) {
            ASSERT_SUCCESS(aws_mqtt_packet_publish_encode(&encoded_buffer, &publish_packet));
        }

        size_t fragment_lengths[] = {1, 2, 3, 5, 7, 11, 23, 37, 67, 131};

        for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
            size_t fragment_length = fragment_lengths[j];

            struct aws_byte_cursor packet_cursor = aws_byte_cursor_from_buf(&encoded_buffer);
            while (packet_cursor.len > 0) {
                size_t advance = aws_min_size(packet_cursor.len, fragment_length);
                struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

                ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
            }
        }

        ASSERT_INT_EQUALS(4 * AWS_ARRAY_SIZE(fragment_lengths), test_context.packet_count[AWS_MQTT_PACKET_PUBLISH]);

        aws_byte_buf_clean_up(&encoded_buffer);
        aws_mem_release(allocator, raw_payload);

        s_clean_up_decoding_test_context(&test_context);
    }

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_publish, s_mqtt_frame_and_decode_publish_fn)

static int s_mqtt_frame_and_decode_suback_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    struct aws_mqtt_packet_suback suback_packet;
    ASSERT_SUCCESS(aws_mqtt_packet_suback_init(&suback_packet, allocator, 1234));

    uint8_t sample_return_codes[] = {0x00, 0x01, 0x02, 0x80, 0x01};
    for (size_t i = 0; i < AWS_ARRAY_SIZE(sample_return_codes); ++i) {
        aws_mqtt_packet_suback_add_return_code(&suback_packet, sample_return_codes[i]);
    }

    test_context.expected_packet = &suback_packet;

    struct aws_byte_buf encoded_buffer;
    aws_byte_buf_init(&encoded_buffer, allocator, 100 * TEST_ADJACENT_PACKET_COUNT);

    for (size_t j = 0; j < TEST_ADJACENT_PACKET_COUNT; ++j) {
        ASSERT_SUCCESS(aws_mqtt_packet_suback_encode(&encoded_buffer, &suback_packet));
    }

    size_t fragment_lengths[] = {1, 2, 3, 5, 7, 11, 23, 37, 67, 143};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        size_t fragment_length = fragment_lengths[j];

        struct aws_byte_cursor packet_cursor = aws_byte_cursor_from_buf(&encoded_buffer);
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
        }
    }

    ASSERT_INT_EQUALS(4 * AWS_ARRAY_SIZE(fragment_lengths), test_context.packet_count[AWS_MQTT_PACKET_SUBACK]);

    aws_mqtt_packet_suback_clean_up(&suback_packet);
    aws_byte_buf_clean_up(&encoded_buffer);

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_suback, s_mqtt_frame_and_decode_suback_fn)

static int s_mqtt_frame_and_decode_unsuback_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    struct aws_mqtt_packet_ack unsuback_packet;
    ASSERT_SUCCESS(aws_mqtt_packet_unsuback_init(&unsuback_packet, 1234));

    test_context.expected_packet = &unsuback_packet;

    struct aws_byte_buf encoded_buffer;
    aws_byte_buf_init(&encoded_buffer, allocator, 100 * TEST_ADJACENT_PACKET_COUNT);

    for (size_t j = 0; j < TEST_ADJACENT_PACKET_COUNT; ++j) {
        ASSERT_SUCCESS(aws_mqtt_packet_ack_encode(&encoded_buffer, &unsuback_packet));
    }

    size_t fragment_lengths[] = {1, 2, 3, 5, 7, 11, 23};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        size_t fragment_length = fragment_lengths[j];

        struct aws_byte_cursor packet_cursor = aws_byte_cursor_from_buf(&encoded_buffer);
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
        }
    }

    ASSERT_INT_EQUALS(4 * AWS_ARRAY_SIZE(fragment_lengths), test_context.packet_count[AWS_MQTT_PACKET_UNSUBACK]);

    aws_byte_buf_clean_up(&encoded_buffer);

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_unsuback, s_mqtt_frame_and_decode_unsuback_fn)

static int s_mqtt_frame_and_decode_puback_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    struct aws_mqtt_packet_ack puback_packet;
    ASSERT_SUCCESS(aws_mqtt_packet_puback_init(&puback_packet, 1234));

    test_context.expected_packet = &puback_packet;

    struct aws_byte_buf encoded_buffer;
    aws_byte_buf_init(&encoded_buffer, allocator, 100 * TEST_ADJACENT_PACKET_COUNT);

    for (size_t j = 0; j < TEST_ADJACENT_PACKET_COUNT; ++j) {
        ASSERT_SUCCESS(aws_mqtt_packet_ack_encode(&encoded_buffer, &puback_packet));
    }

    size_t fragment_lengths[] = {1, 2, 3, 5, 7, 11, 23};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        size_t fragment_length = fragment_lengths[j];

        struct aws_byte_cursor packet_cursor = aws_byte_cursor_from_buf(&encoded_buffer);
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
        }
    }

    ASSERT_INT_EQUALS(4 * AWS_ARRAY_SIZE(fragment_lengths), test_context.packet_count[AWS_MQTT_PACKET_PUBACK]);

    aws_byte_buf_clean_up(&encoded_buffer);

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_puback, s_mqtt_frame_and_decode_puback_fn)

static int s_mqtt_frame_and_decode_pingresp_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    struct aws_mqtt_packet_connection pingresp_packet;
    ASSERT_SUCCESS(aws_mqtt_packet_pingresp_init(&pingresp_packet));

    test_context.expected_packet = &pingresp_packet;

    struct aws_byte_buf encoded_buffer;
    aws_byte_buf_init(&encoded_buffer, allocator, 100 * TEST_ADJACENT_PACKET_COUNT);

    for (size_t j = 0; j < TEST_ADJACENT_PACKET_COUNT; ++j) {
        ASSERT_SUCCESS(aws_mqtt_packet_connection_encode(&encoded_buffer, &pingresp_packet));
    }

    size_t fragment_lengths[] = {1, 2, 3, 5, 7, 11};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        size_t fragment_length = fragment_lengths[j];

        struct aws_byte_cursor packet_cursor = aws_byte_cursor_from_buf(&encoded_buffer);
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
        }
    }

    ASSERT_INT_EQUALS(4 * AWS_ARRAY_SIZE(fragment_lengths), test_context.packet_count[AWS_MQTT_PACKET_PINGRESP]);

    aws_byte_buf_clean_up(&encoded_buffer);

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_pingresp, s_mqtt_frame_and_decode_pingresp_fn)

static int s_mqtt_frame_and_decode_connack_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    struct aws_mqtt_packet_connack connack_packet;
    ASSERT_SUCCESS(aws_mqtt_packet_connack_init(&connack_packet, true, AWS_MQTT_CONNECT_NOT_AUTHORIZED));

    test_context.expected_packet = &connack_packet;

    struct aws_byte_buf encoded_buffer;
    aws_byte_buf_init(&encoded_buffer, allocator, 100 * TEST_ADJACENT_PACKET_COUNT);

    for (size_t j = 0; j < TEST_ADJACENT_PACKET_COUNT; ++j) {
        ASSERT_SUCCESS(aws_mqtt_packet_connack_encode(&encoded_buffer, &connack_packet));
    }

    size_t fragment_lengths[] = {1, 2, 3, 5, 7, 11, 23};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        size_t fragment_length = fragment_lengths[j];

        struct aws_byte_cursor packet_cursor = aws_byte_cursor_from_buf(&encoded_buffer);
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
        }
    }

    ASSERT_INT_EQUALS(4 * AWS_ARRAY_SIZE(fragment_lengths), test_context.packet_count[AWS_MQTT_PACKET_CONNACK]);

    aws_byte_buf_clean_up(&encoded_buffer);

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_connack, s_mqtt_frame_and_decode_connack_fn)

static int s_mqtt_frame_and_decode_bad_remaining_length_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    /* QoS 0 Publish "Packet" data where the remaining length vli-encoding is illegal */
    uint8_t bad_packet_data[] = {0x30, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};

    size_t fragment_lengths[] = {1, 2, 3, 5, 13};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        aws_mqtt311_decoder_reset_for_new_connection(decoder);

        size_t fragment_length = fragment_lengths[j];
        struct aws_byte_cursor packet_cursor =
            aws_byte_cursor_from_array(bad_packet_data, AWS_ARRAY_SIZE(bad_packet_data));
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            /* If this or a previous call contains the final 0x80 of the invalid vli encoding, then decode must fail */
            bool should_fail = (fragment_cursor.ptr + fragment_cursor.len) - bad_packet_data > 4;
            if (should_fail) {
                ASSERT_FAILS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
            } else {
                ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
            }
        }
    }

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_bad_remaining_length, s_mqtt_frame_and_decode_bad_remaining_length_fn)

static int s_mqtt_frame_and_decode_unsupported_packet_type_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    /* Pingreq packet, no handler installed */
    uint8_t pingreq_packet_data[] = {192, 0};

    size_t fragment_lengths[] = {1, 2};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        aws_mqtt311_decoder_reset_for_new_connection(decoder);

        size_t fragment_length = fragment_lengths[j];
        struct aws_byte_cursor packet_cursor =
            aws_byte_cursor_from_array(pingreq_packet_data, AWS_ARRAY_SIZE(pingreq_packet_data));
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            /* If this is the final call, it should fail-but-not-crash as there's no handler */
            bool should_fail = packet_cursor.len == 0;
            if (should_fail) {
                ASSERT_FAILS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
            } else {
                ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
            }
        }
    }

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_unsupported_packet_type, s_mqtt_frame_and_decode_unsupported_packet_type_fn)

static int s_mqtt_frame_and_decode_bad_flags_for_packet_type_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct mqtt_311_decoding_test_context test_context;
    s_init_decoding_test_context(&test_context, allocator);

    struct aws_mqtt311_decoder *decoder = &test_context.decoder;

    /* start with a valid suback */
    struct aws_mqtt_packet_suback suback_packet;
    ASSERT_SUCCESS(aws_mqtt_packet_suback_init(&suback_packet, allocator, 1234));

    uint8_t sample_return_codes[] = {0x00, 0x01, 0x02, 0x80, 0x01};
    for (size_t i = 0; i < AWS_ARRAY_SIZE(sample_return_codes); ++i) {
        aws_mqtt_packet_suback_add_return_code(&suback_packet, sample_return_codes[i]);
    }

    /* encode it */
    struct aws_byte_buf encoded_buffer;
    aws_byte_buf_init(&encoded_buffer, allocator, 100 * TEST_ADJACENT_PACKET_COUNT);

    ASSERT_SUCCESS(aws_mqtt_packet_suback_encode(&encoded_buffer, &suback_packet));

    /* suback flags should be zero; mess that up */
    encoded_buffer.buffer[0] |= 0x05;

    size_t fragment_lengths[] = {1, 2, 3, 5, 7, 11, 23};

    for (size_t j = 0; j < AWS_ARRAY_SIZE(fragment_lengths); ++j) {
        size_t fragment_length = fragment_lengths[j];

        aws_mqtt311_decoder_reset_for_new_connection(decoder);

        struct aws_byte_cursor packet_cursor = aws_byte_cursor_from_buf(&encoded_buffer);
        while (packet_cursor.len > 0) {
            size_t advance = aws_min_size(packet_cursor.len, fragment_length);
            struct aws_byte_cursor fragment_cursor = aws_byte_cursor_advance(&packet_cursor, advance);

            /* If this is the final call, it should fail-but-not-crash as the full decode should fail */
            bool should_fail = packet_cursor.len == 0;
            if (should_fail) {
                ASSERT_FAILS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
            } else {
                ASSERT_SUCCESS(aws_mqtt311_decoder_on_bytes_received(decoder, fragment_cursor));
            }
        }
    }

    aws_mqtt_packet_suback_clean_up(&suback_packet);
    aws_byte_buf_clean_up(&encoded_buffer);

    s_clean_up_decoding_test_context(&test_context);

    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_frame_and_decode_bad_flags_for_packet_type, s_mqtt_frame_and_decode_bad_flags_for_packet_type_fn)
