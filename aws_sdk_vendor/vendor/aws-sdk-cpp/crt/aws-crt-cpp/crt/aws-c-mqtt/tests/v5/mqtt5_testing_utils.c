/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt5_testing_utils.h"

#include <aws/common/clock.h>
#include <aws/io/channel_bootstrap.h>
#include <aws/io/event_loop.h>
#include <aws/io/host_resolver.h>
#include <aws/mqtt/private/v5/mqtt5_decoder.h>
#include <aws/mqtt/private/v5/mqtt5_encoder.h>
#include <aws/mqtt/private/v5/mqtt5_utils.h>
#include <aws/mqtt/v5/mqtt5_client.h>

#include <aws/testing/aws_test_harness.h>

#include <inttypes.h>

int aws_mqtt5_test_verify_user_properties_raw(
    size_t property_count,
    const struct aws_mqtt5_user_property *properties,
    size_t expected_count,
    const struct aws_mqtt5_user_property *expected_properties) {

    ASSERT_UINT_EQUALS(expected_count, property_count);

    for (size_t i = 0; i < expected_count; ++i) {
        const struct aws_mqtt5_user_property *expected_property = &expected_properties[i];
        struct aws_byte_cursor expected_name = expected_property->name;
        struct aws_byte_cursor expected_value = expected_property->value;

        bool found = false;
        for (size_t j = 0; j < property_count; ++j) {
            const struct aws_mqtt5_user_property *nv_pair = &properties[j];

            if (aws_byte_cursor_compare_lexical(&expected_name, &nv_pair->name) == 0 &&
                aws_byte_cursor_compare_lexical(&expected_value, &nv_pair->value) == 0) {
                found = true;
                break;
            }
        }

        if (!found) {
            return AWS_OP_ERR;
        }
    }

    return AWS_OP_SUCCESS;
}

static int s_compute_connack_variable_length_fields(
    const struct aws_mqtt5_packet_connack_view *connack_view,
    uint32_t *total_remaining_length,
    uint32_t *property_length) {

    size_t local_property_length =
        aws_mqtt5_compute_user_property_encode_length(connack_view->user_properties, connack_view->user_property_count);

    ADD_OPTIONAL_U32_PROPERTY_LENGTH(connack_view->session_expiry_interval, local_property_length);
    ADD_OPTIONAL_U16_PROPERTY_LENGTH(connack_view->receive_maximum, local_property_length);
    ADD_OPTIONAL_U8_PROPERTY_LENGTH(connack_view->maximum_qos, local_property_length);
    ADD_OPTIONAL_U8_PROPERTY_LENGTH(connack_view->retain_available, local_property_length);
    ADD_OPTIONAL_U32_PROPERTY_LENGTH(connack_view->maximum_packet_size, local_property_length);
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(connack_view->assigned_client_identifier, local_property_length);
    ADD_OPTIONAL_U16_PROPERTY_LENGTH(connack_view->topic_alias_maximum, local_property_length);
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(connack_view->reason_string, local_property_length);
    ADD_OPTIONAL_U8_PROPERTY_LENGTH(connack_view->wildcard_subscriptions_available, local_property_length);
    ADD_OPTIONAL_U8_PROPERTY_LENGTH(connack_view->subscription_identifiers_available, local_property_length);
    ADD_OPTIONAL_U8_PROPERTY_LENGTH(connack_view->shared_subscriptions_available, local_property_length);
    ADD_OPTIONAL_U16_PROPERTY_LENGTH(connack_view->server_keep_alive, local_property_length);
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(connack_view->response_information, local_property_length);
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(connack_view->server_reference, local_property_length);
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(connack_view->authentication_method, local_property_length);
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(connack_view->authentication_data, local_property_length);

    *property_length = (uint32_t)local_property_length;

    size_t property_length_encoding_length = 0;
    if (aws_mqtt5_get_variable_length_encode_size(local_property_length, &property_length_encoding_length)) {
        return AWS_OP_ERR;
    }

    /* reason code (1 byte) + flags (1 byte) */
    *total_remaining_length = *property_length + (uint32_t)property_length_encoding_length + 2;

    return AWS_OP_SUCCESS;
}

int aws_mqtt5_encoder_begin_connack(struct aws_mqtt5_encoder *encoder, const void *packet_view) {

    const struct aws_mqtt5_packet_connack_view *connack_view = packet_view;

    uint32_t total_remaining_length = 0;
    uint32_t property_length = 0;
    if (s_compute_connack_variable_length_fields(connack_view, &total_remaining_length, &property_length)) {
        int error_code = aws_last_error();
        AWS_LOGF_ERROR(
            AWS_LS_MQTT5_GENERAL,
            "(%p) mqtt5 client encoder - failed to compute variable length values for CONNACK packet with error "
            "%d(%s)",
            (void *)encoder->config.client,
            error_code,
            aws_error_debug_str(error_code));
        return AWS_OP_ERR;
    }

    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL,
        "(%p) mqtt5 client encoder - setting up encode for a CONNACK packet with remaining length %" PRIu32,
        (void *)encoder->config.client,
        total_remaining_length);

    uint8_t flags = connack_view->session_present ? 1 : 0;

    ADD_ENCODE_STEP_U8(encoder, aws_mqtt5_compute_fixed_header_byte1(AWS_MQTT5_PT_CONNACK, 0));
    ADD_ENCODE_STEP_VLI(encoder, total_remaining_length);
    ADD_ENCODE_STEP_U8(encoder, flags);
    ADD_ENCODE_STEP_U8(encoder, (uint8_t)connack_view->reason_code);
    ADD_ENCODE_STEP_VLI(encoder, property_length);

    if (property_length > 0) {
        ADD_ENCODE_STEP_OPTIONAL_U32_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_SESSION_EXPIRY_INTERVAL, connack_view->session_expiry_interval);
        ADD_ENCODE_STEP_OPTIONAL_U16_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_RECEIVE_MAXIMUM, connack_view->receive_maximum);
        ADD_ENCODE_STEP_OPTIONAL_U8_PROPERTY(encoder, AWS_MQTT5_PROPERTY_TYPE_MAXIMUM_QOS, connack_view->maximum_qos);
        ADD_ENCODE_STEP_OPTIONAL_U8_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_RETAIN_AVAILABLE, connack_view->retain_available);
        ADD_ENCODE_STEP_OPTIONAL_U32_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_MAXIMUM_PACKET_SIZE, connack_view->maximum_packet_size);
        ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_ASSIGNED_CLIENT_IDENTIFIER, connack_view->assigned_client_identifier);
        ADD_ENCODE_STEP_OPTIONAL_U16_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_TOPIC_ALIAS_MAXIMUM, connack_view->topic_alias_maximum);
        ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_REASON_STRING, connack_view->reason_string);
        ADD_ENCODE_STEP_OPTIONAL_U8_PROPERTY(
            encoder,
            AWS_MQTT5_PROPERTY_TYPE_WILDCARD_SUBSCRIPTIONS_AVAILABLE,
            connack_view->wildcard_subscriptions_available);
        ADD_ENCODE_STEP_OPTIONAL_U8_PROPERTY(
            encoder,
            AWS_MQTT5_PROPERTY_TYPE_SUBSCRIPTION_IDENTIFIERS_AVAILABLE,
            connack_view->subscription_identifiers_available);
        ADD_ENCODE_STEP_OPTIONAL_U8_PROPERTY(
            encoder,
            AWS_MQTT5_PROPERTY_TYPE_SHARED_SUBSCRIPTIONS_AVAILABLE,
            connack_view->shared_subscriptions_available);
        ADD_ENCODE_STEP_OPTIONAL_U16_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_SERVER_KEEP_ALIVE, connack_view->server_keep_alive);
        ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_RESPONSE_INFORMATION, connack_view->response_information);
        ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_SERVER_REFERENCE, connack_view->server_reference);
        ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_AUTHENTICATION_METHOD, connack_view->authentication_method);
        ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
            encoder, AWS_MQTT5_PROPERTY_TYPE_AUTHENTICATION_DATA, connack_view->authentication_data);

        aws_mqtt5_add_user_property_encoding_steps(
            encoder, connack_view->user_properties, connack_view->user_property_count);
    }

    return AWS_OP_SUCCESS;
}

int aws_mqtt5_encoder_begin_pingresp(struct aws_mqtt5_encoder *encoder, const void *packet_view) {
    (void)packet_view;

    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL,
        "(%p) mqtt5 client encoder - setting up encode for a PINGRESP packet",
        (void *)encoder->config.client);

    /* A ping response is just a fixed header with a 0-valued remaining length which we encode as a 0 u8 */
    ADD_ENCODE_STEP_U8(encoder, aws_mqtt5_compute_fixed_header_byte1(AWS_MQTT5_PT_PINGRESP, 0));
    ADD_ENCODE_STEP_U8(encoder, 0);

    return AWS_OP_SUCCESS;
}

static int s_compute_suback_variable_length_fields(
    const struct aws_mqtt5_packet_suback_view *suback_view,
    uint32_t *total_remaining_length,
    uint32_t *property_length) {
    /* User Properties length */
    size_t local_property_length =
        aws_mqtt5_compute_user_property_encode_length(suback_view->user_properties, suback_view->user_property_count);
    /* Optional Reason String */
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(suback_view->reason_string, local_property_length);

    *property_length = (uint32_t)local_property_length;

    size_t local_total_remaining_length = 0;
    if (aws_mqtt5_get_variable_length_encode_size(local_property_length, &local_total_remaining_length)) {
        return AWS_OP_ERR;
    }

    /* Packet Identifier (2 bytes) */
    local_total_remaining_length += 2;

    /* Reason Codes (1 byte each) */
    local_total_remaining_length += suback_view->reason_code_count;

    /* Add property length */
    *total_remaining_length = *property_length + (uint32_t)local_total_remaining_length;

    return AWS_OP_SUCCESS;
}

int aws_mqtt5_encoder_begin_suback(struct aws_mqtt5_encoder *encoder, const void *packet_view) {

    const struct aws_mqtt5_packet_suback_view *suback_view = packet_view;

    uint32_t total_remaining_length = 0;
    uint32_t property_length = 0;
    if (s_compute_suback_variable_length_fields(suback_view, &total_remaining_length, &property_length)) {
        int error_code = aws_last_error();
        AWS_LOGF_ERROR(
            AWS_LS_MQTT5_GENERAL,
            "(%p) mqtt5 client encoder - failed to compute variable length values for SUBACK packet with error "
            "%d(%s)",
            (void *)encoder->config.client,
            error_code,
            aws_error_debug_str(error_code));
        return AWS_OP_ERR;
    }

    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL,
        "(%p) mqtt5 client encoder - setting up encode for a SUBACK packet with remaining length %" PRIu32,
        (void *)encoder->config.client,
        total_remaining_length);

    ADD_ENCODE_STEP_U8(encoder, aws_mqtt5_compute_fixed_header_byte1(AWS_MQTT5_PT_SUBACK, 0));
    ADD_ENCODE_STEP_VLI(encoder, total_remaining_length);
    ADD_ENCODE_STEP_U16(encoder, suback_view->packet_id);
    ADD_ENCODE_STEP_VLI(encoder, property_length);

    ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
        encoder, AWS_MQTT5_PROPERTY_TYPE_REASON_STRING, suback_view->reason_string);

    aws_mqtt5_add_user_property_encoding_steps(encoder, suback_view->user_properties, suback_view->user_property_count);

    for (size_t i = 0; i < suback_view->reason_code_count; ++i) {
        ADD_ENCODE_STEP_U8(encoder, suback_view->reason_codes[i]);
    }

    return AWS_OP_SUCCESS;
}

static int s_compute_unsuback_variable_length_fields(
    const struct aws_mqtt5_packet_unsuback_view *unsuback_view,
    uint32_t *total_remaining_length,
    uint32_t *property_length) {
    /* User Properties length */
    size_t local_property_length = aws_mqtt5_compute_user_property_encode_length(
        unsuback_view->user_properties, unsuback_view->user_property_count);
    /* Optional Reason String */
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(unsuback_view->reason_string, local_property_length);

    *property_length = (uint32_t)local_property_length;

    size_t local_total_remaining_length = 0;
    if (aws_mqtt5_get_variable_length_encode_size(local_property_length, &local_total_remaining_length)) {
        return AWS_OP_ERR;
    }

    /* Packet Identifier (2 bytes) */
    local_total_remaining_length += 2;

    /* Reason Codes (1 byte each) */
    local_total_remaining_length += unsuback_view->reason_code_count;

    /* Add property length */
    *total_remaining_length = *property_length + (uint32_t)local_total_remaining_length;

    return AWS_OP_SUCCESS;
}

int aws_mqtt5_encoder_begin_unsuback(struct aws_mqtt5_encoder *encoder, const void *packet_view) {

    const struct aws_mqtt5_packet_unsuback_view *unsuback_view = packet_view;

    uint32_t total_remaining_length = 0;
    uint32_t property_length = 0;
    if (s_compute_unsuback_variable_length_fields(unsuback_view, &total_remaining_length, &property_length)) {
        int error_code = aws_last_error();
        AWS_LOGF_ERROR(
            AWS_LS_MQTT5_GENERAL,
            "(%p) mqtt5 client encoder - failed to compute variable length values for UNSUBACK packet with error "
            "%d(%s)",
            (void *)encoder->config.client,
            error_code,
            aws_error_debug_str(error_code));
        return AWS_OP_ERR;
    }

    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL,
        "(%p) mqtt5 client encoder - setting up encode for an UNSUBACK packet with remaining length %" PRIu32,
        (void *)encoder->config.client,
        total_remaining_length);

    ADD_ENCODE_STEP_U8(encoder, aws_mqtt5_compute_fixed_header_byte1(AWS_MQTT5_PT_UNSUBACK, 0));
    ADD_ENCODE_STEP_VLI(encoder, total_remaining_length);
    ADD_ENCODE_STEP_U16(encoder, unsuback_view->packet_id);
    ADD_ENCODE_STEP_VLI(encoder, property_length);

    ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
        encoder, AWS_MQTT5_PROPERTY_TYPE_REASON_STRING, unsuback_view->reason_string);

    aws_mqtt5_add_user_property_encoding_steps(
        encoder, unsuback_view->user_properties, unsuback_view->user_property_count);

    for (size_t i = 0; i < unsuback_view->reason_code_count; ++i) {
        ADD_ENCODE_STEP_U8(encoder, unsuback_view->reason_codes[i]);
    }

    return AWS_OP_SUCCESS;
}

static int s_compute_puback_variable_length_fields(
    const struct aws_mqtt5_packet_puback_view *puback_view,
    uint32_t *total_remaining_length,
    uint32_t *property_length) {
    /* User Properties length */
    size_t local_property_length =
        aws_mqtt5_compute_user_property_encode_length(puback_view->user_properties, puback_view->user_property_count);
    /* Optional Reason String */
    ADD_OPTIONAL_CURSOR_PROPERTY_LENGTH(puback_view->reason_string, local_property_length);

    *property_length = (uint32_t)local_property_length;

    size_t local_total_remaining_length = 0;
    if (aws_mqtt5_get_variable_length_encode_size(local_property_length, &local_total_remaining_length)) {
        return AWS_OP_ERR;
    }

    /* Packet Identifier (2 bytes) */
    local_total_remaining_length += 2;

    /* Reason Code */
    local_total_remaining_length += 1;

    /* Add property length */
    *total_remaining_length = *property_length + (uint32_t)local_total_remaining_length;

    return AWS_OP_SUCCESS;
}

int aws_mqtt5_encoder_begin_puback(struct aws_mqtt5_encoder *encoder, const void *packet_view) {

    const struct aws_mqtt5_packet_puback_view *puback_view = packet_view;

    uint32_t total_remaining_length = 0;
    uint32_t property_length = 0;
    if (s_compute_puback_variable_length_fields(puback_view, &total_remaining_length, &property_length)) {
        int error_code = aws_last_error();
        AWS_LOGF_ERROR(
            AWS_LS_MQTT5_GENERAL,
            "(%p) mqtt5 client encoder - failed to compute variable length values for PUBACK packet with error "
            "%d(%s)",
            (void *)encoder->config.client,
            error_code,
            aws_error_debug_str(error_code));
        return AWS_OP_ERR;
    }

    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL,
        "(%p) mqtt5 client encoder - setting up encode for an PUBACK packet with remaining length %" PRIu32,
        (void *)encoder->config.client,
        total_remaining_length);

    ADD_ENCODE_STEP_U8(encoder, aws_mqtt5_compute_fixed_header_byte1(AWS_MQTT5_PT_PUBACK, 0));
    ADD_ENCODE_STEP_VLI(encoder, total_remaining_length);
    ADD_ENCODE_STEP_U16(encoder, puback_view->packet_id);
    ADD_ENCODE_STEP_U8(encoder, puback_view->reason_code);
    ADD_ENCODE_STEP_VLI(encoder, property_length);
    ADD_ENCODE_STEP_OPTIONAL_CURSOR_PROPERTY(
        encoder, AWS_MQTT5_PROPERTY_TYPE_REASON_STRING, puback_view->reason_string);
    aws_mqtt5_add_user_property_encoding_steps(encoder, puback_view->user_properties, puback_view->user_property_count);

    return AWS_OP_SUCCESS;
}

void aws_mqtt5_encode_init_testing_function_table(struct aws_mqtt5_encoder_function_table *function_table) {
    *function_table = *g_aws_mqtt5_encoder_default_function_table;
    function_table->encoders_by_packet_type[AWS_MQTT5_PT_PINGRESP] = &aws_mqtt5_encoder_begin_pingresp;
    function_table->encoders_by_packet_type[AWS_MQTT5_PT_CONNACK] = &aws_mqtt5_encoder_begin_connack;
    function_table->encoders_by_packet_type[AWS_MQTT5_PT_SUBACK] = &aws_mqtt5_encoder_begin_suback;
    function_table->encoders_by_packet_type[AWS_MQTT5_PT_UNSUBACK] = &aws_mqtt5_encoder_begin_unsuback;
    function_table->encoders_by_packet_type[AWS_MQTT5_PT_PUBACK] = &aws_mqtt5_encoder_begin_puback;
}

static int s_aws_mqtt5_decoder_decode_pingreq(struct aws_mqtt5_decoder *decoder) {
    if (decoder->packet_cursor.len != 0) {
        goto error;
    }

    uint8_t expected_first_byte = aws_mqtt5_compute_fixed_header_byte1(AWS_MQTT5_PT_PINGREQ, 0);
    if (decoder->packet_first_byte != expected_first_byte || decoder->remaining_length != 0) {
        goto error;
    }

    if (decoder->options.on_packet_received != NULL) {
        (*decoder->options.on_packet_received)(AWS_MQTT5_PT_PINGREQ, NULL, decoder->options.callback_user_data);
    }

    return AWS_OP_SUCCESS;

error:

    AWS_LOGF_ERROR(
        AWS_LS_MQTT5_GENERAL, "(%p) aws_mqtt5_decoder - PINGREQ decode failure", decoder->options.callback_user_data);
    return aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
}

/* decode function for all CONNECT properties.  Movable to test-only code if we switched to a decoding function table */
static int s_read_connect_property(
    struct aws_mqtt5_packet_connect_storage *storage,
    struct aws_byte_cursor *packet_cursor) {
    int result = AWS_OP_ERR;

    uint8_t property_type = 0;
    AWS_MQTT5_DECODE_U8(packet_cursor, &property_type, done);

    struct aws_mqtt5_packet_connect_view *storage_view = &storage->storage_view;

    switch (property_type) {
        case AWS_MQTT5_PROPERTY_TYPE_SESSION_EXPIRY_INTERVAL:
            AWS_MQTT5_DECODE_U32_OPTIONAL(
                packet_cursor,
                &storage->session_expiry_interval_seconds,
                &storage_view->session_expiry_interval_seconds,
                done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_RECEIVE_MAXIMUM:
            AWS_MQTT5_DECODE_U16_OPTIONAL(
                packet_cursor, &storage->receive_maximum, &storage_view->receive_maximum, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_MAXIMUM_PACKET_SIZE:
            AWS_MQTT5_DECODE_U32_OPTIONAL(
                packet_cursor, &storage->maximum_packet_size_bytes, &storage_view->maximum_packet_size_bytes, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_TOPIC_ALIAS_MAXIMUM:
            AWS_MQTT5_DECODE_U16_OPTIONAL(
                packet_cursor, &storage->topic_alias_maximum, &storage_view->topic_alias_maximum, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_REQUEST_RESPONSE_INFORMATION:
            AWS_MQTT5_DECODE_U8_OPTIONAL(
                packet_cursor,
                &storage->request_response_information,
                &storage_view->request_response_information,
                done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_REQUEST_PROBLEM_INFORMATION:
            AWS_MQTT5_DECODE_U8_OPTIONAL(
                packet_cursor, &storage->request_problem_information, &storage_view->request_problem_information, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_AUTHENTICATION_METHOD:
            AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR_OPTIONAL(
                packet_cursor, &storage->authentication_method, &storage_view->authentication_method, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_AUTHENTICATION_DATA:
            AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR_OPTIONAL(
                packet_cursor, &storage->authentication_data, &storage_view->authentication_data, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_USER_PROPERTY:
            if (aws_mqtt5_decode_user_property(packet_cursor, &storage->user_properties)) {
                goto done;
            }
            break;

        default:
            goto done;
    }

    result = AWS_OP_SUCCESS;

done:

    if (result != AWS_OP_SUCCESS) {
        aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
    }

    return result;
}

/* decode function for all will properties.  Movable to test-only code if we switched to a decoding function table */
static int s_read_will_property(
    struct aws_mqtt5_packet_connect_storage *connect_storage,
    struct aws_mqtt5_packet_publish_storage *will_storage,
    struct aws_byte_cursor *packet_cursor) {
    int result = AWS_OP_ERR;

    uint8_t property_type = 0;
    AWS_MQTT5_DECODE_U8(packet_cursor, &property_type, done);

    struct aws_mqtt5_packet_connect_view *connect_storage_view = &connect_storage->storage_view;
    struct aws_mqtt5_packet_publish_view *will_storage_view = &will_storage->storage_view;

    switch (property_type) {
        case AWS_MQTT5_PROPERTY_TYPE_WILL_DELAY_INTERVAL:
            AWS_MQTT5_DECODE_U32_OPTIONAL(
                packet_cursor,
                &connect_storage->will_delay_interval_seconds,
                &connect_storage_view->will_delay_interval_seconds,
                done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_PAYLOAD_FORMAT_INDICATOR:
            AWS_MQTT5_DECODE_U8_OPTIONAL(
                packet_cursor, &will_storage->payload_format, &will_storage_view->payload_format, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_MESSAGE_EXPIRY_INTERVAL:
            AWS_MQTT5_DECODE_U32_OPTIONAL(
                packet_cursor,
                &will_storage->message_expiry_interval_seconds,
                &will_storage_view->message_expiry_interval_seconds,
                done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_CONTENT_TYPE:
            AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR_OPTIONAL(
                packet_cursor, &will_storage->content_type, &will_storage_view->content_type, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_RESPONSE_TOPIC:
            AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR_OPTIONAL(
                packet_cursor, &will_storage->response_topic, &will_storage_view->response_topic, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_CORRELATION_DATA:
            AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR_OPTIONAL(
                packet_cursor, &will_storage->correlation_data, &will_storage_view->correlation_data, done);
            break;

        case AWS_MQTT5_PROPERTY_TYPE_USER_PROPERTY:
            if (aws_mqtt5_decode_user_property(packet_cursor, &will_storage->user_properties)) {
                goto done;
            }
            break;

        default:
            goto done;
    }

    result = AWS_OP_SUCCESS;

done:

    if (result != AWS_OP_SUCCESS) {
        aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
    }

    return result;
}

/*
 * Decodes a CONNECT packet whose data must be in the scratch buffer.
 * Could be moved to test-only if we used a function table for per-packet-type decoding.
 */
static int s_aws_mqtt5_decoder_decode_connect(struct aws_mqtt5_decoder *decoder) {
    struct aws_mqtt5_packet_connect_storage connect_storage;
    struct aws_mqtt5_packet_publish_storage publish_storage;
    int result = AWS_OP_ERR;
    bool has_will = false;

    if (aws_mqtt5_packet_connect_storage_init_from_external_storage(&connect_storage, decoder->allocator)) {
        return AWS_OP_ERR;
    }

    if (aws_mqtt5_packet_publish_storage_init_from_external_storage(&publish_storage, decoder->allocator)) {
        goto done;
    }

    uint8_t first_byte = decoder->packet_first_byte;
    /* CONNECT flags must be zero by protocol */
    if ((first_byte & 0x0F) != 0) {
        goto done;
    }

    struct aws_byte_cursor packet_cursor = decoder->packet_cursor;
    if (decoder->remaining_length != (uint32_t)packet_cursor.len) {
        goto done;
    }

    struct aws_byte_cursor protocol_cursor = aws_byte_cursor_advance(&packet_cursor, 7);
    if (!aws_byte_cursor_eq(&protocol_cursor, &g_aws_mqtt5_connect_protocol_cursor)) {
        goto done;
    }

    uint8_t connect_flags = 0;
    AWS_MQTT5_DECODE_U8(&packet_cursor, &connect_flags, done);

    struct aws_mqtt5_packet_connect_view *connect_storage_view = &connect_storage.storage_view;
    struct aws_mqtt5_packet_publish_view *will_storage_view = &publish_storage.storage_view;

    connect_storage_view->clean_start = (connect_flags & AWS_MQTT5_CONNECT_FLAGS_CLEAN_START_BIT) != 0;

    AWS_MQTT5_DECODE_U16(&packet_cursor, &connect_storage_view->keep_alive_interval_seconds, done);

    uint32_t connect_property_length = 0;
    AWS_MQTT5_DECODE_VLI(&packet_cursor, &connect_property_length, done);
    if (connect_property_length > packet_cursor.len) {
        goto done;
    }

    struct aws_byte_cursor connect_property_cursor = aws_byte_cursor_advance(&packet_cursor, connect_property_length);
    while (connect_property_cursor.len > 0) {
        if (s_read_connect_property(&connect_storage, &connect_property_cursor)) {
            goto done;
        }
    }

    connect_storage_view->user_property_count = aws_mqtt5_user_property_set_size(&connect_storage.user_properties);
    connect_storage_view->user_properties = connect_storage.user_properties.properties.data;

    AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR(&packet_cursor, &connect_storage_view->client_id, done);

    has_will = (connect_flags & AWS_MQTT5_CONNECT_FLAGS_WILL_BIT) != 0;
    if (has_will) {
        uint32_t will_property_length = 0;
        AWS_MQTT5_DECODE_VLI(&packet_cursor, &will_property_length, done);
        if (will_property_length > packet_cursor.len) {
            goto done;
        }

        struct aws_byte_cursor will_property_cursor = aws_byte_cursor_advance(&packet_cursor, will_property_length);
        while (will_property_cursor.len > 0) {
            if (s_read_will_property(&connect_storage, &publish_storage, &will_property_cursor)) {
                goto done;
            }
        }

        AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR(&packet_cursor, &will_storage_view->topic, done);
        AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR(&packet_cursor, &will_storage_view->payload, done);

        /* apply will flags from the connect flags to the will's storage */
        will_storage_view->qos = (enum aws_mqtt5_qos)(
            (connect_flags >> AWS_MQTT5_CONNECT_FLAGS_WILL_QOS_BIT_POSITION) &
            AWS_MQTT5_CONNECT_FLAGS_WILL_QOS_BIT_MASK);
        will_storage_view->retain = (connect_flags & AWS_MQTT5_CONNECT_FLAGS_WILL_RETAIN_BIT) != 0;

        will_storage_view->user_property_count = aws_mqtt5_user_property_set_size(&publish_storage.user_properties);
        will_storage_view->user_properties = publish_storage.user_properties.properties.data;
    }

    if ((connect_flags & AWS_MQTT5_CONNECT_FLAGS_USER_NAME_BIT) != 0) {
        AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR_OPTIONAL(
            &packet_cursor, &connect_storage.username, &connect_storage_view->username, done);
    }

    if ((connect_flags & AWS_MQTT5_CONNECT_FLAGS_PASSWORD_BIT) != 0) {
        AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR_OPTIONAL(
            &packet_cursor, &connect_storage.password, &connect_storage_view->password, done);
    }

    if (packet_cursor.len == 0) {
        result = AWS_OP_SUCCESS;
    }

done:

    if (result == AWS_OP_SUCCESS) {
        if (decoder->options.on_packet_received != NULL) {
            if (has_will) {
                connect_storage.storage_view.will = &publish_storage.storage_view;
            }

            result = (*decoder->options.on_packet_received)(
                AWS_MQTT5_PT_CONNECT, &connect_storage.storage_view, decoder->options.callback_user_data);
        }
    } else {
        AWS_LOGF_ERROR(
            AWS_LS_MQTT5_GENERAL,
            "(%p) aws_mqtt5_decoder - CONNECT decode failure",
            decoder->options.callback_user_data);
        aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
    }

    aws_mqtt5_packet_publish_storage_clean_up(&publish_storage);
    aws_mqtt5_packet_connect_storage_clean_up(&connect_storage);

    return result;
}

/* decode function for subscribe properties.  Movable to test-only code if we switched to a decoding function table */
static int s_read_subscribe_property(
    struct aws_mqtt5_packet_subscribe_storage *subscribe_storage,
    struct aws_byte_cursor *packet_cursor) {
    int result = AWS_OP_ERR;

    uint8_t property_type = 0;
    AWS_MQTT5_DECODE_U8(packet_cursor, &property_type, done);

    struct aws_mqtt5_packet_subscribe_view *storage_view = &subscribe_storage->storage_view;

    switch (property_type) {
        case AWS_MQTT5_PROPERTY_TYPE_SUBSCRIPTION_IDENTIFIER:
            AWS_MQTT5_DECODE_VLI(packet_cursor, &subscribe_storage->subscription_identifier, done);
            storage_view->subscription_identifier = &subscribe_storage->subscription_identifier;

            break;
        case AWS_MQTT5_PROPERTY_TYPE_USER_PROPERTY:
            if (aws_mqtt5_decode_user_property(packet_cursor, &subscribe_storage->user_properties)) {
                goto done;
            }
            break;

        default:
            goto done;
    }

    result = AWS_OP_SUCCESS;

done:

    if (result != AWS_OP_SUCCESS) {
        aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
    }

    return result;
}

/*
 * Decodes a SUBSCRIBE packet whose data must be in the scratch buffer.
 */
static int s_aws_mqtt5_decoder_decode_subscribe(struct aws_mqtt5_decoder *decoder) {
    struct aws_mqtt5_packet_subscribe_storage subscribe_storage;
    int result = AWS_OP_ERR;

    if (aws_mqtt5_packet_subscribe_storage_init_from_external_storage(&subscribe_storage, decoder->allocator)) {
        goto done;
    }

    struct aws_mqtt5_packet_subscribe_view *storage_view = &subscribe_storage.storage_view;

    /* SUBSCRIBE flags must be 2 by protocol*/
    uint8_t first_byte = decoder->packet_first_byte;
    if ((first_byte & 0x0F) != 2) {
        goto done;
    }

    struct aws_byte_cursor packet_cursor = decoder->packet_cursor;
    if (decoder->remaining_length != (uint32_t)packet_cursor.len) {
        goto done;
    }

    uint16_t packet_id = 0;
    AWS_MQTT5_DECODE_U16(&packet_cursor, &packet_id, done);
    subscribe_storage.storage_view.packet_id = packet_id;

    uint32_t property_length = 0;
    AWS_MQTT5_DECODE_VLI(&packet_cursor, &property_length, done);
    if (property_length > packet_cursor.len) {
        goto done;
    }

    struct aws_byte_cursor property_cursor = aws_byte_cursor_advance(&packet_cursor, property_length);
    while (property_cursor.len > 0) {
        if (s_read_subscribe_property(&subscribe_storage, &property_cursor)) {
            goto done;
        }
    }

    while (packet_cursor.len > 0) {
        struct aws_mqtt5_subscription_view subscription_view;
        AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR(&packet_cursor, &subscription_view.topic_filter, done);

        uint8_t subscription_options = 0;
        AWS_MQTT5_DECODE_U8(&packet_cursor, &subscription_options, done);

        subscription_view.no_local = (subscription_options & AWS_MQTT5_SUBSCRIBE_FLAGS_NO_LOCAL) != 0;
        subscription_view.retain_as_published =
            (subscription_options & AWS_MQTT5_SUBSCRIBE_FLAGS_RETAIN_AS_PUBLISHED) != 0;
        subscription_view.qos = (enum aws_mqtt5_qos)(
            (subscription_options >> AWS_MQTT5_SUBSCRIBE_FLAGS_QOS_BIT_POSITION) &
            AWS_MQTT5_SUBSCRIBE_FLAGS_QOS_BIT_MASK);
        subscription_view.retain_handling_type = (enum aws_mqtt5_retain_handling_type)(
            (subscription_options >> AWS_MQTT5_SUBSCRIBE_FLAGS_RETAIN_HANDLING_TYPE_BIT_POSITION) &
            AWS_MQTT5_SUBSCRIBE_FLAGS_RETAIN_HANDLING_TYPE_BIT_MASK);

        if (aws_array_list_push_back(&subscribe_storage.subscriptions, &subscription_view)) {
            goto done;
        }
    }

    storage_view->subscription_count = aws_array_list_length(&subscribe_storage.subscriptions);
    storage_view->subscriptions = subscribe_storage.subscriptions.data;
    storage_view->user_property_count = aws_mqtt5_user_property_set_size(&subscribe_storage.user_properties);
    storage_view->user_properties = subscribe_storage.user_properties.properties.data;

    if (packet_cursor.len == 0) {
        result = AWS_OP_SUCCESS;
    }

done:

    if (result == AWS_OP_SUCCESS) {
        if (decoder->options.on_packet_received != NULL) {
            result = (*decoder->options.on_packet_received)(
                AWS_MQTT5_PT_SUBSCRIBE, &subscribe_storage.storage_view, decoder->options.callback_user_data);
        }
    } else {
        AWS_LOGF_ERROR(
            AWS_LS_MQTT5_GENERAL,
            "(%p) aws_mqtt5_decoder - SUBSCRIBE decode failure",
            decoder->options.callback_user_data);
        aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
    }

    aws_mqtt5_packet_subscribe_storage_clean_up(&subscribe_storage);

    return result;
}

/* decode function for unsubscribe properties.  Movable to test-only code if we switched to a decoding function table */
static int s_read_unsubscribe_property(
    struct aws_mqtt5_packet_unsubscribe_storage *unsubscribe_storage,
    struct aws_byte_cursor *packet_cursor) {
    int result = AWS_OP_ERR;

    uint8_t property_type = 0;
    AWS_MQTT5_DECODE_U8(packet_cursor, &property_type, done);

    switch (property_type) {
        case AWS_MQTT5_PROPERTY_TYPE_USER_PROPERTY:
            if (aws_mqtt5_decode_user_property(packet_cursor, &unsubscribe_storage->user_properties)) {
                goto done;
            }
            break;

        default:
            goto done;
    }

    result = AWS_OP_SUCCESS;

done:

    if (result != AWS_OP_SUCCESS) {
        aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
    }

    return result;
}

/*
 * Decodes an UNSUBSCRIBE packet whose data must be in the scratch buffer.
 */
static int s_aws_mqtt5_decoder_decode_unsubscribe(struct aws_mqtt5_decoder *decoder) {
    struct aws_mqtt5_packet_unsubscribe_storage unsubscribe_storage;
    int result = AWS_OP_ERR;

    if (aws_mqtt5_packet_unsubscribe_storage_init_from_external_storage(&unsubscribe_storage, decoder->allocator)) {
        goto done;
    }

    struct aws_mqtt5_packet_unsubscribe_view *storage_view = &unsubscribe_storage.storage_view;

    /* UNSUBSCRIBE flags must be 2 by protocol*/
    uint8_t first_byte = decoder->packet_first_byte;
    if ((first_byte & 0x0F) != 2) {
        goto done;
    }

    struct aws_byte_cursor packet_cursor = decoder->packet_cursor;
    if (decoder->remaining_length != (uint32_t)packet_cursor.len) {
        goto done;
    }

    uint16_t packet_id = 0;
    AWS_MQTT5_DECODE_U16(&packet_cursor, &packet_id, done);
    unsubscribe_storage.storage_view.packet_id = packet_id;

    uint32_t property_length = 0;
    AWS_MQTT5_DECODE_VLI(&packet_cursor, &property_length, done);
    if (property_length > packet_cursor.len) {
        goto done;
    }

    struct aws_byte_cursor property_cursor = aws_byte_cursor_advance(&packet_cursor, property_length);
    while (property_cursor.len > 0) {
        if (s_read_unsubscribe_property(&unsubscribe_storage, &property_cursor)) {
            goto done;
        }
    }

    while (packet_cursor.len > 0) {
        struct aws_byte_cursor topic;
        AWS_MQTT5_DECODE_LENGTH_PREFIXED_CURSOR(&packet_cursor, &topic, done);
        if (aws_array_list_push_back(&unsubscribe_storage.topic_filters, &topic)) {
            goto done;
        }
    }

    storage_view->topic_filter_count = aws_array_list_length(&unsubscribe_storage.topic_filters);
    storage_view->topic_filters = unsubscribe_storage.topic_filters.data;
    storage_view->user_property_count = aws_mqtt5_user_property_set_size(&unsubscribe_storage.user_properties);
    storage_view->user_properties = unsubscribe_storage.user_properties.properties.data;

    result = AWS_OP_SUCCESS;

done:

    if (result == AWS_OP_SUCCESS) {
        if (decoder->options.on_packet_received != NULL) {
            result = (*decoder->options.on_packet_received)(
                AWS_MQTT5_PT_UNSUBSCRIBE, &unsubscribe_storage.storage_view, decoder->options.callback_user_data);
        }
    } else {
        AWS_LOGF_ERROR(
            AWS_LS_MQTT5_GENERAL,
            "(%p) aws_mqtt5_decoder - UNSUBSCRIBE decode failure",
            decoder->options.callback_user_data);
        aws_raise_error(AWS_ERROR_MQTT5_DECODE_PROTOCOL_ERROR);
    }

    aws_mqtt5_packet_unsubscribe_storage_clean_up(&unsubscribe_storage);

    return result;
}

void aws_mqtt5_decode_init_testing_function_table(struct aws_mqtt5_decoder_function_table *function_table) {
    *function_table = *g_aws_mqtt5_default_decoder_table;
    function_table->decoders_by_packet_type[AWS_MQTT5_PT_PINGREQ] = &s_aws_mqtt5_decoder_decode_pingreq;
    function_table->decoders_by_packet_type[AWS_MQTT5_PT_CONNECT] = &s_aws_mqtt5_decoder_decode_connect;
    function_table->decoders_by_packet_type[AWS_MQTT5_PT_SUBSCRIBE] = &s_aws_mqtt5_decoder_decode_subscribe;
    function_table->decoders_by_packet_type[AWS_MQTT5_PT_UNSUBSCRIBE] = &s_aws_mqtt5_decoder_decode_unsubscribe;
}

static int s_aws_mqtt5_mock_test_fixture_on_packet_received_fn(
    enum aws_mqtt5_packet_type type,
    void *packet_view,
    void *decoder_callback_user_data) {

    struct aws_mqtt5_mock_server_packet_record packet_record;
    AWS_ZERO_STRUCT(packet_record);

    struct aws_mqtt5_server_mock_connection_context *server_connection = decoder_callback_user_data;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = server_connection->test_fixture;

    switch (type) {
        case AWS_MQTT5_PT_CONNECT:
            packet_record.packet_storage =
                aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_packet_connect_storage));
            aws_mqtt5_packet_connect_storage_init(packet_record.packet_storage, test_fixture->allocator, packet_view);
            break;

        case AWS_MQTT5_PT_DISCONNECT:
            packet_record.packet_storage =
                aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_packet_disconnect_storage));
            aws_mqtt5_packet_disconnect_storage_init(
                packet_record.packet_storage, test_fixture->allocator, packet_view);
            break;

        case AWS_MQTT5_PT_SUBSCRIBE:
            packet_record.packet_storage =
                aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_packet_subscribe_storage));
            aws_mqtt5_packet_subscribe_storage_init(packet_record.packet_storage, test_fixture->allocator, packet_view);
            break;

        case AWS_MQTT5_PT_UNSUBSCRIBE:
            packet_record.packet_storage =
                aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_packet_unsubscribe_storage));
            aws_mqtt5_packet_unsubscribe_storage_init(
                packet_record.packet_storage, test_fixture->allocator, packet_view);
            break;

        case AWS_MQTT5_PT_PUBLISH:
            packet_record.packet_storage =
                aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_packet_publish_storage));
            aws_mqtt5_packet_publish_storage_init(packet_record.packet_storage, test_fixture->allocator, packet_view);
            break;

        case AWS_MQTT5_PT_PUBACK:
            packet_record.packet_storage =
                aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_packet_puback_storage));
            aws_mqtt5_packet_puback_storage_init(packet_record.packet_storage, test_fixture->allocator, packet_view);
            break;

        default:
            break;
    }

    packet_record.storage_allocator = test_fixture->allocator;
    packet_record.timestamp = (*test_fixture->client_vtable.get_current_time_fn)();
    packet_record.packet_type = type;

    aws_mutex_lock(&test_fixture->lock);
    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL, "mqtt5 mock server received packet of type %s", aws_mqtt5_packet_type_to_c_string(type));
    aws_array_list_push_back(&test_fixture->server_received_packets, &packet_record);
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);

    int result = AWS_OP_SUCCESS;
    aws_mqtt5_on_mock_server_packet_received_fn *packet_handler =
        test_fixture->server_function_table->packet_handlers[type];
    if (packet_handler != NULL) {
        result =
            (*packet_handler)(packet_view, server_connection, server_connection->test_fixture->mock_server_user_data);
    }

    return result;
}

static int s_process_read_message(
    struct aws_channel_handler *handler,
    struct aws_channel_slot *slot,
    struct aws_io_message *message) {

    struct aws_mqtt5_server_mock_connection_context *server_connection = handler->impl;

    if (message->message_type != AWS_IO_MESSAGE_APPLICATION_DATA) {
        return AWS_OP_ERR;
    }

    struct aws_byte_cursor message_cursor = aws_byte_cursor_from_buf(&message->message_data);

    int result = aws_mqtt5_decoder_on_data_received(&server_connection->decoder, message_cursor);
    if (result != AWS_OP_SUCCESS) {
        aws_channel_shutdown(server_connection->channel, aws_last_error());
        goto done;
    }

    aws_channel_slot_increment_read_window(slot, message->message_data.len);

done:

    aws_mem_release(message->allocator, message);

    return AWS_OP_SUCCESS;
}

static int s_shutdown(
    struct aws_channel_handler *handler,
    struct aws_channel_slot *slot,
    enum aws_channel_direction dir,
    int error_code,
    bool free_scarce_resources_immediately) {

    (void)handler;

    return aws_channel_slot_on_handler_shutdown_complete(slot, dir, error_code, free_scarce_resources_immediately);
}

static size_t s_initial_window_size(struct aws_channel_handler *handler) {
    (void)handler;

    return SIZE_MAX;
}

static void s_destroy(struct aws_channel_handler *handler) {
    struct aws_mqtt5_server_mock_connection_context *server_connection = handler->impl;

    aws_event_loop_cancel_task(
        aws_channel_get_event_loop(server_connection->channel), &server_connection->service_task);

    aws_mqtt5_decoder_clean_up(&server_connection->decoder);
    aws_mqtt5_encoder_clean_up(&server_connection->encoder);
    aws_mqtt5_inbound_topic_alias_resolver_clean_up(&server_connection->inbound_alias_resolver);

    aws_mem_release(server_connection->allocator, server_connection);
}

static size_t s_message_overhead(struct aws_channel_handler *handler) {
    (void)handler;

    return 0;
}

static struct aws_channel_handler_vtable s_mqtt5_mock_server_channel_handler_vtable = {
    .process_read_message = &s_process_read_message,
    .process_write_message = NULL,
    .increment_read_window = NULL,
    .shutdown = &s_shutdown,
    .initial_window_size = &s_initial_window_size,
    .message_overhead = &s_message_overhead,
    .destroy = &s_destroy,
};

static void s_mock_server_service_task_fn(struct aws_task *task, void *arg, enum aws_task_status status) {
    if (status != AWS_TASK_STATUS_RUN_READY) {
        return;
    }

    struct aws_mqtt5_server_mock_connection_context *server_connection = arg;

    aws_mqtt5_mock_server_service_fn *service_fn =
        server_connection->test_fixture->server_function_table->service_task_fn;
    if (service_fn != NULL) {
        (*service_fn)(server_connection, server_connection->test_fixture->mock_server_user_data);
    }

    uint64_t now = 0;
    aws_high_res_clock_get_ticks(&now);
    uint64_t next_service_time = now + aws_timestamp_convert(1, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);

    aws_event_loop_schedule_task_future(
        aws_channel_get_event_loop(server_connection->channel), task, next_service_time);
}

static void s_on_incoming_channel_setup_fn(
    struct aws_server_bootstrap *bootstrap,
    int error_code,
    struct aws_channel *channel,
    void *user_data) {
    (void)bootstrap;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = user_data;

    if (!error_code) {
        struct aws_channel_slot *test_handler_slot = aws_channel_slot_new(channel);
        aws_channel_slot_insert_end(channel, test_handler_slot);

        struct aws_mqtt5_server_mock_connection_context *server_connection =
            aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_server_mock_connection_context));
        server_connection->allocator = test_fixture->allocator;
        server_connection->channel = channel;
        server_connection->test_fixture = test_fixture;
        server_connection->slot = test_handler_slot;
        server_connection->handler.alloc = server_connection->allocator;
        server_connection->handler.vtable = &s_mqtt5_mock_server_channel_handler_vtable;
        server_connection->handler.impl = server_connection;

        aws_task_init(
            &server_connection->service_task,
            s_mock_server_service_task_fn,
            server_connection,
            "mock_server_service_task_fn");
        aws_event_loop_schedule_task_now(aws_channel_get_event_loop(channel), &server_connection->service_task);

        aws_channel_slot_set_handler(server_connection->slot, &server_connection->handler);

        aws_mqtt5_encode_init_testing_function_table(&server_connection->encoding_table);

        struct aws_mqtt5_encoder_options encoder_options = {
            .client = NULL,
            .encoders = &server_connection->encoding_table,
        };

        aws_mqtt5_encoder_init(&server_connection->encoder, server_connection->allocator, &encoder_options);

        aws_mqtt5_decode_init_testing_function_table(&server_connection->decoding_table);

        struct aws_mqtt5_decoder_options decoder_options = {
            .callback_user_data = server_connection,
            .on_packet_received = s_aws_mqtt5_mock_test_fixture_on_packet_received_fn,
            .decoder_table = &server_connection->decoding_table,
        };

        aws_mqtt5_decoder_init(&server_connection->decoder, server_connection->allocator, &decoder_options);
        aws_mqtt5_inbound_topic_alias_resolver_init(
            &server_connection->inbound_alias_resolver, server_connection->allocator);
        aws_mqtt5_inbound_topic_alias_resolver_reset(
            &server_connection->inbound_alias_resolver, test_fixture->maximum_inbound_topic_aliases);
        aws_mqtt5_decoder_set_inbound_topic_alias_resolver(
            &server_connection->decoder, &server_connection->inbound_alias_resolver);

        aws_mutex_lock(&test_fixture->lock);
        test_fixture->server_channel = channel;
        aws_mutex_unlock(&test_fixture->lock);

        /*
         * Just like the tls tests in aws-c-io, it's possible for the server channel setup to execute after the client
         * channel setup has already posted data to the socket.  In this case, the read notification gets lost because
         * the server hasn't subscribed to it yet and then we hang and time out.  So do the same thing we do for
         * tls server channel setup and force a read of the socket after we're fully initialized.
         */
        aws_channel_trigger_read(channel);
    }
}

static void s_on_incoming_channel_shutdown_fn(
    struct aws_server_bootstrap *bootstrap,
    int error_code,
    struct aws_channel *channel,
    void *user_data) {
    (void)bootstrap;
    (void)error_code;
    (void)channel;
    (void)user_data;
}

static void s_on_listener_destroy(struct aws_server_bootstrap *bootstrap, void *user_data) {
    (void)bootstrap;
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = user_data;
    aws_mutex_lock(&test_fixture->lock);
    test_fixture->listener_destroyed = true;
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_one(&test_fixture->signal);
}

static bool s_is_listener_destroyed(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;
    return test_fixture->listener_destroyed;
}

static void s_wait_on_listener_cleanup(struct aws_mqtt5_client_mock_test_fixture *test_fixture) {
    aws_mutex_lock(&test_fixture->lock);
    aws_condition_variable_wait_pred(&test_fixture->signal, &test_fixture->lock, s_is_listener_destroyed, test_fixture);
    aws_mutex_unlock(&test_fixture->lock);
}

static bool s_has_client_terminated(void *arg) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = arg;
    return test_fixture->client_terminated;
}

static void s_wait_for_client_terminated(struct aws_mqtt5_client_mock_test_fixture *test_context) {
    aws_mutex_lock(&test_context->lock);
    aws_condition_variable_wait_pred(&test_context->signal, &test_context->lock, s_has_client_terminated, test_context);
    aws_mutex_unlock(&test_context->lock);
}

void s_aws_mqtt5_test_fixture_lifecycle_event_handler(const struct aws_mqtt5_client_lifecycle_event *event) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = event->user_data;

    struct aws_mqtt5_lifecycle_event_record *record =
        aws_mem_calloc(test_fixture->allocator, 1, sizeof(struct aws_mqtt5_lifecycle_event_record));
    record->allocator = test_fixture->allocator;
    aws_high_res_clock_get_ticks(&record->timestamp);
    record->event = *event;

    if (event->settings != NULL) {
        record->settings_storage = *event->settings;
        record->event.settings = &record->settings_storage;
    }

    if (event->disconnect_data != NULL) {
        aws_mqtt5_packet_disconnect_storage_init(
            &record->disconnect_storage, record->allocator, event->disconnect_data);
        record->event.disconnect_data = &record->disconnect_storage.storage_view;
    }

    if (event->connack_data != NULL) {
        aws_mqtt5_packet_connack_storage_init(&record->connack_storage, record->allocator, event->connack_data);
        record->event.connack_data = &record->connack_storage.storage_view;
    }

    aws_mutex_lock(&test_fixture->lock);
    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL,
        "mqtt5 mock server received lifecycle event of type %s",
        aws_mqtt5_client_lifecycle_event_type_to_c_string(event->event_type));
    aws_array_list_push_back(&test_fixture->lifecycle_events, &record);
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);

    aws_mqtt5_client_connection_event_callback_fn *event_callback = test_fixture->original_lifecycle_event_handler;
    if (event_callback != NULL) {
        struct aws_mqtt5_client_lifecycle_event event_copy = *event;
        event_copy.user_data = test_fixture->original_lifecycle_event_handler_user_data;

        (*event_callback)(&event_copy);
    }
}

void s_aws_mqtt5_test_fixture_state_changed_callback(
    struct aws_mqtt5_client *client,
    enum aws_mqtt5_client_state old_state,
    enum aws_mqtt5_client_state new_state,
    void *vtable_user_data) {
    (void)old_state;
    (void)client;

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = vtable_user_data;

    aws_mutex_lock(&test_fixture->lock);
    AWS_LOGF_DEBUG(
        AWS_LS_MQTT5_GENERAL,
        "mqtt5 mock server received client state change to %s",
        aws_mqtt5_client_state_to_c_string(new_state));
    aws_array_list_push_back(&test_fixture->client_states, &new_state);
    aws_mutex_unlock(&test_fixture->lock);
}

static void s_aws_mqtt5_test_fixture_statistics_changed_callback(
    struct aws_mqtt5_client *client,
    struct aws_mqtt5_operation *operation,
    void *vtable_user_data) {
    (void)operation;

    struct aws_mqtt5_client_mock_test_fixture *test_fixture = vtable_user_data;

    struct aws_mqtt5_client_operation_statistics stats;
    AWS_ZERO_STRUCT(stats);

    aws_mutex_lock(&test_fixture->lock);
    aws_mqtt5_client_get_stats(client, &stats);
    aws_array_list_push_back(&test_fixture->client_statistics, &stats);
    aws_mutex_unlock(&test_fixture->lock);
}

static void s_on_test_client_termination(void *user_data) {
    struct aws_mqtt5_client_mock_test_fixture *test_fixture = user_data;

    aws_mutex_lock(&test_fixture->lock);
    test_fixture->client_terminated = true;
    aws_mutex_unlock(&test_fixture->lock);
    aws_condition_variable_notify_all(&test_fixture->signal);
}

int aws_mqtt5_client_mock_test_fixture_init(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    struct aws_allocator *allocator,
    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *options) {

    AWS_ZERO_STRUCT(*test_fixture);

    test_fixture->allocator = allocator;

    aws_mutex_init(&test_fixture->lock);
    aws_condition_variable_init(&test_fixture->signal);

    test_fixture->server_function_table = options->server_function_table;
    test_fixture->mock_server_user_data = options->mock_server_user_data;

    struct aws_socket_options socket_options = {
        .connect_timeout_ms = 1000,
        .domain = AWS_SOCKET_LOCAL,
    };

    test_fixture->socket_options = socket_options;
    test_fixture->server_elg = aws_event_loop_group_new_default(allocator, 1, NULL);
    test_fixture->server_bootstrap = aws_server_bootstrap_new(allocator, test_fixture->server_elg);

    test_fixture->client_elg = aws_event_loop_group_new_default(allocator, 4, NULL);
    struct aws_host_resolver_default_options resolver_options = {
        .el_group = test_fixture->client_elg,
        .max_entries = 1,
    };
    test_fixture->host_resolver = aws_host_resolver_new_default(allocator, &resolver_options);

    struct aws_client_bootstrap_options bootstrap_options = {
        .event_loop_group = test_fixture->client_elg,
        .user_data = test_fixture,
        .host_resolver = test_fixture->host_resolver,
    };

    test_fixture->client_bootstrap = aws_client_bootstrap_new(allocator, &bootstrap_options);

    aws_socket_endpoint_init_local_address_for_test(&test_fixture->endpoint);

    struct aws_server_socket_channel_bootstrap_options server_bootstrap_options = {
        .bootstrap = test_fixture->server_bootstrap,
        .host_name = test_fixture->endpoint.address,
        .port = test_fixture->endpoint.port,
        .socket_options = &test_fixture->socket_options,
        .incoming_callback = s_on_incoming_channel_setup_fn,
        .shutdown_callback = s_on_incoming_channel_shutdown_fn,
        .destroy_callback = s_on_listener_destroy,
        .user_data = test_fixture,
    };
    test_fixture->listener = aws_server_bootstrap_new_socket_listener(&server_bootstrap_options);

    test_fixture->original_lifecycle_event_handler = options->client_options->lifecycle_event_handler;
    test_fixture->original_lifecycle_event_handler_user_data =
        options->client_options->lifecycle_event_handler_user_data;
    options->client_options->lifecycle_event_handler = &s_aws_mqtt5_test_fixture_lifecycle_event_handler;
    options->client_options->lifecycle_event_handler_user_data = test_fixture;

    options->client_options->host_name = aws_byte_cursor_from_c_str(test_fixture->endpoint.address);
    options->client_options->port = test_fixture->endpoint.port;
    options->client_options->socket_options = &test_fixture->socket_options;
    options->client_options->bootstrap = test_fixture->client_bootstrap;

    options->client_options->client_termination_handler = s_on_test_client_termination;
    options->client_options->client_termination_handler_user_data = test_fixture;

    test_fixture->client = aws_mqtt5_client_new(allocator, options->client_options);

    test_fixture->client_vtable = *aws_mqtt5_client_get_default_vtable();
    test_fixture->client_vtable.on_client_state_change_callback_fn = s_aws_mqtt5_test_fixture_state_changed_callback;
    test_fixture->client_vtable.on_client_statistics_changed_callback_fn =
        s_aws_mqtt5_test_fixture_statistics_changed_callback;
    test_fixture->client_vtable.vtable_user_data = test_fixture;

    aws_mqtt5_client_set_vtable(test_fixture->client, &test_fixture->client_vtable);

    aws_array_list_init_dynamic(
        &test_fixture->server_received_packets, allocator, 10, sizeof(struct aws_mqtt5_mock_server_packet_record));

    aws_array_list_init_dynamic(
        &test_fixture->lifecycle_events, allocator, 10, sizeof(struct aws_mqtt5_lifecycle_event_record *));

    aws_array_list_init_dynamic(&test_fixture->client_states, allocator, 10, sizeof(enum aws_mqtt5_client_state));
    aws_array_list_init_dynamic(
        &test_fixture->client_statistics, allocator, 10, sizeof(struct aws_mqtt5_client_operation_statistics));

    return AWS_OP_SUCCESS;
}

static void s_destroy_packet_storage(
    void *packet_storage,
    struct aws_allocator *allocator,
    enum aws_mqtt5_packet_type packet_type) {
    switch (packet_type) {
        case AWS_MQTT5_PT_CONNECT:
            aws_mqtt5_packet_connect_storage_clean_up(packet_storage);
            break;

        case AWS_MQTT5_PT_DISCONNECT:
            aws_mqtt5_packet_disconnect_storage_clean_up(packet_storage);
            break;

        case AWS_MQTT5_PT_SUBSCRIBE:
            aws_mqtt5_packet_subscribe_storage_clean_up(packet_storage);
            break;

        case AWS_MQTT5_PT_SUBACK:
            aws_mqtt5_packet_suback_storage_clean_up(packet_storage);
            break;

        case AWS_MQTT5_PT_UNSUBSCRIBE:
            aws_mqtt5_packet_unsubscribe_storage_clean_up(packet_storage);
            break;

        case AWS_MQTT5_PT_UNSUBACK:
            aws_mqtt5_packet_unsuback_storage_clean_up(packet_storage);
            break;

        case AWS_MQTT5_PT_PUBLISH:
            aws_mqtt5_packet_publish_storage_clean_up(packet_storage);
            break;

        case AWS_MQTT5_PT_PUBACK:
            /* TODO */
            break;

        case AWS_MQTT5_PT_PINGREQ:
            AWS_FATAL_ASSERT(packet_storage == NULL);
            break;

        default:
            AWS_FATAL_ASSERT(false);
    }

    if (packet_storage != NULL) {
        aws_mem_release(allocator, packet_storage);
    }
}

static void s_destroy_lifecycle_event_storage(struct aws_mqtt5_lifecycle_event_record *event) {
    aws_mqtt5_packet_connack_storage_clean_up(&event->connack_storage);
    aws_mqtt5_packet_disconnect_storage_clean_up(&event->disconnect_storage);

    aws_mem_release(event->allocator, event);
}

void aws_mqtt5_client_mock_test_fixture_clean_up(struct aws_mqtt5_client_mock_test_fixture *test_fixture) {
    aws_mqtt5_client_release(test_fixture->client);
    s_wait_for_client_terminated(test_fixture);
    aws_client_bootstrap_release(test_fixture->client_bootstrap);
    aws_host_resolver_release(test_fixture->host_resolver);
    aws_server_bootstrap_destroy_socket_listener(test_fixture->server_bootstrap, test_fixture->listener);

    s_wait_on_listener_cleanup(test_fixture);

    aws_server_bootstrap_release(test_fixture->server_bootstrap);
    aws_event_loop_group_release(test_fixture->server_elg);
    aws_event_loop_group_release(test_fixture->client_elg);

    for (size_t i = 0; i < aws_array_list_length(&test_fixture->server_received_packets); ++i) {
        struct aws_mqtt5_mock_server_packet_record *packet = NULL;
        aws_array_list_get_at_ptr(&test_fixture->server_received_packets, (void **)&packet, i);

        s_destroy_packet_storage(packet->packet_storage, packet->storage_allocator, packet->packet_type);
    }

    aws_array_list_clean_up(&test_fixture->server_received_packets);

    for (size_t i = 0; i < aws_array_list_length(&test_fixture->lifecycle_events); ++i) {
        struct aws_mqtt5_lifecycle_event_record *event = NULL;
        aws_array_list_get_at(&test_fixture->lifecycle_events, &event, i);

        s_destroy_lifecycle_event_storage(event);
    }

    aws_array_list_clean_up(&test_fixture->lifecycle_events);
    aws_array_list_clean_up(&test_fixture->client_states);
    aws_array_list_clean_up(&test_fixture->client_statistics);

    aws_mutex_clean_up(&test_fixture->lock);
    aws_condition_variable_clean_up(&test_fixture->signal);
}

#define AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs, rhs)                                                               \
    if ((lhs) != (rhs)) {                                                                                              \
        return false;                                                                                                  \
    }

#define AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs, rhs)                                                      \
    if (((lhs) != NULL) != ((rhs) != NULL)) {                                                                          \
        return false;                                                                                                  \
    }                                                                                                                  \
    if (((lhs) != NULL) && ((rhs) != NULL) && (*(lhs) != *(rhs))) {                                                    \
        return false;                                                                                                  \
    }

#define AWS_MQTT5_CLIENT_TEST_CHECK_CURSOR_EQUALS(lhs, rhs)                                                            \
    if (!aws_byte_cursor_eq(&(lhs), &(rhs))) {                                                                         \
        return false;                                                                                                  \
    }

#define AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs, rhs)                                                   \
    if (((lhs) != NULL) != ((rhs) != NULL)) {                                                                          \
        return false;                                                                                                  \
    }                                                                                                                  \
    if (((lhs) != NULL) && ((rhs) != NULL) && (!aws_byte_cursor_eq(lhs, rhs))) {                                       \
        return false;                                                                                                  \
    }

#define AWS_MQTT5_CLIENT_TEST_CHECK_USER_PROPERTIES(lhs_props, lhs_prop_count, rhs_props, rhs_prop_count)              \
    if (aws_mqtt5_test_verify_user_properties_raw((lhs_prop_count), (lhs_props), (rhs_prop_count), (rhs_props)) !=     \
        AWS_OP_SUCCESS) {                                                                                              \
        return false;                                                                                                  \
    }

static bool s_aws_publish_packets_equal(
    const struct aws_mqtt5_packet_publish_view *lhs,
    const struct aws_mqtt5_packet_publish_view *rhs) {

    // Don't check packet id intentionally

    AWS_MQTT5_CLIENT_TEST_CHECK_CURSOR_EQUALS(lhs->payload, rhs->payload);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->qos, rhs->qos);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->duplicate, rhs->duplicate);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->retain, rhs->retain);
    AWS_MQTT5_CLIENT_TEST_CHECK_CURSOR_EQUALS(lhs->topic, rhs->topic);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->payload_format, rhs->payload_format);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(
        lhs->message_expiry_interval_seconds, rhs->message_expiry_interval_seconds);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->topic_alias, rhs->topic_alias);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->response_topic, rhs->response_topic);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->correlation_data, rhs->correlation_data);

    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->subscription_identifier_count, rhs->subscription_identifier_count);
    for (size_t i = 0; i < lhs->subscription_identifier_count; ++i) {
        AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->subscription_identifiers[i], rhs->subscription_identifiers[i]);
    }

    AWS_MQTT5_CLIENT_TEST_CHECK_USER_PROPERTIES(
        lhs->user_properties, lhs->user_property_count, rhs->user_properties, rhs->user_property_count);

    return true;
}

static bool s_aws_connect_packets_equal(
    const struct aws_mqtt5_packet_connect_view *lhs,
    const struct aws_mqtt5_packet_connect_view *rhs) {
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->keep_alive_interval_seconds, rhs->keep_alive_interval_seconds);
    AWS_MQTT5_CLIENT_TEST_CHECK_CURSOR_EQUALS(lhs->client_id, rhs->client_id);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->username, rhs->username);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->password, rhs->password);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->clean_start, rhs->clean_start);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(
        lhs->session_expiry_interval_seconds, rhs->session_expiry_interval_seconds);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(
        lhs->request_response_information, rhs->request_response_information);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->request_problem_information, rhs->request_problem_information);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->receive_maximum, rhs->receive_maximum);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->topic_alias_maximum, rhs->topic_alias_maximum);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->maximum_packet_size_bytes, rhs->maximum_packet_size_bytes);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->will_delay_interval_seconds, rhs->will_delay_interval_seconds);

    if ((lhs->will != NULL) != (rhs->will != NULL)) {
        return false;
    }

    if (lhs->will) {
        if (!s_aws_publish_packets_equal(lhs->will, rhs->will)) {
            return false;
        }
    }

    AWS_MQTT5_CLIENT_TEST_CHECK_USER_PROPERTIES(
        lhs->user_properties, lhs->user_property_count, rhs->user_properties, rhs->user_property_count);

    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->authentication_method, rhs->authentication_method);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->authentication_data, rhs->authentication_data);

    return true;
}

static bool s_aws_disconnect_packets_equal(
    const struct aws_mqtt5_packet_disconnect_view *lhs,
    const struct aws_mqtt5_packet_disconnect_view *rhs) {

    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->reason_code, rhs->reason_code);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->reason_string, rhs->reason_string);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(
        lhs->session_expiry_interval_seconds, rhs->session_expiry_interval_seconds);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->server_reference, rhs->server_reference);

    AWS_MQTT5_CLIENT_TEST_CHECK_USER_PROPERTIES(
        lhs->user_properties, lhs->user_property_count, rhs->user_properties, rhs->user_property_count);

    return true;
}

static bool s_are_subscription_views_equal(
    const struct aws_mqtt5_subscription_view *lhs,
    const struct aws_mqtt5_subscription_view *rhs) {
    AWS_MQTT5_CLIENT_TEST_CHECK_CURSOR_EQUALS(lhs->topic_filter, rhs->topic_filter);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->qos, rhs->qos);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->no_local, rhs->no_local);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->retain_as_published, rhs->retain_as_published);
    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->retain_handling_type, rhs->retain_handling_type);

    return true;
}

static bool s_aws_subscribe_packets_equal(
    const struct aws_mqtt5_packet_subscribe_view *lhs,
    const struct aws_mqtt5_packet_subscribe_view *rhs) {

    // Don't check packet id intentionally

    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->subscription_count, rhs->subscription_count);
    for (size_t i = 0; i < lhs->subscription_count; ++i) {
        const struct aws_mqtt5_subscription_view *lhs_sub_view = &lhs->subscriptions[i];
        const struct aws_mqtt5_subscription_view *rhs_sub_view = &rhs->subscriptions[i];

        if (!s_are_subscription_views_equal(lhs_sub_view, rhs_sub_view)) {
            return false;
        }
    }

    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_INT_EQUALS(lhs->subscription_identifier, rhs->subscription_identifier);

    AWS_MQTT5_CLIENT_TEST_CHECK_USER_PROPERTIES(
        lhs->user_properties, lhs->user_property_count, rhs->user_properties, rhs->user_property_count);

    return true;
}

static bool s_aws_unsubscribe_packets_equal(
    const struct aws_mqtt5_packet_unsubscribe_view *lhs,
    const struct aws_mqtt5_packet_unsubscribe_view *rhs) {

    // Don't check packet id intentionally

    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->topic_filter_count, rhs->topic_filter_count);
    for (size_t i = 0; i < lhs->topic_filter_count; ++i) {
        struct aws_byte_cursor lhs_topic_filter = lhs->topic_filters[i];
        struct aws_byte_cursor rhs_topic_filter = rhs->topic_filters[i];

        AWS_MQTT5_CLIENT_TEST_CHECK_CURSOR_EQUALS(lhs_topic_filter, rhs_topic_filter);
    }

    AWS_MQTT5_CLIENT_TEST_CHECK_USER_PROPERTIES(
        lhs->user_properties, lhs->user_property_count, rhs->user_properties, rhs->user_property_count);

    return true;
}

static bool s_aws_puback_packets_equal(
    const struct aws_mqtt5_packet_puback_view *lhs,
    const struct aws_mqtt5_packet_puback_view *rhs) {

    // Don't check packet id intentionally

    AWS_MQTT5_CLIENT_TEST_CHECK_INT_EQUALS(lhs->reason_code, rhs->reason_code);
    AWS_MQTT5_CLIENT_TEST_CHECK_OPTIONAL_CURSOR_EQUALS(lhs->reason_string, rhs->reason_string);

    AWS_MQTT5_CLIENT_TEST_CHECK_USER_PROPERTIES(
        lhs->user_properties, lhs->user_property_count, rhs->user_properties, rhs->user_property_count);

    return true;
}

bool aws_mqtt5_client_test_are_packets_equal(
    enum aws_mqtt5_packet_type packet_type,
    void *lhs_packet_storage,
    void *rhs_packet_storage) {
    switch (packet_type) {
        case AWS_MQTT5_PT_CONNECT:
            return s_aws_connect_packets_equal(
                &((struct aws_mqtt5_packet_connect_storage *)lhs_packet_storage)->storage_view,
                &((struct aws_mqtt5_packet_connect_storage *)rhs_packet_storage)->storage_view);

        case AWS_MQTT5_PT_DISCONNECT:
            return s_aws_disconnect_packets_equal(
                &((struct aws_mqtt5_packet_disconnect_storage *)lhs_packet_storage)->storage_view,
                &((struct aws_mqtt5_packet_disconnect_storage *)rhs_packet_storage)->storage_view);

        case AWS_MQTT5_PT_SUBSCRIBE:
            return s_aws_subscribe_packets_equal(
                &((struct aws_mqtt5_packet_subscribe_storage *)lhs_packet_storage)->storage_view,
                &((struct aws_mqtt5_packet_subscribe_storage *)rhs_packet_storage)->storage_view);

        case AWS_MQTT5_PT_UNSUBSCRIBE:
            return s_aws_unsubscribe_packets_equal(
                &((struct aws_mqtt5_packet_unsubscribe_storage *)lhs_packet_storage)->storage_view,
                &((struct aws_mqtt5_packet_unsubscribe_storage *)rhs_packet_storage)->storage_view);

        case AWS_MQTT5_PT_PUBLISH:
            return s_aws_publish_packets_equal(
                &((struct aws_mqtt5_packet_publish_storage *)lhs_packet_storage)->storage_view,
                &((struct aws_mqtt5_packet_publish_storage *)rhs_packet_storage)->storage_view);

        case AWS_MQTT5_PT_PUBACK:
            return s_aws_puback_packets_equal(
                &((struct aws_mqtt5_packet_puback_storage *)lhs_packet_storage)->storage_view,
                &((struct aws_mqtt5_packet_puback_storage *)rhs_packet_storage)->storage_view);

        case AWS_MQTT5_PT_PINGREQ:
        case AWS_MQTT5_PT_PINGRESP:
            return true;

        default:
            return false;
    }
}

size_t aws_mqtt5_linked_list_length(struct aws_linked_list *list) {
    size_t length = 0;
    struct aws_linked_list_node *node = aws_linked_list_begin(list);
    while (node != aws_linked_list_end(list)) {
        ++length;
        node = aws_linked_list_next(node);
    }

    return length;
}

#define TEST_IO_MESSAGE_LENGTH 4096

int aws_mqtt5_mock_server_send_packet(
    struct aws_mqtt5_server_mock_connection_context *connection,
    enum aws_mqtt5_packet_type packet_type,
    void *packet) {
    aws_mqtt5_encoder_append_packet_encoding(&connection->encoder, packet_type, packet);

    struct aws_io_message *message = aws_channel_acquire_message_from_pool(
        connection->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, TEST_IO_MESSAGE_LENGTH);
    if (message == NULL) {
        return AWS_OP_ERR;
    }

    enum aws_mqtt5_encoding_result result =
        aws_mqtt5_encoder_encode_to_buffer(&connection->encoder, &message->message_data);
    AWS_FATAL_ASSERT(result == AWS_MQTT5_ER_FINISHED);

    if (aws_channel_slot_send_message(connection->slot, message, AWS_CHANNEL_DIR_WRITE)) {
        aws_mem_release(message->allocator, message);
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}
