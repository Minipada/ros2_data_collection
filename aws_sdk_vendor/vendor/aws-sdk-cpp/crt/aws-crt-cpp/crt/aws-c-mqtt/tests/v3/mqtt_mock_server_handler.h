#ifndef MQTT_MOCK_SERVER_HANDLER_H
#define MQTT_MOCK_SERVER_HANDLER_H
/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/mqtt/private/packets.h>

struct aws_channel_handler;
struct aws_channel_slot;

static const int MOCK_LOG_SUBJECT = 60000;

struct mqtt_decoded_packet {
    enum aws_mqtt_packet_type type;
    struct aws_allocator *alloc;

    /* CONNECT */
    bool clean_session;
    bool has_will;
    bool will_retain;
    bool has_password;
    bool has_username;
    uint16_t keep_alive_timeout;
    enum aws_mqtt_qos will_qos;
    struct aws_byte_cursor client_identifier; /* These cursors live with the received_message */
    struct aws_byte_cursor will_topic;
    struct aws_byte_cursor will_message;
    struct aws_byte_cursor username;
    struct aws_byte_cursor password;

    /* PUBLISH SUBSCRIBE UNSUBSCRIBE */
    uint16_t packet_identifier;
    struct aws_byte_cursor topic_name;         /* PUBLISH topic */
    struct aws_byte_cursor publish_payload;    /* PUBLISH payload */
    struct aws_array_list sub_topic_filters;   /* list of aws_mqtt_subscription for SUBSCRIBE */
    struct aws_array_list unsub_topic_filters; /* list of aws_byte_cursor for UNSUBSCRIBE */
    bool duplicate;                            /* PUBLISH only */

    /* index of the received packet, indicating when it's received by the server */
    size_t index;
};

struct aws_channel_handler *new_mqtt_mock_server(struct aws_allocator *allocator);
void destroy_mqtt_mock_server(struct aws_channel_handler *handler);
void mqtt_mock_server_handler_update_slot(struct aws_channel_handler *handler, struct aws_channel_slot *slot);

/**
 * Mock server sends a publish packet back to client
 */
int mqtt_mock_server_send_publish(
    struct aws_channel_handler *handler,
    struct aws_byte_cursor *topic,
    struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain);

/**
 * Mock server sends a publish packet back to client with user-controlled packet id
 */
int mqtt_mock_server_send_publish_by_id(
    struct aws_channel_handler *handler,
    uint16_t packet_id,
    struct aws_byte_cursor *topic,
    struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain);

/**
 * Sets whether or not connacks return session present
 */
void mqtt_mock_server_set_session_present(struct aws_channel_handler *handler, bool session_present);

/**
 * Sets whether or not server should reflect received PUBLISH packets.
 */
void mqtt_mock_server_set_publish_reflection(struct aws_channel_handler *handler, bool reflect_publishes);

/**
 * Set max number of PINGRESP that mock server will send back to client
 */
void mqtt_mock_server_set_max_ping_resp(struct aws_channel_handler *handler, size_t max_ping);

/**
 * Set max number of CONACK that mock server will send back to client
 */
void mqtt_mock_server_set_max_connack(struct aws_channel_handler *handler, size_t connack_avail);

/**
 * Disable the automatically response (suback/unsuback/puback) to the client
 */
void mqtt_mock_server_disable_auto_ack(struct aws_channel_handler *handler);

/**
 * Enable the automatically response (suback/unsuback/puback) to the client
 */
void mqtt_mock_server_enable_auto_ack(struct aws_channel_handler *handler);

/**
 * Sets what reason code to return in subacks
 */
void mqtt_mock_server_suback_reason_code(struct aws_channel_handler *handler, uint8_t reason_code);

/**
 * Send response back the client given the packet ID
 */
int mqtt_mock_server_send_unsuback(struct aws_channel_handler *handler, uint16_t packet_id);
int mqtt_mock_server_send_puback(struct aws_channel_handler *handler, uint16_t packet_id);

int mqtt_mock_server_send_single_suback(
    struct aws_channel_handler *handler,
    uint16_t packet_id,
    enum aws_mqtt_qos return_code);
/**
 * Wait for puback_count PUBACK packages from client
 */
void mqtt_mock_server_wait_for_pubacks(struct aws_channel_handler *handler, size_t puback_count);

/**
 * Getters for decoded packets, call mqtt_mock_server_decode_packets first.
 */
size_t mqtt_mock_server_decoded_packets_count(struct aws_channel_handler *handler);
/**
 * Get the decoded packet by index
 */
struct mqtt_decoded_packet *mqtt_mock_server_get_decoded_packet_by_index(struct aws_channel_handler *handler, size_t i);
/**
 * Get the latest received packet by index
 */
struct mqtt_decoded_packet *mqtt_mock_server_get_latest_decoded_packet(struct aws_channel_handler *handler);
/**
 * Get the decoded packet by packet_id started from search_start_idx (included), Note: it may have multiple packets with
 * the same ID, this will return the earliest received on with the packet_id. If out_idx is not NULL, the index of found
 * packet will be stored at there, and if failed to find the packet, it will be set to SIZE_MAX, and the return value
 * will be NULL.
 */
struct mqtt_decoded_packet *mqtt_mock_server_find_decoded_packet_by_id(
    struct aws_channel_handler *handler,
    size_t search_start_idx,
    uint16_t packet_id,
    size_t *out_idx);
/**
 * Get the decoded packet by type started from search_start_idx (included), Note: it may have multiple packets with
 * the same type, this will return the earliest received on with the packet_id. If out_idx is not NULL, the index of
 * found packet will be stored at there, and if failed to find the packet, it will be set to SIZE_MAX, and the return
 * value will be NULL.
 */
struct mqtt_decoded_packet *mqtt_mock_server_find_decoded_packet_by_type(
    struct aws_channel_handler *handler,
    size_t search_start_idx,
    enum aws_mqtt_packet_type type,
    size_t *out_idx);

/**
 * Run all received messages through, and decode the messages.
 */
int mqtt_mock_server_decode_packets(struct aws_channel_handler *handler);

/**
 * Returns the number of PINGs the server has gotten
 */
size_t mqtt_mock_server_get_ping_count(struct aws_channel_handler *handler);

#endif /* MQTT_MOCK_SERVER_HANDLER_H */
