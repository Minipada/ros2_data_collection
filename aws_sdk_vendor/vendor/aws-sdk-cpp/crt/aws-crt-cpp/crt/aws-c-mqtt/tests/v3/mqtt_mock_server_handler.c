/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqtt_mock_server_handler.h"

#include <aws/common/condition_variable.h>
#include <aws/common/mutex.h>
#include <aws/io/channel.h>
#include <aws/testing/aws_test_harness.h>

/* 10sec */
#define CVAR_TIMEOUT (10 * (int64_t)1000000000)

struct mqtt_mock_server_handler {
    struct aws_channel_handler handler;

    struct aws_channel_slot *slot;

    /* partial incoming packet, finish decoding when the rest arrives */
    struct aws_byte_buf pending_packet;

    /* Lock must be held when accessing "synced" data */
    struct {
        struct aws_mutex lock;
        struct aws_condition_variable cvar;

        size_t ping_resp_avail;
        size_t pubacks_received;
        size_t ping_received;
        size_t connacks_avail;
        bool session_present;
        bool reflect_publishes;
        bool auto_ack;
        uint8_t suback_reason_code;

        /* last ID used when sending PUBLISH (QoS1+) to client */
        uint16_t last_packet_id;

        /* contains aws_byte_buf with raw bytes of each packet received. */
        struct aws_array_list raw_packets;

        /* progress decoding from raw_packets to decoded_packets*/
        size_t decoded_index;
    } synced;

    /* contains mqtt_decoded_packet* for each packet received
     * only accessed from main thread by test code */
    struct aws_array_list decoded_packets;
};

struct mqtt_decoded_packet *s_mqtt_decoded_packet_create(struct aws_allocator *alloc) {
    struct mqtt_decoded_packet *packet = aws_mem_calloc(alloc, 1, sizeof(struct mqtt_decoded_packet));
    packet->alloc = alloc;
    aws_array_list_init_dynamic(&packet->sub_topic_filters, alloc, 1, sizeof(struct aws_mqtt_subscription));
    aws_array_list_init_dynamic(&packet->unsub_topic_filters, alloc, 1, sizeof(struct aws_byte_cursor));
    return packet;
}

static void s_mqtt_decoded_packet_destroy(struct mqtt_decoded_packet *packet) {
    aws_array_list_clean_up(&packet->sub_topic_filters);
    aws_array_list_clean_up(&packet->unsub_topic_filters);
    aws_mem_release(packet->alloc, packet);
}

static int s_mqtt_mock_server_handler_process_packet(
    struct mqtt_mock_server_handler *server,
    struct aws_byte_cursor *message_cur) {
    struct aws_byte_buf received_message;
    aws_byte_buf_init_copy_from_cursor(&received_message, server->handler.alloc, *message_cur);
    aws_mutex_lock(&server->synced.lock);
    aws_array_list_push_back(&server->synced.raw_packets, &received_message);
    aws_mutex_unlock(&server->synced.lock);

    struct aws_byte_cursor message_cur_cpy = *message_cur;
    int err = 0;

    enum aws_mqtt_packet_type packet_type = aws_mqtt_get_packet_type(message_cur_cpy.ptr);
    switch (packet_type) {
        case AWS_MQTT_PACKET_CONNECT: {
            size_t connacks_available = 0;
            bool session_present = false;
            aws_mutex_lock(&server->synced.lock);
            AWS_LOGF_DEBUG(
                MOCK_LOG_SUBJECT,
                "server, CONNECT received, %llu available connacks.",
                (long long unsigned)server->synced.connacks_avail);
            connacks_available = server->synced.connacks_avail > 0 ? server->synced.connacks_avail-- : 0;
            session_present = server->synced.session_present;
            aws_mutex_unlock(&server->synced.lock);

            if (connacks_available) {
                struct aws_io_message *connack_msg =
                    aws_channel_acquire_message_from_pool(server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, 256);

                struct aws_mqtt_packet_connack conn_ack;
                err |= aws_mqtt_packet_connack_init(&conn_ack, session_present, AWS_MQTT_CONNECT_ACCEPTED);
                err |= aws_mqtt_packet_connack_encode(&connack_msg->message_data, &conn_ack);
                if (aws_channel_slot_send_message(server->slot, connack_msg, AWS_CHANNEL_DIR_WRITE)) {
                    err |= 1;
                    AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "Failed to send connack with error %d", aws_last_error());
                }
            }
            break;
        }

        case AWS_MQTT_PACKET_DISCONNECT:
            AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "server, DISCONNECT received");

            err |= aws_channel_shutdown(server->slot->channel, AWS_OP_SUCCESS);
            break;

        case AWS_MQTT_PACKET_PINGREQ: {
            AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "server, PINGREQ received");

            size_t ping_resp_available = 0;
            aws_mutex_lock(&server->synced.lock);
            ping_resp_available = server->synced.ping_resp_avail > 0 ? server->synced.ping_resp_avail-- : 0;
            server->synced.ping_received += 1;
            aws_mutex_unlock(&server->synced.lock);

            if (ping_resp_available) {
                struct aws_io_message *ping_resp =
                    aws_channel_acquire_message_from_pool(server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, 256);
                struct aws_mqtt_packet_connection packet;
                err |= aws_mqtt_packet_pingresp_init(&packet);
                err |= aws_mqtt_packet_connection_encode(&ping_resp->message_data, &packet);
                err |= aws_channel_slot_send_message(server->slot, ping_resp, AWS_CHANNEL_DIR_WRITE);
            }
            break;
        }

        case AWS_MQTT_PACKET_SUBSCRIBE: {
            AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "server, SUBSCRIBE received");

            struct aws_mqtt_packet_subscribe subscribe_packet;
            err |= aws_mqtt_packet_subscribe_init(&subscribe_packet, server->handler.alloc, 0);
            err |= aws_mqtt_packet_subscribe_decode(message_cur, &subscribe_packet);

            aws_mutex_lock(&server->synced.lock);
            bool auto_ack = server->synced.auto_ack;
            uint8_t reason_code = server->synced.suback_reason_code;
            aws_mutex_unlock(&server->synced.lock);

            if (auto_ack) {
                struct aws_io_message *suback_msg =
                    aws_channel_acquire_message_from_pool(server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, 256);
                struct aws_mqtt_packet_suback suback;
                err |= aws_mqtt_packet_suback_init(&suback, server->handler.alloc, subscribe_packet.packet_identifier);
                const size_t num_filters = aws_array_list_length(&subscribe_packet.topic_filters);
                for (size_t i = 0; i < num_filters; ++i) {
                    err |= aws_mqtt_packet_suback_add_return_code(&suback, reason_code);
                }
                err |= aws_mqtt_packet_suback_encode(&suback_msg->message_data, &suback);
                err |= aws_channel_slot_send_message(server->slot, suback_msg, AWS_CHANNEL_DIR_WRITE);
                aws_mqtt_packet_suback_clean_up(&suback);
            }
            aws_mqtt_packet_subscribe_clean_up(&subscribe_packet);
            break;
        }

        case AWS_MQTT_PACKET_UNSUBSCRIBE: {
            AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "server, UNSUBSCRIBE received");

            struct aws_mqtt_packet_unsubscribe unsubscribe_packet;
            err |= aws_mqtt_packet_unsubscribe_init(&unsubscribe_packet, server->handler.alloc, 0);
            err |= aws_mqtt_packet_unsubscribe_decode(message_cur, &unsubscribe_packet);

            aws_mutex_lock(&server->synced.lock);
            bool auto_ack = server->synced.auto_ack;
            aws_mutex_unlock(&server->synced.lock);

            if (auto_ack) {
                struct aws_io_message *unsuback_msg =
                    aws_channel_acquire_message_from_pool(server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, 256);
                struct aws_mqtt_packet_ack unsuback;
                err |= aws_mqtt_packet_unsuback_init(&unsuback, unsubscribe_packet.packet_identifier);
                err |= aws_mqtt_packet_ack_encode(&unsuback_msg->message_data, &unsuback);
                err |= aws_channel_slot_send_message(server->slot, unsuback_msg, AWS_CHANNEL_DIR_WRITE);
            }
            aws_mqtt_packet_unsubscribe_clean_up(&unsubscribe_packet);
            break;
        }

        case AWS_MQTT_PACKET_PUBLISH: {
            AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "server, PUBLISH received");

            struct aws_mqtt_packet_publish publish_packet;
            err |= aws_mqtt_packet_publish_decode(message_cur, &publish_packet);

            aws_mutex_lock(&server->synced.lock);
            bool auto_ack = server->synced.auto_ack;
            bool reflect_publishes = server->synced.reflect_publishes;
            aws_mutex_unlock(&server->synced.lock);

            uint8_t qos = (publish_packet.fixed_header.flags >> 1) & 0x3;
            // Do not send puback if qos0
            if (auto_ack && qos != 0) {
                struct aws_io_message *puback_msg =
                    aws_channel_acquire_message_from_pool(server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, 256);
                struct aws_mqtt_packet_ack puback;
                err |= aws_mqtt_packet_puback_init(&puback, publish_packet.packet_identifier);
                err |= aws_mqtt_packet_ack_encode(&puback_msg->message_data, &puback);
                err |= aws_channel_slot_send_message(server->slot, puback_msg, AWS_CHANNEL_DIR_WRITE);
            }

            if (reflect_publishes) {
                struct aws_io_message *publish_msg =
                    aws_channel_acquire_message_from_pool(server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, 1024);
                struct aws_mqtt_packet_publish publish;
                // reusing the packet identifier here is weird but reasonably safe, they're separate id spaces
                err |= aws_mqtt_packet_publish_init(
                    &publish,
                    false,
                    qos,
                    false,
                    publish_packet.topic_name,
                    publish_packet.packet_identifier,
                    publish_packet.payload);
                err |= aws_mqtt_packet_publish_encode(&publish_msg->message_data, &publish);
                err |= aws_channel_slot_send_message(server->slot, publish_msg, AWS_CHANNEL_DIR_WRITE);
            }
            break;
        }

        case AWS_MQTT_PACKET_PUBACK:
            AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "server, PUBACK received");

            aws_mutex_lock(&server->synced.lock);
            server->synced.pubacks_received++;
            aws_mutex_unlock(&server->synced.lock);
            err |= aws_condition_variable_notify_one(&server->synced.cvar);
            break;

        default:
            break;
    }
    if (err) {
        AWS_LOGF_DEBUG(MOCK_LOG_SUBJECT, "server, process packet failed, the package type is %d", packet_type);
        /* crash */
        AWS_FATAL_ASSERT(!err);
    }
    return AWS_OP_SUCCESS;
}

static int s_mqtt_mock_server_handler_process_read_message(
    struct aws_channel_handler *handler,
    struct aws_channel_slot *slot,
    struct aws_io_message *message) {

    struct mqtt_mock_server_handler *server = handler->impl;

    struct aws_byte_cursor message_cursor = aws_byte_cursor_from_buf(&message->message_data);

    if (server->pending_packet.len) {
        size_t to_read = server->pending_packet.capacity - server->pending_packet.len;

        bool packet_complete = true;
        if (to_read > message_cursor.len) {
            to_read = message_cursor.len;
            packet_complete = false;
        }

        struct aws_byte_cursor chunk = aws_byte_cursor_advance(&message_cursor, to_read);
        aws_byte_buf_write_from_whole_cursor(&server->pending_packet, chunk);

        if (!packet_complete) {
            goto cleanup;
        }

        struct aws_byte_cursor packet_data = aws_byte_cursor_from_buf(&server->pending_packet);
        s_mqtt_mock_server_handler_process_packet(server, &packet_data);
    }

    while (message_cursor.len) {
        struct aws_byte_cursor header_decode = message_cursor;
        struct aws_mqtt_fixed_header packet_header;
        AWS_ZERO_STRUCT(packet_header);
        int result = aws_mqtt_fixed_header_decode(&header_decode, &packet_header);

        const size_t fixed_header_size = message_cursor.len - header_decode.len;

        if (result) {
            if (aws_last_error() == AWS_ERROR_SHORT_BUFFER) {
                AWS_FATAL_ASSERT(
                    packet_header.remaining_length > 0 &&
                    "need to handle getting partial packet, but not enough to know total length");
                aws_byte_buf_init(
                    &server->pending_packet, server->handler.alloc, fixed_header_size + packet_header.remaining_length);
                aws_byte_buf_write_from_whole_cursor(&server->pending_packet, message_cursor);
                aws_reset_error();
                goto cleanup;
            }
        }
        struct aws_byte_cursor packet_data =
            aws_byte_cursor_advance(&message_cursor, fixed_header_size + packet_header.remaining_length);
        s_mqtt_mock_server_handler_process_packet(server, &packet_data);
    }
cleanup:
    aws_mem_release(message->allocator, message);
    aws_channel_slot_increment_read_window(slot, message->message_data.len);
    return AWS_OP_SUCCESS;
}

struct mqtt_mock_server_send_args {
    struct aws_channel_task task;
    struct mqtt_mock_server_handler *server;
    struct aws_byte_buf data;
};

static void s_mqtt_send_in_thread(struct aws_channel_task *channel_task, void *arg, enum aws_task_status status) {
    (void)channel_task;
    struct mqtt_mock_server_send_args *send_args = arg;

    if (status == AWS_TASK_STATUS_RUN_READY) {
        struct aws_io_message *msg = aws_channel_acquire_message_from_pool(
            send_args->server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, send_args->data.len);
        AWS_FATAL_ASSERT(aws_byte_buf_write_from_whole_buffer(&msg->message_data, send_args->data));
        AWS_FATAL_ASSERT(0 == aws_channel_slot_send_message(send_args->server->slot, msg, AWS_CHANNEL_DIR_WRITE));
    }

    aws_byte_buf_clean_up(&send_args->data);
    aws_mem_release(send_args->server->handler.alloc, send_args);
}

static struct mqtt_mock_server_send_args *s_mqtt_send_args_create(struct mqtt_mock_server_handler *server) {
    struct mqtt_mock_server_send_args *args =
        aws_mem_calloc(server->handler.alloc, 1, sizeof(struct mqtt_mock_server_send_args));
    aws_channel_task_init(&args->task, s_mqtt_send_in_thread, args, "mqtt_mock_server_send_in_thread");
    args->server = server;
    aws_byte_buf_init(&args->data, server->handler.alloc, 1024);
    return args;
}

int mqtt_mock_server_send_publish_by_id(
    struct aws_channel_handler *handler,
    uint16_t packet_id,
    struct aws_byte_cursor *topic,
    struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain) {

    struct mqtt_mock_server_handler *server = handler->impl;

    struct mqtt_mock_server_send_args *args = s_mqtt_send_args_create(server);

    struct aws_mqtt_packet_publish publish;
    ASSERT_SUCCESS(aws_mqtt_packet_publish_init(&publish, retain, qos, dup, *topic, packet_id, *payload));
    ASSERT_SUCCESS(aws_mqtt_packet_publish_encode(&args->data, &publish));

    aws_channel_schedule_task_now(server->slot->channel, &args->task);

    return AWS_OP_SUCCESS;
}

int mqtt_mock_server_send_publish(
    struct aws_channel_handler *handler,
    struct aws_byte_cursor *topic,
    struct aws_byte_cursor *payload,
    bool dup,
    enum aws_mqtt_qos qos,
    bool retain) {

    struct mqtt_mock_server_handler *server = handler->impl;
    aws_mutex_lock(&server->synced.lock);
    uint16_t id = qos == 0 ? 0 : ++server->synced.last_packet_id;
    aws_mutex_unlock(&server->synced.lock);

    return mqtt_mock_server_send_publish_by_id(handler, id, topic, payload, dup, qos, retain);
}

int mqtt_mock_server_send_single_suback(
    struct aws_channel_handler *handler,
    uint16_t packet_id,
    enum aws_mqtt_qos return_code) {

    struct mqtt_mock_server_handler *server = handler->impl;

    struct mqtt_mock_server_send_args *args = s_mqtt_send_args_create(server);

    struct aws_mqtt_packet_suback suback;
    ASSERT_SUCCESS(aws_mqtt_packet_suback_init(&suback, server->handler.alloc, packet_id));
    ASSERT_SUCCESS(aws_mqtt_packet_suback_add_return_code(&suback, return_code));
    ASSERT_SUCCESS(aws_mqtt_packet_suback_encode(&args->data, &suback));
    aws_mqtt_packet_suback_clean_up(&suback);

    aws_channel_schedule_task_now(server->slot->channel, &args->task);

    return AWS_OP_SUCCESS;
}

static int s_mqtt_mock_server_handler_process_write_message(
    struct aws_channel_handler *handler,
    struct aws_channel_slot *slot,
    struct aws_io_message *message) {
    (void)handler;
    (void)slot;
    (void)message;

    return AWS_OP_SUCCESS;
}

static int s_mqtt_mock_server_handler_increment_read_window(
    struct aws_channel_handler *handler,
    struct aws_channel_slot *slot,
    size_t size) {
    (void)handler;
    aws_channel_slot_increment_read_window(slot, size);
    return AWS_OP_SUCCESS;
}

void mqtt_mock_server_handler_update_slot(struct aws_channel_handler *handler, struct aws_channel_slot *slot) {
    struct mqtt_mock_server_handler *server = handler->impl;
    server->slot = slot;
}

static int s_mqtt_mock_server_handler_shutdown(
    struct aws_channel_handler *handler,
    struct aws_channel_slot *slot,
    enum aws_channel_direction dir,
    int error_code,
    bool free_scarce_resources_immediately) {
    (void)handler;
    return aws_channel_slot_on_handler_shutdown_complete(slot, dir, error_code, free_scarce_resources_immediately);
}

static size_t s_mqtt_mock_server_handler_initial_window_size(struct aws_channel_handler *handler) {
    (void)handler;
    return SIZE_MAX;
}

static size_t s_mqtt_mock_server_handler_message_overhead(struct aws_channel_handler *handler) {
    (void)handler;
    return 0;
}

static void s_mqtt_mock_server_handler_destroy(struct aws_channel_handler *handler) {
    (void)handler;
}

static struct aws_channel_handler_vtable s_mqtt_mock_server_handler_vtable = {
    .process_read_message = s_mqtt_mock_server_handler_process_read_message,
    .process_write_message = s_mqtt_mock_server_handler_process_write_message,
    .increment_read_window = s_mqtt_mock_server_handler_increment_read_window,
    .shutdown = s_mqtt_mock_server_handler_shutdown,
    .initial_window_size = s_mqtt_mock_server_handler_initial_window_size,
    .message_overhead = s_mqtt_mock_server_handler_message_overhead,
    .destroy = s_mqtt_mock_server_handler_destroy,
};

struct aws_channel_handler *new_mqtt_mock_server(struct aws_allocator *allocator) {

    struct mqtt_mock_server_handler *server = aws_mem_calloc(allocator, 1, sizeof(struct mqtt_mock_server_handler));
    aws_array_list_init_dynamic(&server->decoded_packets, allocator, 4, sizeof(struct mqtt_decoded_packet *));
    aws_array_list_init_dynamic(&server->synced.raw_packets, allocator, 4, sizeof(struct aws_byte_buf));

    server->handler.impl = server;
    server->handler.vtable = &s_mqtt_mock_server_handler_vtable;
    server->handler.alloc = allocator;
    server->synced.ping_resp_avail = SIZE_MAX;
    server->synced.connacks_avail = SIZE_MAX;
    server->synced.auto_ack = true;
    server->synced.suback_reason_code = AWS_MQTT_QOS_EXACTLY_ONCE;
    aws_mutex_init(&server->synced.lock);
    aws_condition_variable_init(&server->synced.cvar);

    return &server->handler;
}

void destroy_mqtt_mock_server(struct aws_channel_handler *handler) {
    struct mqtt_mock_server_handler *server = handler->impl;

    for (size_t i = 0; i < aws_array_list_length(&server->decoded_packets); ++i) {
        struct mqtt_decoded_packet *packet = NULL;
        aws_array_list_get_at(&server->decoded_packets, &packet, i);
        s_mqtt_decoded_packet_destroy(packet);
    }
    aws_array_list_clean_up(&server->decoded_packets);

    for (size_t i = 0; i < aws_array_list_length(&server->synced.raw_packets); ++i) {
        struct aws_byte_buf *byte_buf_ptr = NULL;
        aws_array_list_get_at_ptr(&server->synced.raw_packets, (void **)&byte_buf_ptr, i);
        aws_byte_buf_clean_up(byte_buf_ptr);
    }
    aws_array_list_clean_up(&server->synced.raw_packets);

    aws_mutex_clean_up(&server->synced.lock);
    aws_condition_variable_clean_up(&server->synced.cvar);
    aws_mem_release(handler->alloc, server);
}

void mqtt_mock_server_set_session_present(struct aws_channel_handler *handler, bool session_present) {
    struct mqtt_mock_server_handler *server = handler->impl;

    aws_mutex_lock(&server->synced.lock);
    server->synced.session_present = session_present;
    aws_mutex_unlock(&server->synced.lock);
}

void mqtt_mock_server_set_publish_reflection(struct aws_channel_handler *handler, bool reflect_publishes) {
    struct mqtt_mock_server_handler *server = handler->impl;

    aws_mutex_lock(&server->synced.lock);
    server->synced.reflect_publishes = reflect_publishes;
    aws_mutex_unlock(&server->synced.lock);
}

void mqtt_mock_server_set_max_ping_resp(struct aws_channel_handler *handler, size_t max_ping) {
    struct mqtt_mock_server_handler *server = handler->impl;

    aws_mutex_lock(&server->synced.lock);
    server->synced.ping_resp_avail = max_ping;
    aws_mutex_unlock(&server->synced.lock);
}

void mqtt_mock_server_set_max_connack(struct aws_channel_handler *handler, size_t connack_avail) {
    struct mqtt_mock_server_handler *server = handler->impl;

    aws_mutex_lock(&server->synced.lock);
    server->synced.connacks_avail = connack_avail;
    aws_mutex_unlock(&server->synced.lock);
}

void mqtt_mock_server_disable_auto_ack(struct aws_channel_handler *handler) {
    struct mqtt_mock_server_handler *server = handler->impl;

    aws_mutex_lock(&server->synced.lock);
    server->synced.auto_ack = false;
    aws_mutex_unlock(&server->synced.lock);
}

void mqtt_mock_server_enable_auto_ack(struct aws_channel_handler *handler) {
    struct mqtt_mock_server_handler *server = handler->impl;

    aws_mutex_lock(&server->synced.lock);
    server->synced.auto_ack = true;
    aws_mutex_unlock(&server->synced.lock);
}

void mqtt_mock_server_suback_reason_code(struct aws_channel_handler *handler, uint8_t reason_code) {
    struct mqtt_mock_server_handler *server = handler->impl;

    aws_mutex_lock(&server->synced.lock);
    server->synced.suback_reason_code = reason_code;
    aws_mutex_unlock(&server->synced.lock);
}

struct mqtt_mock_server_ack_args {
    struct aws_channel_task task;
    struct aws_mqtt_packet_ack ack;
    struct mqtt_mock_server_handler *server;
};

static void s_send_ack_in_thread(struct aws_channel_task *channel_task, void *arg, enum aws_task_status status) {
    (void)channel_task;
    (void)status;
    struct mqtt_mock_server_ack_args *ack_args = arg;
    struct aws_io_message *msg =
        aws_channel_acquire_message_from_pool(ack_args->server->slot->channel, AWS_IO_MESSAGE_APPLICATION_DATA, 256);
    AWS_FATAL_ASSERT(0 == aws_mqtt_packet_ack_encode(&msg->message_data, &ack_args->ack));
    AWS_FATAL_ASSERT(0 == aws_channel_slot_send_message(ack_args->server->slot, msg, AWS_CHANNEL_DIR_WRITE));

    aws_mem_release(ack_args->server->handler.alloc, ack_args);
}

static int s_send_ack(struct aws_channel_handler *handler, uint16_t packet_id, enum aws_mqtt_packet_type type) {
    struct mqtt_mock_server_handler *server = handler->impl;
    struct mqtt_mock_server_ack_args *args =
        aws_mem_calloc(server->handler.alloc, 1, sizeof(struct mqtt_mock_server_ack_args));
    args->server = server;
    aws_channel_task_init(&args->task, s_send_ack_in_thread, args, "send ack in thread");
    switch (type) {
        case AWS_MQTT_PACKET_PUBACK:
            ASSERT_SUCCESS(aws_mqtt_packet_puback_init(&args->ack, packet_id));
            break;
        case AWS_MQTT_PACKET_UNSUBACK:
            ASSERT_SUCCESS(aws_mqtt_packet_unsuback_init(&args->ack, packet_id));
            break;
        default:
            AWS_FATAL_ASSERT(0);
            break;
    }

    aws_channel_schedule_task_now(server->slot->channel, &args->task);
    return AWS_OP_SUCCESS;
}

int mqtt_mock_server_send_unsuback(struct aws_channel_handler *handler, uint16_t packet_id) {
    return s_send_ack(handler, packet_id, AWS_MQTT_PACKET_UNSUBACK);
}
int mqtt_mock_server_send_puback(struct aws_channel_handler *handler, uint16_t packet_id) {
    return s_send_ack(handler, packet_id, AWS_MQTT_PACKET_PUBACK);
}

struct puback_waiter {
    struct mqtt_mock_server_handler *server;
    size_t wait_for_count;
};

static bool s_is_pubacks_complete(void *arg) {
    struct puback_waiter *waiter = arg;

    return waiter->server->synced.pubacks_received >= waiter->wait_for_count;
}

void mqtt_mock_server_wait_for_pubacks(struct aws_channel_handler *handler, size_t puback_count) {
    struct mqtt_mock_server_handler *server = handler->impl;

    struct puback_waiter waiter;
    waiter.server = server;
    waiter.wait_for_count = puback_count;

    aws_mutex_lock(&server->synced.lock);
    AWS_FATAL_ASSERT(
        0 == aws_condition_variable_wait_for_pred(
                 &server->synced.cvar, &server->synced.lock, CVAR_TIMEOUT, s_is_pubacks_complete, &waiter));
    aws_mutex_unlock(&server->synced.lock);
}

size_t mqtt_mock_server_decoded_packets_count(struct aws_channel_handler *handler) {
    struct mqtt_mock_server_handler *server = handler->impl;
    size_t count = aws_array_list_length(&server->decoded_packets);
    return count;
}

struct mqtt_decoded_packet *mqtt_mock_server_get_decoded_packet_by_index(
    struct aws_channel_handler *handler,
    size_t i) {
    struct mqtt_mock_server_handler *server = handler->impl;
    AWS_FATAL_ASSERT(mqtt_mock_server_decoded_packets_count(handler) > i);
    struct mqtt_decoded_packet *packet = NULL;
    aws_array_list_get_at(&server->decoded_packets, &packet, i);
    return packet;
}

struct mqtt_decoded_packet *mqtt_mock_server_get_latest_decoded_packet(struct aws_channel_handler *handler) {
    size_t packet_count = mqtt_mock_server_decoded_packets_count(handler);
    AWS_FATAL_ASSERT(packet_count > 0);
    return mqtt_mock_server_get_decoded_packet_by_index(handler, packet_count - 1);
}

struct mqtt_decoded_packet *mqtt_mock_server_find_decoded_packet_by_id(
    struct aws_channel_handler *handler,
    size_t search_start_idx,
    uint16_t packet_id,
    size_t *out_idx) {
    struct mqtt_mock_server_handler *server = handler->impl;
    size_t len = aws_array_list_length(&server->decoded_packets);
    AWS_FATAL_ASSERT(search_start_idx < len);
    for (size_t i = search_start_idx; i < len; i++) {
        struct mqtt_decoded_packet *packet = NULL;
        aws_array_list_get_at(&server->decoded_packets, &packet, i);
        if (packet->packet_identifier == packet_id) {
            if (out_idx) {
                *out_idx = i;
            }
            return packet;
        }
    }
    if (out_idx) {
        *out_idx = SIZE_MAX;
    }
    return NULL;
}

struct mqtt_decoded_packet *mqtt_mock_server_find_decoded_packet_by_type(
    struct aws_channel_handler *handler,
    size_t search_start_idx,
    enum aws_mqtt_packet_type type,
    size_t *out_idx) {
    struct mqtt_mock_server_handler *server = handler->impl;
    size_t len = aws_array_list_length(&server->decoded_packets);
    AWS_FATAL_ASSERT(search_start_idx < len);
    for (size_t i = search_start_idx; i < len; i++) {
        struct mqtt_decoded_packet *packet = NULL;
        aws_array_list_get_at(&server->decoded_packets, &packet, i);
        if (packet->type == type) {
            if (out_idx) {
                *out_idx = i;
            }
            return packet;
        }
    }
    if (out_idx) {
        *out_idx = SIZE_MAX;
    }
    return NULL;
}

int mqtt_mock_server_decode_packets(struct aws_channel_handler *handler) {
    struct mqtt_mock_server_handler *server = handler->impl;
    struct aws_allocator *alloc = handler->alloc;

    /* NOTE: if there's an error in this function we may not unlock, but don't care because
     * this is only called from main test thread which will fail if this errors */
    aws_mutex_lock(&server->synced.lock);

    struct aws_array_list raw_packets = server->synced.raw_packets;
    size_t length = aws_array_list_length(&raw_packets);
    for (size_t index = server->synced.decoded_index; index < length; index++) {
        struct aws_byte_buf raw_packet = {0};
        aws_array_list_get_at(&raw_packets, &raw_packet, index);
        struct aws_byte_cursor message_cur = aws_byte_cursor_from_buf(&raw_packet);

        struct mqtt_decoded_packet *packet = s_mqtt_decoded_packet_create(alloc);
        packet->index = index;
        packet->type = aws_mqtt_get_packet_type(message_cur.ptr);

        switch (packet->type) {
            case AWS_MQTT_PACKET_CONNECT: {
                struct aws_mqtt_packet_connect connect_packet;
                AWS_ZERO_STRUCT(connect_packet);
                ASSERT_SUCCESS(aws_mqtt_packet_connect_decode(&message_cur, &connect_packet));
                packet->clean_session = connect_packet.clean_session;
                packet->has_will = connect_packet.has_will;
                packet->will_retain = connect_packet.will_retain;
                packet->has_password = connect_packet.has_password;
                packet->has_username = connect_packet.has_username;
                packet->keep_alive_timeout = connect_packet.keep_alive_timeout;
                packet->will_qos = connect_packet.will_qos;
                packet->client_identifier = connect_packet.client_identifier;
                if (packet->has_will) {
                    packet->will_topic = connect_packet.will_topic;
                    packet->will_message = connect_packet.will_message;
                }
                if (packet->has_username) {
                    packet->username = connect_packet.username;
                }
                if (packet->has_password) {
                    packet->password = connect_packet.password;
                }
                break;
            }
            case AWS_MQTT_PACKET_SUBSCRIBE: {
                struct aws_mqtt_packet_subscribe subscribe_packet;
                AWS_ZERO_STRUCT(subscribe_packet);
                ASSERT_SUCCESS(aws_mqtt_packet_subscribe_init(&subscribe_packet, alloc, 0));
                ASSERT_SUCCESS(aws_mqtt_packet_subscribe_decode(&message_cur, &subscribe_packet));
                packet->packet_identifier = subscribe_packet.packet_identifier;
                /* copy the array one by one for simplicity */
                for (size_t i = 0; i < aws_array_list_length(&subscribe_packet.topic_filters); i++) {
                    struct aws_mqtt_subscription val;
                    aws_array_list_get_at(&subscribe_packet.topic_filters, &val, i);
                    aws_array_list_push_back(&packet->sub_topic_filters, &val);
                }
                aws_mqtt_packet_subscribe_clean_up(&subscribe_packet);
                break;
            }
            case AWS_MQTT_PACKET_UNSUBSCRIBE: {
                struct aws_mqtt_packet_unsubscribe unsubscribe_packet;
                AWS_ZERO_STRUCT(unsubscribe_packet);
                ASSERT_SUCCESS(aws_mqtt_packet_unsubscribe_init(&unsubscribe_packet, alloc, 0));
                ASSERT_SUCCESS(aws_mqtt_packet_unsubscribe_decode(&message_cur, &unsubscribe_packet));
                packet->packet_identifier = unsubscribe_packet.packet_identifier;
                /* copy the array one by one for simplicity */
                for (size_t i = 0; i < aws_array_list_length(&unsubscribe_packet.topic_filters); i++) {
                    struct aws_byte_cursor val;
                    aws_array_list_get_at(&unsubscribe_packet.topic_filters, &val, i);
                    aws_array_list_push_back(&packet->unsub_topic_filters, &val);
                }
                aws_mqtt_packet_unsubscribe_clean_up(&unsubscribe_packet);
                break;
            }
            case AWS_MQTT_PACKET_PUBLISH: {
                struct aws_mqtt_packet_publish publish_packet;
                ASSERT_SUCCESS(aws_mqtt_packet_publish_decode(&message_cur, &publish_packet));
                packet->packet_identifier = publish_packet.packet_identifier;
                packet->topic_name = publish_packet.topic_name;
                packet->publish_payload = publish_packet.payload;
                packet->duplicate = aws_mqtt_packet_publish_get_dup(&publish_packet);
                break;
            }
            case AWS_MQTT_PACKET_PUBACK: {
                struct aws_mqtt_packet_ack puback;
                ASSERT_SUCCESS(aws_mqtt_packet_ack_decode(&message_cur, &puback));
                packet->packet_identifier = puback.packet_identifier;
                break;
            }
            case AWS_MQTT_PACKET_DISCONNECT:
            case AWS_MQTT_PACKET_PINGREQ:
                /* Nothing to decode, just record that type of packet has received */
                break;
            default:
                AWS_FATAL_ASSERT(0 && "mock unsupported packet type decoded");
        }

        aws_array_list_push_back(&server->decoded_packets, &packet);
    }
    server->synced.decoded_index = length;
    aws_mutex_unlock(&server->synced.lock);
    return AWS_OP_SUCCESS;
}

size_t mqtt_mock_server_get_ping_count(struct aws_channel_handler *handler) {
    struct mqtt_mock_server_handler *server = handler->impl;
    aws_mutex_lock(&server->synced.lock);
    size_t count = server->synced.ping_received;
    aws_mutex_unlock(&server->synced.lock);
    return count;
}
