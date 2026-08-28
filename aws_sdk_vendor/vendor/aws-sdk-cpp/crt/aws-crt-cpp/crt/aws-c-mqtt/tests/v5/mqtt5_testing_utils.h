#ifndef MQTT_MQTT5_TESTING_UTILS_H
#define MQTT_MQTT5_TESTING_UTILS_H
/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/mqtt.h>

#include <aws/common/array_list.h>
#include <aws/common/condition_variable.h>
#include <aws/common/mutex.h>
#include <aws/io/channel.h>
#include <aws/mqtt/private/v5/mqtt5_client_impl.h>
#include <aws/mqtt/private/v5/mqtt5_decoder.h>
#include <aws/mqtt/private/v5/mqtt5_encoder.h>
#include <aws/mqtt/v5/mqtt5_types.h>

struct aws_event_loop_group;

struct aws_mqtt5_mock_server_packet_record {
    struct aws_allocator *storage_allocator;

    uint64_t timestamp;

    void *packet_storage;
    enum aws_mqtt5_packet_type packet_type;
};

struct aws_mqtt5_lifecycle_event_record {
    struct aws_allocator *allocator;

    uint64_t timestamp;

    struct aws_mqtt5_client_lifecycle_event event;

    struct aws_mqtt5_negotiated_settings settings_storage;
    struct aws_mqtt5_packet_disconnect_storage disconnect_storage;
    struct aws_mqtt5_packet_connack_storage connack_storage;
};

struct aws_mqtt5_server_mock_connection_context {
    struct aws_allocator *allocator;

    struct aws_channel *channel;
    struct aws_channel_handler handler;
    struct aws_channel_slot *slot;

    struct aws_mqtt5_encoder_function_table encoding_table;
    struct aws_mqtt5_encoder encoder;

    struct aws_mqtt5_decoder_function_table decoding_table;
    struct aws_mqtt5_decoder decoder;
    struct aws_mqtt5_inbound_topic_alias_resolver inbound_alias_resolver;

    struct aws_mqtt5_client_mock_test_fixture *test_fixture;

    struct aws_task service_task;
};

typedef int(aws_mqtt5_on_mock_server_packet_received_fn)(
    void *packet_view,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *packet_received_user_data);

typedef void(
    aws_mqtt5_mock_server_service_fn)(struct aws_mqtt5_server_mock_connection_context *mock_server, void *user_data);

struct aws_mqtt5_mock_server_vtable {
    aws_mqtt5_on_mock_server_packet_received_fn *packet_handlers[16];
    aws_mqtt5_mock_server_service_fn *service_task_fn;
};

struct aws_mqtt5_client_mqtt5_mock_test_fixture_options {
    struct aws_mqtt5_client_options *client_options;
    const struct aws_mqtt5_mock_server_vtable *server_function_table;

    void *mock_server_user_data;
};

struct aws_mqtt5_client_mock_test_fixture {
    struct aws_allocator *allocator;

    struct aws_event_loop_group *client_elg;
    struct aws_event_loop_group *server_elg;
    struct aws_host_resolver *host_resolver;
    struct aws_client_bootstrap *client_bootstrap;
    struct aws_server_bootstrap *server_bootstrap;
    struct aws_socket_endpoint endpoint;
    struct aws_socket_options socket_options;
    struct aws_socket *listener;
    struct aws_channel *server_channel;

    const struct aws_mqtt5_mock_server_vtable *server_function_table;
    void *mock_server_user_data;

    struct aws_mqtt5_client_vtable client_vtable;
    struct aws_mqtt5_client *client;

    aws_mqtt5_client_connection_event_callback_fn *original_lifecycle_event_handler;
    void *original_lifecycle_event_handler_user_data;

    uint16_t maximum_inbound_topic_aliases;

    struct aws_mutex lock;
    struct aws_condition_variable signal;
    struct aws_array_list server_received_packets;
    struct aws_array_list lifecycle_events;
    struct aws_array_list client_states;
    struct aws_array_list client_statistics;
    bool listener_destroyed;
    bool subscribe_complete;
    bool disconnect_completion_callback_invoked;
    bool client_terminated;
    uint32_t total_pubacks_received;
    uint32_t publishes_received;
    uint32_t successful_pubacks_received;
    uint32_t timeouts_received;

    uint32_t server_maximum_inflight_publishes;
    uint32_t server_current_inflight_publishes;
};

struct mqtt5_client_test_options {
    struct aws_mqtt5_client_topic_alias_options topic_aliasing_options;
    struct aws_mqtt5_packet_connect_view connect_options;
    struct aws_mqtt5_client_options client_options;
    struct aws_mqtt5_mock_server_vtable server_function_table;
};

struct aws_mqtt5_mock_server_reconnect_state {
    size_t required_connection_count_threshold;

    size_t connection_attempts;
    uint64_t connect_timestamp;

    uint64_t successful_connection_disconnect_delay_ms;
};

int aws_mqtt5_test_verify_user_properties_raw(
    size_t property_count,
    const struct aws_mqtt5_user_property *properties,
    size_t expected_count,
    const struct aws_mqtt5_user_property *expected_properties);

void aws_mqtt5_encode_init_testing_function_table(struct aws_mqtt5_encoder_function_table *function_table);

void aws_mqtt5_decode_init_testing_function_table(struct aws_mqtt5_decoder_function_table *function_table);

int aws_mqtt5_client_mock_test_fixture_init(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    struct aws_allocator *allocator,
    struct aws_mqtt5_client_mqtt5_mock_test_fixture_options *options);

void aws_mqtt5_client_mock_test_fixture_clean_up(struct aws_mqtt5_client_mock_test_fixture *test_fixture);

bool aws_mqtt5_client_test_are_packets_equal(
    enum aws_mqtt5_packet_type packet_type,
    void *lhs_packet_storage,
    void *rhs_packet_storage);

size_t aws_mqtt5_linked_list_length(struct aws_linked_list *list);

void aws_mqtt5_client_test_init_default_options(struct mqtt5_client_test_options *test_options);

void aws_wait_for_connected_lifecycle_event(struct aws_mqtt5_client_mock_test_fixture *test_context);
void aws_wait_for_stopped_lifecycle_event(struct aws_mqtt5_client_mock_test_fixture *test_context);

int aws_verify_received_packet_sequence(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    struct aws_mqtt5_mock_server_packet_record *expected_packets,
    size_t expected_packets_count);

int aws_mqtt5_mock_server_handle_connect_always_fail(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

void aws_mqtt5_wait_for_n_lifecycle_events(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    enum aws_mqtt5_client_lifecycle_event_type type,
    size_t count);

int aws_verify_reconnection_exponential_backoff_timestamps(struct aws_mqtt5_client_mock_test_fixture *test_fixture);

int aws_verify_client_state_sequence(
    struct aws_mqtt5_client_mock_test_fixture *test_context,
    enum aws_mqtt5_client_state *expected_states,
    size_t expected_states_count);

int aws_mqtt5_mock_server_handle_connect_always_succeed(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

int aws_mqtt5_mock_server_send_packet(
    struct aws_mqtt5_server_mock_connection_context *connection,
    enum aws_mqtt5_packet_type packet_type,
    void *packet);

int aws_mqtt5_mock_server_handle_connect_succeed_on_nth(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

int aws_mqtt5_mock_server_handle_publish_puback(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

int aws_mqtt5_mock_server_handle_publish_puback_and_forward(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

int aws_mqtt5_mock_server_handle_unsubscribe_unsuback_success(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

int aws_mqtt5_mock_server_send_packet(
    struct aws_mqtt5_server_mock_connection_context *connection,
    enum aws_mqtt5_packet_type packet_type,
    void *packet);

int aws_mqtt5_server_send_suback_on_subscribe(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

int aws_mqtt5_mock_server_handle_connect_honor_session_unconditional(
    void *packet,
    struct aws_mqtt5_server_mock_connection_context *connection,
    void *user_data);

void aws_wait_for_n_lifecycle_events(
    struct aws_mqtt5_client_mock_test_fixture *test_fixture,
    enum aws_mqtt5_client_lifecycle_event_type event_type,
    size_t expected_event_count);

extern const struct aws_string *g_default_client_id;

#define RECONNECT_TEST_MIN_BACKOFF 500
#define RECONNECT_TEST_MAX_BACKOFF 5000
#define RECONNECT_TEST_BACKOFF_RESET_DELAY 5000

#endif /* MQTT_MQTT5_TESTING_UTILS_H */
