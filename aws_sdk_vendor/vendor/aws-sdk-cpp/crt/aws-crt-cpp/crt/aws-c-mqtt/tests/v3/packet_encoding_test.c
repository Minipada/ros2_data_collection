/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/mqtt/private/packets.h>

#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable : 4232) /* function pointer to dll symbol */
#endif

enum { S_BUFFER_SIZE = 128 };

struct packet_test_fixture;

/* Function type used to init and cleanup a fixture */
typedef int(packet_init_fn)(struct packet_test_fixture *);
/* Function used to encode a packet (this should be set to a function from packets.h) */
typedef int(packet_encode_fn)(struct aws_byte_buf *, void *);
/* Function used to decode a packet (this should be set to a function from packets.h) */
typedef int(packet_decode_fn)(struct aws_byte_cursor *, void *);
/* Function used to check if two packets are equal */
typedef bool(packet_eq_fn)(void *, void *, size_t);

/* Helper for comparing the fixed headers of packets */
static bool s_fixed_header_eq(struct aws_mqtt_fixed_header *l, struct aws_mqtt_fixed_header *r) {

    return l->packet_type == r->packet_type && l->flags == r->flags && l->remaining_length == r->remaining_length;
}

/* Default packet compare function, checks headers then memcmps the rest */
static bool s_packet_eq_default(void *a, void *b, size_t size) {

    static const size_t HEADER_SIZE = sizeof(struct aws_mqtt_fixed_header);

    return s_fixed_header_eq(a, b) &&
           memcmp((uint8_t *)a + HEADER_SIZE, (uint8_t *)b + HEADER_SIZE, size - HEADER_SIZE) == 0;
}

/* Contains all of the information required to run a packet's test case */
struct packet_test_fixture {
    enum aws_mqtt_packet_type type;
    size_t size;
    packet_init_fn *init;
    packet_encode_fn *encode;
    packet_decode_fn *decode;
    packet_init_fn *teardown;
    packet_eq_fn *equal;
    struct aws_allocator *allocator;

    void *in_packet;
    void *out_packet;
    struct aws_byte_buf buffer;
};

static int s_packet_test_before(struct aws_allocator *allocator, void *ctx) {

    struct packet_test_fixture *fixture = ctx;
    fixture->allocator = allocator;

    /* Setup the fixture */
    fixture->in_packet = aws_mem_acquire(allocator, fixture->size);
    ASSERT_NOT_NULL(fixture->in_packet);
    memset(fixture->in_packet, 0, fixture->size);

    fixture->out_packet = aws_mem_acquire(allocator, fixture->size);
    ASSERT_NOT_NULL(fixture->out_packet);
    memset(fixture->out_packet, 0, fixture->size);

    return AWS_OP_SUCCESS;
}

static int s_packet_test_run(struct aws_allocator *allocator, void *ctx) {

    struct packet_test_fixture *fixture = ctx;

    aws_byte_buf_init(&fixture->buffer, allocator, S_BUFFER_SIZE);

    /* Init the in_packet & buffer */
    ASSERT_SUCCESS(fixture->init(fixture));

    /* Encode */

    /* Create the output buffer */
    struct aws_byte_buf output_buffer;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buffer, allocator, S_BUFFER_SIZE));

    /* Encode the packet */
    ASSERT_SUCCESS(fixture->encode(&output_buffer, fixture->in_packet));

    /* Compare the buffers */
    ASSERT_BIN_ARRAYS_EQUALS(fixture->buffer.buffer, fixture->buffer.len, output_buffer.buffer, output_buffer.len);

    aws_byte_buf_clean_up(&output_buffer);

    /* Decode */

    /* Decode the buffer */
    struct aws_byte_cursor cursor = aws_byte_cursor_from_buf(&fixture->buffer);
    ASSERT_SUCCESS(fixture->decode(&cursor, fixture->out_packet));

    /* Compare the packets */
    if (fixture->equal) {
        ASSERT_TRUE(fixture->equal(fixture->out_packet, fixture->in_packet, fixture->size));
    } else {
        ASSERT_TRUE(s_packet_eq_default(fixture->out_packet, fixture->in_packet, fixture->size));
    }

    return AWS_OP_SUCCESS;
}

static int s_packet_test_after(struct aws_allocator *allocator, int setup_result, void *ctx) {
    (void)setup_result;

    struct packet_test_fixture *fixture = ctx;

    /* Tear down the packet & buffer */
    if (fixture->teardown) {
        fixture->teardown(fixture);
    }

    /* Tear down the fixture */
    aws_mem_release(allocator, fixture->in_packet);
    aws_mem_release(allocator, fixture->out_packet);
    aws_byte_buf_clean_up(&fixture->buffer);

    return AWS_OP_SUCCESS;
}

#define PACKET_TEST_NAME(e_type, t_name, s_name, i, t, e)                                                              \
    static struct packet_test_fixture mqtt_packet_##t_name##_fixture = {                                               \
        .type = AWS_MQTT_PACKET_##e_type,                                                                              \
        .size = sizeof(struct aws_mqtt_packet_##s_name),                                                               \
        .init = (i),                                                                                                   \
        .encode = (packet_encode_fn *)&aws_mqtt_packet_##s_name##_encode,                                              \
        .decode = (packet_decode_fn *)&aws_mqtt_packet_##s_name##_decode,                                              \
        .teardown = (t),                                                                                               \
        .equal = (e),                                                                                                  \
    };                                                                                                                 \
    AWS_TEST_CASE_FIXTURE(                                                                                             \
        mqtt_packet_##t_name,                                                                                          \
        s_packet_test_before,                                                                                          \
        s_packet_test_run,                                                                                             \
        s_packet_test_after,                                                                                           \
        &mqtt_packet_##t_name##_fixture)

#define PACKET_TEST(e_type, s_name, i, t, e) PACKET_TEST_NAME(e_type, s_name, s_name, i, t, e)

static uint8_t s_client_id[] = "Test Client ID";
enum { CLIENT_ID_LEN = sizeof(s_client_id) };
static uint8_t s_topic_name[] = "test/topic";
enum { TOPIC_NAME_LEN = sizeof(s_topic_name) };
static uint8_t s_payload[] = "This s_payload contains data. It is some good ol' fashioned data.";
enum { PAYLOAD_LEN = sizeof(s_payload) };
static uint8_t s_username[] = "admin";
enum { USERNAME_LEN = sizeof(s_username) };
static uint8_t s_password[] = "12345";
enum { PASSWORD_LEN = sizeof(s_password) };

/*****************************************************************************/
/* Ack                                                                       */

static int s_test_ack_init(struct packet_test_fixture *fixture) {

    /* Init buffer */
    uint8_t packet_id = (uint8_t)(fixture->type + 7);

    /* clang-format off */
    uint8_t header[] = {
        (uint8_t)(fixture->type << 4),  /* Packet type */
        2,                              /* Remaining length */
        0, packet_id,                   /* Packet identifier */
    };

    /* Init packet */
    switch (fixture->type) {
        case AWS_MQTT_PACKET_PUBACK:
            ASSERT_SUCCESS(aws_mqtt_packet_puback_init(fixture->in_packet, packet_id));
            break;
        case AWS_MQTT_PACKET_PUBREC:
            ASSERT_SUCCESS(aws_mqtt_packet_pubrec_init(fixture->in_packet, packet_id));
            break;
        case AWS_MQTT_PACKET_PUBREL:
            ASSERT_SUCCESS(aws_mqtt_packet_pubrel_init(fixture->in_packet, packet_id));
            /* if pubrel, bit 1 in flags must be set */
            header[0] |= 0x2;
            break;
        case AWS_MQTT_PACKET_PUBCOMP:
            ASSERT_SUCCESS(aws_mqtt_packet_pubcomp_init(fixture->in_packet, packet_id));
            break;
        case AWS_MQTT_PACKET_UNSUBACK:
            ASSERT_SUCCESS(aws_mqtt_packet_unsuback_init(fixture->in_packet, packet_id));
            break;
        default:
            AWS_ASSUME(false);
            break;
    }

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));

    return AWS_OP_SUCCESS;
}
#define PACKET_TEST_ACK(e_type, name)                   \
    PACKET_TEST_NAME(e_type, name, ack, &s_test_ack_init, NULL, NULL)
PACKET_TEST_ACK(PUBACK, puback)
PACKET_TEST_ACK(PUBREC, pubrec)
PACKET_TEST_ACK(PUBREL, pubrel)
PACKET_TEST_ACK(PUBCOMP, pubcomp)
PACKET_TEST_ACK(UNSUBACK, unsuback)
#undef PACKET_TEST_ACK

/*****************************************************************************/
/* Connect                                                                   */

static int s_test_connect_init(struct packet_test_fixture *fixture) {

    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_connect_init(
        fixture->in_packet,
        aws_byte_cursor_from_array(s_client_id, CLIENT_ID_LEN),
        false,
        0));

    /* Init buffer */
    /* clang-format off */
    uint8_t header[] = {
        AWS_MQTT_PACKET_CONNECT << 4,   /* Packet type */
        10 + 2 + CLIENT_ID_LEN,         /* Remaining length */
        0, 4, 'M', 'Q', 'T', 'T',       /* Protocol name */
        4,                              /* Protocol level */
        0,                              /* Connect Flags */
        0, 0,                           /* Keep alive */
    };
    /* clang-format on */

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));
    aws_byte_buf_write_u8(&fixture->buffer, 0);
    aws_byte_buf_write_u8(&fixture->buffer, CLIENT_ID_LEN);
    aws_byte_buf_write(&fixture->buffer, s_client_id, CLIENT_ID_LEN);

    return AWS_OP_SUCCESS;
}
static bool s_test_connect_eq(void *a, void *b, size_t size) {

    (void)size;

    struct aws_mqtt_packet_connect *l = a;
    struct aws_mqtt_packet_connect *r = b;

    return s_fixed_header_eq(&l->fixed_header, &r->fixed_header) && l->clean_session == r->clean_session &&
           l->has_will == r->has_will && l->will_qos == r->will_qos && l->will_retain == r->will_retain &&
           l->has_password == r->has_password && l->has_username == r->has_username &&
           l->keep_alive_timeout == r->keep_alive_timeout &&
           aws_byte_cursor_eq(&l->client_identifier, &r->client_identifier) &&
           aws_byte_cursor_eq(&l->will_topic, &r->will_topic) && aws_byte_cursor_eq(&l->username, &r->username) &&
           aws_byte_cursor_eq(&l->password, &r->password);
}
PACKET_TEST(CONNECT, connect, &s_test_connect_init, NULL, &s_test_connect_eq)

static int s_test_connect_will_init(struct packet_test_fixture *fixture) {
    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_connect_init(
        fixture->in_packet, aws_byte_cursor_from_array(s_client_id, CLIENT_ID_LEN), false, 0));
    ASSERT_SUCCESS(aws_mqtt_packet_connect_add_will(
        fixture->in_packet,
        aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN),
        AWS_MQTT_QOS_EXACTLY_ONCE,
        true /*retain*/,
        aws_byte_cursor_from_array(s_payload, PAYLOAD_LEN)));

    /* Init buffer */
    /* clang-format off */
    uint8_t header[] = {
        AWS_MQTT_PACKET_CONNECT << 4,   /* Packet type */
        10 + (2 + CLIENT_ID_LEN) + (2 + TOPIC_NAME_LEN) + (2 + PAYLOAD_LEN), /* Remaining length */
        0, 4, 'M', 'Q', 'T', 'T',       /* Protocol name */
        4,                              /* Protocol level */
                                        /* Connect Flags: */
        (1 << 2)                        /*   Will flag, bit 2 */
        | (AWS_MQTT_QOS_EXACTLY_ONCE << 3)/* Will QoS, bits 4-3 */
        | (1 << 5),                     /*   Will Retain, bit 5 */

        0, 0,                           /* Keep alive */
    };
    /* clang-format on */

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));
    /* client identifier */
    aws_byte_buf_write_be16(&fixture->buffer, CLIENT_ID_LEN);
    aws_byte_buf_write(&fixture->buffer, s_client_id, CLIENT_ID_LEN);
    /* will topic */
    aws_byte_buf_write_be16(&fixture->buffer, TOPIC_NAME_LEN);
    aws_byte_buf_write(&fixture->buffer, s_topic_name, TOPIC_NAME_LEN);
    /* will payload */
    aws_byte_buf_write_be16(&fixture->buffer, PAYLOAD_LEN);
    aws_byte_buf_write(&fixture->buffer, s_payload, PAYLOAD_LEN);

    return AWS_OP_SUCCESS;
}
PACKET_TEST_NAME(CONNECT, connect_will, connect, &s_test_connect_will_init, NULL, &s_test_connect_eq)

static uint8_t s_empty_payload[] = "";
enum { EMPTY_PAYLOAD_LEN = 0 };

static int s_test_connect_empty_payload_will_init(struct packet_test_fixture *fixture) {
    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_connect_init(
        fixture->in_packet, aws_byte_cursor_from_array(s_client_id, CLIENT_ID_LEN), false, 0));
    ASSERT_SUCCESS(aws_mqtt_packet_connect_add_will(
        fixture->in_packet,
        aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN),
        AWS_MQTT_QOS_EXACTLY_ONCE,
        true /*retain*/,
        aws_byte_cursor_from_array(s_empty_payload, EMPTY_PAYLOAD_LEN)));

    /* Init buffer */
    /* clang-format off */
    uint8_t header[] = {
        AWS_MQTT_PACKET_CONNECT << 4,   /* Packet type */
        10 + (2 + CLIENT_ID_LEN) + (2 + TOPIC_NAME_LEN) + (2 + EMPTY_PAYLOAD_LEN), /* Remaining length */
        0, 4, 'M', 'Q', 'T', 'T',       /* Protocol name */
        4,                              /* Protocol level */
                                        /* Connect Flags: */
        (1 << 2)                        /*   Will flag, bit 2 */
        | (AWS_MQTT_QOS_EXACTLY_ONCE << 3)/* Will QoS, bits 4-3 */
        | (1 << 5),                     /*   Will Retain, bit 5 */

        0, 0,                           /* Keep alive */
    };
    /* clang-format on */

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));
    /* client identifier */
    aws_byte_buf_write_be16(&fixture->buffer, CLIENT_ID_LEN);
    aws_byte_buf_write(&fixture->buffer, s_client_id, CLIENT_ID_LEN);
    /* will topic */
    aws_byte_buf_write_be16(&fixture->buffer, TOPIC_NAME_LEN);
    aws_byte_buf_write(&fixture->buffer, s_topic_name, TOPIC_NAME_LEN);
    /* will payload */
    aws_byte_buf_write_be16(&fixture->buffer, EMPTY_PAYLOAD_LEN);
    aws_byte_buf_write(&fixture->buffer, s_empty_payload, EMPTY_PAYLOAD_LEN);

    return AWS_OP_SUCCESS;
}
PACKET_TEST_NAME(
    CONNECT,
    connect_empty_payload_will,
    connect,
    &s_test_connect_empty_payload_will_init,
    NULL,
    &s_test_connect_eq)

static int s_test_connect_password_init(struct packet_test_fixture *fixture) {
    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_connect_init(
        fixture->in_packet, aws_byte_cursor_from_array(s_client_id, CLIENT_ID_LEN), false, 0xBEEF));
    ASSERT_SUCCESS(aws_mqtt_packet_connect_add_credentials(
        fixture->in_packet,
        aws_byte_cursor_from_array(s_username, USERNAME_LEN),
        aws_byte_cursor_from_array(s_password, PASSWORD_LEN)));

    /* Init buffer */
    /* clang-format off */
    uint8_t header[] = {
        AWS_MQTT_PACKET_CONNECT << 4,   /* Packet type */
        10 + (2 + CLIENT_ID_LEN) + (2 + USERNAME_LEN) + (2 + PASSWORD_LEN), /* Remaining length */
        0, 4, 'M', 'Q', 'T', 'T',       /* Protocol name */
        4,                              /* Protocol level */
        (1 << 7) | (1 << 6),            /* Connect Flags: username bit 7, password bit 6 */
        0xBE, 0xEF,                     /* Keep alive */
    };
    /* clang-format on */

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));

    /* client identifier */
    aws_byte_buf_write_be16(&fixture->buffer, CLIENT_ID_LEN);
    aws_byte_buf_write(&fixture->buffer, s_client_id, CLIENT_ID_LEN);
    /* username */
    aws_byte_buf_write_be16(&fixture->buffer, USERNAME_LEN);
    aws_byte_buf_write(&fixture->buffer, s_username, USERNAME_LEN);
    /* password */
    aws_byte_buf_write_be16(&fixture->buffer, PASSWORD_LEN);
    aws_byte_buf_write(&fixture->buffer, s_password, PASSWORD_LEN);

    return AWS_OP_SUCCESS;
}
PACKET_TEST_NAME(CONNECT, connect_password, connect, &s_test_connect_password_init, NULL, &s_test_connect_eq)

static int s_test_connect_all_init(struct packet_test_fixture *fixture) {
    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_connect_init(
        fixture->in_packet, aws_byte_cursor_from_array(s_client_id, CLIENT_ID_LEN), false, 0));
    ASSERT_SUCCESS(aws_mqtt_packet_connect_add_will(
        fixture->in_packet,
        aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN),
        AWS_MQTT_QOS_EXACTLY_ONCE,
        true /*retain*/,
        aws_byte_cursor_from_array(s_payload, PAYLOAD_LEN)));
    ASSERT_SUCCESS(aws_mqtt_packet_connect_add_credentials(
        fixture->in_packet,
        aws_byte_cursor_from_array(s_username, USERNAME_LEN),
        aws_byte_cursor_from_array(s_password, PASSWORD_LEN)));

    /* Init buffer */
    /* clang-format off */
    uint8_t header[] = {
        AWS_MQTT_PACKET_CONNECT << 4,   /* Packet type */
        10 + (2 + CLIENT_ID_LEN) + (2 + TOPIC_NAME_LEN) + (2 + PAYLOAD_LEN) + (2 + USERNAME_LEN) + (2 + PASSWORD_LEN), /* Remaining length */
        0, 4, 'M', 'Q', 'T', 'T',       /* Protocol name */
        4,                              /* Protocol level */
                                        /* Connect Flags: */
        (1 << 2)                        /*   Will flag, bit 2 */
        | (AWS_MQTT_QOS_EXACTLY_ONCE << 3)/* Will QoS, bits 4-3 */
        | (1 << 5)                      /*   Will Retain, bit 5 */
        | (1 << 7) | (1 << 6),            /* username bit 7, password bit 6 */
        0, 0,                           /* Keep alive */
    };
    /* clang-format on */

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));
    /* client identifier */
    aws_byte_buf_write_be16(&fixture->buffer, CLIENT_ID_LEN);
    aws_byte_buf_write(&fixture->buffer, s_client_id, CLIENT_ID_LEN);
    /* will topic */
    aws_byte_buf_write_be16(&fixture->buffer, TOPIC_NAME_LEN);
    aws_byte_buf_write(&fixture->buffer, s_topic_name, TOPIC_NAME_LEN);
    /* will payload */
    aws_byte_buf_write_be16(&fixture->buffer, PAYLOAD_LEN);
    aws_byte_buf_write(&fixture->buffer, s_payload, PAYLOAD_LEN);
    /* username */
    aws_byte_buf_write_be16(&fixture->buffer, USERNAME_LEN);
    aws_byte_buf_write(&fixture->buffer, s_username, USERNAME_LEN);
    /* password */
    aws_byte_buf_write_be16(&fixture->buffer, PASSWORD_LEN);
    aws_byte_buf_write(&fixture->buffer, s_password, PASSWORD_LEN);

    return AWS_OP_SUCCESS;
}
PACKET_TEST_NAME(CONNECT, connect_all, connect, &s_test_connect_all_init, NULL, &s_test_connect_eq)

/*****************************************************************************/
/* Connack                                                                   */

static int s_test_connack_init(struct packet_test_fixture *fixture) {

    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_connack_init(fixture->in_packet, true, AWS_MQTT_CONNECT_ACCEPTED));

    /* Init buffer */
    /* clang-format off */
    uint8_t header[] = {
        AWS_MQTT_PACKET_CONNACK << 4,   /* Packet type */
        2,                              /* Remaining length */
        1,                              /* Acknowledge flags */
        AWS_MQTT_CONNECT_ACCEPTED,      /* Return code */
    };
    /* clang-format on */

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));

    return AWS_OP_SUCCESS;
}
PACKET_TEST(CONNACK, connack, &s_test_connack_init, NULL, NULL)

/*****************************************************************************/
/* Publish                                                                   */

static int s_test_publish_qos0_dup_init(struct packet_test_fixture *fixture) {

    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_publish_init(
        fixture->in_packet,
        false /* retain */,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        true /* dup */,
        aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN),
        0,
        aws_byte_cursor_from_array(s_payload, PAYLOAD_LEN)));

    /* Init buffer */
    /* clang-format off */
    aws_byte_buf_write_u8(
        &fixture->buffer,
        (AWS_MQTT_PACKET_PUBLISH << 4) /* Packet type bits 7-4 */
        | (1 << 3) /* DUP bit 3 */
        | (AWS_MQTT_QOS_AT_MOST_ONCE << 1) /* QoS bits 2-1 */
        | 0 /* RETAIN bit 0 */);
    aws_byte_buf_write_u8(
        &fixture->buffer, 2 + TOPIC_NAME_LEN + PAYLOAD_LEN); /* Remaining length */
    aws_byte_buf_write_u8(
        &fixture->buffer, 0); /* Topic name len byte 1 */
    aws_byte_buf_write_u8(
        &fixture->buffer, TOPIC_NAME_LEN); /* Topic name len byte 2 */
    aws_byte_buf_write(
        &fixture->buffer, s_topic_name, TOPIC_NAME_LEN); /* Topic name */
    aws_byte_buf_write(
        &fixture->buffer, s_payload, PAYLOAD_LEN); /* payload */
    /* clang-format on */

    return AWS_OP_SUCCESS;
}

static int s_test_publish_qos2_retain_init(struct packet_test_fixture *fixture) {

    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_publish_init(
        fixture->in_packet,
        true /* retain */,
        AWS_MQTT_QOS_EXACTLY_ONCE,
        false /* dup */,
        aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN),
        7,
        aws_byte_cursor_from_array(s_payload, PAYLOAD_LEN)));

    /* Init buffer */
    /* clang-format off */
    aws_byte_buf_write_u8(
        &fixture->buffer,
        (AWS_MQTT_PACKET_PUBLISH << 4) /* Packet type bits 7-4 */
        | (0 << 3) /* DUP bit 3 */
        | (AWS_MQTT_QOS_EXACTLY_ONCE << 1) /* QoS bits 2-1 */
        | 1 /* RETAIN bit 0 */);
    aws_byte_buf_write_u8(
        &fixture->buffer, 4 + TOPIC_NAME_LEN + PAYLOAD_LEN); /* Remaining length */
    aws_byte_buf_write_u8(
        &fixture->buffer, 0); /* Topic name len byte 1 */
    aws_byte_buf_write_u8(
        &fixture->buffer, TOPIC_NAME_LEN); /* Topic name len byte 2 */
    aws_byte_buf_write(
        &fixture->buffer, s_topic_name, TOPIC_NAME_LEN); /* Topic name */
    aws_byte_buf_write_u8(
        &fixture->buffer, 0);       /* Packet id byte 1 */
    aws_byte_buf_write_u8(
        &fixture->buffer, 7);       /* Packet id byte 2 */
    aws_byte_buf_write(
        &fixture->buffer, s_payload, PAYLOAD_LEN); /* payload */
    /* clang-format on */

    return AWS_OP_SUCCESS;
}

static int s_test_publish_empty_payload_init(struct packet_test_fixture *fixture) {

    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_publish_init(
        fixture->in_packet,
        false /* retain */,
        AWS_MQTT_QOS_AT_MOST_ONCE,
        true /* dup */,
        aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN),
        0,
        aws_byte_cursor_from_array(s_empty_payload, EMPTY_PAYLOAD_LEN)));

    /* Init buffer */
    /* clang-format off */
    aws_byte_buf_write_u8(
        &fixture->buffer,
        (AWS_MQTT_PACKET_PUBLISH << 4) /* Packet type bits 7-4 */
        | (1 << 3) /* DUP bit 3 */
        | (AWS_MQTT_QOS_AT_MOST_ONCE << 1) /* QoS bits 2-1 */
        | 0 /* RETAIN bit 0 */);
    aws_byte_buf_write_u8(
        &fixture->buffer, 2 + TOPIC_NAME_LEN + EMPTY_PAYLOAD_LEN); /* Remaining length */
    aws_byte_buf_write_u8(
        &fixture->buffer, 0); /* Topic name len byte 1 */
    aws_byte_buf_write_u8(
        &fixture->buffer, TOPIC_NAME_LEN); /* Topic name len byte 2 */
    aws_byte_buf_write(
        &fixture->buffer, s_topic_name, TOPIC_NAME_LEN); /* Topic name */
    aws_byte_buf_write(
        &fixture->buffer, s_empty_payload, EMPTY_PAYLOAD_LEN); /* payload */
    /* clang-format on */

    return AWS_OP_SUCCESS;
}

static bool s_test_publish_eq(void *a, void *b, size_t size) {

    (void)size;

    struct aws_mqtt_packet_publish *l = a;
    struct aws_mqtt_packet_publish *r = b;

    return s_fixed_header_eq(&l->fixed_header, &r->fixed_header) && l->packet_identifier == r->packet_identifier &&
           aws_byte_cursor_eq(&l->topic_name, &r->topic_name) && aws_byte_cursor_eq(&l->payload, &r->payload);
}
PACKET_TEST_NAME(PUBLISH, publish_qos0_dup, publish, &s_test_publish_qos0_dup_init, NULL, &s_test_publish_eq)
PACKET_TEST_NAME(PUBLISH, publish_qos2_retain, publish, &s_test_publish_qos2_retain_init, NULL, &s_test_publish_eq)
PACKET_TEST_NAME(PUBLISH, publish_empty_payload, publish, &s_test_publish_empty_payload_init, NULL, &s_test_publish_eq)

/*****************************************************************************/
/* Subscribe                                                                 */

static int s_test_subscribe_init(struct packet_test_fixture *fixture) {

    /* Init packets */
    ASSERT_SUCCESS(aws_mqtt_packet_subscribe_init(fixture->in_packet, fixture->allocator, 7));
    ASSERT_SUCCESS(aws_mqtt_packet_subscribe_init(fixture->out_packet, fixture->allocator, 0));

    ASSERT_SUCCESS(aws_mqtt_packet_subscribe_add_topic(
        fixture->in_packet, aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN), AWS_MQTT_QOS_EXACTLY_ONCE));

    /* Init buffer */ /* clang-format off */
    aws_byte_buf_write_u8(
        &fixture->buffer, (AWS_MQTT_PACKET_SUBSCRIBE << 4) | 0x2); /* Packet type & flags */
    aws_byte_buf_write_u8(
        &fixture->buffer, 4 + TOPIC_NAME_LEN + 1); /* Remaining length */
    aws_byte_buf_write_u8(
        &fixture->buffer, 0);
    aws_byte_buf_write_u8(
        &fixture->buffer, 7);
    aws_byte_buf_write_u8(
        &fixture->buffer, 0);
    aws_byte_buf_write_u8(
        &fixture->buffer, TOPIC_NAME_LEN);
    aws_byte_buf_write(
        &fixture->buffer, s_topic_name, TOPIC_NAME_LEN);
    aws_byte_buf_write_u8(
        &fixture->buffer, AWS_MQTT_QOS_EXACTLY_ONCE);
    /* clang-format on */

    return AWS_OP_SUCCESS;
}
static int s_test_subscribe_clean_up(struct packet_test_fixture *fixture) {

    aws_mqtt_packet_subscribe_clean_up(fixture->in_packet);
    aws_mqtt_packet_subscribe_clean_up(fixture->out_packet);

    return AWS_OP_SUCCESS;
}
static bool s_test_subscribe_eq(void *a, void *b, size_t size) {

    (void)size;

    struct aws_mqtt_packet_subscribe *l = a;
    struct aws_mqtt_packet_subscribe *r = b;

    if (!s_fixed_header_eq(&l->fixed_header, &r->fixed_header) || l->packet_identifier != r->packet_identifier) {
        return false;
    }

    const size_t length = aws_array_list_length(&l->topic_filters);
    if (length != aws_array_list_length(&r->topic_filters)) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        struct aws_mqtt_subscription *lt = NULL;
        aws_array_list_get_at_ptr(&l->topic_filters, (void **)&lt, i);
        struct aws_mqtt_subscription *rt = NULL;
        aws_array_list_get_at_ptr(&r->topic_filters, (void **)&rt, i);
        AWS_ASSUME(lt && rt);

        if (lt->qos != rt->qos) {
            return false;
        }
        if (!aws_byte_cursor_eq(&lt->topic_filter, &rt->topic_filter)) {
            return false;
        }
    }

    return true;
}
PACKET_TEST(SUBSCRIBE, subscribe, &s_test_subscribe_init, &s_test_subscribe_clean_up, &s_test_subscribe_eq)

/*****************************************************************************/
/* Suback                                                                */

static int s_test_suback_init(struct packet_test_fixture *fixture) {

    /* Init packets */
    ASSERT_SUCCESS(aws_mqtt_packet_suback_init(fixture->in_packet, fixture->allocator, 7));
    ASSERT_SUCCESS(aws_mqtt_packet_suback_init(fixture->out_packet, fixture->allocator, 0));

    ASSERT_SUCCESS(aws_mqtt_packet_suback_add_return_code(fixture->in_packet, AWS_MQTT_QOS_EXACTLY_ONCE));
    ASSERT_SUCCESS(aws_mqtt_packet_suback_add_return_code(fixture->in_packet, AWS_MQTT_QOS_FAILURE));

    /* Init buffer */ /* clang-format off */
    aws_byte_buf_write_u8(
        &fixture->buffer, (AWS_MQTT_PACKET_SUBACK << 4) | 0x0); /* Packet type & flags */
    aws_byte_buf_write_u8(
        &fixture->buffer, 2/* variable header */ + 2/* payload */); /* Remaining length */
    aws_byte_buf_write_u8(
        &fixture->buffer, 0);
    aws_byte_buf_write_u8(
        &fixture->buffer, 7);
    aws_byte_buf_write_u8(
        &fixture->buffer, AWS_MQTT_QOS_EXACTLY_ONCE); /* Payload */
    aws_byte_buf_write_u8(
        &fixture->buffer, AWS_MQTT_QOS_FAILURE); /* Payload */
    /* clang-format on */

    return AWS_OP_SUCCESS;
}
static int s_test_suback_clean_up(struct packet_test_fixture *fixture) {

    aws_mqtt_packet_suback_clean_up(fixture->in_packet);
    aws_mqtt_packet_suback_clean_up(fixture->out_packet);

    return AWS_OP_SUCCESS;
}
static bool s_test_suback_eq(void *a, void *b, size_t size) {

    (void)size;

    struct aws_mqtt_packet_suback *l = a;
    struct aws_mqtt_packet_suback *r = b;

    if (!s_fixed_header_eq(&l->fixed_header, &r->fixed_header) || l->packet_identifier != r->packet_identifier) {
        return false;
    }

    const size_t length = aws_array_list_length(&l->return_codes);
    if (length != aws_array_list_length(&r->return_codes)) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        uint8_t lt = 0;
        aws_array_list_get_at(&l->return_codes, (void *)&lt, i);
        uint8_t rt = 0;
        aws_array_list_get_at(&r->return_codes, (void *)&rt, i);
        AWS_ASSUME(lt && rt);
    }

    return true;
}
PACKET_TEST(SUBACK, suback, &s_test_suback_init, &s_test_suback_clean_up, &s_test_suback_eq)

/*****************************************************************************/
/* Unsubscribe                                                               */

static int s_test_unsubscribe_init(struct packet_test_fixture *fixture) {

    /* Init packet */
    ASSERT_SUCCESS(aws_mqtt_packet_unsubscribe_init(fixture->in_packet, fixture->allocator, 7));
    ASSERT_SUCCESS(aws_mqtt_packet_unsubscribe_init(fixture->out_packet, fixture->allocator, 0));

    ASSERT_SUCCESS(aws_mqtt_packet_unsubscribe_add_topic(
        fixture->in_packet, aws_byte_cursor_from_array(s_topic_name, TOPIC_NAME_LEN)));

    /* Init buffer */
    /* clang-format off */
    aws_byte_buf_write_u8(
        &fixture->buffer, (AWS_MQTT_PACKET_UNSUBSCRIBE << 4) | 0x2); /* Packet type & flags */
    aws_byte_buf_write_u8(
        &fixture->buffer, 4 + TOPIC_NAME_LEN); /* Remaining length */
    aws_byte_buf_write_u8(
        &fixture->buffer, 0);
    aws_byte_buf_write_u8(
        &fixture->buffer, 7);
    aws_byte_buf_write_u8(
        &fixture->buffer, 0);
    aws_byte_buf_write_u8(
        &fixture->buffer, TOPIC_NAME_LEN);
    aws_byte_buf_write(
        &fixture->buffer, s_topic_name, TOPIC_NAME_LEN);
    /* clang-format on */

    return AWS_OP_SUCCESS;
}
static int s_test_unsubscribe_clean_up(struct packet_test_fixture *fixture) {

    aws_mqtt_packet_unsubscribe_clean_up(fixture->in_packet);
    aws_mqtt_packet_unsubscribe_clean_up(fixture->out_packet);

    return AWS_OP_SUCCESS;
}
static bool s_test_unsubscribe_eq(void *a, void *b, size_t size) {

    (void)size;

    struct aws_mqtt_packet_unsubscribe *l = a;
    struct aws_mqtt_packet_unsubscribe *r = b;

    if (!s_fixed_header_eq(&l->fixed_header, &r->fixed_header) || l->packet_identifier != r->packet_identifier) {
        return false;
    }

    const size_t length = aws_array_list_length(&l->topic_filters);
    if (length != aws_array_list_length(&r->topic_filters)) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        struct aws_byte_cursor *lt = NULL;
        aws_array_list_get_at_ptr(&l->topic_filters, (void **)&lt, i);
        struct aws_byte_cursor *rt = NULL;
        aws_array_list_get_at_ptr(&r->topic_filters, (void **)&rt, i);
        AWS_ASSUME(lt && rt);

        if (!aws_byte_cursor_eq(lt, rt)) {
            return false;
        }
    }

    return true;
}
PACKET_TEST(UNSUBSCRIBE, unsubscribe, &s_test_unsubscribe_init, &s_test_unsubscribe_clean_up, &s_test_unsubscribe_eq)

/*****************************************************************************/
/* Connection                                                                */

static int s_test_connection_init(struct packet_test_fixture *fixture) {

    /* Init packet */
    switch (fixture->type) {
        case AWS_MQTT_PACKET_PINGREQ:
            ASSERT_SUCCESS(aws_mqtt_packet_pingreq_init(fixture->in_packet));
            break;
        case AWS_MQTT_PACKET_PINGRESP:
            ASSERT_SUCCESS(aws_mqtt_packet_pingresp_init(fixture->in_packet));
            break;
        case AWS_MQTT_PACKET_DISCONNECT:
            ASSERT_SUCCESS(aws_mqtt_packet_disconnect_init(fixture->in_packet));
            break;
        default:
            AWS_FATAL_ASSERT(false);
            break;
    }

    /* Init buffer */
    /* clang-format off */
    uint8_t header[] = {
        (uint8_t)(fixture->type << 4),  /* Packet type */
        0,                              /* Remaining length */
    };
    /* clang-format on */

    aws_byte_buf_write(&fixture->buffer, header, sizeof(header));

    return AWS_OP_SUCCESS;
}
#define PACKET_TEST_CONNETION(e_type, name)                                                                            \
    PACKET_TEST_NAME(e_type, name, connection, &s_test_connection_init, NULL, NULL)
PACKET_TEST_CONNETION(PINGREQ, pingreq)
PACKET_TEST_CONNETION(PINGRESP, pingresp)
PACKET_TEST_CONNETION(DISCONNECT, disconnect)
#undef PACKET_TEST_CONNETION

static int s_mqtt_packet_connack_decode_failure_reserved_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_buf encoded_packet;
    aws_byte_buf_init(&encoded_packet, allocator, 1024);

    struct aws_mqtt_packet_connack connack;
    ASSERT_SUCCESS(aws_mqtt_packet_connack_init(&connack, true, AWS_MQTT_CONNECT_SERVER_UNAVAILABLE));

    ASSERT_SUCCESS(aws_mqtt_packet_connack_encode(&encoded_packet, &connack));

    struct aws_byte_cursor decode_cursor = aws_byte_cursor_from_buf(&encoded_packet);
    struct aws_mqtt_packet_connack decoded_connack;
    ASSERT_SUCCESS(aws_mqtt_packet_connack_decode(&decode_cursor, &decoded_connack));

    /* mess up the fixed header reserved bits */
    encoded_packet.buffer[0] |= 0x01;

    decode_cursor = aws_byte_cursor_from_buf(&encoded_packet);
    ASSERT_FAILS(aws_mqtt_packet_connack_decode(&decode_cursor, &decoded_connack));

    aws_byte_buf_clean_up(&encoded_packet);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_packet_connack_decode_failure_reserved, s_mqtt_packet_connack_decode_failure_reserved_fn)

static int s_mqtt_packet_ack_decode_failure_reserved_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_buf encoded_packet;
    aws_byte_buf_init(&encoded_packet, allocator, 1024);

    struct aws_mqtt_packet_ack puback;
    ASSERT_SUCCESS(aws_mqtt_packet_puback_init(&puback, 5));

    ASSERT_SUCCESS(aws_mqtt_packet_ack_encode(&encoded_packet, &puback));

    struct aws_byte_cursor decode_cursor = aws_byte_cursor_from_buf(&encoded_packet);
    struct aws_mqtt_packet_ack decoded_ack;
    ASSERT_SUCCESS(aws_mqtt_packet_ack_decode(&decode_cursor, &decoded_ack));

    /* mess up the fixed header reserved bits */
    encoded_packet.buffer[0] |= 0x0F;

    decode_cursor = aws_byte_cursor_from_buf(&encoded_packet);
    ASSERT_FAILS(aws_mqtt_packet_ack_decode(&decode_cursor, &decoded_ack));

    aws_byte_buf_clean_up(&encoded_packet);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_packet_ack_decode_failure_reserved, s_mqtt_packet_ack_decode_failure_reserved_fn)

static int s_mqtt_packet_pingresp_decode_failure_reserved_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    struct aws_byte_buf encoded_packet;
    aws_byte_buf_init(&encoded_packet, allocator, 1024);

    struct aws_mqtt_packet_connection pingresp;
    ASSERT_SUCCESS(aws_mqtt_packet_pingresp_init(&pingresp));

    ASSERT_SUCCESS(aws_mqtt_packet_connection_encode(&encoded_packet, &pingresp));

    struct aws_byte_cursor decode_cursor = aws_byte_cursor_from_buf(&encoded_packet);
    struct aws_mqtt_packet_connection decoded_pingresp;
    ASSERT_SUCCESS(aws_mqtt_packet_connection_decode(&decode_cursor, &decoded_pingresp));

    /* mess up the fixed header reserved bits */
    encoded_packet.buffer[0] |= 0x08;

    decode_cursor = aws_byte_cursor_from_buf(&encoded_packet);
    ASSERT_FAILS(aws_mqtt_packet_connection_decode(&decode_cursor, &decoded_pingresp));

    aws_byte_buf_clean_up(&encoded_packet);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_packet_pingresp_decode_failure_reserved, s_mqtt_packet_pingresp_decode_failure_reserved_fn)

#ifdef _MSC_VER
#    pragma warning(pop)
#endif
