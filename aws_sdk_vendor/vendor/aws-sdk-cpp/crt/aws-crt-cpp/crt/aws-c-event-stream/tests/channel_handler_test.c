/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/event-stream/event_stream_channel_handler.h>
#include <aws/testing/aws_test_harness.h>
#include <aws/testing/io_testing_channel.h>

struct test_data {
    struct aws_allocator *allocator;
    struct testing_channel testing_channel;
    struct aws_channel_handler *handler;
    aws_event_stream_channel_handler_on_message_received_fn *received_fn;
    void *user_data;
};

static struct test_data s_test_data;

static void s_fixture_on_message(struct aws_event_stream_message *message, int error_code, void *user_data) {
    struct test_data *test_data = user_data;
    test_data->received_fn(message, error_code, test_data->user_data);
}

static int s_fixture_setup(struct aws_allocator *allocator, void *ctx) {
    aws_event_stream_library_init(allocator);
    struct test_data *test_data = ctx;
    AWS_ZERO_STRUCT(*test_data);

    test_data->allocator = allocator;

    struct aws_testing_channel_options testing_channel_options = {
        .clock_fn = aws_high_res_clock_get_ticks,
    };
    ASSERT_SUCCESS(testing_channel_init(&test_data->testing_channel, allocator, &testing_channel_options));

    struct aws_channel_slot *slot = aws_channel_slot_new(test_data->testing_channel.channel);
    ASSERT_NOT_NULL(slot);
    ASSERT_SUCCESS(aws_channel_slot_insert_end(test_data->testing_channel.channel, slot));

    struct aws_event_stream_channel_handler_options options = {
        .initial_window_size = SIZE_MAX,
        .user_data = ctx,
        .on_message_received = s_fixture_on_message,
    };
    test_data->handler = aws_event_stream_channel_handler_new(allocator, &options);
    ASSERT_NOT_NULL(test_data->handler);
    ASSERT_SUCCESS(aws_channel_slot_set_handler(slot, test_data->handler));
    testing_channel_run_currently_queued_tasks(&test_data->testing_channel);

    return AWS_OP_SUCCESS;
}

static int s_fixture_shutdown(struct aws_allocator *allocator, int setup_result, void *ctx) {
    (void)allocator;
    struct test_data *test_data = ctx;

    if (!setup_result) {
        testing_channel_clean_up(&test_data->testing_channel);
    }

    aws_event_stream_library_clean_up();

    return AWS_OP_SUCCESS;
}

struct single_message_test_data {
    int last_error_code;
    struct aws_event_stream_message received_msg_cpy;
};

static void s_test_on_single_message(struct aws_event_stream_message *message, int error_code, void *user_data) {

    struct single_message_test_data *msg_test_data = user_data;
    msg_test_data->last_error_code = error_code;

    if (!error_code) {
        struct aws_byte_buf message_buf = aws_byte_buf_from_array(
            aws_event_stream_message_buffer(message), aws_event_stream_message_total_length(message));
        aws_event_stream_message_from_buffer_copy(
            &msg_test_data->received_msg_cpy, s_test_data.allocator, &message_buf);
    }
}

/* send various valid messages in serial to make sure the happy path of message parsing is correct. */
static int s_test_channel_handler_single_valid_messages_parse(struct aws_allocator *allocator, void *ctx) {
    struct test_data *test_data = ctx;

    struct single_message_test_data message_test_data;
    AWS_ZERO_STRUCT(message_test_data);

    test_data->received_fn = s_test_on_single_message;
    test_data->user_data = &message_test_data;

    uint8_t empty_message[] = {
        0x00,
        0x00,
        0x00,
        0x10,
        0x00,
        0x00,
        0x00,
        0x00,
        0x05,
        0xc2,
        0x48,
        0xeb,
        0x7d,
        0x98,
        0xc8,
        0xff,
    };

    struct aws_byte_cursor empty_message_cursor = aws_byte_cursor_from_array(empty_message, sizeof(empty_message));
    ASSERT_SUCCESS(testing_channel_push_read_data(&s_test_data.testing_channel, empty_message_cursor));
    ASSERT_UINT_EQUALS(AWS_OP_SUCCESS, message_test_data.last_error_code);
    ASSERT_UINT_EQUALS(0x00000010, aws_event_stream_message_total_length(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x00000000, aws_event_stream_message_headers_len(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x05c248eb, aws_event_stream_message_prelude_crc(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x7d98c8ff, aws_event_stream_message_message_crc(&message_test_data.received_msg_cpy));
    aws_event_stream_message_clean_up(&message_test_data.received_msg_cpy);
    AWS_ZERO_STRUCT(message_test_data);

    uint8_t no_headers_data[] = {
        0x00, 0x00, 0x00, 0x1D, 0x00, 0x00, 0x00, 0x00, 0xfd, 0x52, 0x8c, 0x5a, 0x7b, 0x27, 0x66,
        0x6f, 0x6f, 0x27, 0x3a, 0x27, 0x62, 0x61, 0x72, 0x27, 0x7d, 0xc3, 0x65, 0x39, 0x36,
    };

    struct aws_byte_cursor no_headers_cur = aws_byte_cursor_from_array(no_headers_data, sizeof(no_headers_data));
    ASSERT_SUCCESS(testing_channel_push_read_data(&s_test_data.testing_channel, no_headers_cur));
    ASSERT_UINT_EQUALS(AWS_OP_SUCCESS, message_test_data.last_error_code);
    ASSERT_UINT_EQUALS(0x0000001D, aws_event_stream_message_total_length(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x00000000, aws_event_stream_message_headers_len(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0xfd528c5a, aws_event_stream_message_prelude_crc(&message_test_data.received_msg_cpy));

    const char *expected_str = "{'foo':'bar'}";
    ASSERT_UINT_EQUALS(strlen(expected_str), aws_event_stream_message_payload_len(&message_test_data.received_msg_cpy));

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_str,
        strlen(expected_str),
        aws_event_stream_message_payload(&message_test_data.received_msg_cpy),
        aws_event_stream_message_payload_len(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0xc3653936, aws_event_stream_message_message_crc(&message_test_data.received_msg_cpy));

    aws_event_stream_message_clean_up(&message_test_data.received_msg_cpy);
    AWS_ZERO_STRUCT(message_test_data);

    uint8_t headers_test_data[] = {
        0x00, 0x00, 0x00, 0x3D, 0x00, 0x00, 0x00, 0x20, 0x07, 0xFD, 0x83, 0x96, 0x0C, 'c',  'o',  'n',
        't',  'e',  'n',  't',  '-',  't',  'y',  'p',  'e',  0x07, 0x00, 0x10, 'a',  'p',  'p',  'l',
        'i',  'c',  'a',  't',  'i',  'o',  'n',  '/',  'j',  's',  'o',  'n',  0x7b, 0x27, 0x66, 0x6f,
        0x6f, 0x27, 0x3a, 0x27, 0x62, 0x61, 0x72, 0x27, 0x7d, 0x8D, 0x9C, 0x08, 0xB1,
    };

    struct aws_byte_cursor headers_test_cur = aws_byte_cursor_from_array(headers_test_data, sizeof(headers_test_data));

    ASSERT_SUCCESS(testing_channel_push_read_data(&s_test_data.testing_channel, headers_test_cur));
    ASSERT_UINT_EQUALS(AWS_OP_SUCCESS, message_test_data.last_error_code);

    ASSERT_UINT_EQUALS(0x0000003D, aws_event_stream_message_total_length(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x00000020, aws_event_stream_message_headers_len(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x07FD8396, aws_event_stream_message_prelude_crc(&message_test_data.received_msg_cpy));

    ASSERT_UINT_EQUALS(strlen(expected_str), aws_event_stream_message_payload_len(&message_test_data.received_msg_cpy));

    ASSERT_BIN_ARRAYS_EQUALS(
        expected_str,
        strlen(expected_str),
        aws_event_stream_message_payload(&message_test_data.received_msg_cpy),
        aws_event_stream_message_payload_len(&message_test_data.received_msg_cpy));

    struct aws_array_list headers;
    ASSERT_SUCCESS(aws_event_stream_headers_list_init(&headers, allocator));

    ASSERT_SUCCESS(aws_event_stream_message_headers(&message_test_data.received_msg_cpy, &headers));
    ASSERT_UINT_EQUALS(1, headers.length, );
    struct aws_event_stream_header_value_pair header;
    ASSERT_SUCCESS(aws_array_list_front(&headers, &header));

    const char *content_type = "content-type";
    const char *content_type_value = "application/json";

    struct aws_byte_buf header_name_buf = aws_event_stream_header_name(&header);
    ASSERT_BIN_ARRAYS_EQUALS(content_type, strlen(content_type), header_name_buf.buffer, header_name_buf.len);

    struct aws_byte_buf header_value_buf = aws_event_stream_header_value_as_string(&header);

    ASSERT_BIN_ARRAYS_EQUALS(
        content_type_value, strlen(content_type_value), header_value_buf.buffer, header_value_buf.len);

    ASSERT_UINT_EQUALS(0x8D9C08B1, aws_event_stream_message_message_crc(&message_test_data.received_msg_cpy));

    aws_event_stream_headers_list_cleanup(&headers);
    aws_event_stream_message_clean_up(&message_test_data.received_msg_cpy);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_channel_handler_single_valid_messages_parse,
    s_fixture_setup,
    s_test_channel_handler_single_valid_messages_parse,
    s_fixture_shutdown,
    &s_test_data)

struct multiple_message_test_data {
    int last_error_code;
    struct aws_event_stream_message *received_msgs_cpy;
    size_t msg_count;
};

static void s_test_on_multiple_message(struct aws_event_stream_message *message, int error_code, void *user_data) {

    struct multiple_message_test_data *msg_test_data = user_data;
    msg_test_data->last_error_code = error_code;

    if (!error_code) {
        struct aws_byte_buf message_buf = aws_byte_buf_from_array(
            aws_event_stream_message_buffer(message), aws_event_stream_message_total_length(message));
        struct aws_event_stream_message msg_cpy;
        AWS_ZERO_STRUCT(msg_cpy);
        aws_event_stream_message_from_buffer_copy(&msg_cpy, s_test_data.allocator, &message_buf);
        msg_test_data->received_msgs_cpy[msg_test_data->msg_count++] = msg_cpy;
    }
}

/* send various valid messages as a batch to make sure the happy path of message parsing is correct. */
static int s_test_channel_handler_multiple_valid_messages_parse(struct aws_allocator *allocator, void *ctx) {
    struct test_data *test_data = ctx;

    uint8_t multi_message[] = {
        0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x05, 0xc2, 0x48, 0xeb, 0x7d, 0x98, 0xc8, 0xff,

        0x00, 0x00, 0x00, 0x1D, 0x00, 0x00, 0x00, 0x00, 0xfd, 0x52, 0x8c, 0x5a, 0x7b, 0x27, 0x66, 0x6f,
        0x6f, 0x27, 0x3a, 0x27, 0x62, 0x61, 0x72, 0x27, 0x7d, 0xc3, 0x65, 0x39, 0x36,

        0x00, 0x00, 0x00, 0x3D, 0x00, 0x00, 0x00, 0x20, 0x07, 0xFD, 0x83, 0x96, 0x0C, 'c',  'o',  'n',
        't',  'e',  'n',  't',  '-',  't',  'y',  'p',  'e',  0x07, 0x00, 0x10, 'a',  'p',  'p',  'l',
        'i',  'c',  'a',  't',  'i',  'o',  'n',  '/',  'j',  's',  'o',  'n',  0x7b, 0x27, 0x66, 0x6f,
        0x6f, 0x27, 0x3a, 0x27, 0x62, 0x61, 0x72, 0x27, 0x7d, 0x8D, 0x9C, 0x08, 0xB1,
    };

    /* fuzz the parser's boundary conditions */
    for (size_t fragment_size = 1; fragment_size <= sizeof(multi_message); ++fragment_size) {

        struct multiple_message_test_data message_test_data;
        AWS_ZERO_STRUCT(message_test_data);

        struct aws_event_stream_message messages[100];
        AWS_ZERO_ARRAY(messages);
        message_test_data.received_msgs_cpy = messages;

        test_data->received_fn = s_test_on_multiple_message;
        test_data->user_data = &message_test_data;

        size_t processed = 0;
        while (processed < sizeof(multi_message)) {
            size_t remaining = sizeof(multi_message) - processed;
            size_t to_copy = fragment_size < remaining ? fragment_size : remaining;
            struct aws_byte_cursor multi_message_cursor =
                aws_byte_cursor_from_array(multi_message + processed, to_copy);
            processed += to_copy;
            ASSERT_SUCCESS(testing_channel_push_read_data(&s_test_data.testing_channel, multi_message_cursor));
        }

        ASSERT_UINT_EQUALS(AWS_OP_SUCCESS, message_test_data.last_error_code);
        ASSERT_UINT_EQUALS(3, message_test_data.msg_count);

        ASSERT_UINT_EQUALS(0x00000010, aws_event_stream_message_total_length(&message_test_data.received_msgs_cpy[0]));
        ASSERT_UINT_EQUALS(0x00000000, aws_event_stream_message_headers_len(&message_test_data.received_msgs_cpy[0]));
        ASSERT_UINT_EQUALS(0x05c248eb, aws_event_stream_message_prelude_crc(&message_test_data.received_msgs_cpy[0]));
        ASSERT_UINT_EQUALS(0x7d98c8ff, aws_event_stream_message_message_crc(&message_test_data.received_msgs_cpy[0]));
        aws_event_stream_message_clean_up(&message_test_data.received_msgs_cpy[0]);

        ASSERT_UINT_EQUALS(0x0000001D, aws_event_stream_message_total_length(&message_test_data.received_msgs_cpy[1]));
        ASSERT_UINT_EQUALS(0x00000000, aws_event_stream_message_headers_len(&message_test_data.received_msgs_cpy[1]));
        ASSERT_UINT_EQUALS(0xfd528c5a, aws_event_stream_message_prelude_crc(&message_test_data.received_msgs_cpy[1]));

        const char *expected_str = "{'foo':'bar'}";
        ASSERT_UINT_EQUALS(
            strlen(expected_str), aws_event_stream_message_payload_len(&message_test_data.received_msgs_cpy[1]));

        ASSERT_BIN_ARRAYS_EQUALS(
            expected_str,
            strlen(expected_str),
            aws_event_stream_message_payload(&message_test_data.received_msgs_cpy[1]),
            aws_event_stream_message_payload_len(&message_test_data.received_msgs_cpy[1]));
        ASSERT_UINT_EQUALS(0xc3653936, aws_event_stream_message_message_crc(&message_test_data.received_msgs_cpy[1]));

        aws_event_stream_message_clean_up(&message_test_data.received_msgs_cpy[1]);

        ASSERT_UINT_EQUALS(0x0000003D, aws_event_stream_message_total_length(&message_test_data.received_msgs_cpy[2]));
        ASSERT_UINT_EQUALS(0x00000020, aws_event_stream_message_headers_len(&message_test_data.received_msgs_cpy[2]));
        ASSERT_UINT_EQUALS(0x07FD8396, aws_event_stream_message_prelude_crc(&message_test_data.received_msgs_cpy[2]));

        ASSERT_UINT_EQUALS(
            strlen(expected_str), aws_event_stream_message_payload_len(&message_test_data.received_msgs_cpy[2]));

        ASSERT_BIN_ARRAYS_EQUALS(
            expected_str,
            strlen(expected_str),
            aws_event_stream_message_payload(&message_test_data.received_msgs_cpy[2]),
            aws_event_stream_message_payload_len(&message_test_data.received_msgs_cpy[2]));

        struct aws_array_list headers;
        ASSERT_SUCCESS(aws_event_stream_headers_list_init(&headers, allocator));

        ASSERT_SUCCESS(aws_event_stream_message_headers(&message_test_data.received_msgs_cpy[2], &headers));
        ASSERT_UINT_EQUALS(1, headers.length, );
        struct aws_event_stream_header_value_pair header;
        ASSERT_SUCCESS(aws_array_list_front(&headers, &header));

        const char *content_type = "content-type";
        const char *content_type_value = "application/json";

        struct aws_byte_buf header_name_buf = aws_event_stream_header_name(&header);
        ASSERT_BIN_ARRAYS_EQUALS(content_type, strlen(content_type), header_name_buf.buffer, header_name_buf.len);

        struct aws_byte_buf header_value_buf = aws_event_stream_header_value_as_string(&header);

        ASSERT_BIN_ARRAYS_EQUALS(
            content_type_value, strlen(content_type_value), header_value_buf.buffer, header_value_buf.len);

        ASSERT_UINT_EQUALS(0x8D9C08B1, aws_event_stream_message_message_crc(&message_test_data.received_msgs_cpy[2]));

        aws_event_stream_headers_list_cleanup(&headers);
        aws_event_stream_message_clean_up(&message_test_data.received_msgs_cpy[2]);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_channel_handler_multiple_valid_messages_parse,
    s_fixture_setup,
    s_test_channel_handler_multiple_valid_messages_parse,
    s_fixture_shutdown,
    &s_test_data)

/* send various valid messages in serial to make sure the happy path of message parsing is correct. */
static int s_test_channel_handler_corrupted_crc_fails(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    struct test_data *test_data = ctx;

    struct single_message_test_data message_test_data;
    AWS_ZERO_STRUCT(message_test_data);

    test_data->received_fn = s_test_on_single_message;
    test_data->user_data = &message_test_data;

    /* altered the 9th byte to a single bit flip */
    uint8_t empty_message[] = {
        0x00,
        0x00,
        0x00,
        0x10,
        0x00,
        0x00,
        0x00,
        0x00,
        0x05,
        0xc3,
        0x48,
        0xeb,
        0x7d,
        0x98,
        0xc8,
        0xff,
    };

    struct aws_byte_cursor empty_message_cursor = aws_byte_cursor_from_array(empty_message, sizeof(empty_message));
    ASSERT_SUCCESS(testing_channel_push_read_data(&s_test_data.testing_channel, empty_message_cursor));
    ASSERT_UINT_EQUALS(AWS_ERROR_EVENT_STREAM_PRELUDE_CHECKSUM_FAILURE, message_test_data.last_error_code);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_channel_handler_corrupted_crc_fails,
    s_fixture_setup,
    s_test_channel_handler_corrupted_crc_fails,
    s_fixture_shutdown,
    &s_test_data)

/* send various valid messages in serial to make sure the happy path of message parsing is correct. */
static int s_test_channel_handler_large_msg_success(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;

    struct test_data *test_data = ctx;

    struct single_message_test_data message_test_data;
    AWS_ZERO_STRUCT(message_test_data);

    test_data->received_fn = s_test_on_single_message;
    test_data->user_data = &message_test_data;

    /* message length is 16MB + 1 byte. We used to have our stream message limit set at 16MB. Now this test validates
     * that we can send messages > 16MB */
    uint8_t empty_message[] = {
        0x01,
        0x00,
        0x00,
        0x01,
        0x00,
        0x00,
        0x00,
        0x00,
        0x94,
        0xE8,
        0xF6,
        0x47,
        0xC8,
        0x86,
        0xFB,
        0xA0,
    };

    struct aws_byte_cursor empty_message_cursor = aws_byte_cursor_from_array(empty_message, sizeof(empty_message));
    ASSERT_SUCCESS(testing_channel_push_read_data(&s_test_data.testing_channel, empty_message_cursor));
    ASSERT_UINT_EQUALS(AWS_OP_SUCCESS, message_test_data.last_error_code);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_channel_handler_large_msg_success,
    s_fixture_setup,
    s_test_channel_handler_large_msg_success,
    s_fixture_shutdown,
    &s_test_data)

static void s_on_message_written(struct aws_event_stream_message *message, int error_code, void *user_data) {
    struct single_message_test_data *msg_test_data = user_data;
    msg_test_data->last_error_code = error_code;

    if (!error_code) {
        struct aws_byte_buf message_buf = aws_byte_buf_from_array(
            aws_event_stream_message_buffer(message), aws_event_stream_message_total_length(message));
        aws_event_stream_message_from_buffer_copy(
            &msg_test_data->received_msg_cpy, s_test_data.allocator, &message_buf);
    }
}

static int s_test_channel_handler_write_message(struct aws_allocator *allocator, void *ctx) {
    struct test_data *test_data = ctx;

    struct single_message_test_data message_test_data;
    AWS_ZERO_STRUCT(message_test_data);

    uint8_t empty_message[] = {
        0x00,
        0x00,
        0x00,
        0x10,
        0x00,
        0x00,
        0x00,
        0x00,
        0x05,
        0xc2,
        0x48,
        0xeb,
        0x7d,
        0x98,
        0xc8,
        0xff,
    };

    struct aws_byte_buf empty_message_buf = aws_byte_buf_from_array(empty_message, sizeof(empty_message));
    struct aws_event_stream_message msg_to_send;
    AWS_ZERO_STRUCT(msg_to_send);
    ASSERT_SUCCESS(aws_event_stream_message_from_buffer(&msg_to_send, allocator, &empty_message_buf));
    ASSERT_SUCCESS(aws_event_stream_channel_handler_write_message(
        test_data->handler, &msg_to_send, s_on_message_written, &message_test_data));
    testing_channel_drain_queued_tasks(&s_test_data.testing_channel);
    ASSERT_UINT_EQUALS(0x00000010, aws_event_stream_message_total_length(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x00000000, aws_event_stream_message_headers_len(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x05c248eb, aws_event_stream_message_prelude_crc(&message_test_data.received_msg_cpy));
    ASSERT_UINT_EQUALS(0x7d98c8ff, aws_event_stream_message_message_crc(&message_test_data.received_msg_cpy));
    aws_event_stream_message_clean_up(&message_test_data.received_msg_cpy);
    aws_event_stream_message_clean_up(&msg_to_send);

    struct aws_linked_list *list = testing_channel_get_written_message_queue(&test_data->testing_channel);
    ASSERT_FALSE(aws_linked_list_empty(list));
    struct aws_linked_list_node *node = aws_linked_list_front(list);
    ASSERT_NOT_NULL(node);
    struct aws_io_message *message = AWS_CONTAINER_OF(node, struct aws_io_message, queueing_handle);
    struct aws_event_stream_message sent_msg;
    AWS_ZERO_STRUCT(sent_msg);

    ASSERT_SUCCESS(aws_event_stream_message_from_buffer(&sent_msg, allocator, &message->message_data));
    ASSERT_UINT_EQUALS(0x00000010, aws_event_stream_message_total_length(&sent_msg));
    ASSERT_UINT_EQUALS(0x00000000, aws_event_stream_message_headers_len(&sent_msg));
    ASSERT_UINT_EQUALS(0x05c248eb, aws_event_stream_message_prelude_crc(&sent_msg));
    ASSERT_UINT_EQUALS(0x7d98c8ff, aws_event_stream_message_message_crc(&sent_msg));
    aws_event_stream_message_clean_up(&sent_msg);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE_FIXTURE(
    test_channel_handler_write_message,
    s_fixture_setup,
    s_test_channel_handler_write_message,
    s_fixture_shutdown,
    &s_test_data)
