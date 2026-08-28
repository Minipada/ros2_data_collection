/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/encoding.h>
#include <aws/mqtt/mqtt.h>

#include <aws/testing/aws_test_harness.h>

struct utf8_example {
    const char *name;
    struct aws_byte_cursor text;
};

static struct utf8_example s_valid_mqtt_utf8_examples[] = {
    {
        .name = "1 letter",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("a"),
    },
    {
        .name = "Several ascii letters",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("ascii word"),
    },
    {
        .name = "empty string",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(""),
    },
    {
        .name = "2 byte codepoint",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xC2\xA3"),
    },
    {
        .name = "3 byte codepoint",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xE2\x82\xAC"),
    },
    {
        .name = "4 byte codepoint",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xF0\x90\x8D\x88"),
    },
    {
        .name = "A variety of different length codepoints",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL(
            "\xF0\x90\x8D\x88\xE2\x82\xAC\xC2\xA3\x24\xC2\xA3\xE2\x82\xAC\xF0\x90\x8D\x88"),
    },
    {
        .name = "UTF8 BOM",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEF\xBB\xBF"),
    },
    {
        .name = "UTF8 BOM plus extra",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEF\xBB\xBF\x24\xC2\xA3"),
    },
    {
        .name = "First possible 3 byte codepoint",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xE0\xA0\x80"),
    },
    {
        .name = "First possible 4 byte codepoint",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xF0\x90\x80\x80"),
    },
    {
        .name = "Last possible 2 byte codepoint",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xDF\xBF"),
    },
    {
        .name = "Last valid codepoint before prohibited range U+D800 - U+DFFF",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xED\x9F\xBF"),
    },
    {
        .name = "Next valid codepoint after prohibited range U+D800 - U+DFFF",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEE\x80\x80"),
    },
    {
        .name = "Boundary condition",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEF\xBF\xBD"),
    },
    {
        .name = "Boundary condition",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xF4\x90\x80\x80"),
    },
};

static struct utf8_example s_illegal_mqtt_utf8_examples[] = {
    {
        .name = "non character U+0000",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\x00"),
    },
    {
        .name = "Codepoint in prohibited range U+0001 - U+001F (in the middle)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\x04"),
    },
    {
        .name = "Codepoint in prohibited range U+0001 - U+001F (boundary)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\x1F"),
    },
    {
        .name = "Codepoint in prohibited range U+007F - U+009F (min: U+7F)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\x7F"),
    },
    {
        .name = "Codepoint in prohibited range U+007F - U+009F (in the middle u+8F)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xC2\x8F"),
    },
    {
        .name = "Codepoint in prohibited range U+007F - U+009F (boundary U+9F)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xC2\x9F"),
    },
    {
        .name = "non character end with U+FFFF",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEF\xBF\xBF"),
    },
    {
        .name = "non character end with U+FFFE",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xF7\xBF\xBF\xBE"),
    },
    {
        .name = "non character in  U+FDD0 - U+FDEF (lower bound)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEF\xB7\x90"),
    },
    {
        .name = "non character in  U+FDD0 - U+FDEF (in middle)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEF\xB7\xA1"),
    },
    {
        .name = "non character in  U+FDD0 - U+FDEF (upper bound)",
        .text = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("\xEF\xB7\xAF"),
    }};

static int s_mqtt_utf8_encoded_string_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    /* Check the valid test cases */
    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_valid_mqtt_utf8_examples); ++i) {
        struct utf8_example example = s_valid_mqtt_utf8_examples[i];
        printf("valid example [%zu]: %s\n", i, example.name);
        ASSERT_SUCCESS(aws_mqtt_validate_utf8_text(example.text));
    }

    /* Glue all the valid test cases together, they ought to pass */
    struct aws_byte_buf all_good_text;
    aws_byte_buf_init(&all_good_text, allocator, 1024);
    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_valid_mqtt_utf8_examples); ++i) {
        aws_byte_buf_append_dynamic(&all_good_text, &s_valid_mqtt_utf8_examples[i].text);
    }
    ASSERT_SUCCESS(aws_mqtt_validate_utf8_text(aws_byte_cursor_from_buf(&all_good_text)));
    aws_byte_buf_clean_up(&all_good_text);

    /* Check the illegal test cases */
    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_illegal_mqtt_utf8_examples); ++i) {
        struct utf8_example example = s_illegal_mqtt_utf8_examples[i];
        printf("illegal example [%zu]: %s\n", i, example.name);
        ASSERT_FAILS(aws_mqtt_validate_utf8_text(example.text));
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(mqtt_utf8_encoded_string_test, s_mqtt_utf8_encoded_string_test)