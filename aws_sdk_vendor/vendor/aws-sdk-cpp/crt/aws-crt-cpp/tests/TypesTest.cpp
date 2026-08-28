/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/Types.h>
#include <aws/testing/aws_test_harness.h>

using namespace Aws::Crt;

static int s_base64_round_trip(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::String test_data = "foobar";
        Aws::Crt::String expected = "Zm9vYmFy";

        const Aws::Crt::Vector<uint8_t> test_vector(test_data.begin(), test_data.end());
        const Aws::Crt::Vector<uint8_t> expected_vector(expected.begin(), expected.end());

        Aws::Crt::String encoded = Aws::Crt::Base64Encode(test_vector);
        ASSERT_BIN_ARRAYS_EQUALS(expected.data(), expected.size(), encoded.data(), encoded.size());

        Aws::Crt::Vector<uint8_t> decoded = Aws::Crt::Base64Decode(encoded);
        ASSERT_BIN_ARRAYS_EQUALS(test_vector.data(), test_vector.size(), decoded.data(), decoded.size());
    }

    return 0;
}

AWS_TEST_CASE(Base64RoundTrip, s_base64_round_trip)

static int s_int_array_list_to_vector(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        size_t list_size = 10;
        aws_array_list intList;
        ASSERT_SUCCESS(
            aws_array_list_init_dynamic(&intList, allocator, list_size, sizeof(int)),
            "List setup should have been successful. err code %d",
            aws_last_error());
        ASSERT_INT_EQUALS(0, intList.length, "List size should be 0.");
        ASSERT_INT_EQUALS(
            list_size,
            intList.current_size / sizeof(int),
            "Allocated list size should be %d.",
            (int)list_size * sizeof(int));

        Aws::Crt::Vector<int> intVector = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        for (size_t i = 0; i < list_size; i++)
        {
            ASSERT_SUCCESS(
                aws_array_list_push_back(&intList, (void *)&intVector[i]),
                "List push failed with error code %d",
                aws_last_error());
        }

        Aws::Crt::Vector<int> resVector = Aws::Crt::ArrayListToVector<int>(&intList);

        aws_array_list_clean_up(&intList);

        ASSERT_UINT_EQUALS(10u, resVector.size());
        for (size_t i = 0; i < list_size; i++)
        {
            ASSERT_INT_EQUALS(i + 1, resVector[i]);
        }
    }

    return 0;
}

AWS_TEST_CASE(TestIntArrayListToVector, s_int_array_list_to_vector)

static int s_byte_cursor_array_list_to_vector(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        size_t list_size = 10;
        aws_array_list cursorList;
        ASSERT_SUCCESS(
            aws_array_list_init_dynamic(&cursorList, allocator, list_size, sizeof(ByteCursor)),
            "List setup should have been successful. err code %d",
            aws_last_error());
        ASSERT_INT_EQUALS(0, cursorList.length, "List size should be 0.");
        ASSERT_INT_EQUALS(
            list_size,
            cursorList.current_size / sizeof(ByteCursor),
            "Allocated list size should be %d.",
            (int)list_size * sizeof(ByteCursor));

        Aws::Crt::Vector<ByteCursor> byteCursorVector(list_size);
        for (size_t i = 0; i < list_size; i++)
        {
            byteCursorVector[i].len = i;
            byteCursorVector[i].ptr = (uint8_t *)0x01234F;
            ASSERT_SUCCESS(
                aws_array_list_push_back(&cursorList, (void *)&byteCursorVector[i]),
                "List push failed with error code %d",
                aws_last_error());
        }

        Aws::Crt::Vector<ByteCursor> resVector = Aws::Crt::ArrayListToVector<ByteCursor>(&cursorList);

        aws_array_list_clean_up(&cursorList);

        ASSERT_UINT_EQUALS(10u, resVector.size());
        for (size_t i = 0; i < list_size; i++)
        {
            ASSERT_INT_EQUALS(i, resVector[i].len);
        }
    }

    return 0;
}

AWS_TEST_CASE(TestByteCursorArrayListToVector, s_byte_cursor_array_list_to_vector)

static int s_byte_buff_init_delete(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        const auto targetLength = 8;
        auto byteBuff = ByteBufInit(allocator, targetLength);
        ASSERT_UINT_EQUALS(targetLength, byteBuff.len);
        ASSERT_TRUE(byteBuff.buffer != nullptr);
        ByteBufDelete(byteBuff);
        ASSERT_UINT_EQUALS(targetLength, 0);
        ASSERT_TRUE(byteBuff.buffer == nullptr);
        return 0;
    }
}

AWS_TEST_CASE(TestByteBufInitDelete, s_byte_buff_init_delete)
