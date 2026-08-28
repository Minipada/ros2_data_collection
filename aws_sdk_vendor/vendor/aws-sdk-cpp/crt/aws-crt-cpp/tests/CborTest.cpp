/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/cbor/Cbor.h>
#include <aws/testing/aws_test_harness.h>

using namespace Aws::Crt;

static int s_CborSanityTest(struct aws_allocator *allocator, void *ctx)
{
    /**
     * Simply test every method works.
     */
    (void)ctx;
    {
        ApiHandle apiHandle(allocator);
        Cbor::CborEncoder encoder(allocator);

        // Define expected values
        auto expected_uint_val = 42ULL;
        auto expected_negint_val = 123ULL;
        auto expected_float_val = 3.14;
        ByteCursor expected_bytes_val = aws_byte_cursor_from_c_str("write more");
        ByteCursor expected_text_val = aws_byte_cursor_from_c_str("test");
        size_t expected_array_size = 5;
        size_t expected_map_size = 3;
        auto expected_tag_val = 999ULL;
        bool expected_bool_val = true;

        encoder.WriteUInt(expected_uint_val);
        encoder.WriteNegInt(expected_negint_val);
        encoder.WriteFloat(expected_float_val);
        encoder.WriteBytes(expected_bytes_val);
        encoder.WriteText(expected_text_val);
        encoder.WriteArrayStart(expected_array_size);
        encoder.WriteMapStart(expected_map_size);
        encoder.WriteTag(expected_tag_val);
        encoder.WriteBool(expected_bool_val);
        encoder.WriteNull();
        encoder.WriteUndefined();
        encoder.WriteBreak();
        encoder.WriteIndefBytesStart();
        encoder.WriteIndefTextStart();
        encoder.WriteIndefArrayStart();
        encoder.WriteIndefMapStart();

        ByteCursor cursor = encoder.GetEncodedData();

        Cbor::CborDecoder decoder(cursor, allocator);

        // Check for UInt
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::UInt);
        ASSERT_TRUE(decoder.PopNextUnsignedIntVal().value() == expected_uint_val);

        // Check for NegInt
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::NegInt);
        ASSERT_TRUE(decoder.PopNextNegativeIntVal().value() == expected_negint_val);

        // Check for Float
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Float);
        ASSERT_TRUE(decoder.PopNextFloatVal().value() == expected_float_val);

        // Check for Bytes
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Bytes);
        ByteCursor decoded_val = decoder.PopNextBytesVal().value();
        ASSERT_TRUE(aws_byte_cursor_eq(&decoded_val, &expected_bytes_val));

        // Check for Text
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Text);
        decoded_val = decoder.PopNextTextVal().value();
        ASSERT_TRUE(aws_byte_cursor_eq(&decoded_val, &expected_text_val));

        // Check for ArrayStart
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::ArrayStart);
        ASSERT_TRUE(decoder.PopNextArrayStart().value() == expected_array_size);

        // Check for MapStart
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::MapStart);
        ASSERT_TRUE(decoder.PopNextMapStart().value() == expected_map_size);
        // Check for Tag
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Tag);
        ASSERT_TRUE(decoder.PopNextTagVal().value() == expected_tag_val);

        // Check for Bool
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Bool);
        ASSERT_TRUE(decoder.PopNextBooleanVal().value() == expected_bool_val);

        // Check for Null
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Null);
        ASSERT_TRUE(decoder.ConsumeNextWholeDataItem());

        // Check for Undefined
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Undefined);
        ASSERT_TRUE(decoder.ConsumeNextWholeDataItem());

        // Check for Break
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::Break);
        ASSERT_TRUE(decoder.ConsumeNextSingleElement());

        // Check for IndefBytesStart
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::IndefBytesStart);
        ASSERT_TRUE(decoder.ConsumeNextSingleElement());

        // Check for IndefTextStart
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::IndefTextStart);
        ASSERT_TRUE(decoder.ConsumeNextSingleElement());

        // Check for IndefArrayStart
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::IndefArrayStart);
        ASSERT_TRUE(decoder.ConsumeNextSingleElement());

        // Check for IndefMapStart
        ASSERT_TRUE(decoder.PeekType().value() == Cbor::CborType::IndefMapStart);
        ASSERT_TRUE(decoder.ConsumeNextSingleElement());

        auto length = decoder.GetRemainingLength();
        ASSERT_UINT_EQUALS(0, length);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(CborSanityTest, s_CborSanityTest)

static void s_encode_timestamp_helper(
    Cbor::CborEncoder &encoder,
    const std::chrono::system_clock::time_point &timePoint) noexcept
{
    /* Get seconds with MS precision. */
    std::chrono::duration<double, std::chrono::seconds::period> timestamp(timePoint.time_since_epoch());
    double seconds = timestamp.count();

    encoder.WriteTag(AWS_CBOR_TAG_EPOCH_TIME);
    // Use the encoder to write the duration in seconds
    encoder.WriteFloat(seconds);
}

static int s_decode_timestamp_helper(Cbor::CborDecoder &decoder, std::chrono::system_clock::time_point &outTimePoint)
{
    if (decoder.PeekType().value() != Cbor::CborType::Tag)
    {
        return AWS_OP_ERR;
    }
    if (decoder.PopNextTagVal().value() != AWS_CBOR_TAG_EPOCH_TIME)
    {
        return AWS_OP_ERR;
    }
    Cbor::CborType out_type = decoder.PeekType().value();
    switch (out_type)
    {
        case Cbor::CborType::UInt:
        case Cbor::CborType::NegInt:
        {
            int64_t secs_timestamp = 0;
            uint64_t unsigned_val = 0;
            if (out_type == Cbor::CborType::UInt)
            {
                unsigned_val = decoder.PopNextUnsignedIntVal().value();
                if (unsigned_val > INT64_MAX)
                {
                    /* Overflow */
                    return AWS_OP_ERR;
                }
                secs_timestamp = (int64_t)unsigned_val;
            }
            else
            {
                unsigned_val = decoder.PopNextNegativeIntVal().value();
                if (unsigned_val > INT64_MAX)
                {
                    /* Overflow */
                    return AWS_OP_ERR;
                }
                /* convert the encoded number to real negative number */
                secs_timestamp = (int64_t)(-1 - unsigned_val);
            }
            std::chrono::duration<int64_t, std::chrono::seconds::period> timestamp(secs_timestamp);
            outTimePoint = std::chrono::system_clock::time_point(timestamp);
            return AWS_OP_SUCCESS;
        }
        case Cbor::CborType::Float:
        {
            double double_val = decoder.PopNextFloatVal().value();
            std::chrono::duration<double, std::chrono::seconds::period> timestamp(double_val);
            outTimePoint = std::chrono::system_clock::time_point(
                std::chrono::duration_cast<std::chrono::system_clock::duration>(timestamp));
            return AWS_OP_SUCCESS;
        }
        default:
            break;
    }

    return AWS_OP_ERR;
}

static bool s_check_time_point_equals_ms_precision(
    const std::chrono::time_point<std::chrono::system_clock> &a,
    const std::chrono::time_point<std::chrono::system_clock> &b)
{
    using Ms = std::chrono::milliseconds;
    auto a_ms = std::chrono::duration_cast<Ms>(a.time_since_epoch());
    auto b_ms = std::chrono::duration_cast<Ms>(b.time_since_epoch());
    if (a_ms == b_ms)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static int s_CborTimeStampTest(struct aws_allocator *allocator, void *ctx)
{
    /**
     * Example of how timestamp will be encoded and decoded with `std::chrono::system_clock::time_point`
     */
    (void)ctx;
    {
        ApiHandle apiHandle(allocator);
        Cbor::CborEncoder encoder(allocator);
        const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

        s_encode_timestamp_helper(encoder, now);
        ByteCursor cursor = encoder.GetEncodedData();

        Cbor::CborDecoder decoder(cursor);
        std::chrono::time_point<std::chrono::system_clock> decoded;
        ASSERT_SUCCESS(s_decode_timestamp_helper(decoder, decoded));

        /* We only check the ms precision */
        ASSERT_TRUE(s_check_time_point_equals_ms_precision(decoded, now));
        auto length = decoder.GetRemainingLength();
        ASSERT_UINT_EQUALS(0, length);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(CborTimeStampTest, s_CborTimeStampTest)
