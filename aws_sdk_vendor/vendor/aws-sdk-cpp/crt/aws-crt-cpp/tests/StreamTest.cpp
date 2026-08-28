/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>

#include <aws/crt/io/Stream.h>

#include <aws/common/byte_buf.h>
#include <aws/io/stream.h>

#include <aws/testing/aws_test_harness.h>

#include <sstream>

static int s_StreamTestCreateDestroyWrapper(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        auto stringStream = Aws::Crt::MakeShared<std::stringstream>(allocator, "SomethingInteresting");
        Aws::Crt::Io::StdIOStreamInputStream inputStream(stringStream, allocator);

        ASSERT_TRUE(static_cast<bool>(inputStream));
        ASSERT_NOT_NULL(inputStream.GetUnderlyingStream());
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StreamTestCreateDestroyWrapper, s_StreamTestCreateDestroyWrapper)

static const char *STREAM_CONTENTS = "SomeContents";

static int s_StreamTestLength(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        auto stringStream = Aws::Crt::MakeShared<std::stringstream>(allocator, STREAM_CONTENTS);

        Aws::Crt::Io::StdIOStreamInputStream wrappedStream(stringStream, allocator);

        int64_t length = 0;
        ASSERT_SUCCESS(aws_input_stream_get_length(wrappedStream.GetUnderlyingStream(), &length));
        ASSERT_TRUE(static_cast<uint64_t>(length) == strlen(STREAM_CONTENTS));
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StreamTestLength, s_StreamTestLength)

static int s_StreamTestRead(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        auto stringStream = Aws::Crt::MakeShared<Aws::Crt::StringStream>(allocator, STREAM_CONTENTS);

        Aws::Crt::Io::StdIOStreamInputStream wrappedStream(stringStream, allocator);

        aws_byte_buf buffer;
        AWS_ZERO_STRUCT(buffer);
        aws_byte_buf_init(&buffer, allocator, 256);

        ASSERT_SUCCESS(aws_input_stream_read(wrappedStream.GetUnderlyingStream(), &buffer));

        ASSERT_TRUE(buffer.len == strlen(STREAM_CONTENTS));
        ASSERT_BIN_ARRAYS_EQUALS(STREAM_CONTENTS, buffer.len, buffer.buffer, buffer.len);

        aws_byte_buf_clean_up(&buffer);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StreamTestRead, s_StreamTestRead)

static int s_StreamTestReadEmpty(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        auto stringStream = Aws::Crt::MakeShared<Aws::Crt::StringStream>(allocator, "");

        Aws::Crt::Io::StdIOStreamInputStream wrappedStream(stringStream, allocator);

        aws_byte_buf buffer;
        AWS_ZERO_STRUCT(buffer);
        aws_byte_buf_init(&buffer, allocator, 256);

        ASSERT_SUCCESS(aws_input_stream_read(wrappedStream.GetUnderlyingStream(), &buffer));

        ASSERT_TRUE(buffer.len == 0);

        aws_byte_buf_clean_up(&buffer);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StreamTestReadEmpty, s_StreamTestReadEmpty)

static const int64_t BEGIN_SEEK_OFFSET = 4;

static int s_StreamTestSeekBegin(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        auto stringStream = Aws::Crt::MakeShared<Aws::Crt::StringStream>(allocator, STREAM_CONTENTS);

        Aws::Crt::Io::StdIOStreamInputStream wrappedStream(stringStream, allocator);

        ASSERT_SUCCESS(aws_input_stream_seek(wrappedStream.GetUnderlyingStream(), BEGIN_SEEK_OFFSET, AWS_SSB_BEGIN));

        aws_byte_buf buffer;
        AWS_ZERO_STRUCT(buffer);
        aws_byte_buf_init(&buffer, allocator, 256);

        ASSERT_SUCCESS(aws_input_stream_read(wrappedStream.GetUnderlyingStream(), &buffer));

        ASSERT_TRUE(buffer.len == strlen(STREAM_CONTENTS) - BEGIN_SEEK_OFFSET);
        ASSERT_BIN_ARRAYS_EQUALS(STREAM_CONTENTS + BEGIN_SEEK_OFFSET, buffer.len, buffer.buffer, buffer.len);

        aws_byte_buf_clean_up(&buffer);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StreamTestSeekBegin, s_StreamTestSeekBegin)

static const int64_t END_SEEK_OFFSET = -4;

static int s_StreamTestSeekEnd(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        auto stringStream = Aws::Crt::MakeShared<Aws::Crt::StringStream>(allocator, STREAM_CONTENTS);

        Aws::Crt::Io::StdIOStreamInputStream wrappedStream(stringStream, allocator);

        ASSERT_SUCCESS(aws_input_stream_seek(wrappedStream.GetUnderlyingStream(), END_SEEK_OFFSET, AWS_SSB_END));

        aws_byte_buf buffer;
        AWS_ZERO_STRUCT(buffer);
        aws_byte_buf_init(&buffer, allocator, 256);

        ASSERT_SUCCESS(aws_input_stream_read(wrappedStream.GetUnderlyingStream(), &buffer));

        ASSERT_TRUE(buffer.len == -END_SEEK_OFFSET);
        ASSERT_BIN_ARRAYS_EQUALS(
            STREAM_CONTENTS + strlen(STREAM_CONTENTS) + END_SEEK_OFFSET, buffer.len, buffer.buffer, buffer.len);

        aws_byte_buf_clean_up(&buffer);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StreamTestSeekEnd, s_StreamTestSeekEnd)

/* Test that C/C++ has the refcount on the stream will keep the object alive */
static int s_StreamTestRefcount(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        aws_input_stream *c_stream = NULL;
        {
            auto stringStream = Aws::Crt::MakeShared<Aws::Crt::StringStream>(allocator, STREAM_CONTENTS);
            /* Make a shared pointer for stream as the C side will ONLY interact with the shared pointer initialed
             * stream */
            std::shared_ptr<Aws::Crt::Io::StdIOStreamInputStream> wrappedStream =
                Aws::Crt::MakeShared<Aws::Crt::Io::StdIOStreamInputStream>(allocator, stringStream, allocator);

            /* C side keep a reference on it. */
            aws_input_stream_acquire(wrappedStream->GetUnderlyingStream());
            /* C side release a reference on it. So that it drops to zero from the C point of view, but as C++ still
             * holding it, it's still valid to be used */
            aws_input_stream_release(wrappedStream->GetUnderlyingStream());
            /* Test that you can still use it correctly */
            int64_t length = 0;
            ASSERT_SUCCESS(aws_input_stream_get_length(wrappedStream->GetUnderlyingStream(), &length));
            ASSERT_TRUE(static_cast<uint64_t>(length) == strlen(STREAM_CONTENTS));

            /* C side keep a reference on it. */
            aws_input_stream_acquire(wrappedStream->GetUnderlyingStream());
            c_stream = wrappedStream->GetUnderlyingStream();
        }
        /* C++ object is now out of scope, but as C side still holding the reference to it, it still avaliable to be
         * invoked from C */
        int64_t length = 0;
        ASSERT_SUCCESS(aws_input_stream_get_length(c_stream, &length));
        ASSERT_TRUE(static_cast<uint64_t>(length) == strlen(STREAM_CONTENTS));
        /* Release the refcount from C to clean up resource without leak */
        aws_input_stream_release(c_stream);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StreamTestRefcount, s_StreamTestRefcount)
