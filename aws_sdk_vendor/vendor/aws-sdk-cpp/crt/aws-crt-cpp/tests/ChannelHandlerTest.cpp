/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/io/ChannelHandler.h>
#include <aws/testing/aws_test_harness.h>

#include <utility>

class ChannelHandlerMock : public Aws::Crt::Io::ChannelHandler
{
  public:
    ChannelHandlerMock(Aws::Crt::Allocator *allocator) : Aws::Crt::Io::ChannelHandler(allocator) {}

    ~ChannelHandlerMock() {}

    int ProcessReadMessage(struct aws_io_message *message) override
    {
        ReceivedReadMessage =
            Aws::Crt::String(reinterpret_cast<const char *>(message->message_data.buffer), message->message_data.len);
        return AWS_OP_SUCCESS;
    }

    int ProcessWriteMessage(struct aws_io_message *message) override
    {
        ReceivedWriteMessage =
            Aws::Crt::String(reinterpret_cast<const char *>(message->message_data.buffer), message->message_data.len);
        return AWS_OP_SUCCESS;
    }

    int IncrementReadWindow(size_t size) override
    {
        WindowIncrement = size;
        return AWS_OP_SUCCESS;
    }

    void ProcessShutdown(Aws::Crt::Io::ChannelDirection dir, int errorCode, bool freeScarceResourcesImmediately)
        override
    {
        ShutdownDir = dir;
        ShutdownErrorCode = errorCode;
        FreeScarceResourcesImmediately = freeScarceResourcesImmediately;
    }

    size_t InitialWindowSize() override { return InitialWindowSizeMock; }

    size_t MessageOverhead() override { return MessageOverheadMock; }

    size_t InitialWindowSizeMock;
    size_t MessageOverheadMock;
    int ShutdownErrorCode;
    Aws::Crt::Io::ChannelDirection ShutdownDir;
    bool FreeScarceResourcesImmediately;
    size_t WindowIncrement;
    Aws::Crt::String ReceivedReadMessage;
    Aws::Crt::String ReceivedWriteMessage;
};

static int s_TestChannelHandlerInterop(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        auto channelHandlerCls = Aws::Crt::MakeShared<ChannelHandlerMock>(allocator, allocator);
        auto channelHandler = channelHandlerCls->SeatForCInterop(channelHandlerCls);

        channelHandlerCls->InitialWindowSizeMock = 13;
        channelHandlerCls->MessageOverheadMock = 32;

        auto reportedWindowSize = aws_channel_handler_initial_window_size(channelHandler);
        ASSERT_UINT_EQUALS(channelHandlerCls->InitialWindowSizeMock, reportedWindowSize);

        auto reportedMessageOverhead = channelHandler->vtable->message_overhead(channelHandler);
        ASSERT_UINT_EQUALS(channelHandlerCls->MessageOverheadMock, reportedMessageOverhead);

        ASSERT_SUCCESS(aws_channel_handler_increment_read_window(channelHandler, NULL, 10u));
        ASSERT_UINT_EQUALS(10u, channelHandlerCls->WindowIncrement);

        ASSERT_SUCCESS(aws_channel_handler_shutdown(channelHandler, NULL, AWS_CHANNEL_DIR_READ, 5, true));
        ASSERT_INT_EQUALS(5, channelHandlerCls->ShutdownErrorCode);
        ASSERT_TRUE(channelHandlerCls->FreeScarceResourcesImmediately);
        ASSERT_TRUE(Aws::Crt::Io::ChannelDirection::Read == channelHandlerCls->ShutdownDir);

        const char *readMessage = "Test Read Dir";
        struct aws_io_message message;
        AWS_ZERO_STRUCT(message);
        message.message_data = aws_byte_buf_from_c_str(readMessage);

        ASSERT_SUCCESS(aws_channel_handler_process_read_message(channelHandler, NULL, &message));
        ASSERT_STR_EQUALS(readMessage, channelHandlerCls->ReceivedReadMessage.c_str());

        const char *writeMessage = "Test Write Dir";
        message.message_data = aws_byte_buf_from_c_str(writeMessage);

        ASSERT_SUCCESS(aws_channel_handler_process_write_message(channelHandler, NULL, &message));
        ASSERT_STR_EQUALS(writeMessage, channelHandlerCls->ReceivedWriteMessage.c_str());

        // force it to free itself from the C side. This test will fail if it doesn't cause the C++ object to
        // free itself.
        aws_channel_handler_destroy(channelHandler);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(ChannelHandlerInterop, s_TestChannelHandlerInterop)
