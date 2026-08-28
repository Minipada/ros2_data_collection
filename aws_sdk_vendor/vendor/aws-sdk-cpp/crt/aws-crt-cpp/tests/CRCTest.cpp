/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/checksum/CRC.h>
#include <aws/testing/aws_test_harness.h>

static int s_TestCRC32Piping(struct aws_allocator *allocator, void *)
{
    Aws::Crt::ApiHandle apiHandle(allocator);
    uint8_t data[32] = {0};

    Aws::Crt::ByteCursor dataCur = aws_byte_cursor_from_array(data, sizeof(data));

    ASSERT_UINT_EQUALS(0x190A55AD, Aws::Crt::Checksum::ComputeCRC32(dataCur));

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(CRC32Piping, s_TestCRC32Piping)

static int s_TestCRC32CPiping(struct aws_allocator *allocator, void *)
{
    Aws::Crt::ApiHandle apiHandle(allocator);
    uint8_t data[32] = {0};

    Aws::Crt::ByteCursor dataCur = aws_byte_cursor_from_array(data, sizeof(data));

    ASSERT_UINT_EQUALS(0x8A9136AA, Aws::Crt::Checksum::ComputeCRC32C(dataCur));

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(CRC32CPiping, s_TestCRC32CPiping)

static int s_TestCRC64NVMEPiping(struct aws_allocator *allocator, void *)
{
    Aws::Crt::ApiHandle apiHandle(allocator);
    uint8_t data[32] = {0};

    Aws::Crt::ByteCursor dataCur = aws_byte_cursor_from_array(data, sizeof(data));

    ASSERT_UINT_EQUALS(0xCF3473434D4ECF3B, Aws::Crt::Checksum::ComputeCRC64NVME(dataCur));

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(CRC64NVMEPiping, s_TestCRC64NVMEPiping)
