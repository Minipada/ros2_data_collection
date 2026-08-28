/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/crypto/Hash.h>
#include <aws/testing/aws_test_harness.h>

#include <utility>

#if !BYO_CRYPTO
static int s_TestSHA256ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Crypto::Hash sha256 = Aws::Crt::Crypto::Hash::CreateSHA256(allocator);
        ASSERT_TRUE(sha256);

        Aws::Crt::ByteCursor input = aws_byte_cursor_from_c_str("abc");
        uint8_t expected[] = {
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
            0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
        };
        Aws::Crt::ByteBuf expectedBuf = Aws::Crt::ByteBufFromArray(expected, sizeof(expected));

        uint8_t output[Aws::Crt::Crypto::SHA256_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(sha256.Update(input));
        ASSERT_TRUE(sha256.Digest(outputBuf));
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::SHA256_DIGEST_SIZE, sha256.DigestSize());

        ASSERT_FALSE(sha256);

        ASSERT_BIN_ARRAYS_EQUALS(expectedBuf.buffer, expectedBuf.len, outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(SHA256ResourceSafety, s_TestSHA256ResourceSafety)

static int s_TestMD5ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Crypto::Hash md5 = Aws::Crt::Crypto::Hash::CreateMD5(allocator);
        ASSERT_TRUE(md5);

        Aws::Crt::ByteCursor input = aws_byte_cursor_from_c_str("abc");
        uint8_t expected[] = {
            0x90,
            0x01,
            0x50,
            0x98,
            0x3c,
            0xd2,
            0x4f,
            0xb0,
            0xd6,
            0x96,
            0x3f,
            0x7d,
            0x28,
            0xe1,
            0x7f,
            0x72,
        };
        Aws::Crt::ByteBuf expectedBuf = Aws::Crt::ByteBufFromArray(expected, sizeof(expected));

        uint8_t output[Aws::Crt::Crypto::MD5_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(md5.Update(input));
        ASSERT_TRUE(md5.Digest(outputBuf));
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::MD5_DIGEST_SIZE, md5.DigestSize());

        ASSERT_FALSE(md5);

        ASSERT_BIN_ARRAYS_EQUALS(expectedBuf.buffer, expectedBuf.len, outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MD5ResourceSafety, s_TestMD5ResourceSafety)

static int s_TestSHA1ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        Aws::Crt::Crypto::Hash sha1 = Aws::Crt::Crypto::Hash::CreateSHA1(allocator);
        ASSERT_TRUE(sha1);

        Aws::Crt::ByteCursor input =
            aws_byte_cursor_from_c_str("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnop"
                                       "jklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu");
        uint8_t expected[] = {0xa4, 0x9b, 0x24, 0x46, 0xa0, 0x2c, 0x64, 0x5b, 0xf4, 0x19,
                              0xf9, 0x95, 0xb6, 0x70, 0x91, 0x25, 0x3a, 0x04, 0xa2, 0x59};
        Aws::Crt::ByteBuf expectedBuf = Aws::Crt::ByteBufFromArray(expected, sizeof(expected));

        uint8_t output[Aws::Crt::Crypto::SHA1_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(sha1.Update(input));
        ASSERT_TRUE(sha1.Digest(outputBuf));
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::SHA1_DIGEST_SIZE, sha1.DigestSize());
        ASSERT_FALSE(sha1);

        ASSERT_BIN_ARRAYS_EQUALS(expectedBuf.buffer, expectedBuf.len, outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(SHA1ResourceSafety, s_TestSHA1ResourceSafety)

#else

class ByoCryptoHashInterceptor : public Aws::Crt::Crypto::ByoHash
{
  public:
    ByoCryptoHashInterceptor(size_t digestSize, Aws::Crt::Allocator *allocator, Aws::Crt::String output)
        : ByoHash(digestSize, allocator), m_output(std::move(output))
    {
    }

    ~ByoCryptoHashInterceptor() {}

    bool UpdateInternal(const Aws::Crt::ByteCursor &toHash) noexcept
    {
        m_receivedInput = Aws::Crt::String(reinterpret_cast<const char *>(toHash.ptr), toHash.len);
        return true;
    }

    bool DigestInternal(Aws::Crt::ByteBuf &output, size_t) noexcept
    {
        aws_byte_buf_write(&output, reinterpret_cast<const uint8_t *>(m_output.data()), m_output.length());
        return true;
    }

    const Aws::Crt::String &GetReceivedInput() const { return m_receivedInput; }

  private:
    Aws::Crt::String m_receivedInput;
    Aws::Crt::String m_output;
};

static int s_TestSHA256ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        uint8_t expected[] = {
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
            0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
        };
        Aws::Crt::String expectedStr = Aws::Crt::String(reinterpret_cast<const char *>(expected), sizeof(expected));

        apiHandle.SetBYOCryptoNewSHA256Callback(
            [&](size_t digestSize, Aws::Crt::Allocator *allocator)
            { return Aws::Crt::MakeShared<ByoCryptoHashInterceptor>(allocator, digestSize, allocator, expectedStr); });

        Aws::Crt::Crypto::Hash sha256 = Aws::Crt::Crypto::Hash::CreateSHA256(allocator);
        ASSERT_TRUE(sha256);

        Aws::Crt::ByteCursor input = aws_byte_cursor_from_c_str("abc");

        uint8_t output[Aws::Crt::Crypto::SHA256_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(sha256.Update(input));
        ASSERT_TRUE(sha256.Digest(outputBuf));
        ASSERT_FALSE(sha256);

        ASSERT_BIN_ARRAYS_EQUALS(expectedStr.c_str(), expectedStr.length(), outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(SHA256ResourceSafety, s_TestSHA256ResourceSafety)

static int s_TestSHA1ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        uint8_t expected[] = {
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41,
            0x40, 0xde, 0x5d, 0xae, 0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3,
        };
        Aws::Crt::String expectedStr = Aws::Crt::String(reinterpret_cast<const char *>(expected), sizeof(expected));

        apiHandle.SetBYOCryptoNewSHA1Callback(
            [&](size_t digestSize, Aws::Crt::Allocator *allocator)
            { return Aws::Crt::MakeShared<ByoCryptoHashInterceptor>(allocator, digestSize, allocator, expectedStr); });

        Aws::Crt::Crypto::Hash sha1 = Aws::Crt::Crypto::Hash::CreateSHA1(allocator);
        ASSERT_TRUE(sha1);

        Aws::Crt::ByteCursor input = aws_byte_cursor_from_c_str("abc");

        uint8_t output[Aws::Crt::Crypto::SHA1_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(sha1.Update(input));
        ASSERT_TRUE(sha1.Digest(outputBuf));
        ASSERT_FALSE(sha1);

        ASSERT_BIN_ARRAYS_EQUALS(expectedStr.c_str(), expectedStr.length(), outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(SHA1ResourceSafety, s_TestSHA1ResourceSafety)

static int s_TestMD5ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        uint8_t expected[] = {
            0x90,
            0x01,
            0x50,
            0x98,
            0x3c,
            0xd2,
            0x4f,
            0xb0,
            0xd6,
            0x96,
            0x3f,
            0x7d,
            0x28,
            0xe1,
            0x7f,
            0x72,
        };
        Aws::Crt::String expectedStr = Aws::Crt::String(reinterpret_cast<const char *>(expected), sizeof(expected));

        apiHandle.SetBYOCryptoNewMD5Callback(
            [&](size_t digestSize, struct aws_allocator *allocator)
            { return Aws::Crt::MakeShared<ByoCryptoHashInterceptor>(allocator, digestSize, allocator, expectedStr); });

        Aws::Crt::Crypto::Hash md5 = Aws::Crt::Crypto::Hash::CreateMD5(allocator);
        ASSERT_TRUE(md5);

        Aws::Crt::ByteCursor input = aws_byte_cursor_from_c_str("abc");

        uint8_t output[Aws::Crt::Crypto::MD5_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(md5.Update(input));
        ASSERT_TRUE(md5.Digest(outputBuf));
        ASSERT_FALSE(md5);

        ASSERT_BIN_ARRAYS_EQUALS(expectedStr.c_str(), expectedStr.length(), outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(MD5ResourceSafety, s_TestMD5ResourceSafety)

#endif /* BYO_CRYPTO */
