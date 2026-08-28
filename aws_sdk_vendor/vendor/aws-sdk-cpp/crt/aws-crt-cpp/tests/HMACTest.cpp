/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/crypto/HMAC.h>
#include <aws/testing/aws_test_harness.h>

#if !BYO_CRYPTO

static int s_TestSHA256HMACResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        uint8_t secret[] = {
            0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
            0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        };

        Aws::Crt::ByteCursor secretCur = aws_byte_cursor_from_array(secret, sizeof(secret));

        Aws::Crt::Crypto::HMAC sha256Hmac = Aws::Crt::Crypto::HMAC::CreateSHA256HMAC(allocator, secretCur);
        ASSERT_TRUE(sha256Hmac);

        Aws::Crt::ByteCursor input = aws_byte_cursor_from_c_str("Hi There");
        uint8_t expected[] = {
            0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53, 0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b, 0xf1, 0x2b,
            0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7, 0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7,
        };
        Aws::Crt::ByteBuf expectedBuf = Aws::Crt::ByteBufFromArray(expected, sizeof(expected));

        uint8_t output[Aws::Crt::Crypto::SHA256_HMAC_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::SHA256_HMAC_DIGEST_SIZE, sha256Hmac.DigestSize());
        ASSERT_TRUE(sha256Hmac.Update(input));
        ASSERT_TRUE(sha256Hmac.Digest(outputBuf));
        ASSERT_FALSE(sha256Hmac);

        ASSERT_BIN_ARRAYS_EQUALS(expectedBuf.buffer, expectedBuf.len, outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}

#else
class ByoCryptoHMACInterceptor : public Aws::Crt::Crypto::ByoHMAC
{
  public:
    ByoCryptoHMACInterceptor(
        size_t digestSize,
        Aws::Crt::Allocator *allocator,
        const Aws::Crt::ByteCursor &secret,
        Aws::Crt::String output)
        : ByoHMAC(digestSize, secret, allocator), m_output(std::move(output))
    {
        m_secret = Aws::Crt::String(reinterpret_cast<const char *>(secret.ptr), secret.len);
    }

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
    const Aws::Crt::String &GetSecret() const { return m_secret; }

  private:
    Aws::Crt::String m_secret;
    Aws::Crt::String m_receivedInput;
    Aws::Crt::String m_output;
};

static int s_TestSHA256HMACResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        uint8_t secret[] = {
            0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
            0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        };

        Aws::Crt::ByteCursor secretCur = aws_byte_cursor_from_array(secret, sizeof(secret));

        Aws::Crt::ByteCursor input = aws_byte_cursor_from_c_str("Hi There");
        uint8_t expected[] = {
            0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53, 0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b, 0xf1, 0x2b,
            0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7, 0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7,
        };
        Aws::Crt::String expectedStr(reinterpret_cast<const char *>(expected), sizeof(expected));

        apiHandle.SetBYOCryptoNewSHA256HMACCallback(
            [&](size_t digest_size, const Aws::Crt::ByteCursor &secretCur, Aws::Crt::Allocator *allocator) {
                return Aws::Crt::MakeShared<ByoCryptoHMACInterceptor>(
                    allocator, digest_size, allocator, secretCur, expectedStr);
            });

        uint8_t output[Aws::Crt::Crypto::SHA256_HMAC_DIGEST_SIZE] = {0};
        Aws::Crt::ByteBuf outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        Aws::Crt::Crypto::HMAC sha256Hmac = Aws::Crt::Crypto::HMAC::CreateSHA256HMAC(allocator, secretCur);
        ASSERT_TRUE(sha256Hmac.Update(input));
        ASSERT_TRUE(sha256Hmac.Digest(outputBuf));
        ASSERT_FALSE(sha256Hmac);

        ASSERT_BIN_ARRAYS_EQUALS(expectedStr.c_str(), expectedStr.length(), outputBuf.buffer, outputBuf.len);
    }

    return AWS_OP_SUCCESS;
}
#endif

AWS_TEST_CASE(SHA256HMACResourceSafety, s_TestSHA256HMACResourceSafety)
