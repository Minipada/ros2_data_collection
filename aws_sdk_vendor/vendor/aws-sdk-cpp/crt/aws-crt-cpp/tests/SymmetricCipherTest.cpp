/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/crypto/SymmetricCipher.h>
#include <aws/testing/aws_test_harness.h>

#include <utility>

static int s_TestAES_256_CBC_Generated_Materials_ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        auto cbcCipher = Aws::Crt::Crypto::SymmetricCipher::CreateAES_256_CBC_Cipher();

        ASSERT_TRUE(cbcCipher);
        ASSERT_TRUE(cbcCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        auto input = aws_byte_cursor_from_c_str("abc");

        uint8_t output[Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE * 2] = {0};
        auto outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(cbcCipher.Encrypt(input, outputBuf));
        ASSERT_TRUE(cbcCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(cbcCipher.FinalizeEncryption(outputBuf));
        ASSERT_TRUE(cbcCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_FALSE(cbcCipher);

        ASSERT_TRUE(cbcCipher.Reset());
        ASSERT_TRUE(cbcCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        auto decryptInput = Aws::Crt::ByteCursorFromByteBuf(outputBuf);
        outputBuf.len = 0;

        ASSERT_TRUE(cbcCipher.Decrypt(decryptInput, outputBuf));
        ASSERT_TRUE(cbcCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(cbcCipher.FinalizeDecryption(outputBuf));
        ASSERT_TRUE(cbcCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_BIN_ARRAYS_EQUALS(input.ptr, input.len, outputBuf.buffer, outputBuf.len);

        ASSERT_FALSE(cbcCipher);

        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_KEY_SIZE_BYTES, cbcCipher.GetKey().len);
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE, cbcCipher.GetIV().len);

        ASSERT_FALSE(cbcCipher);

        // check IV generates if a key is provided but iv is not
        uint8_t key[Aws::Crt::Crypto::AES_256_KEY_SIZE_BYTES] = {0xDD};
        auto keyCur = Aws::Crt::ByteCursorFromArray(key, sizeof(key));
        cbcCipher = Aws::Crt::Crypto::SymmetricCipher::CreateAES_256_CBC_Cipher(keyCur);
        ASSERT_TRUE(cbcCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(cbcCipher);
        ASSERT_BIN_ARRAYS_EQUALS(keyCur.ptr, keyCur.len, cbcCipher.GetKey().ptr, cbcCipher.GetKey().len);
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE, cbcCipher.GetIV().len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(AES_256_CBC_Generated_Materials_ResourceSafety, s_TestAES_256_CBC_Generated_Materials_ResourceSafety)

static int s_TestAES_256_CTR_Generated_Materials_ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        auto ctrCipher = Aws::Crt::Crypto::SymmetricCipher::CreateAES_256_CTR_Cipher();
        ASSERT_TRUE(ctrCipher);
        ASSERT_TRUE(ctrCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        auto input = aws_byte_cursor_from_c_str("abc");

        uint8_t output[Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE * 2] = {0};
        auto outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(ctrCipher.Encrypt(input, outputBuf));
        ASSERT_TRUE(ctrCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(ctrCipher.FinalizeEncryption(outputBuf));
        ASSERT_TRUE(ctrCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_FALSE(ctrCipher);

        ASSERT_TRUE(ctrCipher.Reset());
        ASSERT_TRUE(ctrCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        auto decryptInput = Aws::Crt::ByteCursorFromByteBuf(outputBuf);
        outputBuf.len = 0;

        ASSERT_TRUE(ctrCipher.Decrypt(decryptInput, outputBuf));
        ASSERT_TRUE(ctrCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(ctrCipher.FinalizeDecryption(outputBuf));
        ASSERT_TRUE(ctrCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_BIN_ARRAYS_EQUALS(input.ptr, input.len, outputBuf.buffer, outputBuf.len);

        ASSERT_FALSE(ctrCipher);

        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_KEY_SIZE_BYTES, ctrCipher.GetKey().len);
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE, ctrCipher.GetIV().len);

        ASSERT_FALSE(ctrCipher);

        // check IV generates if a key is provided but iv is not
        uint8_t key[Aws::Crt::Crypto::AES_256_KEY_SIZE_BYTES] = {0xDD};
        auto keyCur = Aws::Crt::ByteCursorFromArray(key, sizeof(key));
        ctrCipher = Aws::Crt::Crypto::SymmetricCipher::CreateAES_256_CTR_Cipher(keyCur);
        ASSERT_TRUE(ctrCipher);
        ASSERT_TRUE(ctrCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_BIN_ARRAYS_EQUALS(keyCur.ptr, keyCur.len, ctrCipher.GetKey().ptr, ctrCipher.GetKey().len);
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE, ctrCipher.GetIV().len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(AES_256_CTR_Generated_Materials_ResourceSafety, s_TestAES_256_CTR_Generated_Materials_ResourceSafety)

static int s_TestAES_256_GCM_Generated_Materials_ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        auto gcmCipher = Aws::Crt::Crypto::SymmetricCipher::CreateAES_256_GCM_Cipher();
        ASSERT_TRUE(gcmCipher);
        ASSERT_TRUE(gcmCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        auto input = aws_byte_cursor_from_c_str("abc");

        uint8_t output[Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE * 2] = {0};
        auto outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(gcmCipher.Encrypt(input, outputBuf));
        ASSERT_TRUE(gcmCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(gcmCipher.FinalizeEncryption(outputBuf));
        ASSERT_TRUE(gcmCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_FALSE(gcmCipher);

        auto tagCur = gcmCipher.GetTag();
        auto tagBuf = Aws::Crt::ByteBufNewCopy(allocator, tagCur.ptr, tagCur.len);
        tagCur = Aws::Crt::ByteCursorFromByteBuf(tagBuf);

        ASSERT_TRUE(gcmCipher.Reset());
        ASSERT_TRUE(gcmCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        gcmCipher.SetTag(tagCur);
        auto decryptInput = Aws::Crt::ByteCursorFromByteBuf(outputBuf);
        outputBuf.len = 0;

        ASSERT_TRUE(gcmCipher.Decrypt(decryptInput, outputBuf));
        ASSERT_TRUE(gcmCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(gcmCipher.FinalizeDecryption(outputBuf));
        ASSERT_TRUE(gcmCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_BIN_ARRAYS_EQUALS(input.ptr, input.len, outputBuf.buffer, outputBuf.len);

        ASSERT_FALSE(gcmCipher);

        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_KEY_SIZE_BYTES, gcmCipher.GetKey().len);
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE - 4, gcmCipher.GetIV().len);
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE, gcmCipher.GetTag().len);

        ASSERT_FALSE(gcmCipher);

        // check IV generates if a key is provided but iv is not
        uint8_t key[Aws::Crt::Crypto::AES_256_KEY_SIZE_BYTES] = {0xDD};
        auto keyCur = Aws::Crt::ByteCursorFromArray(key, sizeof(key));
        gcmCipher = Aws::Crt::Crypto::SymmetricCipher::CreateAES_256_GCM_Cipher(keyCur);
        ASSERT_TRUE(gcmCipher);
        ASSERT_TRUE(gcmCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_BIN_ARRAYS_EQUALS(keyCur.ptr, keyCur.len, gcmCipher.GetKey().ptr, gcmCipher.GetKey().len);
        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE - 4, gcmCipher.GetIV().len);

        Aws::Crt::ByteBufDelete(tagBuf);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(AES_256_GCM_Generated_Materials_ResourceSafety, s_TestAES_256_GCM_Generated_Materials_ResourceSafety)

static int s_TestAES_256_Keywrap_Generated_Materials_ResourceSafety(struct aws_allocator *allocator, void *)
{
    {
        Aws::Crt::ApiHandle apiHandle(allocator);
        auto keywrapCipher = Aws::Crt::Crypto::SymmetricCipher::CreateAES_256_KeyWrap_Cipher();
        ASSERT_TRUE(keywrapCipher);
        ASSERT_TRUE(keywrapCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        auto input = aws_byte_cursor_from_c_str("abcdefghijklmnopqrstuvwxyz123456");

        uint8_t output[Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE * 3] = {0};
        auto outputBuf = Aws::Crt::ByteBufFromEmptyArray(output, sizeof(output));

        ASSERT_TRUE(keywrapCipher.Encrypt(input, outputBuf));
        ASSERT_TRUE(keywrapCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(keywrapCipher.FinalizeEncryption(outputBuf));
        ASSERT_TRUE(keywrapCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_FALSE(keywrapCipher);

        ASSERT_TRUE(keywrapCipher.Reset());
        ASSERT_TRUE(keywrapCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);

        uint8_t decryptOutput[Aws::Crt::Crypto::AES_256_CIPHER_BLOCK_SIZE * 3] = {0};
        auto decryptOutputBuf = Aws::Crt::ByteBufFromEmptyArray(decryptOutput, sizeof(decryptOutput));

        auto decryptInput = Aws::Crt::ByteCursorFromByteBuf(outputBuf);

        ASSERT_TRUE(keywrapCipher.Decrypt(decryptInput, decryptOutputBuf));
        ASSERT_TRUE(keywrapCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Ready);
        ASSERT_TRUE(keywrapCipher.FinalizeDecryption(decryptOutputBuf));
        ASSERT_TRUE(keywrapCipher.GetState() == Aws::Crt::Crypto::SymmetricCipherState::Finalized);

        ASSERT_BIN_ARRAYS_EQUALS(input.ptr, input.len, decryptOutputBuf.buffer, decryptOutputBuf.len);

        ASSERT_FALSE(keywrapCipher);

        ASSERT_UINT_EQUALS(Aws::Crt::Crypto::AES_256_KEY_SIZE_BYTES, keywrapCipher.GetKey().len);
        ASSERT_UINT_EQUALS(0u, keywrapCipher.GetIV().len);

        ASSERT_FALSE(keywrapCipher);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    AES_256_Keywrap_Generated_Materials_ResourceSafety,
    s_TestAES_256_Keywrap_Generated_Materials_ResourceSafety)
