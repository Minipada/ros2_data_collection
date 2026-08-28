/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/cal/ed25519.h>
#include <aws/cal/private/opensslcrypto_common.h>

#include <aws/common/device_random.h>
#include <aws/common/encoding.h>

#include <openssl/evp.h>

#if defined(OPENSSL_IS_OPENSSL) && OPENSSL_VERSION_NUMBER < 0x10101000L
/* ed25519 support does not exist prior to 1.1.1 */
#    define LIBCRYPTO_DOES_NOT_SUPPORT_ED25519
#endif

struct aws_ed25519_key_pair_impl {
    struct aws_allocator *allocator;
    EVP_PKEY *key;
};

int aws_ed25519_key_pair_get_private_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out);

int aws_ed25519_key_pair_get_public_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out);

static const size_t s_private_key_size = 32;
static const size_t s_public_key_size = 32;

int s_byte_buf_write_be32_with_err(struct aws_byte_buf *buf, uint32_t x) {
    return aws_byte_buf_write_be32(buf, x) ? AWS_OP_SUCCESS : AWS_ERROR_SHORT_BUFFER;
}

void aws_ed25519_key_pair_destroy_impl(struct aws_ed25519_key_pair_impl *key_pair) {
    if (key_pair == NULL) {
        return;
    }

    if (key_pair->key != NULL) {
        EVP_PKEY_free(key_pair->key);
    }

    aws_mem_release(key_pair->allocator, key_pair);
}

struct aws_ed25519_key_pair_impl *aws_ed25519_key_pair_new_generate_impl(struct aws_allocator *allocator) {
#if defined(LIBCRYPTO_DOES_NOT_SUPPORT_ED25519)
    /* Compile time check on whether we compiled against libcrypto that supported ed25519
     * Note: skipping explicit runtime check here because EVP_PKEY_CTX_new_id existed before ed25519 support was added,
     * but algo was not defined, so ctx init will fail on old versions at runtime.
     */
    aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM);
    return NULL;
#else
    EVP_PKEY *pkey = NULL;

    EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new_id(EVP_PKEY_ED25519, NULL);
    if (ctx == NULL) {
        aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM);
        return NULL;
    }

    if (aws_reinterpret_lc_evp_error_as_crt(EVP_PKEY_keygen_init(ctx), "EVP_PKEY_keygen_init", AWS_LS_CAL_ED25519)) {
        goto on_error;
    }

    if (aws_reinterpret_lc_evp_error_as_crt(EVP_PKEY_keygen(ctx, &pkey), "EVP_PKEY_keygen", AWS_LS_CAL_ED25519)) {
        goto on_error;
    }

    struct aws_ed25519_key_pair_impl *key_pair = aws_mem_calloc(allocator, 1, sizeof(struct aws_ed25519_key_pair_impl));
    key_pair->key = pkey;
    key_pair->allocator = allocator;

    EVP_PKEY_CTX_free(ctx);
    return key_pair;

on_error:
    EVP_PKEY_CTX_free(ctx);
    return NULL;
#endif
}

static struct aws_byte_cursor s_key_type_literal = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("ssh-ed25519");

int s_ed25519_openssh_encode_public_key(const struct aws_ed25519_key_pair_impl *key_pair, struct aws_byte_buf *out) {
    if (s_byte_buf_write_be32_with_err(out, (uint32_t)s_key_type_literal.len) != AWS_OP_SUCCESS ||
        aws_byte_buf_append(out, &s_key_type_literal) != AWS_OP_SUCCESS) {
        return AWS_OP_ERR;
    }

    if (s_byte_buf_write_be32_with_err(out, 32) != AWS_OP_SUCCESS ||
        aws_ed25519_key_pair_get_public_key_impl(key_pair, AWS_CAL_ED25519_KEY_EXPORT_RAW, out) != AWS_OP_SUCCESS) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

/**
 * format here is b64 of the following structure
 * string "ssh-ed25519" #literal
 * string key
 * Note: string is always u32 size followed by the data. all multibyte ints are in big-endian
 */
int s_ed25519_export_public_openssh(const struct aws_ed25519_key_pair_impl *key_pair, struct aws_byte_buf *out) {
    uint8_t key_data[4 /*id len*/ + 11 /* ssh-ed25519 literal */ + 4 /*key len*/ + 32 /* key */] = {0};
    struct aws_byte_buf key_buf = aws_byte_buf_from_empty_array(key_data, AWS_ARRAY_SIZE(key_data));

    if (s_ed25519_openssh_encode_public_key(key_pair, &key_buf)) {
        return AWS_OP_ERR;
    }

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_buf(&key_buf);

    if (aws_base64_encode(&key_cur, out) != AWS_OP_SUCCESS) {
        return AWS_OP_ERR;
    }

    return AWS_OP_SUCCESS;
}

int s_ed25519_export_public_raw(const struct aws_ed25519_key_pair_impl *key_pair, struct aws_byte_buf *out) {
#if defined(LIBCRYPTO_DOES_NOT_SUPPORT_ED25519)
    return aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM);
#else
    size_t remaining = out->capacity - out->len;
    if (remaining < s_public_key_size) {
        return aws_raise_error(AWS_ERROR_SHORT_BUFFER);
    }

    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_get_raw_public_key(key_pair->key, out->buffer + out->len, &remaining),
            "EVP_PKEY_get_raw_public_key",
            AWS_LS_CAL_ED25519)) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }
    if (remaining != s_public_key_size) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }
    out->len += s_public_key_size;

    return AWS_OP_SUCCESS;
#endif
}

int aws_ed25519_key_pair_get_public_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out) {
    AWS_PRECONDITION(key_pair);
    AWS_PRECONDITION(aws_byte_buf_is_valid(out));

    switch (format) {
        case AWS_CAL_ED25519_KEY_EXPORT_RAW:
            return s_ed25519_export_public_raw(key_pair, out);
        case AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64:
            return s_ed25519_export_public_openssh(key_pair, out);
        default:
            return aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_KEY_FORMAT);
    }
}

size_t aws_ed25519_key_pair_get_public_key_size_impl(enum aws_ed25519_key_export_format format) {
    switch (format) {
        case AWS_CAL_ED25519_KEY_EXPORT_RAW:
            return s_public_key_size;
        case AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64:
            return 68;
        default:
            aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_KEY_FORMAT);
            return 0;
    }
}

static struct aws_byte_cursor s_private_magic = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("openssh-key-v1");
static struct aws_byte_cursor s_private_none_literal = AWS_BYTE_CUR_INIT_FROM_STRING_LITERAL("none");

/**
 * Openssl only added helpers for this format in 3.0 so we are out of luck with lc and boringssl.
 * Hence lets just implement export ourselves.
 * Some of the spec features (encryption, comments) are not supported.
 * High level format is:
 * string "openssh-key-v1\0" #literal
 * string cipher #literal none since we dont support enc
 * string kdf #literal none since we dont support enc
 * string kdf options #empty since we dont support enc
 * u32 num keys
 * string public key #openssh encoded version
 * string private key blob
 * - u32 check #random num
 * - u32 check #same check repeated
 * - string "ssh-ed25519" #literal
 * - string raw pub key
 * - string raw priv key
 * - string comment # no comment for now
 * - padding to 8 bytes # just add bytes 1, 2, 3, ... until priv block is divisible by 8
 * Note: string is always u32 size followed by the data. all multibyte ints are in big-endian
 */
int s_ed25519_export_private_openssh(const struct aws_ed25519_key_pair_impl *key_pair, struct aws_byte_buf *out) {

    struct aws_byte_buf key_buf;
    aws_byte_buf_init(&key_buf, key_pair->allocator, 312);

    /* magic */
    if (aws_byte_buf_append(&key_buf, &s_private_magic) != AWS_OP_SUCCESS) {
        goto on_error;
    }
    if (!aws_byte_buf_write_u8(&key_buf, 0)) {
        aws_raise_error(AWS_ERROR_SHORT_BUFFER);
        goto on_error;
    }

    /* cipher name (we dont support it now, but still need to write out 0) */
    if (s_byte_buf_write_be32_with_err(&key_buf, 4) != AWS_OP_SUCCESS ||
        aws_byte_buf_append(&key_buf, &s_private_none_literal) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* kdf name (we dont support it now, but still need to write out 0) */
    if (s_byte_buf_write_be32_with_err(&key_buf, 4) != AWS_OP_SUCCESS ||
        aws_byte_buf_append(&key_buf, &s_private_none_literal) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* kdf options (we dont support it now, but still need to write out 0) */
    if (s_byte_buf_write_be32_with_err(&key_buf, 0) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* number of keys */
    if (s_byte_buf_write_be32_with_err(&key_buf, 1) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* encoded public key */
    const size_t pub_encoded_len = 4 /*id len*/ + 11 /* ssh-ed25519 literal */ + 4 /*key len*/ + 32 /* key */;
    if (s_byte_buf_write_be32_with_err(&key_buf, (uint32_t)pub_encoded_len) != AWS_OP_SUCCESS ||
        s_ed25519_openssh_encode_public_key(key_pair, &key_buf) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    size_t priv_block_len = 4 +                                          /* check1 */
                            4 +                                          /* check2 */
                            4 + s_key_type_literal.len +                 /* key type string */
                            4 + s_public_key_size +                      /* public key */
                            4 + s_private_key_size + s_public_key_size + /* private key (includes public) */
                            4 + 0; /* comment (0, since comment not currently supported) */

    /* pad block to the next multiple of 8 */
    size_t priv_block_padded_len = (priv_block_len + 7) & ~7;

    if (s_byte_buf_write_be32_with_err(&key_buf, (uint32_t)priv_block_padded_len) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    uint32_t check = 0;
    if (aws_device_random_u32(&check) != AWS_OP_SUCCESS) {
        aws_raise_error(AWS_ERROR_RANDOM_GEN_FAILED);
        goto on_error;
    }

    /* check (and yeah its written twice on purpose) */
    if (s_byte_buf_write_be32_with_err(&key_buf, check) != AWS_OP_SUCCESS ||
        s_byte_buf_write_be32_with_err(&key_buf, check) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* key type */
    if (s_byte_buf_write_be32_with_err(&key_buf, (uint32_t)s_key_type_literal.len) != AWS_OP_SUCCESS ||
        aws_byte_buf_append(&key_buf, &s_key_type_literal) != AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* public key (raw) */
    if (s_byte_buf_write_be32_with_err(&key_buf, (uint32_t)s_public_key_size) != AWS_OP_SUCCESS ||
        aws_ed25519_key_pair_get_public_key_impl(key_pair, AWS_CAL_ED25519_KEY_EXPORT_RAW, &key_buf) !=
            AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* private key - seed + pub (raw) */
    if (s_byte_buf_write_be32_with_err(&key_buf, (uint32_t)(s_private_key_size + s_public_key_size)) !=
            AWS_OP_SUCCESS ||
        aws_ed25519_key_pair_get_private_key_impl(key_pair, AWS_CAL_ED25519_KEY_EXPORT_RAW, &key_buf) !=
            AWS_OP_SUCCESS ||
        aws_ed25519_key_pair_get_public_key_impl(key_pair, AWS_CAL_ED25519_KEY_EXPORT_RAW, &key_buf) !=
            AWS_OP_SUCCESS) {
        goto on_error;
    }

    /* comment */
    if (s_byte_buf_write_be32_with_err(&key_buf, 0) != AWS_OP_SUCCESS) {
        aws_raise_error(AWS_ERROR_SHORT_BUFFER);
        goto on_error;
    }

    /* padding */
    for (uint8_t i = 1; i < (priv_block_padded_len - priv_block_len + 1); ++i) {
        if (!aws_byte_buf_write_u8(&key_buf, i)) {
            aws_raise_error(AWS_ERROR_SHORT_BUFFER);
            goto on_error;
        }
    }

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_buf(&key_buf);

    if (aws_base64_encode(&key_cur, out) != AWS_OP_SUCCESS) {
        aws_byte_buf_clean_up(&key_buf);
        return AWS_OP_ERR;
    }

    aws_byte_buf_clean_up(&key_buf);
    return AWS_OP_SUCCESS;

on_error:
    aws_byte_buf_clean_up(&key_buf);
    return AWS_OP_ERR;
}

int s_ed25519_export_private_raw(const struct aws_ed25519_key_pair_impl *key_pair, struct aws_byte_buf *out) {
#if defined(LIBCRYPTO_DOES_NOT_SUPPORT_ED25519)
    return aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM);
#else
    size_t remaining = out->capacity - out->len;
    if (remaining < s_private_key_size) {
        return aws_raise_error(AWS_ERROR_SHORT_BUFFER);
    }

    /**
     * RFC defines private key to be 64 bytes (seed + pub).
     * Old versions of openssl did return it that way, but at some point (seems to be around 1.1.1l) they switched to
     * just returning seed. So for consistency lets also return just the seed.
     * Which on older versions of openssl just means reading first 32 bytes.
     */
    remaining = s_private_key_size;

    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_get_raw_private_key(key_pair->key, out->buffer + out->len, &remaining),
            "EVP_PKEY_get_raw_private_key",
            AWS_LS_CAL_ED25519)) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }

    if (remaining != s_private_key_size) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }
    out->len += s_private_key_size;

    return AWS_OP_SUCCESS;
#endif
}

int aws_ed25519_key_pair_get_private_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out) {
    AWS_PRECONDITION(key_pair);
    AWS_PRECONDITION(aws_byte_buf_is_valid(out));

    switch (format) {
        case AWS_CAL_ED25519_KEY_EXPORT_RAW:
            return s_ed25519_export_private_raw(key_pair, out);
        case AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64:
            return s_ed25519_export_private_openssh(key_pair, out);
        default:
            return aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_KEY_FORMAT);
    }
}

size_t aws_ed25519_key_pair_get_private_key_size_impl(enum aws_ed25519_key_export_format format) {
    switch (format) {
        case AWS_CAL_ED25519_KEY_EXPORT_RAW:
            return s_private_key_size;
        case AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64:
            return 312;
        default:
            aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_KEY_FORMAT);
            return 0;
    }
}
