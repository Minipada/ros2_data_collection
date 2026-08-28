/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/cal/private/opensslcrypto_common.h>
#include <aws/cal/private/rsa.h>

#include <aws/cal/cal.h>
#include <aws/common/encoding.h>

#define OPENSSL_SUPPRESS_DEPRECATED
#include <openssl/evp.h>

#include <openssl/rsa.h>

struct lc_rsa_key_pair {
    struct aws_rsa_key_pair base;
    EVP_PKEY *key;
};

static void s_rsa_destroy_key(void *key_pair) {
    if (key_pair == NULL) {
        return;
    }

    struct aws_rsa_key_pair *base = key_pair;
    struct lc_rsa_key_pair *impl = base->impl;

    if (impl->key != NULL) {
        EVP_PKEY_free(impl->key);
    }

    aws_rsa_key_pair_base_clean_up(base);

    aws_mem_release(base->allocator, impl);
}

static int s_set_encryption_ctx_from_algo(EVP_PKEY_CTX *ctx, enum aws_rsa_encryption_algorithm algorithm) {
    if (algorithm == AWS_CAL_RSA_ENCRYPTION_PKCS1_5) {
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_PADDING), "EVP_PKEY_CTX_set_rsa_padding", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }

    } else if (algorithm == AWS_CAL_RSA_ENCRYPTION_OAEP_SHA256 || algorithm == AWS_CAL_RSA_ENCRYPTION_OAEP_SHA512) {
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_OAEP_PADDING),
                "EVP_PKEY_CTX_set_rsa_padding",
                AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }

        const EVP_MD *md = algorithm == AWS_CAL_RSA_ENCRYPTION_OAEP_SHA256 ? EVP_sha256() : EVP_sha512();
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_rsa_oaep_md(ctx, md), "EVP_PKEY_CTX_set_rsa_oaep_md", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }
    } else {
        return aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM);
    }

    return AWS_OP_SUCCESS;
}

static int s_rsa_encrypt(
    const struct aws_rsa_key_pair *key_pair,
    enum aws_rsa_encryption_algorithm algorithm,
    struct aws_byte_cursor plaintext,
    struct aws_byte_buf *out) {
    struct lc_rsa_key_pair *key_pair_impl = key_pair->impl;

    EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new(key_pair_impl->key, NULL);
    if (ctx == NULL) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }

    if (aws_reinterpret_lc_evp_error_as_crt(EVP_PKEY_encrypt_init(ctx), "EVP_PKEY_encrypt_init", AWS_LS_CAL_RSA)) {
        goto on_error;
    }

    if (s_set_encryption_ctx_from_algo(ctx, algorithm)) {
        goto on_error;
    }

    size_t needed_buffer_len = 0;
    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_encrypt(ctx, NULL, &needed_buffer_len, plaintext.ptr, plaintext.len),
            "EVP_PKEY_encrypt get length",
            AWS_LS_CAL_RSA)) {
        goto on_error;
    }

    size_t ct_len = out->capacity - out->len;
    if (needed_buffer_len > ct_len) {
        /*
         * OpenSSL 3 seems to no longer fail if the buffer is too short.
         * Instead it seems to write out enough data to fill the buffer and then
         * updates the out_len to full buffer. It does not seem to corrupt
         * memory after the buffer, but behavior is non-ideal.
         * Let get length needed for buffer from api first and then manually ensure that
         * buffer we have is big enough.
         */
        aws_raise_error(AWS_ERROR_SHORT_BUFFER);
        goto on_error;
    }

    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_encrypt(ctx, out->buffer + out->len, &ct_len, plaintext.ptr, plaintext.len),
            "EVP_PKEY_encrypt",
            AWS_LS_CAL_RSA)) {
        goto on_error;
    }
    out->len += ct_len;

    EVP_PKEY_CTX_free(ctx);
    return AWS_OP_SUCCESS;

on_error:
    EVP_PKEY_CTX_free(ctx);
    return AWS_OP_ERR;
}

static int s_rsa_decrypt(
    const struct aws_rsa_key_pair *key_pair,
    enum aws_rsa_encryption_algorithm algorithm,
    struct aws_byte_cursor ciphertext,
    struct aws_byte_buf *out) {
    struct lc_rsa_key_pair *key_pair_impl = key_pair->impl;

    EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new(key_pair_impl->key, NULL);
    if (ctx == NULL) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }

    if (aws_reinterpret_lc_evp_error_as_crt(EVP_PKEY_decrypt_init(ctx), "EVP_PKEY_decrypt_init", AWS_LS_CAL_RSA)) {
        goto on_error;
    }

    if (s_set_encryption_ctx_from_algo(ctx, algorithm)) {
        goto on_error;
    }

    size_t needed_buffer_len = 0;
    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_decrypt(ctx, NULL, &needed_buffer_len, ciphertext.ptr, ciphertext.len),
            "EVP_PKEY_decrypt get length",
            AWS_LS_CAL_RSA)) {
        goto on_error;
    }

    size_t ct_len = out->capacity - out->len;
    if (needed_buffer_len > ct_len) {
        /*
         * manual short buffer length check for OpenSSL 3.
         * refer to encrypt implementation for more details
         */
        aws_raise_error(AWS_ERROR_SHORT_BUFFER);
        goto on_error;
    }

    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_decrypt(ctx, out->buffer + out->len, &ct_len, ciphertext.ptr, ciphertext.len),
            "EVP_PKEY_decrypt",
            AWS_LS_CAL_RSA)) {
        goto on_error;
    }
    out->len += ct_len;

    EVP_PKEY_CTX_free(ctx);
    return AWS_OP_SUCCESS;

on_error:
    EVP_PKEY_CTX_free(ctx);
    return AWS_OP_ERR;
}

static int s_set_signature_ctx_from_algo(EVP_PKEY_CTX *ctx, enum aws_rsa_signature_algorithm algorithm) {
    if (algorithm == AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA256) {
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_PADDING), "EVP_PKEY_CTX_set_rsa_padding", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_signature_md(ctx, EVP_sha256()), "EVP_PKEY_CTX_set_signature_md", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }
    } else if (algorithm == AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1) {
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_PADDING), "EVP_PKEY_CTX_set_rsa_padding", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_signature_md(ctx, EVP_sha1()), "EVP_PKEY_CTX_set_signature_md", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }
    } else if (algorithm == AWS_CAL_RSA_SIGNATURE_PSS_SHA256) {
        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_PSS_PADDING),
                "EVP_PKEY_CTX_set_rsa_padding",
                AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }

#if defined(OPENSSL_IS_BORINGSSL) || OPENSSL_VERSION_NUMBER < 0x10100000L
        int saltlen = -1; /* RSA_PSS_SALTLEN_DIGEST not defined in BoringSSL and old versions of openssl */
#else
        int saltlen = RSA_PSS_SALTLEN_DIGEST;
#endif

        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_rsa_pss_saltlen(ctx, saltlen), "EVP_PKEY_CTX_set_rsa_pss_saltlen", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }

        if (aws_reinterpret_lc_evp_error_as_crt(
                EVP_PKEY_CTX_set_signature_md(ctx, EVP_sha256()), "EVP_PKEY_CTX_set_signature_md", AWS_LS_CAL_RSA)) {
            return AWS_OP_ERR;
        }
    } else {
        return aws_raise_error(AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM);
    }

    return AWS_OP_SUCCESS;
}

static int s_rsa_sign(
    const struct aws_rsa_key_pair *key_pair,
    enum aws_rsa_signature_algorithm algorithm,
    struct aws_byte_cursor digest,
    struct aws_byte_buf *out) {
    struct lc_rsa_key_pair *key_pair_impl = key_pair->impl;

    EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new(key_pair_impl->key, NULL);
    if (ctx == NULL) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }

    if (aws_reinterpret_lc_evp_error_as_crt(EVP_PKEY_sign_init(ctx), "EVP_PKEY_sign_init", AWS_LS_CAL_RSA)) {
        goto on_error;
    }

    if (s_set_signature_ctx_from_algo(ctx, algorithm)) {
        goto on_error;
    }

    size_t needed_buffer_len = 0;
    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_sign(ctx, NULL, &needed_buffer_len, digest.ptr, digest.len),
            "EVP_PKEY_sign get length",
            AWS_LS_CAL_RSA)) {
        goto on_error;
    }

    size_t ct_len = out->capacity - out->len;
    if (needed_buffer_len > ct_len) {
        /*
         * manual short buffer length check for OpenSSL 3.
         * refer to encrypt implementation for more details.
         * OpenSSL3 actually does throw an error here, but error code comes from
         * component that does not exist in OpenSSL 1.x. So check manually right
         * now and we can figure out how to handle it better, once we can
         * properly support OpenSSL 3.
         */
        aws_raise_error(AWS_ERROR_SHORT_BUFFER);
        goto on_error;
    }

    if (aws_reinterpret_lc_evp_error_as_crt(
            EVP_PKEY_sign(ctx, out->buffer + out->len, &ct_len, digest.ptr, digest.len),
            "EVP_PKEY_sign",
            AWS_LS_CAL_RSA)) {
        goto on_error;
    }
    out->len += ct_len;

    EVP_PKEY_CTX_free(ctx);
    return AWS_OP_SUCCESS;

on_error:
    EVP_PKEY_CTX_free(ctx);
    return AWS_OP_ERR;
}

static int s_rsa_verify(
    const struct aws_rsa_key_pair *key_pair,
    enum aws_rsa_signature_algorithm algorithm,
    struct aws_byte_cursor digest,
    struct aws_byte_cursor signature) {
    struct lc_rsa_key_pair *key_pair_impl = key_pair->impl;

    EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new(key_pair_impl->key, NULL);
    if (ctx == NULL) {
        return aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
    }

    if (aws_reinterpret_lc_evp_error_as_crt(EVP_PKEY_verify_init(ctx), "EVP_PKEY_verify_init", AWS_LS_CAL_RSA)) {
        goto on_error;
    }

    if (s_set_signature_ctx_from_algo(ctx, algorithm)) {
        goto on_error;
    }

    int error_code = EVP_PKEY_verify(ctx, signature.ptr, signature.len, digest.ptr, digest.len);
    EVP_PKEY_CTX_free(ctx);

    /* Verify errors slightly differently from the rest of evp functions.
     * 0 indicates signature does not pass verification, it's not necessarily an error. */
    if (error_code > 0) {
        return AWS_OP_SUCCESS;
    } else if (error_code == 0) {
        return aws_raise_error(AWS_ERROR_CAL_SIGNATURE_VALIDATION_FAILED);
    } else {
        return aws_reinterpret_lc_evp_error_as_crt(error_code, "EVP_PKEY_verify", AWS_LS_CAL_RSA);
    }

on_error:
    EVP_PKEY_CTX_free(ctx);
    return AWS_OP_ERR;
}

static struct aws_rsa_key_vtable s_rsa_key_pair_vtable = {
    .encrypt = s_rsa_encrypt,
    .decrypt = s_rsa_decrypt,
    .sign = s_rsa_sign,
    .verify = s_rsa_verify,
};

struct aws_rsa_key_pair *aws_rsa_key_pair_new_from_private_key_pkcs1_impl(
    struct aws_allocator *allocator,
    struct aws_byte_cursor key) {
    struct lc_rsa_key_pair *key_pair_impl = aws_mem_calloc(allocator, 1, sizeof(struct lc_rsa_key_pair));

    aws_ref_count_init(&key_pair_impl->base.ref_count, &key_pair_impl->base, s_rsa_destroy_key);
    key_pair_impl->base.impl = key_pair_impl;
    key_pair_impl->base.allocator = allocator;
    aws_byte_buf_init_copy_from_cursor(&key_pair_impl->base.priv, allocator, key);

    RSA *rsa = NULL;
    EVP_PKEY *private_key = NULL;

    if (d2i_RSAPrivateKey(&rsa, (const uint8_t **)&key.ptr, key.len) == NULL) {
        aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
        goto on_error;
    }

    private_key = EVP_PKEY_new();
    if (private_key == NULL || EVP_PKEY_assign_RSA(private_key, rsa) == 0) {
        RSA_free(rsa);
        aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
        goto on_error;
    }

    key_pair_impl->key = private_key;

    key_pair_impl->base.vtable = &s_rsa_key_pair_vtable;
    key_pair_impl->base.key_size_in_bits = EVP_PKEY_bits(key_pair_impl->key);

    return &key_pair_impl->base;

on_error:
    if (private_key) {
        EVP_PKEY_free(private_key);
    }

    s_rsa_destroy_key(&key_pair_impl->base);
    return NULL;
}

struct aws_rsa_key_pair *aws_rsa_key_pair_new_from_public_key_pkcs1_impl(
    struct aws_allocator *allocator,
    struct aws_byte_cursor key) {
    struct lc_rsa_key_pair *key_pair_impl = aws_mem_calloc(allocator, 1, sizeof(struct lc_rsa_key_pair));

    aws_ref_count_init(&key_pair_impl->base.ref_count, &key_pair_impl->base, s_rsa_destroy_key);
    key_pair_impl->base.impl = key_pair_impl;
    key_pair_impl->base.allocator = allocator;
    aws_byte_buf_init_copy_from_cursor(&key_pair_impl->base.pub, allocator, key);

    RSA *rsa = NULL;
    EVP_PKEY *public_key = NULL;

    if (d2i_RSAPublicKey(&rsa, (const uint8_t **)&key.ptr, key.len) == NULL) {
        aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
        goto on_error;
    }

    public_key = EVP_PKEY_new();
    if (public_key == NULL || EVP_PKEY_assign_RSA(public_key, rsa) == 0) {
        RSA_free(rsa);
        aws_raise_error(AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED);
        goto on_error;
    }

    key_pair_impl->key = public_key;

    key_pair_impl->base.vtable = &s_rsa_key_pair_vtable;
    key_pair_impl->base.key_size_in_bits = EVP_PKEY_bits(key_pair_impl->key);

    return &key_pair_impl->base;

on_error:
    if (public_key) {
        EVP_PKEY_free(public_key);
    }
    s_rsa_destroy_key(&key_pair_impl->base);
    return NULL;
}
