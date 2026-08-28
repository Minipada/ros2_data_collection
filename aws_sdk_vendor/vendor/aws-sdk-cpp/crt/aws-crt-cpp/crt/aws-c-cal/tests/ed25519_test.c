/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/cal/ed25519.h>
#include <aws/common/platform.h>

#include <aws/testing/aws_test_harness.h>

#include "test_case_helper.h"

#if defined(AWS_OS_LINUX)
#    include <aws/cal/private/opensslcrypto_common.h>
#endif

static int s_ed25519_key_pair_generate_test(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    aws_cal_library_test_init(allocator);

    struct aws_ed25519_key_pair *pair = aws_ed25519_key_pair_new_generate(allocator);

    if (pair == NULL && aws_last_error() == AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM) {
#if defined(AWS_USE_LIBCRYPTO_TO_SUPPORT_ED25519_EVERYWHERE)
        ASSERT_TRUE(false);
#endif

#if defined(AWS_OS_LINUX)
#    if defined(OPENSSL_IS_OPENSSL)
#        if OPENSSL_VERSION_NUMBER >= 0x10101000L
        ASSERT_TRUE(false);
#        endif
#    endif
#endif

        return AWS_OP_SKIP;
    }

    ASSERT_NOT_NULL(pair);

    struct aws_byte_buf buf_pub_ssh;
    aws_byte_buf_init(
        &buf_pub_ssh, allocator, aws_ed25519_key_pair_get_public_key_size(AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64));

    ASSERT_SUCCESS(aws_ed25519_key_pair_get_public_key(pair, AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64, &buf_pub_ssh));
    ASSERT_UINT_EQUALS(buf_pub_ssh.len, 68);

    struct aws_byte_buf buf_pub_raw;
    aws_byte_buf_init(
        &buf_pub_raw, allocator, aws_ed25519_key_pair_get_public_key_size(AWS_CAL_ED25519_KEY_EXPORT_RAW));

    ASSERT_SUCCESS(aws_ed25519_key_pair_get_public_key(pair, AWS_CAL_ED25519_KEY_EXPORT_RAW, &buf_pub_raw));
    ASSERT_UINT_EQUALS(buf_pub_raw.len, 32);

    struct aws_byte_buf buf_priv_ssh;
    aws_byte_buf_init(
        &buf_priv_ssh, allocator, aws_ed25519_key_pair_get_private_key_size(AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64));

    ASSERT_SUCCESS(aws_ed25519_key_pair_get_private_key(pair, AWS_CAL_ED25519_KEY_EXPORT_OPENSSH_B64, &buf_priv_ssh));
    ASSERT_UINT_EQUALS(buf_priv_ssh.len, 312);

    struct aws_byte_buf buf_priv_raw;
    aws_byte_buf_init(
        &buf_priv_raw, allocator, aws_ed25519_key_pair_get_private_key_size(AWS_CAL_ED25519_KEY_EXPORT_RAW));

    ASSERT_SUCCESS(aws_ed25519_key_pair_get_private_key(pair, AWS_CAL_ED25519_KEY_EXPORT_RAW, &buf_priv_raw));
    ASSERT_UINT_EQUALS(buf_priv_raw.len, 32);

    ASSERT_NOT_NULL(pair);
    aws_ed25519_key_pair_release(pair);

    aws_byte_buf_clean_up(&buf_pub_ssh);
    aws_byte_buf_clean_up(&buf_priv_ssh);
    aws_byte_buf_clean_up(&buf_pub_raw);
    aws_byte_buf_clean_up(&buf_priv_raw);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(ed25519_key_pair_generate_test, s_ed25519_key_pair_generate_test)
