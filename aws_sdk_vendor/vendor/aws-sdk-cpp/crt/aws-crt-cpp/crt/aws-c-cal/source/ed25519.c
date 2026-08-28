/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/cal/ed25519.h>
#include <aws/common/ref_count.h>

struct aws_ed25519_key_pair_impl;

#ifndef BYO_CRYPTO

extern struct aws_ed25519_key_pair_impl *aws_ed25519_key_pair_new_generate_impl(struct aws_allocator *allocator);

extern void aws_ed25519_key_pair_destroy_impl(struct aws_ed25519_key_pair_impl *key_pair_impl);

extern int aws_ed25519_key_pair_get_public_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out);

extern size_t aws_ed25519_key_pair_get_public_key_size_impl(enum aws_ed25519_key_export_format format);

extern int aws_ed25519_key_pair_get_private_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out);

extern size_t aws_ed25519_key_pair_get_private_key_size_impl(enum aws_ed25519_key_export_format format);

#else /* BYO_CRYPTO */

struct aws_ed25519_key_pair_impl *aws_ed25519_key_pair_new_generate_impl(struct aws_allocator *allocator) {
    (void)allocator;
    abort();
}

void aws_ed25519_key_pair_destroy_impl(struct aws_ed25519_key_pair_impl *key_pair_impl) {
    (void)key_pair_impl;
    abort();
}

int aws_ed25519_key_pair_get_public_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out) {
    (void)key_pair;
    (void)format;
    (void)out;
    abort();
}

size_t aws_ed25519_key_pair_get_public_key_size_impl(enum aws_ed25519_key_export_format format) {
    (void)format;
    abort();
}

int aws_ed25519_key_pair_get_private_key_impl(
    const struct aws_ed25519_key_pair_impl *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out) {
    (void)key_pair;
    (void)format;
    (void)out;
    abort();
}

size_t aws_ed25519_key_pair_get_private_key_size_impl(enum aws_ed25519_key_export_format format) {
    (void)format;
    abort();
}

#endif /* BYO_CRYPTO */

struct aws_ed25519_key_pair {
    struct aws_allocator *allocator;
    struct aws_ref_count ref_count;

    struct aws_ed25519_key_pair_impl *key;
};

static void s_ed25519_destroy_key(void *key_pair) {
    if (key_pair == NULL) {
        return;
    }

    struct aws_ed25519_key_pair *ed25519_key_pair = (struct aws_ed25519_key_pair *)(key_pair);

    if (ed25519_key_pair->key != NULL) {
        aws_ed25519_key_pair_destroy_impl(ed25519_key_pair->key);
    }

    aws_mem_release(ed25519_key_pair->allocator, ed25519_key_pair);
}

struct aws_ed25519_key_pair *aws_ed25519_key_pair_new_generate(struct aws_allocator *allocator) {

    struct aws_ed25519_key_pair_impl *key_impl = aws_ed25519_key_pair_new_generate_impl(allocator);

    if (key_impl == NULL) {
        return NULL;
    }

    struct aws_ed25519_key_pair *key_pair = aws_mem_calloc(allocator, 1, sizeof(struct aws_ed25519_key_pair));

    aws_ref_count_init(&key_pair->ref_count, key_pair, s_ed25519_destroy_key);
    key_pair->allocator = allocator;
    key_pair->key = key_impl;

    return key_pair;
}

struct aws_ed25519_key_pair *aws_ed25519_key_pair_acquire(struct aws_ed25519_key_pair *key_pair) {
    if (key_pair != NULL) {
        aws_ref_count_acquire(&key_pair->ref_count);
    }
    return key_pair;
}

struct aws_ed25519_key_pair *aws_ed25519_key_pair_release(struct aws_ed25519_key_pair *key_pair) {
    if (key_pair != NULL) {
        aws_ref_count_release(&key_pair->ref_count);
    }
    return NULL;
}

int aws_ed25519_key_pair_get_public_key(
    const struct aws_ed25519_key_pair *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out) {
    return aws_ed25519_key_pair_get_public_key_impl(key_pair->key, format, out);
}

size_t aws_ed25519_key_pair_get_public_key_size(enum aws_ed25519_key_export_format format) {
    return aws_ed25519_key_pair_get_public_key_size_impl(format);
}

int aws_ed25519_key_pair_get_private_key(
    const struct aws_ed25519_key_pair *key_pair,
    enum aws_ed25519_key_export_format format,
    struct aws_byte_buf *out) {
    return aws_ed25519_key_pair_get_private_key_impl(key_pair->key, format, out);
}

size_t aws_ed25519_key_pair_get_private_key_size(enum aws_ed25519_key_export_format format) {
    return aws_ed25519_key_pair_get_private_key_size_impl(format);
}
