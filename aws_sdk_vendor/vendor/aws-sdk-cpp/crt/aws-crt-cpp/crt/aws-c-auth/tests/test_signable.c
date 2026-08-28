/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_signable.h"

#include <aws/auth/signable.h>
#include <aws/common/string.h>

struct aws_signable_test_impl {
    struct aws_input_stream *payload;
    struct aws_array_list headers;
    struct aws_byte_cursor uri;
    struct aws_byte_cursor method;
};

static int s_aws_signable_test_get_property(
    const struct aws_signable *signable,
    const struct aws_string *name,
    struct aws_byte_cursor *out_value) {

    struct aws_signable_test_impl *impl = signable->impl;

    AWS_ZERO_STRUCT(*out_value);

    if (aws_string_eq(name, g_aws_http_uri_property_name)) {
        *out_value = impl->uri;
    } else if (aws_string_eq(name, g_aws_http_method_property_name)) {
        *out_value = impl->method;
    } else {
        return aws_raise_error(AWS_ERROR_INVALID_ARGUMENT);
    }

    return AWS_OP_SUCCESS;
}

static int s_aws_signable_test_get_property_list(
    const struct aws_signable *signable,
    const struct aws_string *name,
    struct aws_array_list **out_list) {

    struct aws_signable_test_impl *impl = signable->impl;

    *out_list = NULL;

    if (aws_string_eq(name, g_aws_http_headers_property_list_name)) {
        *out_list = &impl->headers;
    } else {
        return aws_raise_error(AWS_ERROR_UNSUPPORTED_OPERATION);
    }

    return AWS_OP_SUCCESS;
}

static int s_aws_signable_test_get_payload_stream(
    const struct aws_signable *signable,
    struct aws_input_stream **out_input_stream) {
    struct aws_signable_test_impl *impl = signable->impl;

    *out_input_stream = impl->payload;

    return AWS_OP_SUCCESS;
}

static void s_aws_signable_test_destroy(struct aws_signable *signable) {
    if (signable == NULL) {
        return;
    }

    struct aws_signable_test_impl *impl = signable->impl;
    if (impl != NULL) {
        aws_array_list_clean_up(&impl->headers);
    }

    aws_mem_release(signable->allocator, signable);
}

static struct aws_signable_vtable s_signable_test_vtable = {
    .get_property = s_aws_signable_test_get_property,
    .get_property_list = s_aws_signable_test_get_property_list,
    .get_payload_stream = s_aws_signable_test_get_payload_stream,
    .destroy = s_aws_signable_test_destroy,
};

struct aws_signable *aws_signable_new_test(
    struct aws_allocator *allocator,
    struct aws_byte_cursor *method,
    struct aws_byte_cursor *uri,
    struct aws_signable_property_list_pair *headers,
    size_t header_count,
    struct aws_input_stream *body_stream) {

    struct aws_signable *signable = NULL;
    struct aws_signable_test_impl *impl = NULL;
    aws_mem_acquire_many(
        allocator, 2, &signable, sizeof(struct aws_signable), &impl, sizeof(struct aws_signable_test_impl));

    AWS_ZERO_STRUCT(*signable);
    AWS_ZERO_STRUCT(*impl);

    signable->allocator = allocator;
    signable->vtable = &s_signable_test_vtable;
    signable->impl = impl;

    if (aws_array_list_init_dynamic(
            &impl->headers, allocator, header_count, sizeof(struct aws_signable_property_list_pair))) {
        goto on_error;
    }

    for (size_t i = 0; i < header_count; ++i) {
        aws_array_list_push_back(&impl->headers, &headers[i]);
    }

    impl->payload = body_stream;
    impl->method = *method;
    impl->uri = *uri;

    return signable;

on_error:

    aws_signable_destroy(signable);

    return NULL;
}
