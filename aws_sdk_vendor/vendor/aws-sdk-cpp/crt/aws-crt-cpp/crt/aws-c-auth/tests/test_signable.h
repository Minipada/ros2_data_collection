#ifndef AWS_AUTH_TEST_SIGNABLE_H
#define AWS_AUTH_TEST_SIGNABLE_H

/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/auth/auth.h>

struct aws_byte_cursor;
struct aws_input_stream;
struct aws_signable;
struct aws_signable_property_list_pair;

struct aws_signable *aws_signable_new_test(
    struct aws_allocator *allocator,
    struct aws_byte_cursor *method,
    struct aws_byte_cursor *uri,
    struct aws_signable_property_list_pair *headers,
    size_t header_count,
    struct aws_input_stream *body_stream);

#endif /* AWS_AUTH_TEST_SIGNABLE_H */
