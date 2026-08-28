/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/mqtt/mqtt.h>
#include <aws/mqtt/private/v5/rate_limiters.h>

#include <aws/common/clock.h>
#include <aws/testing/aws_test_harness.h>

static uint64_t s_test_time = 0;
static int s_get_test_time(uint64_t *time) {
    *time = s_test_time;

    return AWS_OP_SUCCESS;
}

static int s_rate_limiter_token_bucket_init_invalid_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_rate_limiter_token_bucket token_bucket;

    struct aws_rate_limiter_token_bucket_options invalid_options1 = {
        .clock_fn = s_get_test_time,
        .tokens_per_second = 0,
        .maximum_token_count = 1,
    };

    ASSERT_FAILS(aws_rate_limiter_token_bucket_init(&token_bucket, &invalid_options1));

    struct aws_rate_limiter_token_bucket_options invalid_options2 = {
        .clock_fn = s_get_test_time,
        .tokens_per_second = 1000,
        .maximum_token_count = 0,
    };

    ASSERT_FAILS(aws_rate_limiter_token_bucket_init(&token_bucket, &invalid_options2));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rate_limiter_token_bucket_init_invalid, s_rate_limiter_token_bucket_init_invalid_fn)

static int s_rate_limiter_token_bucket_regeneration_integral_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_rate_limiter_token_bucket_options options = {
        .clock_fn = s_get_test_time,
        .tokens_per_second = 5,
        .maximum_token_count = 10,
    };

    struct aws_rate_limiter_token_bucket token_bucket;
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_init(&token_bucket, &options));

    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 0));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 1));
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_take_tokens(&token_bucket, 0));
    ASSERT_FAILS(aws_rate_limiter_token_bucket_take_tokens(&token_bucket, 1));
    ASSERT_INT_EQUALS(0, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 0));
    ASSERT_INT_EQUALS(
        2 * AWS_TIMESTAMP_NANOS, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 10));

    /* one second elapsed, should be able to take 5 tokens now */
    s_test_time = AWS_TIMESTAMP_NANOS;
    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 5));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 6));
    ASSERT_INT_EQUALS(AWS_TIMESTAMP_NANOS, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 10));
    ASSERT_INT_EQUALS(0, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 1));
    ASSERT_INT_EQUALS(0, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_take_tokens(&token_bucket, 5));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 1));
    ASSERT_FAILS(aws_rate_limiter_token_bucket_take_tokens(&token_bucket, 1));
    ASSERT_INT_EQUALS(
        2 * AWS_TIMESTAMP_NANOS, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 10));

    /* three more elapsed seconds, regen should be maxed but clamped */
    s_test_time += 3 * (uint64_t)AWS_TIMESTAMP_NANOS;
    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 10));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 11));
    ASSERT_INT_EQUALS(0, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 10));
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_take_tokens(&token_bucket, 10));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 1));
    ASSERT_FAILS(aws_rate_limiter_token_bucket_take_tokens(&token_bucket, 1));
    ASSERT_INT_EQUALS(
        2 * (uint64_t)AWS_TIMESTAMP_NANOS, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 10));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rate_limiter_token_bucket_regeneration_integral, s_rate_limiter_token_bucket_regeneration_integral_fn)

static int s_rate_limiter_token_bucket_regeneration_fractional_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_rate_limiter_token_bucket_options options = {
        .clock_fn = s_get_test_time,
        .initial_token_count = 2,
        .tokens_per_second = 3,
        .maximum_token_count = 20,
    };

    struct aws_rate_limiter_token_bucket token_bucket;
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_init(&token_bucket, &options));

    uint64_t initial_wait_for_3 = AWS_TIMESTAMP_NANOS / 3 + 1;
    uint64_t initial_wait_for_5 = AWS_TIMESTAMP_NANOS;
    uint64_t initial_wait_for_6 = AWS_TIMESTAMP_NANOS + AWS_TIMESTAMP_NANOS / 3 + 1;
    uint64_t initial_wait_for_7 = AWS_TIMESTAMP_NANOS + AWS_TIMESTAMP_NANOS / 3 * 2 + 1;
    uint64_t initial_wait_for_8 = 2 * AWS_TIMESTAMP_NANOS;
    uint64_t initial_wait_for_11 = 3 * (uint64_t)AWS_TIMESTAMP_NANOS;

    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 2));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(initial_wait_for_3, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(initial_wait_for_5, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));
    ASSERT_INT_EQUALS(initial_wait_for_6, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 6));
    ASSERT_INT_EQUALS(initial_wait_for_7, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 7));
    ASSERT_INT_EQUALS(initial_wait_for_8, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 8));

    /* one nanosecond elapsed, wait should shrink by 1 */
    s_test_time = 1;
    ASSERT_INT_EQUALS(
        initial_wait_for_3 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    /* idempotent check */
    ASSERT_INT_EQUALS(
        initial_wait_for_3 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(
        initial_wait_for_5 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));
    ASSERT_INT_EQUALS(
        initial_wait_for_6 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 6));
    ASSERT_INT_EQUALS(
        initial_wait_for_7 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 7));
    ASSERT_INT_EQUALS(
        initial_wait_for_8 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 8));

    s_test_time = 2;
    ASSERT_INT_EQUALS(
        initial_wait_for_3 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(
        initial_wait_for_5 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));
    ASSERT_INT_EQUALS(
        initial_wait_for_6 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 6));
    ASSERT_INT_EQUALS(
        initial_wait_for_7 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 7));
    ASSERT_INT_EQUALS(
        initial_wait_for_8 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 8));

    /* one nanosecond short of a token's worth of time, nothing should change */
    s_test_time = AWS_TIMESTAMP_NANOS / 3;
    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 2));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(
        initial_wait_for_3 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(
        initial_wait_for_5 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));
    ASSERT_INT_EQUALS(
        initial_wait_for_6 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 6));
    ASSERT_INT_EQUALS(
        initial_wait_for_7 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 7));
    ASSERT_INT_EQUALS(
        initial_wait_for_8 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 8));

    /* one more nanosecond, should give us a token */
    s_test_time += 1;
    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 3));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 4));
    ASSERT_INT_EQUALS(
        initial_wait_for_3 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(
        initial_wait_for_5 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));
    ASSERT_INT_EQUALS(
        initial_wait_for_6 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 6));
    ASSERT_INT_EQUALS(
        initial_wait_for_7 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 7));
    ASSERT_INT_EQUALS(
        initial_wait_for_8 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 8));

    /* now let's do multi-second plus fractional */
    s_test_time += (uint64_t)AWS_TIMESTAMP_NANOS * 2 + AWS_TIMESTAMP_NANOS / 2;
    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 10));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 11));
    ASSERT_INT_EQUALS(0, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 10));
    ASSERT_INT_EQUALS(
        initial_wait_for_11 - s_test_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 11));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rate_limiter_token_bucket_regeneration_fractional, s_rate_limiter_token_bucket_regeneration_fractional_fn)

#define REGENERATION_INTERVAL 9973

static int s_rate_limiter_token_bucket_fractional_iteration_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_rate_limiter_token_bucket_options options = {
        .clock_fn = s_get_test_time,
        .initial_token_count = 0,
        .tokens_per_second = 7,
        .maximum_token_count = 100,
    };

    s_test_time = 47;

    struct aws_rate_limiter_token_bucket token_bucket;
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_init(&token_bucket, &options));

    size_t iterations = 2 * AWS_TIMESTAMP_NANOS / REGENERATION_INTERVAL + 3;

    uint64_t expected_wait_time = (uint64_t)3 * AWS_TIMESTAMP_NANOS;

    for (size_t i = 0; i < iterations; ++i) {
        ASSERT_INT_EQUALS(expected_wait_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 21));

        s_test_time += REGENERATION_INTERVAL;
        expected_wait_time -= REGENERATION_INTERVAL;
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rate_limiter_token_bucket_fractional_iteration, s_rate_limiter_token_bucket_fractional_iteration_fn)

#define LARGE_REGENERATION_INTERVAL (43 * (uint64_t)AWS_TIMESTAMP_NANOS + AWS_TIMESTAMP_NANOS / 13)

static int s_rate_limiter_token_bucket_large_fractional_iteration_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_rate_limiter_token_bucket_options options = {
        .clock_fn = s_get_test_time,
        .initial_token_count = 0,
        .tokens_per_second = 7,
        .maximum_token_count = 100000,
    };

    s_test_time = 47;

    struct aws_rate_limiter_token_bucket token_bucket;
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_init(&token_bucket, &options));

    uint64_t expected_wait_time = aws_timestamp_convert(1001, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);

    while (expected_wait_time >= LARGE_REGENERATION_INTERVAL) {
        ASSERT_INT_EQUALS(
            expected_wait_time, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 7007));

        s_test_time += LARGE_REGENERATION_INTERVAL;
        expected_wait_time -= LARGE_REGENERATION_INTERVAL;
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rate_limiter_token_bucket_large_fractional_iteration,
    s_rate_limiter_token_bucket_large_fractional_iteration_fn)

#define TOKEN_REGENERATION_RATE_REAL 111111

static int s_rate_limiter_token_bucket_real_iteration_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_rate_limiter_token_bucket_options options = {
        .initial_token_count = 0,
        .tokens_per_second = TOKEN_REGENERATION_RATE_REAL,
        .maximum_token_count = 100,
    };

    struct aws_rate_limiter_token_bucket token_bucket;
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_init(&token_bucket, &options));

    uint64_t start_time = 0;
    aws_high_res_clock_get_ticks(&start_time);

    uint64_t tokens_taken = 0;
    while (tokens_taken < TOKEN_REGENERATION_RATE_REAL * 3) {
        uint64_t wait = aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 1);
        if (wait > 0) {
            aws_thread_current_sleep(wait);
        }

        if (!aws_rate_limiter_token_bucket_take_tokens(&token_bucket, 1)) {
            ++tokens_taken;
        }
    }

    uint64_t end_time = 0;
    aws_high_res_clock_get_ticks(&end_time);

    uint64_t elapsed_time = end_time - start_time;
    uint64_t expected_elapsed = aws_timestamp_convert(3, AWS_TIMESTAMP_SECS, AWS_TIMESTAMP_NANOS, NULL);

    ASSERT_TRUE(elapsed_time > (uint64_t)(expected_elapsed * .99));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rate_limiter_token_bucket_real_iteration, s_rate_limiter_token_bucket_real_iteration_fn)

static int s_rate_limiter_token_bucket_reset_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    struct aws_rate_limiter_token_bucket_options options = {
        .clock_fn = s_get_test_time,
        .initial_token_count = 2,
        .tokens_per_second = 3,
        .maximum_token_count = 20,
    };

    struct aws_rate_limiter_token_bucket token_bucket;
    ASSERT_SUCCESS(aws_rate_limiter_token_bucket_init(&token_bucket, &options));

    uint64_t initial_wait_for_3 = AWS_TIMESTAMP_NANOS / 3 + 1;
    uint64_t initial_wait_for_5 = AWS_TIMESTAMP_NANOS;

    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 2));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(initial_wait_for_3, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(initial_wait_for_5, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));

    s_test_time = AWS_TIMESTAMP_NANOS * 2 + 1;
    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 8));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 9));

    aws_rate_limiter_token_bucket_reset(&token_bucket);
    ASSERT_TRUE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 2));
    ASSERT_FALSE(aws_rate_limiter_token_bucket_can_take_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(initial_wait_for_3, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 3));
    ASSERT_INT_EQUALS(initial_wait_for_5, aws_rate_limiter_token_bucket_compute_wait_for_tokens(&token_bucket, 5));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rate_limiter_token_bucket_reset, s_rate_limiter_token_bucket_reset_fn)
