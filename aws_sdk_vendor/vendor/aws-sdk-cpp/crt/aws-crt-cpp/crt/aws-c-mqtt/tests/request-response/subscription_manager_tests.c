/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/array_list.h>
#include <aws/mqtt/private/request-response/protocol_adapter.h>
#include <aws/mqtt/private/request-response/subscription_manager.h>

#include <aws/testing/aws_test_harness.h>

enum aws_protocol_adapter_api_type {
    PAAT_SUBSCRIBE,
    PAAT_UNSUBSCRIBE,
};

struct aws_protocol_adapter_api_record {
    enum aws_protocol_adapter_api_type type;

    struct aws_byte_buf topic_filter;
    struct aws_byte_cursor topic_filter_cursor;

    uint32_t timeout;
};

static void s_aws_protocol_adapter_api_record_init_from_subscribe(
    struct aws_protocol_adapter_api_record *record,
    struct aws_allocator *allocator,
    struct aws_byte_cursor topic_filter,
    uint32_t timeout) {
    AWS_ZERO_STRUCT(*record);
    record->type = PAAT_SUBSCRIBE;
    record->timeout = timeout;

    aws_byte_buf_init_copy_from_cursor(&record->topic_filter, allocator, topic_filter);
    record->topic_filter_cursor = aws_byte_cursor_from_buf(&record->topic_filter);
}

static void s_aws_protocol_adapter_api_record_init_from_unsubscribe(
    struct aws_protocol_adapter_api_record *record,
    struct aws_allocator *allocator,
    struct aws_byte_cursor topic_filter,
    uint32_t timeout) {
    AWS_ZERO_STRUCT(*record);
    record->type = PAAT_UNSUBSCRIBE;
    record->timeout = timeout;

    aws_byte_buf_init_copy_from_cursor(&record->topic_filter, allocator, topic_filter);
    record->topic_filter_cursor = aws_byte_cursor_from_buf(&record->topic_filter);
}

static void s_aws_protocol_adapter_api_record_clean_up(struct aws_protocol_adapter_api_record *record) {
    aws_byte_buf_clean_up(&record->topic_filter);
}

struct aws_mqtt_protocol_adapter_mock_impl {
    struct aws_allocator *allocator;
    struct aws_mqtt_protocol_adapter base;

    struct aws_array_list api_records;
    bool is_connected;
    size_t subscribe_count;
};

static void s_aws_mqtt_protocol_adapter_mock_destroy(void *impl) {
    struct aws_mqtt_protocol_adapter_mock_impl *adapter = impl;

    size_t record_count = aws_array_list_length(&adapter->api_records);
    for (size_t i = 0; i < record_count; ++i) {
        struct aws_protocol_adapter_api_record *record = NULL;
        aws_array_list_get_at_ptr(&adapter->api_records, (void **)&record, i);

        s_aws_protocol_adapter_api_record_clean_up(record);
    }

    aws_array_list_clean_up(&adapter->api_records);

    aws_mem_release(adapter->allocator, adapter);
}

static int s_aws_mqtt_protocol_adapter_mock_subscribe(
    void *impl,
    struct aws_protocol_adapter_subscribe_options *options) {
    struct aws_mqtt_protocol_adapter_mock_impl *adapter = impl;

    struct aws_protocol_adapter_api_record record;
    s_aws_protocol_adapter_api_record_init_from_subscribe(
        &record, adapter->allocator, options->topic_filter, options->ack_timeout_seconds);

    aws_array_list_push_back(&adapter->api_records, &record);

    return AWS_OP_SUCCESS;
}

static int s_aws_mqtt_protocol_adapter_mock_unsubscribe(
    void *impl,
    struct aws_protocol_adapter_unsubscribe_options *options) {
    struct aws_mqtt_protocol_adapter_mock_impl *adapter = impl;

    struct aws_protocol_adapter_api_record record;
    s_aws_protocol_adapter_api_record_init_from_unsubscribe(
        &record, adapter->allocator, options->topic_filter, options->ack_timeout_seconds);

    aws_array_list_push_back(&adapter->api_records, &record);

    return AWS_OP_SUCCESS;
}

static bool s_aws_mqtt_protocol_adapter_mock_is_connected(void *impl) {
    struct aws_mqtt_protocol_adapter_mock_impl *adapter = impl;

    return adapter->is_connected;
}

static struct aws_mqtt_protocol_adapter_vtable s_protocol_adapter_mock_vtable = {
    .aws_mqtt_protocol_adapter_destroy_fn = s_aws_mqtt_protocol_adapter_mock_destroy,
    .aws_mqtt_protocol_adapter_subscribe_fn = s_aws_mqtt_protocol_adapter_mock_subscribe,
    .aws_mqtt_protocol_adapter_unsubscribe_fn = s_aws_mqtt_protocol_adapter_mock_unsubscribe,
    .aws_mqtt_protocol_adapter_publish_fn = NULL,
    .aws_mqtt_protocol_adapter_is_connected_fn = s_aws_mqtt_protocol_adapter_mock_is_connected,
};

static struct aws_mqtt_protocol_adapter *s_aws_mqtt_mock_protocol_adapter_new(
    struct aws_allocator *allocator,
    const struct aws_mqtt_protocol_adapter_vtable *vtable,
    bool is_connected) {
    struct aws_mqtt_protocol_adapter_mock_impl *adapter =
        aws_mem_calloc(allocator, 1, sizeof(struct aws_mqtt_protocol_adapter_mock_impl));

    adapter->allocator = allocator;
    adapter->base.impl = adapter;
    if (vtable != NULL) {
        adapter->base.vtable = vtable;
    } else {
        adapter->base.vtable = &s_protocol_adapter_mock_vtable;
    }
    adapter->is_connected = is_connected;
    aws_array_list_init_dynamic(&adapter->api_records, allocator, 10, sizeof(struct aws_protocol_adapter_api_record));

    return &adapter->base;
}

static bool s_protocol_adapter_api_records_equal(
    struct aws_protocol_adapter_api_record *record1,
    struct aws_protocol_adapter_api_record *record2) {
    if (record1->type != record2->type) {
        return false;
    }

    if (record1->timeout != record2->timeout) {
        return false;
    }

    if (!aws_byte_cursor_eq(&record1->topic_filter_cursor, &record2->topic_filter_cursor)) {
        return false;
    }

    return true;
}

static bool s_api_records_contains_record(
    struct aws_mqtt_protocol_adapter *protocol_adapter,
    struct aws_protocol_adapter_api_record *expected_record) {

    struct aws_mqtt_protocol_adapter_mock_impl *adapter = protocol_adapter->impl;

    size_t record_count = aws_array_list_length(&adapter->api_records);
    for (size_t i = 0; i < record_count; ++i) {
        struct aws_protocol_adapter_api_record *actual_record = NULL;
        aws_array_list_get_at_ptr(&adapter->api_records, (void **)&actual_record, i);

        if (s_protocol_adapter_api_records_equal(expected_record, actual_record)) {
            return true;
        }
    }

    return false;
}

static bool s_api_records_contains_range(
    struct aws_mqtt_protocol_adapter *protocol_adapter,
    size_t range_start,
    size_t range_end,
    struct aws_protocol_adapter_api_record *expected_records) {
    struct aws_mqtt_protocol_adapter_mock_impl *adapter = protocol_adapter->impl;

    size_t record_count = aws_array_list_length(&adapter->api_records);
    if (record_count < range_end) {
        return false;
    }

    for (size_t i = range_start; i < range_end; ++i) {
        struct aws_protocol_adapter_api_record *expected_record = &expected_records[i - range_start];

        struct aws_protocol_adapter_api_record *actual_record = NULL;
        aws_array_list_get_at_ptr(&adapter->api_records, (void **)&actual_record, i);

        if (!s_protocol_adapter_api_records_equal(expected_record, actual_record)) {
            return false;
        }
    }

    return true;
}

static bool s_api_records_equals(
    struct aws_mqtt_protocol_adapter *protocol_adapter,
    size_t record_count,
    struct aws_protocol_adapter_api_record *expected_records) {
    struct aws_mqtt_protocol_adapter_mock_impl *adapter = protocol_adapter->impl;

    return aws_array_list_length(&adapter->api_records) == record_count &&
           s_api_records_contains_range(protocol_adapter, 0, record_count, expected_records);
}

//////////////////////////////////////////////////////////////

struct aws_subscription_status_record {
    enum aws_rr_subscription_event_type type;
    struct aws_byte_buf topic_filter;
    struct aws_byte_cursor topic_filter_cursor;
    uint64_t operation_id;
};

static void s_aws_subscription_status_record_init_from_event(
    struct aws_subscription_status_record *record,
    struct aws_allocator *allocator,
    const struct aws_rr_subscription_status_event *event) {
    AWS_ZERO_STRUCT(*record);
    record->type = event->type;
    record->operation_id = event->operation_id;

    aws_byte_buf_init_copy_from_cursor(&record->topic_filter, allocator, event->topic_filter);
    record->topic_filter_cursor = aws_byte_cursor_from_buf(&record->topic_filter);
}

static void s_aws_subscription_status_record_clean_up(struct aws_subscription_status_record *record) {
    aws_byte_buf_clean_up(&record->topic_filter);
}

//////////////////////////////////////////////////////////////

struct aws_subscription_manager_test_fixture {
    struct aws_allocator *allocator;

    struct aws_mqtt_protocol_adapter *mock_protocol_adapter;
    struct aws_rr_subscription_manager subscription_manager;

    struct aws_array_list subscription_status_records;
};

static void s_aws_rr_subscription_status_event_test_callback_fn(
    const struct aws_rr_subscription_status_event *event,
    void *userdata) {
    struct aws_subscription_manager_test_fixture *fixture = userdata;

    struct aws_subscription_status_record record;
    s_aws_subscription_status_record_init_from_event(&record, fixture->allocator, event);

    aws_array_list_push_back(&fixture->subscription_status_records, &record);
}

struct aws_subscription_manager_test_fixture_options {
    uint32_t operation_timeout_seconds;
    size_t max_request_response_subscriptions;
    size_t max_streaming_subscriptions;
    bool start_connected;
    const struct aws_mqtt_protocol_adapter_vtable *adapter_vtable;
};

static const uint32_t DEFAULT_SM_TEST_TIMEOUT = 5;

static int s_aws_subscription_manager_test_fixture_init(
    struct aws_subscription_manager_test_fixture *fixture,
    struct aws_allocator *allocator,
    const struct aws_subscription_manager_test_fixture_options *options) {
    AWS_ZERO_STRUCT(*fixture);

    struct aws_subscription_manager_test_fixture_options default_options = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = DEFAULT_SM_TEST_TIMEOUT,
        .start_connected = true,
    };

    if (options == NULL) {
        options = &default_options;
    }

    fixture->allocator = allocator;
    fixture->mock_protocol_adapter =
        s_aws_mqtt_mock_protocol_adapter_new(allocator, options->adapter_vtable, options->start_connected);

    aws_array_list_init_dynamic(
        &fixture->subscription_status_records, allocator, 10, sizeof(struct aws_subscription_status_record));

    struct aws_rr_subscription_manager_options subscription_manager_options = {
        .max_request_response_subscriptions = options->max_request_response_subscriptions,
        .max_streaming_subscriptions = options->max_streaming_subscriptions,
        .operation_timeout_seconds = options->operation_timeout_seconds,
        .subscription_status_callback = s_aws_rr_subscription_status_event_test_callback_fn,
        .userdata = fixture};
    aws_rr_subscription_manager_init(
        &fixture->subscription_manager, allocator, fixture->mock_protocol_adapter, &subscription_manager_options);

    return AWS_OP_SUCCESS;
}

static void s_aws_subscription_manager_test_fixture_clean_up(struct aws_subscription_manager_test_fixture *fixture) {

    aws_rr_subscription_manager_clean_up(&fixture->subscription_manager);
    aws_mqtt_protocol_adapter_destroy(fixture->mock_protocol_adapter);

    size_t record_count = aws_array_list_length(&fixture->subscription_status_records);
    for (size_t i = 0; i < record_count; ++i) {
        struct aws_subscription_status_record *record = NULL;
        aws_array_list_get_at_ptr(&fixture->subscription_status_records, (void **)&record, i);

        s_aws_subscription_status_record_clean_up(record);
    }

    aws_array_list_clean_up(&fixture->subscription_status_records);
}

static bool s_find_subscription_event_record(
    struct aws_subscription_manager_test_fixture *fixture,
    struct aws_subscription_status_record *expected_record,
    size_t *index) {

    size_t record_count = aws_array_list_length(&fixture->subscription_status_records);
    for (size_t i = 0; i < record_count; ++i) {

        struct aws_subscription_status_record *actual_record = NULL;
        aws_array_list_get_at_ptr(&fixture->subscription_status_records, (void **)&actual_record, i);

        if (expected_record->type != actual_record->type) {
            continue;
        }

        if (expected_record->operation_id != actual_record->operation_id) {
            continue;
        }

        if (!aws_byte_cursor_eq(&expected_record->topic_filter_cursor, &actual_record->topic_filter_cursor)) {
            continue;
        }

        *index = i;
        return true;
    }

    return false;
}

static bool s_contains_subscription_event_record(
    struct aws_subscription_manager_test_fixture *fixture,
    struct aws_subscription_status_record *record) {
    size_t index = 0;
    return s_find_subscription_event_record(fixture, record, &index);
}

static bool s_contains_subscription_event_records(
    struct aws_subscription_manager_test_fixture *fixture,
    size_t record_count,
    struct aws_subscription_status_record *records) {
    for (size_t i = 0; i < record_count; ++i) {
        if (!s_contains_subscription_event_record(fixture, &records[i])) {
            return false;
        }
    }

    return true;
}

static bool s_contains_subscription_event_sequential_records(
    struct aws_subscription_manager_test_fixture *fixture,
    size_t record_count,
    struct aws_subscription_status_record *records) {
    size_t previous_index = 0;
    for (size_t i = 0; i < record_count; ++i) {
        size_t index = 0;
        if (!s_find_subscription_event_record(fixture, &records[i], &index)) {
            return false;
        }

        if (i > 0) {
            if (index <= previous_index) {
                return false;
            }
        }
        previous_index = index;
    }

    return true;
}

static char s_hello_world1[] = "hello/world1";
static struct aws_byte_cursor s_hello_world1_cursor = {
    .ptr = (uint8_t *)s_hello_world1,
    .len = AWS_ARRAY_SIZE(s_hello_world1) - 1,
};

static char s_hello_world2[] = "hello/world2";
static struct aws_byte_cursor s_hello_world2_cursor = {
    .ptr = (uint8_t *)s_hello_world2,
    .len = AWS_ARRAY_SIZE(s_hello_world2) - 1,
};

static char s_hello_world3[] = "hello/world3";
static struct aws_byte_cursor s_hello_world3_cursor = {
    .ptr = (uint8_t *)s_hello_world3,
    .len = AWS_ARRAY_SIZE(s_hello_world3) - 1,
};

/*
 * Verify: Acquiring a new subscription triggers a protocol client subscribe and returns SUBSCRIBING
 */
static int s_rrsm_acquire_subscribing_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world3_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 1, expected_subscribes));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    struct aws_rr_acquire_subscription_options acquire3_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world3_cursor,
        .topic_filter_count = 1,
        .operation_id = 3,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire3_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 3, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_subscribing, s_rrsm_acquire_subscribing_fn)

/*
 * Verify: Acquiring a new multi-topic-filter subscription triggers two protocol client subscribes and returns
 * SUBSCRIBING
 */
static int s_rrsm_acquire_multi_subscribing_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_byte_cursor multi_filters[] = {
        s_hello_world1_cursor,
        s_hello_world2_cursor,
    };

    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_filters,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_filters),
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_multi_subscribing, s_rrsm_acquire_multi_subscribing_fn)

/*
 * Verify: Acquiring an existing, incomplete subscription does not trigger a protocol client subscribe and returns
 * SUBSCRIBING
 */
static int s_rrsm_acquire_existing_subscribing_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 1, expected_subscribes));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    struct aws_rr_acquire_subscription_options reacquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 3,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &reacquire1_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    struct aws_rr_acquire_subscription_options reacquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 4,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &reacquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_existing_subscribing, s_rrsm_acquire_existing_subscribing_fn)

/*
 * Verify: Acquiring multiple existing, incomplete subscriptions does not trigger a protocol client subscribe and
 * returns SUBSCRIBING
 */
static int s_rrsm_acquire_multi_existing_subscribing_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_byte_cursor multi_subs[] = {
        s_hello_world1_cursor,
        s_hello_world2_cursor,
    };
    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));
    ASSERT_TRUE(
        s_api_records_equals(fixture.mock_protocol_adapter, AWS_ARRAY_SIZE(expected_subscribes), expected_subscribes));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_multi_existing_subscribing, s_rrsm_acquire_multi_existing_subscribing_fn)

/*
 * Verify: A two-subscription acquire where one is already subscribing triggers a single subscribe and returns
 * SUBSCRIBING
 */
static int s_rrsm_acquire_multi_partially_subscribed_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_byte_cursor multi_subs[] = {
        s_hello_world1_cursor,
        s_hello_world2_cursor,
    };

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 1, expected_subscribes));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_multi_partially_subscribed, s_rrsm_acquire_multi_partially_subscribed_fn)

/*
 * Verify: Acquiring an existing, completed request subscription does not trigger a protocol client subscribe and
 * returns SUBSCRIBED.  Verify request and streaming subscription events are emitted.
 */
static int s_rrsm_acquire_existing_subscribed_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 1, expected_subscribes));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    struct aws_protocol_adapter_subscription_event successful_subscription1_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription1_event);

    struct aws_subscription_status_record expected_subscription_events[] = {
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 1,
        },
        {
            .type = ARRSET_STREAMING_SUBSCRIPTION_ESTABLISHED,
            .topic_filter_cursor = s_hello_world2_cursor,
            .operation_id = 2,
        }};
    ASSERT_TRUE(s_contains_subscription_event_sequential_records(&fixture, 1, expected_subscription_events));

    struct aws_protocol_adapter_subscription_event successful_subscription2_event = {
        .topic_filter = s_hello_world2_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription2_event);

    ASSERT_TRUE(s_contains_subscription_event_sequential_records(&fixture, 2, expected_subscription_events));

    struct aws_rr_acquire_subscription_options reacquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 3,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBED, aws_rr_subscription_manager_acquire_subscription(manager, &reacquire1_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    struct aws_rr_acquire_subscription_options reacquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 4,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBED, aws_rr_subscription_manager_acquire_subscription(manager, &reacquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_existing_subscribed, s_rrsm_acquire_existing_subscribed_fn)

/*
 * Verify: A multi-sub acquire where all subscriptions are currently subscribed
 * returns SUBSCRIBED.  Verify subscription events are emitted.
 */
static int s_rrsm_acquire_multi_existing_subscribed_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_byte_cursor multi_subs[] = {
        s_hello_world1_cursor,
        s_hello_world2_cursor,
    };

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));
    ASSERT_TRUE(
        s_api_records_equals(fixture.mock_protocol_adapter, AWS_ARRAY_SIZE(expected_subscribes), expected_subscribes));

    struct aws_protocol_adapter_subscription_event successful_subscription1_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription1_event);

    struct aws_protocol_adapter_subscription_event successful_subscription2_event = {
        .topic_filter = s_hello_world2_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription2_event);

    struct aws_subscription_status_record expected_subscription_events[] = {
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 1,
        },
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world2_cursor,
            .operation_id = 1,
        }};
    ASSERT_TRUE(s_contains_subscription_event_sequential_records(
        &fixture, AWS_ARRAY_SIZE(expected_subscription_events), expected_subscription_events));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBED, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 2, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_multi_existing_subscribed, s_rrsm_acquire_multi_existing_subscribed_fn)

/*
 * Verify: Acquiring a new request-response subscription when there is no room returns BLOCKED
 */
static int s_rrsm_acquire_blocked_rr_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 1,
        .operation_timeout_seconds = 30,
    };
    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire1_options));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire2_options));

    // no room, but it could potentially free up in the future
    struct aws_rr_acquire_subscription_options acquire3_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world3_cursor,
        .topic_filter_count = 1,
        .operation_id = 3,
    };
    ASSERT_INT_EQUALS(
        AASRT_BLOCKED,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire3_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_blocked_rr, s_rrsm_acquire_blocked_rr_fn)

/*
 * Verify: A two-subscription acquire returns BLOCKED when only one space is available
 */
static int s_rrsm_acquire_multi_blocked_rr_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 1,
        .operation_timeout_seconds = 30,
    };
    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire1_options));

    struct aws_byte_cursor multi_subs[] = {
        s_hello_world2_cursor,
        s_hello_world3_cursor,
    };
    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(
        AASRT_BLOCKED,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire2_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_multi_blocked_rr, s_rrsm_acquire_multi_blocked_rr_fn)

/*
 * Verify: Acquiring a new eventstream subscription when there is no room, but room could free up later, returns BLOCKED
 */
static int s_rrsm_acquire_blocked_eventstream_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 1,
        .operation_timeout_seconds = DEFAULT_SM_TEST_TIMEOUT,
    };
    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_protocol_adapter_connection_event connected_event = {
        .event_type = AWS_PACET_CONNECTED,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(&fixture.subscription_manager, &connected_event);

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire1_options));

    struct aws_protocol_adapter_subscription_event subscribe_success_event = {
        .event_type = AWS_PASET_SUBSCRIBE,
        .topic_filter = s_hello_world1_cursor,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(
        &fixture.subscription_manager, &subscribe_success_event);

    // release and trigger an unsubscribe
    struct aws_rr_release_subscription_options release1_options = {
        .operation_id = 1,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
    };
    aws_rr_subscription_manager_release_subscription(&fixture.subscription_manager, &release1_options);
    aws_rr_subscription_manager_purge_unused(&fixture.subscription_manager);

    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // acquire while the streaming unsubscribe is in-progress should return blocked
    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(
        AASRT_BLOCKED,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire2_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_blocked_eventstream, s_rrsm_acquire_blocked_eventstream_fn)

/*
 * Verify: A two-subscription streaming acquire when there is no room, but room could free up later, returns BLOCKED
 */
static int s_rrsm_acquire_multi_blocked_eventstream_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = DEFAULT_SM_TEST_TIMEOUT,
    };
    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_protocol_adapter_connection_event connected_event = {
        .event_type = AWS_PACET_CONNECTED,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(&fixture.subscription_manager, &connected_event);

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire1_options));

    struct aws_protocol_adapter_subscription_event subscribe_success_event = {
        .event_type = AWS_PASET_SUBSCRIBE,
        .topic_filter = s_hello_world1_cursor,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(
        &fixture.subscription_manager, &subscribe_success_event);

    // release and trigger an unsubscribe
    struct aws_rr_release_subscription_options release1_options = {
        .operation_id = 1,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
    };
    aws_rr_subscription_manager_release_subscription(&fixture.subscription_manager, &release1_options);
    aws_rr_subscription_manager_purge_unused(&fixture.subscription_manager);

    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // multi-acquire while the streaming unsubscribe is in-progress should return blocked
    struct aws_byte_cursor multi_subs[] = {
        s_hello_world2_cursor,
        s_hello_world3_cursor,
    };
    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(
        AASRT_BLOCKED,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire2_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_multi_blocked_eventstream, s_rrsm_acquire_multi_blocked_eventstream_fn)

static int s_do_acquire_no_capacity_test(struct aws_subscription_manager_test_fixture *fixture) {
    struct aws_rr_subscription_manager *manager = &fixture->subscription_manager;

    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_NO_CAPACITY, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    return AWS_OP_SUCCESS;
}

/*
 * Verify: Acquiring a new eventstream subscription when the budget is 0 returns NO_CAPACITY
 */
static int s_rrsm_acquire_no_capacity_max1_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 0,
        .operation_timeout_seconds = 30,
    };
    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    ASSERT_SUCCESS(s_do_acquire_no_capacity_test(&fixture));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_no_capacity_max1, s_rrsm_acquire_no_capacity_max1_fn)

/*
 * Verify: Acquiring a new eventstream subscription when there is no room, and no room could free up later, returns
 * NO_CAPACITY
 */
static int s_rrsm_acquire_no_capacity_too_many_event_stream_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 4,
        .operation_timeout_seconds = 30,
    };
    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    for (size_t i = 0; i < 4; ++i) {
        char topic_filter_buffer[256];
        snprintf(topic_filter_buffer, AWS_ARRAY_SIZE(topic_filter_buffer), "hello/world/%d", (int)(i + 1));
        struct aws_byte_cursor topic_filter_cursor = aws_byte_cursor_from_c_str(topic_filter_buffer);

        struct aws_rr_acquire_subscription_options acquire_options = {
            .type = ARRST_EVENT_STREAM,
            .topic_filters = &topic_filter_cursor,
            .topic_filter_count = 1,
            .operation_id = i + 2,
        };
        ASSERT_INT_EQUALS(
            AASRT_SUBSCRIBING,
            aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire_options));
    }

    // 1 more, should fail
    ASSERT_SUCCESS(s_do_acquire_no_capacity_test(&fixture));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_no_capacity_too_many_event_stream, s_rrsm_acquire_no_capacity_too_many_event_stream_fn)

/*
 * Verify: A two-subscription streaming acquire, where there is only room for one, and no room could free up later,
 * returns NO_CAPACITY
 */
static int s_rrsm_acquire_multi_no_capacity_event_stream_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = 30,
    };
    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };

    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire1_options));

    struct aws_byte_cursor multi_subs[] = {
        s_hello_world2_cursor,
        s_hello_world3_cursor,
    };
    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(
        AASRT_NO_CAPACITY,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire2_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_multi_no_capacity_event_stream, s_rrsm_acquire_multi_no_capacity_event_stream_fn)

/*
 * Verify: Acquiring an existing subscription with an unequal subscription type returns FAILURE
 */
static int s_rrsm_acquire_failure_mixed_subscription_types_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire1_options));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(
        AASRT_FAILURE,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire2_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_failure_mixed_subscription_types, s_rrsm_acquire_failure_mixed_subscription_types_fn)

/*
 * Verify: A two-subscription acquire that partially overlaps a subscription of different type results in FAILURE
 */
static int s_rrsm_acquire_multi_failure_mixed_subscription_types_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(
        AASRT_SUBSCRIBING,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire1_options));

    struct aws_byte_cursor multi_subs[] = {
        s_hello_world2_cursor,
        s_hello_world1_cursor,
    };
    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(
        AASRT_FAILURE,
        aws_rr_subscription_manager_acquire_subscription(&fixture.subscription_manager, &acquire2_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrsm_acquire_multi_failure_mixed_subscription_types,
    s_rrsm_acquire_multi_failure_mixed_subscription_types_fn)

/*
 * Verify: Acquiring a poisoned streaming subscription results in failure.
 */
static int s_rrsm_acquire_failure_poisoned_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    struct aws_protocol_adapter_subscription_event unretryable_failure_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_MQTT_PROTOCOL_ADAPTER_FAILING_REASON_CODE,
        .retryable = false,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &unretryable_failure_event);

    struct aws_subscription_status_record expected_subscription_events[] = {{
        .type = ARRSET_STREAMING_SUBSCRIPTION_HALTED,
        .topic_filter_cursor = s_hello_world1_cursor,
        .operation_id = 1,
    }};
    ASSERT_TRUE(s_contains_subscription_event_sequential_records(&fixture, 1, expected_subscription_events));

    struct aws_rr_acquire_subscription_options reacquire1_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 3,
    };
    ASSERT_INT_EQUALS(AASRT_FAILURE, aws_rr_subscription_manager_acquire_subscription(manager, &reacquire1_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_failure_poisoned, s_rrsm_acquire_failure_poisoned_fn)

/*
 * Verify: A request subscription that resolves successfully invokes callbacks for every operation listener; releasing
 * both references and purging will trigger an unsubscribe of the first subscription
 */
static int s_rrsm_release_unsubscribes_request_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));

    struct aws_protocol_adapter_subscription_event successful_subscription_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription_event);

    // verify two success callbacks
    struct aws_subscription_status_record expected_subscription_events[] = {
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 1,
        },
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 2,
        }};
    ASSERT_TRUE(s_contains_subscription_event_records(&fixture, 2, expected_subscription_events));

    // verify no unsubscribes
    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // release once, verify no unsubscribe
    struct aws_rr_release_subscription_options release1_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release1_options);
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // release second
    struct aws_rr_release_subscription_options release2_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release2_options);
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    aws_rr_subscription_manager_purge_unused(manager);

    // now the unsubscribe should be present
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_release_unsubscribes_request, s_rrsm_release_unsubscribes_request_fn)

/*
 * Verify: A multi subscription that resolves successfully invokes callbacks for every operation listener; releasing
 * all references and purging will trigger an unsubscribe of both subscriptions
 */
static int s_rrsm_release_multi_unsubscribes_request_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_byte_cursor multi_subs[] = {
        s_hello_world1_cursor,
        s_hello_world2_cursor,
    };
    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));

    struct aws_protocol_adapter_subscription_event successful_subscription_event1 = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription_event1);

    struct aws_protocol_adapter_subscription_event successful_subscription_event2 = {
        .topic_filter = s_hello_world2_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription_event2);

    // verify four success callbacks
    struct aws_subscription_status_record expected_subscription_events[] = {
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 1,
        },
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 2,
        },
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world2_cursor,
            .operation_id = 1,
        },
        {
            .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
            .topic_filter_cursor = s_hello_world2_cursor,
            .operation_id = 2,
        },
    };
    ASSERT_TRUE(s_contains_subscription_event_records(
        &fixture, AWS_ARRAY_SIZE(expected_subscription_events), expected_subscription_events));

    // verify no unsubscribes
    struct aws_protocol_adapter_api_record expected_unsubscribe1 = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe1));

    struct aws_protocol_adapter_api_record expected_unsubscribe2 = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world2_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe2));

    // release once, verify no unsubscribes
    struct aws_rr_release_subscription_options release1_options = {
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 1,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release1_options);
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe1));
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe2));

    // release second
    struct aws_rr_release_subscription_options release2_options = {
        .topic_filters = multi_subs,
        .topic_filter_count = AWS_ARRAY_SIZE(multi_subs),
        .operation_id = 2,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release2_options);
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe1));
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe2));

    aws_rr_subscription_manager_purge_unused(manager);

    // now the unsubscribes should be present
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe1));
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe2));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_release_multi_unsubscribes_request, s_rrsm_release_multi_unsubscribes_request_fn)

/*
 * Verify: A streaming subscription that resolves successfully invokes callbacks for every operation listener; releasing
 * both references and calling a new acquire will trigger an unsubscribe of the first subscription
 */
static int s_rrsm_release_unsubscribes_streaming_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));

    struct aws_protocol_adapter_subscription_event successful_subscription_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription_event);

    // verify two success callbacks
    struct aws_subscription_status_record expected_subscription_events[] = {
        {
            .type = ARRSET_STREAMING_SUBSCRIPTION_ESTABLISHED,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 1,
        },
        {
            .type = ARRSET_STREAMING_SUBSCRIPTION_ESTABLISHED,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 2,
        }};
    ASSERT_TRUE(s_contains_subscription_event_records(&fixture, 2, expected_subscription_events));

    // verify no unsubscribes
    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // release once, verify no unsubscribe
    struct aws_rr_release_subscription_options release1_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release1_options);
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // release second
    struct aws_rr_release_subscription_options release2_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release2_options);
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    aws_rr_subscription_manager_purge_unused(manager);

    // now the unsubscribe should be present
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_release_unsubscribes_streaming, s_rrsm_release_unsubscribes_streaming_fn)

static int s_rrsm_do_unsubscribe_test(struct aws_allocator *allocator, bool should_succeed) {
    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .operation_timeout_seconds = DEFAULT_SM_TEST_TIMEOUT,
        .start_connected = true,
    };

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));

    // budget of 2, so new acquires should be blocked
    struct aws_rr_acquire_subscription_options acquire3_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world3_cursor,
        .topic_filter_count = 1,
        .operation_id = 3,
    };
    ASSERT_INT_EQUALS(AASRT_BLOCKED, aws_rr_subscription_manager_acquire_subscription(manager, &acquire3_options));

    // complete the subscribe
    struct aws_protocol_adapter_subscription_event successful_subscription_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription_event);

    // still blocked
    ASSERT_INT_EQUALS(AASRT_BLOCKED, aws_rr_subscription_manager_acquire_subscription(manager, &acquire3_options));

    // release
    struct aws_rr_release_subscription_options release1_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release1_options);

    // unsubscribe should be visible, but we're still blocked because it hasn't completed
    aws_rr_subscription_manager_purge_unused(manager);
    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // complete the unsubscribe
    struct aws_protocol_adapter_subscription_event successful_unsubscribe_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_UNSUBSCRIBE,
        .error_code = should_succeed ? AWS_ERROR_SUCCESS : AWS_ERROR_MQTT5_UNSUBSCRIBE_OPTIONS_VALIDATION,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_unsubscribe_event);

    aws_rr_subscription_manager_purge_unused(manager);

    // a successful unsubscribe should clear space, a failed one should not
    ASSERT_INT_EQUALS(
        (should_succeed ? AASRT_SUBSCRIBING : AASRT_BLOCKED),
        aws_rr_subscription_manager_acquire_subscription(manager, &acquire3_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

/*
 * Verify: Releasing a subscription and successfully unsubscribing frees up space for new acquires
 */
static int s_rrsm_release_unsubscribe_success_clears_space_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_rrsm_do_unsubscribe_test(allocator, true);
}

AWS_TEST_CASE(rrsm_release_unsubscribe_success_clears_space, s_rrsm_release_unsubscribe_success_clears_space_fn)

/*
 * Verify: Releasing a subscription but failing to unsubscribe does not free up space for new acquires
 */
static int s_rrsm_release_unsubscribe_failure_blocked_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_rrsm_do_unsubscribe_test(allocator, false);
}

AWS_TEST_CASE(rrsm_release_unsubscribe_failure_blocked, s_rrsm_release_unsubscribe_failure_blocked_fn)

static int s_aws_mqtt_protocol_adapter_mock_subscribe_fails_first_time(
    void *impl,
    struct aws_protocol_adapter_subscribe_options *options) {
    (void)impl;
    (void)options;

    struct aws_mqtt_protocol_adapter_mock_impl *mock_impl = impl;

    if (mock_impl->subscribe_count++ == 0) {
        return aws_raise_error(AWS_ERROR_INVALID_ARGUMENT);
    } else {
        return AWS_OP_SUCCESS;
    }
}

/*
 * Verify: Acquiring a new request subscription but synchronously failing the protocol adapter subscribe returns FAILURE
 */
static int s_rrsm_acquire_failure_subscribe_sync_failure_request_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_protocol_adapter_vtable failing_vtable = s_protocol_adapter_mock_vtable;
    failing_vtable.aws_mqtt_protocol_adapter_subscribe_fn = s_aws_mqtt_protocol_adapter_mock_subscribe_fails_first_time;

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = DEFAULT_SM_TEST_TIMEOUT,
        .start_connected = true,
        .adapter_vtable = &failing_vtable,
    };

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_FAILURE, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrsm_acquire_failure_subscribe_sync_failure_request,
    s_rrsm_acquire_failure_subscribe_sync_failure_request_fn)

/*
 * Verify: Acquiring a new streaming subscription but synchronously failing the protocol adapter subscribe returns
 * FAILURE Trying again also fails even though the mock subscribe succeeds after the first try.
 */
static int s_rrsm_acquire_failure_subscribe_sync_failure_streaming_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_mqtt_protocol_adapter_vtable failing_vtable = s_protocol_adapter_mock_vtable;
    failing_vtable.aws_mqtt_protocol_adapter_subscribe_fn = s_aws_mqtt_protocol_adapter_mock_subscribe_fails_first_time;

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = DEFAULT_SM_TEST_TIMEOUT,
        .start_connected = true,
        .adapter_vtable = &failing_vtable,
    };

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_FAILURE, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    struct aws_rr_acquire_subscription_options acquire_options2 = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_FAILURE, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options2));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrsm_acquire_failure_subscribe_sync_failure_streaming,
    s_rrsm_acquire_failure_subscribe_sync_failure_streaming_fn)

/*
 * Verify: Completing a request subscription-acquire with a failing reason code emits a subscription failed event
 */
static int s_rrsm_acquire_request_subscribe_failure_event_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    // complete the subscribe with a failure
    struct aws_protocol_adapter_subscription_event failed_subscription_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_MQTT_PROTOCOL_ADAPTER_FAILING_REASON_CODE,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &failed_subscription_event);

    // verify subscribe failure event emission
    struct aws_subscription_status_record expected_subscription_event = {
        .type = ARRSET_REQUEST_SUBSCRIBE_FAILURE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .operation_id = 1,
    };

    ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_event));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(rrsm_acquire_request_subscribe_failure_event, s_rrsm_acquire_request_subscribe_failure_event_fn)

/*
 * Verify: Completing a streaming subscription-acquire with a retryable failing failure triggers a resubscribe attempt
 */
static int s_rrsm_acquire_streaming_subscribe_failure_retryable_resubscribe_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    // complete the subscribe with a retryable failure
    struct aws_protocol_adapter_subscription_event failed_subscription_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_MQTT_PROTOCOL_ADAPTER_FAILING_REASON_CODE,
        .retryable = true,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &failed_subscription_event);

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    ASSERT_TRUE(
        s_api_records_equals(fixture.mock_protocol_adapter, AWS_ARRAY_SIZE(expected_subscribes), expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrsm_acquire_streaming_subscribe_failure_retryable_resubscribe,
    s_rrsm_acquire_streaming_subscribe_failure_retryable_resubscribe_fn)

static enum aws_rr_subscription_event_type s_compute_expected_subscription_event_offline_acquire_online(
    enum aws_rr_subscription_type subscription_type,
    bool success) {
    if (subscription_type == ARRST_REQUEST_RESPONSE) {
        if (success) {
            return ARRSET_REQUEST_SUBSCRIBE_SUCCESS;
        } else {
            return ARRSET_REQUEST_SUBSCRIBE_FAILURE;
        }
    } else {
        if (success) {
            return ARRSET_STREAMING_SUBSCRIPTION_ESTABLISHED;
        } else {
            return ARRSET_STREAMING_SUBSCRIPTION_HALTED;
        }
    }
}

static int s_do_offline_acquire_online_test(
    struct aws_allocator *allocator,
    enum aws_rr_subscription_type subscription_type,
    bool success) {
    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture_options fixture_config = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = DEFAULT_SM_TEST_TIMEOUT,
        .start_connected = false,
    };

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, &fixture_config));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = subscription_type,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    // nothing should happen while offline
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 0, NULL));

    // go online, should trigger a subscribe
    struct aws_protocol_adapter_connection_event online_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = false,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(&fixture.subscription_manager, &online_event);

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 1, expected_subscribes));

    // complete the subscribe, verify a subscription success/failure event
    struct aws_protocol_adapter_subscription_event subscription_event = {
        .event_type = AWS_PASET_SUBSCRIBE,
        .topic_filter = s_hello_world1_cursor,
        .error_code = success ? AWS_ERROR_SUCCESS : AWS_ERROR_MQTT_PROTOCOL_ADAPTER_FAILING_REASON_CODE,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &subscription_event);

    struct aws_subscription_status_record expected_subscription_event = {
        .type = s_compute_expected_subscription_event_offline_acquire_online(subscription_type, success),
        .topic_filter_cursor = s_hello_world1_cursor,
        .operation_id = 1,
    };
    ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_event));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

/*
 * Verify: Acquiring a new request subscription while offline returns SUBSCRIBING.  Going online triggers a protocol
 * adapter subscribe.  Completing the subscription successfully emits a request subscribe success event.
 */
static int s_rrsm_offline_acquire_request_online_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_offline_acquire_online_test(allocator, ARRST_REQUEST_RESPONSE, true);
}

AWS_TEST_CASE(rrsm_offline_acquire_request_online_success, s_rrsm_offline_acquire_request_online_success_fn)

/*
 * Verify: Acquiring a new request subscription while offline returns SUBSCRIBING.  Going online triggers a protocol
 * adapter subscribe.  Completing the subscription with a failure emits a request subscribe failure event.
 */
static int s_rrsm_offline_acquire_request_online_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_offline_acquire_online_test(allocator, ARRST_REQUEST_RESPONSE, false);
}

AWS_TEST_CASE(rrsm_offline_acquire_request_online_failure, s_rrsm_offline_acquire_request_online_failure_fn)

/*
 * Verify: Acquiring a new steaming subscription while offline returns SUBSCRIBING.  Going online triggers a protocol
 * adapter subscribe.  Completing the subscription successfully emits a streaming subscription established event.
 */
static int s_rrsm_offline_acquire_streaming_online_success_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_offline_acquire_online_test(allocator, ARRST_EVENT_STREAM, true);
}

AWS_TEST_CASE(rrsm_offline_acquire_streaming_online_success, s_rrsm_offline_acquire_streaming_online_success_fn)

/*
 * Verify: Acquiring a new request subscription while offline returns SUBSCRIBING.  Going online triggers a protocol
 * adapter subscribe.  Completing the subscription with a failure emits a streaming subscription halted event.
 */
static int s_rrsm_offline_acquire_streaming_online_failure_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_offline_acquire_online_test(allocator, ARRST_EVENT_STREAM, false);
}

AWS_TEST_CASE(rrsm_offline_acquire_streaming_online_failure, s_rrsm_offline_acquire_streaming_online_failure_fn)

static int s_do_offline_acquire_release_online_test(
    struct aws_allocator *allocator,
    enum aws_rr_subscription_type subscription_type) {
    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    // trigger online -> offline
    struct aws_protocol_adapter_connection_event offline_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &offline_event);

    // acquire
    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = subscription_type,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    // nothing should happen while offline
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 0, NULL));

    // release while offline, nothing should happen
    struct aws_rr_release_subscription_options release_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release_options);
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 0, NULL));

    // go online, nothing should be sent to the protocol adapter
    struct aws_protocol_adapter_connection_event online_event = {
        .event_type = AWS_PACET_CONNECTED,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &online_event);
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 0, NULL));

    // trigger a different subscription, verify it's the only thing that has reached the protocol adapter
    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = subscription_type,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));

    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world2_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };
    ASSERT_TRUE(s_api_records_equals(fixture.mock_protocol_adapter, 1, expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

/*
 * Verify: Acquiring and releasing a subscription while offline and then going online should remove the
 * subscription without invoking any protocol adapter APIs.
 */
static int s_rrsm_offline_acquire_release_request_online_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_offline_acquire_release_online_test(allocator, ARRST_REQUEST_RESPONSE);
}

AWS_TEST_CASE(rrsm_offline_acquire_release_request_online, s_rrsm_offline_acquire_release_request_online_fn)

/*
 * Verify: Acquiring and releasing a subscription while offline and then going online should remove the
 * subscription without invoking any protocol adapter APIs.
 */
static int s_rrsm_offline_acquire_release_streaming_online_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_offline_acquire_release_online_test(allocator, ARRST_EVENT_STREAM);
}

AWS_TEST_CASE(rrsm_offline_acquire_release_streaming_online, s_rrsm_offline_acquire_release_streaming_online_fn)

static int s_do_acquire_success_offline_release_acquire2_no_unsubscribe_test(
    struct aws_allocator *allocator,
    enum aws_rr_subscription_type subscription_type) {

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    // acquire
    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = subscription_type,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    // successfully complete subscription
    struct aws_protocol_adapter_subscription_event subscription_event = {
        .event_type = AWS_PASET_SUBSCRIBE,
        .topic_filter = s_hello_world1_cursor,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &subscription_event);

    struct aws_subscription_status_record expected_subscription_event = {
        .type = subscription_type == ARRST_REQUEST_RESPONSE ? ARRSET_REQUEST_SUBSCRIBE_SUCCESS
                                                            : ARRSET_STREAMING_SUBSCRIPTION_ESTABLISHED,
        .topic_filter_cursor = s_hello_world1_cursor,
        .operation_id = 1,
    };
    ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_event));

    // trigger online -> offline
    struct aws_protocol_adapter_connection_event offline_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &offline_event);

    // release
    struct aws_rr_release_subscription_options release_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release_options);

    // acquire something different, normally that triggers an unsubscribe, but we're offline
    struct aws_rr_acquire_subscription_options acquire2_options = {
        .type = subscription_type,
        .topic_filters = &s_hello_world2_cursor,
        .topic_filter_count = 1,
        .operation_id = 2,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire2_options));

    // verify no unsubscribe has been sent
    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // online, verify unsubscribe
    struct aws_protocol_adapter_connection_event online_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = true,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &online_event);

    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

/*
 * Verify: Releasing an active request subscription while offline should not invoke an unsubscribe until back online
 */
static int s_rrsm_acquire_request_success_offline_release_acquire2_no_unsubscribe_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_acquire_success_offline_release_acquire2_no_unsubscribe_test(allocator, ARRST_REQUEST_RESPONSE);
}

AWS_TEST_CASE(
    rrsm_acquire_request_success_offline_release_acquire2_no_unsubscribe,
    s_rrsm_acquire_request_success_offline_release_acquire2_no_unsubscribe_fn)

/*
 * Verify: Releasing an active streaming subscription while offline should not invoke an unsubscribe until back online
 */
static int s_rrsm_acquire_streaming_success_offline_release_acquire2_no_unsubscribe_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_acquire_success_offline_release_acquire2_no_unsubscribe_test(allocator, ARRST_REQUEST_RESPONSE);
}

AWS_TEST_CASE(
    rrsm_acquire_streaming_success_offline_release_acquire2_no_unsubscribe,
    s_rrsm_acquire_streaming_success_offline_release_acquire2_no_unsubscribe_fn)

static int s_do_rrsm_acquire_clean_up_test(
    struct aws_allocator *allocator,
    enum aws_rr_subscription_type subscription_type,
    bool complete_subscribe,
    bool clean_up_while_connected) {

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    // acquire
    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = subscription_type,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    // successfully complete subscription if desired
    if (complete_subscribe) {
        struct aws_protocol_adapter_subscription_event subscription_event = {
            .event_type = AWS_PASET_SUBSCRIBE,
            .topic_filter = s_hello_world1_cursor,
            .error_code = AWS_ERROR_SUCCESS,
        };
        aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &subscription_event);

        struct aws_subscription_status_record expected_subscription_event = {
            .type = subscription_type == ARRST_REQUEST_RESPONSE ? ARRSET_REQUEST_SUBSCRIBE_SUCCESS
                                                                : ARRSET_STREAMING_SUBSCRIPTION_ESTABLISHED,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 1,
        };
        ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_event));
    }

    if (!clean_up_while_connected) {
        // trigger online -> offline
        struct aws_protocol_adapter_connection_event offline_event = {
            .event_type = AWS_PACET_DISCONNECTED,
        };
        aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &offline_event);
    }

    // clean up subscription manager
    aws_rr_subscription_manager_clean_up(manager);

    // verify an unsubscribe was sent even though we are offline
    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // prevent the fixture cleanup from double freeing the subscription manager that was already cleaned up by
    // reinitializing the subscription manager
    struct aws_rr_subscription_manager_options subscription_manager_options = {
        .max_request_response_subscriptions = 2,
        .max_streaming_subscriptions = 2,
        .operation_timeout_seconds = 5,
        .subscription_status_callback = s_aws_rr_subscription_status_event_test_callback_fn,
        .userdata = &fixture,
    };
    aws_rr_subscription_manager_init(
        &fixture.subscription_manager, allocator, fixture.mock_protocol_adapter, &subscription_manager_options);

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

/*
 * Verify: Calling clean up while a request subscription is active triggers an immediate unsubscribe
 */
static int s_rrsm_acquire_request_success_clean_up_unsubscribe_override_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_rrsm_acquire_clean_up_test(allocator, ARRST_REQUEST_RESPONSE, true, true);
}

AWS_TEST_CASE(
    rrsm_acquire_request_success_clean_up_unsubscribe_override,
    s_rrsm_acquire_request_success_clean_up_unsubscribe_override_fn)

/*
 * Verify: Calling clean up while a streaming subscription is active triggers an immediate unsubscribe
 */
static int s_rrsm_acquire_streaming_success_clean_up_unsubscribe_override_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_rrsm_acquire_clean_up_test(allocator, ARRST_EVENT_STREAM, true, true);
}

AWS_TEST_CASE(
    rrsm_acquire_streaming_success_clean_up_unsubscribe_override,
    s_rrsm_acquire_streaming_success_clean_up_unsubscribe_override_fn)

/*
 * Verify: Calling clean up while a request subscription is pending triggers an immediate unsubscribe
 */
static int s_rrsm_acquire_request_pending_clean_up_unsubscribe_override_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_rrsm_acquire_clean_up_test(allocator, ARRST_REQUEST_RESPONSE, false, true);
}

AWS_TEST_CASE(
    rrsm_acquire_request_pending_clean_up_unsubscribe_override,
    s_rrsm_acquire_request_pending_clean_up_unsubscribe_override_fn)

/*
 * Verify: Calling clean up while a streaming subscription is pending triggers an immediate unsubscribe
 */
static int s_rrsm_acquire_streaming_pending_clean_up_unsubscribe_override_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_rrsm_acquire_clean_up_test(allocator, ARRST_EVENT_STREAM, false, true);
}

AWS_TEST_CASE(
    rrsm_acquire_streaming_pending_clean_up_unsubscribe_override,
    s_rrsm_acquire_streaming_pending_clean_up_unsubscribe_override_fn)

/*
 * Verify: Calling clean up while offline and a request subscription is pending triggers an immediate unsubscribe
 */
static int s_rrsm_offline_acquire_request_pending_clean_up_unsubscribe_override_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_rrsm_acquire_clean_up_test(allocator, ARRST_REQUEST_RESPONSE, false, false);
}

AWS_TEST_CASE(
    rrsm_offline_acquire_request_pending_clean_up_unsubscribe_override,
    s_rrsm_offline_acquire_request_pending_clean_up_unsubscribe_override_fn)

/*
 * Verify: Calling clean up while offline and a streaming subscription is pending triggers an immediate unsubscribe
 */
static int s_rrsm_offline_acquire_streaming_pending_clean_up_unsubscribe_override_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_do_rrsm_acquire_clean_up_test(allocator, ARRST_EVENT_STREAM, false, false);
}

AWS_TEST_CASE(
    rrsm_offline_acquire_streaming_pending_clean_up_unsubscribe_override,
    s_rrsm_offline_acquire_streaming_pending_clean_up_unsubscribe_override_fn)

static int s_rrsm_do_no_session_subscription_ended_test(
    struct aws_allocator *allocator,
    bool offline_while_unsubscribing) {
    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    // acquire
    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    // successfully complete subscription
    struct aws_protocol_adapter_subscription_event subscription_event = {
        .event_type = AWS_PASET_SUBSCRIBE,
        .topic_filter = s_hello_world1_cursor,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &subscription_event);

    struct aws_subscription_status_record expected_subscription_event = {
        .type = ARRSET_REQUEST_SUBSCRIBE_SUCCESS,
        .topic_filter_cursor = s_hello_world1_cursor,
        .operation_id = 1,
    };
    ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_event));

    // release if appropriate
    if (offline_while_unsubscribing) {
        struct aws_rr_release_subscription_options release_options = {
            .topic_filters = &s_hello_world1_cursor,
            .topic_filter_count = 1,
            .operation_id = 1,
        };
        aws_rr_subscription_manager_release_subscription(manager, &release_options);

        struct aws_protocol_adapter_api_record expected_unsubscribe = {
            .type = PAAT_UNSUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        };
        ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

        // trigger an event that would cull unused subscriptions, causing an unsubscribe
        aws_rr_subscription_manager_purge_unused(manager);

        // now the unsubscribe should be present
        ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));
    }

    // online -> offline
    struct aws_protocol_adapter_connection_event offline_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &offline_event);

    // offline -> online (no session)
    struct aws_protocol_adapter_connection_event online_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = false,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &online_event);

    // verify subscription lost emitted
    if (!offline_while_unsubscribing) {
        struct aws_subscription_status_record expected_subscription_ended_event = {
            .type = ARRSET_REQUEST_SUBSCRIPTION_ENDED,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 1,
        };
        ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_ended_event));
    }

    // if we were unsubscribing, verify reacquire is blocked and then complete the unsubscribe
    struct aws_rr_acquire_subscription_options reacquire_options = {
        .type = ARRST_REQUEST_RESPONSE,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 3,
    };

    if (offline_while_unsubscribing) {
        ASSERT_INT_EQUALS(AASRT_BLOCKED, aws_rr_subscription_manager_acquire_subscription(manager, &reacquire_options));

        struct aws_protocol_adapter_subscription_event unsubscribe_event = {
            .event_type = AWS_PASET_UNSUBSCRIBE,
            .topic_filter = s_hello_world1_cursor,
            .error_code = AWS_ERROR_SUCCESS,
        };
        aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &unsubscribe_event);
    }

    // verify we can reacquire
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &reacquire_options));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

/*
 * Verify: If the client fails to rejoin a session, a SUBSCRIPTION_ENDED event is emitted for active request
 * subscriptions and that subscription can successfully be reacquired
 */
static int s_rrsm_acquire_request_success_offline_online_no_session_subscription_ended_can_reacquire_fn(
    struct aws_allocator *allocator,
    void *ctx) {
    (void)ctx;

    return s_rrsm_do_no_session_subscription_ended_test(allocator, false);
}

AWS_TEST_CASE(
    rrsm_acquire_request_success_offline_online_no_session_subscription_ended_can_reacquire,
    s_rrsm_acquire_request_success_offline_online_no_session_subscription_ended_can_reacquire_fn)

/*
 * Verify: If the client fails to rejoin a session, a SUBSCRIPTION_ENDED event is emitted for unsubscribing
 * request subscriptions
 */
static int s_rrsm_request_subscription_ended_while_unsubscribing_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_rrsm_do_no_session_subscription_ended_test(allocator, true);
}

AWS_TEST_CASE(
    rrsm_request_subscription_ended_while_unsubscribing,
    s_rrsm_request_subscription_ended_while_unsubscribing_fn)

/*
 * Verify: If the client fails to rejoin a session, a SUBSCRIPTION_LOST event is emitted for streaming subscriptions,
 * and a resubscribe is triggered
 */
static int s_rrsm_streaming_subscription_lost_resubscribe_on_no_session_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    // acquire
    struct aws_rr_acquire_subscription_options acquire_options = {
        .type = ARRST_EVENT_STREAM,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire_options));

    // successfully complete subscription
    struct aws_protocol_adapter_subscription_event subscription_event = {
        .event_type = AWS_PASET_SUBSCRIBE,
        .topic_filter = s_hello_world1_cursor,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &subscription_event);

    struct aws_subscription_status_record expected_subscription_event = {
        .type = ARRSET_STREAMING_SUBSCRIPTION_ESTABLISHED,
        .topic_filter_cursor = s_hello_world1_cursor,
        .operation_id = 1,
    };
    ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_event));

    // online -> offline
    struct aws_protocol_adapter_connection_event offline_event = {
        .event_type = AWS_PACET_DISCONNECTED,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &offline_event);

    // offline -> online (no session)
    struct aws_protocol_adapter_connection_event online_event = {
        .event_type = AWS_PACET_CONNECTED,
        .joined_session = false,
    };
    aws_rr_subscription_manager_on_protocol_adapter_connection_event(manager, &online_event);

    // verify subscription lost on rejoin
    struct aws_subscription_status_record expected_subscription_ended_event = {
        .type = ARRSET_STREAMING_SUBSCRIPTION_LOST,
        .topic_filter_cursor = s_hello_world1_cursor,
        .operation_id = 1,
    };
    ASSERT_TRUE(s_contains_subscription_event_record(&fixture, &expected_subscription_ended_event));

    // verify resubscribe submitted to the protocol adapter
    struct aws_protocol_adapter_api_record expected_subscribes[] = {
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
        {
            .type = PAAT_SUBSCRIBE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .timeout = DEFAULT_SM_TEST_TIMEOUT,
        },
    };

    ASSERT_TRUE(
        s_api_records_equals(fixture.mock_protocol_adapter, AWS_ARRAY_SIZE(expected_subscribes), expected_subscribes));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    rrsm_streaming_subscription_lost_resubscribe_on_no_session,
    s_rrsm_streaming_subscription_lost_resubscribe_on_no_session_fn)

static int s_do_purge_test(struct aws_allocator *allocator, enum aws_rr_subscription_type subscription_type) {
    aws_mqtt_library_init(allocator);

    struct aws_subscription_manager_test_fixture fixture;
    ASSERT_SUCCESS(s_aws_subscription_manager_test_fixture_init(&fixture, allocator, NULL));

    struct aws_rr_subscription_manager *manager = &fixture.subscription_manager;

    struct aws_rr_acquire_subscription_options acquire1_options = {
        .type = subscription_type,
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    ASSERT_INT_EQUALS(AASRT_SUBSCRIBING, aws_rr_subscription_manager_acquire_subscription(manager, &acquire1_options));

    struct aws_protocol_adapter_subscription_event successful_subscription_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_SUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_subscription_event);

    struct aws_subscription_status_record expected_empty_subscription_events[] = {
        {
            .type = ARRSET_SUBSCRIPTION_EMPTY,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 0,
        },
    };

    struct aws_subscription_status_record expected_unsubscribe_events[] = {
        {
            .type = ARRSET_UNSUBSCRIBE_COMPLETE,
            .topic_filter_cursor = s_hello_world1_cursor,
            .operation_id = 0,
        },
    };

    ASSERT_FALSE(s_contains_subscription_event_records(&fixture, 1, expected_empty_subscription_events));
    ASSERT_FALSE(s_contains_subscription_event_records(&fixture, 1, expected_unsubscribe_events));

    // verify no unsubscribes
    struct aws_protocol_adapter_api_record expected_unsubscribe = {
        .type = PAAT_UNSUBSCRIBE,
        .topic_filter_cursor = s_hello_world1_cursor,
        .timeout = DEFAULT_SM_TEST_TIMEOUT,
    };
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // release once, verify no unsubscribe
    struct aws_rr_release_subscription_options release1_options = {
        .topic_filters = &s_hello_world1_cursor,
        .topic_filter_count = 1,
        .operation_id = 1,
    };
    aws_rr_subscription_manager_release_subscription(manager, &release1_options);
    ASSERT_FALSE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // verify empty event, but no unsubscribe event yet
    ASSERT_TRUE(s_contains_subscription_event_records(&fixture, 1, expected_empty_subscription_events));
    ASSERT_FALSE(s_contains_subscription_event_records(&fixture, 1, expected_unsubscribe_events));

    // purge
    aws_rr_subscription_manager_purge_unused(manager);

    // now the unsubscribe should be present
    ASSERT_TRUE(s_api_records_contains_record(fixture.mock_protocol_adapter, &expected_unsubscribe));

    // complete the unsubscribe
    struct aws_protocol_adapter_subscription_event successful_unsubscribe_event = {
        .topic_filter = s_hello_world1_cursor,
        .event_type = AWS_PASET_UNSUBSCRIBE,
        .error_code = AWS_ERROR_SUCCESS,
    };
    aws_rr_subscription_manager_on_protocol_adapter_subscription_event(manager, &successful_unsubscribe_event);

    // verify unsubscribe attempt emission
    ASSERT_TRUE(s_contains_subscription_event_records(&fixture, 1, expected_unsubscribe_events));

    s_aws_subscription_manager_test_fixture_clean_up(&fixture);
    aws_mqtt_library_clean_up();

    return AWS_OP_SUCCESS;
}

static int s_rrsm_request_subscription_purge_events_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_purge_test(allocator, ARRST_REQUEST_RESPONSE);
}

AWS_TEST_CASE(rrsm_request_subscription_purge_events, s_rrsm_request_subscription_purge_events_fn)

static int s_rrsm_streaming_subscription_purge_events_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    return s_do_purge_test(allocator, ARRST_EVENT_STREAM);
}

AWS_TEST_CASE(rrsm_streaming_subscription_purge_events, s_rrsm_streaming_subscription_purge_events_fn)
