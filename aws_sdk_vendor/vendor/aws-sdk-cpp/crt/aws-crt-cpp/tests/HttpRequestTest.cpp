/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>

#include <aws/crt/http/HttpRequestResponse.h>

#include <aws/http/request_response.h>

#include <aws/testing/aws_test_harness.h>

#include <sstream>

using namespace Aws::Crt::Http;

static int s_HttpRequestTestCreateDestroy(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Http::HttpRequest request(allocator);
        request.SetMethod(aws_byte_cursor_from_c_str("GET"));
        request.SetPath(aws_byte_cursor_from_c_str("/index"));

        std::shared_ptr<Aws::Crt::Io::IStream> stream =
            Aws::Crt::MakeShared<std::stringstream>(allocator, "TestContent");
        request.SetBody(stream);

        std::shared_ptr<Aws::Crt::Io::IStream> stream2 =
            Aws::Crt::MakeShared<std::stringstream>(allocator, "SomeOtherContent");
        request.SetBody(stream2);

        HttpHeader header1 = {aws_byte_cursor_from_c_str("Host"), aws_byte_cursor_from_c_str("www.test.com")};
        request.AddHeader(header1);

        HttpHeader header2 = {aws_byte_cursor_from_c_str("Authorization"), aws_byte_cursor_from_c_str("sadf")};
        request.AddHeader(header2);

        HttpHeader header3 = {aws_byte_cursor_from_c_str("UserAgent"), aws_byte_cursor_from_c_str("unit-tests-1.0")};
        request.AddHeader(header3);

        request.EraseHeader(2);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(HttpRequestTestCreateDestroy, s_HttpRequestTestCreateDestroy)
