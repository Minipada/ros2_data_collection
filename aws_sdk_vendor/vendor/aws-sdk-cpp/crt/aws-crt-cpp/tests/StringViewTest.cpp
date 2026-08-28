/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/Types.h>
#include <aws/testing/aws_test_harness.h>

static int s_test_string_view(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        const char *data = "abc123xyz";
        Aws::Crt::StringView sv(data);

        // test accessors
        {
            ASSERT_INT_EQUALS(9u, sv.size());
            ASSERT_FALSE(sv.empty());
            ASSERT_PTR_EQUALS(data, sv.data());
            ASSERT_TRUE(*(sv.begin()) == 'a');
            ASSERT_TRUE(*(sv.cbegin()) == 'a');
            ASSERT_TRUE(*(sv.rbegin()) == 'z');
            ASSERT_TRUE(*(sv.crbegin()) == 'z');
            ASSERT_TRUE(sv[0] == 'a');
            ASSERT_TRUE(sv[3] == '1');
            ASSERT_TRUE(sv[6] == 'x');
            ASSERT_TRUE(sv[8] == 'z');
            ASSERT_TRUE(sv.front() == 'a');
            ASSERT_TRUE(sv.back() == 'z');
            ASSERT_TRUE(sv.at(4) == '2');
        }

        auto subsv = sv.substr(3, 4);
        // test substr
        {
            ASSERT_INT_EQUALS(4u, subsv.size());
            ASSERT_PTR_EQUALS(data + 3, subsv.data());
            ASSERT_TRUE(subsv.front() == '1');
            ASSERT_TRUE(subsv.back() == 'x');
        }

        // test modifiers
        {
            sv.remove_prefix(3);
            ASSERT_PTR_EQUALS(data + 3, sv.data());
            ASSERT_INT_EQUALS(6u, sv.size());
            ASSERT_TRUE(sv.front() == '1');

            sv.remove_suffix(3);
            ASSERT_PTR_EQUALS(data + 3, sv.data());
            ASSERT_INT_EQUALS(3u, sv.size());
            ASSERT_TRUE(sv.front() == '1');
            ASSERT_TRUE(sv.back() == '3');
        }

        const char *data1 = "123456789";
        Aws::Crt::StringView sv1(data1);

        // test swap
        {
            sv.swap(sv1);
            ASSERT_PTR_EQUALS(data1, sv.data());
            ASSERT_INT_EQUALS(9u, sv.size());

            ASSERT_PTR_EQUALS(data + 3, sv1.data());
            ASSERT_INT_EQUALS(3u, sv1.size());
        }

        const char *data2 = "123456abc123xyzabc";
        Aws::Crt::StringView sv2(data2);

        // test find utils
        {
            ASSERT_INT_EQUALS(0u, sv2.find('1'));
            ASSERT_INT_EQUALS(10u, sv2.find('2', 3));
            ASSERT_INT_EQUALS(Aws::Crt::StringView::npos, sv2.find('A'));
            ASSERT_INT_EQUALS(6u, sv2.find("abc123", 0, 3));
            ASSERT_INT_EQUALS(Aws::Crt::StringView::npos, sv2.find("abc45", 0, 4));
            ASSERT_INT_EQUALS(Aws::Crt::StringView::npos, sv2.rfind("abc123", 0, 4));
            ASSERT_INT_EQUALS(6u, sv2.rfind("abc123", 13, 4));

            ASSERT_INT_EQUALS(6u, sv2.find_first_of("abc", 0, 2));
            ASSERT_INT_EQUALS(6u, sv2.find_first_of("abc", 0, 3));

            ASSERT_INT_EQUALS(16u, sv2.find_last_of("abc", 17, 2));
            ASSERT_INT_EQUALS(15u, sv2.find_last_of("abc", 16, 1));
            ASSERT_INT_EQUALS(17u, sv2.find_last_of("abc", Aws::Crt::StringView::npos, 3));

            ASSERT_INT_EQUALS(2u, sv2.find_first_not_of("123", 0, 2));
            ASSERT_INT_EQUALS(3u, sv2.find_first_not_of("123", 0, 4));

            ASSERT_INT_EQUALS(17u, sv2.find_last_not_of("abc", 17, 2));
            ASSERT_INT_EQUALS(16u, sv2.find_last_not_of("123", 16, 1));
            ASSERT_INT_EQUALS(15u, sv2.find_last_not_of("bc", Aws::Crt::StringView::npos, 3));
        }

        // test compare
        {
            ASSERT_TRUE(sv2.compare(sv) > 0);
            ASSERT_TRUE(sv.compare(sv2) < 0);
            ASSERT_TRUE(subsv.compare(sv2.substr(9, 4)) == 0);
        }

        size_t hashVal = std::hash<Aws::Crt::StringView>()(sv);
        size_t hashVal2 = std::hash<Aws::Crt::StringView>()(sv2);
        size_t subHashVal = std::hash<Aws::Crt::StringView>()(subsv);
        size_t subHashVal2 = std::hash<Aws::Crt::StringView>()(sv2.substr(9, 4));

        // test hash
        {
            ASSERT_TRUE(hashVal != hashVal2);
            ASSERT_UINT_EQUALS(subHashVal, subHashVal2);

            // compare with std::string hash
            ASSERT_UINT_EQUALS(std::hash<std::string>()("123456abc123xyzabc"), hashVal2);
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(StringViewTest, s_test_string_view)
