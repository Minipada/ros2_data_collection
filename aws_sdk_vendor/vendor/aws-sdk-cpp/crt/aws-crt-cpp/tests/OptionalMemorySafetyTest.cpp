/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/Optional.h>
#include <aws/testing/aws_test_harness.h>

const char *s_test_str = "This is a string, that should be long enough to avoid small string optimizations";

static int s_OptionalCopySafety(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Optional<Aws::Crt::String> str1(s_test_str);
        Aws::Crt::Optional<Aws::Crt::String> strCpyAssigned = str1;
        Aws::Crt::Optional<Aws::Crt::String> strCpyConstructedOptional(strCpyAssigned);
        Aws::Crt::Optional<Aws::Crt::String> strCpyConstructedValue(*strCpyAssigned);

        // now force data access just to check there's not a segfault hiding somewhere.
        ASSERT_STR_EQUALS(s_test_str, str1->c_str());
        ASSERT_STR_EQUALS(s_test_str, strCpyAssigned->c_str());
        ASSERT_STR_EQUALS(s_test_str, strCpyConstructedOptional->c_str());
        ASSERT_STR_EQUALS(s_test_str, strCpyConstructedValue->c_str());
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(OptionalCopySafety, s_OptionalCopySafety)

static int s_OptionalMoveSafety(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Optional<Aws::Crt::String> str1(s_test_str);
        Aws::Crt::Optional<Aws::Crt::String> strMoveAssigned = std::move(str1);
        ASSERT_STR_EQUALS(s_test_str, strMoveAssigned->c_str());

        Aws::Crt::Optional<Aws::Crt::String> strMoveValueAssigned = std::move(*strMoveAssigned);
        ASSERT_STR_EQUALS(s_test_str, strMoveValueAssigned->c_str());

        Aws::Crt::Optional<Aws::Crt::String> strMoveConstructed(std::move(strMoveValueAssigned));
        ASSERT_STR_EQUALS(s_test_str, strMoveConstructed->c_str());

        Aws::Crt::Optional<Aws::Crt::String> strMoveValueConstructed(std::move(*strMoveConstructed));
        ASSERT_STR_EQUALS(s_test_str, strMoveValueConstructed->c_str());
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(OptionalMoveSafety, s_OptionalMoveSafety)

class EmplaceTester
{
  public:
    int a;
    static size_t ctorCallCount;
    static size_t dtorCallCount;
    EmplaceTester(int val) : a(val) { ctorCallCount += 1; }
    ~EmplaceTester()
    {
        a = -1337;
        dtorCallCount += 1;
    }
    EmplaceTester(const EmplaceTester &) = delete;
    EmplaceTester(EmplaceTester &&) = delete;
};
size_t EmplaceTester::ctorCallCount = 0;
size_t EmplaceTester::dtorCallCount = 0;

static int s_OptionalEmplace(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::Optional<Aws::Crt::String> str1{Aws::Crt::InPlace, s_test_str};
        ASSERT_STR_EQUALS(s_test_str, str1->c_str());

        ASSERT_INT_EQUALS(0, EmplaceTester::ctorCallCount);
        ASSERT_INT_EQUALS(0, EmplaceTester::dtorCallCount);
        // Aws::Crt::Optional<MyTestClass> opt1(MyTestClass(5)); // error: call to deleted constructor of 'MyTestClass'
        Aws::Crt::Optional<EmplaceTester> opt1{Aws::Crt::InPlace, 5}; // but this is allowed
        ASSERT_INT_EQUALS(5, opt1->a);
        ASSERT_INT_EQUALS(1, EmplaceTester::ctorCallCount);
        ASSERT_INT_EQUALS(0, EmplaceTester::dtorCallCount);

        opt1.emplace(100);
        ASSERT_INT_EQUALS(100, opt1->a);
        // If optional already contains a value before the call, the contained value is destroyed.
        ASSERT_INT_EQUALS(2, EmplaceTester::ctorCallCount);
        ASSERT_INT_EQUALS(1, EmplaceTester::dtorCallCount);
    }
    ASSERT_INT_EQUALS(2, EmplaceTester::dtorCallCount);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(OptionalEmplace, s_OptionalEmplace)

class CopyMoveTester
{
  public:
    struct Initer
    {
    };

    CopyMoveTester() = default;
    explicit CopyMoveTester(const Initer &) : m_initer_copied(true) {}
    explicit CopyMoveTester(Initer &&) : m_initer_moved(true) {}

    CopyMoveTester(const CopyMoveTester &) : m_copied(true), m_moved(false) {}
    CopyMoveTester(CopyMoveTester &&) : m_copied(false), m_moved(true) {}

    CopyMoveTester &operator=(const CopyMoveTester &)
    {
        m_copied = true;
        m_moved = false;
        m_initer_copied = false;
        m_initer_moved = false;
        return *this;
    }
    CopyMoveTester &operator=(CopyMoveTester &&)
    {
        m_copied = false;
        m_moved = true;
        m_initer_copied = false;
        m_initer_moved = false;
        return *this;
    }

    CopyMoveTester &operator=(const Initer &)
    {
        m_copied = false;
        m_moved = false;
        m_initer_copied = true;
        m_initer_moved = false;
        return *this;
    }

    CopyMoveTester &operator=(Initer &&)
    {
        m_copied = false;
        m_moved = false;
        m_initer_copied = false;
        m_initer_moved = true;
        return *this;
    }

    ~CopyMoveTester() {}

    bool m_copied = false;
    bool m_moved = false;
    bool m_initer_copied = false;
    bool m_initer_moved = false;
};

static int s_OptionalCopyAndMoveSemantics(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        CopyMoveTester initialItem;
        ASSERT_FALSE(initialItem.m_copied);
        ASSERT_FALSE(initialItem.m_moved);

        {
            // Optional(const U&), where U == T
            Aws::Crt::Optional<CopyMoveTester> copyConstructedValue(initialItem);
            ASSERT_TRUE(copyConstructedValue->m_copied);
            ASSERT_FALSE(copyConstructedValue->m_moved);

            // Optional(const Optional<U>&), where U == T
            Aws::Crt::Optional<CopyMoveTester> copyConstructedOptional(copyConstructedValue);
            ASSERT_TRUE(copyConstructedOptional->m_copied);
            ASSERT_FALSE(copyConstructedOptional->m_moved);
        }

        {
            // operator=(const U&), where U == T
            Aws::Crt::Optional<CopyMoveTester> copyAssignedValue;
            // Assignment to empty Optional.
            copyAssignedValue = initialItem;
            ASSERT_TRUE(copyAssignedValue->m_copied);
            ASSERT_FALSE(copyAssignedValue->m_moved);
            // Assignment to non-empty Optional.
            copyAssignedValue = initialItem;
            ASSERT_TRUE(copyAssignedValue->m_copied);
            ASSERT_FALSE(copyAssignedValue->m_moved);
        }

        {
            // operator=(const U&), where U != T
            Aws::Crt::Optional<CopyMoveTester> copyAssignedOtherValue;
            CopyMoveTester::Initer copyIniter;
            // Assignment to empty Optional.
            copyAssignedOtherValue = copyIniter;
            ASSERT_FALSE(copyAssignedOtherValue->m_copied);
            ASSERT_FALSE(copyAssignedOtherValue->m_moved);
            ASSERT_TRUE(copyAssignedOtherValue->m_initer_copied);
            ASSERT_FALSE(copyAssignedOtherValue->m_initer_moved);
            // Assignment to non-empty Optional.
            copyAssignedOtherValue = copyIniter;
            ASSERT_FALSE(copyAssignedOtherValue->m_copied);
            ASSERT_FALSE(copyAssignedOtherValue->m_moved);
            ASSERT_TRUE(copyAssignedOtherValue->m_initer_copied);
            ASSERT_FALSE(copyAssignedOtherValue->m_initer_moved);
        }

        {
            // operator=(const Optional<U>&), where U == T
            Aws::Crt::Optional<CopyMoveTester> copyAssignedOptional;
            Aws::Crt::Optional<CopyMoveTester> tester = CopyMoveTester();
            // Assignment to empty Optional.
            copyAssignedOptional = tester;
            ASSERT_TRUE(copyAssignedOptional->m_copied);
            ASSERT_FALSE(copyAssignedOptional->m_moved);
            // Assignment to non-empty Optional.
            copyAssignedOptional = tester;
            ASSERT_TRUE(copyAssignedOptional->m_copied);
            ASSERT_FALSE(copyAssignedOptional->m_moved);
        }

        {
            // operator=(const Optional<U>&), where U != T
            Aws::Crt::Optional<CopyMoveTester> copyAssignedOtherOptional;
            Aws::Crt::Optional<CopyMoveTester::Initer> copyIniterOptional = CopyMoveTester::Initer();
            // Assignment to empty Optional.
            copyAssignedOtherOptional = copyIniterOptional;
            ASSERT_FALSE(copyAssignedOtherOptional->m_copied);
            ASSERT_FALSE(copyAssignedOtherOptional->m_moved);
            ASSERT_TRUE(copyAssignedOtherOptional->m_initer_copied);
            ASSERT_FALSE(copyAssignedOtherOptional->m_initer_moved);
            // Assignment to non-empty Optional.
            copyAssignedOtherOptional = copyIniterOptional;
            ASSERT_FALSE(copyAssignedOtherOptional->m_copied);
            ASSERT_FALSE(copyAssignedOtherOptional->m_moved);
            ASSERT_TRUE(copyAssignedOtherOptional->m_initer_copied);
            ASSERT_FALSE(copyAssignedOtherOptional->m_initer_moved);
        }

        {
            // Optional(U&&), where U == T
            Aws::Crt::Optional<CopyMoveTester> moveConstructedValue(std::move(initialItem));
            ASSERT_FALSE(moveConstructedValue->m_copied);
            ASSERT_TRUE(moveConstructedValue->m_moved);

            // Optional(Optional<U>&&), where U == T
            Aws::Crt::Optional<CopyMoveTester> moveConstructedOptional(std::move(moveConstructedValue));
            ASSERT_FALSE(moveConstructedOptional->m_copied);
            ASSERT_TRUE(moveConstructedOptional->m_moved);
        }

        {
            // operator=(U&&), where U == T
            Aws::Crt::Optional<CopyMoveTester> moveAssignedValue;
            CopyMoveTester tester;
            // Assignment to empty Optional.
            moveAssignedValue = std::move(tester);
            ASSERT_FALSE(moveAssignedValue->m_copied);
            ASSERT_TRUE(moveAssignedValue->m_moved);
            // Assignment to non-empty Optional.
            tester = CopyMoveTester();
            moveAssignedValue = std::move(tester);
            ASSERT_FALSE(moveAssignedValue->m_copied);
            ASSERT_TRUE(moveAssignedValue->m_moved);
        }

        {
            // operator=(U&&), where U != T
            Aws::Crt::Optional<CopyMoveTester> moveAssignedOtherValue;
            CopyMoveTester::Initer moveIniter;
            // Assignment to empty Optional.
            moveAssignedOtherValue = std::move(moveIniter);
            ASSERT_FALSE(moveAssignedOtherValue->m_copied);
            ASSERT_FALSE(moveAssignedOtherValue->m_moved);
            ASSERT_FALSE(moveAssignedOtherValue->m_initer_copied);
            ASSERT_TRUE(moveAssignedOtherValue->m_initer_moved);
            // Assignment to non-empty Optional.
            moveIniter = CopyMoveTester::Initer();
            moveAssignedOtherValue = std::move(moveIniter);
            ASSERT_FALSE(moveAssignedOtherValue->m_copied);
            ASSERT_FALSE(moveAssignedOtherValue->m_moved);
            ASSERT_FALSE(moveAssignedOtherValue->m_initer_copied);
            ASSERT_TRUE(moveAssignedOtherValue->m_initer_moved);
        }

        {
            // operator=(Optional<U>&&), where U == T
            Aws::Crt::Optional<CopyMoveTester> moveAssignedOptional;
            Aws::Crt::Optional<CopyMoveTester> tester = CopyMoveTester();
            // Assignment to empty Optional.
            moveAssignedOptional = std::move(tester);
            ASSERT_FALSE(moveAssignedOptional->m_copied);
            ASSERT_TRUE(moveAssignedOptional->m_moved);
            // Assignment to non-empty Optional.
            tester = CopyMoveTester();
            moveAssignedOptional = std::move(tester);
            ASSERT_FALSE(moveAssignedOptional->m_copied);
            ASSERT_TRUE(moveAssignedOptional->m_moved);
        }

        {
            // operator=(Optional<U>&&), where U != T
            Aws::Crt::Optional<CopyMoveTester> moveAssignedOtherOptional;
            Aws::Crt::Optional<CopyMoveTester::Initer> moveIniterOptional = CopyMoveTester::Initer();
            // Assignment to empty Optional.
            moveAssignedOtherOptional = std::move(moveIniterOptional);
            ASSERT_FALSE(moveAssignedOtherOptional->m_copied);
            ASSERT_FALSE(moveAssignedOtherOptional->m_moved);
            ASSERT_FALSE(moveAssignedOtherOptional->m_initer_copied);
            ASSERT_TRUE(moveAssignedOtherOptional->m_initer_moved);
            // Assignment to non-empty Optional.
            moveIniterOptional = CopyMoveTester::Initer();
            moveAssignedOtherOptional = std::move(moveIniterOptional);
            ASSERT_FALSE(moveAssignedOtherOptional->m_copied);
            ASSERT_FALSE(moveAssignedOtherOptional->m_moved);
            ASSERT_FALSE(moveAssignedOtherOptional->m_initer_copied);
            ASSERT_TRUE(moveAssignedOtherOptional->m_initer_moved);
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(OptionalCopyAndMoveSemantics, s_OptionalCopyAndMoveSemantics)
