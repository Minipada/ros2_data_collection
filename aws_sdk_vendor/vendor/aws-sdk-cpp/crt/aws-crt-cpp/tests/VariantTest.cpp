/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/Variant.h>
#include <aws/testing/aws_test_harness.h>

const char *s_variant_test_str = "This is a string, that should be long enough to avoid small string optimizations";

static int s_VariantBasicOperandsCompile(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        {
            using MyTestVariant1 = Aws::Crt::Variant<int, char, Aws::Crt::String>;
            MyTestVariant1 var1;
            MyTestVariant1 var1CpyAssigned;
            var1CpyAssigned = var1;
            MyTestVariant1 var1CpyConstructedVariant(var1CpyAssigned);

            MyTestVariant1 var1a = Aws::Crt::String(s_variant_test_str);
            var1 = var1a;
            MyTestVariant1 var1aCpyAssigned;
            var1CpyAssigned = var1a;
            MyTestVariant1 var1aCpyConstructedVariant(var1aCpyAssigned);
        }

        {
            // just a different order or types
            using MyTestVariant2 = Aws::Crt::Variant<Aws::Crt::String, int, char>;
            MyTestVariant2 var2;
            MyTestVariant2 var2CpyAssigned;
            var2CpyAssigned = var2;
            MyTestVariant2 var2CpyConstructedVariant(var2CpyAssigned);

            MyTestVariant2 var2a = Aws::Crt::String(s_variant_test_str);
            var2 = var2a;
            MyTestVariant2 var2aCpyAssigned;
            var2CpyAssigned = var2a;
            MyTestVariant2 var2aCpyConstructedVariant(var2aCpyAssigned);
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(VariantCompiles, s_VariantBasicOperandsCompile)

static int s_VariantConstructor(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        {
            using VariantIntCharString = Aws::Crt::Variant<int, char, Aws::Crt::String>;
            VariantIntCharString var1 = Aws::Crt::String(s_variant_test_str);
            ASSERT_STR_EQUALS(s_variant_test_str, var1.get<2>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, var1.get<Aws::Crt::String>().c_str());

            VariantIntCharString var1copy = var1;
            ASSERT_STR_EQUALS(s_variant_test_str, var1copy.get<2>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, var1copy.get<Aws::Crt::String>().c_str());

            VariantIntCharString var1move = std::move(var1);
            ASSERT_STR_EQUALS(s_variant_test_str, var1move.get<2>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, var1move.get<Aws::Crt::String>().c_str());
        }
        {
            using VariantStringCharInt = Aws::Crt::Variant<Aws::Crt::String, int, char>;
            VariantStringCharInt var1{Aws::Crt::InPlaceTypeT<Aws::Crt::String>(), s_variant_test_str};
            ASSERT_STR_EQUALS(s_variant_test_str, var1.get<0>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, var1.get<Aws::Crt::String>().c_str());

            VariantStringCharInt var1copy = var1;
            ASSERT_STR_EQUALS(s_variant_test_str, var1copy.get<0>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, var1copy.get<Aws::Crt::String>().c_str());

            VariantStringCharInt var1move = std::move(var1);
            ASSERT_STR_EQUALS(s_variant_test_str, var1move.get<0>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, var1move.get<Aws::Crt::String>().c_str());

            VariantStringCharInt var1default;
            ASSERT_STR_EQUALS("", var1default.get<0>().c_str());
            ASSERT_STR_EQUALS("", var1default.get<Aws::Crt::String>().c_str());
        }

        {
            class MyTestVirtualClass
            {
              public:
                int *m_pState;

                MyTestVirtualClass(int *state) : m_pState(state) { *m_pState += 1; }

                virtual ~MyTestVirtualClass() { *m_pState -= 10; }
            };
            class MyTestVirtualClassChild : MyTestVirtualClass
            {
              public:
                int *m_pStateChild;

                MyTestVirtualClassChild(int *state, int *childState)
                    : MyTestVirtualClass(state), m_pStateChild(childState)
                {
                    *m_pStateChild += 2;
                }

                virtual ~MyTestVirtualClassChild() { *m_pStateChild -= 20; }
            };

            using MyTestVariant = Aws::Crt::Variant<MyTestVirtualClass, MyTestVirtualClassChild, Aws::Crt::String>;
            // Test for constructing from one of alternative types with a virtual destructor
            {
                int parentState = 0;
                int childState = 0;
                {
                    MyTestVariant myTestVariant{MyTestVirtualClassChild(&parentState, &childState)};
                    // original MyTestVirtualClassChild was constructed, implicitly moved to variant and destructed
                    ASSERT_INT_EQUALS(-9, parentState);
                    ASSERT_INT_EQUALS(-18, childState);
                }
                // destructor of move-constructed MyTestVirtualClassChild has updated the values by pointers one more
                // time
                ASSERT_INT_EQUALS(-19, parentState);
                ASSERT_INT_EQUALS(-38, childState);
            }
            // Test for in-place (without moving or copying) constructing from one of alternative types with a virtual
            // destructor
            {
                int parentState = 0;
                int childState = 0;
                {
                    MyTestVariant myTestVariant{
                        Aws::Crt::InPlaceTypeT<MyTestVirtualClassChild>(), &parentState, &childState};
                    // constructor of MyTestVirtualClassChild was called only once, destructor was not called (yet)
                    ASSERT_INT_EQUALS(1, parentState);
                    ASSERT_INT_EQUALS(2, childState);
                }
                // destructor MyTestVirtualClassChild has been called once
                ASSERT_INT_EQUALS(-9, parentState);
                ASSERT_INT_EQUALS(-18, childState);
            }
            // Test for in-place assignment
            {
                int parentState = 0;
                int childState = 0;
                {
                    MyTestVariant myTestVariant{
                        Aws::Crt::InPlaceTypeT<MyTestVirtualClassChild>(), &parentState, &childState};
                    myTestVariant.emplace<MyTestVirtualClass>(&parentState);
                    // both were destructed but only a parent got constructed once again
                    ASSERT_INT_EQUALS(-8, parentState);
                    ASSERT_INT_EQUALS(-18, childState);

                    myTestVariant.emplace<Aws::Crt::String>("A replacement string");
                    ASSERT_STR_EQUALS("A replacement string", myTestVariant.get_if<2>()->c_str());
                }
                ASSERT_INT_EQUALS(-18, parentState);
                ASSERT_INT_EQUALS(-18, childState); // destructor of child was not called
            }
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(VariantConstructor, s_VariantConstructor)

static int s_VariantOperatorEquals(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        {
            using VariantIntCharString = Aws::Crt::Variant<int, char, Aws::Crt::String>;

            VariantIntCharString var1(int(5));
            ASSERT_INT_EQUALS(5, var1.get<int>());

            VariantIntCharString var2(int(10));
            ASSERT_INT_EQUALS(10, var2.get<int>());

            var1 = var2;
            ASSERT_INT_EQUALS(10, var1.get<int>());

            VariantIntCharString varStr1{Aws::Crt::InPlaceTypeT<Aws::Crt::String>(), s_variant_test_str};
            ASSERT_STR_EQUALS(s_variant_test_str, varStr1.get<2>().c_str());
            VariantIntCharString varStr2;
            ASSERT_INT_EQUALS(0, varStr2.get<int>());
            varStr2 = varStr1;
            ASSERT_STR_EQUALS(s_variant_test_str, varStr1.get<Aws::Crt::String>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, varStr2.get<2>().c_str());

            const VariantIntCharString varStr3(std::move(varStr1));
            ASSERT_STR_EQUALS(s_variant_test_str, varStr3.get<Aws::Crt::String>().c_str());
            ASSERT_STR_EQUALS(s_variant_test_str, varStr3.get_if<2>()->c_str());
            ASSERT_TRUE(varStr1.get<Aws::Crt::String>().empty());

            VariantIntCharString varStr4(varStr3);
            ASSERT_STR_EQUALS(s_variant_test_str, varStr3.get_if<2>()->c_str()); // not moved
            ASSERT_STR_EQUALS(s_variant_test_str, varStr4.get_if<2>()->c_str()); // copied

            varStr1 = std::move(varStr4);
            ASSERT_TRUE(varStr4.get<2>().empty());                           // moved from
            ASSERT_STR_EQUALS(s_variant_test_str, varStr1.get<2>().c_str()); // moved here
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(VariantOperatorEquals, s_VariantOperatorEquals)

struct TestStringOnlyVisitor
{
    /* can't specialize member function templates, so using such syntax of dummy structs */
    template <typename... Args> struct MyVisitUtil
    {
        static void Visit(Args &...)
        {
            ; // not a string
        }
    };

    template <typename AlternativeT> void operator()(AlternativeT &val) const
    {
        MyVisitUtil<typename std::remove_reference<AlternativeT>::type>::Visit(val);
    }
};

template <> struct TestStringOnlyVisitor::MyVisitUtil<Aws::Crt::String>
{
    static void Visit(Aws::Crt::String &val)
    {
        auto index = val.find("another");
        val.replace(index, 7, "visited");
    }
};

static int s_VariantEmplace(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        {
            using VariantIntCharString = Aws::Crt::Variant<int, char, Aws::Crt::String>;

            VariantIntCharString var1(char('a'));
            ASSERT_INT_EQUALS('a', var1.get<char>());

            var1.emplace<int>(65535);
            ASSERT_INT_EQUALS(65535, var1.get<int>());

            var1.emplace<0>(1337);
            ASSERT_INT_EQUALS(1337, var1.get<int>());

            var1.emplace<Aws::Crt::String>(Aws::Crt::String("This is a string."));
            ASSERT_STR_EQUALS("This is a string.", var1.get<Aws::Crt::String>().c_str());

            var1.emplace<2>(Aws::Crt::String("This is another string."));
            ASSERT_STR_EQUALS("This is another string.", var1.get<Aws::Crt::String>().c_str());

            var1.Visit(TestStringOnlyVisitor());
            ASSERT_STR_EQUALS("This is visited string.", var1.get<Aws::Crt::String>().c_str());
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(VariantEmplace, s_VariantEmplace)

/* This is an example of a template visitor that accepts an alternative template and handles as a template */
struct TestVisitor
{
    Aws::Crt::String m_visitorResult;

    template <typename AlternativeT> void operator()(AlternativeT &val)
    {
        Aws::Crt::StringStream stringStream;
        stringStream << "Alternative value: " << val;
        m_visitorResult = stringStream.str();
    }
};

/* This is an example of a visitor that accepts an alternative template and has a specialization for types for custom
 * logic per type */
struct TestVisitorCustomizedPerType
{
    Aws::Crt::String m_visitorResult;
    /* can't specialize member function templates, so using such syntax of dummy structs */
    template <typename... Args> struct MyVisitUtil
    {
    };

    template <typename AlternativeT> void operator()(AlternativeT &val)
    {
        using TypedRealVisitor = MyVisitUtil<typename std::remove_reference<AlternativeT>::type>;
        m_visitorResult = TypedRealVisitor::Visit(val);
    }
};

template <> struct TestVisitorCustomizedPerType::MyVisitUtil<Aws::Crt::String>
{
    static Aws::Crt::String Visit(Aws::Crt::String &val) { return Aws::Crt::String("String has: " + val); }
};
template <> struct TestVisitorCustomizedPerType::MyVisitUtil<int>
{
    static Aws::Crt::String Visit(int &val)
    {
        Aws::Crt::StringStream stringStream;
        stringStream << "Int has: " << val;
        return stringStream.str();
    }
};
template <> struct TestVisitorCustomizedPerType::MyVisitUtil<char>
{
    static Aws::Crt::String Visit(char &val)
    {
        Aws::Crt::StringStream stringStream;
        stringStream << "Char has: " << val;
        return stringStream.str();
    }
};

static int s_VariantVisitor(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        {
            using VariantStringIntChar = Aws::Crt::Variant<int, char, Aws::Crt::String>;
            TestVisitor visitor;
            TestVisitorCustomizedPerType specializedVisitor;

            VariantStringIntChar var1(char('a'));

            var1.Visit(visitor);
            ASSERT_STR_EQUALS("Alternative value: a", visitor.m_visitorResult.c_str());
            var1.Visit(specializedVisitor);
            ASSERT_STR_EQUALS("Char has: a", specializedVisitor.m_visitorResult.c_str());

            var1.emplace<int>(5061992);
            var1.Visit(visitor);
            ASSERT_STR_EQUALS("Alternative value: 5061992", visitor.m_visitorResult.c_str());
            var1.Visit(specializedVisitor);
            ASSERT_STR_EQUALS("Int has: 5061992", specializedVisitor.m_visitorResult.c_str());

            var1.emplace<Aws::Crt::String>("Meow");
            var1.Visit(visitor);
            ASSERT_STR_EQUALS("Alternative value: Meow", visitor.m_visitorResult.c_str());
            var1.Visit(specializedVisitor);
            ASSERT_STR_EQUALS("String has: Meow", specializedVisitor.m_visitorResult.c_str());
        }
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(VariantVisitor, s_VariantVisitor)