/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/JsonObject.h>
#include <aws/testing/aws_test_harness.h>

static int s_BasicJsonParsing(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        const Aws::Crt::String jsonValue =
            "{\"testStringKey\":\"testStringValue\", \"testIntKey\":10, "
            "\"testBoolKey\":false, \"array\": [\"stringArrayEntry1\", \"stringArrayEntry2\"], "
            "\"object\": {\"testObjectStringKey\":\"testObjectStringValue\"}}";

        Aws::Crt::JsonObject value(jsonValue);
        ASSERT_TRUE(value.WasParseSuccessful());
        auto view = value.View();
        ASSERT_TRUE(value.GetErrorMessage().empty());
        ASSERT_TRUE("testStringValue" == view.GetString("testStringKey"));
        ASSERT_INT_EQUALS(10, view.GetInteger("testIntKey"));
        ASSERT_FALSE(view.GetBool("testBoolKey"));
        ASSERT_TRUE(view.GetJsonObject("object").AsString().empty());
        ASSERT_TRUE("stringArrayEntry1" == view.GetArray("array")[0].AsString());
        ASSERT_TRUE("stringArrayEntry2" == view.GetArray("array")[1].AsString());
        ASSERT_TRUE("testObjectStringValue" == view.GetJsonObject("object").GetString("testObjectStringKey"));
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(BasicJsonParsing, s_BasicJsonParsing)

static int s_JsonNullParseTest(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        const Aws::Crt::String jsonValue = "{\"testStringKey\":null,\"testIntKey\":10,"
                                           "\"array\":[null,\"stringArrayEntry\"],"
                                           "\"object\":{\"testObjectStringKey\":null}}";

        Aws::Crt::JsonObject value(jsonValue);
        ASSERT_TRUE(value.WasParseSuccessful());

        auto str = value.View().WriteCompact(true);
        ASSERT_STR_EQUALS(jsonValue.c_str(), str.c_str());
        str = value.View().WriteCompact(false);
        ASSERT_STR_EQUALS(jsonValue.c_str(), str.c_str());
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(JsonNullParsing, s_JsonNullParseTest)

static int s_JsonNullNestedObjectTest(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        const Aws::Crt::String jsonValue = "{\"testStringKey\":null,\"testIntKey\":10,"
                                           "\"array\":[null,\"stringArrayEntry\"],"
                                           "\"object\":{\"testObjectStringKey\":null}}";

        Aws::Crt::JsonObject value(jsonValue);
        ASSERT_TRUE(value.WasParseSuccessful());

        Aws::Crt::JsonObject doc;
        doc.WithObject("null_members", jsonValue);

        const Aws::Crt::String expectedValue = "{\"null_members\":{\"testStringKey\":null,\"testIntKey\":10,"
                                               "\"array\":[null,\"stringArrayEntry\"],"
                                               "\"object\":{\"testObjectStringKey\":null}}}";
        auto str = doc.View().WriteCompact(true);
        ASSERT_STR_EQUALS(expectedValue.c_str(), str.c_str());
        str = doc.View().WriteCompact(false);
        ASSERT_STR_EQUALS(expectedValue.c_str(), str.c_str());
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(JsonNullNestedObject, s_JsonNullNestedObjectTest)

static int s_JsonExplicitNullTest(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        const Aws::Crt::String expectedValue = "{\"testKey\":null}";

        Aws::Crt::JsonObject doc;
        Aws::Crt::JsonObject nullObject;
        nullObject.AsNull();

        doc.WithObject("testKey", nullObject);

        auto str = doc.View().WriteCompact(true);
        ASSERT_STR_EQUALS(expectedValue.c_str(), str.c_str());
        str = doc.View().WriteCompact(false);
        ASSERT_STR_EQUALS(expectedValue.c_str(), str.c_str());
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(JsonExplicitNull, s_JsonExplicitNullTest)

static int s_JsonBoolTest(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        Aws::Crt::JsonObject object;
        object.WithBool("my_true_bool", true).WithBool("my_false_bool", false);

        ASSERT_TRUE(object.View().GetJsonObject("my_false_bool").IsBool());  // pass
        ASSERT_FALSE(object.View().GetJsonObject("my_false_bool").AsBool()); // pass
        ASSERT_FALSE(object.View().GetBool("my_false_bool"));                // pass

        ASSERT_TRUE(object.View().GetJsonObject("my_true_bool").IsBool()); // pass
        ASSERT_TRUE(object.View().GetJsonObject("my_true_bool").AsBool()); // pass
        ASSERT_TRUE(object.View().GetBool("my_true_bool"));                // fail ?!?!
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(JsonBoolTest, s_JsonBoolTest)

static int s_JsonMoveTest(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;
    {
        Aws::Crt::ApiHandle apiHandle(allocator);

        // use WithArray(key, &&obj) to move objectVector into object1
        Aws::Crt::Vector<Aws::Crt::JsonObject> objectVector;
        objectVector.resize(3);
        objectVector[0].WithString("a", "AAAAAAAA");
        objectVector[1].WithString("b", "BBBBBBBB");
        objectVector[2].WithString("c", "CCCCCCCC");

        Aws::Crt::JsonObject object1;
        object1.WithArray("arrayOfObjs", std::move(objectVector));
        ASSERT_TRUE(object1.View().IsObject());
        ASSERT_TRUE(object1.View().GetJsonObject("arrayOfObjs").IsListType());
        ASSERT_UINT_EQUALS(3, object1.View().GetArray("arrayOfObjs").size());
        ASSERT_TRUE(object1.View().GetArray("arrayOfObjs")[0].GetString("a") == "AAAAAAAA");

        // use move-constructor to replace object1 with object2
        Aws::Crt::JsonObject object2(std::move(object1));
        ASSERT_FALSE(object1.View().IsObject());
        ASSERT_TRUE(object2.View().GetJsonObject("arrayOfObjs").IsListType());
        ASSERT_UINT_EQUALS(3, object2.View().GetArray("arrayOfObjs").size());
        ASSERT_TRUE(object2.View().GetArray("arrayOfObjs")[0].GetString("a") == "AAAAAAAA");

        // use move-copy to replace object2 with object3
        Aws::Crt::JsonObject object3;
        object3 = std::move(object2);
        ASSERT_FALSE(object2.View().IsObject());
        ASSERT_TRUE(object3.View().GetJsonObject("arrayOfObjs").IsListType());
        ASSERT_TRUE(object3.View().GetJsonObject("arrayOfObjs").IsListType());
        ASSERT_UINT_EQUALS(3, object3.View().GetArray("arrayOfObjs").size());
        ASSERT_TRUE(object3.View().GetArray("arrayOfObjs")[0].GetString("a") == "AAAAAAAA");

        // use AsObject(&&) to replace object3 with object4
        Aws::Crt::JsonObject object4;
        object4.AsObject(std::move(object3));
        ASSERT_FALSE(object3.View().IsObject());
        ASSERT_TRUE(object4.View().GetJsonObject("arrayOfObjs").IsListType());
        ASSERT_UINT_EQUALS(3, object4.View().GetArray("arrayOfObjs").size());
        ASSERT_TRUE(object4.View().GetArray("arrayOfObjs")[0].GetString("a") == "AAAAAAAA");

        // use AsArray(&&) to move objectVector into jsonArray
        Aws::Crt::Vector<Aws::Crt::JsonObject> anotherObjectVector;
        anotherObjectVector.resize(2);
        anotherObjectVector[0].WithString("zero", "Number#0");
        anotherObjectVector[1].WithString("one", "Number#1");

        Aws::Crt::JsonObject jsonArray;
        jsonArray.AsArray(std::move(anotherObjectVector));
        ASSERT_TRUE(jsonArray.View().IsListType());
        ASSERT_UINT_EQUALS(2, jsonArray.View().AsArray().size());
        ASSERT_TRUE(jsonArray.View().AsArray()[0].GetString("zero") == "Number#0");
    }
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(JsonMoveTest, s_JsonMoveTest)
