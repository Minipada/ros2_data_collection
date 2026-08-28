/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/crt/Api.h>
#include <aws/crt/JsonObject.h>
#include <aws/crt/Types.h>
#include <aws/crt/endpoints/RuleEngine.h>
#include <aws/testing/aws_test_harness.h>

using namespace Aws::Crt;

const char sample_ruleset[] = R"({
          "version": "1.0",
          "serviceId": "example",
          "parameters": {
            "Region": {
              "type": "string",
              "builtIn": "AWS::Region",
              "documentation": "The region to dispatch the request to"
            }
          },
          "rules": [
            {
              "documentation": "rules for when region isSet",
              "type": "tree",
              "conditions": [
                {
                  "fn": "isSet",
                  "argv": [
                    {
                      "ref": "Region"
                    }
                  ]
                }
              ],
              "rules": [
                {
                  "type": "endpoint",
                  "conditions": [
                    {
                      "fn": "aws.partition",
                      "argv": [
                        {
                          "ref": "Region"
                        }
                      ],
                      "assign": "partitionResult"
                    }
                  ],
                  "endpoint": {
                    "url": "https://example.{Region}.{partitionResult#dnsSuffix}",
                    "headers": {
                      "x-amz-region": [
                        "{Region}"
                      ],
                      "x-amz-multi": [
                        "*",
                        "{Region}"
                      ]
                    },
                    "properties": {
                      "authSchemes": [
                        {
                          "name": "sigv4",
                          "signingName": "serviceName",
                          "signingRegion": "{Region}"
                        }
                      ]
                    }
                  }
                },
                {
                  "type": "error",
                  "documentation": "invalid region value",
                  "conditions": [],
                  "error": "unable to determine endpoint for region: {Region}"
                }
              ]
            },
            {
              "type": "endpoint",
              "documentation": "the single service global endpoint",
              "conditions": [],
              "endpoint": {
                "url": "https://example.amazonaws.com"
              }
            }
          ]
        })";

const char sample_partitions[] = R"({
    "version": "1.1",
    "partitions": [
      {
        "id": "aws",
        "regionRegex": "^(us|eu|ap|sa|ca|me|af)\\-\\w+\\-\\d+$",
        "regions": {
          "af-south-1": {
          },
          "af-east-1": {},
          "ap-northeast-1": {},
          "ap-northeast-2": {},
          "ap-northeast-3": {},
          "ap-south-1": {},
          "ap-southeast-1": {},
          "ap-southeast-2": {},
          "ap-southeast-3": {},
          "ca-central-1": {},
          "eu-central-1": {},
          "eu-north-1": {},
          "eu-south-1": {},
          "eu-west-1": {},
          "eu-west-2": {},
          "eu-west-3": {},
          "me-south-1": {},
          "sa-east-1": {},
          "us-east-1": {},
          "us-east-2": {},
          "us-west-1": {},
          "us-west-2": {},
          "aws-global": {}
        },
        "outputs": {
          "name": "aws",
          "dnsSuffix": "amazonaws.com",
          "dualStackDnsSuffix": "api.aws",
          "supportsFIPS": true,
          "supportsDualStack": true
        }
      }
    ]
  })";

static int s_TestRuleEngine(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;

    Aws::Crt::ApiHandle apiHandle(allocator);

    ByteCursor ruleset_cur = ByteCursorFromCString(sample_ruleset);
    ByteCursor partitions_cur = ByteCursorFromCString(sample_partitions);
    Aws::Crt::Endpoints::RuleEngine engine(ruleset_cur, partitions_cur, allocator);
    ASSERT_NOT_NULL(engine);
    Aws::Crt::Endpoints::RequestContext context(allocator);
    context.AddString(ByteCursorFromCString("Region"), ByteCursorFromCString("us-west-2"));

    auto resolved = engine.Resolve(context);
    ASSERT_TRUE(resolved.has_value());
    ASSERT_TRUE(resolved->IsEndpoint());

    ASSERT_TRUE(resolved->GetUrl()->compare("https://example.us-west-2.amazonaws.com") == 0);

    ASSERT_TRUE(resolved->GetHeaders()->at("x-amz-region")[0].compare("us-west-2") == 0);

    auto expected =
        R"(
    {
      "authSchemes": [
        {
          "name": "sigv4",
          "signingName": "serviceName",
          "signingRegion": "us-west-2"
        }
      ]
    })";

    String props(resolved->GetProperties()->begin(), resolved->GetProperties()->end());
    auto actual = JsonObject(props);
    ASSERT_TRUE(actual == JsonObject(expected));

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(RuleEngine, s_TestRuleEngine)

static int s_TestRuleEngineContextParams(struct aws_allocator *allocator, void *ctx)
{
    (void)ctx;

    Aws::Crt::ApiHandle apiHandle(allocator);

    Aws::Crt::Endpoints::RequestContext context(allocator);
    context.AddString(ByteCursorFromCString("Region"), ByteCursorFromCString("us-west-2"));
    context.AddBoolean(ByteCursorFromCString("AValidBoolParam"), false);
    context.AddStringArray(ByteCursorFromCString("StringArray1"), {});
    context.AddStringArray(
        ByteCursorFromCString("StringArray2"), {ByteCursorFromCString("a"), ByteCursorFromCString("b")});

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(RuleEngineContextParams, s_TestRuleEngineContextParams)
