// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// The S3 (aws-sdk-cpp) implementation of the ObjectStore interface — the only Uploader
// piece that links aws_sdk_vendor. Configured to match the Records-side aws_s3 Vector
// sink (explicit endpoint/region/path-style/credentials, or ambient AWS credentials).
// Verified against RustFS (PutObject + multipart) before adoption. Synchronous, so the
// upload logic needs no async runtime.
#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentials.h>
#include <aws/core/utils/memory/stl/AWSStringStream.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/S3ClientConfiguration.h>
#include <aws/s3/model/CompleteMultipartUploadRequest.h>
#include <aws/s3/model/CompletedMultipartUpload.h>
#include <aws/s3/model/CompletedPart.h>
#include <aws/s3/model/CreateMultipartUploadRequest.h>
#include <aws/s3/model/GetObjectRequest.h>
#include <aws/s3/model/HeadObjectRequest.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <aws/s3/model/UploadPartRequest.h>

#include <atomic>
#include <memory>
#include <sstream>

#include "dc_bridge/uploader/object_store.hpp"

namespace dc_bridge
{

namespace
{

Aws::SDKOptions& sdk_options()
{
  static Aws::SDKOptions options;
  return options;
}
std::atomic<bool> g_aws_initialized{ false };

class S3ObjectStore : public ObjectStore
{
public:
  S3ObjectStore(Aws::S3::S3Client client, std::string bucket) : client_(std::move(client)), bucket_(std::move(bucket))
  {
  }

  std::optional<std::uint64_t> head(const std::string& key) override
  {
    Aws::S3::Model::HeadObjectRequest req;
    req.SetBucket(bucket_);
    req.SetKey(key);
    auto outcome = client_.HeadObject(req);
    if (outcome.IsSuccess())
    {
      return static_cast<std::uint64_t>(outcome.GetResult().GetContentLength());
    }
    if (outcome.GetError().GetResponseCode() == Aws::Http::HttpResponseCode::NOT_FOUND)
    {
      return std::nullopt;
    }
    throw ObjectStoreError("head '" + key + "': " + outcome.GetError().GetExceptionName() + ": " +
                           outcome.GetError().GetMessage());
  }

  void put(const std::string& key, const std::string& bytes) override
  {
    Aws::S3::Model::PutObjectRequest req;
    req.SetBucket(bucket_);
    req.SetKey(key);
    auto body = Aws::MakeShared<Aws::StringStream>("dc_put");
    body->write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    req.SetBody(body);
    req.SetContentLength(static_cast<long long>(bytes.size()));
    auto outcome = client_.PutObject(req);
    if (!outcome.IsSuccess())
    {
      throw ObjectStoreError("put '" + key + "': " + outcome.GetError().GetExceptionName() + ": " +
                             outcome.GetError().GetMessage());
    }
  }

  std::string create_multipart(const std::string& key) override
  {
    Aws::S3::Model::CreateMultipartUploadRequest req;
    req.SetBucket(bucket_);
    req.SetKey(key);
    auto outcome = client_.CreateMultipartUpload(req);
    if (!outcome.IsSuccess())
    {
      throw ObjectStoreError("create_multipart '" + key + "': " + outcome.GetError().GetMessage());
    }
    return outcome.GetResult().GetUploadId();
  }

  std::string put_part(const std::string& key, const std::string& upload_id, int part_number,
                       const std::string& bytes) override
  {
    Aws::S3::Model::UploadPartRequest req;
    req.SetBucket(bucket_);
    req.SetKey(key);
    req.SetUploadId(upload_id);
    req.SetPartNumber(part_number);
    auto body = Aws::MakeShared<Aws::StringStream>("dc_part");
    body->write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    req.SetBody(body);
    req.SetContentLength(static_cast<long long>(bytes.size()));
    auto outcome = client_.UploadPart(req);
    if (!outcome.IsSuccess())
    {
      throw ObjectStoreError("put_part " + std::to_string(part_number) + " of '" + key +
                             "': " + outcome.GetError().GetMessage());
    }
    return outcome.GetResult().GetETag();
  }

  void complete_multipart(const std::string& key, const std::string& upload_id,
                          const std::vector<std::string>& part_etags) override
  {
    Aws::S3::Model::CompletedMultipartUpload completed;
    for (std::size_t i = 0; i < part_etags.size(); ++i)
    {
      Aws::S3::Model::CompletedPart part;
      part.SetPartNumber(static_cast<int>(i) + 1);
      part.SetETag(part_etags[i]);
      completed.AddParts(part);
    }
    Aws::S3::Model::CompleteMultipartUploadRequest req;
    req.SetBucket(bucket_);
    req.SetKey(key);
    req.SetUploadId(upload_id);
    req.SetMultipartUpload(completed);
    auto outcome = client_.CompleteMultipartUpload(req);
    if (!outcome.IsSuccess())
    {
      throw ObjectStoreError("complete_multipart '" + key + "': " + outcome.GetError().GetMessage());
    }
  }

private:
  Aws::S3::S3Client client_;
  std::string bucket_;
};

}  // namespace

void init_aws_api()
{
  bool expected = false;
  if (g_aws_initialized.compare_exchange_strong(expected, true))
  {
    Aws::InitAPI(sdk_options());
  }
}

void shutdown_aws_api()
{
  bool expected = true;
  if (g_aws_initialized.compare_exchange_strong(expected, false))
  {
    Aws::ShutdownAPI(sdk_options());
  }
}

Storage make_s3_storage(const std::string& name, const S3Params& params)
{
  init_aws_api();

  Aws::S3::S3ClientConfiguration cfg;
  if (params.region)
  {
    cfg.region = *params.region;
  }
  if (params.endpoint)
  {
    cfg.endpointOverride = *params.endpoint;
    // http endpoints are allowed — robot-LAN RustFS/MinIO deployments are rarely
    // TLS-terminated (matches the Records-side aws_s3 sink's allow_http behavior).
    if (params.endpoint->rfind("http://", 0) == 0)
    {
      cfg.scheme = Aws::Http::Scheme::HTTP;
      cfg.verifySSL = false;
    }
  }
  // force_path_style: true → path-style addressing (useVirtualAddressing = false),
  // required for custom endpoints. When unset, leave the SDK default.
  if (params.force_path_style)
  {
    cfg.useVirtualAddressing = !*params.force_path_style;
  }

  Aws::S3::S3Client client = [&]() {
    if (params.auth)
    {
      Aws::Auth::AWSCredentials creds(params.auth->access_key_id, params.auth->secret_access_key);
      return Aws::S3::S3Client(creds, nullptr, cfg);
    }
    // Ambient AWS credentials (environment / instance profile).
    return Aws::S3::S3Client(cfg);
  }();

  Storage storage;
  storage.name = name;
  storage.url_prefix = "s3://" + params.bucket + "/";
  storage.key_prefix = params.key_prefix.value_or("");
  storage.store = std::make_shared<S3ObjectStore>(std::move(client), params.bucket);
  return storage;
}

}  // namespace dc_bridge
