// The object-storage abstraction the Uploader (ADR-0005) is built on. `ObjectStore` is
// the minimal synchronous interface the upload logic needs — head/put plus the
// low-level multipart primitives whose persistent upload id makes interrupted transfers
// resumable. The real S3 implementation (aws-sdk-cpp) lives in src/uploader/
// s3_object_store.cpp; an in-memory fake in the tests exercises the logic without any
// cloud dependency, keeping the upload logic testable and aws-free.
#ifndef DC_BRIDGE__UPLOADER__OBJECT_STORE_HPP_
#define DC_BRIDGE__UPLOADER__OBJECT_STORE_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "dc_bridge/render.hpp"  // S3Params

namespace dc_bridge
{

/// Thrown by ObjectStore operations on a transport/remote error (as opposed to a
/// benign "object not found", which head() reports as nullopt).
class ObjectStoreError : public std::runtime_error
{
public:
  explicit ObjectStoreError(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

/// Minimal synchronous object-storage interface (aws-sdk-cpp is blocking, so there's no
/// async layer). Part numbers are 1-based (S3 convention); put_part returns the part's
/// ETag, which complete_multipart needs back.
class ObjectStore
{
public:
  virtual ~ObjectStore() = default;

  /// The object's size if it exists, nullopt if it does not. Throws ObjectStoreError on
  /// a transport error.
  virtual std::optional<std::uint64_t> head(const std::string& key) = 0;
  virtual void put(const std::string& key, const std::string& bytes) = 0;

  virtual std::string create_multipart(const std::string& key) = 0;
  virtual std::string put_part(const std::string& key, const std::string& upload_id, int part_number,
                               const std::string& bytes) = 0;
  virtual void complete_multipart(const std::string& key, const std::string& upload_id,
                                  const std::vector<std::string>& part_etags) = 0;
};

/// One `receives: files` Destination, ready to upload to. Holds an ObjectStore plus the
/// Humble-compatible status-row bits (name, url_prefix, key_prefix).
struct Storage
{
  std::string name;        ///< Destination name (remote_paths key + status storage_type).
  std::string url_prefix;  ///< "s3://<bucket>/" — the remote_path status field prefix.
  std::string key_prefix;  ///< Prepended to every object key (the Destination's key_prefix).
  std::shared_ptr<ObjectStore> store;

  std::string object_key(const std::string& remote_path) const
  {
    return key_prefix + remote_path;
  }
};

/// Builds an S3-backed Storage for a `type: s3, receives: files` Destination (defined in
/// s3_object_store.cpp, the only Uploader piece that links aws-sdk-cpp). Credentials /
/// endpoint semantics match the Records-side aws_s3 Vector sink. Call init_aws_api()
/// once before the first use.
Storage make_s3_storage(const std::string& name, const S3Params& params);

/// Initializes / shuts down the AWS SDK (Aws::InitAPI / Aws::ShutdownAPI). init is
/// idempotent; call once at process start before any make_s3_storage use.
void init_aws_api();
void shutdown_aws_api();

}  // namespace dc_bridge

#endif  // DC_BRIDGE__UPLOADER__OBJECT_STORE_HPP_
