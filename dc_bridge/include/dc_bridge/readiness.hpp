// Readiness: tracks whether Vector is currently accepting shipper ingest protocol
// connections, shared between a background prober and the ROS readiness service
// (ADR-0006 — the Bridge has no lifecycle state beyond "process up + ready service
// answers").
#ifndef DC_BRIDGE__READINESS_HPP_
#define DC_BRIDGE__READINESS_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace dc_bridge
{

/// A copyable handle over a shared ready/not-ready flag: copies share the same
/// underlying atomic (the background prober sets it, the service reads it).
class Readiness
{
public:
  Readiness() : ready_(std::make_shared<std::atomic<bool>>(false))
  {
  }

  void set_ready(bool ready)
  {
    ready_->store(ready, std::memory_order_seq_cst);
  }
  bool is_ready() const
  {
    return ready_->load(std::memory_order_seq_cst);
  }

private:
  std::shared_ptr<std::atomic<bool>> ready_;
};

/// A plain TCP connect probe against Vector's shipper ingest protocol listen address:
/// true means "accepting connections", false means it doesn't (yet, or anymore).
bool probe(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout);

}  // namespace dc_bridge

#endif  // DC_BRIDGE__READINESS_HPP_
