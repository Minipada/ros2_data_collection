#include "dc_destinations/plugins/flb_null.hpp"

namespace dc_destinations
{

FlbNull::FlbNull() : dc_destinations::FlbDestination()
{
}

void FlbNull::onConfigure()
{
}

void FlbNull::initFlbOutputPlugin()
{
  out_ffd_ = flb_output(ctx_, "null", NULL);
  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output null plugin");
  }
}

FlbNull::~FlbNull() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbNull, dc_core::Destination)
