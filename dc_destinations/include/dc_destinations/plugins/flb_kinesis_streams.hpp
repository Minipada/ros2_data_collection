#ifndef DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_KINESIS_STREAMS_HPP_
#define DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_KINESIS_STREAMS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_destinations/destination_server.hpp"
#include "dc_destinations/flb_destination.hpp"

namespace dc_destinations
{

class FlbKinesisStreams : public dc_destinations::FlbDestination
{
public:
  FlbKinesisStreams();
  ~FlbKinesisStreams();

protected:
  /**
   * @brief Configuration of behavior action
   */
  void initFlbOutputPlugin() override;
  void onConfigure() override;

  std::string region_;
  std::string stream_;
  std::string role_arn_;
  std::string time_key_;
  std::string time_key_format_;
  std::string log_key_;
  std::string endpoint_;
  std::string auto_retry_requests_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__PLUGINS__DESTINATIONS__FLB_KINESIS_STREAMS_HPP_
