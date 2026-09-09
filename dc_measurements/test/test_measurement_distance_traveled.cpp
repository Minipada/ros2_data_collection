// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "measurement_test_bench.hpp"

class MeasurementDistanceTraveledTest : public MeasurementBench
{
protected:
  MeasurementDistanceTraveledTest() : MeasurementBench("distance_traveled")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ms_node_);
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("distance_traveled.plugin", std::string("dc_measurements/DistanceTraveled"));
    ms_node_->declare_parameter("distance_traveled.group_key", std::string("distance_traveled"));
    ms_node_->declare_parameter("distance_traveled.topic_output", std::string("/dc/measurement/distance_traveled"));
    ms_node_->declare_parameter("distance_traveled.polling_interval", 50);
    // DistanceTraveled::onConfigure() reads "transform_timeout" via the non-throwing
    // get_parameter(name, out) overload without ever declaring it (a known pre-existing bug,
    // documented in distance_traveled.cpp) -- if left undeclared, transform_timeout_ keeps
    // whatever garbage value happened to be on the stack. Declared explicitly here so this test
    // exercises real, defined behaviour instead of UB.
    ms_node_->declare_parameter("distance_traveled.transform_timeout", 0.5);
  }

  void broadcastMapToBaseLink(double x, double y)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = ms_node_->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST_F(MeasurementDistanceTraveledTest, PublishesDistanceFromOriginOnFirstFix)
{
  declareCommonParameters();
  startLifecycleNode();

  // last_x_/last_y_ start at (0, 0), so the first successful transform lookup reports the
  // straight-line distance from the origin. Keep re-broadcasting until a Record carrying
  // "distance_traveled" shows up (earlier collect() cycles may fire before the transform is in
  // the tf buffer, publishing an empty "{}" Record instead, which a later Record supersedes in
  // data_json_). tf2_ros::TransformListener spins on its own background thread
  // (MeasurementServer never passed it an explicit node/executor, so it defaults to one); the
  // sleep between spins in spinUntil gives that thread real scheduling opportunities instead of
  // this loop busy-spinning a core out from under it.
  ASSERT_TRUE(spinUntil(
      [this] {
        broadcastMapToBaseLink(3.0, 4.0);
        return callback_active_ && data_json_.contains("distance_traveled");
      },
      10000))
      << "never observed a Record with \"distance_traveled\" -- tf broadcast likely never reached the buffer";

  // sqrt(3^2 + 4^2)
  EXPECT_NEAR(data_json_["distance_traveled"].get<double>(), 5.0, 1e-2);
}

TEST_F(MeasurementDistanceTraveledTest, NoTransformProducesNoPublish)
{
  declareCommonParameters();
  startLifecycleNode();

  // With no transform ever broadcast, every collect() cycle returns an empty StringStamped, and
  // Measurement::publish() (measurement.hpp) drops empty messages outright (logs a WARN, never
  // calls data_pub_->publish()) rather than publishing "{}" -- so the callback can never fire.
  // Poll through several collect() cycles (polling_interval=50) and assert it stays that way,
  // same pattern as test_measurement_ip_camera.cpp's NoSegmentsYetProducesNoPublish.
  spinFor(300);

  EXPECT_FALSE(callback_active_);
}

DC_MEASUREMENT_TEST_MAIN()
