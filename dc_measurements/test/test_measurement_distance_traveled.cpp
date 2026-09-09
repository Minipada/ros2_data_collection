// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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

  // Bounded poll of canTransform on a test-owned tf listener: sendTransform is fire-and-forget,
  // so the fixture must not assume the frame is already there. Keeps re-broadcasting because a
  // listener that subscribed after the first sendTransform would otherwise never get one.
  void waitForMapFrame()
  {
    auto tf_wait_node = std::make_shared<rclcpp::Node>("distance_traveled_tf_wait");
    tf2_ros::Buffer tf_buffer(tf_wait_node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer, tf_wait_node, /*spin_thread=*/true);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!tf_buffer.canTransform("map", "base_link", tf2::TimePointZero))
    {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline)
          << "map->base_link never reached a tf buffer -- broadcast likely never left the fixture";
      broadcastMapToBaseLink(3.0, 4.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST_F(MeasurementDistanceTraveledTest, PublishesDistanceFromOriginOnFirstFix)
{
  declareCommonParameters();

  // The plugin's timer starts at activation: have the frame in tf first, so the first collect()
  // cycle is a real fix rather than an empty Record.
  waitForMapFrame();
  startLifecycleNode();

  // last_x_/last_y_ start at (0, 0), so the first successful transform lookup reports the
  // straight-line distance from the origin. Assert on the *first* Record carrying
  // "distance_traveled": with a static pose every later cycle reports 0.0 from the previous
  // fix, and a loaded runner can let a second cycle queue up before the spin runs, so the
  // latest Record is not necessarily the one under test (#519). Keep re-broadcasting inside the
  // wait: earlier collect() cycles may fire before the transform is in the tf buffer, publishing
  // an empty "{}" Record (no distance key) instead. tf2_ros::TransformListener spins on its own
  // background thread (MeasurementServer never passed it an explicit node/executor, so it
  // defaults to one); the sleep between spins in spinUntil gives that thread real scheduling
  // opportunities instead of this loop busy-spinning a core out from under it.
  double first_distance = 0.0;
  ASSERT_TRUE(spinUntil(
      [&] {
        broadcastMapToBaseLink(3.0, 4.0);
        if (callback_active_ && data_json_.contains("distance_traveled"))
        {
          first_distance = data_json_["distance_traveled"].get<double>();
          return true;
        }
        return false;
      },
      10000))
      << "never observed a Record with \"distance_traveled\" -- tf broadcast likely never reached the buffer";

  // sqrt(3^2 + 4^2)
  EXPECT_NEAR(first_distance, 5.0, 1e-2);
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
