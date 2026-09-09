// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "measurement_test_bench.hpp"

class MeasurementPositionTest : public MeasurementBench
{
protected:
  MeasurementPositionTest() : MeasurementBench("position")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ms_node_);
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("position.plugin", std::string("dc_measurements/Position"));
    ms_node_->declare_parameter("position.group_key", std::string("position"));
    ms_node_->declare_parameter("position.topic_output", std::string("/dc/measurement/position"));
    ms_node_->declare_parameter("position.polling_interval", 50);
  }

  void broadcastMapToBaseLink(double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = ms_node_->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST_F(MeasurementPositionTest, PublishesPoseFromTransform)
{
  declareCommonParameters();
  startLifecycleNode();

  // The very first collect() cycles may fire before the transform below is in the tf buffer,
  // yielding an empty Record ("{}"), which still gets published; keep broadcasting until a
  // Record carrying "x" actually shows up (a later Record supersedes an earlier empty one in
  // data_json_). tf2_ros::TransformListener spins on its own background thread
  // (MeasurementServer never passed it an explicit node/executor, so it defaults to one); the
  // sleep between spins in spinUntil gives that thread real scheduling opportunities instead of
  // this loop busy-spinning a core out from under it.
  const double yaw = 1.5707963267948966;  // pi / 2
  ASSERT_TRUE(spinUntil(
      [&, yaw] {
        broadcastMapToBaseLink(2.0, 3.0, yaw);
        return callback_active_ && data_json_.contains("x");
      },
      10000))
      << "never observed a Record with \"x\" -- tf broadcast likely never reached the buffer";

  EXPECT_NEAR(data_json_["x"].get<double>(), 2.0, 1e-3);
  EXPECT_NEAR(data_json_["y"].get<double>(), 3.0, 1e-3);
  EXPECT_NEAR(data_json_["yaw"].get<double>(), yaw, 1e-3);
}

TEST_F(MeasurementPositionTest, NoTransformProducesNoPublish)
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
