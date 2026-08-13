#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementStorageTest : public ::testing::Test
{
protected:
  MeasurementStorageTest()
  {
    SetUp();
  }

  ~MeasurementStorageTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "storage" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/storage", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementStorageTest::storageDataCallback, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void storageDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementStorageTest, PublishesFreeAndCapacityForConfiguredPath)
{
  ms_node_->declare_parameter("storage.plugin", std::string("dc_measurements/Storage"));
  ms_node_->declare_parameter("storage.group_key", std::string("storage"));
  ms_node_->declare_parameter("storage.topic_output", std::string("/dc/measurement/storage"));
  ms_node_->declare_parameter("storage.path", std::string("/tmp"));

  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  ASSERT_TRUE(data_json_.contains("capacity"));
  ASSERT_TRUE(data_json_.contains("free"));
  ASSERT_TRUE(data_json_.contains("free_percent"));

  auto capacity = data_json_["capacity"].get<int64_t>();
  auto free = data_json_["free"].get<int64_t>();
  auto free_percent = data_json_["free_percent"].get<double>();

  EXPECT_GT(capacity, 0);
  EXPECT_GE(free, 0);
  EXPECT_LE(free, capacity);
  EXPECT_GE(free_percent, 0.0);
  EXPECT_LE(free_percent, 100.0);
}

// A missing mandatory parameter (e.g. "storage.path", which has no default) is not something
// this suite can safely test: dc_util::get_param_or_fatal() calls exit(-1) directly rather than
// throwing a catchable exception, so it takes the whole test binary down -- including whatever
// other TEST_F cases share the process -- rather than just failing one assertion. Left untested
// here; a real regression test for that path would need a gtest death test in a suite of its own.

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
