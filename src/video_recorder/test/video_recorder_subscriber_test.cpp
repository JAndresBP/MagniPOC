#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <vector>
#include <functional>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "video_recorder/video_recorder_subscriber.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <sensor_msgs/image_encodings.hpp>

using namespace testing;
using namespace std::chrono_literals;

class MockFrameHandler {
    public:
        MOCK_METHOD(void, onFrame, (const std::vector<unsigned char>&), ());
};

TEST(VideoRecorderSubscriberTest, CallsFrameCallback) {
  //Arrange
  MockFrameHandler mock;
  EXPECT_CALL(mock,onFrame(ElementsAre(1,2,3))).Times(1);

  //create subscription
  rclcpp::NodeOptions options;
  auto subscriber = std::make_shared<video_recorder::VideoRecorderSubscriber>(options);
  subscriber->Subscribe("/raspicam/image_raw", std::bind(&MockFrameHandler::onFrame, &mock, std::placeholders::_1));
  
  //create fake publisher
  auto curr_time = subscriber->now();
  auto publisher_node = rclcpp::Node::make_shared("test_pub_node");

  image_transport::ImageTransport it(publisher_node);
  auto publisher = it.advertise("/raspicam/image_raw",1);
  
  //define expected message
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = curr_time;
  msg->height = 1;
  msg->width = 3;
  msg->encoding = sensor_msgs::image_encodings::MONO8;
  msg->is_bigendian = false;
  msg->step = msg->width;
  msg->data = {1,2,3};
  
  //Act
  publisher.publish(std::move(msg));

  auto start_time = subscriber->now();
  while (rclcpp::ok() && (subscriber->now() - start_time) <= rclcpp::Duration(2s)) {
    rclcpp::spin_some(subscriber->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
}

// TEST(VideoRecorderSubscriberTest, SubscribeThrowsAnExcpetionWhenTopicIsNullOrEmpty) {
//   ClassA a;
//   EXPECT_THROW(a.divide(1, 0), std::runtime_error);
// }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}