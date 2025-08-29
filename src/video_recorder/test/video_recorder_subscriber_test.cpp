#include <gtest/gtest.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "video_recorder/video_recorder_subscriber.hpp"

using namespace testing;

// class MockFrameHandler {
//     public:
//         MOCK_METHOD(void, onFrame, (const std::vector<unsigned char>&), ());
// };

TEST(VideoRecorderSubscriberTest, CallsFrameCallback) {
  //Arrange
  rclcpp::NodeOptions options;
  auto subscriberPtr = std::make_unique<video_recorder::VideoRecorderSubscriber>(options);
  subscriberPtr->Subscribe("/raspicam/image_raw/compressed", [](const std::vector<uint8_t>& msg){});
  //Act
  //Assert
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