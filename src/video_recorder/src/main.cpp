#include "rclcpp/rclcpp.hpp"
#include "video_recorder/mjpeg_recorder.hpp"
#include "video_recorder/video_recorder_subscriber.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto recorder = std::make_shared<video_recorder::MJPEGRecorder>();
    auto node = std::make_shared<video_recorder::VideoRecorderSubscriber>(options);
    node->Subscribe("raspicam/image_raw/compressed", [recorder](std::vector<unsigned char> frame){});
    rclcpp::spin(node);
    return 0;
}