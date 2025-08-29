#ifndef VIDEO_RECORDER_VIDEO_RECORDER_SUB_HPP
#define VIDEO_RECORDER_VIDEO_RECORDER_SUB_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace video_recorder {
    class VideoRecorderSubscriber: public rclcpp::Node
    {
        private:
            using FrameCallback = std::function<void(const std::vector<uint8_t>&)>;
            FrameCallback frame_callback_;
            image_transport::Subscriber sub_;
            void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        public:
            
            int Subscribe(const std::string& topic_name, FrameCallback callback);
            VideoRecorderSubscriber(rclcpp::NodeOptions& options);
            ~VideoRecorderSubscriber() = default;

    };
}
#endif