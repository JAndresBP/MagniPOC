#include "video_recorder/video_recorder_subscriber.hpp"
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace video_recorder {
    VideoRecorderSubscriber::VideoRecorderSubscriber(rclcpp::NodeOptions& options)
        :Node(__func__, options)
    {
        this->declare_parameter<std::string>("image_transport", "compressed");
    };

    int VideoRecorderSubscriber::Subscribe(const std::string& topic_name, FrameCallback callback){
        
        RCLCPP_INFO(this->get_logger(),"create subscriber");
        frame_callback_ = callback;
        
        image_transport::ImageTransport it{shared_from_this()};
        image_transport::TransportHints hints(this);
        
        sub_ = it.subscribe(topic_name,1,&VideoRecorderSubscriber::ImageCallback,this,&hints);
        return 0;
    };

    void VideoRecorderSubscriber::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
        RCLCPP_INFO(this->get_logger(),"msg received");
        if(frame_callback_ != nullptr){
            RCLCPP_INFO(this->get_logger(),"invoke callback");
            frame_callback_(msg->data);
        }
    };
}