#ifndef VIDEO_RECORDER_VIDEO_RECORDER_HPP
#define VIDEO_RECORDER_VIDEO_RECORDER_HPP

#include <string>
#include <vector>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>

namespace video_recorder {
    class VideoRecorder {
        
        public:
            virtual int StartRecording();
            virtual int StopRecording();
            virtual ~VideoRecorder();
        
        protected:
            VideoRecorder(std::string topic);

        private:
            std::string topic_;
            std::vector<std::byte> input_buffer_;
    };
}

#endif