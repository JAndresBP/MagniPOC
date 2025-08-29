#ifndef VIDEO_RECORDER_VIDEO_RECORDER_HPP
#define VIDEO_RECORDER_VIDEO_RECORDER_HPP

#include <string>
#include <vector>

namespace video_recorder {
    class VideoRecorder {
        
        public:
            virtual int StartRecording();
            virtual int StopRecording();
            virtual ~VideoRecorder();
        
        protected:
            VideoRecorder();

        private:
            std::vector<unsigned char> input_buffer_;
    };
}

#endif