#ifndef VIDEO_RECORDER_MJPEG_RECORDER_HPP
#define VIDEO_RECORDER_MJPEG_RECORDER_HPP

#include <string>
#include <vector>
#include "video_recorder/video_recorder.hpp"

namespace video_recorder {
    class MJPEGRecorder: public VideoRecorder {
        
        public:
            ~MJPEGRecorder();
            MJPEGRecorder();    
            int StartRecording() override;
            int StopRecording() override;
            
    };
}

#endif