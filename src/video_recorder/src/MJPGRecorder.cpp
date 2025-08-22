#include "video_recorder/VideRecorder.hpp"
#include <string>

namespace video_recorder {
    class MJPGRecorder: public video_recorder::VideoRecorder
    {
        public:
            MJPGRecorder(std::string& topic): VideoRecorder(topic) {
                
            }

            int StartRecording() override {

            }
    };
}



