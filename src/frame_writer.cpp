#include "frame_writer.h"


CustomFrameWriter::CustomFrameWriter(std::string out_filename, int numFrames, double frameRate, cv::Size frameSize):
_numFrames(numFrames)
{
    // Make unique pointer of video writer object using filenama, codec , fps and frame_size
    _vWrite = std::make_unique<cv::VideoWriter>(out_filename, cv::VideoWriter::fourcc('M','J','P','G'), frameRate, frameSize);
}

CustomFrameWriter::~CustomFrameWriter()
{
    // Release the video writer object
    _vWrite->release();
}

bool CustomFrameWriter::writeFramesFromQueueToVideo(MessageQueue<cv::Mat> &msgq)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    try
    {
        int count = 0;

	// if msg queue is not empty, write the frames 
        while(!msgq.is_empty())
        {
            cv::Mat frame = msgq.receive();
            _vWrite->write(frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            count++;
        }
        
        if(count == 0) return false;

        std::cout << "Written " << count << " frames" << std::endl;

        return true;
    }
    catch(...)
    {
        return false;
    }
}
