#include "frame_grabber.h"

FrameGrabber::FrameGrabber(std::string filename)
{
    // Make unique pointer using filename
    _vcap = std::make_unique<cv::VideoCapture>(filename);

    // Check if the video file is opened without error
    if(!_vcap->isOpened())
    {
        std::cout << "Error: not able to open Video File/ Video File not Found" << std::endl;
        exit(-1);
    }
    
    // Load frame Rate of the video
    _frameRate = _vcap->get(cv::CAP_PROP_FPS);

    // Load video file codec format
    _fourCC = _vcap->get(cv::CAP_PROP_FOURCC);

    // Load frame size of the video
    _frameSize = cv::Size( _vcap->get(cv::CAP_PROP_FRAME_WIDTH), _vcap->get(cv::CAP_PROP_FRAME_HEIGHT));

    // Load number of frames in video
    _numFrames = _vcap->get(cv::CAP_PROP_FRAME_COUNT);
}

FrameGrabber::~FrameGrabber()
{
    // Release the video capture object
    _vcap->release();
}

void FrameGrabber::grabFramesFromVideo(MessageQueue<cv::Mat> &msgq)
{
    int count;
    cv::Mat frame;
    // Read and capture the frames untill there is frame in the video stream
    
    while(_vcap->read(frame))
    {
	    //Prevent overloading of queue
	    while(msgq.get_size() >= 500) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); }

        // Add the frame in the queue using 'send, of the message queue
        msgq.send(std::move(frame));

        // Increment the count
        ++count;

        // Print the number of frames read every 20 frames
        if(count % 10 == 0)
        {
            std::cout << "Read " << count << " frames from total " << _numFrames << " frames" << std::endl;
        }
    }

    // Check all the frames in the video is read
    if (count == _numFrames)
    {
        std::cout << "Read the whole video file having " << count << " frames" << std::endl;     
    }
    else
    {
        _numFrames = count;
        std::cout << "Read " << count << " frames" << std::endl;     
    }    
}


