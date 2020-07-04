#ifndef FRAME_GRABBER_H
#define FRAME_GRABBER_H

#include <memory>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "message_queue.h"

// Class which provides functionality to load and grab frames from video in using OpenCV
class FrameGrabber
{
  public:

    // Constructor and Destructor

    FrameGrabber(std::string filename);
    ~FrameGrabber();

    // Getter Functions
   
    cv::Size getFrameSize() {return _frameSize; }
    double getFrameRate() {return _frameRate; }
    int getVideoCodec() {return _fourCC; }
    int getNumberOfFrames() {return _numFrames; }

    // Function to capture frames from video
    void grabFramesFromVideo(MessageQueue<cv::Mat> &msgq);

  private:
    // Unique pointer holding the video capture object
    std::unique_ptr<cv::VideoCapture> _vcap;

    // Frame Rate of the video
    double _frameRate;

    // Video file codec format
    int _fourCC = 0;

    // Frame size of the video
    cv::Size _frameSize;

    // Number of frames in video
    int _numFrames;
};

#endif

