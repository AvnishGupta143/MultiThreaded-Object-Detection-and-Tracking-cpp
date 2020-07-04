#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "message_queue.h"
#include "tracking_msg.h"
#include <tracker.h>

class ObjectTracker
{

  public:
    // Constructor / Destructor
    ObjectTracker();
    ~ObjectTracker();
  
    // Method to track object in the given frame
    void trackObject(std::vector<Point2D> &input, float &delT);

    // Method to track object in the queue having frames
    void trackObjectInQueue(MessageQueue<TrackingMsg> &msgq, MessageQueue<cv::Mat> &outq);

  private:
    
    // Tracker object
    Tracker tracker;

    // Number of frames tracked
    int _numFramesTracked;

    // Add tracking info to the frame
    void addTrackingInfoToFrame(cv::Mat &frame, std::vector<Point2D> &measured, std::vector<Point2D> &tracked);

};

#endif