#ifndef TRACKINGMSG_H
#define TRACKINGMSG_H

#include "point2D.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <memory>

// Class providing message type for tracking queue
class TrackingMsg
{
  public:
    TrackingMsg(cv::Mat _frame, std::vector<Point2D> _p, float _delT):
    frame(std::move(_frame)), delT(_delT), p(std::move(_p)){};

    // Vector of 2D points
    std::vector<Point2D> p;

    // time deltaT is the time required in frame inference
    float delT;

    // Matrix containing the image frame
    cv::Mat frame;
};

#endif
