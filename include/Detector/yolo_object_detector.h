#ifndef YOLO_DETECTOR_H
#define YOLO_DETECTOR_H

#include <vector>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include "object_detector.h"
#include "point2D.h"
#include "tracking_msg.h"

// class which provides functionality to detect object using YOLO algorithm
class YOLODetector:  public ObjectDetector
{
  public:

    // Constructor and Destructor
    YOLODetector(std::string modelWeights, std::string modelConfig, std::string classesFilename, float confThresh, float nmsThresh);
    ~YOLODetector();

    // Method to detect object in the msgq having frames and put the detected frames in the outq
    void detectObjectInQueue(MessageQueue<cv::Mat> &msgq, MessageQueue<cv::Mat> &outq);

    // Method to detect object in the msgq having frames and put the detected frame and detected points in trackq
    void detectObjectInQueueAndTrack(MessageQueue<cv::Mat> &msgq, MessageQueue<TrackingMsg> &trackq);

    // Method to detect object in the given frame
    void detectObject(cv::Mat &frame, std::vector<cv::Mat> &info);

  protected:

    // Method to the bounding boxes over the confident predictions
    void postProcessDetectedObjectFrame(cv::Mat &frame, const std::vector<cv::Mat> &info);

    // Method to add inference time to detected object frame
    void addDetectionTimeToFrame(cv::Mat &frame);

  private:
    
    // Input width and height of the image to the network
    int _inputWidth;
    int _inputHeight;
    float _frameDetectionTime;
    std::vector<Point2D> _objectCenters;
};

#endif