#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <fstream>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include "message_queue.h"
#include "point2D.h"
#include "tracking_msg.h"

// Abstract class which provides functionality to detect object in the frames and have some functions to write the video file
class ObjectDetector
{
  public:

    // Constructor / Destructor
    ObjectDetector(){};
    ~ObjectDetector(){};
  
    // Method to detect object in the given frame
    virtual void detectObject(cv::Mat &frame, std::vector<cv::Mat> &info) = 0;

    // Method to detect object in the queue having frames
    virtual void detectObjectInQueue(MessageQueue<cv::Mat> &msgq, MessageQueue<cv::Mat> &outq) = 0;

    // Method to detect object in the msgq having frames and put the detected frame and detected points in trackq
    virtual void detectObjectInQueueAndTrack(MessageQueue<cv::Mat> &msgq, MessageQueue<TrackingMsg> &trackq) = 0;

    // Method to find the bounding boxes over the confident predictions
    virtual void postProcessDetectedObjectFrame(cv::Mat &frame, const std::vector<cv::Mat> &info) = 0;

    // Function to add inference time to the detected object frame
    virtual void addDetectionTimeToFrame(cv::Mat &frame) = 0;

    // Draws detected object bounding boxes on supplied video frame
    inline void drawBoundingBoxToFrame(int left, int top, int right, int bottom, cv::Mat& frame)
    {
      //Draw a rectangle displaying the bounding box
      cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255));
    }

    // Function to add object class to the detected object frame
    inline void addObjectClassToFrame(int classId, float conf, int left, int top, cv::Mat& frame)
    {
      std::string label = std::to_string(conf);
      if (!_classes.empty())
	    { 
      	label = _classes[classId] + ":" + label;
    	}
      else
      {
        return;
      }
      
      //Add the label at the top of the bounding box
      int baseLine;
      cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseLine);
      top = std::max(top, labelSize.height);
      cv::rectangle(frame, cv::Point(left, top), cv::Point(left+labelSize.width, top - std::min(top, labelSize.height)), cv::Scalar(255, 0, 255), -1);
      cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255));
    }

    // Function to load the class names in a vector
    inline void loadClasses(std::string classesFilename)
    {
      std::fstream file(classesFilename);
      std::string line;
      while(std::getline(file, line))
      {
        _classes.push_back(line);
      }
      std::cout << "Loaded " << _classes.size() <<" Class Names" << std::endl;
    }

    // Confidence threshold for class
    float _confidenceThresh;

    // Non-max Suppression threshold to remove the access bounding boxes
    float _nonMaxSuppressionThresh;

    // Unique Pointer holding the Object Dectector
    std::unique_ptr<cv::dnn::Net> _detector;

    // Vector containing class names
    std::vector<std::string> _classes;

    // Frames Detected
    int _numFramesDetected{0};

};

#endif
