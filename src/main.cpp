#include <iostream>
#include <future>
#include <string>
#include "frame_grabber.h"
#include "message_queue.h"
#include "yolo_object_detector.h"
#include "frame_writer.h"
#include "object_tracker.h"
#include "tracking_msg.h"

int main() {
    // load input video path
    std::string video_input = "../videos/input_video.mp4";

    // load model config path
    std::string model_config = "../model/yolov3.cfg";

    // load model weights path
    std::string model_weights = "../model/yolov3.weights";

    // load class names path
    std::string class_names = "../model/coco.names";

    // load output video path
    std::string video_output1 = "../videos/project_detect.avi";
    std::string video_output2 = "../videos/project_track_and_detect.avi";

    bool track = true;
    bool detect = true;
    
    // Initialize frame grabber object
    FrameGrabber fg(video_input);

    // Initialize yolo object detector object
    YOLODetector yolo(model_weights, model_config, class_names, 0.5, 0.4);

    // Initialize object tracker object
    ObjectTracker trck;


    // Initialize the input and output message queue
    MessageQueue<cv::Mat> in_queue;
    MessageQueue<cv::Mat> out_queue;

    if(detect && track)
    {
        // Initialize the tracking message queue
	MessageQueue<TrackingMsg> track_queue;

        // Initialize the frame writer object
        CustomFrameWriter fw(video_output2, fg.getNumberOfFrames(), fg.getFrameRate(), fg.getFrameSize());
    
        // Start the task to load the frames
        std::future<void> ftr_grab_frame = std::async(std::launch::async, &FrameGrabber::grabFramesFromVideo, &fg , std::ref(in_queue));
        
        // Start the task to detect the frames
        std::future<void> ftr_detect_frame = std::async(std::launch::async, &YOLODetector::detectObjectInQueueAndTrack , &yolo , std::ref(in_queue), std::ref(track_queue));

        ftr_grab_frame.wait();
        ftr_detect_frame.wait();

        // Start the task to track the frames
        std::future<void> ftr_track_frame = std::async(std::launch::async, &ObjectTracker::trackObjectInQueue , &trck , std::ref(track_queue), std::ref(out_queue));
        
        ftr_track_frame.wait();
        
        // Start the task to write the frames
        std::future<bool> ftr_write_frame = std::async(std::launch::async, &CustomFrameWriter::writeFramesFromQueueToVideo , &fw , std::ref(out_queue));

        ftr_write_frame.wait();
    }
    else
    {
        // Initialize the frame writer object
        CustomFrameWriter fw(video_output1, fg.getNumberOfFrames(), fg.getFrameRate(), fg.getFrameSize());

        // Start the task to load the frames
        std::future<void> ftr_grab_frame = std::async(std::launch::async, &FrameGrabber::grabFramesFromVideo, &fg , std::ref(in_queue));
        
        // Start the task to detect the frames
        std::future<void> ftr_detect_frame = std::async(std::launch::async, &YOLODetector::detectObjectInQueue , &yolo , std::ref(in_queue), std::ref(out_queue));

        ftr_grab_frame.wait();
        ftr_detect_frame.wait();

        std::future<bool> ftr_write_frame = std::async(std::launch::async, &CustomFrameWriter::writeFramesFromQueueToVideo , &fw , std::ref(out_queue));

        ftr_write_frame.wait();
    }

    std::cout << "-------  Done !!  -------" << "\n";

    return 0;
}
