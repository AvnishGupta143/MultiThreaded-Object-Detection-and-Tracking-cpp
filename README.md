# MultiThreaded Object Detection and Tracking

## Description

This repository contains an implementation of a multithreaded application for detecting objects and tracking objects in a user-specified video.  Example output of running the application on the input video (`videos/input_video.mp4`) is the resulting video (`videos/project_track_and_detect.avi`).

The whole detection and tracking application runs on four asynchronous tasks. First task start reading frames using Frame Grabber object (`frame_grabber.h`) and pushes them them into a message queue (`message_queue.h`) which is thread safe implementation using conditional variables. The second tasks detects the frames by pulling the frames from message queue as they become available using Object Detector (`object_detector.h`). The class (`object_detector.h`) is a abtract class. After detection of each frame, the detector puts detected frame along with the required input data for tracking into another queue having a special message is type of tracking messages (`tracking_msg.h`). This two tasks run in parallel.
Once they are completed another task is started, which pulls the tracking messages from another queue and does the tracking using Object Tracker (`object_tracker.h`). The Object Tracker uses the Tracker (`tracker.h`) which provides functionality to update the tracks. The tracks are implements as abstract class (`track.h`). Each frame after tracking is put in the output queue.
After the completion of tracking task, the final task is started using Frame Writer object (`frame_writer.h`) which pulls the messages from the output queue and writes them to the user-specified output video file.

### Red dot on the objects shows the tracking and the green dot shows the position measured by detecting the object and calculating center of bounding box

<img src="data/output.gif"/>

## Detectors:

This project provides the abstact implementation of object detector, hence any custom object detector can be added. Currently, YOLO3-Object-Detector is implemented(`yolo_object_detector.h`).

### YOLO3-Object-Detector

This object detector is trained on coco dataset and can recognize upto 80 different classes of objects in the moving frame.

## Trackers: Kalman

The project provides abstract implementation of the tracker in form of tracks, hence any custom object tracker can be added. Currently, kalman tracker (`kalman_track.h`) is implemented using Kalman-Filter (`kalman_filter.h`). An assignment problem is used to associate the objects detected by detectors and tracked by trackers. Here it is solved using Hungarian Algorithm.

Tracking consist of two major steps:
1. Prediction: Predict the object locations in the next frame.
2. Data association: Use the predicted locations to associate detections across frames to form tracks.

## Dependencies for Running Locally
* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* OpenCV >= 4.1
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* Eigen3
  * Install using "sudo apt-get install libeigen3"

## Basic Build Instructions

1. Clone this repo.

2. Run the following commands to download object detection models from [Darknet](https://pjreddie.com/darknet/)

```
cd CppND-Capstone
mkdir model && cd model
wget https://pjreddie.com/media/files/yolov3.weights
wget https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg?raw=true -O ./yolov3.cfg
```

3. Make a build directory in the top level directory: `mkdir build && cd build`

4. Compile: `cmake .. && make`

5. Run it: `./detect_and_track` (**Runtime can vary depending on the number of frames in the video**)

6. Progress can tracked while the program is running
```
Model Loaded Successfully!
Loaded 80 Class Names
Initialised the tracker
Read 10 frames from total 1260 frames
Read 20 frames from total 1260 frames
Read 30 frames from total 1260 frames
Read 40 frames from total 1260 frames
Read 50 frames from total 1260 frames
....
....
Detected 10 frames
Read 520 frames from total 1260 frames
Detected 20 frames
Read 530 frames from total 1260 frames
Detected 30 frames
Read 540 frames from total 1260 frames
....
....
Tracked 1240 frames
Tracked 1250 frames
Written 1253 frames
-------  Done !!  -------

7. The video can be played from (`videos/project_track_and_detect.avi`) after the program terminates

## Code Instructions

1. Change the model_config file path and model_weight_path in main.cpp
2. Change the input and output video file path in main.cpp
2. Turn the track flag to false in main.cpp if you want to run detection only

## Satisfied Rubric Points

### The application reads data from a file and process the data.

* The project reads frames from a video file using the `ObjectDetector` class (`line 46 include/Detector/object_detector.h`).

### The application implements the abstract classes and pure virtual functions.

* The object detector class (`include/Detector/object_detector.h`) and the tracks class (`include/Tracker/track.h`).

### Use of Object Oriented Programming techniques.

* The classes `FrameGrabber` (`src/frame_grabber.cpp` , `include/frame_grabber.h`), `FrameWriter` (`src/frame_writer.cpp` , `include/frame_writer.h`), `MessageQueue` (`include/message_queue.h`), `YOLODetector` (`include/Detector/yolo_object_detector.h`, `src/yolo_object_detector.cpp`), `KalmanFilter` (`include/Tracker/kalman_filter.h` , `src/kalman_filter.cpp`), `Tracker` (`include/Tracker/tracker.h` , `src/tracker.cpp`) and `KalmanTrack` (`include/Tracker/kalman_track.h` , `src/kalman_track.cpp`).

### Use of Inheritence techniques.

* The class `YOLODetector` is inherited from parent class `ObjectDetector` (`line 15 include/Detector/yolo_object_detector.h`).
* The class `KalmanTrack` is inherited from parent class `Track` (`line 8 include/Tracker/yolo_object_detector.h`).

### Classes use appropriate access specifiers for class members.

* Example of class-specific access specifiers in `Tracker` class definition (`include/Tracker/tracker.h`).

### Use of Overloaded Functions.

* The function getTracks() is overloaded in `Tracker` class (`line 47,49 include/Tracker/tracker.h`).

### Templates generalize functions.

* The `MessageQueue` is a templated class (`include/message_queue.h`).

### Use of references in function declarations.

* Example of method that uses reference in function declaration is the implementation of the `YOLODetector` constructor (`line 4 src/yolo_object_detector.cpp`).

### Use of scope / Resource Acquisition Is Initialization (RAII) where appropriate.

* Example use of RAII can be seen acquisition and release of locks in `MessageQueue` (`lines 19,55,62 include/message_queue.h`).

### Use of move semantics to move data, instead of copying it, where possible.

* Example use of move semantics is the pushing and removing of items  in `MessageQueue` (`line 41 include/message_queue.h`).

### Use of smart pointers.

* Example use of the smart pointers (std::unique_ptr) is (`line 7 src/yolo_object_detector.cpp`), (`line 15 src/tracker.cpp`).

### Use of multithreading.

* The project uses two asynchronous tasks (std::async) (`lines 54,57,63,68 src/main.cpp`).

### Use of condition variable.

* A condition variable is used in the project in the implementation of message queue (`lines 38,28 include/message_queue.h`).

## References

* The video used in this repository was taken from the repository [udacity/CarND-Vehicle-Detection](udacity/CarND-Vehicle-Detection).

* OpenCV YOLO Object Detection (https://github.com/opencv/opencv/blob/master/samples/dnn/object_detection.cpp)

* Kalman Filter - [Artificial Intelligence for Robotics](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373#) Udacity Course

* Hungarian Algorithm - [here](http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem)

* Motion-Based Multiple Object Tracking - [here](https://in.mathworks.com/help/vision/examples/motion-based-multiple-object-tracking.html)

* Multiple Object Tracking - [here](https://in.mathworks.com/help/vision/ug/multiple-object-tracking.html)

* Computer Vision for tracking - [here](https://towardsdatascience.com/computer-vision-for-tracking-8220759eee85)
