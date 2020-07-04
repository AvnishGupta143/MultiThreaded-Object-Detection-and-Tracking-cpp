#include "yolo_object_detector.h"

// Constructor
YOLODetector::YOLODetector(std::string modelWeights, std::string modelConfig, std::string classesFilename, float confThresh, float nmsThresh)
{    
    _confidenceThresh = confThresh;
    _nonMaxSuppressionThresh = nmsThresh;
    _detector = std::make_unique<cv::dnn::Net>(cv::dnn::readNetFromDarknet(modelConfig, modelWeights));
    _detector->setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    _detector->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    if(_detector->empty())
    {
        std::cout << "Model Load Error" << std::endl;
        exit(-1);
    }
    else
    {
        std::cout << "Model Loaded Successfully!" << std::endl;
    }
    
    loadClasses(classesFilename);

    // For better detection set 416
    _inputWidth = 320;
    _inputHeight = 320;
}

// Destructor
YOLODetector::~YOLODetector(){};

void YOLODetector::detectObjectInQueue(MessageQueue<cv::Mat> &msgq, MessageQueue<cv::Mat> &outq)
{
    //wait while msg queue is filled with some frames
    while(msgq.is_empty()) std::this_thread::sleep_for(std::chrono::milliseconds(10));

    while(!msgq.is_empty())
    {
        cv::Mat frame = msgq.receive();
        std::vector<cv::Mat> info;
        this->detectObject(frame, info);
        _numFramesDetected ++;
        this->postProcessDetectedObjectFrame(frame, info);

        // Move the detected frame to outer queue
        outq.send(std::move(frame));

        if(_numFramesDetected%10 == 0)
        {
            std::cout << "Detected " << _numFramesDetected << " frames" << std::endl;
        }
    }
}

void YOLODetector::detectObjectInQueueAndTrack(MessageQueue<cv::Mat> &msgq, MessageQueue<TrackingMsg> &trackq)
{
    //wait while msg queue is filled with some frames
    while(msgq.is_empty()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    while(!msgq.is_empty())
    {
        cv::Mat frame = msgq.receive();
        std::vector<cv::Mat> info;
        this->detectObject(frame, info);
        _numFramesDetected ++;
        this->postProcessDetectedObjectFrame(frame, info);

        // Move the detected frame to outer queue
        TrackingMsg trackmsg(frame, _objectCenters, _frameDetectionTime);
        trackq.send(std::move(trackmsg));
        _objectCenters.clear();

        if(_numFramesDetected%10 == 0)
        {
            std::cout << "Detected " << _numFramesDetected << " frames" << std::endl;
        }
    }
}

void YOLODetector::detectObject(cv::Mat &frame, std::vector<cv::Mat> &info)
{
    // Create blob from the Image
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(_inputWidth, _inputHeight), cv::Scalar(0,0,0), true, false);
	
    // Sets the input to the network
	_detector->setInput(blob);

	// Runs the forward pass through the network to get output of the output layers
	_detector->forward(info, _detector->getUnconnectedOutLayersNames());
}

void YOLODetector::postProcessDetectedObjectFrame(cv::Mat &frame, const std::vector<cv::Mat> &info)
{
    this->addDetectionTimeToFrame(frame);

    //vector to hold classes
    std::vector<int> classIds;

    //vector to hold confidence
    std::vector<float> confidences;

    //vector to hold the bounding box positions
    std::vector<cv::Rect> boxes;

    for(int i = 0; i < info.size(); i++)
    {
        //scan through all the boonding boxes output and keep only those with high confidence scores
        float* data = (float*)info[i].data;
        for (int j = 0; j < info[i].rows; ++j, data += info[i].cols)
        {
            cv::Mat scores = info[i].row(j).colRange(5, info[i].cols);
            cv::Point classIdPoint;
	        double confidence;

            // Get the value and location of the maximum score
        	cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

            //check if confidence is greater than the thresh than add the classID, confidence and box to there respective vectors
            if (confidence > _confidenceThresh)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }

    // Apply nonmax suppression
    std::vector<int> box_indices;
    cv::dnn::NMSBoxes(boxes, confidences, _confidenceThresh, _nonMaxSuppressionThresh, box_indices);

    for (size_t i = 0; i < box_indices.size(); ++i)
	{
    	int idx = box_indices[i];
    	cv::Rect box = boxes[idx];
    	this->drawBoundingBoxToFrame(box.x, box.y, box.x + box.width, box.y + box.height, frame);
        this->addObjectClassToFrame(classIds[idx], confidences[idx], box.x, box.y, frame);
        Point2D p;
        p.x = (box.x + box.width + box.x)/2;
        p.y = (box.y + box.height + box.y)/2;
        _objectCenters.push_back(p);
	}
}

void YOLODetector::addDetectionTimeToFrame(cv::Mat &frame)
{
    std::vector<double> layersTimes;    
    float freq = cv::getTickFrequency() / 1000;
    float t = _detector->getPerfProfile(layersTimes) / freq;
    std::string label = "Inference time for the frame : " + std::to_string(t) + " ms";
    cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));
    _frameDetectionTime = t;
}
