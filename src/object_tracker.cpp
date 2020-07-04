#include <object_tracker.h>

// Constructor
ObjectTracker::ObjectTracker()
{
    std::cout << "Initialised the tracker" << std::endl;
    _numFramesTracked = 0;

    // Initialize the object tracker
    tracker.init();
}

// Destructor
ObjectTracker::~ObjectTracker(){};

// TrackObject based on the measured location of the objects
void ObjectTracker::trackObject(std::vector<Point2D> &input, float &delT)
{
    //Update the tracker
    tracker.updateTracks(input, delT);
}

// Track Objects in the queue of Detected frames
void ObjectTracker::trackObjectInQueue(MessageQueue<TrackingMsg> &msgq, MessageQueue<cv::Mat> &outq)
{
    while(!msgq.is_empty())
    {
        // Receive msg from the queue
        TrackingMsg tracking_msg = msgq.receive();

        // Track object
        trackObject(tracking_msg.p, tracking_msg.delT);

        // Get the predicted tracks of the tracked objects
        std::vector<Point2D> tracked_points = tracker.getTracks();
        
        if (tracked_points.size() > 0) 
        {
            // Draw tracking info to frames
            addTrackingInfoToFrame(tracking_msg.frame, tracking_msg.p, tracked_points);
        }
	
        // Send the tracked frames to the queue
        outq.send(std::move(tracking_msg.frame));
        _numFramesTracked++;

        if(_numFramesTracked%10 == 0)
        {
            std::cout << "Tracked " << _numFramesTracked << " frames" << std::endl;
        }
    }
}

void ObjectTracker::addTrackingInfoToFrame(cv::Mat &frame, std::vector<Point2D> &measured, std::vector<Point2D> &tracked)
{
    // Add red circle to the Tracked location
    for(auto &p: tracked)
    {
        cv::circle(frame, cv::Point(p.x, p.y), 6, cv::Scalar(0,0,255), -1);
    }

    // Add green circle to the detected location
    for(auto &p: measured)
    {
        cv::circle(frame, cv::Point(p.x, p.y), 3, cv::Scalar(0,255,0), -1);
    }
}
