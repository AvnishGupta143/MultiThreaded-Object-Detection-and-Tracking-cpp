#ifndef KALMAN_TRACK_H
#define KALMAN_TRACK_H

#include <memory>
#include "kalman_filter.h"
#include "track.h"

class KalmanTrack : public Track
{
    // Pointer to Kalman Filter Object
	std::unique_ptr<KalmanFilter> KF;

  public:

    //initialize state
	void init(Point2D p, float delT);

    //measurement(measurement update) 
	void update(Point2D p);
    
    //This function returns updated gaussian parameters, after motion by prediction
	void predict(float delT);

    // Getter Functions
	Point2D getPos(); 
	Point2D getVel();

    //Is the threshold for last measured time attained
	bool isSane();

};

#endif