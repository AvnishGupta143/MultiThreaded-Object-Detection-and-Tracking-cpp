#ifndef TRACK_H
#define TRACK_H

#include <deque>
#include "point2D.h"

class Track
{
  public:
	int last_measurement_time; 

    // Length of history of the track
	int history_length;

    // Previous pose
	Point2D prev_pos; 

    // queue containing previous pose
	std::deque<Point2D> history_pos; 

    // queue containing previous vel
	std::deque<Point2D> history_vel;

	// history size to maintain
	int history_size;

    //initialize state
	virtual void init(Point2D p, float delT) = 0;

    //measurement(measurement update) 
	virtual void update(Point2D p) = 0;
    
    //This function returns updated gaussian parameters, after motion by prediction
	virtual void predict(float delT) = 0;

    // Getter Functions
	virtual Point2D getPos() = 0; 
	virtual Point2D getVel() = 0;

    //Is the threshold for last measured time attained
	virtual bool isSane() = 0;
};

#endif