#include "kalman_track.h"

// Initialize the track for 4 states - position x , position y, vel x, vel y
void KalmanTrack::init(Point2D p, float delT)
{
  //std::cout<< "Kalman Track Init | Start " << std::endl;

  int n = 4; //number of states
	last_measurement_time = 0;
  history_length = 0;
  history_size = 6;

  KF = std::make_unique<KalmanFilter>(delT);
  //std::cout<< "Kalman Track Init | Made Kalman Filter " << std::endl;

  Eigen::VectorXd v(n);
  v<<p.x,p.y,0,0;
  KF->init(v);
  //std::cout<< "Kalman Track Init | Initialized Kalman Filter with " << v.transpose() << std::endl;
}

// Update the Track
void KalmanTrack::update(Point2D p)
{
  Point2D vel = p - prev_pos;
  prev_pos = p;
  Eigen::VectorXd v(4);
  v << p.x, p.y, vel.x/last_measurement_time, vel.y/last_measurement_time;
  // Update the Kalman
  KF->update(v);
  // Set last measurement time to 0
  last_measurement_time = 0;
  history_length++;
} 

void KalmanTrack::predict(float delT)
{
  // Predict the Kalman
  KF->predict(delT);

  last_measurement_time++;

  // if history size is greater than the history_size to maintain, pop the last element so than new element can be added
  if(history_pos.size() >= history_size){
    history_pos.pop_front();
    history_vel.pop_front();
  }
  history_pos.push_back(getPos());
  history_vel.push_back(getVel());
}

// Return the current tracked pos of the track
Point2D KalmanTrack::getPos()
{
  Eigen::MatrixXd stateM = KF->state();
  Point2D p;
  p.x = stateM(0);
  p.y = stateM(1);
  return p;
}

// Return the current tracked velocity of the track
Point2D KalmanTrack::getVel()
{
  Eigen::MatrixXd stateM = KF->state();
  Point2D p;
  p.x = stateM(2);
  p.y = stateM(3);

  if(history_pos.size()<history_size)
    return p;
  // Take average of all the previous velocities
  for(auto v:history_vel)
    p=v+p;
  
  p = p/(history_size+1);
  
  return p;
}

// Return false if particular track has not been updated since last 10 times
bool KalmanTrack::isSane()
{
  return (last_measurement_time < 10);
}
