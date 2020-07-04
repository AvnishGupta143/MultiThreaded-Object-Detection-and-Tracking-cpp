#include <kalman_track.h>
#include <hungarian.h>
#include <vector>
#include <memory>

class Tracker
{
  private:

    // vector of unique pointers to tracks
	std::vector<std::unique_ptr<KalmanTrack>> tracks;

    // Count of tracks
	int _numTracks;

    // Max assosciated distance
	float _maxAssociationDist;

    // Hungarian Algorithm object
    HungarianAlgorithm HA;

    // To add tracks to initialised vector
	void addTracks(std::vector<Point2D> &input, float delT);

    // vector of type double to a func. with arg
	std::vector<double> findDistToTracks(const Point2D &p);

    // Data assosciation function
	std::vector<int> associate(std::vector<std::vector<double> > &costmatrix);

    // Remove the unwanted tracks
    void removeTracks();

  public:

    //Constructor
    Tracker(){};

    // Initialize the tracker
    void init();

    //Method to update tracks
	void updateTracks(std::vector<Point2D> &input, float delT);

    //Function overloading

	std::vector<Point2D> getTracks();

	std::vector<Point2D> getTracks(std::vector<Point2D> &vels);

};