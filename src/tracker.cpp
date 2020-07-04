#include <tracker.h>

// Initialize the tracker
void Tracker::init(){
	_numTracks = 0;
	_maxAssociationDist = 0.5;
}

// Add the new tracks in the tracker and initialize them with the input
void Tracker::addTracks(std::vector<Point2D> &input, float delT)
{
    for(int i = 0; i < input.size() ; i++)
    {
        //std::cout << "Adding Track " << _numTracks << std::endl;
        tracks.push_back(std::make_unique<KalmanTrack>());
        tracks[tracks.size() - 1]->init(input[i], delT);
        //std::cout << "Added Track " << _numTracks << std::endl;
        _numTracks++;
    }
}

// Calculated the assignment based on hungarian algorithm
std::vector<int> Tracker::associate(std::vector<std::vector<double> > &costmatrix){
	std::vector<int> assignment;
	if(costmatrix.size() == 0)
	{
		return assignment;
	}
	HA.Solve(costmatrix, assignment);
	return assignment;
}

// computes cost between the two points
inline double computeCost(Point2D p1, Point2D p2, Point2D vel){
	return (std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.y-p2.y, 2))*(vel - (p2-p1)).norm());
}

// Find the distance of point p to the track
std::vector<double> Tracker::findDistToTracks(const Point2D &p){
	std::vector<double> out;
	for(auto &t : tracks){
		out.push_back(computeCost(t->getPos(), p, t->getVel()));
	}
	return out;
}

// Update the tracker with the new input
void Tracker::updateTracks(std::vector<Point2D> &input, float delT){
	//std::cout<< "--- Updating Tracks | input size = " << input.size() << " tracks" << tracks.size() << std::endl;

	// Run predition on available Tracks
	for(auto &t : tracks)
		t->predict(delT);

	//std::cout<< "--- Updating Tracks | Predicted on available Tracks" << std::endl;

	// Calculate cost matrix
	std::vector<std::vector<double>> costmatrix;
	std::vector<Point2D> new_tracks, measurement;
	for (auto i : input){
			std::vector<double> d = findDistToTracks(i);
			costmatrix.push_back(d);
	}

	//std::cout<< "--- Updating Tracks | Found costmatrix" << std::endl;

	// Find the mapping based on hungarian using the costmatrix
	std::vector<int> mapping;
	try
	{
		mapping = associate(costmatrix);
		//std::cout<< "--- Updating Tracks | Done association" << std::endl;
	}
	catch(...)
	{
		return;
	}
	
	// Add the new tracks and update the existing tracks based on mapping
	for(int i = 0; i < mapping.size(); i++){
		if(mapping[i] == -1){
			//std::cout<< "--- Updating Tracks | Adding to new track(Mapping not found) " << i << std::endl;
			new_tracks.push_back(input[i]);
		}
		else{
			if(costmatrix[i][mapping[i]] < _maxAssociationDist){
				//std::cout<< "--- Updating Tracks | Adding to new track(Mapping found - updating) " << i << std::endl;
				tracks[mapping[i]]->update(input[i]);
			}
			else
				//std::cout<< "--- Updating Tracks | Adding to new track( > _maxAssociationDist) " << i << std::endl;
				new_tracks.push_back(input[i]);
		}
	}

	// Add the new tracks to the Tracker
	addTracks(new_tracks, delT);
	//std::cout<< "--- Updating Tracks | Adding new tracks to main tracks " << tracks.size() << " " << _numTracks << std::endl;

	// Remove the tracks which are not updated from long time from the tracker
	removeTracks();
	//std::cout<< "--- Updating Tracks | Remaining tracks after removing tracks " << tracks.size() << " " << _numTracks << std::endl;
}

// Remove tracks from the tracker
void Tracker::removeTracks(){
	for(int t = 0; t < tracks.size();){
		if(!tracks[t]->isSane()){
			tracks.erase(tracks.begin() + t);
			t--;
			_numTracks--;
		}
	t++;
	}
}

// Return the tracked pos of all the available tracker
std::vector<Point2D> Tracker::getTracks(){
	std::vector<Point2D> out;
	int c = 0;
	for(auto &t : tracks){	
		//std::cout<< "demanding track pose" << c << std::endl;
		Point2D p = t->getPos();
		out.push_back(p);
		//std::cout<< "got track pose" << p.x << "  " << p.y << std::endl;
		c++;
	}
	return out;
}

//  Return the tracked pos and vel of all the available tracker
std::vector<Point2D> Tracker::getTracks(std::vector<Point2D> &vels){
	std::vector<Point2D> out;
	for(auto &t : tracks){
		Point2D p = t->getPos();
		Point2D v = t->getVel();
		out.push_back(p);
		vels.push_back(v);
	}
	return out;
}
