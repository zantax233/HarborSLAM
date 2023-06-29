#include "include/Track.h" 
using namespace std;

bool Track::mbInitialized = false;

void Track::preProcess(PointCloudT::Ptr &cloud_in){}

void Track::ExtractPlane(PointCloudT::Ptr &cloud_in){}
	
bool Track::InitialGuess(){}

bool Track::PoseRefinement(){}

void Track::processCloud(PointCloudT::Ptr &cloud_in){}

void Track::reset(int state){}