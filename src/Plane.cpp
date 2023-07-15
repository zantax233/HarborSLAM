#include "include/Plane.h"
using namespace std;

namespace HARBOR{

	Plane& Plane::operator=(const Plane& plane){
		if(this == &plane)
			return *this;
		this->mDistance = plane.mDistance;
		this->mNormal = plane.mNormal;
		this->mCentroid = plane.mCentroid;
		this->mPlanePoints = plane.mPlanePoints;
		return *this;
	}

	void Plane::setDistance(double _dist) { this->mDistance = _dist; }
	void Plane::setNormal(Eigen::Vector3d _norm) { this->mNormal = _norm; }
	void Plane::setCentroid(Eigen::Vector3d _cent) { this->mCentroid = _cent; }

	double Plane::getDistance(){ return this->mDistance; }
	Eigen::Vector3d Plane::getNormal(){ return this->mNormal; }
	Eigen::Vector3d Plane::getCentroid(){ return this->mCentroid; }

}// namespace HARBOR