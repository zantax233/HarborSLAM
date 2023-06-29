#include "include/Plane.h"
using namespace std;

Plane& Plane::operator=(const Plane& plane){
		if(this == &plane)
			return *this;
		this->mDistance = plane.mDistance;
		this->mNormal = plane.mNormal;
		this->mCentroid = plane.mCentroid;
		this->mPlanePoints = plane.mPlanePoints;
		return *this;
}

void Plane::setDistance(){}
void Plane::setNormal(){}
void Plane::setCentroid(){}

double Plane::getDistance(){}
Eigen::Vector3d Plane::getNormal(){}
Eigen::Vector3d Plane::getCentroid(){}
