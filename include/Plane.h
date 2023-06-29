#ifndef PLANE_H
#define PLANE_H
#include "include/common.h"

class Plane
{
public:
	Plane() : mPlanePoints(new PointCloudT()), mDistance(0.0){
		mNormal = Eigen::Vector3d::Zero();
		mCentroid = Eigen::Vector3d(0.0, 0.0, 0.0);
	}
	~Plane(){}

    /**
     * @brief assign a new plane with a known plane
     */
    Plane& operator=(const Plane& plane);

    // the main parameters of a plane are its distance(to origin), normal vector and centroid(center)
    void setDistance();
    void setNormal();
    void setCentroid();

    double getDistance();
    Eigen::Vector3d getNormal();
    Eigen::Vector3d getCentroid();

public:
    PointCloudT::Ptr mPlanePoints;// cloud of a fitted plane

protected:
	double mDistance;
	Eigen::Vector3d mNormal;
	Eigen::Vector3d mCentroid;
	
};

#endif

