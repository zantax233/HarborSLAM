#ifndef PLANE_H
#define PLANE_H
#include "include/common.h"

namespace HARBOR{
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
        void setDistance(double _dist);
        void setNormal(Eigen::Vector3d _norm);
        void setCentroid(Eigen::Vector3d _cent);

        double getDistance();
        Eigen::Vector3d getNormal();
        Eigen::Vector3d getCentroid();

    public:
        PointCloudT::Ptr mPlanePoints;// cloud of a fitted plane

    protected:
        double mDistance;
        Eigen::Vector3d mNormal;
        Eigen::Vector3d mCentroid;
        
    };// class Plane

}// namepace HARBOR

#endif

