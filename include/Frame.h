#ifndef FRAME_H
#define FRAME_H
#include "include/common.h"
#include "include/Plane.h"

namespace HARBOR{
    class Frame{
    public:
        Frame() : mInitialCloud(new PointCloudT()){
            mvPlanes.clear();
            mR_rel = Eigen::Matrix3d::Zero();
            mt_rel = Eigen::Vector3d::Zero();
        }
        ~Frame(){}

        /**
         * @brief assign a new frame with a known frame
         */
        Frame& operator=(const Frame& frame);

        void setRotation(const Eigen::Matrix3d _rotation);
        void setTranslation(const Eigen::Vector3d _translation);

        Eigen::Matrix3d& getRotation();
        Eigen::Vector3d& getTranslation();
        
    public:
        PointCloudT::Ptr mInitialCloud;
        std::vector<Plane> mvPlanes;// set of cur planes
        std::vector<int> mvMatchedPlanes;//key : ref planes; value : target planes

    protected:
        Eigen::Matrix3d mR_rel;
        Eigen::Vector3d mt_rel;
        
    };// class Frame

}// namespace HARBOR

#endif