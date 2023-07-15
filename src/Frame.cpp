#include "include/Frame.h"
using namespace std;
using namespace Eigen;

namespace HARBOR{

    Frame& Frame::operator=(const Frame& frame){
        if(this == &frame)
            return *this;
        this->mR_rel = frame.mR_rel;
        this->mt_rel = frame.mt_rel;
        this->mvMatchedPlanes = frame.mvMatchedPlanes;
        this->mInitialCloud = frame.mInitialCloud;
        int n = frame.mvPlanes.size();
        Plane plane;
        for(int i = 0; i < n; i++){
            plane = frame.mvPlanes[i];
            this->mvPlanes.push_back(plane);
        }
        return *this;
    }

    void Frame::setRotation(const Eigen::Matrix3d _rotation){ this->mR_rel = _rotation; }
    void Frame::setTranslation(const Eigen::Vector3d _translation){ this->mt_rel = _translation; }

    Matrix3d& Frame::getRotation(){ return this->mR_rel; }
    Vector3d& Frame::getTranslation(){ return this->mt_rel; }

}// namespace HARBOR