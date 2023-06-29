#include "include/Frame.h"
using namespace std;
using namespace Eigen;

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

void Frame::setRotation(){}
void Frame::setTranslation(){}

Matrix3d Frame::getRotation(){}
Vector3d Frame::getTranslation(){}