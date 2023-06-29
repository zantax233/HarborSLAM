#include "include/Utils.h"
using namespace std;
using namespace Eigen;

Matrix3d Utils::skewSymmetricMatrix(Vector3d &v){
    Matrix3d Sx = Matrix3d::Zero();
    Sx <<  0,    -v[2],  v[1],
           v[2],  0,    -v[0],
          -v[1],  v[0],  0;
    return Sx;
}