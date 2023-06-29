#ifndef UTILS_H
#define UTILS_H

#include "include/common.h"
#include "include/Plane.h"


class Utils{
public:
	
	/**
	 * @brief compute skew symmetric matrix of a 3D vector
	 * @return 3*3 matrix
	 */
	static Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d &v);
};

/**
 * @brief sort planes according to the number of points in descend order
 */
struct CompareByNumOfPlanePoints{
	bool operator()(const Plane& plane1, const Plane& plane2) const {
		return plane1.mPlanePoints->points.size() > plane2.mPlanePoints->points.size();
	}
};

#endif