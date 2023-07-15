#ifndef TRACK_H
#define TRACK_H

#include "include/Plane.h"
#include "include/Frame.h"
#include "include/Utils.h"
#include "include/parameter.h"

namespace HARBOR{
	class Track{
	public:
		Track(std::string _configPath, std::string _poseFile);
		/**
		 * @brief remove invalid points and adjust coordinates
		 * @param [in&out] cloud_in
		 */
		void preProcess(PointCloudT::Ptr &cloud_in);

		/**
		 * @brief extract plane features(compute normal, centroid, distance)
		 * step1 estimate normals of each point
		 * step2 apply region growth to compute all the clusters
		 * step3 extract plane params using RANSAC and reserve inliers
		 * step4 compute centroid, normal and distance, and sort planes
		 * step5 visualization is optional
		 * @param [in] cloud_in : input cloud
		 * @param [out] pCur : fill in Cur with mvPlanes
		 */
		void ExtractPlane(PointCloudT::Ptr &cloud_in);

		/**
		 * @brief iteratively conducting plane correspondence and pose estimation
		 * @param[in] pRef : reference frame
		 * @param[in] pCur : current frame
		 * @param[in&out] mvMatchedPlanes, pRef.mR_rel, pRef.mt_rel
		 * @return bool : indicate success or failure
		 */
		bool InitialGuess();

		/**
		 * @brief pose optimization using GN
		 * @param[in] pRef : reference frame
		 * @param[in] pCur : current frame
		 * @param[in&out] pRef.mR_rel, pRef.mt_rel
		 * @return bool : indicate success or failure
		 */
		bool PoseRefinement();
		
		/**
		 * @brief reset cur frame or ref frame
		 */
		void reset(int state);

		/**
		 * @brief main thread
		 */
		void processCloud(PointCloudT::Ptr &cloud_in);

	public:
		Frame pRef, pCur;
		std::string mPoseFile;
		enum ResetState{ ResetRef, ResetCur, ResetBoth };
		static bool mbInitialized;
	
	protected:
		std::shared_ptr<Parameter> mParamPtr;

	};// class Track

}// namespace HARBOR

#endif