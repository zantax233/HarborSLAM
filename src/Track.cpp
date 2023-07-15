#include "include/Track.h" 
using namespace std;

namespace HARBOR{

	bool Track::mbInitialized = false;

	Track::Track(std::string _configPath, std::string _poseFile){
		mParamPtr = std::make_shared<Parameter>(_configPath);
		mPoseFile = _poseFile;
	}

	void Track::preProcess(PointCloudT::Ptr &cloud_in){
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
	}

	void Track::ExtractPlane(PointCloudT::Ptr &in){
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);// normal and surface curvature
		vector <pcl::PointIndices> clusters;// output -> segmented clusters indices
		
		// 1 normal estimation
		pcl::NormalEstimation<PointT, pcl::Normal> ne;// normal estimator  
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());// for normal estimation
		ne.setInputCloud (in);
		ne.setSearchMethod (tree);
		ne.setKSearch (15);
		ne.setViewPoint(std::numeric_limits<double>::max (),std::numeric_limits<double>::max (), std::numeric_limits<double>::max ());
		ne.compute (*cloud_normals);//通过最近临搜索法找到每个点的最近临点的集合，然后通过这些最近点的集合分布来计算每个点的法线

		// 2 region growth
		pcl::RegionGrowing<PointT, pcl::Normal> reg;
		reg.setMinClusterSize (200);  
		reg.setMaxClusterSize (10000); 
		reg.setSearchMethod (tree);   
		reg.setNumberOfNeighbours (300);  //ini =30
		reg.setInputCloud (in);        
		reg.setInputNormals (cloud_normals);    
		reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI);  //ini =3.
		reg.setCurvatureThreshold (1.0);
		reg.extract (clusters);//保存聚类后每个 cluster 的索引

		for(int i = 0; i < clusters.size(); i++){
			Plane plane;

			PointCloudT::Ptr cluster_points( new PointCloudT());// stroing pointcloud of each cluster
			pcl::PointIndices indices = clusters[i];

			for(int j = 0; j < indices.indices.size(); j++)
				cluster_points->points.push_back(in->points[indices.indices[j]]);
				cluster_points->width = cluster_points->points.size();
				cluster_points->height = 1;
				cluster_points->is_dense = true;

				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
				pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
				
				// 3 extract planes using RANSAC
				pcl::SACSegmentation<PointT> seg;
				seg.setOptimizeCoefficients(true);
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setMaxIterations(1000);//init 1000
				seg.setDistanceThreshold(0.04);//init 0.05
				seg.setInputCloud(cluster_points);

				seg.segment(*inliers, *coefficients);
				if(inliers->indices.size() < 100){
					// too few points for a plane
					continue;
			}

			// 4 saving results
			PointCloudT::Ptr planeCloud(new PointCloudT());
			Eigen::Vector3d centroid = Eigen::Vector3d::Zero(); //每个 cluster 的质心

			for(int m = 0; m < inliers->indices.size(); m++){
				PointT point = cluster_points->points[ inliers->indices[m] ];
				planeCloud->points.push_back(point);
				centroid[0] += point.x;
				centroid[1] += point.y;
				centroid[2] += point.z;
			}
			planeCloud->width = planeCloud->points.size();
			planeCloud->height = 1;
			planeCloud->is_dense = true;
			centroid /= static_cast<float>(planeCloud->points.size());
			Eigen::Vector3d normal(0.0, 0.0, 0.0);
			double distance = 0.0;
			bool USE_REFINEMENT = false;
			
			normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
			distance = coefficients->values[3];
		
			plane.setNormal(normal);
			plane.setDistance(distance);
			plane.setCentroid(centroid);
			plane.mPlanePoints = planeCloud;
			// restoring to current frame
			pCur.mvPlanes.push_back(plane);// Ax + By + Cz + D = 0
		}

		// 5 sort by num of plane points
		std::sort(pCur.mvPlanes.begin(), pCur.mvPlanes.end(), CompareByNumOfPlanePoints());

		// 6 (optional) visualize results
		bool visPlanes = mParamPtr->mbVisualizePlanes;
		#if visPlanes
		// visualize the extracted planes
		pcl::visualization::PCLVisualizer viewer ("Plane viewer");
		viewer.setBackgroundColor(0.0, 0.0, 0.0);
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr plane_cloud( new pcl::PointCloud <pcl::PointXYZRGB>() );

		int r = 0, g = 85, b = 170;
		for(int i = 0; i < pCur.mvPlanes.size(); i++){
			Plane plane = pCur.mvPlanes[i];
			double dist = plane.getDistance();
			Eigen::Vector3d norm = plane.getNormal();
			Eigen::Vector3d cent = plane.getCentroid();
			
			double planeCost = 0.0;
			
			for(auto point : plane.mPlanePoints->points){
				pcl::PointXYZRGB colored_point;
				Eigen::Vector3d p_eig;
				p_eig << point.x, point.y , point.z;
				planeCost += abs( norm.dot(p_eig) + dist );
				{
					colored_point.x = point.x;
					colored_point.y = point.y;
					colored_point.z = point.z;
					colored_point.r = r;
					colored_point.g = g;
					colored_point.b = b;
					plane_cloud->push_back(colored_point);
				}
			}
			// every plane is set a color
			r = (r + 30)%255;
			g = (g + 30)%255;
			b = (b + 30)%255;
			PointT start(cent[0], cent[1], cent[2]);
			PointT end(5*norm[0] + cent[0], 5*norm[1] + cent[1], 5*norm[2] + cent[2]);

			string s = "plane_normal : " + to_string(i);
			viewer.addArrow(end, start, 1.0, 0, 0, false, s);

			cout << i << "th plane has " <<  plane.mPlanePoints->points.size() << " points |||";
			cout << "normal = " << norm.transpose() << ", centroid = " << cent.transpose() << 
				", distance = " << dist << " total cost = " << planeCost <<endl;
		}
		
		viewer.addPointCloud(plane_cloud, "plane_cloud");
		viewer.spin();	
		#endif
	}
		
	bool Track::InitialGuess(){
		
		pRef.setRotation(Eigen::Matrix3d::Identity());
		pRef.setTranslation(Eigen::Vector3d::Zero());
		Eigen::Matrix3d R = pRef.getRotation();
		Eigen::Vector3d t = pRef.getTranslation();
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
		T.rotate(R); 
		T.pretranslate(t);
		
		double matchingThres = mParamPtr->mMatchingThres;// threshold to prevent false matching
		double costThres = mParamPtr->mMatchingCostThres;// threshold to judge whether converge

		int m = pRef.mvPlanes.size(), n = pCur.mvPlanes.size();// num of planes of ref and cur
		double cost = 0.0;// total corresponding cost for one iteration
		uint32_t iter = 0;

		// brute-force matching, each plane i in ref to j in cur with minimum cost
		// two reasons for the discarded: step1,2 or too large cost
		while( iter <= 1.5*mParamPtr->mInitGuessIters ){ // todo set maximum iteration times
			
			pRef.mvMatchedPlanes.resize(m, -1);
			int c = 0;// num of correspondences
			cost = 0.0;

			for(int i = 0; i < m; i++){
				double min_error = DBL_MAX, cur_error = 0.0;
				int min_id = -1;
				for(int j = 0; j < n; j++){
					Plane planeCur = pCur.mvPlanes[j], planeRef = pRef.mvPlanes[i];

					// step1 numbers of plane points restriction
					int numOfCur = planeCur.mPlanePoints->points.size(),
						numOfRef = planeRef.mPlanePoints->points.size();
					int diff = abs(numOfRef - numOfCur), sum = numOfRef + numOfCur;
					if( static_cast<double>(diff)/static_cast<double>(sum) > 0.15 )// varies too largely
						continue;

					// step2 angular check
					Eigen::Vector3d normalRef = planeRef.getNormal(), 
									normalCur = planeCur.getNormal();
					Eigen::Vector3d	normalTrans = R*normalCur;
					double ang_error = abs( acos( normalTrans.dot(normalRef) ) );
					if( ang_error*180/CV_PI > 15.0 )
						continue;
					
					// step3 computing errors of distance and centroid
					double dist_error = abs( t.dot(normalTrans) );    // distance error
					Eigen::Vector3d centCurTrans = T*planeCur.getCentroid(); 
					Eigen::Vector3d centRef = planeRef.getCentroid();
					double dx = centRef[0] - centCurTrans[0],
						   dy = centRef[1] - centCurTrans[1],
						   dz = centRef[2] - centCurTrans[2];
					double cent_error = sqrt( dx*dx + dy*dy + dz*dz );// centroid error

					// 2.4 computing errors
					cur_error = dist_error + cent_error;
					if( cur_error < min_error ){
						min_error = cur_error;
						min_id = j;
					}

				}// -> best match of plane i in ref frame, result with no correct correspondence is set -1

				if( min_error >= costThres ){// no correct match
					pRef.mvMatchedPlanes[i] = -1;
				}
				if( min_id >= 0 ){// match successfully
					cost += min_error;
					c++;
					pRef.mvMatchedPlanes[i] = min_id;
				}

			}// end of iter th matching
			
			// estimating pose
			// for rotation
			Eigen::Matrix<double, 3, Eigen::Dynamic> matNormRef(3, c);
			Eigen::Matrix<double, 3, Eigen::Dynamic> matNormCur(3, c);
			matNormRef.setZero();
			matNormCur.setZero();
			// for translation
			Eigen::Matrix<double, Eigen::Dynamic, 3> N(c, 3);
			Eigen::Matrix<double, Eigen::Dynamic, 1> b(c, 1);
			int cnt = 0;
			for(int i = 0; i < m; i++){
				int matchedId = pRef.mvMatchedPlanes[i];
				if(matchedId != -1){// num of matchedId inequals to -1 = c
					Eigen::Vector3d normalRef = pRef.mvPlanes[i].getNormal(), 
					      			normalCur = pCur.mvPlanes[matchedId].getNormal();

					//for rotation
					matNormRef.col(cnt) = normalRef;
					matNormCur.col(cnt) = normalCur;
					//for translation
					Eigen::Vector3d centCur = pCur.mvPlanes[matchedId].getCentroid();
					double dRef = pRef.mvPlanes[i].getDistance(), 
						   dCur = pCur.mvPlanes[matchedId].getDistance();
					N.row(cnt) = normalRef.transpose();
					b(cnt) = -(normalRef.dot(R*centCur)) - dRef;//b(cnt) = dCur - dRef;
					cnt++;
				}
			}
			Eigen::Matrix3d H = matNormCur*(matNormRef.transpose());//3*3
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
			double detVUT = (V*(U.transpose())).determinant();
			Eigen::Matrix3d temp = Eigen::Matrix3d::Identity();
			temp(2, 2) = detVUT;

			R = V*temp*(U.transpose());
			t = (N.transpose()*N).inverse()*(N.transpose())*b;

			if(iter > 0 && cost/c < costThres){
				cout << "converge at " << iter << " times, " << " cost = " << cost/c << endl;
				cout << "R = " << R << " and t = " << t.transpose() << endl;
				break;
			}

			iter++;
		}

		// (optional) show matching result
		int mm = pRef.mvMatchedPlanes.size();
		if(mParamPtr->mbShowMatResult){
			for(int i = 0; i < mm; i++){
				int j = pRef.mvMatchedPlanes[i];
				if( j == -1 )
					continue;
				Plane plane1 = pRef.mvPlanes[i];
				Plane plane2 = pCur.mvPlanes[j];

				cout<< "ref = " << i << " cur = " << j << endl;
				cout << "ref num = " << plane1.mPlanePoints->points.size() << " || " 
					 << "cur num = " << plane2.mPlanePoints->points.size() << endl;
				cout << "ref normal = " << plane1.getNormal().transpose() << " || "
					 << "cur normal = " << plane2.getNormal().transpose() << endl;
				cout << "ref centroid = " << plane1.getCentroid().transpose() << " || "
					 << "cur centroid = " << plane2.getCentroid().transpose() << endl;
				cout << "ref distance = " << plane1.getDistance() << " || "
					 << "cur distance = " << plane2.getDistance() << endl;
			}
		}
			
		if( iter < mParamPtr->mInitGuessIters ){
			cout << "exit within finite iterations at " << iter << " times" << endl;
			pRef.setRotation(R);
			pRef.setTranslation(t);
			return true;
		}
		else{
			cout << "exceeding maximum iterations" << endl;
			// mR_rel and mt_rel remain unchanged
			return false;
		}
	}

	bool Track::PoseRefinement(){
		Eigen::Matrix3d R_rel = pRef.getRotation();
		Eigen::Vector3d t_rel = pRef.getTranslation();
		Sophus::SE3d T_rel(R_rel, t_rel);

		bool STATE = false;
		double cost = 0.0; // cost for current iteration
		double lastCost = 0.0;// cost for last iteration
		int m = pRef.mvMatchedPlanes.size();
		
		// Gauss-Newton, 
		for(int it = 0; it < mParamPtr->mPoseEstiIters; it++){
			Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
			Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
			cost = 0.0;

			for(int i = 0; i < m; i++){
				if(pRef.mvMatchedPlanes[i] < 0)
					continue;
				// temp variables
				Plane planeRef = pRef.mvPlanes[i], planeCur = pCur.mvPlanes[pRef.mvMatchedPlanes[i]];
				double distRef = planeRef.getDistance(), distCur = planeCur.getDistance();
				Eigen::Vector3d normalRef = planeRef.getNormal();
				Eigen::Vector3d centCur = planeCur.getCentroid();
				Eigen::Vector3d centRefTrans = T_rel*centCur;
				
				// todo error function
				double error = normalRef.dot(R_rel*centCur + t_rel) + distRef;
				cost += abs(error);
				// todo compute inverse variance of normal
				double w = 1.0;
				// Jacobian
				Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
				Eigen::Vector3d error_2_pRef = normalRef;
				Eigen::Matrix<double, 3, 6> pRef_2_pose = Eigen::Matrix<double, 3, 6>::Zero();
				pRef_2_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
				pRef_2_pose.block<3, 3>(0, 3) = -Utils::skewSymmetricMatrix(centRefTrans);
				J = error_2_pRef.transpose()*pRef_2_pose;
				//cout << "J = " << J <<endl;
				// Hessian
				H += w*( J.transpose()*J );
				// b
				b += - w*( J.transpose()*error );
			}
			Eigen::Matrix<double, 6, 1> dx;
			dx = H.ldlt().solve(b);
			if(isnan(dx[0])){
				cout<< "result is nan." << endl;
				break;
			}
			/*
			if(it > 0 && cost > lastCost){
				cout<< "update is not good" <<endl;
				break;
			}
			*/
			T_rel = Sophus::SE3d::exp(dx)*T_rel;
			R_rel = T_rel.rotationMatrix();
			t_rel = T_rel.translation();
			lastCost = cost;

			if(1){
				cout << it <<" th: cost = " << cost << " dx.norm = " << dx.norm() << endl;
			}

			if(dx.norm() < 1e-3){// when converge, saving results
				STATE = true;
				pRef.setRotation(T_rel.rotationMatrix());
				pRef.setTranslation(T_rel.translation());
				break;
			}	
		}
		return STATE;
	}

	void Track::reset(int state){
		if(state == ResetRef || state == ResetBoth){
			pRef.mvMatchedPlanes.clear();
			pRef.setRotation(Eigen::Matrix3d::Identity());
			pRef.setTranslation(Eigen::Vector3d::Zero());
			pRef.mvPlanes.clear();
		}
		if(state == ResetCur || state == ResetBoth){
			pCur.mvMatchedPlanes.clear();
			pCur.setRotation(Eigen::Matrix3d::Identity());
			pCur.setTranslation(Eigen::Vector3d::Zero());
			pCur.mvPlanes.clear();
		}
	}

	void Track::processCloud(PointCloudT::Ptr &cloud_in){
		std::ofstream fout(mPoseFile, std::ios::app);
		if(!fout.is_open())
			cout << "error while opening file" << endl;
		fout.setf(std::ios::fixed, std::ios::floatfield);
		fout.precision(5);

		int state = -1;
		pCur.mInitialCloud = cloud_in;

		preProcess(cloud_in);
		ExtractPlane(cloud_in);

		if(!mbInitialized){// the first frame
			mbInitialized = true;
			pRef = pCur;
			int state = ResetCur;
			reset(state);
			return;
		}

		if(InitialGuess()){
			cout << "InitialGuess Success!" << endl;

			if(PoseRefinement()){
				Eigen::Matrix3d R = pRef.getRotation();
				Eigen::Vector3d t = pRef.getTranslation();

				cout << "optimized pose: R = " << R << " t = " << t << endl;

				// saving result
				Eigen::Quaterniond quat(R);

				fout<< t[0] << " "
					<< t[1] << " "
					<< t[2] << " "
					<< quat.coeffs()[0] << " "
					<< quat.coeffs()[1] << " "
					<< quat.coeffs()[2] << " "
					<< quat.coeffs()[3] << std::endl;
				if(mParamPtr->mbShowAliResult){
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
					// pointcloud of Ref
					for( auto &point : pRef.mInitialCloud->points ){ 
						pcl::PointXYZRGB point_colored;
						point_colored.x = point.x;
						point_colored.y = point.y;
						point_colored.z = point.z;
						point_colored.r = 255;
						point_colored.g = 0;
						point_colored.b = 0;
						matched_cloud->push_back(point_colored);
					}
					// pointcloud of Cur
					for( auto &point : pCur.mInitialCloud->points ){ 
						Eigen::Vector3d pOri = Eigen::Vector3d::Zero();
						pOri << point.x, point.y, point.z;
						Eigen::Vector3d pTrans = Eigen::Vector3d::Zero();
						pTrans = R*pOri + t;
						pcl::PointXYZRGB point_colored;
						point_colored.x = pTrans[0];
						point_colored.y = pTrans[1];
						point_colored.z = pTrans[2];
						point_colored.r = 0;
						point_colored.g = 200;
						point_colored.b = 200;
						matched_cloud->push_back(point_colored);
					}

					pcl::visualization::CloudViewer viewer ("Cluster viewer");
					while (!viewer.wasStopped ())
					{
						viewer.showCloud(matched_cloud);
					}
				}
				state = ResetRef;
				reset(state);
				pRef = pCur;
				state = ResetCur;
				reset(state);
			}
			else{
				cout << "PoseRefinement Failed! Reset..." << endl;
				fout << "PoseRefinement Failed" << endl;
				state = ResetRef;
				reset(state);
				pRef = pCur;
				state = ResetCur;
				reset(state);
			}
		}
		else{
			cout << "InitialGuess Failed! Reset..." << endl;
			fout << "InitialGuess Failed" << endl;
			state = ResetRef;
			reset(state);
			pRef = pCur;
			state = ResetCur;
			reset(state);
		}
		
		fout.close();
	}
	
}// namespace HARBOR