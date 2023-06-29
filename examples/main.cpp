#include "common.h"
using namespace std;

int main(int argc, char** argv){
	cv::String src_path = "/home/wang/sh/extraction/test_data";
	vector<cv::String> filenames;
	cv::glob(src_path, filenames);

    for(size_t i = 0; i < filenames.size(); i++){
		string cloud_name(filenames[i].c_str());
		cout << cloud_name << endl;
		PointCloudT::Ptr pointCloudIn(new PointCloudT());
    	pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name, *pointCloudIn);

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCLVis"));
        viewer->addPointCloud<PointT>(pointCloudIn,"source cloud");
		viewer->spin();
	}
	return 0;
}