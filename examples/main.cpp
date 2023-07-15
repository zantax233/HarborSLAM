#include "include/Track.h"
using namespace std;

int main(int argc, char** argv){
	cv::String src_path = "/home/wang/sh/extraction/test_data";
	string configPath = argv[1];
	string poseFile = argv[2];
	vector<cv::String> filenames;
	cv::glob(src_path, filenames);

	HARBOR::Track track(configPath, poseFile);
    for(size_t i = 0; i < filenames.size(); i++){
		string cloud_name(filenames[i].c_str());
		cout << cloud_name << endl;
		PointCloudT::Ptr pointCloudIn(new PointCloudT());
    	pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name, *pointCloudIn);

		track.processCloud(pointCloudIn);
        
	}
	return 0;
}