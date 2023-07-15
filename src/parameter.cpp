#include "include/parameter.h"
using namespace std;

namespace HARBOR{
    Parameter::Parameter(const std::string _configFile){
        mFilePath = _configFile;
        ReadParams();
    }

    void Parameter::ReadParams(){

        YAML::Node node = YAML::LoadFile(mFilePath);
        mbVisualizePlanes = node["VIS_PLANE"].as<bool>();
        mbShowMatResult = node["SHOW_MATCHING_RESULT"].as<bool>();
        mbShowAliResult = node["SHOW_ALIGNING_RESULT"].as<bool>();
        
        mMatchingThres = node["planeMatchingThreshold"].as<double>();
        mMatchingCostThres = node["matchingCostThreshold"].as<double>();

        mInitGuessIters = node["initialGuessIters"].as<int>();
        mPoseEstiIters = node["poseRefinementIters"].as<int>();
    }
}