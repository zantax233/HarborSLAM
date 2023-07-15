#ifndef PARAMETER_H
#define PARAMETER_H

#include "include/common.h"

namespace HARBOR{

    class Parameter{
    public:
        Parameter(const std::string _configFile);
        void ReadParams();

    public:
        std::string mFilePath;
        bool mbVisualizePlanes;
        bool mbShowMatResult;
        bool mbShowAliResult;

        double mMatchingThres;
        double mMatchingCostThres;

        int mInitGuessIters;
        int mPoseEstiIters;

    };// class Parameter

}// namespace HARBOR

#endif