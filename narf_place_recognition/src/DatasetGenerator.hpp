#ifndef DATASETGENERATOR_H
#define DATASETGENERATOR_H

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "pointmatcher/PointMatcher.h"
#include "boost/shared_ptr.hpp"

typedef PointMatcher<float> PM;
using boost::shared_ptr;

class DatasetGenerator {
    private:
        std::string outputPath;
        int pointCloudIndex;
        geometry_msgs::Pose lastPose;
        geometry_msgs::Pose lastCloudPose;
        shared_ptr<PM::DataPoints> lastPointCloud;

    public:
        DatasetGenerator();
        void managePointCloudMsg(rosbag::MessageInstance const &msg); 
        void manageOdometryMsg(rosbag::MessageInstance const &msg); 

    private:
        void computeCloudOdometry(shared_ptr<PM::DataPoints> currentCloud);
        std::string getCloudFilename();
        std::string appendNum(const std::string &input, const int &numSuffix);
};

#endif
