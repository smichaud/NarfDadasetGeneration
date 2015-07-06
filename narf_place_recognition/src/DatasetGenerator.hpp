#ifndef DATASETGENERATOR_H
#define DATASETGENERATOR_H

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "pointmatcher/PointMatcher.h"
#include "boost/shared_ptr.hpp"
#include <Eigen/Dense>

typedef PointMatcher<float> PM;

using boost::shared_ptr;
using std::string;
using std::endl;

class DatasetGenerator {
    private: //I should avoid ros dependency here...
        string outputPath;
        int pointCloudIndex;
        const int numSuffixWidth;
        float minDistBetweenPointClouds;
        geometry_msgs::Pose lastMsgPose;
        geometry_msgs::Pose lastCloudPose;
        shared_ptr<PM::DataPoints> lastPointCloud;
        geometry_msgs::Pose lastRealPose;

    public:
        DatasetGenerator(const string outputPath);
        void manageOdometryMsg(rosbag::MessageInstance const &msg);
        void managePointCloudMsg(rosbag::MessageInstance const &msg);

    private:
        float getDistFromLastPosition();
        void computeCloudOdometry(shared_ptr<PM::DataPoints> currentCloud);
        void saveOdom();
        string generateCloudFilename();
        string getPaddedNum(const int &numSuffix, const int width);
        Eigen::Vector3f getRollPitchYaw(
                geometry_msgs::Quaternion quaternionMsg);
};

#endif
