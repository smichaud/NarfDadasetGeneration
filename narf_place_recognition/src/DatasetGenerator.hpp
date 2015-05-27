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

class DatasetGenerator {
    private: //I should avoid ros dependency here...
        std::string outputPath;
        int pointCloudIndex;
        geometry_msgs::Pose lastMsgPose;
        geometry_msgs::Pose lastCloudPose;
        shared_ptr<PM::DataPoints> lastPointCloud;
        geometry_msgs::Pose lastRealPose;

    public:
        DatasetGenerator();
        void managePointCloudMsg(rosbag::MessageInstance const &msg);
        void manageOdometryMsg(rosbag::MessageInstance const &msg);

    private:
        void computeCloudOdometry(shared_ptr<PM::DataPoints> currentCloud);
        void saveOdom();
        std::string getCloudFilename();
        std::string appendNum(const std::string &input, const int &numSuffix);
        Eigen::Vector3f getRollPitchYaw(
                geometry_msgs::Quaternion quaternionMsg);
};

#endif
