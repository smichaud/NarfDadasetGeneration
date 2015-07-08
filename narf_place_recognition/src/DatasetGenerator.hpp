#ifndef DATASETGENERATOR_H
#define DATASETGENERATOR_H

#include "Definitions.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <pointmatcher/PointMatcher.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

class DatasetGenerator {
    private:
        std::string outputPath;
        int pointCloudIndex;
        const int numSuffixWidth;

        tf::Pose lastMsgPose;
        tf::Pose lastCloudPose;
        tf::Pose lastCorrectedPose;
        boost::shared_ptr<PM::DataPoints> lastPointCloud;

    public:
        DatasetGenerator(const std::string outputPath);
        void manageOdometryMsg(rosbag::MessageInstance const &msg);
        void managePointCloudMsg(rosbag::MessageInstance const &msg);

    private:
        float getDistFromLastPosition();
        void computeCloudOdometry(
                boost::shared_ptr<PM::DataPoints> currentCloud);
        void saveOdom();
        std::string generateCloudFilename();
        std::string getPaddedNum(const int &numSuffix, const int width);
        Eigen::Vector3f getRollPitchYaw(
                geometry_msgs::Quaternion quaternionMsg);
};

#endif
