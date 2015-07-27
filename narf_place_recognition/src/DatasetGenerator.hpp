#ifndef DATASETGENERATOR_H
#define DATASETGENERATOR_H

#include "Definitions.hpp"

#include <rosbag/bag.h>
#include <ros/console.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <pointmatcher/PointMatcher.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

class DatasetGenerator {
    private:
        std::string outputPath;
        std::string icpConfigPath;
        int totalPointCloudIndex;
        int pointCloudIndex;
        const int numSuffixWidth;
        bool isNextOdomEqualToLast;
        bool isOdomOutput;
        int pointCloudKeepOneOutOf;

        tf::Pose lastMsgPose;
        tf::Pose lastCloudPose;
        Eigen::Matrix4f lastCorrectedPose;
        boost::shared_ptr<PM::DataPoints> lastPointCloud;

    public:
        DatasetGenerator(
                const std::string outputPath,
                const std::string icpConfigPath,
                bool isOdomOutput = true,
                int pointCloudKeepOneOutOf = 1.0);
        void manageOdometryMsg(rosbag::MessageInstance const &msg);
        void managePointCloudMsg(rosbag::MessageInstance const &msg);
        void setNextOdomEqualToLast();

    private:
        void computeCloudOdometry(
                boost::shared_ptr<PM::DataPoints> currentCloud);
        void saveOdom();
        std::string generateCloudFilename();
        std::string getPaddedNum(const int &numSuffix, const int width);
};

#endif
