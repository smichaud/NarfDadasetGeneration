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
        int totalCloudIndex;
        int cloudIndex;
        int currentLoopCloudIndex;
        const int numSuffixWidth;
        bool isNextOdomEqualToFirst;
        bool isNextOdomEqualToLast;
        bool isFirstLoop;
        bool isOdomOutput;
        bool isOdomMergedCloudsSaved;
        int pointCloudKeepOneOutOf;

        tf::Pose lastMsgPose;
        tf::Pose lastCloudPose;
        std::vector<tf::Pose> firstLoopPoses;
        Eigen::Matrix4f lastCorrectedPose;
        boost::shared_ptr<PM::DataPoints> lastPointCloud;

    public:
        DatasetGenerator(
                const std::string outputPath,
                const std::string icpConfigPath,
                bool isOdomOutput = true,
                int pointCloudKeepOneOutOf = 1.0,
                bool isOdomMergedCloudsSaved = false);
        void manageOdometryMsg(rosbag::MessageInstance const &msg);
        void managePointCloudMsg(rosbag::MessageInstance const &msg);
        void setNextOdomEqualToFirst();
        void setNextOdomEqualToLast();

    private:
        void computeCloudOdometry(
                boost::shared_ptr<PM::DataPoints> currentCloud);
        Eigen::Matrix4f setFirstLoopBestMatch();
        tf::Pose getPoseDiffFromLastCloud();
        bool userOdomAdjustment(Eigen::Matrix4f& initTransfo,
                const std::string& filename);
        void saveOdom();
        std::string generateCloudFilename(int cloudIndex);
        std::string getPaddedNum(const int &numSuffix, const int width);
};

#endif
