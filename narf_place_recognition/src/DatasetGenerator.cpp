#include "DatasetGenerator.hpp"
#include "IcpOdometry.hpp"
#include "Conversion.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#include <pointmatcher_ros/point_cloud.h>

using PointMatcher_ros::rosMsgToPointMatcherCloud;
using boost::shared_ptr;

DatasetGenerator::DatasetGenerator(
        const std::string outputPath,
        const std::string icpConfigPath,
        const bool isOdomOutput,
        int pointCloudKeepOneOutOf,
        bool isOdomMergedCloudsSaved):
    outputPath(outputPath),
    icpConfigPath(icpConfigPath),
    totalPointCloudIndex(0),
    pointCloudIndex(0),
    numSuffixWidth(4),
    isNextOdomEqualToFirst(false),
    isNextOdomEqualToLast(false),
    isFirstLoop(true),
    isOdomOutput(isOdomOutput),
    isOdomMergedCloudsSaved(isOdomMergedCloudsSaved),
    lastCorrectedPose(Eigen::Matrix4f::Identity()),
    pointCloudKeepOneOutOf(pointCloudKeepOneOutOf) {
    }

void DatasetGenerator::manageOdometryMsg(rosbag::MessageInstance const &msg) {
    shared_ptr<geometry_msgs::PoseWithCovarianceStamped> odomMsg =
        msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();

    if(odomMsg != NULL) {
        tf::poseMsgToTF(odomMsg->pose.pose, this->lastMsgPose);
    }
}

void DatasetGenerator::managePointCloudMsg(rosbag::MessageInstance const &msg) {
    shared_ptr<sensor_msgs::PointCloud2> cloudMsg =
        msg.instantiate<sensor_msgs::PointCloud2>();

    if(cloudMsg != NULL) {
        if(this->totalPointCloudIndex % this->pointCloudKeepOneOutOf == 0) {
            ROS_INFO_STREAM("Processing cloud " << this->pointCloudIndex);

            shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                        rosMsgToPointMatcherCloud<float>(*cloudMsg)));

            try{
                cloud->save(generateCloudFilename());
            } catch(...) {
                ROS_ERROR("Unable to save in the directory provided.");
            }

            if(this->isOdomOutput) {
                this->computeCloudOdometry(cloud);
                this->saveOdom();
            }

            this->pointCloudIndex++;
            this->lastPointCloud = cloud;
        }
        this->totalPointCloudIndex++;
    }
}

void DatasetGenerator::setNextOdomEqualToFirst() {
    this->isNextOdomEqualToLast = true;
    this->isFirstLoop = false;
}

void DatasetGenerator::setNextOdomEqualToLast() {
    this->isNextOdomEqualToLast = true;
    this->isFirstLoop = false;
}

void DatasetGenerator::computeCloudOdometry(
        shared_ptr<PM::DataPoints> currentCloud) {
    if(this->pointCloudIndex != 0) {
        Eigen::Matrix4f initTransfo = Eigen::Matrix4f::Identity();

        if(this->isNextOdomEqualToFirst) {
            this->setFirstLoopBestMatch();
            //2.0 = 1.0 + ICP
            //2.1 = 2.0 + odom + odom diff closest in 1 + ICP
            //Require: save all loop1 scans odom, load PC from file
        } else if(this->isNextOdomEqualToLast) {
            this->isNextOdomEqualToLast = false;
        } else {
            tf::Pose startPose(this->lastCloudPose);
            tf::Pose endPose(this->lastMsgPose);
            tf::Transform poseDiff = startPose.inverseTimes(endPose);

            initTransfo = Conversion::tfToEigen(poseDiff);
        }

        std::string filename = "";
        if(this->isOdomMergedCloudsSaved) {
            filename += this->outputPath + "scan_merged_";
            filename += this->getPaddedNum(this->pointCloudIndex,
                    this->numSuffixWidth);
        }
        Eigen::Matrix4f icpOdom = IcpOdometry::getCorrectedTransfo(
                *this->lastPointCloud, *currentCloud,
                initTransfo, this->icpConfigPath, filename,
                this->isOdomMergedCloudsSaved);

        this->lastCorrectedPose = Conversion::getPoseComposition(
                this->lastCorrectedPose, icpOdom);
    }

    this->lastCloudPose = this->lastMsgPose;
}

// [TODO]: Working on this method - 2015-08-03 04:48pm
void DatasetGenerator::setFirstLoopBestMatch() {
    int bestIndex = 0;
    int bestDistance = std::numeric_limits<int>::infinity();
    for(int i = 0; i < this->firstLoopPoses.size() ; ++i) {
        tf::Pose currentPose = this->lastCloudPose;
        int distance = 0;
        if(distance < bestDistance) {
           bestDistance = distance;
        }
    }
    boost::shared_ptr<PM::DataPoints> closestPointCloud;
    closestPointCloud->load("");
    this->lastPointCloud = closestPointCloud;
}

void DatasetGenerator::saveOdom() {
    std::string filename = this->outputPath + "scan_";
    filename += this->getPaddedNum(this->pointCloudIndex, this->numSuffixWidth);
    filename += "_info.dat";

    Eigen::Translation3f translation = Conversion::getTranslation(
            this->lastCorrectedPose);
    Eigen::Vector3f rollPitchYaw = Conversion::getRPY(
            this->lastCorrectedPose);

    ROS_INFO_STREAM("Odometry (x,y,z,r,p,y): "
            << translation.x() << ", "
            << translation.y()  << ", "
            << translation.z() << ", "
            << rollPitchYaw(0) << ", "
            << rollPitchYaw(1) << ", "
            << rollPitchYaw(2) << " = "
            << translation.vector().norm() << " m");

    std::ofstream file;
    file.open(filename.c_str());
    file << "Odometry: "
        << translation.x() << ", "
        << translation.y()  << ", "
        << translation.z() << ", "
        << rollPitchYaw(0) << ", "
        << rollPitchYaw(1) << ", "
        << rollPitchYaw(2) << std::endl;

    file.close();

    if(this->isFirstLoop) {
        tf::Pose pose;
        pose.setOrigin(Conversion::eigenToTf(translation));
        pose.setRotation(Conversion::eigenToTf(
                    Conversion::getQuat(this->lastCorrectedPose)));
        this->firstLoopPoses.push_back(pose);
    }
}

std::string DatasetGenerator::generateCloudFilename() {
    std::string filename = "scan_";
    filename += this->getPaddedNum(this->pointCloudIndex, this->numSuffixWidth);
    filename += ".pcd";

    return this->outputPath + filename;
}

std::string DatasetGenerator::getPaddedNum(const int &numSuffix,
        const int width) {
    std::ostringstream output;
    output << std::setfill('0') << std::setw(width) << numSuffix;

    return output.str();
}
