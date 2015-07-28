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
        int pointCloudKeepOneOutOf):
    outputPath(outputPath),
    icpConfigPath(icpConfigPath),
    totalPointCloudIndex(0),
    pointCloudIndex(0),
    numSuffixWidth(4),
    isNextOdomEqualToLast(false),
    isOdomOutput(isOdomOutput),
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

void DatasetGenerator::setNextOdomEqualToLast() {
    this->isNextOdomEqualToLast = true;
}

void DatasetGenerator::computeCloudOdometry(
        shared_ptr<PM::DataPoints> currentCloud) {
    if(this->pointCloudIndex != 0) {
        Eigen::Matrix4f initTransfo = Eigen::Matrix4f::Identity();

        if(this->isNextOdomEqualToLast) {
            this->isNextOdomEqualToLast = false;
        } else {
            tf::Pose startPose(this->lastCloudPose);
            tf::Pose endPose(this->lastMsgPose);
            tf::Transform poseDiff = startPose.inverseTimes(endPose);

            initTransfo = Conversion::tfToEigen(poseDiff);
        }

        Eigen::Matrix4f icpOdom = IcpOdometry::getCorrectedTransfo(
        *this->lastPointCloud, *currentCloud,
        initTransfo, this->icpConfigPath);

        this->lastCorrectedPose = Conversion::getPoseComposition(
                this->lastCorrectedPose, icpOdom);
    }

    this->lastCloudPose = this->lastMsgPose;
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
}

std::string DatasetGenerator::generateCloudFilename() {
    std::string filename = "scan_";
    filename += this->getPaddedNum(pointCloudIndex, this->numSuffixWidth);
    filename += ".pcd";

    return this->outputPath + filename;
}

std::string DatasetGenerator::getPaddedNum(const int &numSuffix,
        const int width) {
    std::ostringstream output;
    output << std::setfill('0') << std::setw(width) << numSuffix;

    return output.str();
}
