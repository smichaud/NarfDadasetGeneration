// [TODO]: Do ICP for next (with odom?) - 2015-05-21 12:36pm
// [TODO]: Try to get the far range pointcloud - 2015-05-21 12:37pm
// If your scan_001_far_ranges.pcd.
// [TODO]: scan_001_info.dat - 2015-05-21 12:38pm
#include "DatasetGenerator.hpp"

#include "pointmatcher_ros/point_cloud.h"

#include <iostream>
#include <iomanip>
#include <fstream>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

using PointMatcher_ros::rosMsgToPointMatcherCloud;

DatasetGenerator::DatasetGenerator(const std::string outputPath):
    outputPath(outputPath),
    pointCloudIndex(0),
    numSuffixWidth(4),
    minDistBetweenPointClouds(4.0) {
        lastRealPose.orientation.w = 1;
}

void DatasetGenerator::manageOdometryMsg(rosbag::MessageInstance const &msg) {
    shared_ptr<geometry_msgs::PoseWithCovarianceStamped> odomMsg =
        msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();

    if(odomMsg != NULL) {
        this->lastMsgPose = odomMsg->pose.pose;
    }
}

void DatasetGenerator::managePointCloudMsg(rosbag::MessageInstance const &msg) {
    shared_ptr<sensor_msgs::PointCloud2> cloudMsg =
        msg.instantiate<sensor_msgs::PointCloud2>();

    if(cloudMsg != NULL
            && getDistFromLastPosition() > this->minDistBetweenPointClouds) {
        shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                    rosMsgToPointMatcherCloud<float>(*cloudMsg)));

        // [TODO]: Remove tmp output when done - 2015-05-25 08:18pm
        //std::cout << "Got pointcloud " << this->pointCloudIndex << std::endl;
        //std::cout << "Pose:" << std::endl;
        //std::cout << this->lastMsgPose.position << std::endl;
        //std::cout << this->lastMsgPose.orientation << std::endl;

        cloud->save(getCloudFilename());
        this->computeCloudOdometry(cloud);
        this->pointCloudIndex++;
        this->lastPointCloud = cloud;
    }
}

float DatasetGenerator::getDistFromLastPosition() {
}

void DatasetGenerator::computeCloudOdometry(
        shared_ptr<PM::DataPoints> currentCloud) {
    if(this->pointCloudIndex != 0) {
        // [TODO]: Compute/write odom - 2015-05-25 03:11pm
        // [TODO]: Convert quaternion to rpy - 2015-05-25 08:17pm
        // lastMsgPose - lastCloudPose = icpApprox
        // lastRealCloudPose + (icpApprox + icpResult) = newRealCloudPose
    }
    saveOdom();
}

void DatasetGenerator::saveOdom() {
    std::string filename = this->outputPath + "scan_";
    filename += this->getPaddedNum(this->pointCloudIndex, this->numSuffixWidth);
    filename += "_info.dat";

    Eigen::Vector3f rollPitchYaw = getRollPitchYaw(
            this->lastRealPose.orientation);

    std::ofstream file;
    file.open(filename.c_str());
    file << this->lastRealPose.position.x << ", "
        << this->lastRealPose.position.y  << ", "
        << this->lastRealPose.position.z << ", "
        << rollPitchYaw(0) << ", "
        << rollPitchYaw(1) << ", "
        << rollPitchYaw(2) << std::endl;

    file.close();
}

std::string DatasetGenerator::getCloudFilename() {
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

Eigen::Vector3f DatasetGenerator::getRollPitchYaw(
        geometry_msgs::Quaternion quaternionMsg) {
    // RPY convention order seems to be ZYX
    tf::Quaternion quaternionTf;

    tf::quaternionMsgToTF(quaternionMsg, quaternionTf);

    double roll, pitch, yaw;
    tf::Matrix3x3(quaternionTf).getRPY(roll, pitch, yaw);

    return Eigen::Vector3f(roll, pitch, yaw);
}
