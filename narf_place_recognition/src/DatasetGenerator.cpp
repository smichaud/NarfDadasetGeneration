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

DatasetGenerator::DatasetGenerator(const std::string outputPath, 
        const std::string icpConfigPath):
    outputPath(outputPath),
    icpConfigPath(icpConfigPath),
    pointCloudIndex(0),
    numSuffixWidth(4),
    lastCorrectedPose(tf::Pose::getIdentity()) {
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
        shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                    rosMsgToPointMatcherCloud<float>(*cloudMsg)));

        cloud->save(generateCloudFilename());
        this->computeCloudOdometry(cloud);

        this->pointCloudIndex++;
        this->lastPointCloud = cloud;
    }
}

void DatasetGenerator::computeCloudOdometry(
        shared_ptr<PM::DataPoints> currentCloud) {
    if(this->pointCloudIndex != 0) {
        tf::Pose startPose(this->lastCloudPose);
        tf::Pose endPose(this->lastMsgPose);
        tf::Transform poseDiff = startPose.inverseTimes(endPose);

        Eigen::Matrix4f initTransfo = Conversion::tfToEigen(poseDiff);
        IcpOdometry::getCorrectedTransfo(*this->lastPointCloud, *currentCloud,
                initTransfo, this->icpConfigPath);
    }
    this->lastCloudPose = this->lastMsgPose;
    
    //saveOdom();
}

void DatasetGenerator::saveOdom() {
    std::string filename = this->outputPath + "scan_";
    filename += this->getPaddedNum(this->pointCloudIndex, this->numSuffixWidth);
    filename += "_info.dat";

    //Eigen::Vector3f rollPitchYaw = getRollPitchYaw(
    //this->lastCorrectedPose.orientation);

    std::ofstream file;
    file.open(filename.c_str());
    //file << this->lastCorrectedPose.position.x << ", "
    //<< this->lastCorrectedPose.position.y  << ", "
    //<< this->lastCorrectedPose.position.z << ", "
    //<< rollPitchYaw(0) << ", "
    //<< rollPitchYaw(1) << ", "
    //<< rollPitchYaw(2) << std::endl;

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
