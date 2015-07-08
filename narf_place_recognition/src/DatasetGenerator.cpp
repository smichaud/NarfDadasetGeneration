#include "DatasetGenerator.hpp"
#include "IcpOdometry.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>

#include <pointmatcher_ros/point_cloud.h>

using PointMatcher_ros::rosMsgToPointMatcherCloud;
using boost::shared_ptr;

DatasetGenerator::DatasetGenerator(const std::string outputPath):
    outputPath(outputPath),
    pointCloudIndex(0),
    numSuffixWidth(4),
    minDistBetweenPointClouds(2.5),
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

    if(cloudMsg != NULL
            && getDistFromLastPosition() > this->minDistBetweenPointClouds) {
        shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                    rosMsgToPointMatcherCloud<float>(*cloudMsg)));

        cloud->save(generateCloudFilename());
        this->computeCloudOdometry(cloud);
        this->pointCloudIndex++;
        this->lastPointCloud = cloud;
    }
}

// [TODO]: Do I really need it since I use SICK... - 2015-07-07 07:28pm
float DatasetGenerator::getDistFromLastPosition() {
    return 50.0; // Added for the Velodyne (so I don't take every cloud)
}

// [TODO]: Try to get the far range pointcloud - 2015-05-21 12:37pm
// If your scan_001_far_ranges.pcd.
// [TODO]: scan_001_info.dat - 2015-05-21 12:38pm
// [TODO]: Compute/write odom - 2015-05-25 03:11pm
// [TODO]: Convert quaternion to rpy - 2015-05-25 08:17pm
// lastMsgPose - lastCloudPose = icpApprox
// lastRealCloudPose + (icpApprox + icpResult) = newRealCloudPose
void DatasetGenerator::computeCloudOdometry(
        shared_ptr<PM::DataPoints> currentCloud) {
    tf::Pose startPose = this->lastCloudPose;
    tf::Pose endPose = this->lastMsgPose;
    tf::Transform poseDiff = startPose.inverseTimes(endPose);

    if(this->pointCloudIndex != 0) {
    }
    //this->lastCloudPose = ;
    saveOdom();
}

void DatasetGenerator::saveOdom() {
    std::string filename = this->outputPath + "scan_";
    filename += this->getPaddedNum(this->pointCloudIndex, this->numSuffixWidth);
    filename += "_info.dat";

    //Eigen::Vector3f rollPitchYaw = getRollPitchYaw(
    //this->lastCorrectedPose.orientation);

    std::ofstream file;
    //file.open(filename.c_str());
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

Eigen::Vector3f DatasetGenerator::getRollPitchYaw(
        geometry_msgs::Quaternion quaternionMsg) {
    // RPY convention order seems to be ZYX
    tf::Quaternion quaternionTf;

    tf::quaternionMsgToTF(quaternionMsg, quaternionTf);

    double roll, pitch, yaw;
    tf::Matrix3x3(quaternionTf).getRPY(roll, pitch, yaw);

    return Eigen::Vector3f(roll, pitch, yaw);
}
