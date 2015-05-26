#include "DatasetGenerator.hpp"
#include "pointmatcher_ros/point_cloud.h"

#include <iostream>
#include <iomanip>
#include <fstream>

using namespace PointMatcher_ros;

DatasetGenerator::DatasetGenerator():
    pointCloudIndex(0), outputPath("/home/smichaud/Desktop/test/") {
    }

void DatasetGenerator::managePointCloudMsg(rosbag::MessageInstance const &msg) {
    shared_ptr<sensor_msgs::PointCloud2> cloudMsg =
        msg.instantiate<sensor_msgs::PointCloud2>();

    if(cloudMsg != NULL) {
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

void DatasetGenerator::manageOdometryMsg(rosbag::MessageInstance const &msg) {
    shared_ptr<geometry_msgs::PoseWithCovarianceStamped> odomMsg =
        msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();

    if(odomMsg != NULL) {
        this->lastMsgPose = odomMsg->pose.pose;
    }
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

// [TODO]: Write this function properly - 2015-05-25 08:16pm
void DatasetGenerator::saveOdom() {
    std::string filename = this->outputPath + "scan_";
    filename = this->appendNum(filename, this->pointCloudIndex);
    filename += "_info.dat";
    std::ofstream file;

    //x y z roll pitch yaw
    file.open(filename.c_str());
    file << this->lastRealPose.position
        << this->lastRealPose.orientation;

    file.close();
}

std::string DatasetGenerator::getCloudFilename() {
    std::string filename = "scan_";
    filename = appendNum(filename, pointCloudIndex);
    filename += ".pcd";

    return this->outputPath + filename;
}

std::string DatasetGenerator::appendNum(const std::string &input,
        const int &numSuffix) {
    std::ostringstream output;
    output << input << std::setfill('0') << std::setw(3) << numSuffix;

    return output.str();
}
