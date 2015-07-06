// [TODO]: Try to get the far range pointcloud - 2015-05-21 12:37pm
// If your scan_001_far_ranges.pcd.
// [TODO]: scan_001_info.dat - 2015-05-21 12:38pm
#include "DatasetGenerator.hpp"
#include "IcpOdometry.hpp"

#include "pointmatcher_ros/point_cloud.h"

#include <iostream>
#include <iomanip>
#include <fstream>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

using PointMatcher_ros::rosMsgToPointMatcherCloud;
using std::ifstream;
using std::cerr;

DatasetGenerator::DatasetGenerator(const string outputPath):
    outputPath(outputPath),
    pointCloudIndex(0),
    numSuffixWidth(4),
    minDistBetweenPointClouds(2.5) {
        lastRealPose.orientation.x = 0;
        lastRealPose.orientation.y = 0;
        lastRealPose.orientation.z = 0;
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

        cloud->save(generateCloudFilename());
        this->computeCloudOdometry(cloud);
        this->pointCloudIndex++;
        this->lastPointCloud = cloud;
    }
}

float DatasetGenerator::getDistFromLastPosition() {
    return 50.0; // Added for the Velodyne (so I don't take every cloud)
}

// [TODO]: Compute/write odom - 2015-05-25 03:11pm
// [TODO]: Convert quaternion to rpy - 2015-05-25 08:17pm
// lastMsgPose - lastCloudPose = icpApprox
// lastRealCloudPose + (icpApprox + icpResult) = newRealCloudPose
void DatasetGenerator::computeCloudOdometry(
        shared_ptr<PM::DataPoints> currentCloud) {
    if(this->pointCloudIndex != 0) {
        PM::ICP icp;

        string configFile = "/home/smichaud/Workspace/NarfDadasetGeneration/narf_place_recognition/config/sick_icp.yaml";
        std::ifstream inputConfigFile(configFile.c_str());
        if (!inputConfigFile.good()) {
            cerr << "Cannot open config file " << configFile << std::endl;
        }

        int cloudDimension = 3;
        PM::TransformationParameters parsedTranslation;
        parsedTranslation = PM::TransformationParameters::Identity(
                cloudDimension+1,cloudDimension+1);

        PM::TransformationParameters parsedRotation;
        parsedRotation = PM::TransformationParameters::Identity(
                cloudDimension+1,cloudDimension+1);

        icp.loadFromYaml(inputConfigFile);
    }
    //this->lastCloudPose = ;
    saveOdom();
}

void DatasetGenerator::saveOdom() {
    string filename = this->outputPath + "scan_";
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

string DatasetGenerator::generateCloudFilename() {
    string filename = "scan_";
    filename += this->getPaddedNum(pointCloudIndex, this->numSuffixWidth);
    filename += ".pcd";

    return this->outputPath + filename;
}

string DatasetGenerator::getPaddedNum(const int &numSuffix,
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
