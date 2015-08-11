#include "DatasetGenerator.hpp"
#include "IcpOdometry.hpp"
#include "Conversion.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <cstdlib>

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
    totalCloudIndex(0),
    cloudIndex(0),
    currentLoopCloudIndex(0),
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
        if(this->totalCloudIndex % this->pointCloudKeepOneOutOf == 0) {
            ROS_INFO_STREAM("Processing cloud " << this->cloudIndex);

            shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                        rosMsgToPointMatcherCloud<float>(*cloudMsg)));

            try{
                cloud->save(generateCloudFilename(this->cloudIndex));
            } catch(...) {
                ROS_ERROR("Unable to save in the directory provided.");
            }

            if(this->isOdomOutput) {
                this->computeCloudOdometry(cloud);
                this->saveOdom();
            }

            this->cloudIndex++;
            this->currentLoopCloudIndex++;
            this->lastPointCloud = cloud;
        }
        this->totalCloudIndex++;
    }
}

void DatasetGenerator::computeCloudOdometry(
        shared_ptr<PM::DataPoints> currentCloud) {
    if(this->cloudIndex != 0) {
        Eigen::Matrix4f initTransfo = Eigen::Matrix4f::Identity();

        if(this->isNextOdomEqualToLast) {
            this->isNextOdomEqualToLast = false;
        } else if(this->isNextOdomEqualToFirst) {
            initTransfo = this->setFirstLoopBestMatch();
        } else {
            initTransfo = Conversion::tfToEigen(
                    this->getPoseDiffFromLastCloud());
        }

        std::string filename = "";
        if(this->isOdomMergedCloudsSaved) {
            filename += this->outputPath + "scan_merged_";
            filename += this->getPaddedNum(this->cloudIndex,
                    this->numSuffixWidth);
        }

        Eigen::Matrix4f icpOdom;
        do {
        icpOdom = IcpOdometry::getCorrectedTransfo(
                *this->lastPointCloud, *currentCloud,
                initTransfo, this->icpConfigPath, filename,
                this->isOdomMergedCloudsSaved);
        } while(this->isOdomMergedCloudsSaved &&
                this->userOdomAdjustment(initTransfo, filename));

        //std::cout << "===== Output ICP difference =====" << std::endl;
        //float yawInit = Conversion::getRPY(initTransfo)(2);
        //float yawCorr = Conversion::getRPY(icpOdom)(2);
        //std::cout << "Yaw Correction : "
            //<< yawCorr-yawInit
            //<< std::endl;
        //if((yawCorr > 0 && yawInit > 0)||(yawCorr < 0 && yawInit < 0)) {
            //std::cout << "Same sign" << std::endl;
        //}
        //std::cout << "=================================" << std::endl;

        this->lastCorrectedPose = Conversion::getPoseComposition(
                this->lastCorrectedPose, icpOdom);
    }

    this->lastCloudPose = this->lastMsgPose;
}

Eigen::Matrix4f DatasetGenerator::setFirstLoopBestMatch() {
    int bestIndex = 0;
    float bestDistance = std::numeric_limits<float>::infinity();
    Eigen::Matrix4f initTransfo = Eigen::Matrix4f::Identity();

    if(this->currentLoopCloudIndex != 0) {
        tf::Pose lastRealPose = Conversion::eigenToTf(this->lastCorrectedPose);
        tf::Pose currentPose = Conversion::getPoseComposition(lastRealPose,
                this->getPoseDiffFromLastCloud());

        for(int i = 0; i < this->firstLoopPoses.size() ; ++i) {
            tf::Pose firstLoopPose = this->firstLoopPoses[i];
            float distance = Conversion::getL2Distance(currentPose,
                    firstLoopPose);
            if(distance < bestDistance) {
                bestIndex = i;
                bestDistance = distance;
            }
        }

        this->lastCorrectedPose = Conversion::tfToEigen(
                this->firstLoopPoses[bestIndex]);

        initTransfo = Conversion::tfToEigen(
                this->firstLoopPoses[bestIndex].inverseTimes(currentPose));
    }

    shared_ptr<PM::DataPoints> closestPointCloud(
            new PM::DataPoints(PM::DataPoints::load(
                    this->generateCloudFilename(bestIndex))));
    this->lastPointCloud = closestPointCloud;

    return initTransfo;
}

bool DatasetGenerator::userOdomAdjustment(Eigen::Matrix4f& initTransfo,
        const std::string& filename) {
    char requireAdjustment;
    std::cout << "Enter (y) if odom need adjustment : " << std::endl;

    std::string viewerApp = "paraview ";
    viewerApp.append(filename + ".vtk &");
    std::system(viewerApp.c_str());

    std::cin.get(requireAdjustment);

    if(requireAdjustment == 'y' || requireAdjustment == 'Y') {
        std::cout << "Magic adjustment will be done !" << std::endl;
        Eigen::AngleAxisf rotTest(0.2, Eigen::Vector3f::UnitZ());
        initTransfo.block<3,3>(0,0) =
            rotTest.toRotationMatrix()*(initTransfo.block<3,3>(0,0));

        return true;
    }

    return false;
}

tf::Pose DatasetGenerator::getPoseDiffFromLastCloud() {
    tf::Pose startPose(this->lastCloudPose);
    tf::Pose endPose(this->lastMsgPose);
    return startPose.inverseTimes(endPose);
}

void DatasetGenerator::saveOdom() {
    std::string filename = this->outputPath + "scan_";
    filename += this->getPaddedNum(this->cloudIndex, this->numSuffixWidth);
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
        << translation.x() << " "
        << translation.y()  << " "
        << translation.z() << " "
        << rollPitchYaw(0) << " "
        << rollPitchYaw(1) << " "
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

void DatasetGenerator::setNextOdomEqualToFirst() {
    this->isNextOdomEqualToFirst = true;
    this->isFirstLoop = false;
    this->currentLoopCloudIndex = 0;
    this->lastCorrectedPose.setIdentity();
}

void DatasetGenerator::setNextOdomEqualToLast() {
    this->isNextOdomEqualToLast = true;
    this->isFirstLoop = false;
    this->currentLoopCloudIndex = 0;
}

std::string DatasetGenerator::generateCloudFilename(int cloudIndex) {
    std::string filename = "scan_";
    filename += this->getPaddedNum(cloudIndex, this->numSuffixWidth);
    filename += ".pcd";

    return this->outputPath + filename;
}

std::string DatasetGenerator::getPaddedNum(const int &numSuffix,
        const int width) {
    std::ostringstream output;
    output << std::setfill('0') << std::setw(width) << numSuffix;

    return output.str();
}
