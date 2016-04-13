#include "DatasetGenerator.hpp"
#include "IcpOdometry.hpp"
#include "Conversion.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <cstdlib>

#include <boost/filesystem.hpp>
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
            std::cout << "" << std::endl;
            ROS_INFO_STREAM("===== Processing cloud " << this->cloudIndex);

            shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                        rosMsgToPointMatcherCloud<float>(*cloudMsg)));


            std::string cloudFilename = generateCloudFilename(this->cloudIndex);
            if(!boost::filesystem::exists(cloudFilename)) {
                try{
                    cloud->save(cloudFilename);
                } catch(...) {
                    ROS_ERROR("Unable to save in the directory provided.");
                }
            } else {
                ROS_INFO_STREAM("Cloud file exists and will not be replaced:"
                        << cloudFilename);
            }


            if(this->isOdomOutput) {
                std::string odomFilename = this->generateOdomFilename(
                        this->cloudIndex);
                if(boost::filesystem::exists(odomFilename)) {
                    ROS_INFO_STREAM("Odometry file exists and will be loaded");
                    this->lastCorrectedPose = this->loadCloudOdometry(
                            this->cloudIndex);
                } else{
                    this->lastCorrectedPose = this->computeCloudOdometry(cloud);
                    this->saveOdom();
                }

                if(this->isFirstLoop) {
                    this->firstLoopPoses.push_back(
                            Conversion::eigenToTf(this->lastCorrectedPose));
                }

                this->printLastCorrectedPose();
                this->lastCloudPose = this->lastMsgPose;
            }


            this->cloudIndex++;
            this->currentLoopCloudIndex++;
            this->lastPointCloud = cloud;
        }
        this->totalCloudIndex++;
    }
}

Eigen::Matrix4f DatasetGenerator::loadCloudOdometry(const int cloudIndex) {
    bool isOdomRetreived = false;
    float x, y, z, roll, pitch, yaw;

    std::string odomFilename = this->generateOdomFilename(this->cloudIndex);
    std::ifstream file;
    file.open(odomFilename.c_str());

    if(file.good()) {
        std::string token;
        try {
            while(!file.eof()) {
                std::string line;
                std::getline(file, line);

                if(!line.empty() && line[0] != '#') {
                    std::stringstream lineStream(line);
                    std::string token;
                    lineStream >> token;

                    if(token == "Odometry:") {
                        lineStream >> x >> y >> z >> roll >> pitch >> yaw;
                        isOdomRetreived = true;
                    }
                }
            }
        } catch(...) {
            ROS_ERROR_STREAM("Unable to process the odometry file: "
                    << odomFilename);
        }
    } else {
        ROS_ERROR_STREAM("Unable to open the odometry file: " << odomFilename);
    }

    file.close();

    if(isOdomRetreived) {
        return Conversion::fromTranslationRPY(
                x, y, z, roll, pitch, yaw);
    } else {
        ROS_ERROR_STREAM("Odometry was not loaded from file: " << odomFilename);
    }

    return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f DatasetGenerator::computeCloudOdometry(
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
            filename += this->outputPath + "scan_";
            filename += this->getPaddedNum(this->cloudIndex,
                    this->numSuffixWidth);
            filename += "_merged";
        }

        Eigen::Matrix4f icpOdom;
        bool isOdomGood = false;
        while(this->isOdomMergedCloudsSaved && !isOdomGood) {
            std::cout << "Init transformation: "
                << Conversion::getTranslation(initTransfo).x()  << " "
                << Conversion::getTranslation(initTransfo).y()  << " "
                << Conversion::getTranslation(initTransfo).z()  << " "
                << Conversion::getRPY(initTransfo).transpose() << std::endl;

            icpOdom = IcpOdometry::getCorrectedTransfo(
                    *this->lastPointCloud, *currentCloud,
                    initTransfo, this->icpConfigPath, filename,
                    this->isOdomMergedCloudsSaved);

            std::cout << "ICP transformation: "
                << Conversion::getTranslation(icpOdom).x()  << " "
                << Conversion::getTranslation(icpOdom).y()  << " "
                << Conversion::getTranslation(icpOdom).z()  << " "
                << Conversion::getRPY(icpOdom).transpose() << std::endl;

            isOdomGood = !this->userOdomAdjustment(initTransfo, filename);
        }

        return Conversion::getPoseComposition(
                this->lastCorrectedPose, icpOdom);
    }

    return Eigen::Matrix4f::Identity();
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

    std::cout << "Best from 1st loop : " << bestIndex << " with initTransfo : "
        << Conversion::getTranslation(initTransfo).x()  << " "
        << Conversion::getTranslation(initTransfo).y()  << " "
        << Conversion::getTranslation(initTransfo).z()  << " "
        << Conversion::getRPY(initTransfo).transpose() << std::endl;

    return initTransfo;
}

bool DatasetGenerator::userOdomAdjustment(Eigen::Matrix4f& initTransfo,
        const std::string& filename) {
    std::string inputBuffer;
    float value;
    std::cout << "Press <enter> if no adjustment required. "
        << "Else initial value to change, enter x or (y)aw or (a)ll: ";

    std::string viewerApp = "paraview ";
    viewerApp.append(filename + ".vtk &");
    std::system(viewerApp.c_str());
    std::string paraviewKillCommand = "pkill -SIGTERM paraview";

    std::getline(std::cin, inputBuffer);

    if(!inputBuffer.empty()) {
        float x = Conversion::getTranslation(initTransfo).x();
        float y = Conversion::getTranslation(initTransfo).y();
        float z = Conversion::getTranslation(initTransfo).z();
        float roll = Conversion::getRPY(initTransfo).x();
        float pitch = Conversion::getRPY(initTransfo).y();
        float yaw = Conversion::getRPY(initTransfo).z();

        if(inputBuffer == "x") {
            std::cout << "Enter x: ";

            std::getline(std::cin, inputBuffer);
            std::stringstream lineStream(inputBuffer);
            lineStream >> x;
        } else if(inputBuffer == "yaw" || inputBuffer == "y") {
            std::cout << "Enter yaw: ";

            std::getline(std::cin, inputBuffer);
            std::stringstream lineStream(inputBuffer);
            lineStream >> yaw;
        } else if(inputBuffer == "all" || inputBuffer == "a") {
            std::cout << "Enter x y z roll pitch yaw: ";

            std::getline(std::cin, inputBuffer);
            std::stringstream lineStream(inputBuffer);
            lineStream >> x >> y >> z >> roll >> pitch >> yaw;
        }

        //Eigen::Matrix4f increment = Conversion::fromTranslationRPY(
                //x,y,z,roll,pitch,yaw);
        //initTransfo = Conversion::getPoseComposition(initTransfo, increment);
        initTransfo = Conversion::fromTranslationRPY(x,y,z,roll,pitch,yaw);

        std::system(paraviewKillCommand.c_str());
        return true;
    }

    std::cout << "No adjusment will be done." << std::endl;

    std::system(paraviewKillCommand.c_str());

    return false;
}

tf::Pose DatasetGenerator::getPoseDiffFromLastCloud() {
    tf::Pose startPose(this->lastCloudPose);
    tf::Pose endPose(this->lastMsgPose);
    return startPose.inverseTimes(endPose);
}

void DatasetGenerator::saveOdom() {
    std::string filename = this->generateOdomFilename(this->cloudIndex);

    Eigen::Translation3f translation = Conversion::getTranslation(
            this->lastCorrectedPose);
    Eigen::Vector3f rollPitchYaw = Conversion::getRPY(
            this->lastCorrectedPose);

    std::ofstream file;
    file.open(filename.c_str());
    file << std::setprecision(30) << "Odometry: "
        << translation.x() << " "
        << translation.y()  << " "
        << translation.z() << " "
        << rollPitchYaw(0) << " "
        << rollPitchYaw(1) << " "
        << rollPitchYaw(2) << std::endl;

    file.close();
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

void DatasetGenerator::printLastCorrectedPose() {
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
}

std::string DatasetGenerator::generateCloudFilename(int cloudIndex) {
    std::string filename = "scan_";
    filename += this->getPaddedNum(cloudIndex, this->numSuffixWidth);
    filename += ".pcd";

    return this->outputPath + filename;
}

std::string DatasetGenerator::generateOdomFilename(int cloudIndex) {
    std::string filename = this->outputPath + "scan_";
    filename += this->getPaddedNum(cloudIndex, this->numSuffixWidth);
    filename += "_info.dat";

    return filename;
}

std::string DatasetGenerator::getPaddedNum(const int &numSuffix,
        const int width) {
    std::ostringstream output;
    output << std::setfill('0') << std::setw(width) << numSuffix;

    return output.str();
}
