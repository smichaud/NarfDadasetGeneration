#include "IcpOdometry.hpp"
#include "Conversion.hpp"

#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <numeric>

#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

Eigen::Matrix4f loadCloudOdometry(std::string odomFilename);
void saveOdom(Eigen::Matrix4f odom, std::string filename);

std::string generateOdomFilename(std::string outputPath, int cloudIndex);
std::string getPaddedNum(const int &numSuffix);
void printPose(Eigen::Matrix4f pose);

int main(int argc, char *argv[]) {
    std::string path = "/home/smichaud/Desktop/Test/OdomCorrect/";
    int loopFirstIndex = 0;
    int loopLastIndex = 80;
    Eigen::Matrix4f correctedOdom = loadCloudOdometry(generateOdomFilename(path, loopFirstIndex));

    saveOdom(correctedOdom, generateOdomFilename(path + "CorrectedOdom/", 0));
    for(int i = loopFirstIndex+1; i <= loopLastIndex; ++i) {
        Eigen::Matrix4f previousOdom = loadCloudOdometry(generateOdomFilename(path, i-1));
        Eigen::Matrix4f currentOdom = loadCloudOdometry(generateOdomFilename(path, i));
        Eigen::Matrix4f odomDiff = Conversion::getPoseDiff(Conversion::eigenToTf(previousOdom), Conversion::eigenToTf(currentOdom));

        Eigen::Quaternionf initHeading = Conversion::getQuat(previousOdom);

        Eigen::Translation3f translation = Conversion::getTranslation(odomDiff);
        Eigen::Translation3f rotatedTranslation = Conversion::getTranslation(Eigen::Transform<float,3,Eigen::Affine>(initHeading*translation).matrix());

        Eigen::Translation3f previousCorrectedPosition = Conversion::getTranslation(correctedOdom);
        Eigen::Translation3f finalPosition = previousCorrectedPosition * rotatedTranslation;

        Eigen::Vector3f finalHeading = Conversion::getRPY(currentOdom);

        correctedOdom = Conversion::fromTranslationRPY(
                finalPosition.x(), finalPosition.y(), finalPosition.z(),
                finalHeading.x(), finalHeading.y(), finalHeading.z());

        saveOdom(correctedOdom, generateOdomFilename(path + "CorrectedOdom/", i));

        //std::cout << "===== Not corrected VS corrected odom =====" << std::endl;
        //printPose(currentOdom);
        //printPose(correctedOdom);

        //std::cout << "Initial translation:";
        //std::cout << Conversion::getTranslation(odomDiff).vector().transpose() << std::endl;
        //std::cout << "Initial heading:";
        //std::cout << Conversion::getRPY(previousOdom).transpose() << std::endl;
        //std::cout << "Rotated translation:";
        //std::cout << Conversion::getTranslation(rotatedTranslation).vector().transpose() << std::endl;
        //std::cout << "Final heading:";
        //std::cout << finalHeading.transpose() << std::endl;
        //std::cout << std::endl;
    }

    return 0;
}

Eigen::Matrix4f loadCloudOdometry(std::string odomFilename) {
    bool isOdomRetreived = false;
    float x, y, z, roll, pitch, yaw;

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

void saveOdom(Eigen::Matrix4f odom, std::string filename) {
    Eigen::Translation3f translation = Conversion::getTranslation(odom);
    Eigen::Vector3f rollPitchYaw = Conversion::getRPY(odom);

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
std::string generateOdomFilename(std::string outputPath, int cloudIndex) {
    std::string filename = outputPath + "scan_";
    filename += getPaddedNum(cloudIndex);
    filename += "_info.dat";

    return filename;
}

std::string getPaddedNum(const int &numSuffix) {
    std::ostringstream output;
    output << std::setfill('0') << std::setw(4) << numSuffix;

    return output.str();
}

void printPose(Eigen::Matrix4f pose) {
    Eigen::Translation3f translation = Conversion::getTranslation(
            pose);
    Eigen::Vector3f rollPitchYaw = Conversion::getRPY(
            pose);

    ROS_INFO_STREAM("Odometry (x y z r p y): "
            << translation.x() << " "
            << translation.y()  << " "
            << translation.z() << " "
            << rollPitchYaw(0) << " "
            << rollPitchYaw(1) << " "
            << rollPitchYaw(2) << " = "
            << translation.vector().norm() << " m");
}
