#include "IcpOdometry.hpp"
#include "Conversion.hpp"

#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <numeric>
#include <string>

#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

bool userOdomAdjustment(Eigen::Matrix4f& initTransfo, const std::string& filename);
void printPose(Eigen::Matrix4f pose);
std::string toString(int num);

int main(int argc, char *argv[]) {
    std::string filepath1 = "/media/D/Datasets/PlaceRecognition/VelodyneForest/Output/scan_0077.pcd";
    std::string filepath2 = "/media/D/Datasets/PlaceRecognition/VelodyneForest/Output/scan_0183.pcd";

    PM::DataPoints initCloud = PM::DataPoints::load(filepath1);
    PM::DataPoints finalCloud = PM::DataPoints::load(filepath2);
    Eigen::Matrix4f initTransfo = Eigen::Matrix4f::Identity();
    std::string icpConfig = "/home/smichaud/Workspace/NarfDadasetGeneration/narf_place_recognition/config/sick_icp.yaml";
    std::string cloudOutputFile = "/home/smichaud/Desktop/Temp/BruteForce";
    bool isOdomMergedCloudsSaved = true;

    Eigen::Matrix4f icpOdom;

    //////////////////// Legit version
    bool isOdomGood = false;
    while(!isOdomGood) {
        icpOdom = IcpOdometry::getCorrectedTransfo(
                initCloud, finalCloud,
                initTransfo, icpConfig, cloudOutputFile,
                isOdomMergedCloudsSaved);
        printPose(icpOdom);
        isOdomGood = !userOdomAdjustment(initTransfo, cloudOutputFile);
    }

    //////////////////// Brute force version
    //std::string inputBuffer;
    //std::cout << "Press <enter>" << std::endl;
    //std::string viewerApp = "paraview ";
    //viewerApp.append(cloudOutputFile + ".vtk &");
    //std::string paraviewKillCommand = "pkill -SIGTERM paraview";

    //int translationRangeValue = 1;
    //int rotationRangeValue = 3;

    //float rotationIncrement = 0.10;
    //float translationIncrement = 0.75;

    //// Offset, aka your initial guess
    //float offsetX = 0.0;
    //float offsetY = 2.0;
    //float offsetYaw = 0.5;

    //int indexForFile = 0;
    //for(int i = -translationRangeValue; i <= translationRangeValue; ++i){
        //for(int j = -translationRangeValue; j <= translationRangeValue; ++j) {
            //for(int k = -rotationRangeValue; k <= rotationRangeValue; ++k) {
                //float testYaw = k*rotationIncrement + offsetYaw;
                //float testX = i*translationIncrement + offsetX;
                //float testY = j*translationIncrement + offsetY;
                //std::cout << "Testing:" << testX << " " << testY << " " << testYaw << std::endl;

                //initTransfo = Conversion::fromTranslationRPY(testX, testY, 0, 0, 0, testYaw);
                //icpOdom = IcpOdometry::getCorrectedTransfo(
                        //initCloud, finalCloud,
                        //initTransfo, icpConfig, cloudOutputFile + toString(indexForFile),
                        //isOdomMergedCloudsSaved);

                //printPose(icpOdom);
                //indexForFile++;

                ////std::cout << "ICP transformation: "
                    ////<< Conversion::getTranslation(icpOdom).x()  << " "
                    ////<< Conversion::getTranslation(icpOdom).y()  << " "
                    ////<< Conversion::getTranslation(icpOdom).z()  << " "
                    ////<< Conversion::getRPY(icpOdom).transpose() << std::endl;

                ////std::system(viewerApp.c_str());
                ////std::getline(std::cin, inputBuffer);
                ////if(!inputBuffer.empty()) {}
                ////std::system(paraviewKillCommand.c_str());
            //}
        //}
    //}
    ///////////////////////////////////////////////////////////////////////////


    std::cout << std::endl << "Final transformation:" << std::endl;
    printPose(icpOdom);

    return 0;
}

bool userOdomAdjustment(Eigen::Matrix4f& initTransfo, const std::string& filename) {
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

std::string toString(int num) {
    std::ostringstream result;
    result << std::setfill('0') << std::setw(3) << num;
    return result.str();
}
