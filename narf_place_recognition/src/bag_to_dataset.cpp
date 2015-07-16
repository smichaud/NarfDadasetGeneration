#include "Definitions.hpp"
#include "DatasetGenerator.hpp"
#include "IcpOdometry.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <Eigen/Dense>

using std::string;
using std::vector;

vector<string> parseBagFiles(string bagFilesArguments);

int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_to_dataset");
    ros::NodeHandle nodeHandle("~");

    string bagFilesArg;
    string outputPath;
    string icpConfigPath;
    string cloudTopic;
    string poseTopic;
    bool isOdomOutput;
    nodeHandle.param("bagFiles", bagFilesArg, string(""));
    nodeHandle.param("outputPath", outputPath, string(""));
    nodeHandle.param("icpConfigPath", icpConfigPath, string(""));
    nodeHandle.param("cloudTopic", cloudTopic, string("/cloud"));
    nodeHandle.param("poseTopic", poseTopic,
            string("/robot_pose_ekf/odom_combined"));
    nodeHandle.param("isOdomOutput", isOdomOutput, bool(true));

    vector<string> topics;
    topics.push_back(cloudTopic);
    topics.push_back(poseTopic);

    DatasetGenerator datasetGenerator(outputPath, icpConfigPath, isOdomOutput);

    vector<string> bagFiles = parseBagFiles(bagFilesArg);

    int bagCount = 1;
    BOOST_FOREACH(const string currentBagFile, bagFiles) {
        rosbag::Bag currentBag;
        try{
            currentBag.open(currentBagFile, rosbag::bagmode::Read);
            rosbag::View view(currentBag, rosbag::TopicQuery(topics));

            ROS_INFO_STREAM("Processing bag file " 
                << bagCount << "/" <<  bagFiles.size() << std::endl);

            BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
                datasetGenerator.manageOdometryMsg(msg);
                datasetGenerator.managePointCloudMsg(msg);
            }

            datasetGenerator.setNextOdomEqualToLast();
            currentBag.close();
        } catch(rosbag::BagIOException exception) {
            ROS_WARN("Unable to open the bag file:");
            ROS_WARN_STREAM(currentBagFile);
        } catch(...) {
            ROS_WARN("Unexpected error occured for the bag file:");
            ROS_WARN_STREAM(currentBagFile);
        }
        bagCount++;
    }

    ROS_INFO("Processing done !");
}


vector<string> parseBagFiles(string bagFilesArguments) {
    vector<string> bagFiles;
    std::istringstream stringStream(bagFilesArguments);

    copy(std::istream_iterator<string>(stringStream),
            std::istream_iterator<string>(),
            std::back_inserter(bagFiles));

    return bagFiles;
}
