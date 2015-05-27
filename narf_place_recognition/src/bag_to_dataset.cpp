#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "boost/foreach.hpp"
#include "DatasetGenerator.hpp"
#include <Eigen/Dense>

int main(int argc, char **argv) {
    rosbag::Bag bag;
    bag.open("/home/smichaud/Desktop/test/test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/cloud"));
    topics.push_back(std::string("/robot_pose_ekf/odom_combined"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    DatasetGenerator datasetGenerator;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
    datasetGenerator.manageOdometryMsg(msg);
    datasetGenerator.managePointCloudMsg(msg);
    }

    bag.close();
}
