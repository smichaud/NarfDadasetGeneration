#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "boost/foreach.hpp"
#include "DatasetGenerator.hpp"

// [TODO]: Do ICP for next (with odom?) - 2015-05-21 12:36pm
// [TODO]: Try to get the far range pointcloud - 2015-05-21 12:37pm
// If your scan_001_far_ranges.pcd.
// [TODO]: scan_001_info.dat - 2015-05-21 12:38pm

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
