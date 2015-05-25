#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "boost/foreach.hpp"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"

#include <iostream>
#include <iomanip>

// [TODO]: Do ICP for next (with odom?) - 2015-05-21 12:36pm
// [TODO]: Try to get the far range pointcloud - 2015-05-21 12:37pm
// If your scan_001_far_ranges.pcd.
// [TODO]: scan_001_info.dat - 2015-05-21 12:38pm

typedef PointMatcher<float> PM;

bool managePointCloudMsg(rosbag::MessageInstance const &msg, int index);
std::string appendNum(std::string input, int numSuffix);

int main(int argc, char **argv) {
    rosbag::Bag bag;
    bag.open("/home/smichaud/Desktop/test/test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/cloud"));
    //topics.push_back(std::string("/robot_pose_ekf/odom"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int pointcloudIndex = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        std::cout << "Msg received :  " << msg.getDataType() << std::endl;

        if(managePointCloudMsg(msg, pointcloudIndex)) {
            pointcloudIndex++;
        }
    }

    bag.close();
}

bool managePointCloudMsg(rosbag::MessageInstance const &msg, int index) {
    boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg =
        msg.instantiate<sensor_msgs::PointCloud2>();

    if(cloud_msg != NULL) {
        PM::DataPoints pmCloud(
                PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
                    *cloud_msg));
        int nbPoints = pmCloud.features.cols();
        ROS_INFO("PointCloud received with %d points", nbPoints);

        std::string path = "/home/smichaud/Desktop/test/";
        std::string filename = "scan_";
        filename = appendNum(filename, index);
        filename += ".pcd";

        pmCloud.save(path + filename);

        return true;
    }
    return false;
}

std::string appendNum(std::string input, int numSuffix) {
    std::ostringstream output;

    output << input << std::setfill('0') << std::setw(3) << numSuffix;

    return output.str();
}
