#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "boost/foreach.hpp"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"

// [TODO]: Save it with num suffix - 2015-05-21 12:36pm
// [TODO]: Do ICP for next (with odom?) - 2015-05-21 12:36pm
// [TODO]: Save as PCD - 2015-05-21 12:36pm
// [TODO]: Try to get the far range pointcloud - 2015-05-21 12:37pm
// [TODO]: scan_001.pcd with scan_001_far.pcd - 2015-05-21 12:38pm
// [TODO]: scan_001_info.dat - 2015-05-21 12:38pm

typedef PointMatcher<float> PM;
using namespace std;

int main(int argc, char **argv) {
    rosbag::Bag bag;
    bag.open("/home/smichaud/Desktop/test/test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/cloud"));
    //topics.push_back(std::string("/robot_pose_ekf/odom"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int sample_index = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        std::cout << "Msg received :  " << msg.getDataType() << std::endl;
        boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg =
            msg.instantiate<sensor_msgs::PointCloud2>();

        if(cloud_msg != NULL) {
            PM::DataPoints pmCloud(
                    PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
                        *cloud_msg));
            int nbPoints = pmCloud.features.cols();
            ROS_INFO("PointCloud received with %d points", nbPoints);

            pmCloud.save("/home/smichaud/Desktop/test/test.pcd");
        }
    }

    bag.close();
}

