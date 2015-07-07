#include "Definitions.hpp"
//#include "DatasetGenerator.hpp"
#include "IcpOdometry.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <Eigen/Dense>

//using std::string;
//using std::vector;

int main(int argc, char **argv) {
    //ros::init(argc, argv, "bag_to_dataset");
    //ros::NodeHandle nodeHandle("~");

    //string bagFile;
    //string cloudTopic;
    //string poseTopic;
    //string outputPath;
    //nodeHandle.param("bagFile", bagFile, string(""));
    //nodeHandle.param("cloudTopic", cloudTopic, string("/cloud"));
    //nodeHandle.param("poseTopic", poseTopic,
    //string("/robot_pose_ekf/odom_combined"));
    //nodeHandle.param("outputPath", outputPath, string(""));

    //vector<string> topics;
    //topics.push_back(cloudTopic);
    //topics.push_back(poseTopic);

    //rosbag::Bag bag;
    //bag.open(bagFile, rosbag::bagmode::Read);
    //rosbag::View view(bag, rosbag::TopicQuery(topics));

    //DatasetGenerator datasetGenerator(outputPath);
    //BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
    //datasetGenerator.manageOdometryMsg(msg);
    //datasetGenerator.managePointCloudMsg(msg);
    //}

    //bag.close();

    //ROS_INFO("bag_to_dataset processing done");

    const char *refFile("/home/smichaud/Desktop/pc0.vtk");
    const char *dataFile("/home/smichaud/Desktop/pc1.vtk");
    const PointCloud startCloud(PointCloud::load(dataFile));
    const PointCloud endCloud(PointCloud::load(refFile));
    Transformation initTransfo = Transformation::Identity();

    icpodometry::getTransfo(startCloud, endCloud, initTransfo,
            "/home/smichaud/Workspace/NarfDadasetGeneration/narf_place_recognition/config/sick_icp.yaml");
}
