#include <gtest/gtest.h>

#include "Conversion.hpp"
#include "IcpOdometry.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_datatypes.h>

TEST(IcpOdometry, QuickTest) {
    PointCloud startCloud(PointCloud::load("./misc/start_cloud.vtk"));
    PointCloud endCloud(PointCloud::load("./misc/end_cloud.vtk"));

    Eigen::Matrix4f initTransfo = Eigen::Matrix4f::Identity();
    initTransfo(0,3) = 4.0;

    std::string configFile = "config/sick_icp.yaml";
    std::string output = "";

    Eigen::Matrix4f finalTransfo = IcpOdometry::getCorrectedTransfo(
            startCloud, endCloud, initTransfo, configFile, output);

    float expectedX = 3.8;
    float expectedY = 0.3;
    float expectedZ = -0.03;
    float finalX = finalTransfo(0,3);
    float finalY = finalTransfo(1,3);
    float finalZ = finalTransfo(2,3);
    float translationTolerance = 0.2;

    ASSERT_NEAR(finalX, expectedX, translationTolerance);
    ASSERT_NEAR(finalY, expectedY, translationTolerance);
    ASSERT_NEAR(finalZ, expectedZ, translationTolerance);
}
