#include <gtest/gtest.h>

#include "Conversion.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

TEST(Conversion, QuarternionsTfToEigenAreEquals) {
    float x = 2;
    float y = 1;
    float z = 3;
    float w = 3;
    tf::Quaternion tfQuat = tf::Quaternion(x, y, z, w);

    Eigen::Quaternionf eigenQuat = Conversion::tfToEigen(tfQuat);

    ASSERT_EQ(eigenQuat.x(), tfQuat.x());
    ASSERT_EQ(eigenQuat.y(), tfQuat.y());
    ASSERT_EQ(eigenQuat.z(), tfQuat.z());
    ASSERT_EQ(eigenQuat.w(), tfQuat.w());
}

TEST(Conversion, QuarternionsEigenToTfAreEquals) {
    float x = 2;
    float y = 1;
    float z = 3;
    float w = 3;
    Eigen::Quaternionf eigenQuat = Eigen::Quaternionf(w, x, y, z);

    tf::Quaternion tfQuat = Conversion::eigenToTf(eigenQuat);

    ASSERT_EQ(tfQuat.x(), eigenQuat.x());
    ASSERT_EQ(tfQuat.y(), eigenQuat.y());
    ASSERT_EQ(tfQuat.z(), eigenQuat.z());
    ASSERT_EQ(tfQuat.w(), eigenQuat.w());
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
