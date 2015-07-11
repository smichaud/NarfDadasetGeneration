#include <gtest/gtest.h>

#include "Conversion.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_datatypes.h>

TEST(Conversion, QuarternionsTfToEigenAreEquals) {
    tf::Quaternion tfQuat = tf::createQuaternionFromRPY(
            M_PI_4, M_PI_4, M_PI_4);

    Eigen::Quaternionf eigenQuat = Conversion::tfToEigen(tfQuat);

    ASSERT_FLOAT_EQ(eigenQuat.x(), tfQuat.x());
    ASSERT_FLOAT_EQ(eigenQuat.y(), tfQuat.y());
    ASSERT_FLOAT_EQ(eigenQuat.z(), tfQuat.z());
    ASSERT_FLOAT_EQ(eigenQuat.w(), tfQuat.w());
}

TEST(Conversion, QuarternionsEigenToTfAreEquals) {
    Eigen::AngleAxisf rollAngle(M_PI_4, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(M_PI_4, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(M_PI_4, Eigen::Vector3f::UnitX());

    Eigen::Quaternionf eigenQuat = rollAngle * yawAngle * pitchAngle;

    tf::Quaternion tfQuat = Conversion::eigenToTf(eigenQuat);

    ASSERT_FLOAT_EQ(tfQuat.x(), eigenQuat.x());
    ASSERT_FLOAT_EQ(tfQuat.y(), eigenQuat.y());
    ASSERT_FLOAT_EQ(tfQuat.z(), eigenQuat.z());
    ASSERT_FLOAT_EQ(tfQuat.w(), eigenQuat.w());
}

TEST(Conversion, EigenTransformationMatrixFromTfIsValid) {
    tf::Vector3 tfTranslation = tf::Vector3(1, 2, 3);
    tf::Quaternion tfQuat = tf::createQuaternionFromRPY(
            M_PI_4, M_PI_4, M_PI_4);

    tf::Transform tfTransfo;
    tfTransfo.setOrigin(tfTranslation);
    tfTransfo.setRotation(tfQuat);

    Eigen::Matrix4f eigenMatrix = Conversion::tfToEigen(tfTransfo);
    Eigen::Transform<float,3,Eigen::Affine> eigenTransform(eigenMatrix);
    Eigen::Quaternionf eigenQuat =
        Eigen::Quaternionf(eigenTransform.rotation());

    ASSERT_FLOAT_EQ(eigenTransform.translation().x(), tfTranslation.x());
    ASSERT_FLOAT_EQ(eigenTransform.translation().y(), tfTranslation.y());
    ASSERT_FLOAT_EQ(eigenTransform.translation().z(), tfTranslation.z());

    ASSERT_FLOAT_EQ(eigenQuat.x(), tfTransfo.getRotation().x());
    ASSERT_FLOAT_EQ(eigenQuat.y(), tfTransfo.getRotation().y());
    ASSERT_FLOAT_EQ(eigenQuat.z(), tfTransfo.getRotation().z());
    ASSERT_FLOAT_EQ(eigenQuat.w(), tfTransfo.getRotation().w());
}
