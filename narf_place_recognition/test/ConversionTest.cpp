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

TEST(Conversion, getRpyIsValid) {
    float roll = 0.1;
    float pitch = 0.5;
    float yaw = 1;

    tf::Transform tfTransfo = tf::Transform(
            tf::createQuaternionFromRPY(roll, pitch, yaw));

    Eigen::Matrix4f mat = Conversion::tfToEigen(tfTransfo);
    Eigen::Vector3f result = Conversion::getRPY(mat);

    float tolerance = 0.000002;
    ASSERT_NEAR(result(0), roll, tolerance);
    ASSERT_NEAR(result(1), pitch, tolerance);
    ASSERT_NEAR(result(2), yaw, tolerance);
}

TEST(Conversion, getTranslationIsValid) {
    float x = 1;
    float y = 2;
    float z = 3;

    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0,3) = x;
    mat(1,3) = y;
    mat(2,3) = z;

    Eigen::Translation3f result = Conversion::getTranslation(mat);

    ASSERT_FLOAT_EQ(result.x(), x);
    ASSERT_FLOAT_EQ(result.y(), y);
    ASSERT_FLOAT_EQ(result.z(), z);
}

TEST(Conversion, getTranslationOnIdentityEqualsZero) {
    Eigen::Translation3f result = Conversion::getTranslation(
            Eigen::Matrix4f::Identity());

    ASSERT_FLOAT_EQ(result.x(), 0);
    ASSERT_FLOAT_EQ(result.y(), 0);
    ASSERT_FLOAT_EQ(result.z(), 0);
}


// Just for my own understanding of the pose mechanic
TEST(Conversion, poseDiffReturnCorrectValue) {
    float roll1 = 0.1;
    float pitch1 = 0.5;
    float yaw1 = 1;

    float roll2 = 0.1;
    float pitch2 = 0.5;
    float yaw2 = 1;

    tf::Pose start;
    start.setOrigin(tf::Vector3(1,1,1));
    start.setRotation(tf::createQuaternionFromRPY(roll1, pitch1, yaw1));
    tf::Pose end;

    Conversion::getPoseDiff(start, end);

    // Order might be different from ROS, but it does not matter.
    Eigen::AngleAxisf rollAngle1(roll1, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch1, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(yaw1, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf quat1 = rollAngle1 * pitchAngle * yawAngle;

    Eigen::AngleAxisf rollAngle2(roll1, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle2(pitch1, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle2(yaw1, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf quat2 = rollAngle2 * pitchAngle2 * yawAngle2;

    // Make sure quat1*quat1.inverse == Identity
    Eigen::Quaternionf identityQuat = quat1*quat1.inverse();
    ASSERT_FLOAT_EQ(identityQuat.x(), Eigen::Quaternionf::Identity().x());
    ASSERT_FLOAT_EQ(identityQuat.y(), Eigen::Quaternionf::Identity().y());
    ASSERT_FLOAT_EQ(identityQuat.z(), Eigen::Quaternionf::Identity().y());
    ASSERT_FLOAT_EQ(identityQuat.w(), Eigen::Quaternionf::Identity().w());

    // Test if the difference quatEnd*inv(quatStart) == quatDiff
    Eigen::Quaternionf composedQuat = quat2*quat1;
    Eigen::Quaternionf quatDiff = composedQuat*quat1.inverse();
    ASSERT_FLOAT_EQ(quatDiff.x(), quat2.x());
    ASSERT_FLOAT_EQ(quatDiff.y(), quat2.y());
    ASSERT_FLOAT_EQ(quatDiff.z(), quat2.z());
    ASSERT_FLOAT_EQ(quatDiff.w(), quat2.w());
}
