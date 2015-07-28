#ifndef CONVERSION_H
#define CONVERSION_H

#include "Definitions.hpp"
#include <pointmatcher/PointMatcher.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

namespace Conversion {
    Eigen::Quaternionf tfToEigen(const tf::Quaternion tfQuat);
    tf::Quaternion eigenToTf(const Eigen::Quaternionf eigenQuat);

    Eigen::Translation3f tfToEigen(const tf::Vector3 tfTranslation);
    tf::Vector3 eigenToTf(const Eigen::Translation3f eigenTranslation);

    Eigen::Matrix4f tfToEigen(const tf::Transform tfTransfo);

    Eigen::Quaternionf getQuat(const Eigen::Matrix4f transfo);
    Eigen::Vector3f getRPY(const Eigen::Matrix4f transfo);
    Eigen::Translation3f getTranslation(const Eigen::Matrix4f transfo);

    Eigen::Matrix4f getPoseDiff(const tf::Pose& start, const tf::Pose& end);
    Eigen::Matrix4f getPoseComposition(const Eigen::Matrix4f& start,
            const Eigen::Matrix4f& increment);
}

#endif
