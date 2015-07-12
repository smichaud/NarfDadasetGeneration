#ifndef CONVERSION_H
#define CONVERSION_H

#include "Definitions.hpp"
#include <pointmatcher/PointMatcher.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

namespace Conversion {
    Eigen::Quaternionf tfToEigen(tf::Quaternion tfQuat);
    tf::Quaternion eigenToTf(Eigen::Quaternionf eigenQuat);

    Eigen::Translation3f tfToEigen(tf::Vector3 tfTranslation);
    tf::Vector3 eigenToTf(Eigen::Translation3f eigenTranslation);

    Eigen::Matrix4f tfToEigen(tf::Transform tfTransfo);

    Eigen::Vector3f getRPY(Eigen::Matrix4f transfo);
    Eigen::Translation3f getTranslation(Eigen::Matrix4f transfo);
}

#endif
