#include "Conversion.hpp"

namespace Conversion {
    Eigen::Quaternionf tfToEigen(tf::Quaternion tfQuat) {
        return Eigen::Quaternionf(
                tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z());
    }

    tf::Quaternion eigenToTf(Eigen::Quaternionf eigenQuat) {
        return tf::Quaternion(
                eigenQuat.x(), eigenQuat.y(), eigenQuat.z(), eigenQuat.w());
    }

    Eigen::Translation3f tfToEigen(tf::Vector3 tfTranslation) {
        return Eigen::Translation3f(
                tfTranslation.x(), tfTranslation.y(), tfTranslation.z());
    }

    tf::Vector3 eigenToTf(Eigen::Translation3f eigenTranslation) {
        return tf::Vector3(eigenTranslation.x(), eigenTranslation.y(),
                eigenTranslation.z());
    }

    Eigen::Matrix4f tfToEigen(tf::Transform tfTransfo) {
        Eigen::Quaternionf eigenQuat = tfToEigen(tfTransfo.getRotation());
        Eigen::Translation3f translation = tfToEigen(tfTransfo.getOrigin());
        Eigen::Transform<float,3,Eigen::Affine> output = translation*eigenQuat;

        return output.matrix();
    }

    Eigen::Vector3f getRPY(Eigen::Matrix4f transfo) {
        Eigen::Quaternionf eigenQuat = Eigen::Quaternionf(
                Eigen::Matrix3f(transfo.block(0,0,3,3)));

        double roll, pitch, yaw;
        tf::Matrix3x3(eigenToTf(eigenQuat)).getRPY(roll, pitch, yaw);

        return Eigen::Vector3f(roll, pitch, yaw);
    }

    Eigen::Translation3f getTranslation(Eigen::Matrix4f transfo) {
        return Eigen::Translation3f(transfo.block(0,2,3,1));
    }
}
