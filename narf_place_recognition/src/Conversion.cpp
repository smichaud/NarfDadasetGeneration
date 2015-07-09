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

    Eigen::Vector3f tfToEigen(tf::Vector3 tfTranslation) {
        return Eigen::Vector3f(
                tfTranslation.x(), tfTranslation.y(), tfTranslation.z());
    }

    tf::Vector3 eigenToTf(Eigen::Vector3f eigenTranslation) {
        return tf::Vector3(eigenTranslation.x(), eigenTranslation.y(),
                eigenTranslation.z());
    }

    Eigen::Matrix4f matrixFromTf(
            tf::Transform tfTransfo) {
        Eigen::Quaternionf eigenQuat = tfToEigen(tfTransfo.getRotation());
        Eigen::Vector3f eigenTranslation = tfToEigen(tfTransfo.getOrigin());

        //Eigen::Transform<float,3,Eigen::Affine> rotation(eigenQuat);
        //Eigen::Transform<float,3,Eigen::Affine> translation(eigenTranslation);
        //Eigen::Transform<float,3,Eigen::Affine> transform =
            //rotation*translation;

        return Eigen::Matrix4f::Identity();
    }
}
