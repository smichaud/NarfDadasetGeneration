#include "IcpOdometry.hpp"
#include <fstream>

namespace IcpOdometry {
    //Note: Cloud origin == sensor. Therefore endCloud must be transformed :)
    Eigen::Matrix4f getCorrectedTransfo(
            const PointCloud &startCloud,
            const PointCloud &endCloud,
            Eigen::Matrix4f initTransfo,
            const std::string &configFile,
            const std::string &cloudsOutputPath) {
        PM::ICP icp;

        std::ifstream configFileStream(configFile.c_str());
        if (configFile.empty() || !configFileStream.good()) {
            std::cerr << "Unable to load ICP configuration file." << std::endl;
            std::cerr << "Default setting will be used." << std::endl;
            icp.setDefault();
        } else {
            icp.loadFromYaml(configFileStream);
        }

        PM::TransformationParameters approxTransfo = initTransfo;
        PM::Transformation* rigidTrans;
        rigidTrans = PM::get().REG(Transformation).
            create("RigidTransformation");

        const PointCloud initializedCloud =
            rigidTrans->compute(endCloud, approxTransfo);
        PM::TransformationParameters errorTransfo =
            icp(initializedCloud, startCloud);

        if(!cloudsOutputPath.empty()) {
            PointCloud adjustedCloud(initializedCloud);
            icp.transformations.apply(adjustedCloud, errorTransfo);

            startCloud.save(cloudsOutputPath + "_start_cloud.vtk");
            endCloud.save(cloudsOutputPath + "_end_cloud_ref.vtk");
            initializedCloud.save(cloudsOutputPath + "_init_cloud.vtk");
            adjustedCloud.save(cloudsOutputPath + "_final_cloud.vtk");
        }

        return errorTransfo*approxTransfo;
    }
}
