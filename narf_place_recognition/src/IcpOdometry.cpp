#include "IcpOdometry.hpp"
#include <fstream>

namespace icpodometry {
    Transformation getTransfo(
            const PointCloud &startCloud,
            const PointCloud &endCloud,
            Transformation initTransfo,
            const std::string &configFile) {
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
            rigidTrans->compute(startCloud, approxTransfo);
        PM::TransformationParameters errorTransfo =
            icp(initializedCloud, endCloud);

        // [TODO]: Remove these lines when calibrated - 2015-07-07 01:46pm
        std::cout << "match ratio: "
            << icp.errorMinimizer->getWeightedPointUsedRatio()
            << std::endl;

        PointCloud adjustedCloud(initializedCloud);
        icp.transformations.apply(adjustedCloud, errorTransfo);

        std::string outputBaseFile = "/home/smichaud/Desktop/test";
        endCloud.save(outputBaseFile + "_ref.vtk");
        startCloud.save(outputBaseFile + "_data_in.vtk");
        adjustedCloud.save(outputBaseFile + "_data_out.vtk");

        return errorTransfo*approxTransfo;
    }
}
