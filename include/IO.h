#include "Headers.h"


namespace cooly
{


    // Note: FolderPath endwith slash while FilePath not
    class PathHelper {
        static std::string GetFolderPath(std::string filePath);

        static std::string GetFileName(std::string filePath);

        static std::string GetFileExt(std::string filePath/*or fileName*/);

        static std::vector<std::string> GetAllFilePathInFolder(std::string folderPath, std::string ext = "*");

        static bool CheckAndCreateFolder(std::string folderPath);

    };

    // Read & Write  Cloud / Mesh
    template <typename PtT>
    pcl::PointCloud<PtT> ReadCloud(const std::string& cloudPath);

    template <typename PtT>
    pcl::PointCloud<PtT> ReadMultiCloud(const std::vector<std::string>& multiCloudPath);

    pcl::PolygonMesh ReadMesh(const std::string& meshPath);

    pcl::PolygonMesh ReadMultiMesh(const std::vector<std::string>& multiMeshPath);

    bool WritePoints(const std::vector<Eigen::Vector3d>& points, const std::string& cloudPLY);


    /// Convert
    CloudXYZ::Ptr ConvertPoints(const std::vector<Eigen::Vector3d>& points);

    template <typename PtT>
    std::vector<Eigen::Vector3d> ConvertPoints(const pcl::PointCloud<PtT>& cloud);

    template <typename PtT>
    pcl::PointCloud<PtT> ReplacePoints(const std::vector<Eigen::Vector3d>& points);





}



