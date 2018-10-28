#include "Headers.h"
#include <pcl/search/kdtree.h>

namespace cooly
{
    /// Normals
    pcl::PointCloud<pcl::Normal>::Ptr
    computeNormals(CloudXYZ::Ptr cloud, int k);

    // Reorient normals to keep consistency
    void reorientNormals(CloudNML::Ptr cloud, int k);

    void flipNormals(CloudNML::Ptr cloud);

    /// Centroid
    template <typename PtT>
    Eigen::Vector3d computeCentroid(const pcl::PointCloud<PtT>& cloud);

    /// Curvatures

    /// PFH & FPFH

    /// NRAF

    /// SHOT

    /// ShapeContext

    class DescriptorMatcher {

    };

}


