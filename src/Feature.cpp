#include "Feature.h"
#include <pcl/features/normal_3d.h>

using namespace pcl;

namespace cooly
{
    PointCloud<Normal>::Ptr computeNormals(CloudXYZ::Ptr cloud, int k)
    {
        NormalEstimation<PtXYZ, Normal> ne;
        ne.setInputCloud(cloud);
        search::KdTree<PtXYZ>::Ptr tree(new search::KdTree<PtXYZ>);
        ne.setSearchMethod(tree);
        ne.setKSearch(k);

        PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
        ne.compute(*normals);

        return normals;
    }

}
