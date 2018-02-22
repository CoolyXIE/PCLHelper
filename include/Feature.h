#include "Headers.h"

/// Normals
void computeNormals(CloudNML::Ptr cloud, int k);

void reorientNormals(CloudNML::Ptr cloud, int k);

void flipNormals(CloudNML::Ptr cloud);

/// Centroid
template <typename PtT>
Eigen::Vector3d computeCentroid(const pcl::PointCloud<PtT>& cloud);





