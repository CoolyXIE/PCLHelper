#pragma once
#include "Headers.h"

namespace cooly
{

    std::vector<Eigen::Vector3f> PlaneCutMesh(const Mesh& mesh, Eigen::Vector4f Plane);

    // Convex Hull
    Mesh ConvexHull(CloudXYZ::Ptr cloud);

    Mesh PoissonReconstruct(CloudNML::Ptr cloud);

    // Poisson Reconstruct With Colors
    Mesh PoissonReconstruct(CloudALL::Ptr cloud);

    Mesh MarchingCubeReconstruct(CloudNML::Ptr cloud);

    Mesh GreedyProjectionTriangulation(CloudNML::Ptr cloud):

}
