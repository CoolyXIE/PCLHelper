#pragma once
#include "Headers.h"

// TODO: Access Correspondences
bool AlignICP(CloudXYZ::Ptr sourceCloud, CloudXYZ::Ptr targetCloud, Eigen::Matrix4d& T,
	double maxCorrDist, int maxIter = 100, double fitnessEps=1e-6, double transformEps=1e-10);

bool AlignICPwithNormal(CloudNML::Ptr sourceCloud, CloudNML::Ptr targetCloud, Eigen::Matrix4d& T,
	double maxCorrDist, int maxIter = 100, double fitnessEps = 1e-6, double transformEps = 1e-10);

bool AlignICPWithColor();

class NonRigidICP {

};

