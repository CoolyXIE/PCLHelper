#pragma once
#include "Headers.h"


Eigen::Matrix4f GetTransformToNewSystem(
	const std::vector<Eigen::Vector3f>& newBasis, Eigen::Vector3f newOrigin);

int GetTranformToInertialSystem(const std::vector<Eigen::Vector3f>& points,
	std::vector<Eigen::Vector3f>& basisVector, Eigen::Matrix4f& T);

inline void ComposeRt(Eigen::Matrix4f& T, Eigen::Matrix3f R, Eigen::Vector3f t)
{
	T = Eigen::Matrix4f::Identity();
	T.topLeftCorner(3, 3) = R;
	T.topRightCorner(3, 1) = t;
};


inline void DecomposeRt(Eigen::Matrix4f T, Eigen::Matrix3f& R, Eigen::Vector3f& t)
{
	R = T.topLeftCorner(3, 3);
	t = T.topRightCorner(3, 1);
};

inline void TransformPoints(const Eigen::Matrix4f& T, std::vector<Eigen::Vector3f>& points)
{
	for (int i = 0; i < points.size(); i++) {
		auto& pt = points[i];
		Eigen::Vector4f ptHomo = T * Eigen::Vector4f(pt.x(), pt.y(), pt.z(), 1);
		pt = ptHomo.topRows(3);
	}
}