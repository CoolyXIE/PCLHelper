#pragma once

// STD
#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointXYZ			PtXYZ;
typedef pcl::PointXYZRGB		PtRGB;
typedef pcl::PointNormal		PtNML;
typedef pcl::PointXYZRGBNormal	PtALL;

typedef pcl::PointCloud<PtXYZ>	CloudXYZ;
typedef pcl::PointCloud<PtRGB>	CloudRGB;
typedef pcl::PointCloud<PtNML>	CloudNML;
typedef pcl::PointCloud<PtALL>	CloudALL;

typedef pcl::PolygonMesh		Mesh;

enum PtType {XYZ, RGB, NORMAL, ALL};
enum Axis {X, Y, Z};
