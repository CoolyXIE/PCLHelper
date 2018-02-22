#pragma once
#include "Headers.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

/// Extract Indices of PointCloud
typedef pcl::BoundingBoxXYZ Box;

template <typename PtT>
pcl::PointCloud<PtT> RemovePoints(pcl::PointCloud<PtT>& cloud, const std::vector<int>& indices, bool inplace = true)
{
	pcl::PointIndices::Ptr outliers(new pcl::PointIndices);
	outliers->indices = indices;

	pcl::ExtractIndices<PtT> extractor;
	extractor.setInputCloud(cloud.makeShared());
	extractor.setIndices(outliers);
	extractor.setNegative(true);

	pcl::PointCloud<PtT> outCloud;
 	if (inplace)
 		extractor.filter(cloud);
 	else
 		extractor.filter(outCloud);

	return outCloud;
}

template <typename PtT>
pcl::PointCloud<PtT> RetainPoints(pcl::PointCloud<PtT>& cloud, const std::vector<int>& indices, bool inplace = true)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	inliers->indices = indices;

	pcl::ExtractIndices<PtT> extractor;
	extractor.setInputCloud(cloud.makeShared());
	extractor.setIndices(inliers);
	extractor.setNegative(false);

	pcl::PointCloud<PtT> outCloud;
	if (inplace)
		extractor.filter(cloud);
	else
		extractor.filter(outCloud);

	return outCloud;
}



/// Denoise PointCloud

void RadiusFilter();

void StatisticalRadiusFilter();

void ClusterFilter(CloudXYZ::Ptr cloud, std::vector<std::vector<int>>& clusterIndices,
	double dist, int minSize = 1, int maxSize = INT_MAX);

std::vector<int> NormalFilter(CloudNML::Ptr cloud, Eigen::Vector3f direction, double cosineThresh);

// radius: kdtree radius
// n: consider isolate normal only when neighbor points size is no less than n, else keep it
// ratio: if similar direction neighbors size proportion larger than ratio, keep this point.
// angle: similar normals = angle between normals of two points less than angle
// @return: outliers' indices
std::vector<int> IsolateNormalFilter(CloudNML::Ptr cloud, double radius, int n, double ratio, double angle);

// TODO: LOP(Local Projection Operators)

/// Sampling PointCloud
template <typename PtT>
void DownSampleVoxelGrid(pcl::PointCloud<PtT>& cloud, Eigen::Vector3f leafSize)
{
	pcl::VoxelGrid<PtT> filter;
	filter.setInputCloud(cloud.makeShared());
	filter.setLeafSize(leafSize.x(), leafSize.y(), leafSize.z());
	filter.filter(cloud);
}

template <typename PtT>
pcl::PointCloud<PtT> DownSampleRandom(pcl::PointCloud<PtT>& cloud, int n)
{
	std::vector<int> indices;
	for (size_t i = 0; i < cloud.points.size(); i++)
		indices.push_back(i);
	std::srand(std::time(0));
	std::random_shuffle(indices.begin(), indices.end());

	pcl::PointCloud<PtT> cloud_sample;
	for (size_t i = 0; i < n; i++) 
		cloud_sample.points.push_back(cloud.points[indices[i]]);
	return cloud_sample;
}

void DownSampleRadius(CloudXYZ::Ptr cloud, double radius);

void DownSampleNormalSpace(CloudNML::Ptr cloud, double ratio);

/// Smooth PointCloud


/// Segment PointCloud
std::vector<pcl::PointIndices> SegmentCloudToCluster(CloudXYZ::Ptr cloud,
	float dist, int minSize = 1, int maxSize = INT_MAX);

template <typename PtT>
int SegmentCloudByPlane(pcl::PointCloud<PtT>& cloud, Eigen::Vector4f plane,
	std::vector<int>& aboveIndices, std::vector<int>& belowIndices);


template <typename PtT>
std::vector<std::vector<int>> SegmentCloudToSlices(
	const pcl::PointCloud<PtT>& cloud, double slice_thickness, Axis axis);

template <typename PtT>
Box getBoundingBox(const pcl::PointCloud<PtT>& cloud);
	
	