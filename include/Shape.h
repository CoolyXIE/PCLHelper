//#pragma once
//#include "Headers.h"
//
///************************************************************************/
///* 3D Shape
///************************************************************************/
//struct Sphere
//{
//	// Constructors
//	Sphere() :radius_(1.0), center_(Eigen::Vector3d(0, 0, 0)) {};
//	Sphere(double radius, Eigen::Vector3d center) :
//		radius_(radius), center_(center) {};
//
//	// Functions
//	bool isInside(Eigen::Vector3d point);
//
//	// Members
//	double radius_;
//	Eigen::Vector3d center_;
//};
//
//struct Cylinder
//{
//	/* Constructors */
//
//	// centre1: 圆柱顶面圆心
//	// centre2: 圆柱底面圆心
//	// radius: 圆柱半径
//	Cylinder(Eigen::Vector3f centre1, Eigen::Vector3f centre2, float radius);
//
//	// centroid: 圆柱中心
//	// axis: 圆柱中轴向量
//	// radius: 圆柱半径
//	// height: 圆柱高度
//	Cylinder(Eigen::Vector3f centroid, Eigen::Vector3f axis, float radius, float height)
//		: centroid(centroid), axis(axis.normalized()), radius(radius), height(height) {};
//
//	~Cylinder() {};
//
//	// 从给定点云中分割出圆柱体内部的点
//	// cloud: 给定点云
//	// indices: 圆柱体内部的点序号
//	// @return: 在圆柱体内部的点数 
//	template <typename PointT>
//	int segmentCloud(const pcl::PointCloud<PointT>& cloud, pcl::PointIndices::Ptr indices)
//	{
//		indices->indices.clear();
//		for (int i = 0; i < cloud.points.size(); i++)
//		{
//			PointT pt = cloud.points[i];
//			Eigen::Vector3f point(pt.x, pt.y, pt.z);
//			if (isPointInside(point))
//				indices->indices.push_back(i);
//		}
//		return indices->indices.size();
//	};
//
//	// 判断一个点是否在圆柱体内
//	bool isPointInside(Eigen::Vector3f point) const;
//
//	// 返回pcl::ModelCoefficients格式的数据结构
//	pcl::ModelCoefficients::Ptr getModelCoefficients() const;
//
//	Eigen::Matrix4f getTransformToCylinderSystem() const;
//
//	//std::vector<cv::Point3f> getSampledPoints(int circleSampleNum, int heightSampleNum);
//
//	/* Static Function */
//
//	// 根据点云拟合一个圆柱体, 使用的是PCL自带的基于RANSAC的算法
//	static Cylinder fitCylinder(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, Eigen::Matrix4f& transformToCylinerSystem);
//
//
//	/* Members */
//
//	float radius;
//	Eigen::Vector3f centroid;
//	Eigen::Vector3f axis;
//	float height;
//};
//
//struct Plane3D
//{
//	/* Constructor */
//
//	// [A, B, C, D] for plane equation Ax+By+Cz+D=0
//	Plane3D(float A, float B, float C, float D) : A(A), B(B), C(C), D(D) {};
//
//	Plane3D(Eigen::Vector4f plane_coeff) :A(plane_coeff(0)), B(plane_coeff(1)), C(plane_coeff(2)), D(plane_coeff(3)) {};
//
//	// 根据法向量和平面上一点确定平面
//	Plane3D(Eigen::Vector3f pt, Eigen::Vector3f normal)
//		:A(normal.x()), B(normal.y()), C(normal.z()), D(-pt.dot(normal)) {};
//
//	Plane3D(pcl::ModelCoefficients::Ptr planeCoeff);
//
//	//Plane3D(cv::Point3f pt1, cv::Point3f pt2, cv::Point3f pt3);
//
//
//
//
//	/* Util function */
//	// 获取到平面坐标系的变换矩阵，平面坐标系定义为xy平面在平面上, z轴朝f(x)>0的点的方向
//	Eigen::Matrix4f getTransformToPlaneSystem(Eigen::Vector3f O)
//	{
//		Eigen::Vector3f pt = createRandPointsOnPlane(1)[0];
//		Eigen::Vector3f axisX = (pt - O).normalized();
//		Eigen::Vector3f axisZ = Eigen::Vector3f(A, B, C).normalized();
//		Eigen::Vector3f axisY = axisZ.cross(axisX);
//		std::vector<Eigen::Vector3f> newBasis = { axisX, axisY, axisZ };
//
//		return getTransformToNewSystem(newBasis, O);
//	};
//
//	// 随机获取平面上的n个点
//	std::vector<Eigen::Vector3f> createRandPointsOnPlane(int n)
//	{
//		std::vector<Eigen::Vector3f> randPts;
//		if (A != 0 && B != 0 && C != 0)
//		{
//			while (randPts.size() != n)
//			{
//				Eigen::Vector3f randPt = Eigen::Vector3f::Random();
//				randPt(2) = -(D + A * randPt(0) + B * randPt(1)) / C;
//				randPts.push_back(randPt);
//
//				//TO-DO: check same point existence
//			}
//		}
//		else
//		{
//			if (A == 0 && B == 0)
//			{
//				float z0 = -D;
//				while (randPts.size() != n)
//				{
//					Eigen::Vector2f randPt2D = Eigen::Vector2f::Random();
//					randPts.push_back(Eigen::Vector3f(randPt2D.x(), randPt2D.y(), z0));
//					//TO-DO: check same point existence
//				}
//			}
//			else if (A == 0 && C == 0)
//			{
//				float y0 = -D;
//				while (randPts.size() != n)
//				{
//					Eigen::Vector2f randPt2D = Eigen::Vector2f::Random();
//					randPts.push_back(Eigen::Vector3f(randPt2D.x(), y0, randPt2D.y()));
//					//TO-DO: check same point existence
//				}
//			}
//			else if (B == 0 && C == 0)
//			{
//				float x0 = -D;
//				while (randPts.size() != n)
//				{
//					Eigen::Vector2f randPt2D = Eigen::Vector2f::Random();
//					randPts.push_back(Eigen::Vector3f(x0, randPt2D.x(), randPt2D.y()));
//					//TO-DO: check same point existence
//				}
//			}
//			// TO-DO: consider this situation
//		}
//		return randPts;
//	};
//
//	/* Member */
//	float A;
//	float B;
//	float C;
//	float D;
//
//
//// 		/* Static Fitting */
//// 		static Plane3D fitPlane3DRANSAC(const std::vector<)
//
//};
//	
///************************************************************************/
///* 2D Shape	二维形状
///************************************************************************/
//
//typedef Eigen::Vector2d Pt2D;
//typedef std::vector<Pt2D> Contourd;
//
//
//struct Circle
//{
//
//public:
//	/* 成员变量 */
//	double x;
//	double y;
//	double r;
//
//	Circle(double x = 0.0f, double y = 0.0f, double r = 0.0f) :x(x), y(y), r(r) {};
//
//public:
//	/* 静态函数 */
//	static Circle fitCircleLS(const Contourd& points);
//
//	static Circle fitCircleRANSAC(const Contourd& points);
//
////	// 根据圆上两点和半径确定一个圆，并返回两点的极角, pt0用于判断圆心在两点所在直线的哪一侧
////	static Circle determinCircle(Point2f pt0, Point2f pt1, Point2f pt2, float r, float &theta1, float &theta2);
////
////	static std::vector<Circle> smoothCircles(Circle startCircle, Circle endCircle, int n);
////
////public:
////	/* get函数 */
////	cv::Point2f getCenter() { return Point2f(x, y); }
////
////	// 计算极角: 给定一个点，确定theta角，这个点不一定在圆上
////	float calcTheta(Point2f pt);
////
////	// 给定Theta角，返回该Theta角对应的圆上一点
////	Point2f getEdgePoint2f(float theta);
////
////	// 采样圆
////	vector<Point2f> getAllEdgePoints(int n);
////
////	// 采样圆弧: thetaBeg可以大于thetaEnd, theta范围必须是[0, 2*PI)
////	vector<Point2f> samplingArcNum(float thetaBeg, float thetaEnd, int n);
////
////	vector<Point2f> samplingArcDelta(float thetaBeg, float thetaEnd, float delta);
////
////	// 在圆内
////	bool isInCircle(Point2f pt)
////	{
////		return (pow(pt.x - x, 2) + pow(pt.y - y, 2) < pow(r, 2));
////	};
////
////	// 在圆上
////	bool isOnCircle(Point2f pt);
////
////	// 在圆外
////	bool isOutCircle(Point2f pt);
//
//
//};
//
//struct Line
//{
//	// Constructors
//	Line(float A, float B, float C) :A(A), B(B), C(C) {};
//	Line(float k, float b) :A(k), B(-1), C(b) {};
//	Line(Pt2D pt1, Pt2D pt2);
//
//	// Functions
//	int intersect(Line line, Pt2D& cross);
//
//	Pt2D projectPtToLine(Pt2D pt);
//	
//	// Static
//	static Line fittingLineLS(const std::vector<Pt2D>& pts);
//
//	static Line fittingLineRANSAC(const std::vector<Pt2D>& pts);
//
//
//	/* Members */
//	float A;
//	float B;
//	float C;
//};
//
//
//
//
