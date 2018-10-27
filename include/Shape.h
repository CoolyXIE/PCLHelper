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
//	// centre1: Բ������Բ��
//	// centre2: Բ������Բ��
//	// radius: Բ���뾶
//	Cylinder(Eigen::Vector3f centre1, Eigen::Vector3f centre2, float radius);
//
//	// centroid: Բ������
//	// axis: Բ����������
//	// radius: Բ���뾶
//	// height: Բ���߶�
//	Cylinder(Eigen::Vector3f centroid, Eigen::Vector3f axis, float radius, float height)
//		: centroid(centroid), axis(axis.normalized()), radius(radius), height(height) {};
//
//	~Cylinder() {};
//
//	// �Ӹ��������зָ��Բ�����ڲ��ĵ�
//	// cloud: ��������
//	// indices: Բ�����ڲ��ĵ����
//	// @return: ��Բ�����ڲ��ĵ��� 
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
//	// �ж�һ�����Ƿ���Բ������
//	bool isPointInside(Eigen::Vector3f point) const;
//
//	// ����pcl::ModelCoefficients��ʽ�����ݽṹ
//	pcl::ModelCoefficients::Ptr getModelCoefficients() const;
//
//	Eigen::Matrix4f getTransformToCylinderSystem() const;
//
//	//std::vector<cv::Point3f> getSampledPoints(int circleSampleNum, int heightSampleNum);
//
//	/* Static Function */
//
//	// ���ݵ������һ��Բ����, ʹ�õ���PCL�Դ��Ļ���RANSAC���㷨
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
//	// ���ݷ�������ƽ����һ��ȷ��ƽ��
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
//	// ��ȡ��ƽ������ϵ�ı任����ƽ������ϵ����Ϊxyƽ����ƽ����, z�ᳯf(x)>0�ĵ�ķ���
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
//	// �����ȡƽ���ϵ�n����
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
///* 2D Shape	��ά��״
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
//	/* ��Ա���� */
//	double x;
//	double y;
//	double r;
//
//	Circle(double x = 0.0f, double y = 0.0f, double r = 0.0f) :x(x), y(y), r(r) {};
//
//public:
//	/* ��̬���� */
//	static Circle fitCircleLS(const Contourd& points);
//
//	static Circle fitCircleRANSAC(const Contourd& points);
//
////	// ����Բ������Ͱ뾶ȷ��һ��Բ������������ļ���, pt0�����ж�Բ������������ֱ�ߵ���һ��
////	static Circle determinCircle(Point2f pt0, Point2f pt1, Point2f pt2, float r, float &theta1, float &theta2);
////
////	static std::vector<Circle> smoothCircles(Circle startCircle, Circle endCircle, int n);
////
////public:
////	/* get���� */
////	cv::Point2f getCenter() { return Point2f(x, y); }
////
////	// ���㼫��: ����һ���㣬ȷ��theta�ǣ�����㲻һ����Բ��
////	float calcTheta(Point2f pt);
////
////	// ����Theta�ǣ����ظ�Theta�Ƕ�Ӧ��Բ��һ��
////	Point2f getEdgePoint2f(float theta);
////
////	// ����Բ
////	vector<Point2f> getAllEdgePoints(int n);
////
////	// ����Բ��: thetaBeg���Դ���thetaEnd, theta��Χ������[0, 2*PI)
////	vector<Point2f> samplingArcNum(float thetaBeg, float thetaEnd, int n);
////
////	vector<Point2f> samplingArcDelta(float thetaBeg, float thetaEnd, float delta);
////
////	// ��Բ��
////	bool isInCircle(Point2f pt)
////	{
////		return (pow(pt.x - x, 2) + pow(pt.y - y, 2) < pow(r, 2));
////	};
////
////	// ��Բ��
////	bool isOnCircle(Point2f pt);
////
////	// ��Բ��
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
