#include "Shape.h"

namespace cooly
{

    //
    //
    //// Line
    //namespace PHL
    //{
    //
    //
    //	cv::Point2f Line::projectPtToLine(cv::Point2f pt)
    //	{
    //		Eigen::Vector3f line(A, B, C);
    //		Eigen::Vector3f vertical(B, -A, A*pt.y - B*pt.x);
    //		Eigen::Vector3f projectedPt = line.cross(vertical);
    //		return cv::Point2f(projectedPt(0) / projectedPt(2), projectedPt(1) / projectedPt(2));
    //	}
    //
    //	Line Line::fittingLineLS(const std::vector<cv::Point2f>& pts)
    //	{
    //		int n = pts.size();
    //		cv::Point2f mean(0, 0);
    //		for (auto pt : pts)
    //			mean += pt;
    //		mean /= n;
    //		float sum_x2 = .0f; // sum of x^2
    //		float sum_xy = .0f;
    //		for (auto pt : pts)
    //		{
    //			sum_x2 += pow(pt.x - mean.x, 2);
    //			sum_xy += (pt.y - mean.y) * (pt.x - mean.x);
    //		}
    //
    //		// y = kx + b
    //
    //		if (sum_x2 == 0)
    //		{
    //			float x0 = mean.x; // line x=x0
    //			return Line(1, 0, -x0);
    //		}
    //		else
    //		{
    //			float k = sum_xy / sum_x2;
    //			float b = mean.y - k * mean.x;
    //			return Line(k, b);
    //		}
    //	}
    //}
    //
    //// Cylinder
    //namespace PHL
    //{
    //	Eigen::Matrix4f Cylinder::getTransformToCylinderSystem() const
    //	{
    //		Plane3D plane(centroid, axis);
    //		return plane.getTransformToPlaneSystem(centroid);
    //	}
    //
    //	bool Cylinder::isPointInside(Eigen::Vector3f point) const
    //	{
    //		Eigen::Vector3f vec = point - centroid;
    //		float projH = fabs(axis.dot(vec));
    //		if (projH > 0.5 *height)
    //			return false;
    //
    //		float projR = sqrt(pow(vec.norm(), 2) - pow(projH, 2));
    //		if (projR > radius)
    //			return false;
    //		return true;
    //	}
    //
    //	std::vector<cv::Point3f> Cylinder::getSampledPoints(int circleSampleNum, int heightSampleNum)
    //	{
    //		Circle circle = Circle(0, 0, radius);
    //		std::vector<cv::Point2f> circlePoints = circle.getAllEdgePoints(circleSampleNum);
    //		float delta = height / heightSampleNum;
    //		std::vector<cv::Point3f> sampledPoints;
    //		for (auto pt2D : circlePoints)
    //		{
    //			for (int i = 0; i < heightSampleNum; i++)
    //				sampledPoints.push_back(cv::Point3f(pt2D.x, pt2D.y, - 0.5 * height + delta * i));
    //		}
    //		Eigen::Matrix4f T = getTransformToCylinderSystem();
    //		transformCvPoints(T.inverse(), sampledPoints);
    //
    //		return sampledPoints;
    //	}
    //
    //}
    //
    //// Circle
    //namespace PHL
    //{
    //	Circle Circle::fitCircleLS(const Contourf& points)
    //	{
    //		Circle box;
    //		box.x = 0.0f;
    //		box.y = 0.0f;
    //		box.r = 0.0f;
    //
    //		if (points.size() < 3)
    //		{
    //			return box;
    //		}
    //
    //		int i = 0;
    //
    //		double X1 = 0;
    //		double Y1 = 0;
    //		double X2 = 0;
    //		double Y2 = 0;
    //		double X3 = 0;
    //		double Y3 = 0;
    //		double X1Y1 = 0;
    //		double X1Y2 = 0;
    //		double X2Y1 = 0;
    //
    //		int Sum = points.size();
    //		for (i = 0; i < Sum; i++)
    //		{
    //			X1 = X1 + points[i].x;
    //			Y1 = Y1 + points[i].y;
    //			X2 = X2 + points[i].x * points[i].x;
    //			Y2 = Y2 + points[i].y * points[i].y;
    //			X3 = X3 + points[i].x * points[i].x * points[i].x;
    //			Y3 = Y3 + points[i].y * points[i].y * points[i].y;
    //
    //			X1Y1 = X1Y1 + points[i].x * points[i].y;
    //			X1Y2 = X1Y2 + points[i].x * points[i].y * points[i].y;
    //			X2Y1 = X2Y1 + points[i].x * points[i].x * points[i].y;
    //		}
    //
    //		double C, D, E, G, H, N;
    //		double a, b, c;
    //		N = points.size();
    //		C = N*X2 - X1*X1;
    //		D = N*X1Y1 - X1*Y1;
    //		E = N*X3 + N*X1Y2 - (X2 + Y2)*X1;
    //		G = N*Y2 - Y1*Y1;
    //		H = N*X2Y1 + N*Y3 - (X2 + Y2)*Y1;
    //		a = (H*D - E*G) / (C*G - D*D);
    //		b = (H*C - E*D) / (D*D - G*C);
    //		c = -(a*X1 + b*Y1 + X2 + Y2) / N;
    //
    //		double A, B, R;
    //		A = a / (-2);
    //		B = b / (-2);
    //		R = sqrt(a*a + b*b - 4 * c) / 2;
    //
    //		box.x = A;
    //		box.y = B;
    //		box.r = R;
    //
    //		return box;
    //	}
    //
    //	vector<Point2f> Circle::getAllEdgePoints(int n)
    //	{
    //		vector<Point2f> points;
    //		float delta = 2 * CV_PI / n;
    //		for (int i = 0; i < n; i++)
    //		{
    //			float theta = i * delta;
    //			points.push_back(Point2f(x + r * cos(theta), y + r * sin(theta)));
    //		}
    //
    //		return points;
    //	}
    //	float Circle::calcTheta(Point2f pt)
    //	{
    //		float cosine = (pt.x - x) / sqrt(pow(pt.x - x, 2) + pow(pt.y - y, 2));
    //		float theta = acos(cosine); // [0, PI]
    //		if (pt.y < y)
    //			theta = CV_PI * 2 - theta;
    //		return theta;
    //	}
    //
    //}

}


