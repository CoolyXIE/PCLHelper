#pragma once
#include "Headers.h"

namespace cooly
{
    class PinholeCamera {
    public:
        PinholeCamera() {};
        PinholeCamera(double fx, double fy, double cx, double cy, int width, int height);
        ~PinholeCamera(){};

    public:
        Eigen::Matrix3d getIntrinsicMatrix() const {
            Eigen::Matrix3d K;
            K << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return K;
        };

        Eigen::MatrixXd getDistortionMatrix() const;

        bool isValid() const;

        Eigen::Vector3d ProjectToSpace(const Eigen::Vector2d& imagePt, double depth) const
        {
            Eigen::Vector3d spacePt;
            spacePt.x() = (imagePt.x() - cx_) / fx_ * depth;
            spacePt.y() = (imagePt.y() - cy_) / fy_ * depth;
            spacePt.z() = depth;
            return spacePt;
        }

        Eigen::Vector2d ProjectToImage(const Eigen::Vector3d& spacePt) const
        {
            Eigen::Vector2d imagePt;
            imagePt.x() = spacePt.x() / spacePt.z() * fx_ + cx_;
            imagePt.y() = spacePt.y() / spacePt.z() * fy_ + cy_;
            return imagePt;
        }

        Eigen::Vector2d ProjectToImageWithDistortion(const Eigen::Vector3d& spacePt) const
        {
            double x = spacePt.x() / spacePt.z();
            double y = spacePt.y() / spacePt.z();
            double r2 = x*x + y*y;
            double radial = 1.0 + distort_[0] * r2
                + distort_[1] * r2 * r2
                + distort_[4] * r2 * r2 * r2;

            double tangential1, tangential2;
            tangential1 = 2.0 * distort_[2] * x * y + distort_[3] * (r2 + 2.0 * x * x);
            tangential2 = distort_[2] * (r2 + 2.0 * y * y) + 2.0 * distort_[3] * x * y;

            Eigen::Vector2d imagePt;
            imagePt.x() = (radial * x + tangential1) * fx_ + cx_;
            imagePt.y() = (radial * y + tangential2) * fy_ + cy_;
            return imagePt;
        }

    // 	void UndistortPoints(std::vector<cv::Point2f>& InputPts, std::vector<cv::Point2f>& OutputPts) const
    // 	{
    // 		if (InputPts.empty()) return;
    // 		if (distortion_parameters_[0] * distortion_parameters_[0] +
    // 			distortion_parameters_[1] * distortion_parameters_[1] +
    // 			distortion_parameters_[2] * distortion_parameters_[2] +
    // 			distortion_parameters_[3] * distortion_parameters_[3] +
    // 			distortion_parameters_[4] * distortion_parameters_[4] > 1e-10)
    // 		{
    // 			cv::Mat InputMat(InputPts.size(), 1, CV_64FC2);
    // 			for (size_t i = 0; i < InputPts.size(); i++)
    // 			{
    // 				InputMat.at<cv::Point2d>(i, 0).x = InputPts[i].x;
    // 				InputMat.at<cv::Point2d>(i, 0).y = InputPts[i].y;
    // 			}
    // 			cv::Mat K = cv::Mat::zeros(3, 3, CV_64FC1);
    // 			K.at<double>(0, 0) = fx_;
    // 			K.at<double>(1, 1) = fy_;
    // 			K.at<double>(2, 2) = 1.0;
    // 			K.at<double>(0, 2) = cx_;
    // 			K.at<double>(1, 2) = cy_;
    // 			cv::Mat distCoeffs;
    // 			if (std::abs(distortion_parameters_[4]) > 1e-10)
    // 			{
    // 				distCoeffs.create(1, 5, CV_64FC1);
    // 				for (int i = 0; i < 5; i++)
    // 					distCoeffs.at<double>(0, i) = distortion_parameters_[i];
    // 			}
    // 			else
    // 			{
    // 				distCoeffs.create(1, 4, CV_64FC1);
    // 				for (int i = 0; i < 4; i++)
    // 					distCoeffs.at<double>(0, i) = distortion_parameters_[i];
    // 			}
    // 			cv::Mat OutputMat(InputPts.size(), 1, CV_64FC2);
    //
    // 			cv::undistortPoints(InputMat, OutputMat, K, distCoeffs);
    // 			OutputPts.resize(InputPts.size());
    // 			for (size_t i = 0; i < OutputPts.size(); i++)
    // 			{
    // 				OutputPts[i].x = OutputMat.at<cv::Point2d>(i, 0).x * fx_ + cx_;
    // 				OutputPts[i].y = OutputMat.at<cv::Point2d>(i, 0).y * fy_ + cy_;
    // 			}
    // 		}
    // 		else
    // 		{
    // 			// do nothing
    // 			OutputPts = InputPts;
    // 		}
    // 	}

    private:
        double fx_, fy_, cx_, cy_;
        int width_, height_;

        // radial: [0,1,4], tangential: [2,3]
        double distort_[5];

    };

    class Image {
    public:
        Image(int width, int height) : width_(width), height_(height) {
            data = new unsigned char[width * height * 3];
        };

        int width_;
        int height_;
        unsigned char* data;
    };

    class DepthImg {
    public:
        DepthImg(int width, int height) : width_(width), height_(height) {
            data = new float[width * height];
        };

        static DepthImg Read(std::string depthImgPath);

        int width_;
        int height_;
        float* data;
    };


    Image RenderImageFromColorCloud(CloudRGB::Ptr colorCloud,
        PinholeCamera camera, Eigen::Matrix4d pose, int width, int height);

    // Image RenderShadingImageFromCloud(CloudALL::Ptr cloud,
    // 	PinholeCamera camera, Eigen::Matrix4d pose, int width, int height,
    // 	Image& depthImg = Image(), Image& mask = Image());

}
