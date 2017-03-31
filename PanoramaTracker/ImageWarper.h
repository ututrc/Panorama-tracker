#pragma once

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <chrono>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace cv;
/*
Class to handle the warping of the images to fit a cylindrical or spherical panorama image
*/
class ImageWarper
{
private:

	detail::CylindricalWarper warper;
	int interpolation_method_ = INTER_NEAREST;

	Mat_<float> K;
	Mat_<float> dist_coeffs;
	float radiansToDegrees(float radians);
	float degreesToRadians(float degrees);
	//Get a binary mask of the warped image to determine in which pixels actual image exists,
	//and in which pixels there is no data
	Mat getMask(Mat warpedImage);
	Mat_<float> getRotationMatrix(float yaw, float pitch, float roll);
	Mat undistortImage(Mat image);

public:
	ImageWarper();
	ImageWarper(float warper_scale, bool use_android);

	//Warp image and return a binary mask_current_view_ 
	Mat warpImageCylindrical(Mat image, float yaw, float pitch, float roll, Mat &result);
};