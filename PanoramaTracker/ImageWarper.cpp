#include "ImageWarper.h"

ImageWarper::ImageWarper() : warper(570)
{

}

ImageWarper::ImageWarper(float warper_scale, bool use_android) : warper(warper_scale)
{
	Mat_<float> cameraParameters(3, 3);
	Mat_<float> distCoeffs(1, 5);
	//Logitech C920 calibration from video
	if (!use_android){
		//cameraParameters(0, 0) = 6.2989003437882866e+002;
		//cameraParameters(0, 1) = 0;
		//cameraParameters(0, 2) = 3.1950000000000000e+002;
		//cameraParameters(1, 0) = 0;
		//cameraParameters(1, 1) = 6.2989003437882866e+002;
		//cameraParameters(1, 2) = 2.3950000000000000e+002;
		//cameraParameters(2, 0) = 0;
		//cameraParameters(2, 1) = 0;
		//cameraParameters(2, 2) = 1;
		cameraParameters(0, 0) = 6.2311888018131981e+002;
		cameraParameters(0, 1) = 0;
		cameraParameters(0, 2) = 3.1950000000000000e+002;
		cameraParameters(1, 0) = 0;
		cameraParameters(1, 1) = 6.2311888018131981e+002;
		cameraParameters(1, 2) = 2.3950000000000000e+002;
		cameraParameters(2, 0) = 0;
		cameraParameters(2, 1) = 0;
		cameraParameters(2, 2) = 1;
		distCoeffs(0, 0) = 1.0190374492489046e-001;
		distCoeffs(0, 1) = -2.1360461522279034e-001;
		distCoeffs(0, 2) = 0;
		distCoeffs(0, 3) = 0;
		distCoeffs(0, 4) = -6.9738993935299143e-002;

	}
	else
	{
		//NVidia Shield calibration from images
		cameraParameters(0, 0) = 6.8655359317770069e+002;
		cameraParameters(0, 1) = 0;
		cameraParameters(0, 2) = 3.1950000000000000e+002;
		cameraParameters(1, 0) = 0;
		cameraParameters(1, 1) = 6.8655359317770069e+002;
		cameraParameters(1, 2) = 2.3950000000000000e+002;
		cameraParameters(2, 0) = 0;
		cameraParameters(2, 1) = 0;
		cameraParameters(2, 2) = 1;
	}
	dist_coeffs = distCoeffs;
	K = cameraParameters;
}

Mat ImageWarper::getMask(Mat warpedImage)
{
	Mat dest, eroded;
	threshold(warpedImage, dest, 0, 250, CV_THRESH_BINARY);
	if (interpolation_method_ == INTER_NEAREST){
		return dest;
	}

	//In order to help the erode function, ensure an edge around the image
	//Without eroding, some interpolation methods cause the mask to function wrongly
	//Top edge
	else{
		for (int i = 0; i < dest.cols; i++)
		{
			dest.at<char>(Point(i, 0)) = 0;
			dest.at<char>(Point(i, dest.rows - 1)) = 0;
		}
		for (int j = 0; j < dest.rows; j++)
		{
			dest.at<char>(Point(0, j)) = 0;
			dest.at<char>(Point(dest.cols - 1, j)) = 0;
		}

		Mat element = getStructuringElement(MORPH_ERODE,
			Size(2 * 4 + 1, 2 * 4 + 1),
			Point(4, 4));
		erode(dest, eroded, element);
		return eroded;
	}
}


Mat ImageWarper::warpImageCylindrical(Mat image, float yaw, float pitch, float roll, Mat& result)
{
	Mat_<float> R = getRotationMatrix(yaw, pitch, roll);
	warper.warp(image, K, R, interpolation_method_, BORDER_CONSTANT, result);
	Mat mask = getMask(result);
	return mask;
}

Mat_<float> ImageWarper::getRotationMatrix(float yaw, float pitch, float roll)
{
	float yawRad = degreesToRadians(yaw);
	float pitchRad = degreesToRadians(pitch);
	float rollRad = degreesToRadians(roll);
	Mat_<float> rotationMatrix(3, 3);
	Mat_<float> rotX(3, 3);
	Mat_<float> rotY(3, 3);
	Mat_<float> rotZ(3, 3);
	rotX(0, 0) = 1;
	rotX(0, 1) = 0;
	rotX(0, 2) = 0;
	rotX(1, 0) = 0;
	rotX(1, 1) = cos(yawRad);
	rotX(1, 2) = -sin(yawRad);
	rotX(2, 0) = 0;
	rotX(2, 1) = sin(yawRad);
	rotX(2, 2) = cos(yawRad);

	rotY(0, 0) = cos(pitchRad);
	rotY(0, 1) = 0;
	rotY(0, 2) = sin(pitchRad);
	rotY(1, 0) = 0;
	rotY(1, 1) = 1;
	rotY(1, 2) = 0;
	rotY(2, 0) = -sin(pitchRad);
	rotY(2, 1) = 0;
	rotY(2, 2) = cos(pitchRad);

	rotZ(0, 0) = cos(rollRad);
	rotZ(0, 1) = -sin(rollRad);
	rotZ(0, 2) = 0;
	rotZ(1, 0) = sin(rollRad);
	rotZ(1, 1) = cos(rollRad);
	rotZ(1, 2) = 0;
	rotZ(2, 0) = 0;
	rotZ(2, 1) = 0;
	rotZ(2, 2) = 1;

	rotationMatrix = rotX * rotY * rotZ;

	return rotationMatrix;
}

Mat ImageWarper::undistortImage(Mat image)
{
	Mat undistorted;
	undistort(image, undistorted, K, dist_coeffs);
	return undistorted;
}

float ImageWarper::radiansToDegrees(float radians)
{
	return radians * (180 / M_PI);
}
float ImageWarper::degreesToRadians(float degrees)
{
	return degrees * (M_PI / 180);
}
