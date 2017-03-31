#pragma once

#include <vector>
#include <opencv/cv.h>

#include <list>

namespace HelpFunctions
{
	//Filter out movement/rotation points that deviate too far from the median movement value
	void filterMovementVectors(const std::vector<float> &x_move_vector, const std::vector<float> &y_move_vector, std::vector<float> &filtered_x_vector, std::vector<float> &filtered_y_vector, float max_diff);
	void filterRotationPoints(const std::vector<cv::Point2d> &vec1, const std::vector<cv::Point2d> &vec2, std::vector<cv::Point2d> &vec1Filtered, std::vector<cv::Point2d> &vec2Filtered);
	
	float calculateAverage(const std::vector<float> &values);
	float calculateTotal(const std::vector<float> &values);
	float calculateMedian(const std::vector<float> &values);
	float calculateStandardDeviation(const std::vector<float> &values);
	float calculateDistanceOfPoints(cv::Point2f pt1, cv::Point2f pt2);

	//Comparation of KeyPoints by their response to use with std::sort
	bool compareKeypoints(cv::KeyPoint kp1, cv::KeyPoint kp2);

}