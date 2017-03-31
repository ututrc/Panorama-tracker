#include "HelpFunctions.h"
#include <iostream>

namespace HelpFunctions{
	void filterMovementVectors(const std::vector<float> &x_move_vector, const std::vector<float> &y_move_vector, 
		std::vector<float> &filtered_x_vector, std::vector<float> &filtered_y_vector, 
		float max_diff)
	{
		float median_x = calculateMedian(x_move_vector);
		float median_y = calculateMedian(y_move_vector);
		//Remove values too far from the median
		for (size_t i = 0; i < x_move_vector.size(); i++)
		{
			if (abs(x_move_vector.at(i) - median_x) < max_diff || abs(y_move_vector.at(i) - median_y) < max_diff)
			{
				filtered_x_vector.push_back(x_move_vector.at(i));
				filtered_y_vector.push_back(y_move_vector.at(i));
			}
		}
	}

	void filterRotationPoints(const std::vector<cv::Point2d> &vec1, const std::vector<cv::Point2d> &vec2, std::vector<cv::Point2d> &vec1Filtered, std::vector<cv::Point2d> &vec2Filtered)
	{
		std::vector<float> x_movements, y_movements;
		//Calculate median distances in x- and y- axis
		for (size_t i = 0; i < vec1.size(); i++)
		{
			x_movements.push_back(vec1.at(i).x - vec2.at(i).x);
			y_movements.push_back(vec1.at(i).y - vec2.at(i).y);
		}
		float median_x = calculateMedian(x_movements);
		float median_y = calculateMedian(y_movements);
		//Remove values too far from the median
		const float max_diff = 3;
		for (size_t i = 0; i < x_movements.size(); i++)
		{
			if (abs(x_movements.at(i) - median_x) < max_diff || abs(y_movements.at(i) - median_y) < max_diff)
			{
				vec1Filtered.push_back(vec1.at(i));
				vec2Filtered.push_back(vec2.at(i));
			}
		}
	}

	float calculateAverage(const std::vector<float> &values)
	{
		if (values.size() == 0) return 0;
		float total = 0;
		for (size_t i = 0; i < values.size(); i++)
		{
			total += values.at(i);
		}
		if (total == 0) return 0;

		return total / values.size();
	}

	float calculateTotal(const std::vector<float>& values)
	{
		float total = 0;
		for (size_t i = 0; i < values.size(); i++)
		{
			total += values.at(i);
		}
		return total;
	}

	float calculateMedian(const std::vector<float>& values)
	{
		if (values.size() == 0) return 0;
		else if (values.size() == 1) return values.at(0);
		std::vector<float> sorted = values;
		std::sort(sorted.begin(), sorted.end());
		if (sorted.size() % 2 == 0)
		{
			int ind1 = sorted.size() / 2;
			int ind2 = ind1 - 1;
			return (sorted.at(ind1) + sorted.at(ind2)) / 2;
		}
		else
		{
			return sorted.at(ceil((float)sorted.size() / 2));
		}
	}

	float calculateStandardDeviation(const std::vector<float> &values)
	{
		float avg = calculateAverage(values);
		float var = 0;
		for (size_t i = 0; i < values.size(); i++)
		{
			float diff = values.at(i) - avg;
			var += diff * diff;
		}
		var = var / values.size();
		float deviation = sqrt(var);
		return deviation;
	}

	float calculateDistanceOfPoints(cv::Point2f pt1, cv::Point2f pt2)
	{
		float dist;
		float t1,t2, t1p2, t2p2;
		t1 = pt1.x - pt2.x;
		t2 = pt1.y - pt2.y;
		t1p2 = t1 * t1;
		t2p2 = t2 * t2;
		dist = sqrt(t1p2 + t2p2);
		return dist;
	}

	bool compareKeypoints(cv::KeyPoint kp1, cv::KeyPoint kp2)
	{
		return (kp1.response > kp2.response);
	}
}