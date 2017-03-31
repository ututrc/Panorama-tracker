#pragma once

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "PanoramaMap.h"

using namespace cv;
class PtFeature
{
public:
	enum KeypointType{ KP_FULL_MAP, KP_HALF_MAP, KP_QUARTER_MAP };

	Point pt_cell;	//Coordinates of the point inside its cell
	Point pt_map;	//Coordinates of the point in the map
	float quality;	//Quality of the point from 0..1

	int movement_x;	//Movement of the feature in the last frame, x
	int movement_y;	//Movement of the feature in the last frame, y

	PtFeature();
	PtFeature(Point ptCell, Point ptMap);
	
	//Get the area around the feature point from the map
	bool GetTemplate(int supportAreaSize, Mat map, Mat &supportArea) const;
	int GetId() const;

private:
	//Used for id generation
	static int ids;
	
	//ID for each feature, used in rotation estimation to mach pairs of features
	int id;
};