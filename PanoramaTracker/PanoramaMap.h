#pragma once

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include "Viewpoint.h"
#define NO_OF_CELLS_X 64
#define NO_OF_CELLS_Y 18
using namespace cv;

/*
Class that holds all the information about the panoramic map(s) used for the tracking
The pyramidic method for tracking is currently deprecated!
*/
class PanoramaMap
{
private:
	//(Full sized) Map dimensions
	float map_width_;
	float map_height_;

	//Map and its down scaled versions for pyramidic tracking
	Mat map_;
	Mat map_half_res_;
	Mat map_quarter_res_;

	/*
	A binary mask_current_view_ showing the current viewpoint of the camera.
	Since the image is warped cylindrically, the image is not rectangular.
	Therefore this mask, which shows which pixels in the image are actual
	image data and which are just black borders
	*/
	Mat mask_current_view_;

	//A binary mask that containts all the pixels that are already mapped
	Mat mask_map_;

	//The mask from the previous frame
	Mat mask_frame_previous_;

	//Was used for loading/saving of maps
	bool map_ready_ = false;

	//Is pyramidical tracking used
	bool pyramidical_;

	//Toggle to true if the loop closing has been done
	bool is_closed_ = false; 
	
	//Variables for jump coordinates in loop closing
	int min_x_jump_;
	int max_x_jump_;
	bool update_smaller_ = false;

	//Get the mask containing all unset pixels using the current viewpoint
	Mat getUnsetPixels(Viewpoint &currentViewpoint, Mat &currentFrame);

public:
	//Whether the handled mask is the one from current frame, one of the whole map or of previous frame
	enum MaskType{MASK_CURRENT, MASK_MAP, MASK_MAP_PREV};

	//Constructors
	PanoramaMap();
	PanoramaMap(float mapWidth, float mapHeight, bool pyramidical);
	
	//Initialize the map when tracking begins
	void InitMap(Mat firstFrame, bool mapReady);
	
	/*
	Update map based on current viewpoint and changed cells. Changed cells are used to 
	update the smaller versions of the map without requiring so much resize operations
	*/
	void UpdateMap(Viewpoint &currentViewpoint, Mat currentFrame, const std::vector<Point> &changedCells);
	
	//Get the image of the requested map size
	Mat GetMap(MapSize mapSize) const;

	//Set a custom image to be used as the map
	void SetMap(Mat map, MapSize mapSize);

	//Status if map is ready or not
	bool Status() const;
	void Status(bool status);
	
	//Set and get the mask of current view of type maskType
	Mat GetMask(MaskType maskType) const;
	void SetMask(MaskType maskType, const Mat &new_mask);
	
	//Get (full) map dimensions
	float GetHeight() const;
	float GetWidth() const;
	
	/*
	Update a single column of cells in the map. Used for updating a part 
	of the map that is already mapped. However, continuos map updates are deprecated
	since they tend to cause drift
	*/
	void UpdateColumn(const Rect &area);

	//Set an area specified by rectangle to be unset in the mask. Used for re-updating map parts
	void setUnsetInMask(Rect area);

	/*Loop closing when 360 circle complete. minPx and maxPx are the positions for
	where the min and max images should be extracted from */
	void LoopClose(float minPx, float maxPx, Size imgSize, Mat minImg, Mat maxImg);
	bool IsClosed() const;
	void GetJumpLimits(int &max, int &min) const;
};