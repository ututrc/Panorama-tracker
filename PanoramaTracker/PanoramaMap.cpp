#include "PanoramaMap.h"


void PanoramaMap::InitMap(Mat firstFrame, bool mapReady)
{
	Status(mapReady);
	
	//Initialize the largest map image
	map_ = Mat(map_height_, map_width_, CV_8U, Scalar(0, 0, 0));
	
	//Set the first frame of the map to the center of the largest map
	firstFrame.copyTo(map_(Rect(map_.cols / 2 - firstFrame.cols / 2, map_.rows / 2 - firstFrame.rows / 2, firstFrame.cols, firstFrame.rows)));
	
	//Create the smaller versions of the map used for pyramidical tracking
	resize(map_, map_half_res_, Size(map_.cols / 2, map_.rows / 2));
	resize(map_, map_quarter_res_, Size(map_.cols / 4, map_.rows / 4));

	//Create the initial mask of the map to determine which pixels are set and which aren't
	threshold(map_, mask_map_, 0, 250, CV_THRESH_BINARY);
}

Mat PanoramaMap::getUnsetPixels(Viewpoint &currentViewpoint, Mat &currentFrame)
{
	Mat test, diff;
	Rect vp = currentViewpoint.GetViewpoint(MAP_SIZE_FULL);

	//Extract the part of the map mask that is currently inside the viewpoint
	mask_frame_previous_ = mask_map_(vp).clone();

	//Determine the pixels that should be extracted from the current frame
	//and set to the map
	bitwise_or(mask_map_(vp), mask_current_view_, mask_map_(vp));
	bitwise_xor(mask_map_(vp), mask_frame_previous_, diff);
	return diff;
}


void PanoramaMap::setUnsetInMask(Rect area)
{
	//Set part of the map of the mask to unset (i.e. black pixels)
	mask_map_(area).setTo(Scalar(0, 0, 0));
}

void PanoramaMap::LoopClose(float minPx, float maxPx, Size imgSize, Mat minImg, Mat maxImg)
{
	Mat d1, d2, res;
	std::vector<KeyPoint> kps1, kps2;
	std::vector<DMatch> matches;

	//Match the images at the edges of the map
	Ptr<ORB> orb = ORB::create(200);
	orb->detectAndCompute(minImg, noArray(), kps1, d1);
	orb->detectAndCompute(maxImg, noArray(), kps2, d2);
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
	matcher->match(d1, d2, matches);

	float x_mov = 0;
	float j = 0;
	for (int i = 0; i < matches.size(); i++)
	{
		DMatch m = matches.at(i);
		if (m.distance < 180){
			j++;
			x_mov += kps1.at(m.queryIdx).pt.x - kps2.at(m.trainIdx).pt.x;
		}
	}

	if (j != 0){
		x_mov = x_mov / j;
	}
	else
	{
		// What to do if the matching fails completely?
		// TODO: for example try matching to another location
		min_x_jump_ = minPx;
		max_x_jump_ = maxPx;
		x_mov = 0;
	}
	min_x_jump_ = minPx;
	max_x_jump_ = maxPx - x_mov;
	is_closed_ = true;
}

bool PanoramaMap::IsClosed() const
{
	return is_closed_;
}

void PanoramaMap::GetJumpLimits(int& max, int& min) const
{
	max = max_x_jump_;
	min = min_x_jump_;
}

PanoramaMap::PanoramaMap()
{
	map_width_ = 0;
	map_height_ = 0;
	pyramidical_ = true;
}

PanoramaMap::PanoramaMap(float mapWidth, float mapHeight, bool pyramidical) : 
	map_width_(mapWidth), 
	map_height_(mapHeight),
	pyramidical_(pyramidical)
{
}

void PanoramaMap::UpdateMap(Viewpoint &currentViewpoint, Mat currentFrame, const std::vector<Point> &changedCells)
{
	//Update map based on the masks
	//Get the unset pixels that should be set to the map from the frame
	Mat unset_in_frame = getUnsetPixels(currentViewpoint, currentFrame);
	Mat pixels_in_mask;
	
	//Get the pixels from the frame to pixels_in_mask_ according to the unset_in_frame mask
	currentFrame.copyTo(pixels_in_mask, unset_in_frame);

	//Set the found pixels in to the map_
	pixels_in_mask.copyTo(map_(currentViewpoint.GetViewpoint(MAP_SIZE_FULL)), unset_in_frame);
	
	//Since resizing is an expensive operation, dont update the smaller maps every frame.
	//Only update them whenever some cell is completely filled
	if (pyramidical_ && changedCells.size() > 0 || update_smaller_){
		resize(map_, map_half_res_, Size(map_.cols / 2, map_.rows / 2));
		resize(map_, map_quarter_res_, Size(map_.cols / 4, map_.rows / 4));
		update_smaller_ = false;
	}
}

Mat PanoramaMap::GetMap(MapSize mapSize) const
{
	if (mapSize == MAP_SIZE_FULL || !pyramidical_)
	{
		return map_;
	}
	else if (mapSize == MAP_SIZE_HALF)
	{
		return map_half_res_;
	}
		return map_quarter_res_;
}

void PanoramaMap::SetMap(Mat map, MapSize mapSize)
{
	if (mapSize == MAP_SIZE_FULL || !pyramidical_)
	{
		map_ = map.clone();
	}
	else if (mapSize == MAP_SIZE_HALF)
	{
		map_half_res_ = map.clone();
	}
	else
	{
		map_quarter_res_ = map.clone();
	}
}

bool PanoramaMap::Status() const
{
	return map_ready_;
}

void PanoramaMap::Status(bool status)
{
	map_ready_ = status;
}



Mat PanoramaMap::GetMask(MaskType maskType) const
{
	if (maskType == MASK_CURRENT)
	{
		return mask_current_view_;
	}
	if (maskType == MASK_MAP)
	{
		return mask_map_;
	}
	return mask_frame_previous_;
}

void PanoramaMap::SetMask(MaskType maskType, const Mat &new_mask)
{
	if (maskType == MASK_CURRENT)
	{
		mask_current_view_ = new_mask;
	}
	else if (maskType == MASK_MAP)
	{
		mask_map_ = new_mask;
	}
	else
	{
		mask_frame_previous_ = new_mask;
	}
}

float PanoramaMap::GetHeight() const
{
	return map_height_;
}

float PanoramaMap::GetWidth() const
{
	return map_width_;
}

void PanoramaMap::UpdateColumn(const Rect &area)
{
	mask_map_(area).setTo(Scalar(0, 0, 0));
	//Also force the update of smaller versions of the map
	update_smaller_ = true;
}