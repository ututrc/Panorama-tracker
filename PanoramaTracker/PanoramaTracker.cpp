#include "PanoramaTracker.h"

PanoramaTracker::PanoramaTracker()
{
	//Use default settings
	PtSettings settings;
	tracker_settings = settings;
	construct();
}

PanoramaTracker::PanoramaTracker(PtSettings settings) : tracker_settings(settings)
{
	construct();
}

void PanoramaTracker::construct(){
	tracker_settings.Set(PT_CELLS_X, NO_OF_CELLS_X);
	tracker_settings.Set(PT_CELLS_Y, NO_OF_CELLS_Y);
	bool android; tracker_settings.Get(PT_USE_ANDROID_SHIELD, android);
	int warper_scale; tracker_settings.Get(PT_WARPER_SCALE, warper_scale);
	warper_ = ImageWarper(warper_scale, android);
	relocalizer = Relocalizer();
}

void PanoramaTracker::InitializeMap(Mat frame, bool mapReady, bool mapLoaded)
{
	Mat gray, warped, mask_curr_view;
	float img_w, img_h;
	float s = (float)map_vertical_degrees_ / 360.0;
	bool android;
	int warper_scale;
	int camera_fov_h, camera_fov_v;
	int cell_width, cell_height;


	//Initialize the warper object and warp the frame
	tracker_settings.Get(PT_USE_ANDROID_SHIELD, android);
	tracker_settings.Get(PT_WARPER_SCALE, warper_scale);
	warper_ = ImageWarper(warper_scale, android);
	cvtColor(frame, gray, CV_BGR2GRAY);
	mask_curr_view = warper_.warpImageCylindrical(gray, -y_rotation_, 0, z_rotation_, warped);

	//Calculate the relations between the input image size and scaled down image size
	x_res_scaling_ = (double)gray.cols / (double)mask_curr_view.cols;
	y_res_scaling_ = (double)gray.rows / (double)mask_curr_view.rows;

	/*
	Get the size of the warped frames at 0 0 0 rotation, and save to settings object
	The size of the warped frames is the size of the actual handled images, since warper
	scales down the image size from the original. I.e. if the input resolution is 640x480, 
	the images being used with the map are smaller
	*/
	img_w = mask_curr_view.cols;
	initial_img_width_ = img_w;
	img_h = mask_curr_view.rows;
	tracker_settings.Set(PT_CAMERA_WIDTH, (int)img_w);
	tracker_settings.Set(PT_CAMERA_HEIGHT, (int)img_h);
	
	//Calculate the parameters used in pixel<->degree conversions
	tracker_settings.Get(PT_CAMERA_FOV_HORIZONTAL, camera_fov_h);
	tracker_settings.Get(PT_CAMERA_FOV_VERTICAL, camera_fov_v);
	pixels_in_circle_x_ = (float)img_w / (float)camera_fov_h * map_horizontal_degrees;
	map_view_pixels_ = (float)img_w / (float)camera_fov_h * map_view_degrees_;
	pixels_in_circle_y_ = (float)img_h / (float)camera_fov_v * 360.0;
	

	//Calculate cell sizes in order to initialize cellmanager
	int map_width = pixels_in_circle_x_;
	int map_height = (int)(s * pixels_in_circle_y_);
	bool pyr; tracker_settings.Get(PT_PYRAMIDICAL, pyr);
	//Check if an old map is loaded, or a new map is being created
	//(Loading old maps is currently deprecated)
	if (!mapLoaded)
	{
		panorama_map = PanoramaMap(map_width, map_height, pyr);
		panorama_map.SetMask(PanoramaMap::MASK_CURRENT, mask_curr_view);
		panorama_map.InitMap(warped, mapReady);
		cell_width = ceil(map_width / NO_OF_CELLS_X);
		cell_height = ceil(map_height / NO_OF_CELLS_Y);
	}
	else
	{
		Mat full_map = panorama_map.GetMap(MAP_SIZE_FULL);
		cell_width = ceil(full_map.cols / NO_OF_CELLS_X);
		cell_height = ceil(full_map.rows / NO_OF_CELLS_Y);
	}
	cell_manager_ = CellManager(cell_width, cell_height);
	std::cout << "Initialized map size: " << map_width << "," << map_height << " with image resolution " << img_w << "," << img_h << std::endl;
	
	//Initialize the viewpoint object
	float orientation_pixels_x, orientation_pixels_y;
	if (!mapLoaded){
		orientation_pixels_x = map_width / 2;
		orientation_pixels_y = map_height / 2;
	}
	else
	{
		Mat full_map = panorama_map.GetMap(MAP_SIZE_FULL);
		orientation_pixels_x = full_map.cols / 2;
		orientation_pixels_y = full_map.rows / 2;
	}
	viewpoint_ = Viewpoint(orientation_pixels_x - mask_curr_view.cols / 2, orientation_pixels_y - mask_curr_view.rows / 2,
		mask_curr_view.cols, mask_curr_view.rows);

	//Check which cells are completely filled after initialization, and search for keypoints in them
	Mat mapMask = panorama_map.GetMask(PanoramaMap::MASK_MAP);
	for (int i = 0; i < NO_OF_CELLS_X; i++)
	{
		for (int j = 0; j < NO_OF_CELLS_Y; j++)
		{
			cell_manager_.UpdateCellStatus(i, j, mapMask);
			//If cell has been filled and map is not loaded using mapmanager
			if (cell_manager_.Status(i,j) && !mapLoaded){
				getKeypoints(i, j, MAP_SIZE_FULL);
				getKeypoints(i, j, MAP_SIZE_QUARTER);
				getKeypoints(i, j, MAP_SIZE_HALF);
			}
		}
	}
}


void PanoramaTracker::updateCurrentFrame(Mat frame)
{
	Mat gray, warped;

	debug_timer_.StartTimer("cvtColor in updateCurrentFrame");
	cvtColor(frame, gray, CV_BGR2GRAY);
	debug_timer_.StopTimer("cvtColor in updateCurrentFrame");

	//Is the non warped frame necessary?? One extra clone...
	current_frame_non_warped_ = gray.clone();

	debug_timer_.StartTimer("warper.warpImageCylindrical");
	Mat mask_curr_frame = warper_.warpImageCylindrical(gray, -y_rotation_, x_rotation_, z_rotation_, warped);
	debug_timer_.StopTimer("warper.warpImageCylindrical");

	panorama_map.SetMask(PanoramaMap::MASK_CURRENT, mask_curr_frame);
	viewpoint_.UpdateViewpointSize(mask_curr_frame.cols, mask_curr_frame.rows, panorama_map.GetWidth(), panorama_map.GetHeight());
	current_frame_ = warped;
	bool pyr; tracker_settings.Get(PT_PYRAMIDICAL, pyr);
	if (pyr){
		resize(current_frame_, current_frame_half_, Size(current_frame_.cols / 2, current_frame_.rows / 2));
		resize(current_frame_, current_frame_quarter_, Size(current_frame_.cols / 4, current_frame_.rows / 4));
	}
}

Mat PanoramaTracker::ViewMap(MapSize mapSize, bool drawCells, bool drawKeypoints, bool showViewpoint, bool moveMap, bool onlyGet) const
{
	int ch = cell_manager_.GetCellHeight(mapSize);
	int cw = cell_manager_.GetCellWidth(mapSize);
	Mat map_colored;
	cvtColor(panorama_map.GetMap(mapSize), map_colored, CV_GRAY2BGR);
	bool pyr; tracker_settings.Get(PT_PYRAMIDICAL, pyr);
	Mat map_clone = map_colored.clone();
	if (drawCells)
	{
		//Draw vertical lines
		for (int i = 0; i < NO_OF_CELLS_X; i++)
		{
			line(map_clone, Point(i*cw, 0), Point(i*cw, map_colored.rows), Scalar(255, 255, 255), 1);
		}
		//Draw horizontal lines
		for (int i = 0; i < NO_OF_CELLS_Y; i++)
		{
			line(map_clone, Point(0, i*ch), Point(map_colored.cols, i*ch), Scalar(255, 255, 255), 1);
		}
	}
	if (drawKeypoints)
	{
		for (int i = 0; i < NO_OF_CELLS_X; i++)
		{
			for (int j = 0; j < NO_OF_CELLS_Y; j++)
			{
				std::vector<PtFeature> keypoints;
				if(mapSize==MAP_SIZE_FULL)keypoints = cell_manager_.GetCellKeypoints(i, j, PtFeature::KP_FULL_MAP);
				else if (mapSize == MAP_SIZE_HALF )keypoints = cell_manager_.GetCellKeypoints(i, j, PtFeature::KP_HALF_MAP);
				else keypoints = cell_manager_.GetCellKeypoints(i, j, PtFeature::KP_QUARTER_MAP);
				for (size_t k = 0; k < keypoints.size(); k++){
					circle(map_clone, Point(keypoints.at(k).pt_map),
						3, Scalar(0, 255, 0), 1);
				}
			}
		}
	}

	if (showViewpoint){
		viewpoint_.DrawViewpoint(map_clone, mapSize);
	}

	//Cut the map to the required size
	Mat shown;
	if (moveMap){
		int f;
		int h, w;
		if (mapSize == MAP_SIZE_FULL)
		{
			f = 1;
		}
		else if (mapSize == MAP_SIZE_HALF)
		{
			f = 2;
		}
		else
		{
			f = 4;
		}
		h = panorama_map.GetHeight() / f;
		w = map_view_pixels_ / f;
		int x = (viewpoint_.x / f + (viewpoint_.width / 2) / f - (w / 2));
		if (x < 0) x = 0;
		if (x + w > panorama_map.GetWidth() / f){
			x = panorama_map.GetWidth() / f - map_view_pixels_ / f;
		}
		Rect visibleMap = Rect(x, 0, w, h);
		shown = map_clone(visibleMap);
	}
	else
	{
		shown = map_clone;
	}
	//Draw orientation texts
	std::ostringstream oss;
	oss << "Orientation x: " << x_rotation_ << ", Orientation y: " << y_rotation_ << ", Rotation: " << z_rotation_;
	putText(shown, oss.str(), Point(20, 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	oss.str("");
	oss << "Tracking status: " << tracking_status << ", quality: " << average_quality_ << ", deviation: " << deviation_;
	putText(shown, oss.str(), Point(20, shown.rows - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	if (!onlyGet) imshow(MAP_WINDOW, shown);
	return shown;
}

Mat PanoramaTracker::GetMapImage() const
{
	int f = 1;	//Full map 1, half map 2, quarter map 4
	int h, w;
	Mat map = panorama_map.GetMap(MAP_SIZE_FULL);
	Mat colored, resized; 
	//"Color" the map in order to draw the viewpoint as a colored rect
	cvtColor(map, colored, CV_GRAY2BGR);
	viewpoint_.DrawViewpoint(colored, MAP_SIZE_FULL);

	//Calculate the part of the map that should be currently visible
	h = panorama_map.GetHeight() / f;
	w = map_view_pixels_ / f;
	int x = (viewpoint_.x / f + (viewpoint_.width / 2) / f - (w / 2));
	if (x < 0) x = 0;
	if (x + w > panorama_map.GetWidth() / f){
		x = panorama_map.GetWidth() / f - map_view_pixels_ / f;
	}
	Rect visibleMap = Rect(x, 0, w, h);
	Mat shown = colored(visibleMap);
	//The map shown in the Unity3D Application doesn't need to be very large
	resize(shown, resized, Size(shown.cols / 4, shown.rows / 4));
	//Viewpoint in the middle of the map
	return resized;
}

Mat PanoramaTracker::GetMapImage(Size size) const{
	int f = 1;	//Full map 1, half map 2, quarter map 4
	int h, w;
	Mat map = panorama_map.GetMap(MAP_SIZE_FULL);
	Mat colored, resized;
	//"Color" the map in order to draw the viewpoint as a colored rect
	cvtColor(map, colored, CV_GRAY2BGR);
	viewpoint_.DrawViewpoint(colored, MAP_SIZE_FULL);

	//Calculate the part of the map that should be currently visible
	h = panorama_map.GetHeight() / f;
	w = map_view_pixels_ / f;
	int x = (viewpoint_.x / f + (viewpoint_.width / 2) / f - (w / 2));
	if (x < 0) x = 0;
	if (x + w > panorama_map.GetWidth() / f){
		x = panorama_map.GetWidth() / f - map_view_pixels_ / f;
	}
	Rect visibleMap = Rect(x, 0, w, h);
	Mat shown = colored(visibleMap);

	resize(shown, resized, size);
	//Viewpoint in the middle of the map
	return resized;
}

int PanoramaTracker::GetViewMapWidth() const
{
	return map_view_pixels_;
}

void PanoramaTracker::estimateOrientation(MapSize mapSize, std::vector<float> &xMovements, std::vector<float> &yMovements, std::vector<float> &qualities)
{
	//First estimate the orientation using the mapSize resolution map_
	//First find cells which are visible if guess is correct
	//For first frame, use the originally initialized cells and their keypoints aka all found keypoints
	//Get 8x8 pixel frames for each keypoint
	int support_area_size;
	tracker_settings.Get(PT_SUPPORT_AREA_SIZE, support_area_size);
	Mat map_clone = panorama_map.GetMap(mapSize);
	//Clear the used point vectors
	bool rotInv; tracker_settings.Get(PT_ROTATION_INVARIANT, rotInv);
	if (rotInv){
		matched_features_.clear();
	}
	bool pyr; tracker_settings.Get(PT_PYRAMIDICAL, pyr);
	//Iterate through all cells
	debug_timer_.StartTimer("matchTemplates");
	for (int i = 0; i < NO_OF_CELLS_X; i++)
	{
		for (int j = 0; j < NO_OF_CELLS_Y; j++)
		{
			//If cell is set and visible, find keypoints
			if (cell_manager_.Status(i,j) && cell_manager_.CellVisible(i,j, viewpoint_.GetViewpoint(MAP_SIZE_FULL)))
			{
				//Get the keypoints of the current cell and iterate each keypoint
				std::vector<PtFeature>* kps;
				if (mapSize == MAP_SIZE_FULL || !pyr)
					kps = cell_manager_.GetCellKeypointsPtr(i, j, PtFeature::KP_FULL_MAP);
				else if (mapSize == MAP_SIZE_HALF)
					kps = cell_manager_.GetCellKeypointsPtr(i, j, PtFeature::KP_HALF_MAP);
				else
					kps = cell_manager_.GetCellKeypointsPtr(i, j, PtFeature::KP_QUARTER_MAP);

				//Iterate through each keypoint, and templatematch them
					for (size_t k = 0; k < kps->size(); k++)
					{
						float movementX, movementY;
						float quality;

						//matchTemplates to calculate movement in y and x direction
						if (matchTemplates(kps->at(k), mapSize, movementX, movementY, quality))
						{
							kps->at(k).movement_x = movementX;
							kps->at(k).movement_y = movementY;
							xMovements.push_back(movementX);
							yMovements.push_back(movementY);
							qualities.push_back(quality);
						}
						//If the tracking fails for some reason, add error values (-1000) as movements
						else
						{
							kps->at(k).movement_x = -1000;
							kps->at(k).movement_y = -1000;
							qualities.push_back(quality);
						}
					}
			}
		}
	}
	debug_timer_.StopTimer("matchTemplates");
}

float PanoramaTracker::trackAndUpdate(MapSize mapSize, std::vector<float> &allQualities)
{
	float filtered_xmove, filtered_ymove;
	std::vector<float> filtered_x_vector, filtered_y_vector;
	std::vector<float> x_move_vector, y_move_vector, quality_vector;
	int factor, max_dev;

	estimateOrientation(mapSize, x_move_vector, y_move_vector, quality_vector);
	//Append the qualities returned by estimating orientation to caller
	allQualities.insert(allQualities.end(), quality_vector.begin(), quality_vector.end());

	if (mapSize == MAP_SIZE_FULL)
	{
		factor = 1;
		tracker_settings.Get(PT_MAX_DEV_FILTERING_FULL, max_dev);
	}
	else if (mapSize == MAP_SIZE_HALF)
	{
		factor = 2;
		tracker_settings.Get(PT_MAX_DEV_FILTERING_HALF, max_dev);
	}
	else
	{
		factor = 4;
		tracker_settings.Get(PT_MAX_DEV_FILTERING_QUARTER, max_dev);
	}

	int xmed = HelpFunctions::calculateMedian(x_move_vector);
	int ymed = HelpFunctions::calculateMedian(y_move_vector);
	HelpFunctions::filterMovementVectors(x_move_vector, y_move_vector, filtered_x_vector, filtered_y_vector,
		max_dev);

	//Update values used for statistics viewing
	if (mapSize == MAP_SIZE_FULL){
		used_kp_full_ = filtered_x_vector.size();
		discarded_kp_full_ = x_move_vector.size() - filtered_x_vector.size();
	}
	else if (mapSize == MAP_SIZE_HALF)
	{
		used_kp_half_ = filtered_x_vector.size();
		discarded_kp_half_ = x_move_vector.size() - filtered_x_vector.size();
	}
	else
	{
		used_kp_quarter_ = filtered_x_vector.size();
		discarded_kp_quarter_ = x_move_vector.size() - filtered_x_vector.size();
	}

	updateFeatures(mapSize, xmed, ymed);
	filtered_xmove = -HelpFunctions::calculateMedian(filtered_x_vector) * factor;
	filtered_ymove = -HelpFunctions::calculateMedian(filtered_y_vector) * factor;
	updateViewpointLocation(filtered_xmove, filtered_ymove);
	float dev_x = HelpFunctions::calculateStandardDeviation(x_move_vector);
	float dev_y = HelpFunctions::calculateStandardDeviation(y_move_vector);
	return (dev_x + dev_y) / 2;
}

void PanoramaTracker::updateFeatures(MapSize mapSize, int medx, int medy)
{
	debug_timer_.StartTimer("updateFeatures");
	int max_diff;
	if (mapSize == MAP_SIZE_FULL) tracker_settings.Get(PT_MAX_DEV_FILTERING_FULL, max_diff);
	else if (mapSize == MAP_SIZE_HALF) tracker_settings.Get(PT_MAX_DEV_FILTERING_HALF, max_diff);
	else tracker_settings.Get(PT_MAX_DEV_FILTERING_QUARTER, max_diff);

	std::vector<float> x_move_vector, y_move_vector;
	for (int i = 0; i < NO_OF_CELLS_X; i++)
	{
		for (int j = 0; j < NO_OF_CELLS_Y; j++)
		{
			bool features_erased = false;
			if (cell_manager_.CellVisible(i, j, viewpoint_.GetViewpoint(MAP_SIZE_FULL))){
				std::vector<PtFeature>* features;
				if (mapSize == MAP_SIZE_FULL) features = cell_manager_.GetCellKeypointsPtr(i,j, PtFeature::KP_FULL_MAP);
				else if (mapSize == MAP_SIZE_HALF) features = cell_manager_.GetCellKeypointsPtr(i,j, PtFeature::KP_HALF_MAP);
				else features = cell_manager_.GetCellKeypointsPtr(i, j, PtFeature::KP_QUARTER_MAP);
				
				//If the features movement is too far from the median movement, lower its quality
				for (std::vector<PtFeature>::iterator it = features->begin(); it != features->end();)
				{
					int x_mov = it->movement_x;
					int y_mov = it->movement_y;
					//Lower the quality if the feature has been updated last frame(!=-1000) and deviation from median is larger than max_diff
					if ((abs(x_mov - medx) > max_diff || abs(y_mov - medy) > max_diff) && x_mov != -1000)
					{
						it->quality -= .05;
					}
					//Else raise the quality higher, but not past 1
					else
					{
						if (it->quality < 1){
							it->quality += .25;
						}
						x_move_vector.push_back(it->movement_x);
						y_move_vector.push_back(it->movement_y);
					}
					//Check feature quality
					if (it->quality <= 0)
					{
						if (mapSize == MAP_SIZE_FULL) removed_points_full_++;
						else if (mapSize == MAP_SIZE_HALF) removed_points_half_++;
						else removed_points_quarter_++;
						features_erased = true;
						it = features->erase(it);
					}
					else
					{
						++it;
					}
				}
				//Here update cell, for now during largest map check only
				//using features_erased ensures that we don't do the check for cells that were already empty, i.e. in a position where no good features are available
				if (features_erased && mapSize == MAP_SIZE_FULL) updateCell(i, j);
			}
		}
	}
	debug_timer_.StopTimer("updateFeatures");

}

void PanoramaTracker::updateCell(int x, int y)
{
	//Find new features, if the cell becomes empty 
	if (cell_manager_.GetCellKeypoints(x,y, PtFeature::KP_FULL_MAP).size() < 1)
	{
		getKeypoints(x, y, MAP_SIZE_FULL);
		std::cout << "keypointsearch" << std::endl;
	}
}

void PanoramaTracker::CalculateOrientation(Mat currentFrame)
{
	//this is the main tracking function!

	debug_timer_.StartTimer("CalculateOrientation");

	//Update current frame data
	debug_timer_.StartTimer("update current frame call");
	updateCurrentFrame(currentFrame);
	debug_timer_.StopTimer("update current frame call");

	float min_tracking_quality;
	tracker_settings.Get(PT_MIN_TRACKING_QUALITY, min_tracking_quality);
	static float degrees_moved_x, degrees_moved_y;
	std::vector<float> all_qualities;
	float previous_x_orientation = x_rotation_;
	float previous_y_orientation = y_rotation_;
	bool sufficient_quality = false;
	Mat tvec, rvec;

	//If tracking, find the estimated orientation for each of the three maps
	float dev1 = 0, dev2 = 0, dev3 = 0;
	if (tracking_status == TRACKING_KEYPOINTS){
		bool pyr; tracker_settings.Get(PT_PYRAMIDICAL, pyr);
		bool rot_invariant; tracker_settings.Get(PT_ROTATION_INVARIANT, rot_invariant);
		//If using pyramidical approach, estimate orientation first for smaller versions of the map
		if (pyr){
			dev1 = trackAndUpdate(MAP_SIZE_QUARTER, all_qualities);
			dev2 = trackAndUpdate(MAP_SIZE_HALF, all_qualities);
		}
		//finally track on the whole map
		dev3 = trackAndUpdate(MAP_SIZE_FULL, all_qualities);

		//If rotation estimation is toggled on
		if (rot_invariant) estimateRotation();

		//Get the average quality and standard deviation of the tracking during the frame
		average_quality_ = HelpFunctions::calculateAverage(all_qualities);
		//Use the deviation from the full map only
		deviation_ = dev3;

		//Determine if the quality is sufficient. Different template matching methods have inverse
		//quality i.e. some methods have 0 as the best result, some have 1 as the best result
		if (template_matching_type == CV_TM_SQDIFF || template_matching_type == CV_TM_SQDIFF_NORMED)
		{
			if (average_quality_ > min_tracking_quality) sufficient_quality = false;
			else sufficient_quality = true;
		}
		else
		{
			if (average_quality_ < min_tracking_quality) sufficient_quality = false;
			else sufficient_quality = true;
		}

		//Toggle sufficient quality to false also if deviations are too large
		int max_dev; tracker_settings.Get(PT_MAX_DEVIATION, max_dev);
		if (dev3 > max_dev) sufficient_quality = false;

		//If quality is too low, toggle tracking status to relocalizing
		if (!sufficient_quality){
			tracking_status = RELOCALIZING;
		}
	}	//END IF tracking_status == TRACKING_KEYPOINTS

	if (tracking_status == RELOCALIZING)
	{
		Relocalize();
	}

	//Don't add new relocalization points if tracking quality is bad, since
	//that will result in relocalizationPoints that have incorrect rotation values
	if (sufficient_quality){
		float diff_x = previous_x_orientation - x_rotation_;
		float diff_y = previous_y_orientation - y_rotation_;
		//Add new relocalization points every x or y degrees
		degrees_moved_x += diff_x;
		degrees_moved_y += diff_y;
		if (degrees_moved_x > 10 || degrees_moved_x < -10 || degrees_moved_y > 10 || degrees_moved_y < -10)
		{
			//Use the non-warped frames with relocalizer <--why?
			relocalizer.AddRelocImage(current_frame_non_warped_, x_rotation_, y_rotation_, z_rotation_);
			degrees_moved_x = 0;
			degrees_moved_y = 0;
		}
	}

	//Don't update the map is tracking quality is bad, or map is considered finished
	if (!panorama_map.Status() && tracking_status == TRACKING_KEYPOINTS && sufficient_quality && !use_cstm_orientation_){
		//Dont update the map every frame, instead wait for a certain amount of movement to avoid accumulating "waves" in the image
		float diff_x = previous_x_orientation - x_rotation_;
		float diff_y = previous_y_orientation - y_rotation_;
		moved_deg_x_ += diff_x;
		moved_deg_y_ += diff_y;
		//After a certain amount of movement is reached, update the map
		if (abs(moved_deg_x_) > 2 || abs(moved_deg_y_) > 2){
			updateMap();
			moved_deg_x_ = 0;
			moved_deg_y_ = 0;
		}
	}

	//Determine if loop closing is required i.e. a full 360 circle has been completed
	if (x_rotation_ < min_rotation_){
		min_rotation_ = x_rotation_;
		min_rot_px_ = viewpoint_.x;
		min_rot_img_ = current_frame_.clone();
	}
	if (x_rotation_ > max_rotation_){
		max_rotation_ = x_rotation_;
		max_rot_px_ = viewpoint_.x;
		max_rot_img_ = current_frame_.clone();
	}

	//If 370 degrees are reached and loop closing is not yet done, call the loop closing function
	if (abs(max_rotation_ - min_rotation_) > 370 && !panorama_map.IsClosed())
	{
		panorama_map.LoopClose(min_rot_px_, max_rot_px_, current_frame_.size(), min_rot_img_, max_rot_img_);
	}

	debug_timer_.StopTimer("CalculateOrientation");
}

void PanoramaTracker::Relocalize()
{
	float x, y, z, quality, min_quality;
	tracker_settings.Get(PT_MIN_RELOC_QUALITY, min_quality);
	relocalizer.Relocalize(current_frame_non_warped_, x, y, z, quality);

	//If the relocalization quality is bad, don't move the viewpoint
	if (quality < min_quality){
		//Calculate the movement amounts in x and y directions and move viewpoint to the new pos accordingly
		float old_x = viewpoint_.x, old_y = viewpoint_.y;
		float new_x = degreesToPixelsX(x) - viewpoint_.width / 2;
		float y_corr = y + map_vertical_degrees_ / 2;
		float perc_of_vertical = y_corr / map_vertical_degrees_;
		float new_y = perc_of_vertical * panorama_map.GetHeight() - viewpoint_.height / 2;
		float diff_x = old_x - new_x;
		float diff_y = old_y - new_y;
		z_rotation_ = z;
		updateViewpointLocation(-diff_x, -diff_y);
		tracking_status = TRACKING_KEYPOINTS;
	}
}


void PanoramaTracker::SetRelocalizer(const Relocalizer& rl)
{
	relocalizer = rl;
}

void PanoramaTracker::SetPanoramaMap(const PanoramaMap& map)
{
	panorama_map = map;
}

float PanoramaTracker::GetOrientationX() const
{
	return x_rotation_;
}

float PanoramaTracker::GetOrientationY() const
{
	return y_rotation_;
}

float PanoramaTracker::GetRotation() const
{
	return z_rotation_;
}

float PanoramaTracker::GetQuality() const
{
	return average_quality_;
}

int PanoramaTracker::GetOrientationXPixels() const{
	//Return the pixel coordinate of viewpoint center
	return ((viewpoint_.x + viewpoint_.width / 2) * x_res_scaling_)-(panorama_map.GetWidth() / 2);
}

int PanoramaTracker::GetOrientationYPixels() const{
	//Return the pixel coordinate of viewpoint center
	return (viewpoint_.y + viewpoint_.height / 2) * x_res_scaling_ - (panorama_map.GetHeight() / 2);
}

void PanoramaTracker::SetOrientation(float x, float y, float z)
{
	use_cstm_orientation_ = true;
	//Calculate the correct differences in pixels in each direction
	float x_diff_deg = x - x_rotation_;
	float y_diff_deg = y- y_rotation_;
	float z_diff_deg = z - z_rotation_;
	std::cout << y_diff_deg << std::endl;

	//Convert each diff to pixels
	float x_mov = degreesToPixelsX(x_diff_deg);
	float y_mov = degreesToPixelsY(y_diff_deg, 0);
	updateViewpointLocation(x_mov, y_mov);
	//If using custom orientation, the CalculateOrientation is not necessarily called, thus the requirement
	//for calling updateMap here. 
	updateMap();
}

std::string PanoramaTracker::GetDebugData()
{
	std::stringstream ss;
	ss << std::setprecision(2);
	ss << "tracking status: " << tracking_status << ";";
	//Add the timer data from DebugTimer class
	for (DebugTimer::TimerResult tr : debug_timer_.GetResults()){
		ss << tr.toString() << std::endl;
	}

	//Add rest of the stuff that we want to have visible in the statistics string
	ss << "used kps/map: " << used_kp_full_ << "," << used_kp_half_ << "," << used_kp_quarter_ << ";"
	<< "discarded kps/map: " << discarded_kp_full_ << "," << discarded_kp_half_ << "," << discarded_kp_quarter_ << ";"
	<< "removed kps/map: " << removed_points_full_ << "," << removed_points_half_ << "," << removed_points_quarter_ << ";"
	<< "average quality: " << average_quality_ << ";"
	<< "tracking deviation: " << deviation_ << ";\n";

	debug_timer_.ClearAll();
	
	return ss.str();
}


void PanoramaTracker::UpdateColumn(Point clickedPoint)
{
	int column = clickedPoint.x / cell_manager_.GetCellHeight(MAP_SIZE_FULL);
	
	//Set the cell statuses of changed cells to false
	for (int i = 0; i < NO_OF_CELLS_Y; i++)
	{
		cell_manager_.Status(column, i, false);
	}
	panorama_map.UpdateColumn(Rect(column * cell_manager_.GetCellHeight(MAP_SIZE_FULL), 0, cell_manager_.GetCellWidth(MAP_SIZE_FULL), panorama_map.GetHeight()));
}


std::vector<PtFeature> PanoramaTracker::getKeypoints(int x, int y, MapSize mapSize)
{
	int cell_width = cell_manager_.GetCellWidth(mapSize);
	int cell_height = cell_manager_.GetCellHeight(mapSize);
	std::vector<KeyPoint> key_points;
	int kp_thresh, max_kp;
	tracker_settings.Get(PT_FAST_KEYPOINT_THRESHOLD, kp_thresh);
	tracker_settings.Get(PT_MAX_KEYPOINTS_PER_CELL, max_kp);
	FAST(cell_manager_.GetCellContents(x, y, mapSize, panorama_map.GetMap(mapSize)), key_points, kp_thresh);

	//Dont accept keypoints that are on the edge of the cell (conflicting support areas, i.e. the support area would
	//extend to the adjacent cell)
	std::vector<KeyPoint> accepted_points;
	int sh;
	tracker_settings.Get(PT_SUPPORT_AREA_SIZE, sh);
	sh = sh / 2;
	for (int i = 0; i < key_points.size(); i++)
	{
		if (key_points.at(i).pt.x - sh >= 0 && key_points.at(i).pt.y - sh >= 0
			&& key_points.at(i).pt.x + sh <= cell_width && key_points.at(i).pt.y + sh <= cell_height){
			accepted_points.push_back(key_points.at(i));
		}
	}
	//Sort the keypoints according to their quality (i.e. the response given by FAST)
	std::sort(accepted_points.begin(), accepted_points.end(), HelpFunctions::compareKeypoints);

	//if there are too many keypoints, only select the first max_kp from the sorted vector
	if (accepted_points.size() > max_kp)
	{
		accepted_points.resize(max_kp);
	}

	//Transform the KeyPoint objects to PtFeature objects
	std::vector<PtFeature> pt_features;
	for (size_t i = 0; i < accepted_points.size(); i++)
	{
		Point map_point(accepted_points.at(i).pt.x + x*cell_width, accepted_points.at(i).pt.y + y*cell_height);
		PtFeature feature(accepted_points.at(i).pt, map_point);
		pt_features.push_back(feature);
	}

	//Put keypoints in appropriate arrays according to the map size being handled
	if (mapSize == MAP_SIZE_FULL)
	{
		cell_manager_.SetCellKeypoints(x, y, PtFeature::KP_FULL_MAP, pt_features);
	}
	if (mapSize == MAP_SIZE_HALF)
	{
		cell_manager_.SetCellKeypoints(x, y, PtFeature::KP_HALF_MAP, pt_features);
	}
	if (mapSize == MAP_SIZE_QUARTER)
	{
		cell_manager_.SetCellKeypoints(x, y, PtFeature::KP_QUARTER_MAP, pt_features);
	}
	
	return pt_features;
}

bool PanoramaTracker::matchTemplates(PtFeature &feature, MapSize mapSize, float &movementX, float &movementY, float &qualityVal)
{
	//Try to find the feature around the area where it was during the previous frame
	Mat current_frame;
	Rect view_point;
	int support_area_search_size;
	int template_size;
	tracker_settings.Get(PT_SUPPORT_AREA_SIZE, template_size);

	//Get the frame used in the tracking according to the map size
	if (mapSize == MAP_SIZE_FULL){
		current_frame = current_frame_;
		tracker_settings.Get(PT_SUPPORT_AREA_SEARCH_SIZE_FULL, support_area_search_size);
	}
	else if (mapSize == MAP_SIZE_HALF){
		current_frame = current_frame_half_;
		tracker_settings.Get(PT_SUPPORT_AREA_SEARCH_SIZE_HALF, support_area_search_size);
	}
	else{
		current_frame = current_frame_quarter_;
		tracker_settings.Get(PT_SUPPORT_AREA_SEARCH_SIZE_QUARTER, support_area_search_size);
	}

	view_point = viewpoint_.GetViewpoint(mapSize);
	
	//Origin point (top left) for the support area that is extracted,
	//i.e. the area around the feature which is used as the template
	Point tmplt_origin;
	tmplt_origin.x = feature.pt_map.x - template_size / 2;
	tmplt_origin.y = feature.pt_map.y - template_size / 2;

	//Check that the area from which the templae is searched for is completely inside the current frame
	//Get the x- and y- coordinates of top left corner of the extractable area
	Point sprt_area_origin;
	sprt_area_origin.x = (template_size / 2) + tmplt_origin.x - view_point.x - support_area_search_size / 2;
	sprt_area_origin.y = (template_size / 2) + tmplt_origin.y - view_point.y - support_area_search_size / 2;

	//If the search area goes across the current frame borders, return false (failed tracking of this template)
	if (sprt_area_origin.x < 0 || sprt_area_origin.x + support_area_search_size >= current_frame.cols
		|| sprt_area_origin.y < 0 || sprt_area_origin.y + support_area_search_size >= current_frame.rows)
	{
		movementX = 0;
		movementY = 0;
		qualityVal = 1;
		return false;
	}

	//Get the actual template around the feature from the map
	Mat feature_template;
	feature.GetTemplate(template_size, panorama_map.GetMap(mapSize), feature_template);

	//Get the support area from which the template is searched from the current image
	Mat map_in_abs_pt = current_frame(Rect(sprt_area_origin.x, sprt_area_origin.y, support_area_search_size, support_area_search_size));

	//Match templates together
	Mat result(map_in_abs_pt.cols - feature_template.cols + 1, map_in_abs_pt.rows - feature_template.rows + 1, CV_32FC1);
	matchTemplate(map_in_abs_pt, feature_template, result, template_matching_type);
	double min_val; double max_val; Point min_loc; Point max_loc;
	Point match_loc;

	//Get the best location, according to what template matching type is used
	//Some template matching methods use 1 as the best value and 0 as worst, and some vice versa
	if (template_matching_type == CV_TM_SQDIFF || template_matching_type == CV_TM_SQDIFF_NORMED)
	{
		minMaxLoc(result, &min_val, nullptr, &min_loc, nullptr);
		match_loc = min_loc;
		qualityVal = min_val;
	}
	else
	{
		minMaxLoc(result, nullptr, &max_val, nullptr, &max_loc);
		match_loc = max_loc;
		qualityVal = max_val;
	}

	//Calculate the difference (movement) by checking how far from the middle the found point is
	//If the camera has not moved the matchLoc should be in the middle of the mapInAbsPt

	//Since match loc is the location of the top left corner of the found area
	movementX = match_loc.x - support_area_search_size / 2 +template_size / 2;
	movementY = match_loc.y - support_area_search_size / 2 +template_size / 2;

	float minQ;
	tracker_settings.Get(PT_MIN_TRACKING_QUALITY, minQ);

	//If tracked quality is good enough, add all the points and their correspondences to vectors
	//that can be used with estimateRigidTransform to estimate the rotation of the camera
	bool rotInv; tracker_settings.Get(PT_ROTATION_INVARIANT, rotInv);
	if (template_matching_type == CV_TM_SQDIFF || template_matching_type == CV_TM_SQDIFF_NORMED){
		if (rotInv && qualityVal < minQ){
			MatchedFeature ft;
			ft.id = feature.GetId();
			ft.movements = Point2d(movementX, movementY);
			matched_features_.push_back(ft);
		}
	}
	else
	{
		if (rotInv){
			MatchedFeature ft;
			ft.id = feature.GetId();
			ft.movements = Point2d(movementX, movementY);
			matched_features_.push_back(ft);
		}
	}

	return true;
}

void PanoramaTracker::estimateRotation()
{
	debug_timer_.StartTimer("EstimateRotation");

	//First get the set of features from the full sized map
	//Find keypoints in visible cells to a vector
	std::vector<PtFeature> visibleFeatures;
	std::vector<Point> visibleFeaturesPt, matchedFeaturesPt;
	for (int i = 0; i < NO_OF_CELLS_X; i++){
		for (int j = 0; j < NO_OF_CELLS_Y; j++){
			if (cell_manager_.CellVisible(i, j, viewpoint_.GetViewpoint(MAP_SIZE_FULL))){
				std::vector<PtFeature> appendaple = cell_manager_.GetCellKeypoints(i, j, PtFeature::KP_FULL_MAP);
				visibleFeatures.insert(visibleFeatures.end(), appendaple.begin(), appendaple.end());
			}
		}
	}

	//Calculate medians for x and y movements for filtering purposes
	std::vector<float> x_movements, y_movements;
	for (MatchedFeature m : matched_features_){
		x_movements.push_back(m.movements.x);
		y_movements.push_back(m.movements.y);
	}
	float x_med = HelpFunctions::calculateMedian(x_movements);
	float y_med = HelpFunctions::calculateMedian(y_movements);

	//Add both matched features and features in the viewpoint to vectors as Point2d
	for (PtFeature f : visibleFeatures){
		for (MatchedFeature m : matched_features_){
			if (f.GetId() == m.id && abs(m.movements.x - x_med) < 3 && abs(m.movements.y - y_med) < 3){
				Point2d vpt(f.pt_map.x - viewpoint_.x, f.pt_map.y - viewpoint_.y);
				matchedFeaturesPt.push_back(Point2d(vpt.x + m.movements.x, vpt.y + m.movements.y));
				visibleFeaturesPt.push_back(vpt);
			}
		}
	}

	float rotation = 0;
	//If there aren't enough visible features, don't estimate the rotation since the quality of the estimation
	//will be insufficient
	if (visibleFeaturesPt.size() > 15){
		Mat_<float> result_mat = estimateRigidTransform(visibleFeaturesPt, matchedFeaturesPt, false);
		if (result_mat.data){
			rotation = atan2(result_mat.at<float>(1, 0), result_mat.at<float>(0, 0)) * 180 / M_PI;
		}
	}
	
	z_rotation_ -= rotation;
	debug_timer_.StopTimer("EstimateRotation");
}

void PanoramaTracker::updateMap()
{
	debug_timer_.StartTimer("UpdateMap");

	//Get all cells that are visible, but not yet filled
	std::vector<Point> unset_cells = cell_manager_.GetUnsetVisibleCells(viewpoint_.GetViewpoint(MAP_SIZE_FULL));
	bool pyr; tracker_settings.Get(PT_PYRAMIDICAL, pyr);
	bool cell_prev, cell_new;
	Mat mapMask = panorama_map.GetMask(PanoramaMap::MASK_MAP);
	for (size_t i = 0; i < unset_cells.size(); i++)
	{
		//Get the status of the cell, then update the status and get the new status.
		//If the status has changed, save the coordinates of the changed cell. 
		cell_prev = cell_manager_.Status(unset_cells.at(i).x, unset_cells.at(i).y);
		cell_manager_.UpdateCellStatus(unset_cells.at(i).x, unset_cells.at(i).y, mapMask);
		cell_new = cell_manager_.Status(unset_cells.at(i).x, unset_cells.at(i).y);

		//If the status changed, push the coordinates of the cell and its contents to vector
		if (cell_prev != cell_new){
			changed_cells_.push_back(Point(unset_cells.at(i).x, unset_cells.at(i).y));
		}
	}

	//Update map after checking the cells, so that changed_cells_ are up to date
	panorama_map.UpdateMap(viewpoint_, current_frame_, changed_cells_);

	//Get new keypoints after the map is updated
	for (size_t i = 0; i < changed_cells_.size(); i++)
	{
		getKeypoints(changed_cells_.at(i).x, changed_cells_.at(i).y, MAP_SIZE_FULL);
		if (pyr){
			getKeypoints(changed_cells_.at(i).x, changed_cells_.at(i).y, MAP_SIZE_HALF);
			getKeypoints(changed_cells_.at(i).x, changed_cells_.at(i).y, MAP_SIZE_QUARTER);
		}
	}
	//Clear the changed cells vector for the next frame
	changed_cells_.clear();
	debug_timer_.StopTimer("UpdateMap");
}


void PanoramaTracker::updateViewpointLocation(float x_move, float y_move)
{
	debug_timer_.StartTimer("updateViewpointLocation");
	//If the loop closing has not been done, move the viewpoint to the correct location 
	if (!panorama_map.IsClosed()){
		viewpoint_.updateViewpointLocation(x_move, y_move, panorama_map.GetWidth(), panorama_map.GetHeight(), panorama_map.GetMask(PanoramaMap::MASK_MAP));
	}
	//If the loop closing is finished, some additional steps are required
	else
	{
		int min, max;
		panorama_map.GetJumpLimits(max, min);
		//If the viewpoint has gone "over" the limits of the jumping point, move the viewpoint over
		//to the other side of the map
		if (viewpoint_.x + x_move > max)
		{
			viewpoint_.updateViewpointLocationCnst(min, viewpoint_.y);
		}

		else if (viewpoint_.x + x_move < min)
		{
			viewpoint_.updateViewpointLocationCnst(max, viewpoint_.y);
		}

		//Otherwise update normally
		else
		{
			viewpoint_.updateViewpointLocation(x_move, y_move, panorama_map.GetWidth(), panorama_map.GetHeight(), panorama_map.GetMask(PanoramaMap::MASK_MAP));
		}
	}
	updateRotations();
	debug_timer_.StopTimer("updateViewpointLocation");
}


void PanoramaTracker::updateRotations()
{
	//Calculate the new x and y rotation values according to the location of the viewpoint
	x_rotation_ = pixelsToDegreesX(viewpoint_.x + viewpoint_.width / 2);
	float a = (viewpoint_.y + viewpoint_.height / 2);
	float perc_of_map = a / panorama_map.GetHeight();
	float p = perc_of_map * map_vertical_degrees_;
	y_rotation_ = (p - map_vertical_degrees_ / 2);
}

float PanoramaTracker::pixelsToDegreesX(float px) const
{
	//Percentage of total map_
	int fov; tracker_settings.Get(PT_CAMERA_FOV_HORIZONTAL, fov);
	float pixels_per_circle = (360.0 / (float)fov) * initial_img_width_;
	float single_px = 360.0 / (float)pixels_per_circle;
	float deg = (px - (panorama_map.GetWidth() / 2)) * single_px;
	return deg;
}

float PanoramaTracker::pixelsToDegreesY(float px) const
{
	float percentage = px / panorama_map.GetHeight();
	return 360 * percentage;
}


float PanoramaTracker::degreesToPixelsX(float degrees) const
{
	int fov; tracker_settings.Get(PT_CAMERA_FOV_HORIZONTAL, fov);
	float pixels_per_circle = (360 / (float)fov) * initial_img_width_;
	float single_px = 360 / (float)pixels_per_circle;
	float px = (degrees / single_px) + (panorama_map.GetWidth() / 2);
	return px;
}

float PanoramaTracker::degreesToPixelsY(float degrees, float corr) const
{
	float degrees_corr = degrees + corr;
	float percentage = degrees_corr / map_vertical_degrees_;
	return panorama_map.GetHeight() * percentage;
}