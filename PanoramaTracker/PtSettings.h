#pragma once
#include <iostream>
//Define for removing all console logging and imshows
//Mainly to be used in the Android plugin for Unity
#define DEBUG_BUILD 

enum SettingValue
{
	PT_CAMERA_WIDTH, PT_CAMERA_HEIGHT, PT_CAMERA_FOV_HORIZONTAL, PT_CAMERA_FOV_VERTICAL,
	PT_SUPPORT_AREA_SIZE, PT_SUPPORT_AREA_SEARCH_SIZE_FULL, PT_SUPPORT_AREA_SEARCH_SIZE_HALF,
	PT_SUPPORT_AREA_SEARCH_SIZE_QUARTER, PT_FAST_KEYPOINT_THRESHOLD, PT_MAX_KEYPOINTS_PER_CELL,
	PT_USE_COLORED_MAP, PT_USE_ORB, PT_MIN_TRACKING_QUALITY, PT_MAX_DEVIATION, PT_MIN_RELOC_QUALITY,
	PT_CELLS_X, PT_CELLS_Y, PT_USE_ANDROID_SHIELD, PT_WARPER_SCALE, PT_PYRAMIDICAL, PT_ROTATION_INVARIANT,
	PT_MAX_DEV_FILTERING_FULL, PT_MAX_DEV_FILTERING_HALF, PT_MAX_DEV_FILTERING_QUARTER
};

/*
Class for storing all the different setting values of the tracker
*/
class PtSettings
{
public:
	
	PtSettings();
	void Set(SettingValue setting, double value);
	void Get(SettingValue setting, int &value) const;
	void Get(SettingValue setting, bool &value) const;
	void Get(SettingValue setting, float &value) const;
private:
	int number_of_cells_x_;
	int number_of_cells_y_;

	int camera_width_;
	int camera_height_;
	int camera_fov_horizontal_;
	int camera_fov_vertical_;
	int support_area_size_;
	int support_area_search_size_full_;
	int support_area_search_size_half_;
	int support_area_search_size_quarter_;
	int fast_keypoint_threshold_;
	int max_keypoints_per_cell_;
	int warper_scale_;
	int max_deviation_;
	int max_dev_filtering_full_;
	int max_dev_filtering_half_;
	int max_dev_filtering_quarter_;
	float min_tracking_quality_;
	float min_relocalization_quality_;
	bool use_colored_map_;
	bool use_orb_;
	bool use_android_shield_;
	bool pyramidical_;
	bool rotation_invariant_;
};