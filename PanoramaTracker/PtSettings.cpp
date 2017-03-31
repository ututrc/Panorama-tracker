#include "PtSettings.h"


PtSettings::PtSettings()
{
	//Default settings
	camera_width_ = 640;
	camera_height_ = 480;
	camera_fov_horizontal_ = 49;
	camera_fov_vertical_ = 43;
	support_area_size_ = 8;
	support_area_search_size_full_ = 16;
	support_area_search_size_half_ = 16;
	support_area_search_size_quarter_ = 16;
	fast_keypoint_threshold_ = 12;
	max_keypoints_per_cell_ = 10;
	warper_scale_ = 407;
	max_dev_filtering_full_ = 6;
	max_dev_filtering_half_ = 4;
	max_dev_filtering_quarter_ = 2;
	min_tracking_quality_ = 0.1;
	min_relocalization_quality_ = 0.07;
	max_deviation_ = 6;
	use_colored_map_ = false;
	use_orb_ = false;
	use_android_shield_ = false;
	pyramidical_ = false;
	rotation_invariant_ = false;
}

void PtSettings::Set(SettingValue setting, double value)
{
	switch (setting)
	{
	case PT_CAMERA_WIDTH:
		camera_width_ = value;
		break;
	case PT_CAMERA_HEIGHT:
		camera_height_ = value;
		break;
	case PT_CAMERA_FOV_HORIZONTAL:
		camera_fov_horizontal_ = value;
		break;
	case PT_CAMERA_FOV_VERTICAL:
		camera_fov_vertical_ = value;
		break;
	case PT_SUPPORT_AREA_SIZE:
		support_area_size_ = value;
		break;
	case PT_SUPPORT_AREA_SEARCH_SIZE_FULL:
		support_area_search_size_full_ = value;
		break;
	case PT_SUPPORT_AREA_SEARCH_SIZE_HALF:
		support_area_search_size_half_ = value;
		break;
	case PT_SUPPORT_AREA_SEARCH_SIZE_QUARTER:
		support_area_search_size_quarter_ = value;
		break;
	case PT_FAST_KEYPOINT_THRESHOLD:
		fast_keypoint_threshold_ = value;
		break;
	case PT_MAX_DEVIATION:
		max_deviation_ = value;
		break;
	case PT_MAX_KEYPOINTS_PER_CELL:
		max_keypoints_per_cell_ = value;
		break;
	case PT_CELLS_X:
		number_of_cells_x_ = value;
		break;
	case PT_CELLS_Y:
		number_of_cells_y_ = value;
		break;
	case PT_WARPER_SCALE:
		warper_scale_ = value;
		break;
	case PT_USE_COLORED_MAP:
		use_colored_map_ = value;
		break;
	case PT_USE_ORB:
		use_orb_ = value;
		break;
	case PT_USE_ANDROID_SHIELD:
		use_android_shield_ = value;
		break;
	case PT_PYRAMIDICAL:
		pyramidical_ = value;
		break;
	case PT_ROTATION_INVARIANT:
		rotation_invariant_ = value;
		break;
	case PT_MIN_TRACKING_QUALITY:
		min_tracking_quality_ = value;
		break;
	case PT_MIN_RELOC_QUALITY:
		min_relocalization_quality_ = value;
		break;
	case PT_MAX_DEV_FILTERING_QUARTER:
		max_dev_filtering_quarter_ = value;
		break;
	case PT_MAX_DEV_FILTERING_HALF:
		max_dev_filtering_half_ = value;
		break;
	case PT_MAX_DEV_FILTERING_FULL:
		max_dev_filtering_full_ = value;
		break;
	default:
		std::cout << "Invalid setting type!" << std::endl;
		break;
	}
}


void PtSettings::Get(SettingValue setting, int& value) const
{
	switch (setting)
	{
	case PT_CAMERA_WIDTH:
		value = camera_width_;
		break;
	case PT_CAMERA_HEIGHT:
		value = camera_height_;
		break;
	case PT_CAMERA_FOV_HORIZONTAL:
		value = camera_fov_horizontal_;
		break;
	case PT_CAMERA_FOV_VERTICAL:
		value = camera_fov_vertical_;
		break;
	case PT_SUPPORT_AREA_SIZE:
		value = support_area_size_;
		break;
	case PT_SUPPORT_AREA_SEARCH_SIZE_FULL:
		value = support_area_search_size_full_;
		break;
	case PT_SUPPORT_AREA_SEARCH_SIZE_HALF:
		value = support_area_search_size_half_;
		break;
	case PT_SUPPORT_AREA_SEARCH_SIZE_QUARTER:
		value = support_area_search_size_quarter_;
		break;
	case PT_FAST_KEYPOINT_THRESHOLD:
		value = fast_keypoint_threshold_;
		break;
	case PT_MAX_KEYPOINTS_PER_CELL:
		value = max_keypoints_per_cell_;
		break;
	case PT_MAX_DEVIATION:
		value = max_deviation_;
		break;
	case PT_CELLS_X:
		value = number_of_cells_x_;
		break;
	case PT_CELLS_Y:
		value = number_of_cells_y_;
		break;
	case PT_WARPER_SCALE:
		value = warper_scale_;
		break;
	case PT_MAX_DEV_FILTERING_QUARTER:
		value = max_dev_filtering_quarter_;
		break;
	case PT_MAX_DEV_FILTERING_HALF:
		value = max_dev_filtering_half_;
		break;
	case PT_MAX_DEV_FILTERING_FULL:
		value = max_dev_filtering_full_;
		break;
	default:
		std::cout << "Invalid setting type!" << std::endl;
		break;
	}
}

void PtSettings::Get(SettingValue setting, bool& value) const
{
	switch (setting)
	{
	case PT_USE_COLORED_MAP:
		value = use_colored_map_;
		break;
	case PT_USE_ORB:
		value = use_orb_;
		break;
	case PT_USE_ANDROID_SHIELD:
		value = use_android_shield_;
		break;
	case PT_PYRAMIDICAL:
		value = pyramidical_;
		break;
	case PT_ROTATION_INVARIANT:
		value = rotation_invariant_;
		break;
	default:
		std::cout << "Invalid setting type!" << std::endl;
		break;
	}
}

void PtSettings::Get(SettingValue setting, float& value) const
{
	switch (setting)
	{
	case PT_MIN_TRACKING_QUALITY:
		value = min_tracking_quality_;
		break;
	case PT_MIN_RELOC_QUALITY:
		value = min_relocalization_quality_;
		break;
	default:
		std::cout << "Invalid setting type!" << std::endl;
		break;
	}
}