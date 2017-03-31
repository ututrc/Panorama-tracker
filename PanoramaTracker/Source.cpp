#include "PanoramaTracker.h"
#include <chrono>
#include <opencv2/highgui.hpp>
#define MAIN_WINDOW "Camera"
#define SETTINGS_WINDOW "Settings"
//#define USE_WEBCAM

bool isVideoPlaying = false;

bool updating_column = false;
//Mouse input functions related to UpdateColumn
std::vector<Point> clicks;
void mouseCallback(int event, int x, int y, int flags, void* point)
{
	Point p;
	if ((event == EVENT_LBUTTONDOWN || event == EVENT_LBUTTONUP) && updating_column)
	{
		p.x = x;
		p.y = y;
		clicks.push_back(p);
	}
}

/*
Sample usage of Panorama Tracker native
*/
int main()
{
	//Create the windows for settings and image
	namedWindow(MAIN_WINDOW);
	namedWindow(SETTINGS_WINDOW);

	//Settings for usage, could also use default settings
	int search_full = 22;
	int search_half = 16;
	int search_quarter = 16;
	int max_keypoints_per_cell = 10;
	int min_tracking_quality = 30;
	int min_reloc_quality = 7;
	int prnt_fps_frames = 50;
	int diff_tot = 0;
	bool mousecallback_set = false;
	//Trackbars for settings in the settings window
	createTrackbar("SS half", SETTINGS_WINDOW, &search_half, 100);
	createTrackbar("SS full", SETTINGS_WINDOW, &search_full, 100);
	createTrackbar("SS quart", SETTINGS_WINDOW, &search_quarter, 100);
	createTrackbar("max kp/cell", SETTINGS_WINDOW, &max_keypoints_per_cell, 100);
	createTrackbar("min q*100: ", SETTINGS_WINDOW, &min_tracking_quality, 300);
	createTrackbar("min r q*100: ", SETTINGS_WINDOW, &min_reloc_quality, 100);
	
	//Create the settings object for the tracker
	PtSettings settings;
	settings.Set(PT_CAMERA_FOV_VERTICAL, 43);
	settings.Set(PT_USE_COLORED_MAP, false);
	settings.Set(PT_MAX_KEYPOINTS_PER_CELL, max_keypoints_per_cell);
	settings.Set(PT_SUPPORT_AREA_SIZE, 8);
	settings.Set(PT_FAST_KEYPOINT_THRESHOLD, 12);
	settings.Set(PT_WARPER_SCALE, 400);
	settings.Set(PT_PYRAMIDICAL, false);
	settings.Set(PT_CAMERA_FOV_HORIZONTAL, 57);
	settings.Set(PT_USE_ANDROID_SHIELD, true);
	settings.Set(PT_ROTATION_INVARIANT, true);

	VideoCapture cap; 
	PanoramaTracker new_tracker;
	MapSize mapSize = MAP_SIZE_FULL;
	//If using live capture from the webcamera
#ifdef USE_WEBCAM
	cap.open(0);
	cap.set(CV_CAP_PROP_SETTINGS, 1);
	//cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	settings.Set(PT_CAMERA_FOV_HORIZONTAL, 55);
	//settings.Set(PT_CAMERA_FOV_HORIZONTAL, 70);
	//settings.Set(PT_USE_ANDROID_SHIELD, true);
	settings.Set(PT_ROTATION_INVARIANT, true);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);
vid_end:
#endif
	//If using a video as input
#ifndef USE_WEBCAM
vid_end:
	cap.open("Videos/rot/rot2.mp4");
#endif
	//Initialize tracker object
	PanoramaTracker pt(settings);
	Mat frame;
	bool initialized = false;
	bool show_keypoints = false;
	bool show_cells = true;

	//When using video input, start tracking in the first frame. 
	//With live input, the user can initialize by pressing space
#ifndef USE_WEBCAM
	cap >> frame;
	pt.InitializeMap(frame, false);
	initialized = true;
#endif

	//Main loop for tracking
	while (true)
	{
		cap >> frame;
		if (frame.data){ 
			imshow(MAIN_WINDOW, frame);
			if (initialized){
				//Set the settings, in case they have been adjusted from the sliders
				pt.tracker_settings.Set(PT_SUPPORT_AREA_SEARCH_SIZE_FULL, search_full);
				pt.tracker_settings.Set(PT_SUPPORT_AREA_SEARCH_SIZE_HALF, search_half);
				pt.tracker_settings.Set(PT_SUPPORT_AREA_SEARCH_SIZE_QUARTER, search_quarter);
				pt.tracker_settings.Set(PT_MIN_TRACKING_QUALITY, (float)min_tracking_quality / 100);
				pt.tracker_settings.Set(PT_MIN_RELOC_QUALITY, (float)min_reloc_quality / 100);

				pt.ViewMap(mapSize, show_cells, show_keypoints, true, true);

				//Setupping the updatable map (click on the map to update the image in that column of cells)
				if (!mousecallback_set)
				{
					setMouseCallback("Map", mouseCallback);
				}
				if (updating_column && clicks.size() > 0)
				{
					pt.UpdateColumn(clicks.at(0));
					clicks.resize(0);
					updating_column = false;
				}

				//Actual tracking function call
				pt.CalculateOrientation(frame);
			}
		}
		//Restart video playback
		else goto vid_end;

		//Print avg frame duration of last 50 frames
		prnt_fps_frames++;
		if (prnt_fps_frames > 50)
		{
			prnt_fps_frames = 0;
			std::cout << "Avg frame duration in last 50 frames: " << diff_tot / 50 << std::endl;
			diff_tot = 0;
		}

		//Handle user keyboard inputs
		auto key = cvWaitKey(1);
		switch (key)
		{
		//Escape to close
		case 27:
			return 0;
		//Space to initialize when live tracking
#ifdef USE_WEBCAM
		case 32:
			if (!initialized)
			{
				pt.InitializeMap(frame, false);
				initialized = true;
			}
			else
			{
				pt.CalculateOrientation(frame);
			}
			break;
#endif
		//c to view cells in the map image
		case 99:
			show_cells = !show_cells;
			break;
		//d to turn on debug mode for template matching
		case 100:
			pt.debug_match_templates = !pt.debug_match_templates;
			break;
		//m to change the viewed map size, if pyramidical mode is on
		//Pyramicidal mode is currently deprecated
		case 109:
			switch (mapSize)
			{
			case MAP_SIZE_FULL:
				mapSize = MAP_SIZE_HALF;
				break;
			case MAP_SIZE_HALF:
				mapSize = MAP_SIZE_QUARTER;
				break;
			case MAP_SIZE_QUARTER:
				mapSize = MAP_SIZE_FULL;
				break;
			}
			break;
		//r to reset the tracker when using live tracking mode
		case 114:
			initialized = false;
			settings.Set(PT_MAX_KEYPOINTS_PER_CELL, max_keypoints_per_cell);
			pt = PanoramaTracker(settings);
			break;
		//x to show keypoints
		case 120:
			show_keypoints = !show_keypoints;
			break;
		//z to activate updating a column. Click on map to update that column of cells.
		case 122:
			std::cout << "Click on map to update a column of data" << std::endl;
			updating_column = true;
			clicks.resize(0);
			break;
		}
	}
}