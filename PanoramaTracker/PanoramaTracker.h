#pragma once

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include "HelpFunctions.h"
#include <chrono>
#include <memory>
#include <iomanip>

#include "PanoramaMap.h"
#include "ImageWarper.h"
#include "PtSettings.h"
#include "Relocalizer.h"
#include "DebugTimer.h"
#include "CellManager.h"
#include "Viewpoint.h"

#define MAP_WINDOW "Map"

using namespace cv;

//Panorama tracker class handles the tracking functionality, and also keeps track of 
//all the cells and their statuses
class PanoramaTracker
{

public:

	//Structure for a feature that has been matched. Contains the ID of the feature, as well as its x and y movements as a Point2d
	struct MatchedFeature{
		Point2d movements;
		int id;
	};
	
	enum TrackingStatus{ TRACKING_KEYPOINTS, RELOCALIZING, STOPPED };
	bool debug_match_templates = false;

	PtSettings tracker_settings;
	TrackingStatus tracking_status = TRACKING_KEYPOINTS;
	Relocalizer relocalizer;
	PanoramaMap panorama_map;

	//Constructors with default settings and with custom settings
	PanoramaTracker();
	PanoramaTracker(PtSettings settings);
	

	/*
	Init map_ by adding frame to middle of the map_. Set mapLoaded = true,
	if a map has been loaded using MapManager
	MAP LOADING IS DEPRECATED
	*/
	void InitializeMap(Mat frame, bool mapReady, bool mapLoaded = false);

	//View the map. Also return the map image for usage with plugins
	Mat ViewMap(MapSize mapSize, bool drawCells = true, bool drawKeypoints = false, bool showViewpoint = false, bool moveMap = false, bool onlyGet = false) const;	

	//Get the map image with viewpoint drawn. Used for Unity integration
	Mat GetMapImage() const;
	Mat GetMapImage(Size size) const;

	//Get the width of the shown map portion (when the map is moved instead of the viewpoint in ViewMap function)
	int GetViewMapWidth() const;

	//The function called each frame, which actually does the tracking and updates the orientation
	void CalculateOrientation(Mat currentFrame);

	/*
	Functions for setting an earlier relocalizer and panrama map.
	Used for loading a previously saved map
	MAP LOADING IS DEPRECATED
	*/
	void SetRelocalizer(const Relocalizer &rl);
	void SetPanoramaMap(const PanoramaMap &map);

	/*
	Get the estimated orientation of the tracker.
	Updated after each CalculateOrientation call
	*/
	float GetOrientationX() const;
	float GetOrientationY() const;
	float GetRotation() const;

	int GetOrientationXPixels() const;
	int GetOrientationYPixels() const;
	
	//Return the tracking quality as value 0..1 where 0 is best possible quality
	float GetQuality() const;

	//Input a custom orientation (used with Unity plugin)
	void SetOrientation(float x, float y, float z);

	//Function that returns a string containing debug data such as runtimes of certain functions
	std::string GetDebugData();

	/*
	Update a single columns data in the map by setting the mask of the column to unseen.
	Deprecated, since updating the map during runtime can cause drift 
	*/
	void UpdateColumn(Point clickedPoint);

private:

	CellManager cell_manager_;
	Viewpoint viewpoint_;
	ImageWarper warper_;
	DebugTimer debug_timer_;

	//Used template matching type, CV_TM_SQDIFF_NORMED is the one that seems to work best
	int template_matching_type = CV_TM_SQDIFF_NORMED;

	//Viewing angles of the map
	int map_vertical_degrees_ = 90;
	/*
	Currently very large, since it is required that the map can be moved 360 degrees in each of the directions
	from the starting position, so that the edge of the map is not reached until a 360 degree movement is certainly
	completed due to loop closing. 
	*/
	int map_horizontal_degrees = 820;

	//Visible degrees in the map when viewing
	int map_view_degrees_ = 360;

	//Calculated automatically
	int map_view_pixels_;

	//Width of the input image before warper scales it down
	int initial_img_width_;

	//Variables used in native android version
	double x_res_scaling_;
	double y_res_scaling_;
	
	//Used for the pixel<->degree conversion 
	int pixels_in_circle_x_;
	int pixels_in_circle_y_;

	/*
	changed_cells_ store info of cells whose status
	has switched during the last frame i.e. cells that have gone from 
	being non-completely filled to completely filled. Stored as x,y point
	*/
	std::vector<Point> changed_cells_;

	//Different versions of the currently input image
	Mat current_frame_;
	Mat current_frame_non_warped_;
	Mat current_frame_half_;
	Mat current_frame_quarter_;

	/*
	The quality and deviation during last frame. Set to 1000, but after
	the start quality is actually a value in range 0..1 and deviation 0..10
	*/
	float average_quality_ = 1000; 
	float deviation_ = 1000;

	//The orientations returned by the tracker
	float x_rotation_ = 0;
	float y_rotation_ = 0;
	float z_rotation_ = 0;

	//Variables used for determining when to do loop closing
	float min_rotation_ = 10000;
	float max_rotation_ = -10000;
	float min_rot_px_;
	float max_rot_px_;
	Mat max_rot_img_;
	Mat min_rot_img_;

	//Points used for rotation estimation
	std::vector<MatchedFeature> matched_features_;

	//How many keypoints were used and dropped last frame
	int used_kp_full_;
	int used_kp_half_;
	int used_kp_quarter_;
	int discarded_kp_full_;
	int discarded_kp_half_;
	int discarded_kp_quarter_;
	int removed_points_full_ = 0;
	int removed_points_half_ = 0;
	int removed_points_quarter_ = 0;

	//If using custom orientation (with SetOrientation)
	bool use_cstm_orientation_ = false;

	/*
	Used for determining when to update the map, since currently the map is not updated every frame.
	Instead the map is updated every x degrees, which avoids "ripples" from accumulating if the estimated
	orientation jumps from one value to another continuously
	*/
	float moved_deg_x_ = 0;	
	float moved_deg_y_ = 0;

	//Function called by both constructors
	void construct();

	//Get FAST keypoints
	std::vector<PtFeature> getKeypoints(int x, int y, MapSize mapSize);

	//Estimate the new orientation_ of the camera from mapSize map_. Outputs vectors of all xMovements, yMovements and qualities of keypoints
	void estimateOrientation(MapSize mapSize, std::vector<float> &xMovements, std::vector<float> &yMovements, std::vector<float> &qualities);

	//Track the movement and update viewpoitn on the mapSize. Return the standard deviation for quality estimation
	float trackAndUpdate(MapSize mapSize, std::vector<float> &allQualities);

	//Update the qualities of the features, and remove features that have too low quality
	void updateFeatures(MapSize mapSize, int medx, int medy);

	//Update the cell to see if it still contains keypoints, or if it shold be re-searched
	void updateCell(int x, int y);

	//Do template matching for comparable area and predicted position of the keypoint.
	//Output movement in x direction, movement in y direction and quality of the found template (max/min value of matchTemplate)
	bool matchTemplates(PtFeature &feature, MapSize mapSize, float &movementX, float &movementY, float &qualityVal);

	//Estimate the rotation around z-axis
	void estimateRotation();

	//Called every frame whenever the tracking is lost
	void Relocalize();
	
	//Update map_ using cell based or mask based extending
	void updateMap();

	//Update all current frame instances and warp the image
	void updateCurrentFrame(Mat frame);
	
	//Move viewpoint to correct position using orientation_pixels_x_
	void updateViewpointLocation(float x_move, float y_move);

	//Move the viewpoints to correct positions using the rotations
	void updateRotations();
	
	//Convert pixel rotation to degree rotation in either x or y axis
	float pixelsToDegreesX(float px) const;
	float pixelsToDegreesY(float px) const;
	
	//Convert degree rotation to pixel rotation in either x or y axis
	float degreesToPixelsX(float degrees) const;
	float degreesToPixelsY(float degrees, float corr) const;
};