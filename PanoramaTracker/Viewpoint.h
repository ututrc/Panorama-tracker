#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

enum MapSize{ MAP_SIZE_FULL, MAP_SIZE_HALF, MAP_SIZE_QUARTER };

using namespace cv;
/*
Contains the information about the viewpoint (the estimated position of the camera over the panorama image)
*/
class Viewpoint
{
public:
	//x and y coordinates of top left corner of the largest viewpoint
	int x, y;				
	
	//Width and height of the viewpoint
	int height, width;

	//Constructors
	Viewpoint();
	Viewpoint(int x, int y, int width, int height);
	
	//Get the viewpoint as a rectangle according to the size of the used map
	Rect GetViewpoint(MapSize mapSize) const;

	//The possibility of size changing when rotation invariant
	void UpdateViewpointSize(int width, int height, int mapWidth, int mapHeight);
	
	//Move viewpoint by x_move and y_move
	void updateViewpointLocation(float x_move, float y_move, int map_width, int map_height, const Mat &mask_image);

	//Move viewpoint to a specific location
	void updateViewpointLocationCnst(float x, float y);


	//Draw viewpoint on top of the map image (image), used in the Unity3D plugin
	void DrawViewpoint(Mat &image, MapSize mapSize) const;

private:

	//The image currently inside the viewpoint
	void updateViewpointContentsMask(const Mat &maskImage);
};