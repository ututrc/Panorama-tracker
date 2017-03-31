#include "Viewpoint.h"
#include <opencv2/video/tracking.hpp>

Viewpoint::Viewpoint()
: x(0), y(0), height(0), width(0)
{
	
}

Viewpoint::Viewpoint(int x, int y, int width, int height)
: x(x), y(y), height(height), width(width)
{
	
}

Rect Viewpoint::GetViewpoint(MapSize mapSize) const
{
	if (mapSize == MAP_SIZE_FULL)
	{
		return Rect(x, y, width, height);
	}
	else if (mapSize == MAP_SIZE_HALF)
	{
		return Rect(x/2, y/2, width/2, height/2);
	}
	else
	{
		return Rect(x/4, y/4, width/4, height/4);
	}
}

void Viewpoint::UpdateViewpointSize(int width, int height, int mapWidth, int mapHeight)
{
	this->height = height;
	this->width = width;
}

void Viewpoint::updateViewpointLocation(float x_move, float y_move, int map_width, int map_height, const Mat &mask_image)
{	
	//Limit viewpoint from moving out of image borders
	if (x + round(x_move) >= 0 && x + round(x_move) + width <= map_width)
	{
		x += round(x_move);
	}

	if (y + round(y_move) > 0 && y + round(y_move) + height < map_height)
	{
		y += round(y_move);
	}
}

void Viewpoint::updateViewpointLocationCnst(float x, float y)
{
	this->x = x;
	this->y = y;
}


void Viewpoint::DrawViewpoint(Mat& image, MapSize mapSize) const
{
	rectangle(image, GetViewpoint(mapSize), Scalar(0, 0, 255), 2);
}

