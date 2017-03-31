#include "PtFeature.h"

int PtFeature::ids = 0;

PtFeature::PtFeature()
: pt_cell(Point(0,0)),
pt_map(Point(0,0)),
quality(0),
movement_x(-1000),
movement_y(-1000)
{
}

PtFeature::PtFeature(Point ptCell, Point ptMap)
: pt_cell(ptCell),
pt_map(ptMap),
quality(1),
movement_x(-1000),
movement_y(-1000)
{
	//Generate running id for each feature
	id = ids;
	ids++;
}

bool PtFeature::GetTemplate(int supportAreaSize, Mat map, Mat &supportArea) const
{
	Point suppAreaOrigin;
	suppAreaOrigin.x = pt_map.x - supportAreaSize / 2;
	suppAreaOrigin.y = pt_map.y - supportAreaSize / 2;

	//Check all conditions
	if (suppAreaOrigin.x >= 0 && suppAreaOrigin.x / 2 < map.cols
		&& suppAreaOrigin.y >= 0 && suppAreaOrigin.y < map.rows){
		//Extract the support area from the map
		supportArea = map(Rect(suppAreaOrigin.x, suppAreaOrigin.y, supportAreaSize, supportAreaSize));
		float movementX, movementY;
		float quality;
		return true;
	}
	return false;
}

int PtFeature::GetId() const{
	return id;
}