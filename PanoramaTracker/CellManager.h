#pragma once
#include "PtFeature.h"

/*
Class to hold info of every cell and manage their information and contents
*/
class CellManager
{
public:
	//Constructors
	CellManager();
	CellManager(int cellWidth, int cellHeight);
	
	//Get the size of the cells, dependant on the used size of the mapmapsize
	int GetCellHeight(MapSize mapSize) const;
	int GetCellWidth(MapSize mapSize) const;

	//Get the Mat of the cells contents from cell x,y
	Mat GetCellContents(int x, int y, MapSize mapSize,const Mat &map) const;

	//Update the status of the cell at x,y by checking if the pixels are set in the mask
	void UpdateCellStatus(int x, int y, Mat &mapMask);

	/*
	Return or set the status of the cell at x,y.
	The status is used to describe whetehr or not the cell is completely filled or not
	*/
	bool Status(int x, int y) const;
	void Status(int x, int y, bool status);

	//Check if the cell is completely within the current viewpoint
	bool CellVisible(int x, int y, const Rect &currentViewpoint) const; 

	//Get all cells (as x,y coordinates) that are visible, but unset
	std::vector<Point> GetUnsetVisibleCells(const Rect &viewpoint) const;

	//Functions for getting and setting keypoints in a cell x, y
	std::vector<PtFeature> GetCellKeypoints(int x, int y, PtFeature::KeypointType KpType) const;
	std::vector<PtFeature>* GetCellKeypointsPtr(int x, int y, PtFeature::KeypointType KpType);
	void SetCellKeypoints(int x, int y, PtFeature::KeypointType kpType, const std::vector<PtFeature> &kps);

private:
	//Cell information
	int cell_rows_;
	int cell_columns_;
	int cell_width_;
	int cell_height_;

	//Vectors containing all keypoints for each cell
	std::vector<PtFeature> cell_keypoints_[NO_OF_CELLS_X][NO_OF_CELLS_Y];
	std::vector<PtFeature> cell_keypoints_half_[NO_OF_CELLS_X][NO_OF_CELLS_Y];
	std::vector<PtFeature> cell_keypoints_quarter_[NO_OF_CELLS_X][NO_OF_CELLS_Y];

	//Statuses of all cells, i.e. are they completely filled with pixels or not
	bool cell_statuses_[NO_OF_CELLS_X][NO_OF_CELLS_Y];

	//Initialize all cell statuses to false
	void initCellStatuses();
};