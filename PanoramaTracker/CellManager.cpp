#include "CellManager.h"

CellManager::CellManager()
: cell_rows_(NO_OF_CELLS_Y),
cell_columns_(NO_OF_CELLS_X),
cell_width_(0),
cell_height_(0)
{
	initCellStatuses();
}

CellManager::CellManager(int cellWidth, int cellHeight)
: cell_rows_(NO_OF_CELLS_Y),
cell_columns_(NO_OF_CELLS_X),
cell_width_(cellWidth),
cell_height_(cellHeight)
{
	initCellStatuses();
}

int CellManager::GetCellHeight(MapSize mapSize) const
{
	if (mapSize ==  MAP_SIZE_FULL)
		return cell_height_;
	if (mapSize == MAP_SIZE_HALF)
		return cell_height_ / 2;
	return cell_height_ / 4;
}

int CellManager::GetCellWidth(MapSize mapSize) const
{
	if (mapSize == MAP_SIZE_FULL)
		return cell_width_;
	if (mapSize == MAP_SIZE_HALF)
		return cell_width_ / 2;
	return cell_width_ / 4;
}

Mat CellManager::GetCellContents(int x, int y, MapSize mapSize, const Mat &map) const
{
	//Get the actual cell width and height according to the size of the used map
	int cw, ch;
	if (mapSize == MAP_SIZE_FULL)
	{
		ch = cell_height_;
		cw = cell_width_;
	}
	else if (mapSize == MAP_SIZE_HALF)
	{
		ch = cell_height_ / 2;
		cw = cell_width_ / 2;
	}
	else
	{
		ch = cell_height_ / 4;
		cw = cell_width_ / 4;
	}
	int cell_starting_point_x = (x)* cw;
	int cell_starting_point_y = (y)* ch;

	return map(Rect(cell_starting_point_x, cell_starting_point_y, cw, ch));
}

void CellManager::initCellStatuses()
{
	for (int i = 0; i < cell_columns_; i++)
	{
		for (int j = 0; j < cell_rows_; j++)
		{
			cell_statuses_[i][j] = false;
		}
	}
}

void CellManager::UpdateCellStatus(int x, int y, Mat &mapMask)
{
	//If cell still has uninitialized pixels, status = false
	//Iterate through the cell
	bool black = false;
	uchar *p;
	for (int j = y * cell_height_; j < y* cell_height_ + cell_height_; j++){
		p = mapMask.ptr<uchar>(j);
		
		for (int i = x * cell_width_; i < x * cell_width_ + cell_width_; i++){
			if (p[i] == 0){
				cell_statuses_[x][y] = false;
				return;
			}
		}
	}
	cell_statuses_[x][y] = true;
}

bool CellManager::Status(int x, int y) const
{
	return cell_statuses_[x][y];
}

void CellManager::Status(int x, int y, bool status)
{
	cell_statuses_[x][y] = status;
}

bool CellManager::CellVisible(int x, int y, const Rect &currentViewpoint) const
{
	//Check that the cell is completely inside the viewpoint
	int cellX, cellY;
	cellX = x*cell_width_;
	cellY = y*cell_height_;
	return (cellX >= currentViewpoint.x && cellX + cell_width_ <= currentViewpoint.x + currentViewpoint.width
		&& cellY >= currentViewpoint.y && cellY + cell_height_ <= currentViewpoint.y + currentViewpoint.height);
}


std::vector<Point> CellManager::GetUnsetVisibleCells(const Rect &viewpoint) const
{
	std::vector<Point> cells;
	for (int i = 0; i < cell_columns_; i++)
	{
		for (int j = 0; j < cell_rows_; j++)
		{
			if (!Status(i, j) && CellVisible(i,j,viewpoint))
			{
				cells.push_back(Point(i, j));
			}
		}
	}
	return cells;
}

std::vector<PtFeature> CellManager::GetCellKeypoints(int x, int y, PtFeature::KeypointType KpType) const
{
	if (KpType == PtFeature::KP_FULL_MAP)
	{
		return cell_keypoints_[x][y];
	}
	else if (KpType == PtFeature::KP_HALF_MAP)
	{
		return cell_keypoints_half_[x][y];
	}
	else
	{
		return cell_keypoints_quarter_[x][y];
	}
}

std::vector<PtFeature>* CellManager::GetCellKeypointsPtr(int x, int y, PtFeature::KeypointType KpType)
{
	if (KpType == PtFeature::KP_FULL_MAP)
	{
		return &cell_keypoints_[x][y];
	}
	else if (KpType == PtFeature::KP_HALF_MAP)
	{
		return &cell_keypoints_half_[x][y];
	}
	else
	{
		return &cell_keypoints_quarter_[x][y];
	}
}

void CellManager::SetCellKeypoints(int x, int y, PtFeature::KeypointType kpType, const std::vector<PtFeature>& kps)
{
	if (kpType == PtFeature::KP_FULL_MAP)
	{
		cell_keypoints_[x][y] = kps;
	}
	else if (kpType == PtFeature::KP_HALF_MAP)
	{
		cell_keypoints_half_[x][y] = kps;
	}
	else
	{
		cell_keypoints_quarter_[x][y] = kps;
	}
}
