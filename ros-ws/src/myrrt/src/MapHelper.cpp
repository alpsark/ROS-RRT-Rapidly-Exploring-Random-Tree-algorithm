/*
 * MapHelper.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: okan
 */

#include "MapHelper.h"

MapHelper::MapHelper(const nav_msgs::OccupancyGrid& _grid) {
	occupancyGrid = _grid;

	// debug tests
	std::cout << "origin.x=" << occupancyGrid.info.origin.position.x << " origin.y="
			<< occupancyGrid.info.origin.position.y << std::endl;
	std::cout << "resolution=" << occupancyGrid.info.resolution << std::endl;
	std::cout << "width=" << occupancyGrid.info.width << " height=" << occupancyGrid.info.height << std::endl;

	resolution = occupancyGrid.info.resolution;
	width = occupancyGrid.info.width;
	height = occupancyGrid.info.height;
	originx = occupancyGrid.info.origin.position.x;
	originy = occupancyGrid.info.origin.position.y;
}

MapHelper::~MapHelper()
{
}

bool MapHelper::isOccupied(int rowIndex, int colIndex)
{
	int linearIndex = rowIndex * width + colIndex;
	int occupancy = (int)occupancyGrid.data[linearIndex];
	for(int i = -10; i < 11 ; i++) {//0.2 inflaiton
		for(int j = -10; j < 11 ; j++) {
                        linearIndex = (rowIndex+j) * width + (colIndex+i);
			occupancy = (int)occupancyGrid.data[linearIndex];
			if (occupancy > 0 || occupancy == -1)
			{
				return true;
			}
		}
	}
	return false;
}

bool MapHelper::isOccupied(double pointx, double pointy)
	{
		pointx = pointx - originx;
		pointy = pointy - originy;
		int row = floor(pointy/resolution);
		int col = floor(pointx/resolution);
		int linearIndex = row * width + col;
		int occupancy = (int)occupancyGrid.data[linearIndex];
	for(int i = -10; i < 11 ; i++) {//0.2 inflaiton
		for(int j = -10; j < 11 ; j++) {
                        linearIndex = (row+j) * width + (col+i);
			occupancy = (int)occupancyGrid.data[linearIndex];
			if (occupancy > 0 || occupancy == -1)
			{
				return true;
			}
		}
	}
	return false;
	}

bool MapHelper::isOccupied(double startx, double starty, double endx, double endy)
	{
		return false;
	}

int MapHelper::getRowIndex(double py) {
		py = py - originy;
		int row = floor(py/resolution);
		return row;
	}

int MapHelper::getColIndex(double px) {
		px = px - originx;
		int col = floor(px/resolution);
		return col;
	}
