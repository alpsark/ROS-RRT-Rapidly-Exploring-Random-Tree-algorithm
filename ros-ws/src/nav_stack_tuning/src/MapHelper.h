/*
 * MapHelper.h
 *
 *  Created on: Apr 11, 2013
 *      Author: okan
 */

#ifndef MAPHELPER_H_
#define MAPHELPER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

class MapHelper
{
public:
	MapHelper(const nav_msgs::OccupancyGrid& _grid);

	virtual ~MapHelper();

	bool isOccupied(double startx, double starty, double endx, double endy);

	bool isOccupied(int rowIndex, int colIndex);

	bool isOccupied(double pointx, double pointy);

	int getRowIndex(double py);

	int getColIndex(double px);

	nav_msgs::OccupancyGrid occupancyGrid;
	double width, height, originx, originy, resolution;
};


#endif /* MAPHELPER_H_ */
