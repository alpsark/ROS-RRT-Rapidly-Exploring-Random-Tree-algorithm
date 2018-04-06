/*
 * MyRRTSolver.h
 *
 *  Created on: Apr 10, 2013
 *      Author: okan
 */

#ifndef MYRRTSOLVER_H_
#define MYRRTSOLVER_H_

#include "graph/MyVertex.h"
#include "graph/MyGraph.h"
#include "MapHelper.h"

#include <geometry_msgs/Pose.h>
#include <vector>
#include <stack>

class MyRRTSolver {
public:
	MyRRTSolver();
	virtual ~MyRRTSolver();

	MyGraph graph;

	double initialx;
	double initialy;
	MyVertex* initialVertex;

	double targetx;
	double targety;
	MyVertex* targetVertex;

	double targetCloseEnough;

	double maxx;
	double minx;

	double maxy;
	double miny;

	double incDistance;

	MapHelper* mapHelper;
	std::vector<MyVertex*> path;

        bool bhm_line(int x1,int y1,int x2,int y2);//rowIndex  Colindex
	void setMapHelper(MapHelper* _mapHelper);

	double getRandomInRange(double min, double max);
	MyVertex* getRandomConf();

	void build();
	void solve(double _initialx, double _initialy, double _targetx, double _targety);
	void constructPath();

	std::vector<geometry_msgs::Pose> getWayPoints();

};

#endif /* MYRRTSOLVER_H_ */
