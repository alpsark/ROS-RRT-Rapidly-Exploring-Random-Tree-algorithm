/*
 * MyEdge.h
 *
 *  Created on: Apr 10, 2013
 *      Author: okan
 */

#ifndef MYEDGE_H_
#define MYEDGE_H_

#include "MyVertex.h"
#include <math.h>

class MyEdge {
public:
	MyEdge();
	MyEdge(MyVertex* _source, MyVertex* _target);
	virtual ~MyEdge();

	MyVertex* source;
	MyVertex* target;

	double distance;

	static double calDistance(MyVertex* v1, MyVertex* v2)
	{
		return sqrt(pow(v1->y - v2->y, 2) + pow(v1->x - v2->x, 2));
	}
};

#endif /* MYEDGE_H_ */
