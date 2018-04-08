/*
 * MyVertex.h
 *
 *  Created on: Apr 10, 2013
 *      Author: okan
 */

#ifndef MYVERTEX_H_
#define MYVERTEX_H_

#include <string>
#include <vector>

class MyVertex {
public:
	MyVertex();
	MyVertex(double _x, double _y);
	virtual ~MyVertex();

	std::vector<MyVertex*> incoming;
	std::vector<MyVertex*> outgoing;

	int id;
	std::string name;
	double x;
	double y;
};

#endif /* MYVERTEX_H_ */
