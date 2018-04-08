/*
 * MyGraph.h
 *
 *  Created on: Apr 10, 2013
 *      Author: okan
 */

#ifndef MYGRAPH_H_
#define MYGRAPH_H_

#include "MyEdge.h"
#include "MyVertex.h"
#include <vector>

class MyGraph {
public:
	MyGraph();
	virtual ~MyGraph();

	std::vector<MyVertex*> vertices;
	std::vector<MyEdge*> edges;

	void addVertex(MyVertex* vertex);
	MyEdge* addEdge(MyVertex* sourceV, MyVertex* targetV);

	MyVertex* getNearestVertex(MyVertex* randVertex);
};

#endif /* MYGRAPH_H_ */
