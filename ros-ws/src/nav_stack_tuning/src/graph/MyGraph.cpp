/*
 * MyGraph.cpp
 *
 *  Created on: Apr 10, 2013
 *      Author: okan
 */

#include "MyGraph.h"

MyGraph::MyGraph() {
	// TODO Auto-generated constructor stub

}

MyGraph::~MyGraph() {
	// TODO Auto-generated destructor stub
}

void MyGraph::addVertex(MyVertex* vertex)
{
	vertices.push_back(vertex);
	vertex->id = vertices.size()-1;
}

MyEdge* MyGraph::addEdge(MyVertex* sourceV, MyVertex* targetV)
{
	MyEdge *edge = new MyEdge(sourceV, targetV);
	edges.push_back(edge);
	return edge;
}

MyVertex* MyGraph::getNearestVertex(MyVertex* randVertex)
{
	double minDist = 10000;
	MyVertex* minVertex = NULL;
	for (int i = 0; i < vertices.size(); i++)
	{
		double dist = MyEdge::calDistance(vertices[i], randVertex);
		if (dist < minDist && dist != 0) {
			minVertex = vertices[i];
			minDist = dist;
		}
	}

	return minVertex;
}
