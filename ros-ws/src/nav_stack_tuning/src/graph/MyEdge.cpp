/*
 * MyEdge.cpp
 *
 *  Created on: Apr 10, 2013
 *      Author: okan
 */

#include "MyEdge.h"

MyEdge::MyEdge() {
	// TODO Auto-generated constructor stub

}

MyEdge::MyEdge(MyVertex* _source, MyVertex* _target)
{
	source = _source;
	target = _target;
	distance = calDistance(source, target);

	source->outgoing.push_back(target);
	target->incoming.push_back(source);
}

MyEdge::~MyEdge() {
	// TODO Auto-generated destructor stub
}
