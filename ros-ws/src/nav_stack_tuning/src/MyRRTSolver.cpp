/*
 * MyRRTSolver.cpp
 *
 *  Created on: Apr 10, 2013
 *      Author: okan
 */

#include "MyRRTSolver.h"
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <sstream>
#include <vector>
MyRRTSolver::MyRRTSolver() {
	initialx = 0;
	initialy = 0;
	initialVertex = NULL;

	targetx = 4;
	targety = 4.5;
	targetVertex = NULL;

	minx = -10;
	maxx = 10;

	miny = -10;
	maxy = 10;

	targetCloseEnough = 0.1;

	incDistance = 0.1;

	srand(time(NULL));
}

MyRRTSolver::~MyRRTSolver() {

}

/**
 * sets the map helper class
 * @param _mapHelper
 */
void MyRRTSolver::setMapHelper(MapHelper* _mapHelper)
{
	mapHelper = _mapHelper;
}

/**
 * sets initial and target positions, then builds and constructs path
 * @param _initialx
 * @param _initialy
 * @param _targetx
 * @param _targety
 */
void MyRRTSolver::solve(double _initialx, double _initialy, double _targetx, double _targety)
{
	initialx = _initialx;
	initialy = _initialy;

	targetx = _targetx;
	targety = _targety;

	build();
	constructPath();
}

/**
 * a helper method to generate uniform random double number in the range of min and max
 * @param min
 * @param max
 * @return
 */
double MyRRTSolver::getRandomInRange(double min, double max)
{
	double u = ((double) rand() / (RAND_MAX));
	u = u * (max - min);
	u = u + min;
	return u;
}

/**
 * creates a random configuration
 * @return
 */
MyVertex* MyRRTSolver::getRandomConf()
{
// DO: implement random configuration function
// DO: create x and y sample values by using getRandomInRange method.
// make sure that you choose values in map range (x:-5,+5, y:-10,+10)

  MyVertex* randVertex =new MyVertex(getRandomInRange(-4.5,4.5),getRandomInRange(-7.5,7.5)) ;
  return randVertex;
}
/**
 * Check line occupancy of two indexes (True means there is collision)
 */

bool MyRRTSolver::bhm_line(int x1,int y1,int x2,int y2)// y rowIndex  x Colindex
{
 // std::cout << "1 : " << x1 << "," << y1 <<  std::endl; 
 //td::cout << "2 : " << x2 << "," << y2 <<  std::endl; 
 int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
 bool pixel_collision = false ;
 dx=x2-x1;
 dy=y2-y1;
 dx1=fabs(dx);
 dy1=fabs(dy);
 px=2*dy1-dx1;
 py=2*dx1-dy1;

  pixel_collision = (*mapHelper).isOccupied( y1 , x1) ; //check collision
	  if(pixel_collision){return pixel_collision;}

  pixel_collision = (*mapHelper).isOccupied( y2 , x2) ; //check collision
	  if(pixel_collision){return pixel_collision;}

 if(dy1<=dx1) {  //x orianted
  if(dx>=0){//start 1 or 2
   x=x1;
   y=y1;
   xe=x2;
  }
  else{
   x=x2;
   y=y2;
   xe=x1;
  }


  for(i=0;x<xe;i++){ //now increasse x and y
   x=x+1;
   if(px<0){
    px=px+2*dy1;
   }
   else {
    if((dx<0 && dy<0) || (dx>0 && dy>0)) {
     y=y+1;
    }
    else {
     y=y-1;
    }
    px=px+2*(dy1-dx1);
   }

    pixel_collision = (*mapHelper).isOccupied(y , x) ; //check collision
	  if(pixel_collision){return pixel_collision;}
  }
 }
 else { //y orianted

  if(dy>=0) {//start 1 or 2
   x=x1;
   y=y1;
   ye=y2;
  }
  else {
   x=x2;
   y=y2;
   ye=y1;
  }

  for(i=0;y<ye;i++){
   y=y+1;
   if(py<=0){
    py=py+2*dx1;
   }
   else{
    if((dx<0 && dy<0) || (dx>0 && dy>0)) {
     x=x+1;
    }
    else {
     x=x-1;
    }
    py=py+2*(dx1-dy1);
   }

    pixel_collision = (*mapHelper).isOccupied(y , x) ; //check collision
	  if(pixel_collision){return pixel_collision;}
  }
 }
	return pixel_collision;
}
/**
 * Builds the RRT tree and exits when a vertice is close enough to targetPosition
 * threshold is given by targetCloseEnough
 */
void MyRRTSolver::build()
{
// TODO: implement random tree building function
// NOTE: you are supposed to add vertices and edges to graph data structure

  // create initial vertex at initial position
  initialVertex = new MyVertex(initialx, initialy);
  targetVertex = new MyVertex(targetx, targety);
  // add initial vertex to graph
  graph.addVertex(initialVertex);
  bool Nonvalid ;

  //check validity of the target
  MyVertex* newValidVertex = new MyVertex(targetx, targety);

	Nonvalid = (*mapHelper).isOccupied(newValidVertex->x , newValidVertex->y);
	 if(Nonvalid){	std::cout << "Target Non valid " << std::endl; return ;}else {std::cout << "Target valid " << std::endl;}
  	//check validity of the inital
  newValidVertex = new MyVertex(initialx, initialy);
	Nonvalid = (*mapHelper).isOccupied(newValidVertex->x , newValidVertex->y);
	 if(Nonvalid){	std::cout << "Initial Non valid " << std::endl; return ;}else {std::cout << "Initial valid " << std::endl;}
   int i = 0 ;

  // DO: construct the RRT tree until the newly created node is close enough the target position and can have valid edge
	while ((MyEdge::calDistance(targetVertex,newValidVertex) > targetCloseEnough) || bhm_line((*mapHelper).getColIndex(newValidVertex->x), (*mapHelper).getRowIndex(newValidVertex->y),(*mapHelper).getColIndex(targetVertex->x), (*mapHelper).getRowIndex(targetVertex->y)) ){
     //first get a random vertex
    newValidVertex = getRandomConf() ; 
	double step_size = 0.5 ; 
	if(MyEdge::calDistance(newValidVertex , graph.getNearestVertex(newValidVertex)) > step_size){
    double m = ((graph.getNearestVertex(newValidVertex)->y) - newValidVertex->y   )/((graph.getNearestVertex(newValidVertex)->x) - newValidVertex->x)	;

	double stepX = step_size  / sqrt(1 + m * m) ;
	newValidVertex = new MyVertex(stepX , m * stepX) ; //index corrected with smaller step same direction
	   }
	   //check if edge is valid (dont go outside map) Bresenhams-Line-Algorithm (it also checks the point itself)
	   Nonvalid = bhm_line((*mapHelper).getColIndex(newValidVertex->x), (*mapHelper).getRowIndex(newValidVertex->y),(*mapHelper).getColIndex(graph.getNearestVertex(newValidVertex)->x), (*mapHelper).getRowIndex(graph.getNearestVertex(newValidVertex)->y)) ; 
			 if(Nonvalid){ 
				std::cout << "Vertex not valid: (" <<newValidVertex->x << "'"<< newValidVertex->y << ")" << std::endl; 
					continue;}
       //no collision now add it to graph
        

       graph.addVertex(newValidVertex);
       graph.addEdge(graph.getNearestVertex(newValidVertex),newValidVertex);	 
 	   std::cout << "Vertex created in: (" << newValidVertex->x << "," <<newValidVertex->y << ")"<< std::endl ; 
	
	} 

       graph.addVertex(targetVertex);
       graph.addEdge(newValidVertex,targetVertex);	
	     std::cout << "Tree constructed " << std::endl;  

}

struct MatchVertex
{
 MatchVertex(const MyVertex* v) : v_(v) {}
 bool operator()(const MyEdge* obj) const
 {
   return obj -> target == v_;
 }
 private:
   const MyVertex* v_;
};

/**
 * by traversing from target to initial configuration fills in path vector
 */
void MyRRTSolver::constructPath()
{
	//DO: by traversing from target to initial configuration fill in path vector
	//path.push_back(targetVertex);
	//path.push_back(closestVertex);
	//MyVertex* source  ; //= graph.getNearestVertex(targetVertex);
	MyVertex* target = targetVertex ;


	//std::vector<std::vector<MyVertex*> > alternating_paths ; 
	//std::vector<std::vector<MyVertex*> > new_alternating_paths ; 
	//std::vector<MyVertex*>  iter_path;
	//alternating_paths.push_back(path);
	//new_alternating_paths.push_back(path);
	//std::vector< std::vector<MyVertex*> >::iterator iter;

	//int count = 2 ;
	while(target != initialVertex){
		//we only have one source node for each target node
		std::vector<MyEdge*>::iterator it = std::find_if(graph.edges.begin(), graph.edges.end(), MatchVertex(target ) );
		path.push_back(target)  ; 
		target = it[0] -> source ; 

		/*alternating_paths=	new_alternating_paths;  
		new_alternating_paths.clear() ; 
		count ++ ;
        std::cout << count <<  '\n';
		for(iter = alternating_paths.begin(); iter != alternating_paths.end() ; iter ++){	
	    std::vector<MyEdge*>::iterator it = graph.edges.begin();
		iter_path = *iter;
	    //find all nodes with given target
	    while((it = std::find_if(it, graph.edges.end(), MatchVertex(iter_path[iter_path.size()-1]) )) != graph.edges.end()){
        	path = *iter  ;		
            source = it[0] -> source; 
            target = it[0] -> target; 
	        path.push_back(source) ;
	        new_alternating_paths.push_back(path) ;			
	        it ++ ;
			if(source == initialVertex){std::cout << "Huzzah: " <<  '\n'; found = true ;  break;}
	   		}
		}*/


	
	}
		path.push_back(target)  ; 
	// print the calculated path for debugging purpose
	std::cout << "----path----- " << std::endl;
	for (int i = 0; i < path.size(); i++)
	{
		std::cout << "(" << path[i]->x << ", " << path[i]->y << ")" << std::endl;
	}
}

/**
 * converts calculated path way points to geometry_msgs type
 * @return
 */
std::vector<geometry_msgs::Pose> MyRRTSolver::getWayPoints()
{
  // generate way points in reverse order
	std::vector<geometry_msgs::Pose> wayPoints;
	for (int i = path.size()-1; i >= 0; i--)
	{
		geometry_msgs::Pose pose;
		pose.position.x = path[i]->x;
		pose.position.y = path[i]->y;
		wayPoints.push_back(pose);
	}

	return wayPoints;
}
