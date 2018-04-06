/*
 * MyRRTWrapper.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: okan
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <vector>

#include "MapHelper.h"
#include "MyRRTSolver.h"
#include "graph/MyVertex.h"
#include "graph/MyGraph.h"
#include "graph/MyEdge.h"

bool gotoPoseCompleted; // keeps the communication between goto_pose package
static int counter = 0; // required to draw with different indexes
std::string frame_id; // required for drawing rrt path on the map
bool useInitialPosition;
bool isTesting;

visualization_msgs::Marker createMarkerFromEdge(MyEdge* edge, int counter, float r, float g, float b, const char* _ns, float lineWidth)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
        
	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = _ns;
	marker.id = counter;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::LINE_STRIP;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	double x = edge->source->x;
	double y = edge->source->y;
	geometry_msgs::Point p1;
	p1.x = x;
	p1.y = y;
	marker.points.push_back(p1);
	x = edge->target->x;
	y = edge->target->y;
	geometry_msgs::Point p2;
	p2.x = x;
	p2.y = y;
	marker.points.push_back(p2);

	marker.pose.position.x = 0; //edge->source->x;
	marker.pose.position.y = 0; //edge->source->y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = lineWidth;
	marker.scale.y = 0.005;
	marker.scale.z = 0.005;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1.0;

	// markers are visible only for 10 minutes
	marker.lifetime = ros::Duration(600.0);

	return marker;
}

void sendVisualizationMessage(ros::Publisher& visualPublisher, MyRRTSolver& solver)
{
	std::cout << "send visualization graph edge size= " << solver.graph.edges.size() << std::endl;
	counter = 0;
	visualization_msgs::MarkerArray graphMarkerArray;

	for (int i=0; i < solver.graph.edges.size(); i++)
	{
		visualization_msgs::Marker marker = createMarkerFromEdge(solver.graph.edges[i], counter, 0.0f, 1.0f, 0.0f, "rrt", 0.01);
		graphMarkerArray.markers.push_back(marker);
		counter++;
	}

	visualPublisher.publish(graphMarkerArray);

	visualization_msgs::MarkerArray pathMarkerArray;

	// draw path
	for (int i = 0; i < solver.path.size()-1; i++)
	{
		MyEdge tempEdge(solver.path[i], solver.path[i+1]);
		visualization_msgs::Marker marker = createMarkerFromEdge(&tempEdge, counter, 1.0f, 0.0f, 0.0f, "path", 0.02);
		pathMarkerArray.markers.push_back(marker);
		counter++;
	}

	visualPublisher.publish(pathMarkerArray);

}

void gotoCommandCompletedCallback(const std_msgs::Bool& _completed)
{
	gotoPoseCompleted = _completed.data;
	std::cout << "gotoCommandCompletedCallback completed=" << gotoPoseCompleted << std::endl;
}

/**
 * note that this node is called as follows
 * rosrun myrrt myrrtwrapper _targetx:=0.1 _targety:=0.2 _frame_id:=map
 * if we want to visualize without using vrep and localization, we should set _frame_id:=world
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
	double initialx;
	double initialy;

	double targetx;
	double targety;

	//BEGIN: ROS
	ros::init(argc,argv,"myrrtwrapper");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	// read parameters
	ros::NodeHandle private_nh("~");
	if (!private_nh.getParam("initialx", initialx)){
		initialx = 0;
	}
	if (!private_nh.getParam("initialy", initialy)) {
		initialy = 0;
	}

	if (!private_nh.getParam("targetx", targetx)) {
		targetx = 0;
	}
	if (!private_nh.getParam("targety", targety)) {
		targety = 0;
	}

	if (!private_nh.getParam("frame_id", frame_id)) {
		frame_id = "odom";
	}

	if (!private_nh.getParam("use_initial", useInitialPosition)) {
		useInitialPosition = false;
	}

	if (!private_nh.getParam("testing", isTesting)) {
		isTesting = false;
	}

	std::cout << "initialx=" << initialx << " initialy=" << initialy << std::endl;
	std::cout << "targetx=" << targetx << " targety=" << targety << std::endl;
	std::cout << "frame_id=" << frame_id << std::endl;
	std::cout << "use_initial" << useInitialPosition << std::endl;

	// create graph publisher
	ros::Publisher graphPublisher = nh.advertise<visualization_msgs::MarkerArray>("/mygraph", 10);

	// create publishers for goto_pose package
	ros::Publisher gotoCommandPub = nh.advertise<geometry_msgs::Pose>("/gotoPose", 10);
	ros::Subscriber gotoCompletedSubs = nh.subscribe("/gotoPoseCompleted", 10, gotoCommandCompletedCallback);

	// create service client to get map from the map_server package
	ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
	nav_msgs::GetMap srvMap;
	nav_msgs::OccupancyGrid myMap;
	bool mapLoaded = false;
	while (!mapLoaded) {
		// get the map
		if (mapClient.call(srvMap))
		{
			std::cout << "map is retrieved successfully" << std::endl;
			const nav_msgs::OccupancyGrid& map(srvMap.response.map);
			myMap = map;
			mapLoaded = true;
		} else {
			ROS_ERROR("map retrieve error");
		}
		usleep(500000);
	}

	// create map helper class
	MapHelper mapHelper(myMap);

	// get the current pose if we do not provide initial position
	if (!useInitialPosition) {
		tf::TransformListener transformListener;
		tf::StampedTransform robotTransform;

		transformListener.waitForTransform("odom","base_link",
						ros::Time::now(),ros::Duration(3.0));
		transformListener.lookupTransform("odom","base_link",
							ros::Time(0),robotTransform);
		initialx = robotTransform.getOrigin().x();
		initialy = robotTransform.getOrigin().y();
	}

	// BEGIN: RRT solver
	// implement your own RRT algorithm by using class template MyRRTSolver
	// you are free to modify in any way you wanted to
	MyRRTSolver solver;
	solver.setMapHelper(&mapHelper);
	solver.solve(initialx, initialy, targetx, targety);

	// get pose commands from the RRT solver
	std::vector<geometry_msgs::Pose> poses = solver.getWayPoints();

	//END: RRT Solver

	//BEGIN: RVIZ VISUALIZATION
	sendVisualizationMessage(graphPublisher, solver);
	//END: RVIZ VISUALIZATION

	gotoPoseCompleted = true;

	// wait for 3 seconds to be make sure that connection is complete and messages will be sent successfully
	usleep(3000000);

	// the goto pose loop will run we do not test the rrt algorithm
	if (!isTesting) {
		// ros loop
		while(ros::ok())
		{
			// if goto pose completed we can send new goto pose command
			if (gotoPoseCompleted && poses.size() > 0) {
				geometry_msgs::Pose pose = poses.front();

				std::cout << "sending command pose.x=" << pose.position.x << " pose.y=" << pose.position.y << std::endl;

				gotoCommandPub.publish(pose);

				gotoPoseCompleted = false;

				// remove the first element
				poses.erase(poses.begin());
			}

			// if we traverse all way points quit the algorithm
			if (poses.size() == 0) {
				ros::shutdown();
			}

			ros::spinOnce();
			rate.sleep();
		}
	}

	//END: ros loop

	return 0;
}

