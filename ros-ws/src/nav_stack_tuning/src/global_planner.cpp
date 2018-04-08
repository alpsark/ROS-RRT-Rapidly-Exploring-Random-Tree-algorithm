	#include <pluginlib/class_list_macros.h>
	#include "global_planner.h"
	#include "math.h"
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

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

 //using namespace std;
 bool gotoPoseCompleted = true;
 static int counter = 0; // required to draw with different indexes
 bool isTesting;
 bool path_created = false;

	double initialx;
	double initialy;

	double targetx;
	double targety;

 std::vector<geometry_msgs::Pose> poses ;

visualization_msgs::Marker createMarkerFromEdge(MyEdge* edge, int counter, float r, float g, float b, const char* _ns, float lineWidth)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom";
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


 //Default Constructor
 namespace global_planner {

 GlobalPlanner::GlobalPlanner (){

 }

 GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

 }

 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
  plan.push_back(start);
  
	//BEGIN: ROS
	//ros::init(argc,argv,"global_planner");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	if(!path_created){
	initialx = start.pose.position.x;
	initialy = start.pose.position.y;
	targetx = goal.pose.position.x;
	targety = goal.pose.position.y;
	//frame_id = "odom";
	//useInitialPosition = false;
	//isTesting = false;
	
	std::cout << "initialx=" << initialx << " initialy=" << initialy << std::endl;
	std::cout << "targetx=" << targetx << " targety=" << targety << std::endl;
	std::cout << "frame_id= " << "odom" << std::endl;
	std::cout << "use_initial " << "false" << std::endl;
	}
	// create graph publisher
	ros::Publisher graphPublisher = nh.advertise<visualization_msgs::MarkerArray>("/mygraph", 10);

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
	 
	tf::TransformListener transformListener;
	tf::StampedTransform robotTransform;



	// BEGIN: RRT solver
	// implement your own RRT algorithm by using class template MyRRTSolver
	// you are free to modify in any way you wanted to
	if(!path_created){
	MyRRTSolver solver;
	solver.setMapHelper(&mapHelper);
	solver.solve(initialx, initialy, targetx, targety);
	//END: RRT Solver
	// get pose commands from the RRT solver
	poses = solver.getWayPoints();
	//BEGIN: RVIZ VISUALIZATION
	sendVisualizationMessage(graphPublisher, solver);
	//END: RVIZ VISUALIZATION
 	path_created = true;
}


	geometry_msgs::PoseStamped new_goal = goal;
	// wait for 3 seconds to be make sure that connection is complete and messages will be sent successfully
	usleep(3000000);

	// the goto pose loop will run we do not test the rrt algorithm

	geometry_msgs::Pose pose ; 
	// get pose information from transform tf
	transformListener.lookupTransform("odom","base_link", ros::Time(0),robotTransform);
	float robotX=robotTransform.getOrigin().x();
	float robotY=robotTransform.getOrigin().y();
	float robotYaw=tf::getYaw(robotTransform.getRotation());

	// if goto pose completed we can send new goto pose command

	if (gotoPoseCompleted && (poses.size() > 0)) {

	// remove the first two elements


  gotoPoseCompleted = false;
	//delete vertexes 2 by two
	if(poses.size() > 3){
    	pose = *(poses.begin()+2);
			poses.erase(poses.begin());
			poses.erase(poses.begin());
	}
	else {
    	pose = *(poses.begin()+2);
			poses.erase(poses.begin());
	}

		
	new_goal.pose.position.x = pose.position.x  ; //pose.position.x ;
	new_goal.pose.position.y = pose.position.y;
	new_goal.pose.orientation.z = pose.orientation.z;
	plan.push_back(new_goal);
	}
  std::cout<<"robot pose: x,y,yaw: "<<robotX<<","<<robotY<<","<<robotYaw <<std::endl;
	std::cout << "Semi-goal pos: " << new_goal.pose.position.x << " pose.y=" << new_goal.pose.position.y << " number of semi-goals left " << poses.size()-2 << std::endl;
		
	if(0.04 < (robotX- new_goal.pose.position.x) * (robotX- new_goal.pose.position.x) + (robotY- new_goal.pose.position.y) * (robotY- new_goal.pose.position.y)) {	
		if( (robotYaw  <  new_goal.pose.orientation.z +  0.3)|| (robotYaw  > new_goal.pose.orientation.z -  0.3)  ) {
      gotoPoseCompleted = true;
}}
	

  return true;
 }
 };
