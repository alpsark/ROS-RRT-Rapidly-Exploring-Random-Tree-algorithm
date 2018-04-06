#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#define PI 3.1415926535897932384626433f
#define MAX_TURN_SPD 0.52359879
#define MIN_TURN_SPD 0.05

#define MAX_LINEAR_SPD 0.7
#define EPS 0.05

class GotoPose
{

    //ros::Subscriber scanSubs;
	ros::Subscriber gotoCommandSubs;
	ros::Publisher completedPub;

    ros::Publisher velPub;
    tf::TransformListener transformListener;
    tf::StampedTransform robotTransform;
    ros::NodeHandle nh;
    ros::Rate rate;

    geometry_msgs::Pose targetPose;
    bool gotoPoseCompleted;

    bool isTargetPoseInit;

public:
    GotoPose():rate(10)
    {
    	isTargetPoseInit = false;
    	gotoPoseCompleted = true;

    	gotoCommandSubs = nh.subscribe("/gotoPose", 10, &GotoPose::gotoCommandCallback, this);
    	completedPub = nh.advertise<std_msgs::Bool>("/gotoPoseCompleted", 1);

        velPub=nh.advertise<geometry_msgs::Twist> ("/cmd_vel",1);

        transformListener.waitForTransform("odom","base_link",
                ros::Time::now(),ros::Duration(3.0));
    }

    void run()
    {

    	while(nh.ok())
    	{
    		// get pose information from transform tf
    		transformListener.lookupTransform("odom","base_link",
    				ros::Time(0),robotTransform);
    		float robotX=robotTransform.getOrigin().x();
    		float robotY=robotTransform.getOrigin().y();
    		float robotYaw=tf::getYaw(robotTransform.getRotation());
    		//std::cout<<"robot pose: x,y,yaw: "<<robotX<<","<<robotY<<","<<robotYaw <<std::endl;

    		// initialize target pose
    		if (!isTargetPoseInit) {
    			targetPose.position.x = robotX;
    			targetPose.position.y = robotY;
    			isTargetPoseInit = true;
    		}

    		// check completed
    		float diffx = targetPose.position.x - robotX;
    		float diffy = targetPose.position.y - robotY;
    		if (fabs(diffx) < EPS && fabs(diffy) < EPS) {
    			if (!gotoPoseCompleted) {
					std_msgs::Bool completedMsg;
					completedMsg.data = true;
					completedPub.publish(completedMsg);
    			}
    			gotoPoseCompleted = true;
    		}

    		// create velocity vector
    		geometry_msgs::Twist moveCmd;
    		moveCmd.linear.x=0;
    		moveCmd.linear.y=0;
    		moveCmd.angular.z=0;

    		// if goto pose is not completed try to go to the position
    		if (!gotoPoseCompleted) {
    			float targetYaw = atan2(diffy, diffx);
    			float yawDiff = normalizeRad(targetYaw - robotYaw);
    			int sign = yawDiff < 0?-1:1;
    			moveCmd.angular.z = yawDiff;

    			if(fabs(moveCmd.angular.z) > MAX_TURN_SPD)
    			{
    				moveCmd.angular.z=MAX_TURN_SPD * sign;
    			}
    			if(fabs(moveCmd.angular.z) < MIN_TURN_SPD)
    			{
    				moveCmd.angular.z = MIN_TURN_SPD * sign;
    			}

    			if (fabs(yawDiff) < 0.2) {
    				float linearK = 0.7;
    				moveCmd.linear.x = linearK * sqrt(pow(diffy,2) + pow(diffx,2));
    				if (moveCmd.linear.x > MAX_LINEAR_SPD)
    				{
    					moveCmd.linear.x = MAX_LINEAR_SPD;
    				}
    			}
    		}

    		velPub.publish(moveCmd);

    		ros::spinOnce();
    		rate.sleep();
    	}
    }

    void gotoCommandCallback(const geometry_msgs::Pose& _targetPose)
    {
    	std::cout<< "gotoCommandCallback target.x=" << _targetPose.position.x << " target.y=" << _targetPose.position.y << std::endl;

    	targetPose.position.x = _targetPose.position.x;
    	targetPose.position.y = _targetPose.position.y;
    	targetPose.position.z = _targetPose.position.z;

    	targetPose.orientation.w = _targetPose.orientation.w;
    	targetPose.orientation.x = _targetPose.orientation.x;
    	targetPose.orientation.y = _targetPose.orientation.y;
    	targetPose.orientation.z = _targetPose.orientation.z;

    	gotoPoseCompleted = false;

    }

    float normalizeRad(float rad)
    {
        while(rad>PI)
        {
            rad=rad-2*PI;
        }
        while(rad<-PI)
        {
            rad=rad+2*PI;
        }
        return rad;
    }
};

int main(int argc,char **argv)
{
    std::cout<<"GotoPose Start..."<<std::endl;
    ros::init(argc,argv,"goto_pose");
    GotoPose gp;
    gp.run();
    std::cout << "GotoPose Finish..." << std::endl;
}
