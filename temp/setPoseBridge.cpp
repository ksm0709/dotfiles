/*	
 * 1. Get "/initialpose"(geometry_msgs::PoseWithCovarianceStamped) topic
 * 2. Request "SetPose"(robot_localization::SetPose) service. 
 */

#include "ros/ros.h"
#include "robot_localization/SetPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

bool pose_recieved;
robot_localization::SetPose setPose_srv;
geometry_msgs::PoseWithCovarianceStamped initPose_msg;

void initPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
	initPose_msg.header = msg->header;
	initPose_msg.pose = msg->pose;	
	pose_recieved = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setPoseBridgeNode");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<robot_localization::SetPose>("set_pose");
	ros::Subscriber initPose_sub = nh.subscribe("/initialpose", 1, initPose_callback);

	ros::Rate loop_rate(10);
	pose_recieved = false;

	while( ros::ok() )
	{
		if( pose_recieved )
		{
			setPose_srv.request.pose = initPose_msg;

			if( client.call(setPose_srv) )
			{
				ROS_INFO("Initial pose reset");
			}
			else
			{
				ROS_INFO("Initial pose reset failed");
			}

			pose_recieved = false;
		}

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
