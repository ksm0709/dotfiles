#include "ros/ros.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include "rtabmap_ros/SetGoal.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"

rtabmap_ros::SetGoal goal_srv;
std_msgs::String goal_msg;
std_msgs::String goal_reached_msg;
std_msgs::String robot_state;
geometry_msgs::PoseWithCovarianceStamped pose_cur;
geometry_msgs::PoseStamped rviz_goal_msg;
bool goal_msg_updated, rviz_goal_msg_updated;
bool goal_reached;

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void genieGoalCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Goal request recieved : %s", msg->data);
	goal_msg.data = msg->data;
	goal_msg_updated = true;
}
//
//void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
//{
//	pose_cur.header = msg->header;
//	pose_cur.pose = msg->pose;
//}

void reachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	goal_reached = msg->data;	
}

void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	rviz_goal_msg.header = msg->header;
	rviz_goal_msg.pose = msg->pose;
	rviz_goal_msg_updated = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_control");
	ros::NodeHandle nh;

	ros::Rate loop_rate(10);

	ros::ServiceClient client = nh.serviceClient<rtabmap_ros::SetGoal>("set_goal");
	ros::Subscriber genie_goal = nh.subscribe("/robot_cmd",3,genieGoalCallback);
	ros::Subscriber goal_reached_sub = nh.subscribe("goal_reached",10,reachedCallback);
	ros::Subscriber rviz_goal_sub = nh.subscribe("/rviz_goal",10,rvizGoalCallback);

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10);
	ros::Publisher goal_reached_pub = nh.advertise<std_msgs::String>("/arrived_status",3);
	ros::Publisher state_pub = nh.advertise<std_msgs::String>("/robot_state",3);
	ros::Publisher rviz_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal",3);

	MoveBaseClient ac("/move_base", true);

	tf::TransformListener listener;
	tf::StampedTransform tf_base;

	goal_srv.request.node_id = 0;
	goal_reached_msg.data = "1";
	robot_state.data = "idle";
	state_pub.publish(robot_state);


	ROS_INFO("Rtabmap goal control node start!");
	
	while( ros::ok() )
	{
		// when new goal is updated
		if( goal_msg_updated )
		{
			// get goal label from topic data
			goal_srv.request.node_label = goal_msg.data;
			goal_msg_updated = false;

			if( client.call(goal_srv) )
			{
				ROS_INFO("Goal request status : Sucessed");
				robot_state.data = "navigation";
			}
			else
				ROS_INFO("Goal request status : Failed");
			
		}
		else if( rviz_goal_msg_updated )
		{
			ROS_INFO("Simple goal request status : Sucessed");
			rviz_goal_pub.publish(rviz_goal_msg);
			rviz_goal_msg_updated = false;

			robot_state.data = "navigation";
		}

		if( goal_reached )
		{
			goal_reached_pub.publish( goal_reached_msg );
			goal_reached = false;

			robot_state.data = "idle";
		}

//		// When move_base action server returned result
//		if( ac.waitForServer(ros::Duration(0.01)) )
//		{
//			// if the action server is in terminal state, robot_state = idle
//			if( ac.getState().isDone() )
//			{
//				robot_state.data = "idle";
//			}
//			else // otherwise, robot_state = navigation
//			{
//				robot_state.data = "navigation";
//			}
//		}
	
		// Get current robot position
		try
		{
			listener.lookupTransform("map", "base_footprint", ros::Time(0), tf_base);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		pose_cur.header.stamp = ros::Time::now();
		pose_cur.header.frame_id = "map";
		pose_cur.pose.pose.position.x = tf_base.getOrigin().x();
		pose_cur.pose.pose.position.y = tf_base.getOrigin().y();
		pose_cur.pose.pose.position.z = 0;

		pose_pub.publish(pose_cur);
		state_pub.publish(robot_state);
		
		loop_rate.sleep();	
		ros::spinOnce();
	}

	return 0;
}
