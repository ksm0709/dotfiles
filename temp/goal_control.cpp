#include "ros/ros.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include "rtabmap_ros/SetGoal.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "tf/transform_listener.h"

#define IDLE  0
#define NAVIGATION 1

enum { px,py,pz,qx,qy,qz,qw };
enum { PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST };

std::vector<double> pose_info;
rtabmap_ros::SetGoal goal_srv;
std_msgs::String goal_msg;
std_msgs::String goal_reached_msg;
std_msgs::String robot_state;
std_srvs::Empty srv_empty;
geometry_msgs::PoseWithCovarianceStamped pose_cur;
geometry_msgs::PoseStamped rviz_goal_msg;
bool goal_msg_updated, rviz_goal_msg_updated;
bool goal_nav;
bool cmd_flag;
double t_tmp;
char state;

std::map<std::string, geometry_msgs::Pose> m; 

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void genieGoalCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Goal request recieved : %s", msg->data);
	goal_msg.data = msg->data;
	goal_msg_updated = true;
}

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	robot_state.data = "navigation";
	state = NAVIGATION;
	cmd_flag = true;
	t_tmp = ros::Time::now().toSec();
}

void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	rviz_goal_msg.header = msg->header;
	rviz_goal_msg.pose = msg->pose;
	rviz_goal_msg_updated = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_control_tmp");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	ros::Rate loop_rate(10);

	ros::ServiceClient client = nh.serviceClient<rtabmap_ros::SetGoal>("set_goal");
	ros::ServiceClient clear = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	ros::Subscriber genie_goal = nh.subscribe("/robot_cmd",3,genieGoalCallback);
	ros::Subscriber rviz_goal_sub = nh.subscribe("/rviz_goal",10,rvizGoalCallback);
	ros::Subscriber vel_sub = nh.subscribe("/cmd_vel",10,cmdvelCallback);

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10);
	ros::Publisher goal_reached_pub = nh.advertise<std_msgs::String>("/arrived_status",1);
	ros::Publisher state_pub = nh.advertise<std_msgs::String>("/robot_state",3);
	ros::Publisher rviz_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",3);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	move_base_msgs::MoveBaseGoal goal;
	MoveBaseClient ac("/move_base", true);
	while( !ac.waitForServer(ros::Duration(1.0)) ){
			ROS_INFO("Wait for move base");
			}

	tf::TransformListener listener;
	tf::StampedTransform tf_base;
	std_srvs::Empty	dummy_srv;
	geometry_msgs::Twist vel_msg;

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
			if( goal_msg.data == std::string("VIP2") )
			{
				vel_msg.linear.x = 0.0;
				vel_msg.angular.z = 6.28/6.0;
				vel_pub.publish(vel_msg);

				ros::Duration(3.0).sleep();
				
				vel_msg.angular.z = 0.0;
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);
			}
			else if( goal_msg.data == std::string("VIP4") )
			{
				vel_msg.linear.x = 0.3;
				vel_msg.angular.z = 0.0;
				vel_pub.publish(vel_msg);

				ros::Duration(4.0).sleep();
				
				vel_msg.linear.x = 0.0;
				vel_msg.angular.z = 0.0;
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);

				continue;
			}
			else if( goal_msg.data == std::string("VIP5") )
			{
				vel_msg.linear.x = 0.0;
				vel_msg.angular.z = 6.28/6.0;
				vel_pub.publish(vel_msg);

				ros::Duration(1.5).sleep();
				
				vel_msg.angular.z = 0.0;
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);

				ros::Duration(0.5).sleep();

				vel_msg.linear.x = 0.60;
				vel_msg.angular.z = (90.0/4.0)*(3.14/180);
				vel_pub.publish(vel_msg);

				ros::Duration(4.0).sleep();
				
				vel_msg.linear.x = 0.0;
				vel_msg.angular.z = 0.0;
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);
				vel_pub.publish(vel_msg);
			}


			clear.call( dummy_srv );
			ros::Duration(1.0).sleep();

			if( nh_private.getParam(goal_msg.data, pose_info) )
			{
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = pose_info[px];	
				goal.target_pose.pose.position.y = pose_info[py];	
				goal.target_pose.pose.position.z = pose_info[pz];	
				goal.target_pose.pose.orientation.x = pose_info[qx];	
				goal.target_pose.pose.orientation.y = pose_info[qy];	
				goal.target_pose.pose.orientation.z = pose_info[qz];	
				goal.target_pose.pose.orientation.w = pose_info[qw];

				ac.sendGoal(goal);

				/*
				rviz_goal_msg.header.stamp=ros::Time::now(); 	
				rviz_goal_msg.header.frame_id = "map";
				rviz_goal_msg.pose.position.x = pose_info[px];	
				rviz_goal_msg.pose.position.y = pose_info[py];	
				rviz_goal_msg.pose.position.z = pose_info[pz];	
				rviz_goal_msg.pose.orientation.x = pose_info[qx];	
				rviz_goal_msg.pose.orientation.y = pose_info[qy];	
				rviz_goal_msg.pose.orientation.z = pose_info[qz];	
				rviz_goal_msg.pose.orientation.w = pose_info[qw];

				rviz_goal_pub.publish(rviz_goal_msg);
				*/
			}
			else
				ROS_ERROR("Getting goal parameter failed!");

			goal_msg_updated = false;
			goal_nav = true;

/*
			// get goal label from topic data
			goal_srv.request.node_label = goal_msg.data;
			goal_msg_updated = false;

			clear.call( dummy_srv );
			ros::Duration(1.0).sleep();

			if( client.call(goal_srv) )
			{
				// goal : last planned pose
				int path_size = goal_srv.response.path_poses.size() - 1;
				rviz_goal_msg.header.stamp=ros::Time::now(); 	
				rviz_goal_msg.header.frame_id = "map";

				if( path_size <= 0 )
				{
					ROS_ERROR("PATH SIZE <= 0");
					continue;
				}
				
				rviz_goal_msg.pose = goal_srv.response.path_poses[path_size];	
				rviz_goal_pub.publish(rviz_goal_msg);
				
				ROS_INFO("Goal request status : Sucessed");
			}
			else
				ROS_INFO("Goal request status : Failed");
*/
			
		}
		else if( rviz_goal_msg_updated )
		{
			ROS_INFO("Simple goal request status : Sucessed");
			rviz_goal_pub.publish(rviz_goal_msg);
			rviz_goal_msg_updated = false;
		}
		
		if( goal_nav )
		{
			if( ac.waitForResult(ros::Duration(0.1)) )
			{
				actionlib::SimpleClientGoalState as = ac.getState();
				if( as.isDone() )  
				{
					switch( as.state_ )
					{
						case PREEMPTED:
						case ABORTED:
							clear.call( dummy_srv );
							ros::Duration(1.0).sleep();

							ac.sendGoal(goal);

							break;

						case SUCCEEDED:
							//goal_reached_pub.publish( goal_reached_msg );
							goal_nav = false;

							break;

						default:

							break;
					}

				}
			}
		}

		if( cmd_flag && ros::Time::now().toSec() - t_tmp > 2.0 )
		{
			robot_state.data = "idle";
			state = IDLE;
			cmd_flag = false;
		}

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
		pose_cur.pose.pose.orientation.x = tf_base.getRotation().x();
		pose_cur.pose.pose.orientation.y = tf_base.getRotation().y();
		pose_cur.pose.pose.orientation.z = tf_base.getRotation().z();
		pose_cur.pose.pose.orientation.w = tf_base.getRotation().w();

		pose_pub.publish(pose_cur);
		state_pub.publish(robot_state);
		
		loop_rate.sleep();	
		ros::spinOnce();
	}

	return 0;
}
