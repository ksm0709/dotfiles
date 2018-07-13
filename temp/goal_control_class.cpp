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

using namespace std;

class goal_control
{

public:
	goal_control(ros::NodeHandle* nh_, ros::NodeHandle* nh_private_)
	:nh(*nh_), nh_private(*nh_private_)
	{
		init();
	}

	void init();
	void run();

private:
	void genieGoalCallback(const std_msgs::String::ConstPtr& msg);
	void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


private:
	enum { px,py,pz,qx,qy,qz,qw };
	enum { PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST };
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	
	bool goal_msg_updated, rviz_goal_msg_updated;
	bool goal_nav;
	bool cmd_flag;
	double t_tmp;
	char state;
	
	std::string cur_pose;
	std::vector<double> pose_info;
	
	// ROS Message & Service
	rtabmap_ros::SetGoal goal_srv;
	std_msgs::String goal_msg;
	std_msgs::String goal_reached_msg;
	std_msgs::String robot_state;
	std_srvs::Empty	dummy_srv;
	geometry_msgs::PoseWithCovarianceStamped pose_cur;
	geometry_msgs::PoseStamped rviz_goal_msg;
	geometry_msgs::Twist vel_msg;

	// ROS Service client
	ros::ServiceClient client;
	ros::ServiceClient clear;

	// ROS Topic subscriber
	ros::Subscriber genie_goal;
	ros::Subscriber vel_sub;

	// ROS Topic publisher
	ros::Publisher pose_pub;
	ros::Publisher goal_reached_pub;
	ros::Publisher state_pub;
	ros::Publisher rviz_goal_pub;
	ros::Publisher vel_pub;

	// ROS Node handler
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;

	// ROS move_base related vars
	move_base_msgs::MoveBaseGoal goal;
	MoveBaseClient* ac;

	// TF vars
	tf::TransformListener listener;
	tf::StampedTransform tf_base;
};


void goal_control::init()
{
	client = nh.serviceClient<rtabmap_ros::SetGoal>("set_goal");
	clear = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	genie_goal = nh.subscribe("/robot_cmd",3,&goal_control::genieGoalCallback,this);
	rviz_goal_sub = nh.subscribe("/rviz_goal",10,&goal_control::rvizGoalCallback,this);
	vel_sub = nh.subscribe("/cmd_vel",10,&goal_control::cmdvelCallback,this);

	pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10);
	goal_reached_pub = nh.advertise<std_msgs::String>("/arrived_status",1);
	state_pub = nh.advertise<std_msgs::String>("/robot_state",3);
	rviz_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",3);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	ac = new MoveBaseClient("/move_base", true);
	while( !ac->waitForServer(ros::Duration(1.0)) ){
			ROS_INFO("Wait for move base");
			}
	
	goal_srv.request.node_id = 0;
	goal_reached_msg.data = "1";
	robot_state.data = "idle";
	state_pub.publish(robot_state);
}

void goal_control::run()
{
	ROS_INFO("Rtabmap goal control node start!");
	ros::Rate loop_rate(10);

	while( ros::ok() )
	{
		// when new goal is updated
		if( goal_msg_updated )
		{
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

				ac->sendGoal(goal);
			}
			else
				ROS_ERROR("Getting goal parameter failed!");

			goal_msg_updated = false;
			goal_nav = true;
		}
		else if( rviz_goal_msg_updated )
		{
			ROS_INFO("Simple goal request status : Sucessed");
			rviz_goal_pub.publish(rviz_goal_msg);
			rviz_goal_msg_updated = false;
		}
		
		if( goal_nav )
		{
			if( ac->waitForResult(ros::Duration(0.1)) )
			{
				actionlib::SimpleClientGoalState as = ac->getState();
				if( as.isDone() )  
				{
					switch( as.state_ )
					{
						case PREEMPTED:
						case ABORTED:
							clear.call( dummy_srv );
							ros::Duration(1.0).sleep();

							ac->sendGoal(goal);

							break;

						case SUCCEEDED:
							//goal_reached_pub.publish( goal_reached_msg );
							goal_nav = false;
							cur_pose = goal_msg.data;

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
}

void goal_control::genieGoalCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Goal request recieved : %s", msg->data);
	goal_msg.data = msg->data;
	goal_msg_updated = true;
}

void goal_control::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	robot_state.data = "navigation";
	state = NAVIGATION;
	cmd_flag = true;
	t_tmp = ros::Time::now().toSec();
}

void goal_control::rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

	goal_control gc(&nh, &nh_private);
	gc.run();
	
	return 0;
}
