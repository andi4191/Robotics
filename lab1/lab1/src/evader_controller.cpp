#include"ros/ros.h"
#include"std_msgs/String.h"
#include "string"
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<stdlib.h>

#define MAX_COUNT 360
//std::string bot_evader="/robot_0";


geometry_msgs::Pose pos;
ros::Publisher publ;
ros::Subscriber scan_val;
ros::Subscriber pose_val;

bool obstacle_in_sight(const float *intensities,const float *view, float thresh )
{	
bool flag=false;
	int i=0;
	for(i=0;i<MAX_COUNT;i++)
	{
		if(intensities[i]==1.0f && view[i]<thresh)
		{
			flag=true;
			break;
		}
	}
	//ROS_INFO("Obstacle sight:%d",flag);
return flag;
	
	
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& mesg)
{
	//ROS_INFO("odomCallback called\n");
	pos=mesg->pose.pose;
	//ROS_INFO("[Evader] angle from origin z[%0.2f]",atan2(pos.position.y,pos.position.x));;
	ROS_INFO("[Evader] broadcast at x[%0.2f] y[%0.2f]",pos.position.x,pos.position.y);
	static tf::TransformBroadcaster br;
		tf::Transform transform;
		
	    //transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
		transform.setOrigin( tf::Vector3(pos.position.x, pos.position.y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, pos.orientation.z);
		//q.setRPY(pos.orientation.z,0,0);
      transform.setRotation(q);
	
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world","/evader"));//,"robot_1/base_link"));
			
}

//This function will detect if the laser is in sight of 
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
		int i;
        bool detect_obst=false;
	
        float range_min=msg->range_min;
        float range_max=msg->range_max;
        float thresh=range_max-2;
        double intensities[MAX_COUNT];
        double ranges[MAX_COUNT];
        

	geometry_msgs::Twist cmd;
	
	cmd.linear.x=2;
	cmd.linear.z=2;
	cmd.linear.y=0;
	cmd.linear.z=0;
	cmd.angular.x=0;
	cmd.angular.y=0;
	cmd.angular.z=0;
		
	bool flag=false;
	
	//int max_n=0,min_n=360;
	for(i=0;i<MAX_COUNT;i++)
	{
		if((msg->intensities[i]==1.0f) && (msg->ranges[i]<thresh))
		{
			
			flag=true;
			break;
		}
	}
	
	if(true==flag)
	{
		int rotate=rand()%361;	
		float val=rotate;  //36% equals 0.628319
		int clock_wise=rand()%2;
		if(clock_wise==0){
			//val=0-val;
		}
			
		cmd.angular.z=0-val;
		cmd.linear.x=0;
	}
	
	publ.publish(cmd);

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"evader_ctrl");
    static ros::NodeHandle node;
	
	//Subscribe to bot1's laser scan topic
	scan_val=node.subscribe("base_scan",10,scanCallback);
	
	//Publish to bot1's cmd_vel to move the bot
	pose_val=node.subscribe("odom",10,odomCallback);
	publ=node.advertise<geometry_msgs::Twist>("cmd_vel",10); 
	//sleep(5);
	ros::Rate loop_rate(5);
	
	while(ros::ok())
	{
		
		ros::spin();    
		loop_rate.sleep();
	}
    
return 0;
}
