#include"ros/ros.h"
#include"std_msgs/String.h"
#include"string"
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<stdlib.h>

typedef enum
{
	FIRST_QUAD = 0,
	SECOND_QUAD,
	THIRD_QUAD,
	FOURTH_QUAD,
	X_AXIS,
	Y_AXIS
}EQuad_t;

typedef enum
{
	UP=0,
	DOWN,
	LEFT,
	RIGHT
}EDir_t;

geometry_msgs::Pose pos;
ros::Publisher publ;
ros::Subscriber pose_val;

void odomCallback(const nav_msgs::Odometry::ConstPtr& mesg)
{
	pos=mesg->pose.pose;
	ROS_INFO("[Pursuer] broadcast at x[%0.2f] y[%0.2f] z[%0.2f]",pos.position.x,pos.position.y,pos.orientation.z);
	static tf::TransformBroadcaster br;
		tf::Transform transform;
		
	  //  transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
        transform.setOrigin( tf::Vector3(pos.position.x, pos.position.y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, pos.orientation.z);
		//q.setRPY(pos.orientation.z,0,0);
        transform.setRotation(q);
	  			
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world","/pursuer"));
		
		
}

int get_quad_info(float x, float y, float my_pos_x, float my_pos_y,int *dir_x, int *dir_y)
{
	int quad=0;
		if(x==my_pos_x) 
		{
			quad=Y_AXIS;
			if(y>my_pos_y)
				*dir_y=UP;
			else
				*dir_y=DOWN;
		}
		else if(y==my_pos_y) 
		{
			quad=X_AXIS;
			if(x>my_pos_x)
				*dir_x=RIGHT;
			else
				*dir_y=LEFT;
		}
		else if(x>my_pos_x)
		{
			if(y>my_pos_y) quad=FIRST_QUAD;
			else
				quad=FOURTH_QUAD;
		}
		else
		{
			//x negative
			if(y>my_pos_y) quad=SECOND_QUAD;
			else
				quad=THIRD_QUAD;
		}
		
	return quad;
}

float get_angle_for_rot(float ref_x, float ref_y)
{
		int dir_x,dir_y;
		int reach_quad=get_quad_info(ref_x,ref_y,0,0,&dir_x,&dir_y);
		
		float ang=0.0;
		double pi_val=3.14159;
		switch(reach_quad)
		{
				case FIRST_QUAD:
									ang=atan2(abs(ref_y),abs(ref_x));
									
									break;
				case SECOND_QUAD:
									ang=atan2(abs(ref_y),abs(ref_x));
									ang=pi_val/2-ang;
									
									break;
				case THIRD_QUAD:
									ang=atan2(abs(ref_y),abs(ref_x));
									ang=ang-pi_val;
									
									break;
				case FOURTH_QUAD:
									ang=atan2(abs(ref_y),abs(ref_x));
									ang=0-ang;				
									
									break;
				case X_AXIS:
									if(dir_y==UP)
										ang=pi_val/2;
									else
										ang=0-pi_val/2;
				
				case Y_AXIS:
									if(dir_x==RIGHT)
										ang=0;
									else
										ang=pi_val;
		}
		
		return ang;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"pursuer_ctrl");
    static ros::NodeHandle node;
	
	pose_val=node.subscribe("odom",10,odomCallback);
	publ=node.advertise<geometry_msgs::Twist>("cmd_vel",10); 
	ros::Rate loop_rate(10);
    
	tf::TransformListener listener;
    static double prev_ang;
	while (ros::ok())
	{
		ros::spinOnce();    
		tf::StampedTransform transform;
		
		try
		{
			ros::Time now=ros::Time::now();
		    //ros::Time past=ros::Time::now()-ros::Duration(1.0);
		   /*
		   	listener.waitForTransform("/pursuer", now,"/evader",past,"/world", ros::Duration(1.0));                 
			listener.lookupTransform("/pursuer", now,"/evader",past,"/world", transform);
			 */
			ros::Time past = now - ros::Duration(2);
		/*	listener.waitForTransform("/pursuer", "/evader",past, ros::Duration(5.0));                 
			listener.lookupTransform("/pursuer", "/evader",past, transform);   
			*/
			 
			listener.waitForTransform("/pursuer",ros::Time(0),"/evader",
                              past,"/world", ros::Duration(0.1));
			listener.lookupTransform("/pursuer", ros::Time(0),"/evader",past,"/world",
							   transform);
			
		}
		catch (tf::TransformException &ex) 
		{
			//ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
				
		geometry_msgs::Twist vel_msg;
		float my_orien=pos.orientation.z;
		float my_pos_x=pos.position.x;
		float my_pos_y=pos.position.y;
		double ref_x=transform.getOrigin().x();
		double ref_y=transform.getOrigin().y();
		
		double pi_val=3.14159;
		//double ang=get_angle_for_rot(ref_x,ref_y);
		double ang=atan2(ref_y,ref_x);
				
		float off_set=1.0;
		float speed=0.5;
		/*
		double ang=atan2(ref_y,ref_x);
		double fact_lin=0;
		if(abs(ang)>0.087f)
		{
				fact_lin=0.5;
				vel_msg.linear.x=fact_lin*sqrt(pow(ref_x,2)+pow(ref_y,2));
				vel_msg.angular.z=vel_msg.linear.x*ang/0.6-my_orien;
				publ.publish(vel_msg);
		}
		else
		{
				fact_lin=0.5;
				vel_msg.linear.x=fact_lin*sqrt(pow(ref_x,2)+pow(ref_y,2));
				vel_msg.angular.z=0;
				publ.publish(vel_msg);
		}
		*/
		if(abs(ang-my_orien)>0.087f)
		{
				off_set=0.1;
				speed=0.3;//slow down
		}
		
		
		vel_msg.linear.x=speed*sqrt(pow(ref_x,2)+pow(ref_y,2));
		//double lvel=vel_msg.linear.x;
		vel_msg.angular.z=off_set*ang-my_orien;
		prev_ang=vel_msg.angular.z;
		
		//ROS_INFO("[Pursuer] evader position from me x[%0.2f] y[%0.2f] z[%0.2f] z_in_deg[%0.2f]",ref_x,ref_y,ang,ang*180/pi_val);
						
		publ.publish(vel_msg);
		loop_rate.sleep();
		
	}
    
return 0;
}
