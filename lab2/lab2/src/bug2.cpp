#include"ros/ros.h"
#include<cmath>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/LaserScan.h>
#include<visualization_msgs/Marker.h>
#define NUM_POINTS 2
#define MAX_COUNT 360


ros::Publisher publ;
int bot_state;
bool atGoal=false;
double dist_thresh=0.8f;
int callib_ang=65;
int k=25;
float lin_vel=0.6f;
double pi=3.1415926535897;
double angle_increment=pi/360;
geometry_msgs::Pose pos;
geometry_msgs::Point obst_pt[NUM_POINTS];
ros::Subscriber pose_val;
ros::Subscriber scan_val;
ros::Subscriber mark_val;
bool pos_received=false;
bool marker_line_avail=false;
int th_ang=120;
bool wall_in_front=false,wall_on_left=false;  //No need for right side wall(Always turn right at wall)
double target_ang;
typedef enum
{
	GOAL_SEEK=0,
	WALL_FOLLOW,
	DONE
}EBotState;
void bug2_alg();
typedef enum
{
	NONE=0,
	EWALL_IN_FRONT,
	EWALL_ON_LEFT,
	EWALL_ON_RIGHT
}EWALL_POSITION;

typedef struct{
	
	float x;
	float y;
}SPoint;

SPoint destpos,mypos,stpos;

std::vector<SPoint> points;

void set_state(int st)
{
	bot_state=st;
}

int get_state()
{
	return bot_state;
}

double get_path_slope(SPoint mypos,SPoint destpos)
{
	double dx=destpos.x-mypos.x;
	if(dx!=0)
		return ((destpos.y-mypos.y)/dx);
			
}

SPoint get_mypos()
{
	SPoint m_pos;
	m_pos.x=pos.position.x;
	m_pos.y=pos.position.y;
	return m_pos;
}

void set_mypos()
{
	mypos.x=pos.position.x;
	mypos.y=pos.position.y;
}

double get_dist(SPoint mypos,SPoint destpos)
{
	return sqrt(pow((destpos.x-mypos.x),2)+pow((destpos.y-mypos.y),2));
}

void set_vel(geometry_msgs::Twist vel_msg)
{
		publ.publish(vel_msg);
}


void odomCallback(const nav_msgs::Odometry::ConstPtr& mesg)
{
	pos=mesg->pose.pose;
	set_mypos();
	//ROS_INFO("odomCallback bot position[%0.2f,%0.2f]", pos.position.x, pos.position.y);
	pos_received=true;
	bug2_alg();
}

double get_x(int k)
{
		return 3.0*(cos(k*angle_increment));
}

double get_y(int k)
{
		return 3.0*(sin(k*angle_increment));
}

void markCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
		bool flg=false;
		int i=0;
		
		SPoint p;
		
		//while(msg->points[i].x!=0 && msg->points[i].y!=0 && msg->points[i].z!=0)
		for(i=0;i<sizeof(msg->points)/sizeof(msg->points[0]);i++)
		{
				p.x=msg->points[i].x;
				p.y=msg->points[i].y;
				//i++;
				ROS_INFO("pt mark [%0.2f,%0.2f]",p.x,p.y);
				points.push_back(p);
		}
		
		//first figure out the wall positions
		//check if the points lie in >120 degree area
		int  k=0;
		bool fflg=false,lflg=false;
		for(i=0;i<points.size();i=i+2)
		{
			k=i;
			while(k!=i+1)
			{
				int incr=0;
				if(points[i].x<get_x(120) && points[i].x>get_x(180)) incr++;
				if(incr>1)
					if(fflg) wall_in_front=true;
					else
					{
						wall_in_front=false;
						fflg=true;
					}
				else
					wall_in_front=false;
						
				incr=0;
				if(points[i].x>get_x(callib_ang)) incr++;
				if(incr>1)
				    if(lflg) wall_on_left=true;
				    else
					{
						wall_on_left=false;
						lflg=true;
					}
				else
					wall_on_left=false;
				
				k++;
			}
		}
				
				
				points.clear();
		
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
		double ranges[MAX_COUNT];
		int i=0;
		for(i=0;i<MAX_COUNT;i++)
		{
			ranges[i]=msg->ranges[i];
		}
		int incr=0;
		for(i=0;i<th_ang;i++)
		{
			if((ranges[180-(th_ang/2)+i])<1.0f) incr++;
		}
		if(incr>1)
		{
			wall_in_front=true;
			//ROS_INFO("WALL_IN_FRONT");
		}
		else
			wall_in_front=false;
		
		incr=0;
		for(i=0;i<callib_ang;i++)
		{
				if(ranges[i]<1.0f) incr++;
		}
		if(incr>10) {
			wall_on_left=true;
			//ROS_INFO("WALL_ON_LEFT");
			}
		else
			wall_on_left=false;
}
	
void subs_seq()
{
	
   ros::NodeHandle n;   
   pose_val=n.subscribe("base_pose_ground_truth",10,odomCallback);
   publ=n.advertise<geometry_msgs::Twist>("cmd_vel",10); 
//   mark_val=n.subscribe("visualization_marker",10,markCallback);
   scan_val=n.subscribe("base_scan",10,scanCallback);
   //ros::Subscriber mark = n.subscribe("marker_for_bug2", 10,markCallback);
	
}

bool check_on_slope()
{
	bool flag=true;
	double val=abs((stpos.x*(destpos.y-mypos.y)+destpos.x*(mypos.y-stpos.y)+mypos.x*(stpos.y-destpos.y))/2);
	if(val>0.6f) //cannot keep exact zero because of ang_vel
		flag=false;
	
	return flag;
}

double get_ang_vel(double target_ang)
{
		double ang;
		if(wall_in_front) return 1;
		else
			ang=target_ang<1?target_ang:1;
		return ang;
}
double get_bot_angle(float x)
{
		return 2*asin(x);
}

double get_ang_vel_for_wfollow(double target_ang)
{
	
	if(wall_in_front)  return 0.5; //reduce the speed
	else if(wall_on_left) return 0;  //stop
	else
		return (-1*0.4); //move towards right away from the wall
}
//Credits:https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf
void bug2_alg()
{	
	double bot_angle=get_bot_angle(pos.orientation.z);//2*asin(pos.orientation.z);
	double dist=sqrt(pow((destpos.x-mypos.x),2)+pow((destpos.y-mypos.y),2));
	target_ang=atan((destpos.y-mypos.y)/(destpos.x-mypos.x))-bot_angle;
	bool val=check_on_slope();
	geometry_msgs::Twist vel_msg;
	double vel_dat=0;
	ROS_INFO("dist from target[%0.2f] dist_thresh[%0.2f]",dist,dist_thresh);
	//Credits:http://spacecraft.ssl.umd.edu/academics/788XF14/788XF14L14/788XF14L14.pathbugsmapsx.pdf
	if(!atGoal)
	{
		    ROS_INFO("not at goal yet");
			if(dist<dist_thresh)
			{
				vel_msg.linear.x=0;
				vel_msg.angular.z=0;
				atGoal=true;
				set_state(DONE);
				publ.publish(vel_msg);
				return;
			}
			else
			{
				//if wall is in front of the bot then stop otherwise bot will collide
				//TODO: Need to tune the linear velocity 
				if(wall_in_front) 
				{
					ROS_INFO("DETECTED WALL in FRONT");
					vel_dat=0.0;
				}
				else
				{
					//Front path clear can move straight
					
					ROS_INFO("NO_WALL in SIGHT from front till now. Heading straight");
					vel_dat=lin_vel;
				}
	
				vel_msg.linear.x=vel_dat;
				
				if(GOAL_SEEK==get_state())
				{
					ROS_INFO("GOAL seek state");
					vel_msg.angular.z=get_ang_vel(target_ang);
					if(wall_in_front)	{set_state(WALL_FOLLOW);ROS_INFO("Wall in front wall follow");}
				
				}
				else
				{
					//Try to move away from the wall towards right for maintaining the distance
					vel_msg.angular.z=-1*get_ang_vel_for_wfollow(target_ang);
					if(val && !wall_in_front)	{set_state(GOAL_SEEK);ROS_INFO("no wall and on slope state change to GOAL_SEEK");}
				}
			}
		ROS_INFO("Publishing to move");
		publ.publish(vel_msg);

		}
	
}

int main( int argc, char** argv)
{
   ros::init(argc, argv, "bug2_ctrl");
   
   subs_seq();

   mypos.x=-8.0;
   mypos.y=-2.0;
   
   stpos.x=-8.0;
   stpos.y=-2.0;
   
   destpos.x=4.5;
   destpos.y=9.0;
   ros::Rate r(10);
   set_state(GOAL_SEEK);
   while (ros::ok())
   {
	   
	    ros::spinOnce();
	   	r.sleep();
	
   }
   return 0;
}
