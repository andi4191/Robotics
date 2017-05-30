#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<stdlib.h>
#include<vector>
#define MAX_COUNT 360
#define MIN_POINT 15
float pi=3.1415926535897;
float ang_offset=pi/2;
typedef struct
{
	float x;
	float y;
	
}SPoint;

int thresh_num=40;
uint32_t so_far_max=0;
//SPoint ref_point1, ref_point2;
float ranges[MAX_COUNT];
float range_min,range_max,angle_increment;
float intensities[MAX_COUNT];
//float dist_thresh=1.0f;
int k=20;
//int max_inliers_ct=0;	

float thresh_dist=0.5f;
ros::Publisher publ;
ros::Publisher marker_pub;


//This function is used to conver the range index selected at random to a coordinate point
SPoint get_point(double r,double theta)
{
	SPoint temp;
	temp.x=r*cos(theta);
	temp.y=r*sin(theta);
	return temp;
	
}

SPoint get_point_from_index(int k)
{
	SPoint temp;
	//to overcome the 90 degree difference cos(180-theta)=sin theta
	temp.x=ranges[k]*sin(k*angle_increment);
	temp.y=-(ranges[k]*cos(k*angle_increment));
	return temp;
	
}

double get_dist(SPoint p1,SPoint p2)
{
	return (sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2)));
}


float calc_dist_from_line(SPoint pt,float slope, float const_k)
{
	float ret_val=abs(slope*pt.x-pt.y+const_k)/(sqrt(slope*slope+1));
}

int find_inliers_number(SPoint p1, SPoint p2,uint32_t *so_far_max,SPoint *pt1, SPoint *pt2)
{
		int max_inliers_ct=0;
		float dist_from_line=0.0f;
		float slope=0.0f;
		bool flag=false;
		int inliers_count=0;
		if(p2.x!=p1.x)	slope=(p2.y-p1.y)/(p2.x-p1.x);
		
		float const_k=p1.y-slope*p1.x;
		int i=0;
		ROS_INFO("find_inliers_number");
		for(i=0;i<MAX_COUNT;i++)
		{
				
				if(intensities[i]==0.0f)
				{
						continue;
				}
				
				
				float theta=i*(pi/360)-ang_offset;
			    SPoint pt=get_point(ranges[i],theta);
				//dist_from_line=calc_dist_from_line(pt,slope,const_k);
				dist_from_line=fabs(slope*pt.x-pt.y+const_k)/sqrt(slope*slope+1);
				//ROS_INFO("RPoints [%0.2f,%0.2f]-[%0.2f,%0.2f] seed point[%0.2f,%0.2f] slope of line[%0.2f] dist[%f] thresh_dist[%0.2f]",p1.x,p1.y,p2.x,p2.y,pt.x,pt.y,slope,dist_from_line,thresh_dist);
				
				if(dist_from_line<thresh_dist)
				{
						
						inliers_count++;
						//ROS_INFO("pt[%0.2f,%0.2f] dist_from_line[%0.2f] thresh_dist[%0.2f] inliers_count[%d] max_inliers_ct[%d]",pt.x,pt.y,dist_from_line,thresh_dist,inliers_count,max_inliers_ct);
						if(inliers_count>max_inliers_ct)
						{
								
									max_inliers_ct=inliers_count;
									*so_far_max=max_inliers_ct;//inliers_count;		
									if(max_inliers_ct>=*so_far_max)
									{					
										    flag=true;						
											pt1->x=p1.x;
											pt2->x=p2.x;
											pt1->y=p1.y;
											pt2->y=p2.y;
											ROS_INFO("Assigned [%f,%f] and [%f,%f]",p1.x,p1.y,p2.x,p2.y);
									}
									
						}
				}
				
		}
		if(flag) ROS_INFO("Finalized max_inliers for [%f,%f] [%f,%f]",pt1->x,pt1->y,pt2->x,pt2->y);
		return max_inliers_ct;
}


void get_two_random_indexes(int *idx1, int *idx2,bool *foun1,bool *foun2)
{
			int index1=0,index2=0,count=0;
			SPoint p1,p2;
			bool found1=false;
			bool found2=false;
			
			do
			{
				index1=rand()%361;
				if(intensities[index1]!=0.0f)
				{	
					found1=true;
					break;
				}
				count++;
			}while(count<100);
	
			count=0;
			do
			{
				index2=rand()%361;
				if(intensities[index2]!=0.0f)
				{	
					found2=true;
					break;
				}
				count++;
			}while(count<100);	
			*idx1=index1;
			*idx2=index2;
			*foun1=found1;
			*foun2=found2;
}

//Credits:https://en.wikipedia.org/wiki/Random_sample_consensus	
bool ransac_algo(std::vector<SPoint>& ref_point)
{
	
	int i=0;
	bool found1=false,found2=false,ret_val=false,obst_det=false;
	SPoint p1,p2;//,point1,point2;
	std::vector<SPoint> inliers;
	std::vector<SPoint> points;
	float range_min=range_min;
    float range_max=range_max;
        
    
    float ang_range=range_max-range_min;
    
    for(i=0;i<MAX_COUNT;i++)
	{
		if(ranges[i]<3.0f)
		{
			obst_det=true;
			break;
		}
	}
	//Collect all the points with range <3 if the obstacle is detected for plotting the line
	if(obst_det)
	{
		
		for(i=0;i<MAX_COUNT;i++)
		{
	
			if(ranges[i]<3.0f)
			{	
	
					SPoint p;
					//Special case for 90 degree
					if(i==180)
					{
							p.x=ranges[i];
							p.y=0;
					}
					else
					{
							p=get_point_from_index(i);
							
					}
			
					points.push_back(p);
	
			}
		}
	
		int j;
		std::vector<SPoint> rest;
		//Will detect points only if threshold number of points detected obstacle. (For preventing flickering of the line)
		while(points.size()>=MIN_POINT)
		{
	
			std::vector<SPoint> temp;
			temp.clear();
				for(i=0;i<k;i++)
				{
					
						std::vector<SPoint> inliers;	//for storing the inliers 
						std::vector<SPoint> outliers;	//for storing the outliers
						int point1=rand()%(points.size()-1);
						int point2=rand()%(points.size()-1);
	
						float slope=0.0f;
						SPoint p1=points[point1];
						SPoint p2=points[point2];
						
						if((p1.x-p2.x) !=0.0f)	 // Check for parallel to y-axis line (Divide by zero condition)
						{
								slope=(p2.y-p1.y)/(p2.x-p1.x);
	
								for(j=0;j<points.size();j++)
								{
									SPoint pt1=points[j];
									//distance of a point from a line |ax1+by1+c|/(sqrt(a*a+b*b+c*c))
									//y-y1=m(x-x1)
									//mx1-y+(y1-mx1)=0   equation of line //TODO: absolute value of float values
									float dist=fabs(pt1.y-slope*pt1.x+slope*p1.x-p1.y)/sqrt(slope*slope+1);
									if(dist<=0.1f) 
									{
											inliers.push_back(pt1);
									}
									else
									{
											outliers.push_back(pt1);
									}
								}
						}
						else  //for parallel to axes points
						{
	
								for(j=0;j<points.size();j++)
								{
									    SPoint pt1=points[j];
										float dist=fabs(pt1.x-p1.x); 
										if(dist<=0.1f)
											inliers.push_back(pt1);
										else
											outliers.push_back(pt1);
								}
							
						}
						//ROS_INFO("temp_size[%d] inliers_size[%d] outliers_size[%d]",temp.size(),inliers.size(),outliers.size());
						if(temp.size()<inliers.size())
						{
								temp.clear();
								for(j=0;j<inliers.size();j++)
								{
									//store inliers into a arbitrary vector
									temp.push_back(inliers[j]);
								}
								
								rest.clear();//empty the vector
								for(j=0;j<outliers.size();j++)
								{
									//ROS_INFO("rest vector content[%d]:%f",j,rest[j]);
									rest.push_back(outliers[j]);
								}
	
						}
						
				}
				float max=-99.9f;
				float min=99.9f;
				int idx1=0,idx2=0;
				for(j=0;j<temp.size();j++)
				{
						//ROS_INFO("temp[%d]:%f,%f",j,temp[j].x,temp[j].y);
						if(temp[j].x>max)
						{
							idx1=j;
							max=temp[j].x;
						}
						if(temp[j].x<min)
						{
							idx2=j;
							min=temp[j].x;
						}	
				}
				if(temp.size()!=0)
				{
						SPoint pts;
						ROS_INFO("idx[%d,%d]",idx1,idx2);
						pts.x=temp[idx1].x;
						pts.y=temp[idx1].y;
						ref_point.push_back(pts);
						pts.x=temp[idx2].x;
						pts.y=temp[idx2].y;
						ref_point.push_back(pts);
						
						
				}
				points.clear();
				//ROS_INFO("Rest size left[%d]",rest.size());
				for(j=0;j<rest.size();j++)
				{
						points.push_back(rest[j]);
				}
				//rest.clear();
		}
		
	}
	//ROS_INFO("RANSAC_FINISHED");
	
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
		int i;
        bool detect_obst=false;
		bool flag=false;
     std::vector<SPoint> ref_point;
     for(i=0;i<MAX_COUNT;i++) 
     {
		ranges[i]=msg->ranges[i];
		intensities[i]=msg->intensities[i];
	 }
	 range_min=msg->range_min;
	 range_max=msg->range_max;
	 angle_increment=msg->angle_increment;
	 //SPoint ref_point1,ref_point2;
	 ROS_INFO("scancallbacks function");
	 //flag=ransac_algo(&ref_point1, &ref_point2);
	 flag=ransac_algo(ref_point);
	 //ROS_INFO("ransac_algo[%d]",flag);
	
	 visualization_msgs::Marker line_strip;
     line_strip.header.frame_id = "base_link";
     line_strip.header.stamp = ros::Time::now();
     line_strip.ns = "lines";
     line_strip.action = visualization_msgs::Marker::ADD;
     line_strip.pose.orientation.w = 1.0;
	 
	 line_strip.id = 1;
     line_strip.type = visualization_msgs::Marker::LINE_LIST;
 
     //Setting width
     line_strip.scale.x = 0.02f;

      // Line strip is green
      line_strip.color.g = 1.0f;
      line_strip.color.a = 1.0;
  
	  geometry_msgs::Point p;
	  if(flag)
	  {
		  for(i=0;i<ref_point.size();i++)
		  {
			  p.x=(int32_t)ref_point[i].x;
			  p.y=(int32_t)ref_point[i].y;
			  p.z=0;
			  line_strip.points.push_back(p);
			  //ROS_INFO("Publishing points p[%0.2f,%0.2f]  to marker loop[%d]",p.x,p.y,i);
		
			  marker_pub.publish(line_strip);
		  }
		/*
		p.x=0.0;
		p.y=0.0;
		p.z=0.0;
		line_strip.points.push_back(p);
		line_strip.points.push_back(p);
		marker_pub.publish(line_strip);
		*/
	  }

	geometry_msgs::Twist cmd;
	double thresh=1.0;
	cmd.linear.x=2;
	cmd.linear.z=2;
	cmd.linear.y=0;
	cmd.linear.z=0;
	cmd.angular.x=0;
	cmd.angular.y=0;
	cmd.angular.z=0;
		
	bool flag1=false;
	
	//int max_n=0,min_n=360;
	for(i=0;i<MAX_COUNT;i++)
	{
		if((msg->intensities[i]==1.0f) && (msg->ranges[i]<thresh))
		{
			
			flag1=true;
			break;
		}
	}
	
	if(true==flag1)
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
 
int main( int argc, char** argv )
{
   ros::init(argc, argv, "lines");
   ros::NodeHandle n;
   marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
   ros::Subscriber scan_val=n.subscribe("base_scan",10,scanCallback);
   publ=n.advertise<geometry_msgs::Twist>("cmd_vel",10); 

   ros::Rate r(10);
   //ROS_INFO("main");
	
   while (ros::ok())
   {
	   ros::spinOnce();
       r.sleep();
    }
 }
