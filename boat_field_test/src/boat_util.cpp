// #ifndef _BOAT_UTIL_H_
// #define _BOAT_UTIL_H_

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <unistd.h>
#include "../../algorithm/line_detect.cpp"
//#include <control/WaypointNavigation.h>

//general defination
#define D_EMERGENCY 1
#define MAX_AVOID_DIST 20
#define ERR_CIRCLE 5
#define ANGLE_DIFF 0.1
#define PROPELLER 100 //-128 to 127
#define STEER 50      //-60 to 60

typedef geometry_msgs::Point P;

ros::Publisher steer,propeller;
//ros::ServiceClient wpclient;
std_msgs::Int8 velout_steer, velout_propeller;

//nav_msgs::Odometry current                                                                
P current_point, target_point,target_dist;
geometry_msgs::Quaternion Quat;

double pose_vector_angle=0,angle_sign=1;
double point_target_angle;
double current_theta,tmp_theta,target_theta;
double line_tan_theta;

//distance to target                                                          
double ob_x,ob_y;
double echo_dist=30;

//flags
bool circle=0;//1 for circles 0 for straight line
bool scanning_finished=0;//1 for finished 0 for scanning
bool find_opening=0; //1 for find openings, 0 for cant find opening
int mode=1;//1 for first cirle, 2 for straight line, 3 for second circle

//line_detect class
Line_Detect ls_line(0,0);

//mode cases
#define FIRST_CIRCLE 1
#define STRAIGHT_LINE 2
#define SECOND_CIRCLE 3


/////////////////////// Callbacks ///////////////////////                         
void EchoCallback(const std_msgs::Float32& dist)
{

 echo_dist=dist.data;

}

void PosCallback(const geometry_msgs::PoseWithCovarianceStamped& state)
{
  double z=state.pose.pose.orientation.z;
  double w=state.pose.pose.orientation.w;
  current_theta=atan2(2*z*w,1-2*z*z);
  
  ob_x=state.pose.pose.position.x+echo_dist*cos(current_theta);
  ob_y=state.pose.pose.position.y+echo_dist*sin(current_theta);

}


double XY_dist(double x,double y)
{
  return sqrt(x*x+y*y);
}

void record_ob()
{
  //record obstacle data
  if (ob_x!=0 && ob_y!=0)
    {
      ls_line.Constr_Vec(ob_x,ob_y);
      ls_line.ls_method(ob_x,ob_y);
    }

}

bool detect_opening()
{
  //Target_To_Current();
  //printf("%f\n",fabs(point_target_angle-current_theta));
  if (ob_x==0 && ob_y==0 && fabs(point_target_angle-current_theta)<ANGLE_DIFF)
    return true;
  else
    return false;
}


int move(P op, P tp, bool circle)
{
  double angle=atan2((tp.y-op.y),(tp.x-op.x));
 
  if (angle<0)
    angle=3.1415*2+angle;

  ros::Rate r(10);
  
  while(fabs(current_point.x-tp.x)>ERR_CIRCLE || fabs(current_point.y-tp.y)>ERR_CIRCLE && ros::ok())
    {
      ros::spinOnce();
      double angle_diff=current_theta-angle; //positive turn right, negative turn left
      
      velout_steer.data=0;
      velout_propeller.data=PROPELLER;
      if (!circle) //not doing circle
	{
	  if (angle_diff>0)
	    {
	      velout_steer.data=std::max(angle_diff/3.14*180,60.00);
	    }
	  if(angle_diff<0)
	    {
	      velout_steer.data=std::min(angle_diff/3.14*180,-60.00);
	    }
	}
      else if (circle) //turning circle
	{
	  
	  record_ob();//record obstacle
	  	  
	  angle_diff=fabs(angle_diff);
	  velout_steer.data=std::max(angle_diff/3.14*180,60.00);
	}
      steer.publish(velout_steer);
      propeller.publish(velout_propeller);
      
      r.sleep();
      
    }
  return 0;
}

int move_to_point(P op, P tp)
{
  double angle=atan2((tp.y-op.y),(tp.x-op.x));
 
  if (angle<0)
    angle=3.1415*2+angle;

  ros::Rate r(10);
  
  while(fabs(current_point.x-tp.x)>ERR_CIRCLE || fabs(current_point.y-tp.y)>ERR_CIRCLE && ros::ok())
    {
      ros::spinOnce();
      double angle_diff=current_theta-angle; //positive turn right, negative turn left
      
      velout_steer.data=0;
      velout_propeller.data=PROPELLER;
      
      if (angle_diff>0)
	{
	  velout_steer.data=std::max(angle_diff/3.14*180,60.00);
	}
      if(angle_diff<0)
	{
	  velout_steer.data=std::min(angle_diff/3.14*180,-60.00);
	}

      if (echo_dist<MAX_AVOID_DIST)
	{
	  std::cout<<"stop and draw circles\n"<<std::endl;
	  velout_steer.data=0;
	  velout_propeller.data=0;
	  
	  steer.publish(velout_steer);
	  propeller.publish(velout_propeller);
	  return 1;
	  
	}

       // if (echo_dist<D_EMERGENCY)
       // 	{
       // 	  std::cout<<"stop and draw circles\n"<<std::endl;
       // 	  velout_steer=0;
       // 	  velout_propeller=0;
	  
       // 	  steer.publish(velout_steer);
       // 	  propeller.publish(velout_propeller);
       // 	  return 2
	  
       // 	}

      steer.publish(velout_steer);
      propeller.publish(velout_propeller);
      
      r.sleep();
      
    }
  return 0;
}

void lost_cow_move(double dir,double dist)
{
  P lost_cow_target;
  if (dir==1)
    {
      lost_cow_target.x=current_point.x+dist*cos(line_tan_theta);
      lost_cow_target.y=current_point.y+dist*sin(line_tan_theta);
      move(current_point,lost_cow_target,0);
    }
  if (dir==-1)
    {
      lost_cow_target.x=current_point.x-dist*3*cos(line_tan_theta);
      lost_cow_target.y=current_point.y-dist*3*sin(line_tan_theta);
      move(current_point,lost_cow_target,0);
    }
  
}

int draw_circle()
{
  
  P new_p;
  new_p.x = current_point.x-2;
  new_p.y = current_point.y-2;

  move(current_point,new_p,1);

  
  line_tan_theta=atan(ls_line.get_coef());

  velout_steer.data=0;
  velout_propeller.data=0;
  
  steer.publish(velout_steer);
  propeller.publish(velout_propeller);

  return 0; //finished, move to next point
  
}


//#endif
