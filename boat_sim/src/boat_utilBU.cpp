
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <unistd.h>
#include "../../algorithm/line_detect.cpp"

typedef geometry_msgs::Point P;

ros::Publisher velout;

//nav_msgs::Odometry current
P current_point, target_point,target_dist;
geometry_msgs::Quaternion Quat;

double pose_vector_angle=0,angle_sign=1;
double point_target_angle;
double current_theta,tmp_theta,target_theta;
double line_tan_theta;

//distance to target
double ob_x,ob_y;
double echo_dist=0;

//flags
bool circle=0;//1 for circles 0 for straight line
bool scanning_finished=0;//1 for finished 0 for scanning
bool find_opening=0; //1 for find openings, 0 for cant find opening
int mode=1;//1 for first cirle, 2 for straight line, 3 for second circle

//velocity msgs
geometry_msgs::Twist velmsg;

//line_detect class
Line_Detect ls_line(0,0);

//mode cases
#define FIRST_CIRCLE 1
#define STRAIGHT_LINE 2
#define SECOND_CIRCLE 3

//general defination
#define MAX_AVOID_DIST 4
#define ERR_CIRCLE 2
#define ANGLE_DIFF 0.1
#define FORWARD_SPEED 1
#define ROTATION_SPEED 0.4

/////////////////////// Callbacks ///////////////////////
void PosCallback(const nav_msgs::Odometry& new_point)
{

  current_point.x=new_point.pose.pose.position.x;
  current_point.y=new_point.pose.pose.position.y;
  current_point.z=new_point.pose.pose.position.z;

  Quat.z=new_point.pose.pose.orientation.z;
  Quat.w=new_point.pose.pose.orientation.w;
  
  current_theta = atan2(2*Quat.z*Quat.w,1-2*Quat.z*Quat.z);
  
  //printf("Current_theta is %f\n",current_theta);
  
  if (current_theta<0)
    {
      current_theta = 2*3.1415926+current_theta;
    }

  if (echo_dist>0 && echo_dist<20)
    {
      ob_x=current_point.x+echo_dist*cos(current_theta);
      ob_y=current_point.y+echo_dist*sin(current_theta);
    }
  else
    {
      ob_x=0;
      ob_y=0;
    }
}

void EchoCallback(const sensor_msgs::LaserScan& echoScan)
{
  int count = (echoScan.angle_max-echoScan.angle_min)/echoScan.angle_increment;
  double dist=10000;
  for (int i=0;i<count;i++)
    {
      if (dist>echoScan.ranges[i])
        dist=echoScan.ranges[i];
    }
  
  echo_dist = dist;

}


void Target_To_Current()
{
  target_dist.y=target_point.y-current_point.y;
  target_dist.x=target_point.x-current_point.x;

  point_target_angle = atan2(target_dist.y,target_dist.x);
  
  pose_vector_angle=(point_target_angle-current_theta)*180/3.1415;
  if (pose_vector_angle>=0)
    angle_sign=1;
  else
    angle_sign=-1;
  pose_vector_angle=abs(pose_vector_angle);

  if (pose_vector_angle>180)
    {
      pose_vector_angle=360-pose_vector_angle;
      angle_sign=-angle_sign;
    }

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
  Target_To_Current();
  //printf("%f\n",fabs(point_target_angle-current_theta));
  if (ob_x==0 && ob_y==0 && fabs(point_target_angle-current_theta)<ANGLE_DIFF)
    return true;
  else
    return false;
}

int draw_circle(double ot,double dir)
{
 
  printf("drawing circle\n");
  velmsg.linear.x=FORWARD_SPEED;
  
  if (echo_dist<1)
    {
      velmsg.angular.z=0;
      velmsg.linear.x=0;
      velout.publish(velmsg);
      return 2; //almost hit the obstacle, STOP NOW!
    }
  
  
  while((current_theta-ot)>=0 || (current_theta-ot)<(-1*ANGLE_DIFF) && ros::ok())
    {

      record_ob();
      if (detect_opening())
	return 1;//now we can just move forward
      
      velmsg.angular.z=0.4*dir;
      velout.publish(velmsg);
      ros::spinOnce();
      usleep(100);
    }

  line_tan_theta=atan(ls_line.get_coef());


  velmsg.angular.z=0;
  velmsg.linear.x=0;
  velout.publish(velmsg);
  ros::spinOnce();
  usleep(500);
  return 0; //finished, move to next point
  
}


int move_to_point(P op,double ot, P tp, double tt)
{
  double dist=XY_dist(tp.x-op.x,tp.y-op.y);
  double angle=atan2((tp.y-op.y),(tp.x-op.x));
  printf("angle is %f\n", angle);
  if (angle<0)
    angle=3.1415*2+angle;

  mode =1;
  ros::Rate r(10);
  printf("moving to point %f, %f\n",tp.x,tp.y);
  
  while(fabs(current_point.x-tp.x)>ERR_CIRCLE || fabs(current_point.y-tp.y)>ERR_CIRCLE && ros::ok())
    {

      ros::spinOnce();
      velmsg.linear.x=FORWARD_SPEED;
      
      double ca=fabs(current_theta-angle);
            
      if (echo_dist<MAX_AVOID_DIST && ca<ANGLE_DIFF)
	{
	  printf("stop to draw circles\n");
	  velmsg.linear.x=0;
	  velmsg.angular.z=0;
	  velout.publish(velmsg);
	  return 1;
	}
      //moving to points
      
      if (ca<ANGLE_DIFF)
	mode=STRAIGHT_LINE;

      if (ca>ANGLE_DIFF && mode==FIRST_CIRCLE)//turning the first circle
	{
	  velmsg.angular.z=ROTATION_SPEED;
	}
      else if (dist>XY_dist(current_point.x-op.x,current_point.y-op.y) && mode==STRAIGHT_LINE)
	{
	  velmsg.angular.z=0;
	}
      else if (fabs(dist-XY_dist(current_point.x-op.x,current_point.y-op.y))<ERR_CIRCLE && mode!=FIRST_CIRCLE)
      	{
      	  mode = SECOND_CIRCLE;
      	}
      if (mode == SECOND_CIRCLE && fabs(tt-current_theta)>ANGLE_DIFF)
      	{
      	  velmsg.angular.z=ROTATION_SPEED;
      	}
      else if (mode == SECOND_CIRCLE && fabs(tt-current_theta)<ANGLE_DIFF)
      	{
      	  velmsg.angular.z=0;
      	  velmsg.linear.x=0;
      	}
      
      velout.publish(velmsg);
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
      move_to_point(current_point,current_theta,lost_cow_target,line_tan_theta);
    }
  if (dir==-1)
    {
      lost_cow_target.x=current_point.x-dist*3*cos(line_tan_theta);
      lost_cow_target.y=current_point.y-dist*3*sin(line_tan_theta);
      move_to_point(current_point,current_theta,lost_cow_target,line_tan_theta);
    }
  

  
}
