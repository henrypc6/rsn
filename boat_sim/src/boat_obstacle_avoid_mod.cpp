#include <stdlib.h>
#include <iostream>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
//#include <control/WaypointNavigation.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <unistd.h>
//#include <control/WaypointNavigation.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <unistd.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include "algorithm/lost_cow_move.cpp"
#include "algorithm/line_detect.cpp"
#include <signal.h>
#include <fstream>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#define UNIT_LENGTH 10
#define MAX_TURN_STEER 50 //-60 to 60
#define MAX_TURN_PROP 100 //-128 to 127
#define ANGLE_DIFF 0.02
#define D_EMERG 2
#define AVOID_DIST 5
#define ERR_CIRCLE 3
#define CIRCLE_LIM 325 //325 degrees for completing the turn
//define the Lost_Cow_Move class

//ros::ServiceClient wpclient;
ros::Publisher velout;
geometry_msgs::Twist velmsg;
typedef geometry_msgs::Point P;

bool currently_recording;
float max_range;

P current_point, target_point,target_dist;
geometry_msgs::Quaternion Quat;

double step_origin_x;
double step_origin_y;
bool continueon;

static void sig_handler(int sig, siginfo_t *si, void* unused){
	printf("Sig received. Hard closing\n");
	exit(EXIT_SUCCESS);
}


void setsigs(){
	struct sigaction sa;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction=sig_handler;

	if(sigaction(SIGTERM,&sa,NULL)==-1) {
		fprintf(stderr,"Error with signal handler\n");
		exit(EXIT_FAILURE);
	}
	if(sigaction(SIGINT,&sa,NULL)==-1) {
		fprintf(stderr,"Error with signal handler\n");
		exit(EXIT_FAILURE);
	}
}



//define the global data
double current_theta;
double echo_dist;
double ob_x,ob_y;
double line_theta;
double current_x,current_y;
double point_target_angle;

bool on=false;
double prev_step;


//target point assigned 
double start_x=0;
double start_y=30;

double shore_x=0;
double shore_y=30;

double target_x=0;
double target_y=30;

double prev_x;
double prev_y;

//set the init_point for the LCM algorithm
double init_x,init_y;

//line_detect class
Line_Detect ls_line(0,0);
Lost_Cow_Move LCM(-1,UNIT_LENGTH); // direction and unit_length

void PosCallback(const nav_msgs::Odometry& new_point)
{

  current_x=new_point.pose.pose.position.x;
  current_y=new_point.pose.pose.position.y;

  Quat.z=new_point.pose.pose.orientation.z;
  Quat.w=new_point.pose.pose.orientation.w;
  
  current_theta = atan2(2*Quat.z*Quat.w,1-2*Quat.z*Quat.z);

  if (echo_dist>0 && echo_dist<30)
    {
      ob_x=current_x+echo_dist*cos(current_theta);
      ob_y=current_y+echo_dist*sin(current_theta);
    }
  else
    {
      ob_x=-100;
      ob_y=-100;
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
    
  if (currently_recording)
    {
      max_range = fmax(max_range,dist);
      ob_x=current_x+dist*cos(current_theta);
      ob_y=current_y+dist*sin(current_theta);
    
    if (dist>29)
      {
	ob_x=-100;
	ob_y=-100;
      }
    
    if (ob_x!=-100 && ob_y!=-100)
      {
	ls_line.Constr_Vec(ob_x,ob_y);
	ls_line.ls_method(ob_x,ob_y);
      }
  }

}

bool detect_opening()
{
  //printf("%f\n",fabs(M_PI/2-current_theta));
  if (ob_x==-100 && ob_y==-100 && fabs(M_PI/2-current_theta)<ANGLE_DIFF*4)
    {
     
      return true;
    }
  else
    return false;
}

bool draw_circle(double dir)
{
        double current_theta_tmp;
	double draw_init_theta = current_theta;
  
	if (draw_init_theta<0)
	  draw_init_theta=3.142*2+draw_init_theta;
	
	if (current_theta<0)
	  current_theta_tmp=3.14*2+current_theta;
	else
	  current_theta_tmp=current_theta;
  
	max_range = -1.0;
	currently_recording=true;

	double angularDist=fmod((current_theta_tmp-draw_init_theta)*dir+4*M_PI,2*M_PI);
	
	int state=1;
	while(state!=3)//positive- counterclock wise-assumption
	  {
	    if (detect_opening())
	      return false;
	    
	    switch (state)
	      {
	      case 1:
		if (angularDist>(M_PI/2) && angularDist<((M_PI/2)*3))
		  state=2;
		else
		  {
		    velmsg.angular.z=MAX_TURN_STEER*dir;
		    velmsg.linear.x=MAX_TURN_PROP;
		    velout.publish(velmsg);

		    usleep(1e5); //10 hz
		    if (current_theta<0){
		      current_theta_tmp=M_PI*2+current_theta;
		    } else{
		      current_theta_tmp=current_theta;
		    }
		    angularDist=fmod((current_theta_tmp-draw_init_theta)*dir+4*M_PI,2*M_PI);
		  }
		break;
		
	      case 2:
		if (angularDist>((M_PI/2)*3))
		  state=3;
		else
		  {
		    velmsg.angular.z=MAX_TURN_STEER*dir;
		    velmsg.linear.x=MAX_TURN_PROP;
		    velout.publish(velmsg);

		    usleep(1e5); //10 hz
		    if (current_theta<0){
		      current_theta_tmp=M_PI*2+current_theta;
		    } else{
		      current_theta_tmp=current_theta;
		    }
		    angularDist=fmod((current_theta_tmp-draw_init_theta)*dir+4*M_PI,2*M_PI);
		  }
		break;
	      }
	  }
	currently_recording=false;
	//return max_range>0.0;
	return true;
}


int move(double tx, double ty,bool stop,bool unit_step)
{
  //double dist=XY_dist(tp.x-current_point.x,tp.y-current_point.y);
  double angle=atan2((ty-current_y),(tx-current_x));
  //printf("angle is %f\n", angle);

  ros::Rate r(10);
  printf("moving to point %f, %f\n",tx,ty);
  
  while(fabs(current_x-tx)>ERR_CIRCLE || fabs(current_y-ty)>ERR_CIRCLE && ros::ok())
    {

      ros::spinOnce();
            
      velmsg.linear.x=MAX_TURN_PROP;
            
      double ca=current_theta-angle;    
      
      if (ca>M_PI)
	ca=M_PI-ca;
      if (ca<(-1)*M_PI)
	ca=(-1)*(ca+M_PI);
      
      if (fabs(fabs(ca)-M_PI)<0.1)
	ca=fabs(ca);
      
      
      velmsg.angular.z=MAX_TURN_STEER*(-1)*ca/10;
      
      if (stop)
	{
	  if (echo_dist>AVOID_DIST)
	    {
	      usleep(1e5);
	      ROS_INFO("Waiting for echo nearby");
	    }
	  else
	    {
	      printf("too close, moving circles\n");
	      return 1;
	 
	    }
	}
      // double t_dist=sqrt((step_origin_x-current_x)*(step_origin_x-current_x)+(step_origin_y-current_y)*(step_origin_y-current_y));
      // if (t_dist>prev_step)
      // 	{	
      // 	  on=true;
      // 	  prev_step=0;
      // 	}
      
      // if (unit_step && on)
      // 	{
	  
      // 	   printf("%f\n",t_dist);
      // 	   if (t_dist>5)
      // 	    {
	      
      // 	      printf("step \n");
      	 
      // 	      step_origin_x=current_x;
      // 	      step_origin_y=current_y;
      // 	      continueon=draw_circle(LCM.get_dir());
      // 	      if (!continueon)
      // 		{
      // 		  return 0;
      // 		}
      // 	    }
      //        }
      
      velout.publish(velmsg);
      r.sleep();
      
    }
  printf("complete move\n");
  return 0;
}

void Target_To_Current()
{
  target_dist.y=target_y-current_y;
  target_dist.x=target_x-current_x;

  point_target_angle = atan2(target_dist.y,target_dist.x);
  
}



int main(int argc, char** argv)
{

  continueon=true;  
  currently_recording=false;
  
  ros::init(argc,argv,"moveit");
  ros::NodeHandle nh;
	
  velout = nh.advertise<geometry_msgs::Twist>("cmd_vel",20);
  
  ROS_INFO("Up and running");
  
  ros::Subscriber Position = nh.subscribe("base_pose_ground_truth",10,PosCallback);
  
  // Subscribe to echo topics
  ros::Subscriber laserin = nh.subscribe("base_scan",10,EchoCallback);
  
// Subscribe
  
  ros::AsyncSpinner spinner(1);
  

  spinner.start();
  setsigs();
  sleep(3);  

  init_x=current_x;
  init_y=current_y;
  LCM.set_init_point(init_x,init_y);

  
  move(shore_x,shore_y,true,false); //only move thats not blocking
  
  //now first circle to construct line
  printf("current_x %f, current_y %f",current_x,current_y);
  continueon = draw_circle(LCM.get_dir());
  
  if (!continueon){
    ROS_WARN("no obstacle detected!");
    exit(EXIT_FAILURE);
  }
	
  
  while(continueon){

    LCM.set_line_theta(0.0);
    prev_step=LCM.get_step()*3/2;
    LCM.next_point();
    target_x = LCM.get_wp_x();
    target_y = LCM.get_wp_y();
    
    step_origin_x=current_x;
    step_origin_y=current_y;
    
    prev_x=target_x;
    prev_y=target_y;
    move(target_x,target_y,false,true); 

    move(shore_x,shore_y,true,false);
	  
    
    continueon = draw_circle((-1)*LCM.get_dir()); //-1 because get_dir is the
    
    if (!continueon)
      continue;
    
    move(prev_x,prev_y,false,false);
  }
  
  printf("done\n");
  move(current_x,current_y+15,false,false); 
  move(start_x,start_y,false,false);
  ROS_INFO("End of execution.");
  return 0;
  
}
