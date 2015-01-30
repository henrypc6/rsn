#include <ros/ros.h>
#include <unistd.h>
#include <stdlib.h>
#include "../../algorithm/line_detect.cpp"
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int8.h>

double Min_Turning_Dist,ob_dist=0,theta,ob_x=0,ob_y=0;
ros::Publisher steer,propeller;
std_msgs::Int8 velout_steer, velout_propeller;

//line detect class
Line_Detect ls_line(0,0);

#define ANGLE_DIFF 0.01
#define STEER 50 //-60 to 60
#define PROPELLER 100 //-128 to 127
#define TIME_LIM 5 //5 seconds

void record_ob()
{
  //record obstacle data
  if (ob_x!=0 && ob_y!=0)
    {
      ls_line.Constr_Vec(ob_x,ob_y);
      ls_line.ls_method(ob_x,ob_y);
    }
}

void draw_circle(double ot)
{


  double time=0;
  double pause_time=1; //1 seconds


  printf("drawing circle...\n");
  while(time<TIME_LIM && ros::ok())
  {      
      record_ob();
           
      velout_steer.data=STEER;
      velout_propeller.data=PROPELLER;

      steer.publish(velout_steer);
      propeller.publish(velout_propeller);
       
      ros::spinOnce();
      time =time+pause_time;
      printf("time is %f\n",time);
      sleep(pause_time);
    }

  //run parallel to the line
  printf("running parallel...\n");
  double ls_angle=atan(ls_line.get_coef());
  printf("theta is %f, ls_angle is %f\n",theta,ls_angle);

  while(fabs(theta-ls_angle)>ANGLE_DIFF && ros::ok())
    {
      printf("theta is %f, ls_angle is %f\n",theta,ls_angle);
      
      record_ob();
      velout_steer.data=STEER;
      velout_propeller.data=PROPELLER;
      
      steer.publish(velout_steer);
      propeller.publish(velout_propeller);

      ros::spinOnce();
      sleep(pause_time);
     
    }
  
  //keep straight for TIME_LIM and stop
  printf("keeping straight...\n");
  time=0;
  while(time<TIME_LIM && ros::ok())
    {
      velout_steer.data=0;
      velout_propeller.data=PROPELLER;
      
      steer.publish(velout_steer);
      propeller.publish(velout_propeller);

      ros::spinOnce();
      sleep(pause_time);
    }
  
  velout_steer.data=0;
  velout_propeller.data=0;
  
  steer.publish(velout_steer);
  propeller.publish(velout_propeller);
  
  ros::spinOnce();
  

  
}

void Echo_DistCallback(const std_msgs::Float32& dist)
{

  ob_dist=dist.data;

}

void xy_assign(const geometry_msgs::PoseWithCovarianceStamped& state)
{
  double z=state.pose.pose.orientation.z;
  double w=state.pose.pose.orientation.w;
  theta=atan2(2*z*w,1-2*z*z);
  
  ob_x=state.pose.pose.position.x+ob_dist*cos(theta);
  ob_y=state.pose.pose.position.y+ob_dist*sin(theta);

}


int main(int argc, char** argv)
{
  Min_Turning_Dist=atoi(argv[1]);
  
  sleep(60);//sleep for 60 seconds wait to start
  
  ros::init(argc,argv,"boat_field_test");
  ros::NodeHandle n;
  
  //publishing the boat control topic
  steer = n.advertise<std_msgs::Int8>("moter_cmd/steer",1);
  propeller=n.advertise<std_msgs::Int8>("motor_cmd/propeller",1);

  ROS_INFO("Up and running");

  ros::Subscriber sub = n.subscribe("Echo_Dist",10,Echo_DistCallback);
  ros::Subscriber sub1= n.subscribe("robot/state",10,xy_assign);
  
  while(ros::ok())
    {
        if (ob_dist<Min_Turning_Dist && ob_dist>0)
	  {
	    draw_circle(theta);
	    ob_dist=0;
	  }

	ros::spinOnce();

	sleep(1);
    }
  

  return 0;
   
}
