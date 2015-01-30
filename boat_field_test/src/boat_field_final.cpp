#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include "boat_util.cpp"
#include <unistd.h>

///////////////
#define LOST_COW_CASE 0
#define MOVE_TO_TARGET_CASE 1
#define EMERGENCY_CASE 2
///////////////



int main (int argc, char** argv) 
{  
  ros::init(argc,argv,"moveit");
  ros::NodeHandle nh;

  //serviceclient init
  //  wpclient=nh.serviceClient<control::WaypointNavigation>("/control/command");  


  //publishing the boat control topic
  steer = nh.advertise<std_msgs::Int8>("moter_cmd/steer",1);
  propeller=nh.advertise<std_msgs::Int8>("motor_cmd/propeller",1);
  
  ROS_INFO("Up and running");

  //target point assigned
  target_point.x=0;
  target_point.y=0;
  target_point.z=0;
  target_theta=0;

  //get the current position of the robot
  ros::Subscriber Position = nh.subscribe("robot/state",10,PosCallback);
  
  // Subscribe to echo topics
  ros::Subscriber Echo = nh.subscribe("Echo_Dist",10,EchoCallback);
  
  // std::cout<<"Echo dist is "<<echo_dist<<std::endl;
  // std::cout<<"Theta is "<<current_theta<<std::endl;
  
  //ros::spin();
  
  ros::Rate r(10); 

  ros::spinOnce(); //run the code twice to get the current_point value
  r.sleep();
  ros::spinOnce();
  r.sleep();

  //////////////////////////moving to target///////////////////

  double lost_cow_dir=1;
  double lost_cow_dist=5; //the distance of each step
        
  int draw_circle_output=0;//0 for moving in lost cow, 2 for emergency stop, 1 for moving to target straight
  int move_to_point_output=1;
      
 
  move_to_point_output = move_to_point(current_point,target_point);
  if (move_to_point_output == 1) //encounter obstacles, need to draw circles
    draw_circle_output = draw_circle();
  
  //finished when reach the target
  if(XY_dist(current_point.x-target_point.x,current_point.y-target_point.y)<ERR_CIRCLE*2)
    {
      std::cout<<"here"<<std::endl;
      return 0;
    }
  printf("draw_circle_output = %d\n",draw_circle_output);
  while(lost_cow_dist<30) 
    {
      switch (draw_circle_output)
	{
	case LOST_COW_CASE: //moving to next point in lost cow approach
	  lost_cow_move(lost_cow_dir,lost_cow_dist);
	  break;
	case MOVE_TO_TARGET_CASE://find opening
	  printf("Continue going\n");	      
	  break;
	case EMERGENCY_CASE://emergency stop
	  printf("Emergency Stop\n");	      
	  break;
	}
      lost_cow_dir=lost_cow_dir*(-1);
      lost_cow_dist=lost_cow_dist*2;
    
      draw_circle_output=draw_circle();
    }
  return 0;
}
