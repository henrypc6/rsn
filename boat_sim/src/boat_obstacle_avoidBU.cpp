#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "boat_utilBU.cpp"
#include <unistd.h>

///////////////
#define LOST_COW_CASE 0
#define MOVE_TO_TARGET_CASE 1
#define EMERGENCY_CASE 2
///////////////

int main (int argc, char** argv)
{
  ros::init(argc,argv,"moveit");
  ros::NodeHandle nh("/");
  

  velout = nh.advertise<geometry_msgs::Twist>("cmd_vel",20);
  
  ROS_INFO("Up and running");

  //target point assigned
  target_point.x=0;
  target_point.y=100;
  target_point.z=0;
  target_theta=0;

  //get the current position of the robot
  ros::Subscriber Position = nh.subscribe("base_pose_ground_truth",10,PosCallback);
  
  // Subscribe to echo topics
  ros::Subscriber laserin = nh.subscribe("base_scan",10,EchoCallback);
  ros::Rate r(10);

  ros::spinOnce(); //run the code twice to get the current_point value
  r.sleep();
  ros::spinOnce();
  r.sleep();

  //////////////////////////moving to target///////////////////

  double lost_cow_dir=1;
  double lost_cow_dist=14;
        
  int draw_circle_output=0;//0 for moving in lost cow, 2 for emergency stop, 1 for moving to target straight
  int move_to_point_output=1;
      
  while(XY_dist(current_point.x-target_point.x,current_point.y-target_point.y)>ERR_CIRCLE && draw_circle_output!=EMERGENCY_CASE && ros::ok())
    {
      move_to_point_output = move_to_point(current_point,current_theta,target_point,target_theta);

      if (move_to_point_output == 1) //encounter obstacles, need to draw circles
	draw_circle_output = draw_circle(current_theta,lost_cow_dir);
      
      if(XY_dist(current_point.x-target_point.x,current_point.y-target_point.y)<ERR_CIRCLE*2)
	{
	  std::cout<<"here"<<std::endl;
	  return 0;
	}
      printf("draw_circle_output = %d\n",draw_circle_output);

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
      
      
      ros::spinOnce();
      usleep(500);
    }
  return 0;
}

