#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <Eigen/Dense>
#include <math.h>

#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "../../algorithm/line_detect.cpp"
#include "../../algorithm/gnuplot_i.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/NavSatFix.h"

void DistAssign(const std_msgs::Float32&);
void xy_assign(const geometry_msgs::PoseWithCovarianceStamped&);
//void xy_assign_gps(const sensor_msgs::NavSatFix&);

ros::Publisher ob_ls_line;

double ob_dist;
double x=0,y=0,theta=0,x_gps=0,y_gps=0;
//obstacle points and vector
geometry_msgs::Point line_spec;
Line_Detect ls_line(0,0);

//gnuplot
Gnuplot g("boat_position");
Gnuplot g_gps("boat_position_gps");
std::vector<double> X,Y,X_gps,Y_gps;

//g.set_style("lines");

Eigen::Matrix3d a(3,3);

void DistAssign(const std_msgs::Float32& msg)
{
  ob_dist=msg.data;
 
}

void xy_assign(const geometry_msgs::PoseWithCovarianceStamped& state)
{
  double z=state.pose.pose.orientation.z;
  double w=state.pose.pose.orientation.w;
  theta=atan2(2*z*w,1-2*z*z);
  
   x=state.pose.pose.position.x+ob_dist*cos(theta);
  y=state.pose.pose.position.y+ob_dist*sin(theta);

  
  
  // if (X.size()>900)
  //   {
  //     X.clear();
  //     Y.clear();
  //   }
    
  // if (x!=X.back() && y!=Y.back())
  //   {
  //     X.push_back(x);
  //     Y.push_back(y);
    
  //     g.reset_plot();
  //     printf("%d\n",X.size());
  //     g.set_style("lines");
  //     g.set_yrange(-30,30);
  //     g.set_xrange(50,150);
      
  //     g.plot_xy(X,Y,"boat_position");
      
  //     g.remove_tmpfiles();
  //   }
    
  if (ob_dist>0)
    {
      ls_line.Constr_Vec(x,y);
      ls_line.ls_method(x,y);
      
      line_spec.x=ls_line.get_coef(); //m
      line_spec.y=ls_line.get_offset();//b
      
      printf("m = %f\nb = %f\n",line_spec.x,line_spec.y);
      ob_ls_line.publish(line_spec);
    }

      
}

int main(int argc, char** argvs)
{
  ros::init(argc,argvs,"echo_ls_line");
  ros::NodeHandle n;
  
  //subscribe
  ros::Subscriber sub = n.subscribe("Echo_Dist",10,DistAssign);
  ros::Subscriber sub1= n.subscribe("robot/state",10,xy_assign);
  //ros::Subscriber sub2= n.subscribe("fix",10,xy_assign_gps);
  //subscribe to status.x, status.y, and status.theta
 
  ob_ls_line = n.advertise<geometry_msgs::Point>("obstacle_line",10);
  
  X.push_back(0);
  Y.push_back(0);
  //g.set_style("lines"); 
  while(ob_dist<=0 && ros::ok())
    {
      ros::spinOnce();
      usleep(100);
    }
  
   ros::spin();
  
}




