//TO DO list
// Add signal handling to close the file pointer

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "../../algorithm/echo_data_filter.cpp"
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "../../algorithm/gnuplot_i.hpp"
#include "Eigen/Dense"

void echo_callback(const std_msgs::UInt8MultiArray&);

FILE *fd;
Gnuplot g("dist_data");
ros::Publisher echo_dist;
ObstacleDist Odist;
//Eigen::MatrixXd calibrate_data(10,500);
int calibrate_cnt=0;

int min_view_dist=0;
double min_sig_strength=0;

void echo_callback(const std_msgs::UInt8MultiArray& msg)
{
  //convert msg.data.data into vector<int>
  std::vector<int> dist;
  std::vector<int> argvs;

  int i=0;
  int range;

  while(msg.data[i]!=0xFC)
    {
      dist.push_back(msg.data[i]);
      i++;
    }
  i=i+1;
  while(i<msg.data.size())
    {
      argvs.push_back(msg.data[i]);

      i++;
    }

  if (argvs.size()!=9)
    printf("Seperator of the msg failed\n");
  range = argvs[0];
  

  //filtering the echo data
  double max_idx=min_view_dist*dist.size()/range;
  for (int i=0;i<max_idx;i++)
    dist[i]=0;
  
  for (int i=max_idx;i<dist.size();i++)
    {
      if (dist[i]<min_sig_strength)
	dist[i]=0;
    }
  
  //plot using gnuplot
  std::vector<double> dist_f(dist.begin(),dist.end());

  // g.reset_plot();
  // g.set_yrange(0,140);
  // g.plot_x(dist_f);
  // g.remove_tmpfiles();
    
  //write the received data to file
  // ROS_INFO("Echo data received...printing to file...");
  
   for (unsigned int i=0;i<dist.size();i++)//starting the first byte from the first range
      fprintf(fd,"%d ",dist[i]);
    fprintf(fd,"\n");
  
  Odist= ObstacleDist(dist_f,range);
  Odist.SetFilter(min_view_dist,min_sig_strength);
  //Avr_Max() method and if it doesnt work, use the Max_Method()
  // -1 means there is no obstacle 
  std_msgs::Float32 D;
  D.data=Odist.Avr_Max();
  
  if (D.data==0)
    D.data=Odist.Max_Method();
  
  if (D.data==0 || fabs(D.data-range)<2)
    D.data=-1;
  
  printf("%f\n",D.data);
  echo_dist.publish(D);
  
}


int main(int argc, char** argvs)
{
   fd = fopen(argvs[3],"a+");
  
  min_view_dist=atoi(argvs[1]);
  min_sig_strength=atoi(argvs[2]);
  
  
  ros::init(argc,argvs,"echo_lis");
  ros::NodeHandle n;
  echo_dist = n.advertise<std_msgs::Float32>("Echo_Dist",10);
  ros::Subscriber sub = n.subscribe("Echo_Data",10,echo_callback);
  ros::spin();
  
  return 0;
}



