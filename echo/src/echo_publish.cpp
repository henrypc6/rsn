#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include "echo_publish.h"
#include "SerialPort.h"

void Data_Publish(int argc,char **argvs, int fd)
{
  int pub_Hz=atoi(argvs[8]);
  int queue_size=atoi(argvs[9]);
  
  /////////////////
  //Initializing the ros node
  ros::init(argc,argvs,"echo_pub");
  ros::NodeHandle n;
  ros::Publisher data_pub = n.advertise<std_msgs::UInt8MultiArray>("Echo_Data",queue_size);
  ros::Rate loop_rate(pub_Hz);

  ////////////////////////////////
  //read the sonar data
  int buf_size=0; //the size of the returned buffer size
  unsigned char buf[1024];
  std_msgs::UInt8MultiArray msg;
  
   while(ros::ok())
    {  
      buf_size=ReadSonarReturnData(fd,&buf[0]);
      
      if (buf_size>0)
	{
	  
	  for (int i=0;i<buf_size;i++)
	    msg.data.push_back(buf[i]);
	  
	  for (int i=1;i<10;i++) //put the switch data in the msg
	    msg.data.push_back(atoi(argvs[i]));
	  
	  data_pub.publish(msg);
	  ROS_INFO("Echo data published");
	  
	  
	}
      ros::spinOnce();
      msg.data.clear();
      usleep(10000);//10 hz
      //loop_rate.sleep();
    }
}
