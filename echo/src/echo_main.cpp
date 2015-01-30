#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#include "SerialPort.h"
#include "echo_publish.h"
#include "ros/ros.h"



int main(int argc, char** argvs)
{
  int fd;
  char *port=argvs[10];
  //opening the serial port
  printf("Opening port %s...",port);
  fd = OpenSerialPort(port);

  if (fd==-1)
    {
      printf("\nFail to open the port: %s\n",port);
      return 1;
    }
  printf("Successful\n");

  //setting switch data
  SetSwitches(argvs);
  
  //publishing data
  Data_Publish(argc,argvs,fd);
  
  close(fd);
  return 0;
}
