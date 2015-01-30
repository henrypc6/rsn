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

unsigned char ser_sw[27];

int head_id = 0x11;
int ma_sl = 0x43;

int range = 10;
int gain = 20;
int absorption=20;
int pulse_length=100;
int ProfileMinRange=10;//profile min range 0-25 with 0.1 increament 
int datapoints=1; //0 for 252, 1 for 500
int switchdelay=0;


int OpenSerialPort(char* port)
{
  int fd;
  struct termios opt;
  
  fd = open(port,O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd==-1)
    return -1;
  
  fcntl(fd,F_SETFL,0);
  
  tcgetattr(fd,&opt);
  
  //set baud rate to 115200 
  cfsetispeed(&opt,B115200);
  cfsetospeed(&opt,B115200);
  
  //The c_cflag member contains two options that should always be enabled, 
  //CLOCAL and CREAD. These will ensure that your program does not become
  // the 'owner' of the port subject to sporatic job control and hangup 
  //signals, and also that the serial interface driver will read incoming
  // data bytes.
  opt.c_cflag |= (CLOCAL | CREAD);

  opt.c_iflag |= IGNBRK;

  //Don't chang any carriage returns or line feeds
  opt.c_iflag &= ~(ICRNL | IGNCR);
  opt.c_oflag &= ~(OPOST | ONLCR);
  opt.c_lflag &= ~(IEXTEN);
  
  //1 stop bit
  opt.c_cflag &= ~CSTOPB;

  //No parity
  opt.c_cflag &= ~PARENB;

  //8 data bits
  opt.c_cflag &= ~CSIZE;
  opt.c_cflag |= CS8;

  //setCTSRTS off
  opt.c_cflag &= ~CRTSCTS;

  //set XONXOFF off
  opt.c_iflag &= ~(IXON | IXOFF | IXANY);

  //set canonical off
  opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHOK | ECHOKE | ECHOCTL);

  //TCSANOW make changes now without waiting for data to complete
  tcsetattr(fd, TCSANOW, &opt);

  return fd;
}

void SendSwitchData(int fd)
{
  int bytes_write=write(fd,&ser_sw[0],27);
   // if (bytes_write==27)
   //  printf("Switch data write to port successfully\n");
}

void SetSwitches(char** argvs)
{
  range=atoi(argvs[1]);
  gain=atoi(argvs[2]);
  absorption=atoi(argvs[3]);
  pulse_length=atoi(argvs[4]);
  ProfileMinRange=atoi(argvs[5]);
  datapoints=atoi(argvs[6]);
  switchdelay=atoi(argvs[7]);
  
  ser_sw[0]  = 0xFE;//Switch Data Hear (1st Byte)
  ser_sw[1]  = 0x44;//Switch Data Header (2nd Byte)
  ser_sw[2]  = head_id;//Head ID
  ser_sw[3]  = range;//Range: 5,10,20,30,40,50 in meters
  ser_sw[4]  = 0;
  ser_sw[5]  = 0;
  ser_sw[6]  = ma_sl;//Master/Slave: Slave mode only (0x43)
  ser_sw[7]  = 0;
  ser_sw[8]  = gain;//Start Gain: 0 to 40dB in 1 dB increments
  ser_sw[9]  = 0;
  ser_sw[10] = absorption;//Absorption: 20 = 0.2dB/m for 675kHz in water
  ser_sw[11] = 0;
  ser_sw[12] = 0;
  ser_sw[13] = 0;
  ser_sw[14] = pulse_length;//Pulse Length: 100 microseconds
  ser_sw[15] = ProfileMinRange;
  ser_sw[16] = 0;
  ser_sw[17] = 0;
  ser_sw[18] = 0;//External Trigger Control

  if(datapoints==0) {
    ser_sw[19] = 25;//Data Points: 25=250 points 'IMX'
  }
  else {
    ser_sw[19] = 50;//Data Points: 50=500 points 'IGX'
  }

  ser_sw[20] = 0;
  ser_sw[21] = 0;
  ser_sw[22] = 0;//Profile: 0=OFF, 1=IPX output
  ser_sw[23] = 0;
  ser_sw[24] = switchdelay;//Switch Delay: (delay in milliseconds)/2
  ser_sw[25] = 0;//Frequency: 0=675kHz
  ser_sw[26] = 0xFD;//Termination Byte - always 0xFD
}

int ReadSonarReturnData(int fd, unsigned char* ret_buf)
{
  int bufsize=1024;
  unsigned char buf[bufsize];
  int bytes_read=0;
  int hb,lo;
  //interval of sending the switch data(sec)
  
  bzero(buf,bufsize);
  bzero(ret_buf,1024);
  SendSwitchData(fd);      
  printf("Reading from the sonar...\n");
  bytes_read = read(fd,buf,bufsize);
  //printf("data size is %d\n",bytes_read);

  if ( ((bytes_read==265) || (bytes_read==513)) && (buf[bytes_read-1]==0xFC) )
    {
      if (Data_Check(buf)==0)
	{
	  for (int i=12;i<bytes_read;i++)
	    {
	      ret_buf[i-12]=buf[i];
	    }
	  printf("Successful\n");
	}
    }
  else
    {
      printf("The size and/or the termination byte of the data is wrong\n");

      bytes_read=-1;
    }
  //  printf("last byte is 0x%02X \n",ret_buf[500]);
  return bytes_read-12;
}

int Data_Check(unsigned char* buf)
{
  int data_good=1; //checking if the data received is good
  //checking the data
  //checking header
  char header[4]={buf[0],buf[1],buf[2],'\0'};
  int data_bytes;
  if (strcmp(&header[0],"IGX")) //IGX
    {
      data_good=0;
      data_bytes=500;
    }
  else if(strcmp(&header[0],"IMX")) //IMX
    {
      data_good=0;
      data_bytes=252;
    }
  else
    {
      data_good=1;//wrong returned data
      data_bytes=0;
    }
  //checking Head ID
  if (buf[3]!=0x11)
    data_good=1;//head ID is always 0x11

  //checking Serial Status
  if (buf[4]!=0xC1)
    data_good=1;//serial status should be 11000001 -> 0xC1

  //Range should be from 5 to 50
  int range =buf[7];
  if (range<5 || range>50)
    data_good=1;

  //Profile range
  int PR_LO,PR_HI,PR;
  PR_HI=(buf[9] & 0x7E)>>1;
  PR_LO=(((buf[9] & 0x01)<<7) | (buf[8] & 0x7F));
  PR=(PR_HI<<8)|PR_LO;

  if (PR<0)
    data_good=1;//the profile range should be none zero
  
  //printf("Profile range is %d\n",PR);
  //Data bytes
  int DB_LO,DB_HI,DB;
  DB_HI=(buf[11] & 0x7E)>>1;
  DB_LO=(((buf[11] & 0x01)<<7) | (buf[10] & 0x7F));
  DB=(DB_HI<<8)|DB_LO;
  
  if (DB!=data_bytes)
    data_good=1;//the DB and data_bytes should be the same

  //check echo data
  
  if (data_good)
    return 0;
  else
    return 1;
}
