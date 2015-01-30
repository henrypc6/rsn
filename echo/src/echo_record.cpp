
FILE *fd;

int main(int argc, char** argvs)
{
  fd=fopen(argvs[1],"a+");
  ros::init(argc,argvs,"echo_record");
  ros::NodeHanle n;
  
  ros::Subcriber echo=n.subscribe("Echo_Data",10,Echo_Callback);
  ros::Subcriber pos=n.subscribe("robot/state",10,Pos_Callback);
  

  
  return 0;
}
