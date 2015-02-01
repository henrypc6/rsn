#include "FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/mutex.hpp>
//#include <boost/atomic.hpp>
#include <queue>  
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <fstream>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#define MODEFORMAT7 MODE_0
#define PIXELFORMAT7 PIXEL_FORMAT_RAW8
#define OFFSETX 328
#define OFFSETY 242
#define WIDTH 640
#define HEIGHT 480

using namespace FlyCapture2;
using namespace cv;
using namespace std;

///////////////////Function Declaration///////////////////////////////////////////////////////
void PrintBuildInfo();
void configureCamera(Camera &camera, PGRGuid* guid);
void PrintCameraInfo( CameraInfo* pCamInfo );
void PrintError( Error error );
Mat  convertImageToCV(Image &image);
void PrintFormat7Capabilities( Format7Info fmt7Info );
void fileWriter();
//////////////////////////////////////////////////////////////////////////////////////////////
typedef struct ImageFile
{
	public:
	 Mat image;
	 string name;
	 ImageFile(Mat im,String n)
	 {
		 image = im;
		 name = n;
	 }
	
}ImageFile;

std::queue<ImageFile> leftQueue;
std::queue<ImageFile> rightQueue;
double latitude;
double longitude;
double altitude;
boost::mutex io_mutex;
bool fileWriterFlag = true;
//////////////////////////////Thread Function////////////////////////////////////////
void fileWriter()
{
	boost::unique_lock<boost::mutex> scoped_lock(io_mutex);
	while(fileWriterFlag)
	{
		while(!leftQueue.empty())
		{
			
			cout<<"sizel:"<<leftQueue.size()<<endl;
			cout<<"Writing left"<<endl;
			ImageFile left = (ImageFile)leftQueue.front();
			imwrite(left.name,left.image);
			
			leftQueue.pop();
			cout<<"Safely written left"<<endl;
			
		}
		
		while(!rightQueue.empty())
		{
			
			cout<<"sizer:"<<rightQueue.size()<<endl;
			cout<<"Writing right"<<endl;
			ImageFile right = (ImageFile)rightQueue.front();
			imwrite(right.name,right.image);
			cout<<"Writing right ended"<<endl;
			rightQueue.pop();
			
			cout<<"Safely written right"<<endl;
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
void PrintFormat7Capabilities( Format7Info fmt7Info )
{
    printf(
        "Max image pixels: (%u, %u)\n"
        "Image Unit size: (%u, %u)\n"
        "Offset Unit size: (%u, %u)\n"
        "Pixel format bitfield: 0x%08x\n",
        fmt7Info.maxWidth,
        fmt7Info.maxHeight,
        fmt7Info.imageHStepSize,
        fmt7Info.imageVStepSize,
        fmt7Info.offsetHStepSize,
        fmt7Info.offsetVStepSize,
        fmt7Info.pixelFormatBitField );
}



Mat  convertImageToCV(Image &rawImage)
{
	Image rgbImage;	
	rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );   
	// convert to OpenCV Mat
	unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();	
	Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);	
	return image;
}

void configureCamera(Camera &camera, PGRGuid* guid)
{
	
	Error error = camera.Connect(guid); 
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		abort();
	}
	CameraInfo camInfo;
	error = camera.GetCameraInfo( &camInfo );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		abort();
	}

	PrintCameraInfo(&camInfo); 	
	
	Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = MODEFORMAT7;
    error = camera.GetFormat7Info( &fmt7Info, &supported );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        abort();
    }

    PrintFormat7Capabilities( fmt7Info );

    if ( (PIXELFORMAT7 & fmt7Info.pixelFormatBitField) == 0 )
    {
        // Pixel format not supported!
		printf("Pixel format is not supported\n");
        abort();
    }
    
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = MODEFORMAT7;
    fmt7ImageSettings.offsetX = OFFSETX;
    fmt7ImageSettings.offsetY = OFFSETY;
    fmt7ImageSettings.width = WIDTH;
    fmt7ImageSettings.height = HEIGHT;
    fmt7ImageSettings.pixelFormat = PIXELFORMAT7;

    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    // Validate the settings to make sure that they are valid
    error = camera.ValidateFormat7Settings(&fmt7ImageSettings,&valid,&fmt7PacketInfo );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        abort();
    }

    if ( !valid )
    {
        // Settings are not valid
		printf("Format7 settings are not valid\n");
        abort();
    }

    // Set the settings to the camera
    error = camera.SetFormat7Configuration(&fmt7ImageSettings,fmt7PacketInfo.recommendedBytesPerPacket );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
       abort();
    }

    // Start capturing images
    error = camera.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        abort();
    }

    // Retrieve frame rate property
    Property frmRate;
    frmRate.type = FRAME_RATE;
    error = camera.GetProperty( &frmRate );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        abort();
    }

    printf( "Frame rate is %3.2f fps\n", frmRate.absValue );

  

	
}



void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf(version,"FlyCapture2 library version: %d.%d.%d.%d\n",fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );
    printf( "%s", version );
    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );
    printf( "%s", timeStamp );
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf
    (
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime 
     );
}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}


void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  cout<<"Latitude:"<<msg->latitude<<endl;
  latitude = msg->latitude;
  longitude = msg->longitude;
  altitude = msg->altitude;
}


int getPixelVal(int cx,int cy, int sqsize, int color,Mat image)
{
  int rowUp =cx+sqsize/2;
  int rowDown = cx -sqsize/2;
  int colUp = cy+sqsize/2;
  int colDown = cy-sqsize/2;
  while(rowUp>=image.rows)
    {
      rowUp--;
    }
  while(colUp>=image.cols)
    {
      colUp--;
    }
  while(rowDown<0)
    {
      rowDown++;
    }
  while(colDown<0)
    {
      colDown++;
    }
  double average = 0;
  int count = 0;
  int i,j;
  for (i = rowDown;i<=rowUp;i++)
    {
      for( j =colDown;j<=colUp; j++)
	{
	  average+= image.at<cv::Vec3b>(i,j)[color];
	  count++;
	}
    }
  
  return round(average/count);
}

Mat getSegmentedImageDarkRed(Mat imag, int colorSquareSize,Mat flagImage,bool useFlag)
{
  Mat image = imag.clone();
  /////////The weird segmentation again///////////////////////////
  for(int i =0;i<image.rows;i++)
    {
      for(int j =0; j<image.cols;j++)
	{
	  int max = -1;
	  int min = 2000;
	  if((useFlag && (flagImage.at<cv::Vec3b>(i,j)[2]!=0 && flagImage.at<cv::Vec3b>(i,j)[1]!=0 && flagImage.at<cv::Vec3b>(i,j)[0]!=0)) ||!useFlag)
	    {
	      
	      int red = /*frameclone.at<cv::Vec3b>(i,j)[2];*/getPixelVal(i,j,colorSquareSize,2,image);
	      int green =/* frameclone.at<cv::Vec3b>(i,j)[1];*/getPixelVal(i,j,colorSquareSize,1,image);
	      int blue = /*frameclone.at<cv::Vec3b>(i,j)[0];*/getPixelVal(i,j,colorSquareSize,0,image);
	      
	      if(green>=red && green >=blue)
		{
		  max = green;
		}
	      else if(blue>=red)
		{
		  max = blue;
		}
	      else
		{
		  max = red;
		}
	      
	      if(red<=blue && blue <=green)
		{
		  min = red;
		}
	      else if(blue<=green)
		{
		  min = blue;
		}
	      else
		{
		  min = green;
		}
	      //frameclone.at<cv::Vec3b>(i,j)[2] = 0;
	      //frameclone.at<cv::Vec3b>(i,j)[1] = 0;
	      //frameclone.at<cv:0:Vec3b>(i,j)[0] = 0;
	      if(max == blue)
		{
		  
		  image.at<cv::Vec3b>(i,j)[2] = 0;
		  image.at<cv::Vec3b>(i,j)[1] = 0;
		  image.at<cv::Vec3b>(i,j)[0] = 0;
		}
	      
	      else if (max == red)
		{
		  if(((red -blue<25|| red -green<25 ) && red>200  ) || red -blue<10||red-green<10)
		    {
		      image.at<cv::Vec3b>(i,j)[2] = 0;
		      image.at<cv::Vec3b>(i,j)[1] = 0;
		      image.at<cv::Vec3b>(i,j)[0] = 0;
		    }
		  
		  else if(green-blue>30  || green+blue>270|| (green-blue<40 && red -green <50 && red>79))
		    {
		      image.at<cv::Vec3b>(i,j)[2] = 0;
		      image.at<cv::Vec3b>(i,j)[1] = 0;
		      image.at<cv::Vec3b>(i,j)[0] = 0;
		    }
		  
		  
		}
	      else if(max == green)
		{
		  //if(green<200)
		  {
		    image.at<cv::Vec3b>(i,j)[2] = 0;
		    image.at<cv::Vec3b>(i,j)[1] = 0;
		    image.at<cv::Vec3b>(i,j)[0] = 0;
		  }
		  
		}
	      else
		{
		  
		  //frameclone.at<cv::Vec3b>(i,j)[0] = max;
		}
	       
	      if (min == red )
		{
		  image.at<cv::Vec3b>(i,j)[2] = 0;
		  image.at<cv::Vec3b>(i,j)[1] = 0;
		  image.at<cv::Vec3b>(i,j)[0] = 0;
		}
	      
	    }
	  
	  else
	    {
	      
	      image.at<cv::Vec3b>(i,j)[2] = 0;
	      image.at<cv::Vec3b>(i,j)[1] = 0;
	      image.at<cv::Vec3b>(i,j)[0] = 0;
	    }
	}
      
    }
  
  return image;
}


//Matching template code
void MatchingMethod(const Mat& img, const Mat& templ, int match_method, Point& matchLoc, Mat& output)
{
  
   /// Source image to display
  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;
  Mat result;
  result.create( result_cols, result_rows, CV_32FC1 );

  
  /// Do the Matching and Normalize
  cv::matchTemplate( img, templ, result, match_method );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  output = result.clone();
  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  //  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }


  return;
}



//Stereo correspondance 
// Need to look at the detail of the matched patch to find way to improve
// and compare with Stereo_BM algorithm and others. 

void stereo_wbc1(const Mat& img1, const Mat& img2, int delta, int max_d, int Match_template, Mat& disparity)
{
int col=img1.cols;
  int row=img1.rows;
  Size size=img1.size();

  int winSize = delta*2+1;

  Mat left(winSize,winSize,CV_16SC2),right(winSize,winSize,CV_16SC2);
  int minVal=0;
  int r_cnt=winSize;
  int c_cnt=1;
  
  
  Mat frameclone1 = img1.clone();
  frameclone1 = getSegmentedImageDarkRed(frameclone1,3,Mat(),false);
  
  frameclone1 = getSegmentedImageDarkRed(frameclone1,12,frameclone1,true);
  
  Mat frameclone2 = img2.clone();
  
  frameclone2 = getSegmentedImageDarkRed(frameclone2,3,Mat(),false);
  
  frameclone2 = getSegmentedImageDarkRed(frameclone2,12,frameclone2,true);

  frameclone1.convertTo(frameclone1, CV_8U);
  frameclone2.convertTo(frameclone2, CV_8U);
  // imshow("apple1", frameclone1);
  // imshow("apple2", frameclone2);
  
  
  // Mat frameclone1=frameclone1.clone();
  // Mat f2=frameclone2.clone();
  
  for (int r=0;r<row-winSize;r+=r_cnt)
    {
      for(int c=0;c<col-winSize;c+=c_cnt)
	{
	  double sum = frameclone2.at<cv::Vec3b>(r,c)[2]+frameclone2.at<cv::Vec3b>(r,c)[1]+frameclone2.at<cv::Vec3b>(r,c)[0];
	

	  if (sum>minVal)
	    {
	      //cout<<sum<<endl;
	      Point p1(c,r);
	      Point p2(min(c+max_d,col),r+winSize);
	      Rect roi_right(c,r,winSize,winSize);
	      Rect roi_left(p1,p2);
	      
	      left=frameclone1(roi_left).clone();
	      right=frameclone2(roi_right).clone();
	      
	      double Max_value=0,disp=0;
	      Point MatchLoc;
	      //cout<<"works"<<endl;
	      //the second matching method
	      Mat MatchRes;
	      MatchingMethod(left,right,Match_template,MatchLoc, MatchRes);
	      disp=MatchLoc.x;
	      
	       Mat tmp(winSize,winSize,CV_32F,disp);
	       tmp.copyTo(disparity(Range(r,r+winSize),Range(c,c+winSize)));
	       //disparity.at<float>(r,c)=100;
	    }
	}
    }
  return;
}


void stereo_wbc2(const Mat& img1, const Mat& img2, int delta, int max_d, int Match_template, Mat& disparity)
{
  int col=img1.cols;
  int row=img1.rows;
  Size size=img1.size();

  int winSize = delta*2+1;

  Mat left(winSize,winSize,CV_16SC2),right(winSize,winSize,CV_16SC2);
  
  int r_cnt=winSize;
  int c_cnt=1;

  int rf1=0,rf2=0,cf1=0,cf2=0;
  int rb1=0,rb2=0,cb1=0,cb2=0;
  bool lock1=false, lock2=false, reverse = false;	    
  int disp=0;
  int minVal=0;
  int r,c;
  for (r=0;r<row-winSize;r+=r_cnt)
    {
      //cout<<r<<endl;
      if (reverse==false)
	{
	  lock1=false;
	  lock2=false;
	      
	  for(c=0;c<col-1;c+=c_cnt)
	    {
	      // Point p1(c,r);
	      // Point p2(c+max_d,r+winSize);
	      // Rect roi_left(c,r,winSize,winSize);
	      // Rect roi_right(p1,p2);
	      
	      // left=img1(roi_left).clone();
	      // right=img2(roi_right).clone();
	  
	      // double Max_value=0,disp=0;
	      // Point MatchLoc;
	  
	      // //the second matching method
	      // Mat MatchRes;
	      // MatchingMethod(right,left,Match_template,MatchLoc, MatchRes);
	      // disp=MatchLoc.x;
	 
	      //the third matching method (delta must be 0)
	      //winSize = 1
	  	  
	      double sum1 = img1.at<cv::Vec3b>(r,c)[2]+img1.at<cv::Vec3b>(r,c)[1]+img1.at<cv::Vec3b>(r,c)[0];
	      double sum2 = img2.at<cv::Vec3b>(r,c)[2]+img2.at<cv::Vec3b>(r,c)[1]+img2.at<cv::Vec3b>(r,c)[0];

	      //cout<<c<<endl;
	      if (sum1>minVal && lock1 == false)
		{
		  lock1 = true;
		  rf1=r;
		  cf1=c;
		}
	      if (sum2>minVal && lock2 == false)
		{
		  lock2 = true;
		  rf2=r;
		  cf2=c;
		}

	      if (lock1 && lock2)
		{
		  reverse = true;
		  continue;
		}
	  
	       //disparity.at<float>(r,c)=MatchLoc.x;
	    }
	}
      if (reverse)
	{
	  lock1=false;
	  lock2=false;
	  for(c=col;c>1;c-=c_cnt)
	    {
	      
	      // Point p1(c,r);
	      // Point p2(c+max_d,r+winSize);
	      // Rect roi_left(c,r,winSize,winSize);
	      // Rect roi_right(p1,p2);
	      
	      // left=img1(roi_left).clone();
	      // right=img2(roi_right).clone();
	  
	      // double Max_value=0,disp=0;
	      // Point MatchLoc;
	  
	      // //the second matching method
	      // Mat MatchRes;
	      // MatchingMethod(right,left,Match_template,MatchLoc, MatchRes);
	      // disp=MatchLoc.x;
	 
	      //the third matching method (delta must be 0)
	      //winSize = 1
	      double sum1 = img1.at<cv::Vec3b>(r,c)[2]+img1.at<cv::Vec3b>(r,c)[1]+img1.at<cv::Vec3b>(r,c)[0];
	      double sum2 = img2.at<cv::Vec3b>(r,c)[2]+img2.at<cv::Vec3b>(r,c)[1]+img2.at<cv::Vec3b>(r,c)[0];
	      if (sum1>minVal && lock1 == false)
		{
		  lock1 = true;
		  rb1=r;
		  cb1=c;
		}
	      if (sum2>minVal && lock2 == false)
		{
		  lock2 = true;
		  rb2=r;
		  cb2=c;
		}

	      if (lock1 == true && lock2 == true)
		{
		  reverse = false;
		  continue;
		}
	  
	    }
	}
      int d1 = cb1-cf1;
      int d2 = cb2-cf2;
      
      cout<<d1<<" "<<d2<<endl;
      if (d1>0 && d2>0)
	disp=(d1+d2)/2;
      else
	disp=0;

       if (disp!=0)
       	{
       	  cout<<"row = "<<r<<endl;
	  cout<<"disp = "<<disp<<endl;
      
       	  for (int i=cf1;i<cb1;i++)
       	    disparity.at<float>(r,i)=disp;
	  
       	}
      
      rf1=0;
      rb1=0;
      cf1=0;
      cb1=0;
      rf2=0;
      cf2=0;
      rb2=0;
      cb2=0;
      reverse=false;
    }
  return;
}



void Stereo_match(Mat& left_img, Mat& right_img, int max_d, int delta, int Match_template)
{
  const char* extri_file = "/home/parallels/ros_ws/sandbox/flycam/src/extrinsics.yml";
  const char* intri_file = "/home/parallels/ros_ws/sandbox/flycam/src/intrinsics.yml";

  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
  int alg = STEREO_SGBM;
  
  int color_mode = alg == STEREO_BM ? 0 : 1;

  Mat img1 = left_img.clone();
  Mat img2 = right_img.clone();
  FileStorage fs(intri_file,FileStorage::READ);

  if (!fs.isOpened())
    {
      printf("Failed to open file%s\n", intri_file);
      exit(1);
    }
  
  Mat M1, D1, M2, D2;
  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  fs.open(extri_file, CV_STORAGE_READ);
  if (!fs.isOpened())
    {
      printf("Failed to open file%s\n", extri_file);
      exit(1);
    }

  Mat R, T, R1, P1, R2, P2;
  fs["R"] >> R;
  fs["T"] >> T;

  Size img_size = img1.size();
  Rect roi1, roi2;
  Mat Q;
  
  //gaussian filter
  // Size FilterSize(7,7);
  // GaussianBlur(img1,img1,FilterSize,1);
  // GaussianBlur(img2,img2,FilterSize,1);
  
  //rectify images
  stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
  
  Mat map11, map12, map21, map22;
  initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
  initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
  
  Mat img1r, img2r;
  remap(img1, img1r, map11, map12, INTER_LINEAR);
  remap(img2, img2r, map21, map22, INTER_LINEAR);
  
  //rectified images
  img1 = img1r;
  img2 = img2r;

  // img1.convertTo(img1,CV_32F);
  // img2.convertTo(img2,CV_32F);

  int col=img1.cols;
  int row=img1.rows;
  Mat disparity(row,col,CV_32F),disp8;
  disparity = Scalar(0);
  int64 t = getTickCount();
  
  stereo_wbc1(img1,img2,delta,max_d, Match_template ,disparity);
  t = getTickCount()-t;
  printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
  
  Mat disp_f, disp_f8,disp_lap,disp_sob,disp_bin,disp_mean8;
  int k_size=21;
  Size filter_size(k_size,k_size);
  vector< vector<Point> > contours; 
  int apples=0;
  Point2f center;
  float radius;

  disparity.convertTo(disp_mean8,CV_8U);
  
  threshold(disp_mean8,disp_bin,0,1,THRESH_BINARY);
  Scalar m = mean(disp_mean8,disp_bin);
    
  boxFilter(disparity,disp_f,-1,filter_size);
  
  Laplacian(disp_f,disp_lap,-1,k_size);

  disp_lap.convertTo(disp_f8,CV_8U);

  findContours(disp_f8,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  
  Scalar color = Scalar(0,255,0,255);
  
  for(int i=0; i<contours.size(); i++)
    {
      
      Mat contourMat = Mat (contours[i]);
      const double per = arcLength( contourMat, true);
      
      if (per>m[0]*5 && per<m[0]*10)
	{
	  minEnclosingCircle(contourMat,center,radius);
	  radius=radius*0.5; // because of the k_size = 21 maded it larger, the radius needs to be decreased to fit
	  circle(img2,center,radius,color);
	  
	  apples++;
	}
    }
  imshow("left",img1);
  imshow("right",img2);
  
  //disparity.convertTo(disp8, CV_8U);
  //imshow("box filted",disp_f);
  //imshow("lap of box filter",disp_lap);
  cout<<"we have "<<apples<< " apples"<<endl;
  //return disparity;
  
}


int main(int argc, char** argv)
{
  //ros::init(argc, argv, "StereoCapturer");
  PrintBuildInfo();
  Error error;
  //ros::NodeHandle fixListener;
  ofstream myWriteFile;
  myWriteFile.open("logCurrentStereo.txt");
  
  BusManager busMgr;
  unsigned int numCameras;
  error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }
  // ros::Subscriber sub = fixListener.subscribe("/mavros/fix", 1000, fixCallback);
  cout<<"Number of cameras detected:"<<numCameras<<endl ;
  if ( numCameras <2 )
    {
      cout<<"Insufficient number of cameras... press Enter to exit."<<endl;
      getchar();
      return -1;
    }
  
  
  
  Camera camera1; 	//Camera 1
  Camera camera2; 	//Camera 2
  PGRGuid guid;
  error = busMgr.GetCameraFromIndex( 0, &guid ); //getting the PGRGuid of the first Camera
  configureCamera(camera1,&guid);
	
	
	
	error = busMgr.GetCameraFromIndex( 1, &guid );		//getting the PGRGuid of the second Camera
	configureCamera(camera2,&guid);
	
	// capture loop
	char key = 0;
	//boost::thread fileWriterThread(fileWriter);
     while(key != 'q')
     	{
		
	  // Get current time from the clock, using microseconds resolution
	  //cout<<"starting here"<<endl;
	  const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
	  string ts("1970-01-01 0:00:00.000");
	  boost::posix_time::ptime myEpoch(boost::posix_time::time_from_string(ts));
	  //	cout<<"I got past this"<<endl;
		boost::posix_time::time_duration diff  = now - myEpoch;
		Image rawImage1;
		
		error =camera1.RetrieveBuffer( &rawImage1 );
		
		if ( error != PGRERROR_OK )
		{
			std::cout << "capture error" << std::endl;
			continue;
		}
		//Mat image1 = convertImageToCV(rawImage1);
		//if(image1.data ) 
		
		Image rgbImage1;	
		rawImage1.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1 );   
		// convert to OpenCV Mat
		unsigned int rowBytes1 = (double)rgbImage1.GetReceivedDataSize()/(double)rgbImage1.GetRows();	
		Mat image1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(),rowBytes1);	
	
		imshow("Camera1_left:", image1);
		//cout<<"I got past this too"<<endl;
		string nameLeft = "left-"+static_cast<ostringstream*>( &(ostringstream() << diff.total_milliseconds()) )->str()+".jpg";
		//cout<<"I got past this"+nameLeft<<endl;
		//imwrite(nameLeft,image1);
		//ImageFile im1(image1,nameLeft);		
		//leftQueue.push(im1);
		//key = cv::waitKey(100);
		Image rawImage2;		
		error =camera2.RetrieveBuffer( &rawImage2 );
		
		if ( error != PGRERROR_OK )
		{
			std::cout << "capture error" << std::endl;
			continue;
		}
		
		
		Image rgbImage2;	
		rawImage2.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2 );   
		// convert to OpenCV Mat
		unsigned int rowBytes2 = (double)rgbImage2.GetReceivedDataSize()/(double)rgbImage2.GetRows();	
		Mat image2 = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes2);	
		
		
			
		//if(image2.data )   
		imshow("Camera2_right:", image2);
		string nameRight = "right-"+static_cast<ostringstream*>( &(ostringstream() << diff.total_milliseconds()) )->str()+".jpg";
		
		// Mat frameclone1 = image1.clone();
		// frameclone1 = getSegmentedImageDarkRed(frameclone1,3,Mat(),false);
		
		// frameclone1 = getSegmentedImageDarkRed(frameclone1,12,frameclone1,true);

		// Mat frameclone2 = image2.clone();

		// frameclone2 = getSegmentedImageDarkRed(frameclone2,3,Mat(),false);
		
		// frameclone2 = getSegmentedImageDarkRed(frameclone2,12,frameclone2,true);
		
		Stereo_match(image1, image2, 100, 1, 1);
		//imshow("Disparity", disparity);
		key = waitKey(100);
		
		
	}

	

	fileWriterFlag = false;
	//fileWriterThread.join();
	myWriteFile.close();
	camera1.StopCapture();
	camera2.StopCapture();
	camera1.Disconnect();
	camera2.Disconnect();
    printf( "Done! Press any key to exit...\n" );
    //getchar();
    waitKey();
	
	
	return 0;
}







