#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <vector>
#include "ros/ros.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "plot/gnuplot_i.hpp"

using namespace cv;
using namespace std;

Mat img1,img2;
int Match_template;
Mat templ;
Point p1,p2,p3,p4;
Point mp;
int match_method;
int max_Trackbar = 5;
int max_d;
char* image_window="Source image";
int delta;
//Gnuplot
Gnuplot g("disparity match");

void MatchingMethod( int, void* );
static void mouseCallback(int, int, int, int, void*);

int main(int argc, char** argv)
{
  const char* im1_file = 0;
  const char* im2_file = 0;
  const char* extri_file = 0;
  const char* intri_file = 0;

  im1_file=argv[1];
  im2_file=argv[2];
  extri_file=argv[3];
  intri_file=argv[4];
  
  max_d=atoi(argv[5]);
  delta = atoi(argv[6]);
  match_method=atoi(argv[7]);

  mp=Point(-1,-1);
  
  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
  int alg = STEREO_SGBM;

  int color_mode = alg == STEREO_BM ? 0 : 1;
  img1 = imread(im1_file, color_mode);
  img2 = imread(im2_file, color_mode);

  FileStorage fs(intri_file,FileStorage::READ);

  if (!fs.isOpened())
    {
      printf("Failed to open file%s\n", intri_file);
      return -1;
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
      return -1;
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

  
  
  //create windows
  namedWindow( image_window, CV_WINDOW_AUTOSIZE );  
  // nameWindow(result_window, CV_WINDOW_AUTOSIZE);
  // nameWindow(template_window, CV_WINDOW_AUTOSIZE);

  
  
  /// Create Trackbar
  char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
  createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod);

  MatchingMethod( 0, 0 );

  setMouseCallback(image_window, mouseCallback, 0);

  waitKey(0);

  return 0;
}

static void mouseCallback(int event, int x, int y, int, void*)
{
  if (event != EVENT_LBUTTONDOWN)
    return;
  mp = Point(x,y);
  Mat img_tmp=img1.clone();

  rectangle(img_tmp,Point(mp.x-delta,mp.y-delta),Point(mp.x+delta,mp.y+delta),(0,0,255),2,8,0);
  p1=Point(mp.x-delta,mp.y-delta);
  p2=Point(mp.x+delta,mp.y+delta);
  templ=img1(Rect(p1,p2)).clone();
  
  imshow(image_window,img_tmp);
  
  return;
  
}

void MatchingMethod(int, void*)
{
  /// Do the Matching and Normalize
  
  if (mp.x==-1 && mp.y==-1)
    return;
      
  p4=p1;
  p3.x=p2.x+max_d;
  p3.y=p2.y;
  
  Mat tmp;
  tmp=img2(Rect(p4,p3)).clone();
  
  Mat res;
  matchTemplate(tmp,templ,res,match_method);
  normalize(res,res,0,1,NORM_MINMAX,-1,Mat());
  
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  
  //minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  minMaxLoc( res, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  double peak;
  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    {
      matchLoc = minLoc; 
      peak = minVal;
    }
  else
    { 
      matchLoc = maxLoc; 
      peak = maxVal;
    }

  matchLoc +=mp;
  //plot the results
  vector<float> V;
  V.assign((float*)res.datastart, (float*)res.dataend);
  g.reset_plot();
  g.set_style("lines");
  g.plot_x(V);
  g.remove_tmpfiles();
  
  //output the signal to noise ratio
  Mat mean;
  Mat stdev;
  meanStdDev(res,mean,stdev);
  Mat signal_noise_ratio;
  signal_noise_ratio = abs((peak-mean)/stdev);
  cout<<"peak = "<<peak<<endl;
  cout<<"mean = "<<mean<<endl;
  cout<<"stdev = "<<stdev<<endl;
  cout<<"sig_noi_ratio = "<<signal_noise_ratio<<endl;
  
  
  Mat img_tmp=img1.clone();
  Mat img_tmpr=img2.clone();

  rectangle(img_tmp,p1,p2,(0,0,255),2,8,0);
  rectangle(img_tmpr, Point(matchLoc.x-delta, matchLoc.y-delta-5), Point(matchLoc.x+delta, matchLoc.y+delta+5),(0,0,255),2,8,0);
  rectangle(img_tmpr, p4,p3,(255,0,0),2,8,0);
  imshow("result", img_tmpr);
  imshow(image_window,img_tmp);
  

  
  return;
  
}
