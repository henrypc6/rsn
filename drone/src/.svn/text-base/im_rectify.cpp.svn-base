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

using namespace cv;
using namespace std;

Mat img1b,img1g,img1R,img2b,img2g,img2R;
Mat img1,img2;
int Match_template;

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
  matchTemplate( img, templ, result, match_method );
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

  /// Show me what you got
  // rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  // rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  // imshow( image_window, img_display );
  // imshow( result_window, result );

  return;
}

//Stereo correspondance 
// Need to look at the detail of the matched patch to find way to improve
// and compare with Stereo_BM algorithm and others. 
void stereo_wbc(const Mat& img1, const Mat& img2, int delta, int max_d, Mat& disparity)
{
  int col=img1.cols;
  int row=img1.rows;
  Size size=img1.size();

  int winSize = delta*2+1;

  Mat left(winSize,winSize,CV_16SC2),right(winSize,winSize,CV_16SC2);
 
  for (int r=0;r<row-winSize;r+=winSize)
    {
      for(int c=0;c<col-winSize-max_d;c++)
	{
	  Point p1(c,r);
	  Point p2(c+max_d,r+winSize);
	  Rect roi_left(c,r,winSize,winSize);
	  Rect roi_right(p1,p2);
	  
	  left=img1(roi_left).clone();
	  right=img2(roi_right).clone();
	  
	  double Max_value=0,disp=0;
	  Point MatchLoc;
	  
	  //the second matching method
	  Mat MatchRes;
	  MatchingMethod(right,left,Match_template,MatchLoc, MatchRes);
	  disp=MatchLoc.x;
	  
	  Mat tmp(winSize,winSize,CV_32F,disp);
	  tmp.copyTo(disparity(Range(r,r+winSize),Range(c,c+winSize)));
	  //disparity.at<float>(r,c)=MatchLoc.x;
	}

    }
    
  return;
}



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
  
  int max_d=atoi(argv[5]);
  int delta = atoi(argv[6]);
  Match_template=atoi(argv[7]);
  
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
  Size FilterSize(7,7);
  GaussianBlur(img1,img1,FilterSize,1);
  GaussianBlur(img2,img2,FilterSize,1);
  
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

  img1.convertTo(img1,CV_32F);
  img2.convertTo(img2,CV_32F);

  int col=img1.cols;
  int row=img1.rows;
  Mat disparity(row,col,CV_32F),disp8;
  int64 t = getTickCount();
  
  stereo_wbc(img1,img2,delta,max_d,disparity);
  t = getTickCount()-t;
  printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
  disparity.convertTo(disp8, CV_8U);

  
  //rectangle(img1r,p1,p2,Scalar::all(0));
  namedWindow("left", 1);
  imshow("left", img1r);
  namedWindow("right", 1);
  imshow("right", img2r);
  namedWindow("disparity", 1);
  imshow("disparity", disp8);

  // string filename = argv[1];
  // filename.append(argv[5]);
  // filename.append("_");
  // filename.append(argv[6]);
  // filename.append("_");
  // filename.append(argv[7]);
  // filename.append(".png");
  // cout<<filename<<endl;
  // imwrite(filename,disp8);
  // imwrite("left_rectify.png",img1r);
  // imwrite("right_rectify.png",img2r);
  printf("press any key to continue...");
  fflush(stdout);
  waitKey();
  printf("\n");

  return 0;
}
