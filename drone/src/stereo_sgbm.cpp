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

Mat img1,img2;
Size img_size;
void Stereo_SGBM(int, void*);

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

  img_size = img1.size();
  Rect roi1, roi2;
  Mat Q;
  

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

  int col=img1.cols;
  int row=img1.rows;

  
  
  Stereo_SGBM(0,0);
  
  return 0;
}

void Stereo_SGBM(int, void*)
{
  StereoSGBM sgbm;
  StereoBM bm;

  Mat disp, disp8;
  int numberOfDisparities = 0;
  int SADWindowSize = 11;
  
  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
  int alg = STEREO_HH;

  int color_mode = alg == STEREO_BM ? 0 : 1;

  numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
  
  sgbm.preFilterCap = 127;
  sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
  
  int cn = img1.channels();
  
  sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.minDisparity = 0;
  sgbm.numberOfDisparities = numberOfDisparities;
  sgbm.uniquenessRatio = 10;
  sgbm.speckleWindowSize = 100;
  sgbm.speckleRange = 32;
  sgbm.disp12MaxDiff = 4;
  sgbm.fullDP = true;
  
  
  int64 t = getTickCount();
  sgbm(img1,img2,disp);  

  t = getTickCount()-t;
  printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
  
  disp.convertTo(disp8, CV_8U);
  
  //  namedWindow("left", 1);
  imshow("left", img1);
  //namedWindow("right", 1);
  imshow("right", img2);
  //namedWindow("disparity", 1);
  imshow("disparity", disp8);
  waitKey();
  return;
}
