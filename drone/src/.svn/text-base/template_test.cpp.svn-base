#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/// Global Variables
Mat img; Mat img2; Mat templ; Mat result;
char* image_window = "Source Image";
char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;
Point p1,p2;
/// Function Headers
void MatchingMethod( int, void* );

/** @function main */
int main( int argc, char** argv )
{
  /// Load image and template
  img = imread( argv[1], 1 );
  img2 = imread( argv[2], 1 );

  p1.x= atoi(argv[3]);
  p1.y= atoi(argv[4]);
  p2.x= atoi(argv[5]);
  p2.y= atoi(argv[6]);

  templ=img2(Rect(p1,p2)).clone();
  
  /// Create windows
  namedWindow( image_window, CV_WINDOW_AUTOSIZE );
  namedWindow( result_window, CV_WINDOW_AUTOSIZE );
  namedWindow( "template image", CV_WINDOW_AUTOSIZE);

  /// Create Trackbar
  char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
  createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );

  MatchingMethod( 0, 0 );

  waitKey(0);
  return 0;
}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
void MatchingMethod( int, void* )
{
  /// Source image to display
  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;
  
  result.create( result_cols, result_rows, CV_32FC1 );

  /// Do the Matching and Normalize
  Point p3;
  Point p4(0,p1.y);
  p3.x=img.cols;
  p3.y=p2.y;
  
  Mat tmp;
  tmp=img(Rect(p4,p3)).clone();

  matchTemplate( img, templ, result, match_method );

  Mat res;
  matchTemplate(tmp, templ, res, match_method);
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  normalize( res, res, 0, 1, NORM_MINMAX, -1, Mat() );
  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  
  //minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  minMaxLoc( res, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  matchLoc.y=matchLoc.y+p4.y;
  cout<<matchLoc.x<<" "<<matchLoc.y<<endl;
  
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), (0,0,255), 2, 8, 0 );
  rectangle( img2, p1,p2, (0,0,255), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ),  (0,0,255), 2, 8, 0 );

  imshow( image_window, img_display );
  imshow( result_window, result );
  imshow("template image",img2);
  return;
}
