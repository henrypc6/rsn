#include <math.h>
#include <algorithm>
#include <stdlib.h>
//#include <Eigen/Dense>
#include <vector>
#include "geometry_msgs/Point.h"


//const int col=18,row=30;
//typedef Eigen::Matrix<double,col,row> M;
typedef geometry_msgs::Point P;

class Line_Detect
{
public:
  Line_Detect(double x,double y);
  
  //y=mx+b
  double get_coef();//m
  double get_offset();//b
  double get_theta();
  void clear();

  void ls_method(double,double);
  //  void hough_method(double,double);
  void Constr_Vec(double,double);
private:
  //x and y point 
  double x;
  double y;
  double m,b;
  std::vector<P> ob_vector;
  
  //ls_method var
  double Sxy, Sxx;
  double xt,yt;
  double num;

  //hough_method var
  //M table;
  
};

Line_Detect::Line_Detect(double newx,double newy):x(newx),y(newy),m(0),b(0)
{
  Sxy=0;
  Sxx=0;
  xt=0;
  yt=0;
  num=0;

}

double Line_Detect::get_coef()
{
  return m;
}

double Line_Detect::get_theta()
{
  double theta = atan(m);
  return theta;//-90 to 90
}

double Line_Detect::get_offset()
{
  return b;
}

void Line_Detect::clear()
{
  //x and y point 
   x=0;
   y=0;
   m=0;
   b=0;
   
   ob_vector.clear();
  
   //ls_method var
   Sxy=0;
   Sxx=0;
   xt=0;
   yt=0;
   num=0;

  //hough_method var
  //M table;
  
}

void Line_Detect::Constr_Vec(double newx,double newy)
{
  x=newx;
  y=newy;
  
  P next_p;
  next_p.x=x;
  next_p.y=y;
  //restrict the size of the vector
  if (ob_vector.size()==500)
    ob_vector.clear();

  ob_vector.push_back(next_p);
}

void Line_Detect::ls_method(double newx, double newy)
{
  x=newx;
  y=newy;

  num++;

  xt +=x;
  yt +=y;
  
  double xm=xt/num;
  double ym=yt/num;

  Sxy += (x-xm)*(y-ym);
  Sxx += (x-xm)*(x-xm);

  if (Sxx!=0)
    {
      m=Sxy/Sxx;
      b=ym-m*xm;
    }
}

// void Line_Detect::hough_method(double newx,double newy)
// {
//   x=newx;
//   y=newy;

//   //col=18;//0-180 theta
//   //row=30;//0-30  distance
  
//   for (int i=0;i<col;i++)
//     {
//       int r=x*cos(i*10)+y*sin(i*10);
      
//       if (r<30)
// 	table(i,r) +=1;
//       else
// 	table(i,row) +=1;
      
//     }
  
// }
