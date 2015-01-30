#include <math.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <algorithm>

class ObstacleDist
{
public:
  //Init the distance(dist) vector
  ObstacleDist();
  ObstacleDist(std::vector<double> dist,int range);
  
  void set_dist(std::vector<double> new_dist);
  std::vector<double> get_dist();

  void SetFilter(double Min_Detect_Dist,double Min_Intensity);
  void Prev_dist(double cur_dist);
  
  double Max_Method();
  double Avr_Max();

private:
  std::vector<double> dist;
  
  int range;
  std::vector<double> prev_d;
  
};

ObstacleDist::ObstacleDist()
{}

ObstacleDist::ObstacleDist(std::vector<double> new_dist,int new_range):dist(new_dist),range(new_range)
{}

void ObstacleDist::set_dist(std::vector<double> new_dist)
{
  dist=new_dist;
}

std::vector<double> ObstacleDist::get_dist()
{
  return dist;
}

void ObstacleDist::Prev_dist(double cur_dist)
{
  //restrict the size of the vector
  if (prev_d.size()==500)
    prev_d.clear();
  
  if (!dist.empty())
    prev_d.push_back(cur_dist);
  

}

double ObstacleDist::Max_Method()
{
  unsigned int i=0;
  unsigned int size=dist.size();
  int max_num=0;
  double max_dist=0,tmp_dist=0;

  for (;i<size;i++)
    {
      if (dist[i]>max_num)
	{
	  max_num=dist[i];
	  tmp_dist=i;
	}
    }

  if (size!=0)
    max_dist=tmp_dist*range/size;
  else 
    max_dist=-1;
  return max_dist;
}

double ObstacleDist::Avr_Max()
{
  double avr_dist=0;
  for (unsigned int i=0;i<dist.size();i++)
    {
      if (dist[i]>126)
	{
	  avr_dist +=i;
	}
    }

  return avr_dist*range/dist.size();
  
}

void ObstacleDist::SetFilter(double Min_Detect_Dist, double Min_Intensity)
{
  int t=Min_Detect_Dist*dist.size()/range;
  for (int i=0;i<t;i++)
    dist[i]=0;

  for (int i=t;i<dist.size();i++)
    {
      if (dist[i]<Min_Intensity)
	dist[i]=0;
    }
}
