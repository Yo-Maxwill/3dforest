//    This file is part of 3DFOREST  www.3dforest.eu
//
//    3DFOREST is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    3DFOREST is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with 3DFOREST.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////

#include "HoughTransform.h"

HoughTransform::HoughTransform()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_cloud = cloud;
}
HoughTransform::HoughTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_cloud = cloud;
}
HoughTransform::~HoughTransform()
{

}
void HoughTransform::set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_cloud = cloud;
}
void  HoughTransform::compute()
{
  int max_it = 200;
  std::vector<float> acc_x(max_it,0);
  std::vector<int> acc_xc(max_it,0);
  std::vector<float> acc_y(max_it,0);
  std::vector<int> acc_yc(max_it,0);
  std::vector<int> acc_r(max_it,0);
  std::vector<int> acc_rc(max_it,0);
  int max_bod = m_cloud->points.size()-5;

    //#pragma omp parallel for
  for(int xa = 0; xa < max_it; xa++)
  {
      //std::srand(std::time(0));
    int a= (rand() %max_bod)+2;
    int b= (rand() %max_bod)+4;
    int c= (rand() %max_bod);

      // vybrat nahodne tri body
    pcl::PointXYZI p1,p2,p3;
    p1 =m_cloud->points.at(a);
    p2 =m_cloud->points.at(b);
    p3 =m_cloud->points.at(c);


// m1 - bod uprostřed (p1,p2), osa ním prochází
    float f1 = (p2.x - p1.x) / (p1.y - p2.y);
    float m1x = (p1.x + p2.x)/2;
    float m1y = (p1.y + p2.y)/2;
    float g1 = m1y - f1*m1x;

    float f2 = (p3.x - p2.x) / (p2.y - p3.y);
    float m2x = (p2.x + p3.x)/2;
    float m2y = (p2.y + p3.y)/2;
    float g2 = m2y - f2*m2x;

   // ošetření degenerovaných případů
   // - tři body na přímce
    float retx,rety;
    int radius;
    if     (f1 == f2)
      continue;
    else if(p1.y == p2.y)
    {
      retx = m1x;
      rety = f1*retx + g1;
      radius = ceil(sqrt((retx-p1.x)*(retx-p1.x) + (rety-p1.y)*(rety-p1.y))*1000);
    }
    else if(p2.y == p3.y)
    {
      retx = m2x;
      rety = f1*retx + g1;
      radius = ceil(sqrt((retx-p1.x)*(retx-p1.x) + (rety-p1.y)*(rety-p1.y))*1000);
    }
    else
    {
      retx = (g2-g1) / (f1 - f2);
      rety = f1*retx + g1;
      radius = ceil(sqrt((retx-p1.x)*(retx-p1.x) + (rety-p1.y)*(rety-p1.y))*1000);
    }
      //jen ulozit
    acc_x.at(xa) = retx;
    acc_y.at(xa)=rety;
    acc_r.at(xa)=radius;
  }


//ACCUULATOR x

  for(int i =0; i < acc_x.size();i++)
  {
    int m=0;
    float qq = acc_x.at(i);

      //#pragma omp parallel for
    for(int j =0; j < acc_x.size();j++)
    {
      float ww = acc_x.at(j);
      if( std::fabs(ww) - std::fabs(qq) >-0.005 && std::fabs(ww) - std::fabs(qq) < 0.005)
      {
        m++;
      }
    }
    acc_xc.at(i) = m;
  }

  int pos_x=0;
  int nej_x = 0;
  for(int k = 0; k < acc_xc.size(); k++)
  {
    int ma = acc_xc.at(k);
    if( ma > nej_x)
    {
      nej_x = ma;
      pos_x = k;
    }
  }
//ACCUULATOR y

  for(int i =0; i < acc_y.size();i++)
  {//ACCUULATOR x
    for(int e =0; e < acc_x.size();e++)
    {
      int m=0;
      float qq = acc_x.at(i);
      //#pragma omp parallel for
      for(int j =0; j < acc_x.size();j++)
      {
        float ww = acc_x.at(j);
        if( std::fabs(ww) - std::fabs(qq) >-0.0049 && std::fabs(ww) - std::fabs(qq) < 0.0049)
        {
          m++;
        }
      }
      acc_xc.at(e) = m;
    }
    int pos_m=0;
    int nej_m = 0;
    for(int k = 0; k < acc_xc.size(); k++)
    {
      int ma = acc_xc.at(k);
      if( ma > nej_m)
      {
        nej_m = ma;
        pos_m = k;
      }
    }
    int m=0;
    float qq = acc_y.at(i);

      //#pragma omp parallel for
    for(int j =0; j < acc_y.size();j++)
    {
      float ww = acc_y.at(j);

      if( std::fabs(ww) - std::fabs(qq) >-0.0049 && std::fabs(ww) - std::fabs(qq) < 0.0049)
      {
        m++;
      }
    }
    acc_yc.at(i) = m;
  }

  int pos_y=0;
  int nej_y = 0;
  for(int k = 0; k < acc_yc.size(); k++)
  {
    int ma = acc_yc.at(k);
    if( ma > nej_y)
    {
      nej_y = ma;
      pos_y = k;
    }
  }
//ACCUULATOR radius

  for(int i =0; i < acc_r.size();i++)
  {
    int m=0;
    float qq = acc_r.at(i);

      //#pragma omp parallel for
    for(int j =0; j < acc_r.size();j++)
    {
      float ww = acc_r.at(j);
      if( std::fabs(ww) - std::fabs(qq) > -5 && std::fabs(ww) - std::fabs(qq) < 5)
      {
        m++;
      }
    }
    acc_rc.at(i) = m;
  }

  int pos_r=0;
  int nej_r = 0;
  for(int k = 0; k < acc_rc.size(); k++)
  {
    int ma = acc_rc.at(k);
    if( ma > nej_r)
    {
      nej_r = ma;
      pos_r = k;
    }
  }
  float rad_f = acc_r.at(pos_r)/10.0;
  if(rad_f < 0)
      rad_f = -0.5;

  pcl::PointXYZI minp,maxp;
  pcl::getMinMax3D(*m_cloud,minp,maxp);

  float z_coord = (maxp.z + minp.z)/2;
  stred c = {acc_x.at(pos_x),acc_y.at(pos_y),z_coord,1,rad_f};
  m_circle = c;
}

stred HoughTransform::get_circle()
{
  return m_circle;
}
