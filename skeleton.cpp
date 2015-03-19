//  3DFOREST - tool for processing lidar data from forest environment>
//    Copyright (C) <2014>  Jan Trochta
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "skeleton.h"
#include <pcl/kdtree/kdtree_flann.h>
//BOOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

typedef float Weight;
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::property<boost::vertex_name_t, std::string> NameProperty;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::undirectedS, NameProperty, EdgeWeightProperty > mygraph;
typedef boost::property_map< mygraph, boost::vertex_index_t>::type VertexIndexMap;
typedef boost::graph_traits <mygraph >::edge_descriptor Edge;
typedef boost::graph_traits <mygraph >::vertex_descriptor Vertex;
typedef boost::property_map < mygraph, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map < mygraph, boost::vertex_name_t >::type NameMap;
typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;


Skeleton::Skeleton()
{
  m_radius = 0.5;
  m_mju = 0.35;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_cloud = cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
  m_sample = cloud1;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
  average = cloud2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZI>);
  repulsion = cloud3;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZI>);
  m_skeleton = cloud4;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZI>);
  m_start = cloud5;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud6 (new pcl::PointCloud<pcl::PointXYZI>);
  m_stop = cloud6;
  sample();

}
Skeleton::Skeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float radius)
{
  m_radius = radius;
  m_mju = 0.35;
  m_cloud = cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
  m_sample = cloud1;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
  average = cloud2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZI>);
  repulsion = cloud3;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZI>);
  m_skeleton = cloud4;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZI>);
  m_start = cloud5;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud6 (new pcl::PointCloud<pcl::PointXYZI>);
  m_stop = cloud6;
  sample();
}
void Skeleton::init()
{
  #pragma omp parallel for
	for(int i=0; i <m_sample->points.size(); i++)
  {
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_cloud);
    pcl::PointXYZI x;
    x = m_sample->points.at(i);
    float sum_w =0;
    float res_x =0;
    float res_y =0;
    float res_z =0;
    float h = 1.1;
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    if( kdtree.radiusSearch(x,h,pointIDv,pointSDv) > 0)
    {
      for(int j =0; j <pointIDv.size(); j++)
      {
        pcl::PointXYZI p;
        p= m_cloud->points.at(pointIDv.at(j));
        float diff = sqrt((p.x-x.x)*(p.x-x.x) + (p.y-x.y)*(p.y-x.y) + (p.z-x.z)*(p.z-x.z));
      //spocitat
    //diff = rozdil mezi bodem a samplem

        float w = exp(-1*(diff*diff)/((h/4)*(h/4)));
        float alpha = w / diff;

        res_x += p.x*alpha;
        res_y += p.y*alpha;
        res_z += p.z*alpha;
        sum_w+=alpha;
      }
    pcl::PointXYZI proj;
    proj.x = res_x/sum_w;
    proj.y = res_y/sum_w;
    proj.z = res_z/sum_w;

    average->points.push_back(proj);
    }
  }
}
void Skeleton::averageTerm()
{
	average->points.resize(m_sample->points.size());
	#pragma omp parallel for
	for(int i=0; i < m_sample->points.size(); i++)
  {
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_cloud);
    pcl::PointXYZI x;
    x = m_sample->points.at(i);

    if(x.intensity == 2)
      continue;
    float sum_w =0;
    float res_x =0;
    float res_y =0;
    float res_z =0;
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    if( kdtree.radiusSearch(x,m_radius,pointIDv,pointSDv) > 0)
    {
      for(int j =0; j <pointIDv.size(); j++)
      {
        pcl::PointXYZI p;
        p= m_cloud->points.at(pointIDv.at(j));
        float diff = sqrt((p.x-x.x)*(p.x-x.x) + (p.y-x.y)*(p.y-x.y) + (p.z-x.z)*(p.z-x.z));
      //spocitat
    //diff = rozdil mezi bodem a samplem
       // double len = sqrt(diff * diff);
        if(diff <= 0.0001 * m_radius)
          diff = m_radius*0.0001;

        float w = exp( -1.0 * (diff*diff)/((m_radius/2)*(m_radius/2))) ;
        float alpha = w / diff;


        res_x += p.x*alpha;
        res_y += p.y*alpha;
        res_z += p.z*alpha;
        sum_w+=alpha;
      }

      pcl::PointXYZI proj;
      proj.x = res_x/sum_w;
      proj.y = res_y/sum_w;
      proj.z = res_z/sum_w;
      average->points.at(i) = proj;
    }
    else
    {
      average->points.at(i) = x;
    }
  }
  // ulozit average
}
void Skeleton::repulsionTerm()
{
  repulsion->points.resize(m_sample->points.size());
  #pragma omp parallel for

	for(int i=0; i < m_sample->points.size(); i++)
  {
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_sample);
    pcl::PointXYZI x;
    x = m_sample->points.at(i);
    if(x.intensity == 2)
      continue;
    float sum_w =0;
//    float res_x =0;
//    float res_y =0;
//    float res_z =0;
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    float rep_x =0;
    float rep_y =0;
    float rep_z =0;
    matrix3x3 mat;

    if( kdtree.radiusSearch(x,m_radius,pointIDv,pointSDv) > 1)
    {
      for(int j =0; j <pointIDv.size(); j++)
      {
        pcl::PointXYZI p;
        p = m_sample->points.at(pointIDv.at(j));
        if(p.x == x.x && p.y == x.y && p.z == x.z)
          continue;

        float diff = sqrt((p.x-x.x)*(p.x-x.x) + (p.y-x.y)*(p.y-x.y) + (p.z-x.z)*(p.z-x.z));
        pcl::PointXYZI p_dif;
        p_dif.x = p.x - x.x;
        p_dif.y = p.y - x.y;
        p_dif.z = p.z - x.z;
        if(diff <= 0.0001 )
          diff = 0.0001;
        float w = exp( -1.0 * (diff*diff)/((m_radius/2)*(m_radius/2))) ;
        float beta = w / (diff * diff);

        matrix3x3 covar = cov(p_dif,w);
        mat += covar;

        float xx;
        if(std::fabs(p_dif.x) < 0.0001)
          xx = 0.0001*beta;
        else
          xx = (p_dif.x)*beta;
        rep_x += xx;

        float yy;
        if(std::fabs(p_dif.y) < 0.0001)
          yy = 0.0001*beta;
        else
          yy =(p_dif.y)*beta;
        rep_y += yy;

        float zz;
        if(std::fabs(p_dif.z) < 0.0001)
          zz = 0.0001*beta;
        else
          zz = (p_dif.z)*beta;
        rep_z += zz;

        sum_w += beta;

      }
      float sig = sigma(mat);

      pcl::PointXYZI rep;
      rep.x = rep_x/sum_w;
      rep.y = rep_y/sum_w;
      rep.z = rep_z/sum_w;
      rep.intensity = sig;
      repulsion->points.at(i) = rep;
    }
    else
    {
      pcl::PointXYZI rep;
      rep.x = 0;
      rep.y = 0;
      rep.z = 0;
      rep.intensity = 0.5;
      repulsion->points.at(i) = rep;
    }
  }
}

void Skeleton::sample()
{// z m_cloud vem každy 20. bod
  int r;
  if(m_cloud->points.size() > 1000)
    r =1;
  if(m_cloud->points.size() > 5000)
    r =2;
  if(m_cloud->points.size() > 10000)
    r =5;
  if(m_cloud->points.size() > 50000)
    r = 20;

  for(int i = 0; i < m_cloud->points.size(); i=i+r )
  {
    pcl::PointXYZI bod;
    bod = m_cloud->points.at(i);
    m_sample->points.push_back(bod);
  }
}
void Skeleton::iterate()
{
  //averageTerm();
  //repulsionTerm();

	int error_x = 0;
	//double max_error = 0;

  float min_sigma = std::numeric_limits<float>::max();
  float max_sigma = -1;

  for(int ii = 0; ii < repulsion->points.size(); ii++)
  {
    pcl::PointXYZI v = repulsion->points.at(ii);

    if (v.intensity < min_sigma)
    {
      min_sigma = v.intensity;
    }
    if (v.intensity > max_sigma)
    {
      max_sigma = v.intensity;
    }
  }

  float mu_max = 0.35;
  float mu_min = 0.15;
  float mu_length = std::abs(mu_max - mu_min);
  float sigma_length = std::abs(max_sigma - min_sigma);
  //#pragma omp parallel for
  for(int i = 0; i < average->points.size(); i++)
  {
    pcl::PointXYZI c;
    c = m_sample->points.at(i) ;
    pcl::PointXYZI av;
    av = average->points.at(i);
    pcl::PointXYZI re;
    re = repulsion->points.at(i);
    if(av.x == 88888888)
      continue;
    if(av.intensity == 2)
      continue;

    float mu = (mu_length / sigma_length) * (re.intensity - min_sigma) + mu_min;

    pcl::PointXYZI bod;
    bod.x = av.x + mu*re.x;
    bod.y = av.y + mu*re.y;
    bod.z = av.z + mu*re.z;
    bod.intensity = mu;

    float diff = sqrt((c.x - bod.x)*(c.x - bod.x) + (c.y - bod.y)*(c.y - bod.y) + (c.z - bod.z)*(c.z - bod.z));
    error_x += diff;
    m_sample->points.at(i) = bod;
  }

  movement_error = error_x/average->points.size();

  removeTooClosePoints();
  //branching();

  average->points.clear();
  repulsion->points.clear();
}
void Skeleton::removeTooClosePoints()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for(int i = 0; i < m_sample->points.size(); i++)
  {
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_sample);
    pcl::PointXYZI x;
    x = m_sample->points.at(i);
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    if( kdtree.radiusSearch(x,0.0001,pointIDv,pointSDv) > 0)
    {
      if(pointIDv.at(0) < i)
      { continue; }
      else
      { cloud->points.push_back(x);}
    }
    else
    { cloud->points.push_back(x);}
  }
  m_sample = cloud;
}
void Skeleton::branching()
{
  // z pro kazdy bod v m_sample
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (m_sample);
  for(int i = 0; i < m_sample->points.size(); i++)
  {
    pcl::PointXYZI x;
    x = m_sample->points.at(i);
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    float sum=0;
    // search 2 nearest
    if( kdtree.nearestKSearch(x,3,pointIDv,pointSDv) > 0)
    {
      // spocitat cos(uhlu) mezi dvema nejblizsimi body
      pcl::PointXYZI b;
      b = m_sample->points.at(pointIDv.at(2));
      pcl::PointXYZI c;
      c = m_sample->points.at(pointIDv.at(1));

      float aa = sqrt((b.x-c.x)*(b.x-c.x) + (b.y-c.y)*(b.y-c.y) + (b.z-c.z)*(b.z-c.z) );
      float bb = sqrt(pointSDv.at(1));
      float cc = sqrt(pointSDv.at(2));
      float cos = (aa*aa  - bb*bb  - cc*cc) / (-2 * bb*cc);

      if (cos < -0.8)
      {
        m_sample->points.at(i).intensity = 2;
        m_skeleton->points.push_back (m_sample->points.at(i));
      }
    }
  }
}
float Skeleton::cosAngle(pcl::PointXYZI a, pcl::PointXYZI b, pcl::PointXYZI c)
{
  float diff_a = sqrt((b.x-c.x)*(b.x-c.x) + (b.y-c.y)*(b.y-c.y) + (b.z-c.z)*(b.z-c.z) );
  float diff_b = sqrt((a.x-c.x)*(a.x-c.x) + (a.y-c.y)*(a.y-c.y) + (a.z-c.z)*(a.z-c.z) );
  float diff_c = sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y) + (b.z-a.z)*(b.z-a.z) );

  float cos = (diff_a*diff_a - diff_b*diff_b - diff_c*diff_c)/(2* diff_b * diff_c);
  return cos;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Skeleton::get_sample()
{
  return m_sample;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Skeleton::get_skeleton()
{
  return m_skeleton;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Skeleton::get_average()
{
  return average;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Skeleton::get_start()
{
  return m_start;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Skeleton::get_stop()
{
  return m_stop;
}
float Skeleton::get_movement_error()
{
  return movement_error;
}
matrix3x3 Skeleton::cov(pcl::PointXYZI bod, float w)
{
  float xt = bod.x * w;
  float yt = bod.y * w;
  float zt = bod.z * w;

  matrix3x3 res;
  res.a = xt * bod.x;
  res.b = xt * bod.y;
  res.c = xt * bod.z;
  res.d = yt * bod.x;
  res.e = yt * bod.y;
  res.f = yt * bod.z;
  res.g = zt * bod.x;
  res.h = zt * bod.y;
  res.i = zt * bod.z;

  return res;
}
matrix3x3 Skeleton::jacobi(matrix3x3 in)
{
  float c_k,s,theta;
  matrix3x3 P,Pt,A,out;
  if( std::abs(in.b)> std::abs(in.c) && std::abs(in.b) > std::abs(in.f))
  {
    if((in.e - in.a) == 0)
    {
     theta = M_PI/4;
    }
    else
    {
      theta=0.5* std::atan(2*in.b/(in.a - in.e));
    }
    c_k=std::cos(theta);
    s=std::sin(theta);

    //  c_k   -s   0
    //   s  c_k  0
    //   0   0   1
    P = {c_k,-1*s,0,s,c_k,0,0,0,1};
    Pt= {c_k,s,0,-1*s,c_k,0,0,0,1};

  A = Pt*P;
//    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
//    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
//    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
//    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
//    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
//    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
//    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
//    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
//    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;

  }
  else if(std::abs(in.b) < std::abs(in.c) && std::abs(in.c) > std::fabs(in.f))
  {
    if((in.i - in.a) == 0)
    {
       theta = M_PI/4;
    }
    else
    {
      theta=0.5* std::atan(2*in.c/(in.a - in.i));
    }
    c_k=std::cos(theta);
    s=std::sin(theta);
    //  c_k 0   -s
    //   0  1   0
    //  s  0   c_k
    P = {c_k,0,-1*s,0,1,0,s,0,c_k};
    Pt= {c_k,0,s,0,1,0,-1*s,0,c_k};


  A = Pt*P;
//    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
//    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
//    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
//    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
//    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
//    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
//    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
//    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
//    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
  }
  else if(std::abs(in.b) < std::abs(in.f) && std::abs(in.f) > std::fabs(in.c))
  {
    if((in.i - in.e) == 0)
    {
    float theta = M_PI/4;
    }
    else
    {
      theta=0.5* std::atan(2*in.f/(in.e - in.i ));

    }
    c_k=std::cos(theta);
    s=std::sin(theta);

    //   1   0   0
    //   0  c_k  -s
    //   0  s   c_k
    P = {1,0,0,0,c_k,-1*s,0,s,c_k};
    Pt= {1,0,0,0,c_k,s,0,-1*s,c_k};

  A = Pt*P;
//    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
//    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
//    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
//    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
//    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
//    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
//    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
//    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
//    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
  }

  else if(in.b == in.c || in.b == in.f)
  {
    if((in.e - in.a) == 0)
    {
     theta = M_PI/4;
    }
    else
    {
      theta=0.5* std::atan(2*in.b/(in.a - in.e));
    }
    c_k=std::cos(theta);
    s=std::sin(theta);

    //  c_k   -s   0
    //   s  c_k  0
    //   0   0   1
    P = {c_k,-1*s,0,s,c_k,0,0,0,1};
    Pt= {c_k,s,0,-1*s,c_k,0,0,0,1};

  A = Pt * P;
//    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
//    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
//    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
//    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
//    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
//    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
//    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
//    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
//    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
  }
  else
  {
    if((in.i - in.e) == 0)
    {
    float theta = M_PI/4;
    }
    else
    {
      theta=0.5* std::atan(2*in.f/(in.e - in.i ));

    }
    c_k=std::cos(theta);
    s=std::sin(theta);

    //   1   0   0
    //   0  c_k  -s
    //   0  s   c_k
    P = {1,0,0,0,c_k,-1*s,0,s,c_k};
    Pt= {1,0,0,0,c_k,s,0,-1*s,c_k};

  A = Pt*P;
//    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
//    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
//    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
//    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
//    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
//    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
//    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
//    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
//    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
  }



  out = A*in;
//  out.a=A.a*P.a+A.b*P.d+A.c*P.g;
//  out.b=A.a*P.b+A.b*P.e+A.c*P.h;
//  out.c=A.a*P.c+A.b*P.f+A.c*P.i;
//  out.d=A.d*P.a+A.e*P.d+A.f*P.g;
//  out.e=A.d*P.b+A.e*P.e+A.f*P.h;
//  out.f=A.d*P.c+A.e*P.f+A.f*P.i;
//  out.g=A.g*P.a+A.h*P.d+A.i*P.g;
//  out.h=A.g*P.b+A.h*P.e+A.i*P.h;
//  out.i=A.g*P.c+A.h*P.f+A.i*P.i;

  return out;
}
float Skeleton::sigma( matrix3x3 in)
{
  matrix3x3 cov = in;
  for(int i = 0; i < 4; i++)
  {
    matrix3x3 cova = jacobi(cov);
    cov = cova;
  }
  float nej;

  if(cov.a > cov.e)
  {
    if(cov.a >  cov.i )
      nej = cov.a;
    else
      nej = cov.i;
  }
  else
  {
    if(cov.e >  cov.i )
      nej = cov.e;
    else
      nej = cov.i;
  }

  float s = nej/(cov.a+cov.e+cov.i);

//  QString m = QString("matice:\n %1\t%2\t%3\n %4\t%5\t%6\n %7\t%8\t%9\n sigma: %10").arg(cov.a).arg(cov.b).arg(cov.c).arg(cov.d).arg(cov.e).arg(cov.f).arg(cov.g).arg(cov.h).arg(cov.i).arg(s);
//  QMessageBox::information(0,("t"),m);
  return s;
}
void Skeleton::graph()
{
  graph_min_max();

  mygraph gg;

  std::vector<Vertex> verts;
  std::vector<Edge> edges;
  std::vector<Vertex> vertexNames;

    //add 7 vertices
  for(int i = 0; i < m_skeleton->points.size(); i++)
  {
    std::stringstream s;
    s << i;
    vertexNames.push_back(i);
    boost::add_vertex(s.str(), gg);
  }
    //pro kazdy bod v skeleton
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (m_skeleton);
  int low = -1;
  float z_look = 99999999999999;
  for(int i = 0; i < m_skeleton->points.size(); i++)
  {
    bool i_used =false;
    pcl::PointXYZI x;
    x = m_skeleton->points.at(i);
    if( z_look > x.z)
    {
      low = i;
      z_look = x.z;
    }
    for(int t = 0; t < m_nodes_stem.size(); t++)
    {
      if(m_nodes_stem.at(t) == i)
      i_used = true;
    }
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    int num = 25;
    // search 5 nearest
    if( kdtree.nearestKSearch(x,num,pointIDv,pointSDv) > 0)
    {
      for(int j = 1; j < num; j++)
      {
        bool j_used =false;
        pcl::PointXYZI bod;
        bod = m_skeleton->points.at(pointIDv.at(j));
        float w = sqrt(pointSDv.at(j));

        for(int t = 0; t < m_nodes_stem.size(); t++)
        {
          if(m_nodes_stem.at(t) == pointIDv.at(j))
            j_used = true;
        }
        if(j_used == true && i_used == true)
          w = 0.00000001;

        boost::add_edge(i, pointIDv.at(j), w, gg);
        add_edge (i, pointIDv.at(j), w, gg);
      }
    }
  }

  std::vector < Edge > spanning_tree;
  std::vector<int> nod;
  std::vector<int> nod2;
  kruskal_minimum_spanning_tree (gg, std::back_inserter(spanning_tree));

  for(std::vector<Edge>::iterator it = spanning_tree.begin(); it != spanning_tree.end(); ++it)
  {
    nod.push_back(source(*it,gg));
    nod2.push_back(target(*it,gg));
  }
  for(int t =0; t < nod.size(); t++)
  {
    pcl::PointXYZI bod;
    bod = m_skeleton->points.at(nod.at(t));
    m_start->points.push_back(bod);

    pcl::PointXYZI bod2;
    bod2 = m_skeleton->points.at(nod2.at(t));
    m_stop->points.push_back(bod2);
  }
  m_start->width = m_start->points.size ();
  m_start->is_dense=true;
  m_start->height=1;

  m_stop->width = m_stop->points.size ();
  m_stop->is_dense=true;
  m_stop->height=1;
}
void Skeleton::graph_min_max()
{
  mygraph g;
  std::vector<Vertex> vertexNames;
  int low = -1;
  int heit = -1;
  float z_look_low = 99999999999999;
  float z_look_heit = -99999999999999;
  for(int i = 0; i < m_skeleton->points.size(); i++)
  {
    pcl::PointXYZI x;
    x = m_skeleton->points.at(i);
    std::stringstream s;
    s << i;
    vertexNames.push_back(i);
    boost::add_vertex(s.str(), g);
    if( z_look_low > x.z)
    {
      low = i;
      z_look_low = x.z;
    }
    if( z_look_heit < x.z)
    {
      heit = i;
      z_look_heit = x.z;
    }
  }
  //pro kazdy bod v skeleton
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (m_skeleton);

  for(int i = 0; i < m_skeleton->points.size(); i++)
  {
    pcl::PointXYZI x;
    x = m_skeleton->points.at(i);
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    int num = 25;
    // search num nearest
    if( kdtree.nearestKSearch(x,num,pointIDv,pointSDv) > 0)
    {
      for(int j = 1; j < num; j++)
      {
        pcl::PointXYZI bod;
        bod = m_skeleton->points.at(pointIDv.at(j));
        float w = sqrt(pointSDv.at(j));

        boost::add_edge(i, pointIDv.at(j), w, g);
        add_edge (i, pointIDv.at(j), w, g);
      }
    }
  }

  // Create things for Dijkstra
  std::vector<Vertex> predecessors(boost::num_vertices(g)); // To store parents
  std::vector<Weight> distances(boost::num_vertices(g)); // To store distances

  IndexMap indexMap = boost::get(boost::vertex_index, g);
  PredecessorMap predecessorMap(&predecessors[0], indexMap);
  DistanceMap distanceMap(&distances[0], indexMap);

  // Compute shortest paths from v0 to all vertices, and store the output in predecessors and distances
  // boost::dijkstra_shortest_paths(g, v0, boost::predecessor_map(predecessorMap).distance_map(distanceMap));
  // This is exactly the same as the above line - it is the idea of "named parameters" - you can pass the
  // prdecessor map and the distance map in any order.
  boost::dijkstra_shortest_paths(g,vertexNames.at(low), boost::distance_map(distanceMap).predecessor_map(predecessorMap));

  typedef std::vector<mygraph::edge_descriptor> PathType;

  PathType path;
  Vertex v = vertexNames.at(heit);
  for(Vertex u = predecessorMap[v]; u != v;  v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
  {
    std::pair<mygraph::edge_descriptor, bool> edgePair = boost::edge(u, v, g);
    mygraph::edge_descriptor edge = edgePair.first;
    path.push_back( edge );
  }

  int nod2;
  NameMap nameMap = boost::get(boost::vertex_name, g);
  for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    m_nodes_stem.push_back(source(*pathIterator,g));
    nod2 = target(*pathIterator,g);
  }
  m_nodes_stem.push_back(nod2);
}
void Skeleton::save_skel(QString File)
{

  QFile file (File);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(3);
  for(int i = 0; i < get_start()->points.size(); i++)
  {
    pcl::PointXYZI it;
    it = get_start()->points.at(i);

    double x_start = it.x;
    double y_start = it.y;
    double z_start = it.z;

    pcl::PointXYZI it2;
    it2 = get_stop()->points.at(i);
    double x_stop = it2.x;
    double y_stop = it2.y;
    double z_stop = it2.z ;

    out << x_start << " " << y_start << " " << z_start << " " << x_stop << " " << y_stop << " " << z_stop<< "\n";
  }
  file.close();
}
class MyVisitor : public boost::default_dfs_visitor
{
  public:

    MyVisitor(std::vector<std::string> vNames):vertNames(vNames){}
   void discover_vertex(const Vertex &s, const mygraph &g)
    {
      VertexIndexMap vMap = get(boost::vertex_index,g);

      std::cout << "Discover: " << s << "\n";
      //g[s].component_id = myNewId;
      vv.push_back(s);
      vint.push_back (s);
      return;
    }
  void tree_edge(Edge e, const mygraph& g) const
  {
    //for(std::vector<Edge>::iterator it = e; it != e; ++it)
    {
      //std::cout << source(*it,g) << ", " << target(*it,g) <<  "\n";
      //nod.push_back(source(*it,g));

    }
  }

  std::vector<Vertex> GetVectorvv() const {return vv; }
  std::vector<int> GetVectorvint() const {return vint; }

 private:
  std::vector<Vertex> vv;
  std::vector<int> vint;
  std::vector<std::string> vertNames;

};
