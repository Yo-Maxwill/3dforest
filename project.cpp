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
#include "project.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
//include INPUT OUTPUT
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <iostream>

//CLOUD
Cloud::Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name)
{
  m_name = name;
  m_Cloud=cloud;
  set_color( QColor( rand(), rand(), rand()));
  m_PointSize=1;
}
Cloud::Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col)
{
  m_name = name;
  m_Cloud=cloud;
  set_color(col);
  m_PointSize=1;
}
Cloud::Cloud()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_name = "";
  m_Cloud=cloud;
  m_PointSize=1;
}
Cloud Cloud::operator=(Cloud &kopie)
{
  Cloud t;
  t.set_name( kopie.get_name());
  t.set_Cloud(kopie.get_Cloud());
  t.set_color( kopie.get_color());
  t.set_Psize(kopie.get_Psize());
  return t;
}
void Cloud::set_color (QColor col)
{
    m_color = col;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud::get_Cloud()
{
  return m_Cloud;
}
QString Cloud::get_name()
{
  return m_name;
}
QColor Cloud::get_color()
{
  return m_color;
}
void Cloud::set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  *m_Cloud = *cloud;
}
void Cloud::set_name(QString name)
{
  m_name = name;
}
void Cloud::set_Psize (int p)
{
  m_PointSize = p;
}
int Cloud::get_Psize()
{
  return m_PointSize;
}

//Tree
Tree::Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col, stred s)
: Cloud(cloud, name, col),
m_dbhCloud(new Cloud())
{
  set_dbh(s);
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
  set_dbhCloud();
}
Tree::Tree (Cloud cloud)
: Cloud(cloud),
m_dbhCloud(new Cloud())
{
  stred c {1,0,0,0,0};
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
  set_dbh(c);
  set_dbhCloud();
}
Tree Tree::operator=(Tree &kopie)
{
  Tree t(kopie);

  t.set_dbh(kopie.get_dbh());
  pcl::getMinMax3D(*get_Cloud(),t.m_minp,t.m_maxp);
  t.set_Psize(kopie.get_Psize());

  return t;
}
void Tree::set_height() //check if tree is connected to terrain!!!
{
  m_height = m_maxp.z - m_pose.z;
}
void Tree::set_dbhCloud()
{

//PAS BODU CCA 10 CM OKOLO 1,3 M
  pcl::PointCloud<pcl::PointXYZI>::Ptr dbhCloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z > (m_pose.z + 1.25) && ith->z < (m_pose.z + 1.35))
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      dbhCloud->points.push_back(bod);
    }
  }
 //voxelize
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (dbhCloud);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*cloud_fil);

  //ulozit mracno
  QString a = QString("%1_dbh").arg(m_name);
  QColor col = QColor(255,0,0);
  Cloud *cl = new Cloud(cloud_fil, a);
  m_dbhCloud = cl;
}
void Tree::set_dbhCloud(Cloud c)
{
  m_dbhCloud->set_Cloud(c.get_Cloud());
}
void Tree::set_dbh()
{
//VOXELIZATION
  if(!m_dbhCloud->get_Cloud()->points.empty())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (m_dbhCloud->get_Cloud());
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_fil);
    //HOUGH TRANSFORM
    std::vector<stred> maxima;
    for(int r=2; r < 55; r++)
    {
      int RR = 0, RRR=0;
      int DD = 0, DDD=0;
      std::vector<stred> stredy;
      for(int j=0; j < cloud_fil->points.size();j++)
      {
        std::vector<stred> st (720);
        std::vector<stred> str;
        pcl::PointXYZI ith = cloud_fil->points.at(j);
        for(int uhel =0; uhel <720; uhel ++)
        {
          float A,B;
          A = ith.x - ((float)r/100) * cos((float)uhel*M_PI/(180*2));
          B = ith.y - ((float)r/100) * sin((float)uhel*M_PI/(180*2));

          float AA = ceilf(A * 1000) / 1000; //zaokrouhleni
          float BB = ceilf(B * 1000) / 1000; //zaokrouhleni

          int s = 0;
          float z = ith.z;
          stred T={AA,BB,z,s,r};
          st.at(uhel) = T; //celkem 720 stredu
        }
        str.push_back(st.at(0));

        for(int i=0; i < st.size(); i++)
        {
          bool used = false;
          stred x = st.at(i);
          #pragma omp parallel for
          for(int k=0; k < str.size(); k++)
          {
            stred y = str.at(k);
            if(x.a- y.a <0.005 && y.a-x.a < 0.005 && x.b -y.b < 0.005 && y.b-x.b < 0.005) // pokud je v rozmezi 1 cm okolo
              used = true;
          }
          if(used == false)
          {
            stredy.push_back(x);
            str.push_back(x);
          }
        }
      }
//ACCUULATOR
      for(int i =0; i< stredy.size();i++)
      {
        int m=0;
        stred q=stredy.at(i);
        #pragma omp parallel for
        for(int j =0; j< stredy.size();j++)
        {
          stred w=stredy.at(j);
      //pokud se shoduje poloha aneni to ten samy bod
          if ((w.a - q.a) < 0.005 &&  (q.a - w.a) < 0.005 && (w.b - q.b) < 0.005 &&  (q.b - w.b) < 0.005)
          {
            #pragma omp atomic
            m++;
          }
        }
        stredy.at(i).i = m;
      }
      sort(stredy.begin(),stredy.end());
      maxima.push_back(stredy.back());
      sort(maxima.begin(),maxima.end());
    }
    m_dbh = maxima.back();
  }
}
void Tree::set_dbhLSR()
{
  if(!m_dbhCloud->get_Cloud()->points.empty())
  {
    //voxels
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (m_dbhCloud->get_Cloud());
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_fil);
    // only one layer
    stred guess = set_dbhLSRALG(cloud_fil);
    m_dbh = set_dbhLSRGEOM(guess,m_dbhCloud->get_Cloud());
  }
  else
  {
    set_dbhCloud();
    set_dbhLSR();
  }
}
stred Tree::set_dbhLSRALG(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  float meanX = 0;
  float meanY = 0;
  float meanZ = 0;
  float n = (float) cloud->points.size();
  float Mz,Mxy,Mxx,Myy,Mxz,Myz,Mzz,Cov_xy,Var_z;
  float A0,A1,A2,A22;
  float Dy,xnew,x,ynew,y;
  float DET,Xcenter,Ycenter;
  Mxx=Myy=Mxy=Mxz=Myz=0.;
    //for each point in dbh_cloud calculate mean coordinate
  for(int m=0; m < n;m++)
  {
    pcl::PointXYZI ith = cloud->points.at(m);
    meanX += ith.x;
    meanY += ith.y;
    meanZ += ith.z;
  }
  meanX /= n;
  meanY /= n;
  meanZ /= n;

  for(int j=0; j < n; j++)
  {
    pcl::PointXYZI ith = cloud->points.at(j);
    float Xi = ith.x - meanX;   //  centered x-coordinates
    float Yi = ith.y - meanY;   //  centered y-coordinates
    float Zi = Xi*Xi + Yi*Yi;
    Mxx += Xi*Xi;
    Myy += Yi*Yi;
    Mxy += Xi*Yi;
    Mxz += Xi*Zi;
    Myz += Yi*Zi;
  }
  Mxx /= n;
  Myy /= n;
  Mxy /= n;
  Mxz /= n;
  Myz /= n;
//    computing the coefficients of the characteristic polynomial
  Mz = Mxx + Myy;
  Cov_xy = Mxx*Myy - Mxy*Mxy;
  Var_z = Mzz - Mz*Mz;
  A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
  A1 = Var_z*Mz + 4*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
  A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
  A22 = A2 + A2;
//    finding the root of the characteristic polynomial
//    using Newton's method starting at x=0
//     (it is guaranteed to converge to the right root)
  x=0;
  y=A0;
	for (int iter=0; iter<10; iter++)  // usually, 4-6 iterations are enough
  {
    Dy = A1 + x*(A22 + 16.*x*x);
    xnew = x - y/Dy;
    if ((xnew == x)||(!std::isfinite(xnew)))
      break;
    ynew = A0 + xnew*(A1 + xnew*(A2 + 4*xnew*xnew));
    if (abs(ynew)>=abs(y))
      break;
    x = xnew;
    y = ynew;
  }
//    computing paramters of the fitting circle
  DET = x*x - x*Mz + Cov_xy;
  Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/(DET*2);
  Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/(DET*2);
  float xx = Xcenter + meanX; //xcoord of center
  float yy = Ycenter + meanY; //ycoord of center
  float Mr=0.; //optimal r computation
  float sum=0.,dx,dy;

  for(int k=0; k < n; k++)
  {
    pcl::PointXYZI ith = cloud->points.at(k);

    dx = ith.x - xx;
    dy = ith.y - yy;
    Mr += sqrt(dx*dx + dy*dy);
  }
  //float r = Mr/n;
  float r= ((Xcenter*Xcenter) + (Ycenter*Ycenter) + Mz - x - x)*1000;
  float rr = ceil(r)/5.0;
  float ii=0;
  stred T={xx,yy,meanZ,ii,rr};
  return T;

}
stred Tree::set_dbhLSRGEOM(stred circ, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  int code,i,iter,inner,IterMAX=99;
  float n = (float) cloud->points.size();
  float factorUp=10.,factorDown=0.04,lambda,ParLimit=1.e+6;
  float dx,dy,ri,u,v;
  float Mu,Mv,Muu,Mvv,Muv,Mr,UUl,VVl,Nl,F1,F2,F3,dX,dY,dR;
  float epsilon=3.e-8;
  float G11,G22,G33,G12,G13,G23,D1,D2,D3;
  float meanX=0;
  float meanY=0;
  float meanZ=0;
  float summ=0.,ddxx,ddyy;
  stredLSR Old,New;
   for(int m=0; m < n;m++)
  {
    pcl::PointXYZI ith = cloud->points.at(m);
    meanX += ith.x;
    meanY += ith.y;
    meanZ += ith.z;
  }
  meanX /= n;
  meanY /= n;
  meanZ /= n;
//       starting with the given initial circle (initial guess)
  New = {circ.a,circ.b,circ.r,0,0,0,0};
//       compute the root-mean-square error
  New.s = Sigma(cloud,New);

//       initializing lambda, iteration counters, and the exit code

    lambda = 0.0001;
    iter = inner = code = 0;

NextIteration:

    Old = New;
    if (++iter > IterMAX)
    {code = 1;  goto enough;}

//       computing moments

    Mu=Mv=Muu=Mvv=Muv=Mr=0.;

    for (i=0; i<n; i++)
    {
      pcl::PointXYZI ith = cloud->points.at(i);

        dx = ith.x - Old.a;
        dy = ith.y - Old.b;
        ri = sqrt(dx*dx + dy*dy);
        u = dx/ri;
        v = dy/ri;
        Mu += u;
        Mv += v;
        Muu += u*u;
        Mvv += v*v;
        Muv += u*v;
        Mr += ri;
    }
    Mu /= n;
    Mv /= n;
    Muu /=n;
    Mvv /=n;
    Muv /=n;
    Mr /= n;

//       computing matrices

    F1 = Old.a + Old.r*Mu - meanX;
    F2 = Old.b + Old.r*Mv - meanY;
    F3 = Old.r - Mr;

    Old.g = New.g = sqrt(F1*F1 + F2*F2 + F3*F3);

try_again:

    UUl = Muu + lambda;
    VVl = Mvv + lambda;
    Nl = 1 + lambda;

//         Cholesly decomposition

    G11 = sqrt(UUl);
    G12 = Muv/G11;
    G13 = Mu/G11;
    G22 = sqrt(VVl - G12*G12);
    G23 = (Mv - G12*G13)/G22;
    G33 = sqrt(Nl - G13*G13 - G23*G23);

    D1 = F1/G11;
    D2 = (F2 - G12*D1)/G22;
    D3 = (F3 - G13*D1 - G23*D2)/G33;

    dR = D3/G33;
    dY = (D2 - G23*dR)/G22;
    dX = (D1 - G12*dY - G13*dR)/G11;

    if ((abs(dR)+abs(dX)+abs(dY))/(1+Old.r) < epsilon) goto enough;

//       updating the parameters

    New.a = Old.a - dX;
    New.b = Old.b - dY;

    if (abs(New.a)>ParLimit || abs(New.b)>ParLimit) {code = 3; goto enough;}

    New.r = Old.r - dR;

    if (New.r <= 0.)
    {
        lambda *= factorUp;
        if (++inner > IterMAX) {code = 2;  goto enough;}
        goto try_again;
    }

//       compute the root-mean-square error

    New.s = Sigma(cloud,New);

//       check if improvement is gained

    if (New.s < Old.s)    //   yes, improvement
    {
        lambda *= factorDown;
        goto NextIteration;
    }
    else                       //   no improvement
    {
      if (++inner > IterMAX)
        {code = 2;  goto enough;}
      lambda *= factorUp;
      goto try_again;
    }
    //       exit
enough:

    Old.i = iter;    // total number of outer iterations (updating the parameters)
    Old.j = inner;   // total number of inner iterations (adjusting lambda)
    stredLSR circlef = Old;
    float r= circlef.r*1000;
    float rr = ceil(r)/10.0;
    stred c = {circlef.a,circlef.b,meanZ,1,rr};
    return c;
}
float Tree::Sigma (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, stredLSR circle)
{
  float sum=0.,dx,dy;
  float n = (float) cloud->points.size();
  for (int i=0; i<n; i++)
  {
    pcl::PointXYZI ith = cloud->points.at(i);
    dx = ith.x - circle.a;
    dy = ith.y - circle.b;
    sum += ((sqrt(dx*dx+dy*dy) - circle.r)*(sqrt(dx*dx+dy*dy) - circle.r));
  }
  return sqrt(sum/n);
}
void Tree::set_dbh(stred s)
{
  m_dbh = s;
}
void Tree::set_position(Cloud teren)
{
  //metrova cast mracna od zeme
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z < (m_minp.z + 0.6) )
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      cloud->points.push_back(bod);
    }
  }
  //VOXELIZATION
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_fil);

  // low points
  pcl::PointCloud<pcl::PointXYZI>::Ptr clow (new pcl::PointCloud<pcl::PointXYZI>);
  for(int i =0; i < cloud_fil->points.size(); i++)
  {
    bool top = false;
    pcl::PointXYZI bod = cloud_fil->points.at(i);
    #pragma omp parallel for
    for(int j = 0; j < cloud_fil->points.size(); j++)
    {
      pcl::PointXYZI por = cloud_fil->points.at(j);

      if( (por.x - bod.x) < 0.01 &&  (bod.x - por.x) < 0.01  &&  (por.y - bod.y) < 0.01 &&  (bod.y - por.y) < 0.01  && bod.z > por.z)
      {
        top =true;
      }
    }
    if (top == false)
      clow->points.push_back(bod);
  }

  //min and max points
  pcl::PointXYZI minp, maxp;
  pcl::getMinMax3D(*clow,minp, maxp);

  m_pose.x = (minp.x+maxp.x)/2;
  m_pose.y = (minp.y+maxp.y)/2;
  m_pose.intensity = 1;

  //nejblizsi bod z terenu

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (teren.get_Cloud());

  std::vector<int> pointId(2);
  std::vector<float> pointSD(2);
  pcl::PointXYZI serchP;
  serchP.x = m_pose.x;
  serchP.y = m_pose.y;
  serchP.z = m_minp.z;
  if ( kdtree.nearestKSearch (serchP, 2, pointId, pointSD) > 0 )
  {
    if( m_minp.z > teren.get_Cloud()->points.at(pointId.front()).z)
    {
      m_pose.z = teren.get_Cloud()->points.at(pointId.front()).z;
    }
    else{m_pose.z = m_minp.z;}
  }
  else
    m_pose.z = m_minp.z;
}
pcl::PointXYZI Tree::get_pose()
{
  return m_pose;
}
stred Tree::get_dbh()
{
  return m_dbh;
}
float Tree::get_height()
{
  float AA = ceilf(m_height * 100) / 100; //zaokrouhleni
  return AA;
}
void Tree::set_lenght()
{
  pcl::PointXYZI pmin,pmax;
  m_lmin.x=9000000;
  m_lmin.y=9000000;
  m_lmin.z=9000000;
  m_lmax.x=-9000000;
  m_lmax.y=-9000000;
  m_lmax.z=-9000000;
  //najdi nejdelsi osu
  if (std::abs(m_maxp.x - m_minp.x) > std::abs(m_maxp.y - m_minp.y) && std::abs(m_maxp.x - m_minp.x) > std::abs(m_maxp.z - m_minp.z))
  {

    for(int j = 0; j< m_Cloud->points.size(); j++)
    {

    // pokud je rozdil bodu a m_maxp mensi nez 1
      if(m_maxp.x - m_Cloud->points.at(j).x < 1  && m_Cloud->points.at(j).x > m_lmax.x)
      {
        m_lmax= m_Cloud->points.at(j);
      }

      if(m_Cloud->points.at(j).x - m_minp.x < 1 && m_Cloud->points.at(j).x < m_lmin.x)
      {
        m_lmin = m_Cloud->points.at(j);
      }
    }
  }
  else if ( (m_maxp.y - m_minp.y) > (m_maxp.x - m_minp.x) && (m_maxp.y - m_minp.y) > (m_maxp.z - m_minp.z))
  {

    for(int j = 0; j< m_Cloud->points.size(); j++)
    {

    // pokud je rozdil bodu a m_maxp mensi nez 10
      if(m_maxp.y - m_Cloud->points.at(j).y < 1  && m_Cloud->points.at(j).y > m_lmax.y)
      {
        m_lmax= m_Cloud->points.at(j);
      }

      if(m_Cloud->points.at(j).y - m_minp.y < 1 && m_Cloud->points.at(j).y < m_lmin.y)
      {
        m_lmin = m_Cloud->points.at(j);
      }
    }
  }
  else
  {// nejdelsi je osa z

    for(int j = 0; j< m_Cloud->points.size(); j++)
    {
    // pokud je rozdil bodu a m_maxp mensi nez 10
      if(m_maxp.z - m_Cloud->points.at(j).z < 1  && m_Cloud->points.at(j).z > m_lmax.z)
      {
        m_lmax= m_Cloud->points.at(j);
      }

      if(m_Cloud->points.at(j).z - m_minp.z < 1 && m_Cloud->points.at(j).z < m_lmin.z)
      {
        m_lmin = m_Cloud->points.at(j);
      }
    }
  }
  //compute lenght
  m_lenght = sqrt((m_lmax.x - m_lmin.x)*(m_lmax.x - m_lmin.x) + (m_lmax.y - m_lmin.y)*(m_lmax.y - m_lmin.y) + (m_lmax.z - m_lmin.z)*(m_lmax.z - m_lmin.z));
}
float Tree::get_lenght()
{
  float AA = ceilf(m_lenght * 100) / 100; //zaokrouhleni
  return AA;
}
median Tree::median_(pcl::PointXYZI input,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (cloud);
  std::vector<int> pointIDv;
  std::vector<float> pointSDv;
  std::vector<float> x_coor;
  std::vector<float> y_coor;
  std::vector<float> z_coor;
  if(kdtree.nearestKSearch(input,5,pointIDv,pointSDv) > 0)
  {
    for(int j =0; j < pointIDv.size();j++)
    {
      pcl::PointXYZI bod;
      bod=cloud->points.at(pointIDv.at(j));
      x_coor.push_back(bod.x);
      y_coor.push_back(bod.y);
      z_coor.push_back(bod.z);
    }
      //init median
    std::sort(x_coor.begin(),x_coor.end());
    std::sort(y_coor.begin(),y_coor.end());
    std::sort(z_coor.begin(),z_coor.end());
    median median;
    median.x=x_coor.at(2);
    median.y=y_coor.at(2);
    median.z=z_coor.at(2);
    median.pi = pointIDv;
    return median;
  }
}
pcl::PointXYZI Tree::get_lpoint(bool low)
{
  if (low ==false)
    {return m_lmin;}
  else
    {return m_lmax;}
}
Cloud Tree::get_dbhCloud()
{
  return *m_dbhCloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Tree::skeleton()
{
  std::vector<median> res;
// pro dany cloud -init
  std::vector<median> med;
  std::vector<median> med_u;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  cloud = get_Cloud();

  for(int i = 0; i< cloud->points.size()-10; i+=10)
  {
    pcl::PointXYZI bod;
    bod = cloud->points.at(i);
    //spocitat mediany
    med.push_back(median_(bod,cloud));
  }

  med_u.resize(med.size());
  //spocitat upravene mediany -wlopinit
  #pragma omp parallel for
  for(int j= 0; j< med.size();j++)
  {
      //med_u.push_back(wlopInit(med.at(j), cloud));
    med_u.at(j) = wlopInit(med.at(j), cloud);
  }
QMessageBox::information(0,("tr"),("iterace begin"));
// tady zacne iterace
  for(int it =0; it < 5; it++)
  {
    std::vector<median> it;
    it = iterate(cloud, med_u);
    med = med_u;
    med_u = it;
  }
// prevod do cloudu
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_res (new pcl::PointCloud<pcl::PointXYZI>);
  for(int j= 0; j< med_u.size();j++)
  {
    pcl::PointXYZI bod;
    bod.x = med_u.at(j).x;
    bod.y = med_u.at(j).y;
    bod.z = med_u.at(j).z;
    bod.intensity = 1;

    cloud_res->points.push_back(bod);
  }
return cloud_res;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Tree::skeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double h)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_res (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (cloud);

  for(int i=0; i <cloud->points.size(); i ++)
  {
    pcl::PointXYZI searchPointV;
    searchPointV=cloud->points.at(i);
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    std::vector<float> x_coor;
    std::vector<float> y_coor;
    std::vector<float> z_coor;
    if(kdtree.radiusSearch(searchPointV,h,pointIDv,pointSDv) > 1)
    {
      for(int j =0; j < pointIDv.size();j++)
      {
        pcl::PointXYZI bod;
        bod=cloud->points.at(pointIDv.at(j));
        x_coor.push_back(bod.x);
        y_coor.push_back(bod.y);
        z_coor.push_back(bod.z);
      }
      //init median
      std::sort(x_coor.begin(),x_coor.end());
      std::sort(y_coor.begin(),y_coor.end());
      std::sort(z_coor.begin(),z_coor.end());
      pcl::PointXYZI median;
      median.x=x_coor.at(x_coor.size()/2);
      median.y=y_coor.at(y_coor.size()/2);
      median.z=z_coor.at(z_coor.size()/2);
      median.intensity=1;
   //pokus
   float f=0;
      for(int j =0; j < pointIDv.size();j++)
      {
        pcl::PointXYZI bod;
        bod=cloud->points.at(pointIDv.at(j));

        float r= std::sqrt((median.x - bod.x)*(median.x - bod.x) + (median.y - bod.y)*(median.y - bod.y) + (median.z - bod.z)*(median.z - bod.z));

        float theta = std::exp(-(r*r)/((h/2)*(h/2)));
        float Rx=
        f+= r*theta + Rx;
      }


//

  //iterate
      for(int a=0;a<100; a++)
      {
        float koefx=0, koefy=0, koefz=0;
        float koefx1=0,koefy1=0,koefz1=0;
        for(int k =0; k < pointIDv.size();k++)
        {
          pcl::PointXYZI bod;
          bod=cloud->points.at(pointIDv.at(k));

          koefx += bod.x/(bod.x-median.x);
          koefx1+= 1/(bod.x-median.x);

          koefy += bod.y/(bod.y-median.y);
          koefy1+= 1/(bod.y-median.y);

          koefz += bod.z/(bod.z-median.z);
          koefz1+= 1/(bod.z-median.z);
        }

        pcl::PointXYZI m;
        m.x = koefx/koefx1;
        m.y = koefy/koefy1;
        m.z = koefz/koefz1;
        if ((median.x - m.x) < 0.001&& (m.x - median.x) < 0.001 && (median.y - m.y)< 0.001 && (m.y - median.y)< 0.001 &&(m.z - median.z < 0.001)&&(median.z - m.z < 0.001))
        {
          median = m;
          QMessageBox::information(0,("i"),("nasel median"));
          break;
        }
        else
        {
          median = m;
        }
      }
    cloud_res->points.push_back(median);
    }
    else
    {
      pcl::PointXYZI m;
      m.x=searchPointV.x;
      m.y=searchPointV.y;
      m.z=searchPointV.z;
      m.intensity=2;
      cloud_res->points.push_back(searchPointV);
    }
  }
  return cloud_res;
}
matrix3x3 Tree::covariance(pcl::PointXYZI input,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  // pro dany bod najdi v cloudu 5 bodu
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (cloud);

    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    std::vector<float> x_coor;
    std::vector<float> y_coor;
    std::vector<float> z_coor;
    float sumx=0,sumy=0,sumz=0;

    if(kdtree.nearestKSearch(input,5,pointIDv,pointSDv) > 0)
    {
      for(int j =0; j < pointIDv.size();j++)
      {
        pcl::PointXYZI bod;
        bod=cloud->points.at(pointIDv.at(j));
        x_coor.push_back(bod.x);
        sumx+=bod.x;
        y_coor.push_back(bod.y);
        sumy+=bod.y;
        z_coor.push_back(bod.z);
        sumz+=bod.z;
      }
// spocitat prumer
      float avx=sumx/pointIDv.size();
      float avy=sumy/pointIDv.size();
      float avz=sumz/pointIDv.size();
      float polex[5];
      float poley[5];
      float polez[5];
      //spocitat  zaklady matice
      for(int j =0; j < pointIDv.size();j++)
      {
        polex[j]=x_coor.at(j)/avx;
        poley[j]=y_coor.at(j)/avx;
        polez[j]=z_coor.at(j)/avx;
      }
  float xx=0;
  float xy=0;
  float xz=0;
  float yy=0;
  float yz=0;
  float zz=0;
  //matice
   for(int k =0; k < 6;k++)
   {
     xx+= (polex[k]*polex[k]);
     xy+= (polex[k]*poley[k]);
     xz+=(polex[k]*polez[k]);
     yy+= (poley[k]*poley[k]);
     yz+=(polez[k]*poley[k]);
     zz+=(polez[k]*polez[k]);
   }
   matrix3x3 out;
   out.a=xx/4;
   out.b=xy/4;
   out.c=xz/4;
   out.d=xy/4;
   out.e=yy/4;
   out.f=yz/4;
   out.g=xz/4;
   out.h=yz/4;
   out.i=zz/4;
   return out;
    }
}
matrix3x3 Tree::jacobi(matrix3x3 in)
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

  //Pt*P
    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;

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


  //Pt*P
    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
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

  //Pt*P
    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
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

  //Pt*P
    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
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

  //Pt*P
    A.a=Pt.a*in.a+Pt.b*in.d+Pt.c*in.g;
    A.b=Pt.a*in.b+Pt.b*in.e+Pt.c*in.h;
    A.c=Pt.a*in.c+Pt.b*in.f+Pt.c*in.i;
    A.d=Pt.d*in.a+Pt.e*in.d+Pt.f*in.g;
    A.e=Pt.d*in.b+Pt.e*in.e+Pt.f*in.h;
    A.f=Pt.d*in.c+Pt.e*in.f+Pt.f*in.i;
    A.g=Pt.g*in.a+Pt.h*in.d+Pt.i*in.g;
    A.h=Pt.g*in.b+Pt.h*in.e+Pt.i*in.h;
    A.i=Pt.g*in.c+Pt.h*in.f+Pt.i*in.i;
  }



//A*in
  out.a=A.a*P.a+A.b*P.d+A.c*P.g;
  out.b=A.a*P.b+A.b*P.e+A.c*P.h;
  out.c=A.a*P.c+A.b*P.f+A.c*P.i;
  out.d=A.d*P.a+A.e*P.d+A.f*P.g;
  out.e=A.d*P.b+A.e*P.e+A.f*P.h;
  out.f=A.d*P.c+A.e*P.f+A.f*P.i;
  out.g=A.g*P.a+A.h*P.d+A.i*P.g;
  out.h=A.g*P.b+A.h*P.e+A.i*P.h;
  out.i=A.g*P.c+A.h*P.f+A.i*P.i;

  return out;
}
median Tree::wlopInit(median med,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud )
{
  float h = 0.02;
  float t_sum = 0;
  float t_x = 0,t_y=0,t_z=0;

  // pro kazdy bod
  for(int i = 0; i< med.pi.size();i++)
  {
    pcl::PointXYZI bod;
    bod = cloud->points.at(med.pi.at(i));
    // spocitat rozdily
    float r_x = std::abs(med.x - bod.x);
    float r_y = std::abs(med.y - bod.y);
    float r_z = std::abs(med.z - bod.z);
    //spocitat norm
    float dist2 = r_x*r_x + r_y*r_y + r_z*r_z;
    float theta = (std::exp(-(dist2)/((h/4)*(h/4))));
    t_sum +=theta;
    t_x += bod.x*theta;
    t_y += bod.y*theta;
    t_z += bod.z*theta;
  }
  median medi_u;
  medi_u.x = t_x/t_sum;
  medi_u.y = t_y/t_sum;
  medi_u.z = t_z/t_sum;
  medi_u.pi = med.pi;

  return medi_u;
}
std::vector<median> Tree::iterate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<median> med)
{
  float mju = 0.35;
  float h =0.05;
  std::vector<median> res;

  // vypocitat celkovu beta funkci
    // pro kazdy median najdi 7 nejblizsich
  for(int k = 0; k < med.size();k++)
  {
    //F2
    // najit 5 nejblizsich
    median v =med.at(k);
    float m_1=1000,m_2=1000,m_3=1000,m_4=1000,m_5=1000;
    int mm_1,mm_2,mm_3,mm_4,mm_5;
    for(int m = 0; m< med.size();m++)
    {
      median q =med.at(m);
      float dist_v =sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
      float dist_q =sqrt(q.x*q.x + q.y*q.y + q.z*q.z);

      if(std::abs(dist_v - dist_q) < m_1 && std::abs(dist_v - dist_q)!= 0)
      {
        m_5 = m_4;
        mm_5=mm_4;
        m_4 = m_3;
        mm_4=mm_3;
        m_3 = m_2;
        mm_3=mm_2;
        m_2 = m_1;
        mm_2=mm_1;
        m_1 = std::abs(dist_v - dist_q);
        mm_1=m;
      }
      else if(std::abs(dist_v - dist_q) < m_2 && std::abs(dist_v - dist_q)!= 0)
      {
        m_5 = m_4;
        mm_5=mm_4;
        m_4 = m_3;
        mm_4=mm_3;
        m_3 = m_2;
        mm_3=mm_2;
        m_2 = std::abs(dist_v - dist_q);
        mm_2=m;

      }
      else if(std::abs(dist_v - dist_q) < m_3 && std::abs(dist_v - dist_q)!= 0)
      {
        m_5 = m_4;
        mm_5=mm_4;
        m_4 = m_3;
        mm_4=mm_3;
        m_3 = std::abs(dist_v - dist_q);
        mm_3=m;
      }
      else if(std::abs(dist_v - dist_q) < m_4 && std::abs(dist_v - dist_q)!= 0)
      {
        m_5 = m_4;
        mm_5=mm_4;
        m_4 = std::abs(dist_v - dist_q);
        mm_4=m;
      }
      else if(std::abs(dist_v - dist_q) < m_5 && std::abs(dist_v - dist_q)!= 0)
      {
        m_5 = std::abs(dist_v - dist_q);
        mm_5=m;
      }
    }

    std::vector<int> nejbl;
    nejbl.push_back(mm_1);
    nejbl.push_back(mm_2);
    nejbl.push_back(mm_3);
    nejbl.push_back(mm_4);
    nejbl.push_back(mm_5);

    float a_sum =0;
    float a_x=0;
    float a_y=0;
    float a_z=0;

    for(int e = 0; e< nejbl.size(); e++)
    {
      median t =med.at(nejbl.at(e));

      float r_x = std::abs(v.x - t.x);
      float r_y = std::abs(v.y - t.y);
      float r_z = std::abs(v.z - t.z);
    //spocitat norm
      float dist2 = r_x*r_x + r_y*r_y + r_z*r_z;
      float theta = (std::exp(-(dist2)/((h/4)*(h/4))));

      float betam= theta/(dist2*dist2);

      a_sum +=betam;
      a_x += r_x*betam;
      a_y += r_y*betam;
      a_z += r_z*betam;
    }

    float F2x = mju * a_x/a_sum;
    float F2y = mju * a_y/a_sum;
    float F2z = mju * a_z/a_sum;


/////////////F1
    float alfa_sum=0;
    float  p_x=0,p_y=0,p_z=0;
    for(int k = 0; k< v.pi.size(); k++)
    {
      pcl::PointXYZI bod;
      bod = cloud->points.at(v.pi.at(k));

      float r_x = std::abs(v.x - bod.x);
      float r_y = std::abs(v.y - bod.x);
      float r_z = std::abs(v.z - bod.x);

      float dist2 = r_x*r_x + r_y*r_y + r_z*r_z;
      //float theta = std::exp(-16*(dist2)/(h*h));

      double len = sqrt(dist2);
      if(len <= 0.001 * h)
        len = h*0.001;

			float theta = exp(-16*(dist2)) / pow(len, 1);


      float alfam= theta; //dist2;

      alfa_sum +=alfam;
      p_x += bod.x*alfam;
      p_y += bod.x*alfam;
      p_z += bod.x*alfam;
      QString aaa = QString ("p_x: %1\np_y: %2\np_z: %3").arg(p_x).arg(p_y).arg(p_z);
      QMessageBox::information(0,("tr"),aaa);
    }
    float F1x = p_x/alfa_sum;
    float F1y = p_y/alfa_sum;
    float F1z = p_z/alfa_sum;


    median med_res;
    med_res.x = F1x + F2x;
    med_res.y = F1y + F2y;
    med_res.z = F1z + F2z;
    med_res.pi = v.pi;

    res.push_back(med_res);


 // QString aa = QString ("r: %1\ntheta: %2\nbeta: %3").arg(F2x).arg(F2y).arg(F2z);
  //QMessageBox::information(0,("tr"),aa);
  }
  return res;
}
float Tree::alfa (float a, float b, float h)
{
  float r=std::abs(a-b);
  float theta = (std::exp(-(r*r)/((h/4)*(h/4))));
  float alfa = theta/r;
  return alfa;
}
float Tree::beta (float a, float b, float h)
{
  float r=std::abs(a-b);
  if(r < 0.001*h)
    r=0.001*h;
  float theta = (std::exp(-(r*r)/((h/4)*(h/4))));

  float beta = theta* std::pow(1/r, 0.35);


  return beta;
}
float Tree::sigma(matrix3x3 in)
{
  matrix3x3 out;
  out = in;
  for(int i =0; i <10; i++)
  {
    matrix3x3 tmp;
    tmp = jacobi(out);
    out=tmp;
  }
  std::vector<float> eigenval;
  eigenval.push_back(out.a);
  eigenval.push_back(out.e);
  eigenval.push_back(out.i);
  std::sort(eigenval.begin(),eigenval.end());

  float sigma = eigenval.at(2)/(eigenval.at(0)+eigenval.at(1)+eigenval.at(2));
  return sigma;

}

//Projekt a jeho metody
Project::Project()
{
m_x=0;
m_y=0; //coordinate system
m_projectName = "default";
m_path = "c:/";
}
Project::~Project()
{

}
Project::Project( QString name)
{
m_x=0;
m_y=0; //coordinate system
m_projectName = name;
}
Project::Project(double x, double y, QString name)
{
m_x=x;
m_y=y; //coordinate system
m_projectName = name;

}
void Project::cleanAll()
{
  m_baseCloud.clear();
  m_terrainCloud.clear();
  m_vegeCloud.clear();
  m_ostCloud.clear();
  m_stromy.clear();
}
QString Project::get_Jmeno_Projektu()
{
return m_projectName;
}
void Project::set_xTransform(double x)
{
  m_x=x;
}
void Project::set_yTransform(double y)
{
  m_y=y;
}
void Project::set_baseCloud(Cloud cloud)
{
  m_baseCloud.push_back(cloud);
}
void Project::set_path(QString path)
{
  m_path = path;
}
void Project::set_TerrainCloud(Cloud cloud)
{
  m_terrainCloud.push_back(cloud);
}
void Project::set_TerrainCloudat(int i, Cloud cloud)
{
  m_terrainCloud.at(i) = cloud;
}
void Project::set_TreeCloudat(int i, Cloud cloud)
{
  Tree t (cloud);
  m_stromy.at(i) = t;
}
void Project::set_treedbh(int i, stred x)
{
  m_stromy.at(i).set_dbh(x);
}
void Project::set_VegeCloud(Cloud cloud)// TODO: nastavit aby neprepisovalo jiz zname, ale spojilo
{
m_vegeCloud.push_back(cloud);

}
 void Project::set_Tree(Cloud cloud)
 {
  m_stromy.push_back(cloud);

  if(m_terrainCloud.size()>0)
  {
    m_stromy.back().set_position(m_terrainCloud.back());
    m_stromy.back().set_height();
  }

  m_stromy.back().set_dbhCloud();

 }
void Project::set_OstCloud(Cloud cloud)
{
  m_ostCloud.push_back(cloud);
}
Cloud Project::get_baseCloud(int i)
{
  return m_baseCloud.at(i);
}
double Project::get_Xtransform()
{
  return m_x;
}
double Project::get_Ytransform()
{
  return m_y;
}
QString Project::get_Path()
{
  return m_path;
}
Cloud Project::get_TerrainCloud(int i)
{
  return m_terrainCloud.at(i);
}
Cloud Project::get_VegeCloud(int i)
{
  return m_vegeCloud.at(i);
}
int Project::get_sizebaseCV()
{
  return m_baseCloud.size();
}
int Project::get_sizeTreeCV()
{
 return m_stromy.size();
}
int Project::get_sizeTerainCV()
{
  return m_terrainCloud.size();
}
int Project::get_sizeostCV()
{
  return m_ostCloud.size();
}
int Project::get_sizevegeCV()
{
  return m_vegeCloud.size();
}
Tree Project::get_TreeCloud(int i)
{
return m_stromy.at(i);
}
Cloud Project::get_ostCloud(int i)
{
return m_ostCloud.at(i);
}
Cloud Project::get_Cloud(QString name)
{
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name()== name)
    return m_baseCloud.at(i);
  }


  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
      return m_terrainCloud.at(i);
      //c = new Cloud( get_TerrainCloud(i));
  }

  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
      return m_vegeCloud.at(i) ;  //c = new Cloud(get_VegeCloud(i));
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
    return m_ostCloud.at(i); //c = new Cloud(get_ostCloud(i));
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
     return m_stromy.at(i); //c = new Cloud(get_TreeCloud(i).get_stromCloud());
  }
 //return *c;
}
void Project::save_newCloud(QString type, QString path)
{
  //save cloud
  QString path1 = save_Cloud(path);
  //otevrit Proj.3df file pro pripsani
  QString projfile = QString("%1/proj.3df").arg(get_Path());
  QFile file (projfile);
  file.open(QIODevice::Append | QIODevice::Text);
  QTextStream out(&file);
  out  << type << " " << path1<<"\n";
  file.close();
}
void Project::save_newCloud(QString type, QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c )
{
  QString path = save_Cloud(name,c);
    //otevrit Proj.3df file pro pripsani
  QString projfile = QString("%1\\proj.3df").arg(get_Path());
  QFile file (projfile);
  file.open(QIODevice::Append | QIODevice::Text);
  QTextStream out(&file);
  out  << type << " " << path<<"\n";
  file.close();
}
QString Project::save_Cloud(QString path)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(path.toUtf8().constData(),*cloud);

  QStringList name = path.split("\\");
  QString file = name.back();
  QStringList ext = file.split(".");

  QString path_out = QString ("%1\\%2.pcd").arg(get_Path()).arg(ext.front());
  pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *cloud);
  return path_out;
}
QString Project::save_Cloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c)
{
  QString path_out = QString ("%1\\%2.pcd").arg(get_Path()).arg(name);
  pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *c);
  return path_out;
}
void Project::delete_Cloud(QString name)
{
  //otevrit soubor proj.3df vymazat radek vyhledany podle zadaneho textu
  QString filepath = QString ("%1/proj.3df").arg(m_path);
  QString fileout = QString ("%1/proj_t.3df").arg(m_path);

  QFile file(filepath);
  QFile fileU(fileout);
  file.open(QIODevice::ReadWrite| QIODevice::Text);
  fileU.open(QIODevice::ReadWrite| QIODevice::Text);
  QTextStream in(&file);
  QTextStream out(&fileU);

  while(!in.atEnd())
  {
    QString line = in.readLine();
    if(!line.contains(name))
      out << line<<"\n";
  }
  //smazat proj a prejmenovat proj_t
  file.close();
  fileU.close();
  QFile::remove(filepath);
  QFile::rename(fileout,filepath);

  remove_file(name);
//  odstranit z vectoru

  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name() == name)
    {
      if (i == 0)
        m_baseCloud.erase(m_baseCloud.begin());
      else
        m_baseCloud.erase(m_baseCloud.begin()+i);
    }

  }

  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
    {
      if (i == 0)
        m_terrainCloud.erase(m_terrainCloud.begin());
      else
        m_terrainCloud.erase(m_terrainCloud.begin()+i);
    }

  }

  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
    {
      if (i == 0)
        m_vegeCloud.erase(m_vegeCloud.begin());
      else
        m_vegeCloud.erase(m_vegeCloud.begin()+i);
    }
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
    if (get_ostCloud(i).get_name() == name)
    {
      if (i == 0)
        m_ostCloud.erase(m_ostCloud.begin());
      else
        m_ostCloud.erase(m_ostCloud.begin()+i);
    }
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      if (i == 0)
        m_stromy.erase(m_stromy.begin());
      else
        m_stromy.erase(m_stromy.begin()+i);
    }
  }

}
void Project::remove_file(QString name)
{
  QMessageBox *msgBox =  new QMessageBox(0);
	msgBox->setText("DELETE");
	QString a = QString("Do you want delete also file from disc??");
	msgBox->setInformativeText(a);
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

	if(msgBox->exec() == QMessageBox::Yes)
  {
    QString filep = QString ("%1/%2").arg(m_path).arg(name);
    QFile::remove(filep);
  }
}
void Project::set_color(QString name, QColor col)
{
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name() == name)
      m_baseCloud.at(i).set_color(col);
  }

  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
       m_terrainCloud.at(i).set_color(col);

  }

  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
       m_vegeCloud.at(i).set_color(col);
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
     m_ostCloud.at(i).set_color(col); //c = new Cloud(get_ostCloud(i));
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
      m_stromy.at(i).set_color(col);
  }
  save_color(name,col);
}
void Project::set_PointSize(QString name, int p)
{
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name() == name)
      m_baseCloud.at(i).set_Psize(p);
  }

  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
       m_terrainCloud.at(i).set_Psize(p);

  }

  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
       m_vegeCloud.at(i).set_Psize(p);
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
     m_ostCloud.at(i).set_Psize(p); //c = new Cloud(get_ostCloud(i));
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
      m_stromy.at(i).set_Psize(p);
  }
}

void Project::save_color(QString name, QColor col)
{
  //open pro file,
  QString filepath = QString ("%1/proj.3df").arg(m_path);
  QString filepathtmp = QString ("%1/proj.tmp").arg(m_path);
  QFile file(filepath);
  QFile tmp (filepathtmp);
  file.open(QIODevice::ReadWrite);
  tmp.open(QIODevice::ReadWrite | QIODevice::Text);
  QTextStream in(&file);
  QTextStream out(&tmp);

  while(!in.atEnd())
  {
    QString line = in.readLine();
    if(line.contains(name))
    {
      QStringList plist = line.split(" ");
      out << plist.at(0)<< " " << plist.at(1)<< " " << col.red()<< " " << col.green()<< " " << col.blue()<< "\n";

    }
    else
    {
      out<<line<<"\n";
    }
  }
  file.close();
  tmp.close();
  //smazat proj.3df a prejmenovat proj.tmp na pro.3df
  QFile::remove(filepath);
  QFile::rename(filepathtmp,filepath);
}
void Project::readAtrs()
{
  QString fileName = QString ("%1/TREE_ATRIBUTES.txt").arg(get_Path());
  QFile file (fileName);
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&file);

  bool first_line = true;
  while(!in.atEnd())
  {
    QString lines = in.readLine();
    QStringList coords = lines.split("\t");
//READ FIRST LINE
    if(first_line == true)
    {
       first_line = false;
    }
//READ REST OF FILE
    else
    {
      for(int i=0; i < m_stromy.size();i++)
      {
        if(coords.at(0) == m_stromy.at(i).get_name())
        {
          float x = (coords.at(6).toDouble()+get_Xtransform());
          float y = (coords.at(7).toDouble()+get_Ytransform());
          float z = (coords.at(5).toFloat()+1.3);
          int j = 0;
          int r = (coords.at(1).toInt()/2);
          stred s = {x,y,z,j,r};
          m_stromy.at(i).set_dbh(s);
          //m_stromy.at(i).set_height();
          //m_stromy.at(i).set_position();
        }
      }
    }
  }
}
void Project::set_Psize(QString name, int p)
{

}
//CLOUD
