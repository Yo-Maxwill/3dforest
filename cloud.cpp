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

#include "cloud.h"
#include "hull.h"
#include "HoughTransform.h"
#include "LeastSquareregression.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//CLOUD
Cloud::Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name)
{
  m_name = name;
  m_Cloud=cloud;
  set_color( QColor( rand()%255, rand()%255, rand()%255));
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
Cloud::~Cloud()
{

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
Tree::Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col)
: Cloud(cloud, name, col)
{
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
  QString a = QString("%1_dbh").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
  m_dbhCloud = new Cloud(cloud_, a);

  QString aa = QString("%1_convex").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
  m_convexhull = new Cloud(cloud_2, aa);
  m_areaconvex = 0;

  QString aaa = QString("%1_concave").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZI>);
  m_concavehull = new Cloud(cloud_3, aaa);
  m_areaconcave = 0;

  QString aaaa = QString("%1_skeleton").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZI>);
  m_skeleton = new Cloud(cloud_3, aaaa);

  set_position();
  set_height();
  set_dbhCloud();
  set_dbhHT();
  set_dbhLSR();
  set_length();

}
Tree::Tree (Cloud cloud)
: Cloud(cloud)
{
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
  QString a = QString("%1_dbh").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
  m_dbhCloud = new Cloud(cloud_, a);


   QString aa = QString("%1_convex").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
  m_convexhull = new Cloud(cloud_2, aa);
  m_areaconvex = 0;

  QString aaa = QString("%1_concave").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZI>);
  m_concavehull = new Cloud(cloud_3, aaa);
  m_areaconcave = 0;

  QString aaaa = QString("%1_skeleton").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZI>);
  m_skeleton = new Cloud(cloud_3, aaaa);

  set_position();
  set_height();
  set_dbhCloud();
  m_dbh_HT = {1,-1,-1,-1,-0.5};
  set_dbhHT();
  set_dbhLSR();
  set_length();
}
Tree Tree::operator=(Tree &kopie)
{
  Tree t(kopie);


  pcl::getMinMax3D(*get_Cloud(),t.m_minp,t.m_maxp);
  t.set_Psize(kopie.get_Psize());
  set_position();
  set_height();
  set_dbhCloud();
  set_length();
  return t;
}
void Tree::set_height() //check if tree is connected to terrain!!!
{
  m_height = m_maxp.z - m_pose.z;
}
void Tree::set_dbhCloud()
{
 // m_dbhCloud = new Cloud();
//points 10 CM around 1.3 m
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
  if(dbhCloud->points.size() > 20)
  {
    //voxelize
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (dbhCloud);
    sor.setLeafSize (0.001f, 0.001f, 0.001f);
    sor.filter (*cloud_fil);

  //save cloud
    QString a = QString("%1_dbh").arg(m_name);
    QColor col = QColor(255,0,0);
    m_dbhCloud->set_Cloud(cloud_fil);
    cloud_fil.reset();
  }
  else
  {
    //save cloud
    QString a = QString("%1_dbh").arg(m_name);
    QColor col = QColor(255,0,0);
    m_dbhCloud->set_Cloud(dbhCloud);
  }
  dbhCloud.reset();
}
void Tree::set_dbhCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_dbhCloud->set_Cloud(cloud);
}
void Tree::set_dbhHT()
{
  if(m_dbhCloud->get_Cloud()->points.size() > 5)
  {
    HoughTransform ht = m_dbhCloud->get_Cloud();
    ht.compute();
    m_dbh_HT = ht.get_circle();
  }
  else
  {
    m_dbh_HT = {1,-1,-1,-1,-0.5};
  }
}
stred Tree::get_dbhHT()
{
  return m_dbh_HT;
}
stred Tree::get_dbhLSR()
{
  return m_dbh_LSR;
}
void Tree::set_dbhLSR()
{
  if(m_dbhCloud->get_Cloud()->points.size() > 5)
  {
    LeastSquaredRegression lsr;
    lsr.set_Cloud(m_dbhCloud->get_Cloud());
    lsr.compute();
    m_dbh_LSR = lsr.get_circle();
  }
  else
  {
    m_dbh_LSR = {1,-1,-1,-1,-0.5};
  }
}

void Tree::set_position()
{
  std::vector<float> x_coor;
  std::vector<float> y_coor;
  std::vector<float> z_coor;
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z < (m_minp.z + 0.6) )
    {
      x_coor.push_back(ith->x);
      y_coor.push_back(ith->y);
      z_coor.push_back(ith->z);
    }
  }

  if( x_coor.size() > 1)
  {
    std::sort(x_coor.begin(),x_coor.end());
    std::sort(y_coor.begin(),y_coor.end());
    std::sort(z_coor.begin(),z_coor.end());

    m_pose.x = x_coor.at(x_coor.size()/2);
    m_pose.y = y_coor.at(y_coor.size()/2);
    m_pose.z = z_coor.at(0);
    m_pose.intensity = 1;
  }
  else
  {
    m_pose = m_minp;
  }
}
void Tree::set_position(Cloud terrain)
{
  int point_number = 5;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (terrain.get_Cloud());

  std::vector<int> pointId(point_number);
  std::vector<float> pointSD(point_number);

  if (kdtree.nearestKSearch (m_pose, point_number, pointId, pointSD) > 0 )
  {
    float med_Z = 0;
    for(int i = 0; i < point_number; i++)
    {
      med_Z += terrain.get_Cloud()->points.at(pointId.at(i)).z;
    }

    m_pose.z = med_Z / point_number;
  }

}
pcl::PointXYZI Tree::get_pose()
{
  return m_pose;
}

float Tree::get_height()
{
  float AA = ceilf(m_height * 100) / 100; //zaokrouhleni
  return AA;
}
void Tree::set_length()
{
  pcl::PointXYZI pmin,pmax;
  m_lmin.x=9000000;
  m_lmin.y=9000000;
  m_lmin.z=9000000;
  m_lmax.x=-9000000;
  m_lmax.y=-9000000;
  m_lmax.z=-9000000;
  //najdi nejdelsi osu
  //X axis
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
  // Y axis
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
  else //Z axis
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
float Tree::get_length()
{
  float AA = ceilf(m_lenght * 100) / 100; //zaokrouhleni
  return AA;
}
pcl::PointXYZI Tree::get_lpoint(bool low)
{
  if (low ==false)
    {return m_lmin;}
  else
    {return m_lmax;}
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Tree::get_dbhCloud()
{
  return m_dbhCloud->get_Cloud();
}
void Tree::set_convexhull()
{
  QString aa = QString("%1_convex").arg(m_name);
  QColor col = get_color();

  Hull *c = new Hull(m_Cloud,aa,col);
  c->set_convexhull();
  set_convexhull(c->get_convexhull());
 // c->set_areavex(c->get_convexhull());

  m_areaconvex = c->get_areaconvex();
  delete c;
}
void Tree::set_convexhull(Cloud c)
{
  m_convexhull->set_Cloud(c.get_Cloud());
 // set_areavex( c);
}
Cloud Tree::get_vexhull()
{
  return *m_convexhull;
}
int Tree::set_concavehull(float maxEdgeLenght = 1.5)
{
  QString aa = QString("%1_concave").arg(m_name);
  QColor col = get_color();

  Hull *c = new Hull(m_Cloud,aa,col);
  int i = c->set_concaveZkracovanim(maxEdgeLenght);

  set_concavehull(c->get_concavehull());

  //c->set_areacave(c->get_concavehull());
  m_areaconcave = c->get_areaconcave();
  delete c;
  return i;
}
void Tree::set_concavehull(Cloud c)
{
  m_concavehull->set_Cloud(c.get_Cloud());
}
Cloud Tree::get_concavehull()
{
  return *m_concavehull;
}

float Tree::get_areaconvex()
{
  return m_areaconvex;
}
float Tree::get_areaconcave()
{
  return m_areaconcave;
}

void Tree::set_skeleton()
{

}
void Tree::set_skeleton(Cloud c)
{
  m_skeleton->set_Cloud(c.get_Cloud());
}
Cloud Tree::get_skeleton()
{
  return *m_skeleton;
}
void Tree::set_positionHT(Cloud terrain)
{
// vypocitat stred v 1,3 (m_dbh) a v 0,65 m nad pozici
  stred c13;
  stred c065;

  if(m_dbhCloud->get_Cloud()->points.size() > 5)
  {
    HoughTransform ht13 = m_dbhCloud->get_Cloud();
    ht13.compute();
    c13 = ht13.get_circle();
  }
  else
  {
    QMessageBox::information(0,("tr"),("not possible to calculate circle in 1.3 m above position."));
    return;
  }

  //set cloud in 065 m above position
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud065 (new pcl::PointCloud<pcl::PointXYZI>);

  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z > (m_pose.z + 0.95) && ith->z < (m_pose.z + 1.05))
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      cloud065->points.push_back(bod);
    }
  }

  // calculate circle
  if(cloud065->points.size() > 5)
  {
    HoughTransform ht065;
    ht065.set_Cloud(cloud065);
    ht065.compute();
    c065 = ht065.get_circle();
  }
  else
  {
    QMessageBox::information(0,("tr"),("not possible to calculate circle in 0.65 m above position."));
    return;
  }

// urcit prusecik
    //parametricka primka
    float vx = c065.a - c13.a;
    float vy = c065.b - c13.b;
    float vz = c065.z - c13.z;

  // rovina
    // najit tři nejbližší body terénu
    pcl::PointXYZ A,B,C;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (terrain.get_Cloud());

    std::vector<int> pointId(5);
    std::vector<float> pointSD(5);

    if (kdtree.nearestKSearch (m_pose, 5, pointId, pointSD) > 0 )
    {
       A.x = terrain.get_Cloud()->points.at(pointId.at(0)).x;
       A.y = terrain.get_Cloud()->points.at(pointId.at(0)).y;
       A.z = terrain.get_Cloud()->points.at(pointId.at(0)).z;

       B.x = terrain.get_Cloud()->points.at(pointId.at(1)).x;
       B.y = terrain.get_Cloud()->points.at(pointId.at(1)).y;
       B.z = terrain.get_Cloud()->points.at(pointId.at(1)).z;

       C.x = terrain.get_Cloud()->points.at(pointId.at(2)).x;
       C.y = terrain.get_Cloud()->points.at(pointId.at(2)).y;
       C.z = terrain.get_Cloud()->points.at(pointId.at(2)).z;
    }
    else
    {
      QMessageBox::information(0,("tr"),("no terrain point found. canceled"));
      return;
    }

    //vytvořit rovnici roviny
    pcl::PointXYZ AB;
    AB.x =B.x - A.x;
    AB.y =B.y - A.y;
    AB.z =B.z - A.z;

    pcl::PointXYZ AC;
    AC.x =C.x - A.x;
    AC.y =C.y - A.y;
    AC.z =C.z - A.z;

    float a = (AB.y*AC.z) - (AB.z*AC.y);
    float b = (AB.z*AC.x) - (AB.x*AC.z);
    float c = (AB.x*AC.y) - (AB.y*AC.x);
    float d = -(a*A.x) - (b*A.y) - (c*A.z);


    //vypocitat t
    float up = -d - (a*c13.a) - (b*c13.b) - (c*c13.z);
    float down =  a*vx + b*vy + c*vz;
    if(down == 0)
    {
      QMessageBox::information(0,("df"),("rovnobezne" ));
      return;
    }
    float t = up/down;
    //dosadit do rovnic primky
    pcl::PointXYZI pos;
    pos.x = c13.a +t*vx;
    pos.y = c13.b +t*vy;
    pos.z = (A.z + B.z + C.z)/3;
    pos.intensity = 1;

    m_pose = pos;
}
void Tree::set_stemCurvature()
{
  m_stemCurvature.clear();
  for(int g = 0; g < m_height; g++)
  {
    float h = g;
    if(g == 0)
      h= 0.65;
    if(g == 1)
      h= 1.3;

    HoughTransform *ht = new HoughTransform();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
    //vybrat body do mracna ktere jsou

    for(int i = 0; i < get_Cloud()->points.size(); i++)
    {
      pcl::PointXYZI ith;
      ith = get_Cloud()->points.at(i);
      if (ith.z > (m_pose.z + h - 0.03) && ith.z < (m_pose.z + h + 0.03) && m_dbh_HT.a -ith.x <5 &&  ith.x - m_dbh_HT.a < 5&& m_dbh_HT.b -ith.y <5 &&   ith.y - m_dbh_HT.b < 5)
      {
        cloud_->points.push_back(ith);
      }
    }
    stred res;
// if the cloud is empty
    if(cloud_->points.size() > 5)
    {
      ht->set_Cloud(cloud_);
      ht->compute();

      res = ht->get_circle();
    }
    else
      res = {-1,-1,-1,-1,-0.5};

// if the circle is two times greater than previous two circles
    if(m_stemCurvature.size() > 2 && res.r > (2* m_stemCurvature.at(m_stemCurvature.size()-2).r) && res.r > (2* m_stemCurvature.at(m_stemCurvature.size()-1).r))
      res = {-1,-1,-1,-1,-0.5};

    m_stemCurvature.push_back(res);
    cloud_.reset();
    delete ht;
  }
}
std::vector<stred> Tree::get_stemCurvature()
{
  return m_stemCurvature;
}

