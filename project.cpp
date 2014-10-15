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
}
Cloud::Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col)
{
  m_name = name;
  m_Cloud=cloud;
  set_color(col);
}
Cloud::Cloud()
{

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
  m_Cloud = cloud;
}
void Cloud::set_name(QString name)
{
  m_name = name;
}

//Tree
Tree::Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col, stred s)
: Cloud(cloud, name, col)
{
  stred c={0,0,0,0,0};
  set_dbh(c);
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
}
Tree::Tree (Cloud cloud)
: Cloud(cloud)
{
  stred c={0,0,0,0,0};
  set_dbh(c);
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
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
    if (ith->z > (m_pose.z + 1.22) && ith->z < (m_pose.z + 1.38))
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      dbhCloud->points.push_back(bod);
    }
  }
  QString a = QString("%1_dbh").arg(m_name);
  QColor col = QColor(255,0,0);
  Cloud *c = new Cloud(dbhCloud, a);
  m_dbhCloud = *c;
}
void Tree::set_dbh()
{
//VOXELIZATION
  if(!m_dbhCloud.get_Cloud()->points.empty())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (m_dbhCloud.get_Cloud());
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
    if (ith->z < (m_minp.z + 1) )
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

pcl::PointXYZI Tree::get_lpoint(bool low)
{
  if (low ==false)
    {return m_lmin;}
  else
    {return m_lmax;}
}
Cloud Tree::get_dbhCloud()
{
  return m_dbhCloud;
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
  m_stromy.at(i) = cloud;
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
  QString projfile = QString("%1\proj.3df").arg(get_Path());
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
  QString projfile = QString("%1/proj.3df").arg(get_Path());
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

  QString path_out = QString ("%1/%2.pcd").arg(get_Path()).arg(ext.front());
  pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *cloud);
  return path_out;
}
QString Project::save_Cloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c)
{
  QString path_out = QString ("%1/%2.pcd").arg(get_Path()).arg(name);
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

//CLOUD
