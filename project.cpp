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
#include "project.h"
#include "cloud.h"
#include "hull.h"

//include INPUT OUTPUT
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <iostream>
#include <string>
#include <QtCore/QString.h>



//Projekt a jeho metody
Project::Project()
{
m_x=0;
m_y=0; //coordinate system
m_z=0;
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
m_z=0;
m_projectName = name;
}
Project::Project(double x, double y, QString name)
{
m_x=x;
m_y=y; //coordinate system
m_z=0;
m_projectName = name;

}
Project::Project(double x, double y, double z, QString name)
{
m_x=x;
m_y=y; //coordinate system
m_z=z;
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
QString Project::get_ProjName()
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
void Project::set_zTransform(double z)
{
  m_z=z;
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
void Project::set_VegeCloud(Cloud cloud)
{
  m_vegeCloud.push_back(cloud);
}
 void Project::set_Tree(Cloud cloud)
 {
  m_stromy.push_back(cloud);

  if(m_terrainCloud.size()>0)
  {
    m_stromy.back().set_position();
    m_stromy.back().set_height();
  }

  m_stromy.back().set_dbhCloud();

 }
void Project::set_dbhCloud(QString name,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_dbhCloud(cloud);
    }
  }
}
void Project::set_treeConvexCloud(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_convexhull();
    }
  }
}
int Project::set_treeConcaveCloud(QString name,float edge)
{
    int errors =0;
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      errors = m_stromy.at(i).set_concavehull(edge);
    }
  }
  return errors;
}
Cloud Project::set_ConcaveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,float edge, QString name, QColor color)
{
    Hull *h = new Hull (cloud, name, color);
    QString m = QString("Pred h->set_concaveZkracovanim(edge)").arg(name);
    int errors = h->set_concaveZkracovanim(edge);

    if(errors>0)
      {
        QString m = QString(" Warning: %1 edge are longer than Maximum Edge Lenght.\n In cloud: %2").arg(errors).arg(name);
        QMessageBox::information(0,("Warning"),m);
      }
    return h->get_concavehull();
}
Cloud Project::set_ConvexCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor color)
{

    Hull *h = new Hull (cloud, name, color);
    QString m = QString("Pred h->set_concaveZkracovanim(edge)").arg(name);
    h->set_convexhull();

    return h->get_convexhull();
}
void Project::set_treePosition(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_position();
    }
  }
}
void Project::set_treePosition(QString name, Cloud terrain)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_position(terrain);
    }
  }
}
void Project::set_treeheigth(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_height();
    }
  }
}
void Project::set_treeDBHCloud(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_dbhCloud();
      m_stromy.at(i).set_dbhHT();
      m_stromy.at(i).set_dbhLSR();

    }
  }
}
void Project::set_skeleton(QString name, Cloud c)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_skeleton(c);
    }
  }
}
void Project::set_length(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_length();
    }
  }
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
double Project::get_Ztransform()
{
  return m_z;
}
QString Project::get_Path()
{
  return m_path;
}
Cloud Project::get_TerrainCloud(int i)
{
  return m_terrainCloud.at(i);
}
Cloud Project::get_TerrainCloud(QString name)
{
  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name() == name)
      return m_terrainCloud.at(i);
  }
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
Tree Project::get_TreeCloud(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
      return m_stromy.at(i);
  }
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
      return m_vegeCloud.at(i) ;
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
    return m_ostCloud.at(i); //c = new Cloud(get_ostCloud(i));
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
     return m_stromy.at(i);
  }
}
void Project::save_newCloud(QString type, QString path)
{
  //save cloud
  QString path1 = save_Cloud(path);
  //otevrit Proj.3df file pro pripsani
  QString projfile = QString("%1/%2.3df").arg(get_Path()).arg(get_ProjName());
  QFile file (projfile);
  if(!file.exists())
  {
    projfile = QString("%1\\proj.3df").arg(get_Path()).arg(get_ProjName());
    QFile file (projfile);
  }

  file.open(QIODevice::Append | QIODevice::Text);
  QTextStream out(&file);
  out  << type << " " << path1<<"\n";
  file.close();
}
void Project::save_newCloud(QString type, QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c )
{
  QString path = save_Cloud(name,c);
    //otevrit Proj.3df file pro pripsani
  QString projfile = QString("%1\\%2.3df").arg(get_Path()).arg(get_ProjName());

  QFile file (projfile);
  if(!file.exists())
  {
    projfile = QString("%1\\proj.3df").arg(get_Path()).arg(get_ProjName());
    QFile file (projfile);
  }
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

  return save_Cloud(ext.front(),cloud);
}
QString Project::save_Cloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c)
{
  QString path_out = QString ("%1\\%2.pcd").arg(get_Path()).arg(name);
  pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *c);
  return path_out;
}
bool Project::cloud_exists(QString name)
{
  // for kazdy cloud v project
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name()== name)
    return true;
  }
  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
      return true;
      //c = new Cloud( get_TerrainCloud(i));
  }
  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
      return true;
  }
  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
    return true;
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
     return true;
  }
  return false;
}
void Project::delete_Cloud(QString name)
{
  //otevrit soubor proj.3df vymazat radek vyhledany podle zadaneho textu
  QString filepath = QString ("%1/%2.3df").arg(m_path).arg(m_projectName);
  QString fileout = QString ("%1/%2_t.3df").arg(m_path).arg(m_projectName);

  QFile file(filepath);
  if(!file.exists())
  {
    filepath = QString("%1\\proj.3df").arg(get_Path());
    fileout = QString("%1\\proj_t.3df").arg(get_Path());
    QFile file (filepath);
  }
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
  int vecsize;
  vecsize= m_baseCloud.size();
  for(int i = 0; i < m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name() == name)
    {
      if (i == 0)
        m_baseCloud.erase(m_baseCloud.begin());
      else
        m_baseCloud.erase(m_baseCloud.begin()+i);
      //m_baseCloud.resize(vecsize - 1);
      return;
    }
  }
  vecsize= m_terrainCloud.size();
  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
    {
      if (i == 0)
        m_terrainCloud.erase(m_terrainCloud.begin());
      else
        m_terrainCloud.erase(m_terrainCloud.begin()+i);
      //m_terrainCloud.resize(vecsize - 1);
      return;
    }
  }
  vecsize= m_vegeCloud.size();
  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
    {
      if (i == 0)
        m_vegeCloud.erase(m_vegeCloud.begin());
      else
        m_vegeCloud.erase(m_vegeCloud.begin()+i);
     // m_vegeCloud.resize(vecsize - 1);
      return;
    }
  }
  vecsize= m_ostCloud.size();
  for(int i = 0; i< m_ostCloud.size(); i++)
  {
    if (get_ostCloud(i).get_name() == name)
    {
      if (i == 0)
        m_ostCloud.erase(m_ostCloud.begin());
      else
        m_ostCloud.erase(m_ostCloud.begin()+i);
     // m_ostCloud.resize(vecsize - 1);
      return;
    }
  }
  vecsize= m_stromy.size();
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      if (i == 0)
        m_stromy.erase(m_stromy.begin());
      else
        m_stromy.erase(m_stromy.begin()+i);
      //m_stromy.resize(vecsize - 1);
      return;
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
  QString filepath = QString ("%1/%2.3df").arg(m_path).arg(m_projectName);
  QString filepathtmp = QString ("%1/%2.tmp").arg(m_path).arg(m_projectName);
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

