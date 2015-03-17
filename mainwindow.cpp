//  <3DFOREST - tool for processing lidar data from forest environment>
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
#include "mainwindow.h"

#include <liblas/liblas.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
//include BASE
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//include INPUT OUTPUT
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//include VISUALIZATION
#include <pcl/visualization/area_picking_event.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
//include OCTREE
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

//include FILTERS
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <vtkWin32OpenGLRenderWindow.h>

////MainWindow
 MainWindow::MainWindow()
 :m_vis (new Visualizer ("3D Viewer"))
{

  Q_INIT_RESOURCE(3dforest);
  setWindowTitle ( QString("3D Forest - Forest lidar data processing tool") );
  m_cloud1 = new Cloud();
  m_cloud = new Cloud();
  Proj = new Project();

//QVTKwidget - visualizer
  qvtkwidget = new QVTKWidget();
  vtkSmartPointer<vtkRenderWindow> renderWindow = m_vis->getRenderWindow();
  qvtkwidget->SetRenderWindow(renderWindow);
//  m_vis->addOrientationMarkerWidgetAxes();
  m_vis->setShowFPS(false);
  //m_vis->setBackgroundColor(10,10,10);
  setCentralWidget(qvtkwidget);
  qvtkwidget->show();

// Tree widget
  treeWidget = new MyTree;
  QDockWidget *dockWidget = new QDockWidget(tr("Pointcloud Layers"), this);
  dockWidget->setAllowedAreas(Qt::LeftDockWidgetArea);
  dockWidget->setMaximumWidth(500);
  dockWidget->setWidget(treeWidget);
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget);

  resize(QSize(1024, 800));
  move(QPoint(50, 50));

  createActions();
  createMenus();

  statusBar()->showMessage(tr("3D FOREST Ready"));
  point_ev = m_vis->registerPointPickingCallback (&MainWindow::pointEvent, *this );
}

//PROJECT methods
void MainWindow::newProject()
{
  ProjImport * wiz = new ProjImport(this);
  wiz->setStartId(1);
  if(wiz->exec() == 1)
  {
    QString filename = QString("%1\\%2\\%3.3df").arg(wiz->field("projectPath").toString()).arg(wiz->field("projectName").toString()).arg(wiz->field("projectName").toString());
    closeProject();
    openProject(filename);
  }
}
void MainWindow::openProject()
{
//SET 3DF FILE
  QString fileName = QFileDialog::getOpenFileName(this,tr("Open project file"),"",tr("files (*.3df)"));
  if (fileName.isEmpty())
    return;
  closeProject();
  openProject(fileName);
}
void MainWindow::openProject(QString path)
{
  QFile file (path);
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&file);

  bool first_line = true;
  while(!in.atEnd())
  {
    QString lines = in.readLine();

    QStringList coords = lines.split(" ");
//READ FIRST LINE

    if(first_line == true)
    {
      if(coords.size() == 5)
      {
        Proj = new Project(coords.at(1).toDouble(), coords.at(2).toDouble(), coords.at(3).toDouble(), coords.at(0).toUtf8().constData());
        Proj->set_path(coords.at(4).toUtf8().constData());

      }
      else
      {
        Proj = new Project(coords.at(1).toDouble(), coords.at(2).toDouble(), coords.at(0).toUtf8().constData());
        Proj->set_path(coords.at(3).toUtf8().constData());
        Proj->set_zTransform(0);
      }
      first_line = false;
      QString a = QString("3DForest - %1 ").arg(coords.at(0).toUtf8().constData());
      setWindowTitle(a);
    }
//READ REST OF FILE
    else
    {
      if(coords.at(0)=="cloud")
      {
        if(coords.size() >3)
        {
          QColor col = QColor(coords.at(2).toInt(),coords.at(3).toInt(),coords.at(4).toInt());
          openCloudFile(coords.at(1),col);
        }
        else
          openCloudFile(coords.at(1));
      }
      if(coords.at(0)=="teren")
      {
        if(coords.size() >3)
        {
        QColor col = QColor(coords.at(2).toInt(),coords.at(3).toInt(),coords.at(4).toInt());
        openTerrainFile(coords.at(1),col);
        }
        else
          openTerrainFile(coords.at(1));
      }
      if (coords.at(0)=="vege")
      {
        if(coords.size() >3)
        {
        QColor col = QColor(coords.at(2).toInt(),coords.at(3).toInt(),coords.at(4).toInt());
        openVegeFile(coords.at(1),col);
        }
        else
          openVegeFile(coords.at(1));
      }
      if (coords.at(0)=="strom")
      {
        if(coords.size() >3)
        {
        QColor col = QColor(coords.at(2).toInt(),coords.at(3).toInt(),coords.at(4).toInt());
        openTreeFile(coords.at(1),col);
        }
        else
          openTreeFile(coords.at(1));
      }
      if (coords.at(0)=="ost")
      {
        if(coords.size() >3)
        {
        QColor col = QColor(coords.at(2).toInt(),coords.at(3).toInt(),coords.at(4).toInt());
        openOstFile(coords.at(1),col);
        }
        else
          openOstFile(coords.at(1));
      }
    }
  }
  file.close();
  //m_vis->resetCamera();
}
void MainWindow::closeProject()
{
  //clean project
  Proj->cleanAll();
  //clean treewidget
  treeWidget->cleanAll();
  //clean qvtwidget
  m_vis->removeAllPointClouds();
  m_vis->removeAllShapes();

  qvtkwidget->update();

}
void MainWindow::importProject()
{
  ProjImport * wiz = new ProjImport(this);
  wiz->setStartId(3);
  if(wiz->exec() == 1)
  {
    QString filename = QString("%1\\%2\\%2.3df").arg(wiz->field("newprojectPath").toString()).arg(wiz->field("newprojectname").toString());
    closeProject();
    openProject(filename);
  }
}
//IMPORT methods

void MainWindow::importtxt()
{
  QStringList ls =QFileDialog::getOpenFileNames(this,tr("open file"),"",tr("files (*.txt *xyz)"));
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }

  for(int i=0; i<=ls.size()-1; i++)
  {
    QString fileName = ls.at(i);
    if( !fileName.isEmpty())
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
      QFile file (fileName);
      file.open(QIODevice::ReadOnly | QIODevice::Text);
      QTextStream in(&file);
      while(!in.atEnd())
      {
        QString line = in.readLine();
        QStringList coords = line.split(" ");
        if(coords.size() >1)
        {
          pcl::PointXYZI data;

          data.x = (float)(coords.at(0).toDouble() + Proj->get_Xtransform());
          data.y = (float)(coords.at(1).toDouble() + Proj->get_Ytransform());
          data.z = coords.at(2).toFloat();
          data.intensity=1;//coords.at(3).toFloat();
          cloud->points.push_back(data);
        }
      }
      file.close();
      //in.~QTextStream();
      cloud->width = cloud->points.size();
      cloud->is_dense=true;
      cloud->height=1;

      QStringList Fname = fileName.split("\\");
      QStringList name = Fname.back().split(".");

      Proj->save_newCloud("cloud",name.at(0),cloud);

      QString fullname = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name.at(0));
      openCloudFile(fullname);
    }
  }
}
void MainWindow::importpts()
{
  QStringList ls =QFileDialog::getOpenFileNames(this,tr("open PTS files"),"",tr("files (*.pts)"));
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }

  for(int i=0; i<=ls.size()-1; i++)
  {
    QString fileName = ls.at(i);
    if( !fileName.isEmpty())
    {
      QStringList Fname = fileName.split("\\");
      QStringList name = Fname.back().split("."); // name.at(0) - NEW FILE NAME
      QFile file (fileName);
      file.open(QIODevice::ReadOnly | QIODevice::Text);
      QTextStream in(&file);
      bool first_line = true;
      int cloud_number = 1;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
      while(!in.atEnd())
      {
        QString line = in.readLine();
        QStringList coords = line.split(" ");

        if(coords.size() != 4 ) // header
        {
          if (first_line != true )
          {
            cloud->width = cloud->points.size();
            cloud->is_dense=true;
            cloud->height=1;
          //cloud name
            QString a = QString ("%1_%2").arg(name.at(0)).arg(cloud_number);
          // save cloud
            Proj->save_newCloud( "cloud", a, cloud);
          //empty cloud
            cloud->points.clear();
            cloud_number++;
          }
          first_line = false;

        }
        else //points
        {
          pcl::PointXYZI data;
          data.x = coords.at(0).toDouble() + Proj->get_Xtransform();
          data.y = coords.at(1).toDouble() + Proj->get_Ytransform();
          data.z = coords.at(2).toDouble() + Proj->get_Ztransform();
          data.intensity=coords.at(3).toFloat();
          cloud->points.push_back(data);
        }
      }
      file.close();
      cloud->width = cloud->points.size();
      cloud->is_dense=true;
      cloud->height=1;
    //cloud name
      QString a = QString ("%1_%2").arg(name.at(0)).arg(cloud_number);
    // save cloud
      Proj->save_newCloud("cloud",a,cloud);
    //empty cloud
      cloud_number++;

      for(int i = 1; i < cloud_number; i++)
      {
        QString fullname = QString("%1\\%2_%3.pcd").arg(Proj->get_Path()).arg(name.at(0)).arg(i);
        openCloudFile(fullname);
      }
    }
  }
}
void MainWindow::importptx()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  QString fileName = QFileDialog::getOpenFileName(this,tr("open file"),"",tr("files (*.ptx)"));
  if (fileName.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","no name filled");
    return;
  }
  else
  {
    QFile file (fileName);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&file);
    int i = 0;
    while(!in.atEnd())
    {
      if (i < 11)
      {
          QString hlavicka = in.readLine();
      }
      else
      {
      QString line = in.readLine();
      QStringList coords = line.split(" ");
      pcl::PointXYZI data;
      double x,y;
      float z,i;
      x = coords.at(0).toDouble();
      y = coords.at(1).toDouble();
      z = coords.at(2).toFloat();
      i = coords.at(3).toFloat();
          if (x != 0 && y != 0 && z != 0 && i != 0.5)
          {
            data.x = x+Proj->get_Xtransform();
            data.y = y+Proj->get_Ytransform();
            data.z = z;
            data.intensity=i;
            cloud->points.push_back(data);
          }
      }
      i++;
    }
    file.close();
    in.~QTextStream();
    cloud->width = cloud->points.size ();
    cloud->is_dense=true;
    cloud->height=1;

    QStringList Fname = fileName.split("/");
    QStringList name = Fname.back().split(".");
    Proj->save_newCloud("cloud",name.at(0),cloud);
    QString fullname = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name.at(0));
//open new file
    openCloudFile(fullname);

  }
}
void MainWindow::importlas()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  QString fileName = QFileDialog::getOpenFileName(this,tr("open file"),"",tr("files (*.las)"));
  if (fileName.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","no name filled");
    return;
  }
  else
  {
    std::ifstream ifs;
    ifs.open(fileName.toUtf8().constData(), std::ios::in | std::ios::binary);

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();

//read data into cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    while (reader.ReadNextPoint())
    {
      liblas::Point const& p = reader.GetPoint();
      pcl::PointXYZI data;

      data.x = p.GetX()+Proj->get_Xtransform();
      data.y = p.GetY()+Proj->get_Ytransform();
      data.z = p.GetZ();
      data.intensity=0;
      cloud->points.push_back(data);
    }
    cloud->width = cloud->points.size ();
    cloud->is_dense=true;
    cloud->height=1;

    QString name = QInputDialog::getText(this, tr("Name of Import File"),tr("Please insert name of\nimported file (e.g. cloud)"));
    QString fullname = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name);
    Proj->save_newCloud("cloud",name,cloud);
//open new file
    openCloudFile(fullname);
  }
}
void MainWindow::importTerrainFile()
{
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("open file"),"",tr("files (*.pcd)"));
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }

  for(int i=0;i<ls.size(); i++)
  {
    QString fileName = ls.at(i);
    openTerrainFile(fileName);
    Proj->save_newCloud("teren",fileName);
  }
}
void MainWindow::importCloud()
{
   QStringList ls = QFileDialog::getOpenFileNames(this,tr("open file"),"",tr("files (*.pcd)"));
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    QString fileName = ls.at(i);
    openCloudFile(fileName);
    Proj->save_newCloud("cloud",fileName);
  }
}
void MainWindow::importVegeCloud()
{
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("open file"),"",tr("files (*.pcd)"));
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }

  for(int i=0;i<ls.size(); i++)
  {
    QString fileName = ls.at(i);
    openVegeFile(fileName);
    Proj->save_newCloud("vege",fileName);
  }
}
void MainWindow::importTreeCloud()
{
  QStringList ls =QFileDialog::getOpenFileNames(this,tr("open file"),"",tr("files (*.pcd)"));
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }

  for(int i=0;i<ls.size(); i++)
  {
    QString fileName = ls.at(i);
    openTreeFile(fileName);
    Proj->save_newCloud("strom",fileName);
  }
}


//OPEN PROTECTED methods
void MainWindow::openTerrainFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);

  Proj->set_TerrainCloud(*c);
  //Proj->save_color(coords.back(),col);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
 // manualSelAct->setEnabled(true);
  //manualADAct->setEnabled(true);
}
void MainWindow::openVegeFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);

  Proj->set_VegeCloud(*c);
  //Proj->save_color(coords.back(),col);
  dispCloud(*c);
  addTreeItem(c->get_name());
  //manualSelAct->setEnabled(true);
  m_vis->resetCamera();
}
void MainWindow::openTreeFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");

  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_Tree(*c);
  //Proj->save_color(coords.back(),col);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
}
void MainWindow::openOstFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);

  Proj->set_OstCloud(*c);
  //Proj->save_color(coords.back(),col);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  //manualSelAct->setEnabled(true);
}
void MainWindow::openCloudFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);


  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);

  Proj->set_baseCloud(*c);
  //Proj->save_color(coords.back(),col);
  dispCloud(*c);
  addTreeItem(c->get_name());
m_vis->resetCamera();
}
void MainWindow::openTerrainFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);

  Proj->set_TerrainCloud(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
       //menu
 // manualADAct->setEnabled(true);
}
void MainWindow::openVegeFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_VegeCloud(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
 // manualSelAct->setEnabled(true);
  m_vis->resetCamera();
}
void MainWindow::openOstFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_OstCloud(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
 // manualSelAct->setEnabled(true);
}
void MainWindow::openTreeFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_Tree(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
}
void MainWindow::openCloudFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_baseCloud(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
}

//EXPORT method
void MainWindow::exportCloud()
{
//vybrat cloud
  QString cloudName = QInputDialog::getItem(this,("select cloud for export into txt"),("name of cloud:"),get_allNames());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  *cloud = *Proj->get_Cloud(cloudName).get_Cloud();

//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("insert file name"),"",tr("files (*.txt)"));
//zapisovat jednotlive radky
  QFile file (newFile);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(3);
  for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
  {
    double x = it->x - Proj->get_Xtransform();
    double y = it->y - Proj->get_Ytransform();
    out << x << " " << y << " " << it->z << "\n";
  }
  file.close();
}
void MainWindow::plysave()
{
  //vybrat cloud
  QStringList names;
  for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
  {
    names << Proj->get_TerrainCloud(i).get_name();
  }
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  QString cloudName = QInputDialog::getItem(this,("select cloud"),("name of cloud for export:"),names);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  *cloud = *Proj->get_Cloud(cloudName).get_Cloud();
//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("insert file name"),"",tr("files (*.ply)"));
  //zapisovat jednotlive radky

  pcl::io::savePLYFileASCII(newFile.toUtf8().constData(),*cloud);

}
void MainWindow::exportPts()
{
//vybrat cloud
  QStringList names;
  for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
  {
    names << Proj->get_TerrainCloud(i).get_name();
  }
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  QString cloudName = QInputDialog::getItem(this,("select cloud for voxelization"),("name of cloud:"),names);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  *cloud = *Proj->get_Cloud(cloudName).get_Cloud();
//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("insert file name"),"",tr("files (*.pts)"));
//zapisovat jednotlive radky
  QFile file (newFile);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(3);

    int pocetbodu = cloud->width;
    out << pocetbodu << "\n";

  for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
  {
    double x = it->x - Proj->get_Xtransform();
    double y = it->y - Proj->get_Ytransform();
    out << x << " " << y << " " << it->z << "\n";
  }
  file.close();
}
void MainWindow::exportConvexTxt()
{
    //vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("insert file name"),"",tr("files (*.txt)"));
   //zapisovat jednotlive radky
  QFile file (newFile);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(3);
  out << "ID;Xstart;Ystart;Xend;Yend;Z \n";
  for(int i =0;i < Proj->get_sizeTreeCV(); i++)
  {
    QString name = Proj->get_TreeCloud(i).get_name();
    Cloud *cloud = new Cloud (Proj->get_TreeCloud(i).get_vexhull());

    for(int j = 1; j < cloud->get_Cloud()->points.size(); j++)
    {
        pcl::PointXYZI bod;
        bod = cloud->get_Cloud()->points.at(j-1);
        double x = bod.x - Proj->get_Xtransform();
        double y = bod.y - Proj->get_Ytransform();
        out << i << ";" << x << ";" << y ;
        bod = cloud->get_Cloud()->points.at(j);
        double xend = bod.x - Proj->get_Xtransform();
        double yend = bod.y - Proj->get_Ytransform();
        double z = bod.z;
        out << ";" << xend << ";" << yend << ";" << z << endl;
    }
  }
  file.close();


}
void MainWindow::exportConcaveTxt()
{
//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("insert file name"),"",tr("files (*.txt)"));
   //zapisovat jednotlive radky
  QFile file (newFile);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(3);
  out << "ID;Xstart;Ystart;Xend;Yend;Z \n";
  for(int i =0;i < Proj->get_sizeTreeCV(); i++)
  {
    QString name = Proj->get_TreeCloud(i).get_name();
    Cloud *cloud = new Cloud (Proj->get_TreeCloud(i).get_concavehull());
    for(int j = 1; j < cloud->get_Cloud()->points.size(); j++)
    {
        pcl::PointXYZI bod;
        bod = cloud->get_Cloud()->points.at(j-1);
        double x = bod.x - Proj->get_Xtransform();
        double y = bod.y - Proj->get_Ytransform();
        out << i << ";" << x << ";" << y ;
        bod = cloud->get_Cloud()->points.at(j);
        double xend = bod.x - Proj->get_Xtransform();
        double yend = bod.y - Proj->get_Ytransform();
        double z = bod.z;
        out << ";" << xend << ";" << yend << ";" << z << endl;
    }
  }
  file.close();
}


//EXIT method
void MainWindow::closeEvent(QCloseEvent *event)
{
  event->accept();
}

//TEREN methods
void MainWindow::voxelgrid()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Terrain voxel");
  in->set_path(Proj->get_Path());
  in->set_description("Terrain is derived from voxels. Input point cloud is reduced into centroid of voxels and only those with lowest z coordinate anre selected as a ground.");
  in->set_inputCloud1("Input cloud:",get_baseNames());
  in->set_outputCloud1("Output cloud of ground:","voxel-terrain");
  in->set_outputCloud2("Output cloud of non-ground:","voxel-vegetation");
  in->set_inputInt("Resolution in cm:","50");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    if(in->get_inputCloud1().isEmpty())
      return;
    float res = in->get_intValue()/100.0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vox;

    vox.setInputCloud (Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    vox.setLeafSize (res, res, res/2);
    vox.filter (*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  //only lowest points
    cloud = lowPoints(cloud_filtered,res/2, true);

//save dem cloud
    Proj->save_newCloud("teren",in->get_outputCloud1(),cloud);
    QString fullnameT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud1());
    openTerrainFile(fullnameT);


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege (new pcl::PointCloud<pcl::PointXYZI>);
  //vybrat mimo spodni voxely -kdtree bude lepsi
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (cloud);

    for(int i=0; i < cloud_filtered->points.size();i++)
    {
      pcl::PointXYZI searchPointV;
      searchPointV=cloud_filtered->points.at(i);
      std::vector<int> pointIDv;
      std::vector<float> pointSDv;

      if(kdtree.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) < 1)
      {
        pcl::PointXYZI in;
        in.x = searchPointV.x;
        in.y = searchPointV.y;
        in.z = searchPointV.z;
        in.intensity = searchPointV.intensity;
        cloud_vege->points.push_back(in);
      }
    }

//save vege cloud
      cloud_vege->width = cloud_vege->points.size ();
      cloud_vege->is_dense=true;
      cloud_vege->height=1;

      Proj->save_newCloud("vege",in->get_outputCloud2(),cloud_vege);
      QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud2());
      openVegeFile(fullnameV);

  }
}

void MainWindow::octreeSlot()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Terrain created with octree ");
  in->set_path(Proj->get_Path());
  in->set_description("Input point cloud is divided into cubes with given resolution and from all cubes, which  contains points are selected lowest."
                        " Output cloud contains point from selected cubes. ");
  in->set_inputCloud1("Input cloud:",get_baseNames());
  in->set_outputCloud1("Output cloud of ground:","Octree-terrain");
  in->set_outputCloud2("Output cloud of non-ground:","Octree-vegetation");
  in->set_inputInt("Resolution","10");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    if(in->get_inputCloud1().isEmpty())
      return;
    float res = in->get_intValue()/100.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp4(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp5(new pcl::PointCloud<pcl::PointXYZI>);

      //velky cyklus
    octree(res*10,Proj->get_Cloud(in->get_inputCloud1()).get_Cloud(),cloud_vege, cloud_tmp);
    octree(res/2,cloud_tmp,cloud_vege, cloud_tmp2);

    //maly cyklus
    octree(res,Proj->get_Cloud(in->get_inputCloud1()).get_Cloud(),cloud_vege, cloud_tmp3);

      //najdi nejblizsi bod v mezi dvema cykly a pokud jsou body od sebe do 0,001 vyber ho a uloz do cloud_tmp6

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (cloud_tmp3);

    for(int i=0; i < cloud_tmp2->points.size();i++)
    {
      pcl::PointXYZI searchPointV;
      searchPointV=cloud_tmp2->points.at(i);
      std::vector<int> pointIDv;
      std::vector<float> pointSDv;

      if(kdtree.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) > 0)
      {
        pcl::PointXYZI in;
        in.x = searchPointV.x;
        in.y = searchPointV.y;
        in.z = searchPointV.z;
        in.intensity = searchPointV.intensity;
        cloud_tmp5->points.push_back(in);
      }
    }
    cloud_tmp5->width = cloud_tmp5->points.size ();
    cloud_tmp5->is_dense=true;
    cloud_tmp5->height=1;

//VEGE CLOUD
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtre;
    kdtre.setInputCloud (cloud_tmp5);
    std::vector<int> pointIDv;

    for(int i=0; i < Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.size();i++)
    {
      pcl::PointXYZI searchPointV;
      searchPointV=Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i);
      std::vector<int> pointIDv;
      std::vector<float> pointSDv;

      if(kdtre.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) < 1)
      {
        pcl::PointXYZI in;
        in.x = searchPointV.x;
        in.y = searchPointV.y;
        in.z = searchPointV.z;
        in.intensity = searchPointV.intensity;
        cloud_vege->points.push_back(in);
      }
    }
    cloud_vege->width = cloud_vege->points.size ();
    cloud_vege->is_dense=true;
    cloud_vege->height=1;

//SAVE
//Terrain
      Cloud *cT = new Cloud(cloud_tmp5,in->get_outputCloud1());
      saveTerrainCloud(cT);
    //vegetation
      Cloud *cV = new Cloud(cloud_vege,in->get_outputCloud2());
      saveVegeCloud(cV);
  }
}
void MainWindow::manualAdjust()
{
  InputDialog *in = new InputDialog(this);
  in->set_path(Proj->get_Path());
  in->set_title("Manual adjustment of terrain cloud");
  in->set_description("Manual editing of point cloud representing terrain. For editing please press key 'x' and outline rectangle."
                        " Selected points will be deleted. and saved in output cloud ");
  in->set_inputCloud1("Input terrain cloud:",get_terrainNames());
  in->set_outputCloud1("Output cloud of deleted points:","terrain-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    QString name = QString("%1.edit").arg(in->get_inputCloud1());
    m_cloud->set_name(name);
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //spustit editacni listu
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopE = editBar->addAction("Stop EDIT");
    connect(stopE,SIGNAL(triggered()),this,SLOT(manualAdjustStop()) );
    QAction *undoAct = editBar->addAction("undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );

    undopoint.clear();
    m_vis->removeAllPointClouds();
    treeWidget->allItemOFF();
    dispCloud(*m_cloud,220,220,0);
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
}
void MainWindow::manualAdjustStop()
{
  //save m_cloud1 into ost cloud
  saveOstCloud(m_cloud1);
//save m_cloud into old file
  QStringList name = m_cloud->get_name().split(".edit");

  QString path_out = QString ("%1\\%2").arg(Proj->get_Path()).arg(name.at(0));
  pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *m_cloud->get_Cloud());

  Proj->get_Cloud(name.at(0)).set_Cloud(m_cloud->get_Cloud());
  treeWidget->itemON(name.at(0));
  m_vis->removeAllPointClouds();
  dispCloud(name.at(0));
  //opencloud
  //openCloudFile(path_out);
  delete editBar;
  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  undopoint.clear();
  area.disconnect();
}

//VEGETATION
void MainWindow::manualSelect()
{
  QStringList names ;
  names << get_vegetationNames() << get_ostNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Manual selection of trees");
  in->set_path(Proj->get_Path());
  in->set_description("Manual editing of point cloud representing vegetation. For editing please press key 'x' and outline rectangle."
                        " Selected points will be deleted. After selection you can choose if you want edit more or you want to close it.");
  in->set_inputCloud1("Input Vegetation cloud:",names);
  in->set_outputCloud1("Output cloud of deleted points:","vegetation-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    m_cloud->set_name(in->get_inputCloud1());
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //spustit editacni listu

    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopEd = editBar->addAction("Stop EDIT");
    connect(stopEd,SIGNAL(triggered()),this,SLOT(manualSelectStop()) );

    QAction *undoAct = editBar->addAction("undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();

    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->removeAllPointClouds();
    treeWidget->allItemOFF();
    dispCloud(*m_cloud,220,220,0);
    m_vis->resetCamera();
    qvtkwidget->update();
  }
}
void MainWindow::manualSelectStop()
{
// save m_cloud as new tree
  saveTreeCloud(m_cloud->get_Cloud());
  //save m_cloud1 into file
  // if exist update file and memory
  QString file = QString("%1.pcd").arg(m_cloud1->get_name());
  if(Proj->cloud_exists(file) == true )
  {
    saveVegeCloud(m_cloud1, true);
    Proj->get_Cloud(file).set_Cloud(m_cloud1->get_Cloud());
  }
  else//save new vege cloud
  {
    saveVegeCloud(m_cloud1, false);
  }


//ask if continue
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("Continue editting?");
	msgBox->setInformativeText("Do you want to continue editting the cloud?");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox->setDefaultButton(QMessageBox::Yes);

  if(msgBox->exec() == QMessageBox::Yes)
  {
    treeWidget->allItemOFF();
    m_cloud->get_Cloud()->clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    *h_cloud += *m_cloud1->get_Cloud();
    m_cloud->set_Cloud(h_cloud);
    m_cloud1->get_Cloud()->points.clear();
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
  }
  else
  {
    treeWidget->allItemOFF();
    removeCloud(m_cloud1->get_name());
    removeCloud(m_cloud->get_name());
    m_cloud->get_Cloud()->clear();
    m_cloud1->get_Cloud()->clear();
    delete editBar;
    area.disconnect();
  }
}

//TREE ATRIBUTES
void MainWindow::treeEdit()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Manual editing of trees");
  in->set_path(Proj->get_Path());
  in->set_description("Manual editing of point cloud representing single tree. For editing please press key 'x' and outline rectangle."
                        " Selected points will be deleted. After selection you can choose if you want edit more or you want to close it.");
  in->set_inputCloud1("Input Tree cloud:",get_treeNames());
  in->set_outputCloud1("Output cloud of deleted points:","tree-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    QString name = QString("%1.edit").arg(in->get_inputCloud1());
    m_cloud->set_name(name);
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());


    //spustit editacni listu
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction("Stop EDIT");
    connect(stopTE,SIGNAL(triggered()),this,SLOT(treeEditStop()) );
    QAction *undoAct = editBar->addAction("undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();

    treeWidget->allItemOFF();
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
}
void MainWindow::treeEditStop()
{
  // save m_cloud as new tree - zmenit ulozeni
  QStringList name = m_cloud->get_name().split(".edit");
  if(m_cloud->get_Cloud()->points.size() > 1)
  {
    QStringList name2 = name.at(0).split(".");
    saveTreeCloud(m_cloud->get_Cloud(),name2.at(0),true);
    Proj->get_TreeCloud(name.at(0)).set_Cloud(m_cloud->get_Cloud());
    treeWidget->itemON(name.at(0));
    dispCloud(Proj->get_TreeCloud(name.at(0)));
  }

  if(m_cloud1->get_Cloud()->points.size() > 1)
  {
    saveOstCloud(m_cloud1);
  }
  removeCloud(m_cloud1->get_name());
  removeCloud(m_cloud->get_name());
  dispCloud(Proj->get_TreeCloud(name.at(0)));
  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  delete editBar;
  area.disconnect();
}

void MainWindow::dbhCloudEdit()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Manual edit of trees");
  in->set_path(Proj->get_Path());
  in->set_description("Manual editing of point cloud representing tree point for DBH computation. For editing please press key 'x' and outline rectangle."
                        " Selected points will be deleted. after editing cloud will be saved.");
  in->set_inputCloud1("Input Tree cloud:",get_treeNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    QString name = QString("%1.edit").arg(in->get_inputCloud1());
    m_cloud->set_name(name);
    m_cloud->set_Cloud(Proj->get_TreeCloud(in->get_inputCloud1()).get_dbhCloud());
    m_cloud->set_Psize(Proj->get_TreeCloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name("rest");
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());


    //spustit editacni listu
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction("Stop EDIT");
    QAction *undoAct = editBar->addAction("undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();

    connect(stopTE,SIGNAL(triggered()),this,SLOT(dbhCloudStopEdit()) );
    treeWidget->allItemOFF();
    m_vis->removeAllPointClouds();
    m_vis->removeAllShapes();
    dispCloud(*m_cloud,220,220,0);
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
}
void MainWindow::dbhCloudStopEdit()
{
  // save m_cloud as new tree_dbh -
  if(m_cloud->get_Cloud()->points.size() > 1)
  {
    QStringList name = m_cloud->get_name().split(".edit");
    Proj->set_dbhCloud(name.at(0),m_cloud->get_Cloud());
  }
  removeCloud(m_cloud1->get_name());
  removeCloud(m_cloud->get_name());


  //dispCloud(Proj->get_TreeCloud(m_cloud->get_name()).get_dbhCloud(), 255,0,0);
  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  delete editBar;
  area.disconnect();
}
void MainWindow::treeAtributes()
{
//TODO: volit ktere parametry budou zapsany
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  ExportAttr *exdialog = new ExportAttr (this);
  exdialog->set_description(" Export of tree attributes. You can select which atributes you want to export into text file.");
  exdialog->set_trees(names);
  int dl = exdialog->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
      //progressbar
    pBar = new QProgressBar(statusBar());
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    QFile file (exdialog->get_outputFile());
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);

//header
    out << "Cloud_name" ;
    if(exdialog->get_Points() == true)
      out << exdialog->get_separator()<< "Points" ;
    if(exdialog->get_Position() == true)
      out << exdialog->get_separator()<< "X_coord_pos" << exdialog->get_separator()<< "Y_coord_pos" << exdialog->get_separator()<< "Z_coord_pos";
    if(exdialog->get_Height() == true)
      out << exdialog->get_separator()<< "Height" ;
    if(exdialog->get_Length() == true)
      out << exdialog->get_separator()<< "Length" ;
    if(exdialog->get_DBH_HT() == true)
      out << exdialog->get_separator()<< "DBH_HT" << exdialog->get_separator()<< "X_coord_HT"<< exdialog->get_separator()<< "Y_coord_HT";
    if(exdialog->get_DBH_LSR() == true)
      out << exdialog->get_separator()<< "DBH_LSR" << exdialog->get_separator()<< "X_coord_LSR"<< exdialog->get_separator()<< "Y_coord_LSR";
    if(exdialog->get_areaconvex() == true)
      out << exdialog->get_separator()<< "CVex_area" ;
    if(exdialog->get_areaconcave() == true)
      out << exdialog->get_separator()<< "CCave_area" ;

    out <<"\n";

//clouds
    if(exdialog->get_treeName() == "All_trees")
    {
      for(int i = 0; i < (names.size()-1); i++ )
      {

        Tree *c = new Tree(Proj->get_TreeCloud(i));
        out << c->get_name();

        if(exdialog->get_Points() == true)
        {
          out << exdialog->get_separator() << c->get_Cloud()->points.size() ;
        }
        if(exdialog->get_Position() == true)
        {
          c->set_position();
          pcl::PointXYZI p;
          p = c->get_pose();
          out << exdialog->get_separator() << (p.x - Proj->get_Xtransform()) << exdialog->get_separator() << (p.y - Proj->get_Ytransform()) << exdialog->get_separator() << (p.z - Proj->get_Ztransform()) ;
        }
        if(exdialog->get_Height() == true)
        {
          c->set_height();
          out << exdialog->get_separator() << c->get_height();
        }
        if(exdialog->get_Length() == true)
        {
         c->set_lenght();
          out << exdialog->get_separator() << c->get_lenght();
        }
        if(exdialog->get_DBH_HT() == true)
        {
          stred s;
          c->set_dbhHT();
          s = c->get_dbhHT();
          out << exdialog->get_separator() << (s.r*2) << exdialog->get_separator() << (s.a - Proj->get_Xtransform()) << exdialog->get_separator() << (s.b - Proj->get_Ytransform()) ;
        }
        if(exdialog->get_DBH_LSR() == true)
        {
          stred s;
          c->set_dbhLSR();
          s = c->get_dbhLSR();
          out << exdialog->get_separator() << (s.r*2) << exdialog->get_separator()<< (s.a - Proj->get_Xtransform()) << exdialog->get_separator() << (s.b - Proj->get_Ytransform());
        }
        if(exdialog->get_areaconvex() == true)
        {
          c->set_convexhull();
          out << exdialog->get_separator()<< c->get_areaconvex();
        }
        if(exdialog->get_areaconcave() == true)
        {
          c->set_concavehull(1.50);
          out << exdialog->get_separator()<< c->get_areaconcave();
        }

        out <<"\n";
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Tree *c = new Tree(Proj->get_TreeCloud(exdialog->get_treeName()));
      out << c->get_name();

      if(exdialog->get_Points() == true)
      {
        out << exdialog->get_separator() << c->get_Cloud()->points.size() ;
      }
      if(exdialog->get_Position() == true)
      {
        c->set_position();
        pcl::PointXYZI p;
        p = c->get_pose();
        out << exdialog->get_separator() << (p.x - Proj->get_Xtransform()) << exdialog->get_separator() << (p.y - Proj->get_Ytransform()) << exdialog->get_separator() << (p.z - Proj->get_Ztransform()) ;
      }
      if(exdialog->get_Height() == true)
      {
        c->set_height();
        out << exdialog->get_separator() << c->get_height();
      }
      if(exdialog->get_Length() == true)
      {
       c->set_lenght();
        out << exdialog->get_separator() << c->get_lenght();
      }
      if(exdialog->get_DBH_HT() == true)
      {
        stred s;
        c->set_dbhHT();
        s = c->get_dbhHT();
        out << exdialog->get_separator() << (s.r*2) << exdialog->get_separator() << (s.a - Proj->get_Xtransform()) << exdialog->get_separator() << (s.b - Proj->get_Ytransform()) ;
      }
      if(exdialog->get_DBH_LSR() == true)
      {
        stred s;
        c->set_dbhLSR();
        s = c->get_dbhLSR();
        out << exdialog->get_separator() << (s.r*2) << exdialog->get_separator()<< (s.a - Proj->get_Xtransform()) << exdialog->get_separator() << (s.b - Proj->get_Ytransform());
      }
      if(exdialog->get_areaconvex() == true)
      {
        c->set_convexhull();
        out << exdialog->get_separator()<< c->get_areaconvex();
      }
      if(exdialog->get_areaconcave() == true)
      {
        c->set_concavehull(1.50);
        out << exdialog->get_separator()<< c->get_areaconcave();
      }
      out <<"\n";
      pBar->setValue(100);
      pBar->update();
    }
    file.close();
  }
  statusBar()->removeWidget(pBar);
}
void MainWindow::convexhull()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute concave hull of tree.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing concvex hull of given tree."
                        " This method serve only for display estimated polygon of convex hull. ");
  in->set_inputCloud1("Input Tree cloud:",names);

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    // position of progress bar should be extreme rightProj->get_TreeCloud(in->get_inputCloud1())
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 0; i < (names.size() - 1); i++ )
      {
        Proj->set_treeConvexCloud(names.at(i));
        QColor col = Proj->get_TreeCloud(i).get_color();
        dispCloud(Proj->get_TreeCloud(i).get_vexhull(),col.red(),col.green(), col.blue());
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->set_treeConvexCloud(in->get_inputCloud1());
      QColor col = Proj->get_TreeCloud(in->get_inputCloud1()).get_color();
      dispCloud(Proj->get_TreeCloud(in->get_inputCloud1()).get_vexhull(),col.red(),col.green(), col.blue());
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}
void MainWindow::concavehull()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute concave hull of tree.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing concave hull of given tree."
                        " This method serve only for display estimated polygon of concave hull. ");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_inputInt("Input Maximal Edge length cm:","150");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 0; i < (names.size() - 1); i++ )
      {
        Proj->set_treeConcaveCloud(names.at(i),(float)in->get_intValue()/100);
        QColor col = Proj->get_TreeCloud(i).get_color();
        dispCloud(Proj->get_TreeCloud(i).get_concavehull(),col.red(),col.green(), col.blue());
        //m_vis->addPolygon<pcl::PointXYZI>(c->get_concavehull().get_Cloud(), c->get_concavehull().get_name().toUtf8().constData());
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->set_treeConcaveCloud(in->get_inputCloud1(),(float)in->get_intValue()/100);
      QColor col = Proj->get_TreeCloud(in->get_inputCloud1()).get_color();
      dispCloud(Proj->get_TreeCloud(in->get_inputCloud1()).get_concavehull(),col.red(),col.green(), col.blue());
      //m_vis->addPolygon(c->get_concavehull()->get_Cloud(),c->get_concavehull()->get_name());
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}
void MainWindow::dbhHT()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute tree DBH using randomized hough transform (RHT) for circle detection.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing tree DBH (Diameter in breat height) for given tree."
                        " This method serve only for display estimated cylinder with computed centre, diameter and with height od 10 cm. ");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 0; i < (names.size() - 1); i++ )
      {
        stred x;
        Proj->get_TreeCloud(i).set_dbhHT();
        x = Proj->get_TreeCloud(i).get_dbhHT();
    // zobrazit cylinder a hodnotu

        QString cl_name = QString ("%1_dbh").arg(Proj->get_TreeCloud(i).get_name());
        Cloud *cl_ = new Cloud(Proj->get_TreeCloud(i).get_dbhCloud(),cl_name ) ;
        dispCloud(*cl_,255, 0, 0);
        //Coeff
        pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
        coef->values.push_back((float)x.a);
        coef->values.push_back((float)x.b);
        coef->values.push_back((float)x.z);
        coef->values.push_back((float)0);
        coef->values.push_back((float)0);
        coef->values.push_back((float)0.1);
        coef->values.push_back((float)x.r/100);
        std::stringstream name ;
        name << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_cylinder_HT";
        m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
        pcl::PointXYZ bod;
        bod.x=x.a+(float)x.r/100;
        bod.y=x.b;
        bod.z=x.z+0.1;
        QString h= QString("%1").arg(x.r*2.0);
        std::stringstream name2 ;
        name2 << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_value_HT";
        m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.2,0.5,0,name2.str());
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      //urcit dbh
      Proj->get_TreeCloud(in->get_inputCloud1()).set_dbhHT();
      stred x = Proj->get_TreeCloud(in->get_inputCloud1()).get_dbhHT();
    // zobrazit cylinder a hodnotu
      QString cl_name = QString ("%1_dbh").arg(Proj->get_TreeCloud(in->get_inputCloud1()).get_name());
      Cloud *cl_ = new Cloud(Proj->get_TreeCloud(in->get_inputCloud1()).get_dbhCloud(),cl_name ) ;
      dispCloud(*cl_,255, 0, 0);
        //Coeff
      pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
      coef->values.push_back((float)x.a);
      coef->values.push_back((float)x.b);
      coef->values.push_back((float)x.z);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0.1);
      coef->values.push_back((float)x.r/100);
      std::stringstream name ;
      name << Proj->get_TreeCloud(in->get_inputCloud1()).get_name().toUtf8().constData() << "_cylinder_HT";
      m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
      pcl::PointXYZ bod;
      bod.x=x.a+(float)x.r/100;
      bod.y=x.b;
      bod.z=x.z+0.1;
      QString h= QString("%1").arg(x.r*2.0);
      std::stringstream name2 ;
      name2 << Proj->get_TreeCloud(in->get_inputCloud1()).get_name().toUtf8().constData() << "_value_HT";
      m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.2,0.5,0,name2.str());
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}
void MainWindow::dbhLSR()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute tree DBH using Least square regression (LSR) for circle detection.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing tree DBH (Diameter in breat height) for given tree."
                        " This method serve only for display estimated cylinder with computed centre, diameter and with height of cylinder 10 cm ");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {

    pBar = new QProgressBar(statusBar());
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 0; i < (names.size()-1); i++ )
      {

        Tree *c = new Tree(Proj->get_TreeCloud(i));
      //DBH_Least square regression
        c->set_dbhLSR();
        float dbhLSRx = c->get_dbhLSR().a;
        float dbhLSRy = c->get_dbhLSR().b;
        float dbhLSRz = c->get_dbhLSR().z;
        float dbhLSR = c->get_dbhLSR().r;

        if(dbhLSR > 5000)
          continue;

    // zobrazit cylinder a hodnotu
        QString cl_name = QString ("%1_dbh").arg(Proj->get_TreeCloud(i).get_name());
        Cloud *cl_ = new Cloud(Proj->get_TreeCloud(i).get_dbhCloud(),cl_name ) ;
        dispCloud(*cl_,255, 0, 0);
        //Coeff
        pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
        coef->values.push_back(dbhLSRx);
        coef->values.push_back(dbhLSRy);
        coef->values.push_back(dbhLSRz);
        coef->values.push_back((float)0);
        coef->values.push_back((float)0);
        coef->values.push_back((float)0.1);
        coef->values.push_back(dbhLSR/100);
        std::stringstream name ;
        name << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_cylinder_LSR";
        m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
        pcl::PointXYZ bod;
        bod.x=dbhLSRx;
        bod.y=dbhLSRy+dbhLSR/100;
        bod.z=dbhLSRz+0.1;
        QString h= QString("%1").arg(dbhLSR*2.0);
        std::stringstream name2 ;
        name2 << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_value_LSR";
        m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0,name2.str());
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Tree *c = new Tree(Proj->get_TreeCloud(in->get_inputCloud1()));
      //DBH_Least square regression
        c->set_dbhLSR();
        float dbhLSRx = c->get_dbhLSR().a;
        float dbhLSRy = c->get_dbhLSR().b;
        float dbhLSRz = c->get_dbhLSR().z;
        float dbhLSR = c->get_dbhLSR().r;

        if(dbhLSR > 5000)
          return;
    // zobrazit cylinder a hodnotu
      QString cl_name = QString ("%1_dbh").arg(Proj->get_TreeCloud(in->get_inputCloud1()).get_name());
      Cloud *cl_ = new Cloud(Proj->get_TreeCloud(in->get_inputCloud1()).get_dbhCloud(),cl_name ) ;
      dispCloud(*cl_,255, 0, 0);
        //Coeff
      pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
      coef->values.push_back(dbhLSRx);
      coef->values.push_back(dbhLSRy);
      coef->values.push_back(dbhLSRz);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0.1);
      coef->values.push_back(dbhLSR/100);
      std::stringstream name ;
      name << Proj->get_TreeCloud(in->get_inputCloud1()).get_name().toUtf8().constData() << "_cylinder_LSR";
      m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
      pcl::PointXYZ bod;
      bod.x=dbhLSRx;
      bod.y=dbhLSRy+dbhLSR/100;
      bod.z=dbhLSRz+0.1;
      QString h= QString("%1").arg(dbhLSR*2.0);
      std::stringstream name2 ;
      name2 << Proj->get_TreeCloud(in->get_inputCloud1()).get_name().toUtf8().constData() << "_value_LSR";
      m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0,name2.str());
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}
void MainWindow::height()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute tree height.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing tree height using highest point of tree cloud and tree position."
                        " This method serve only for display estimated line connecting tree position and highest point with label of difference between point in Z axis.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 0; i < names.size()-1; i++ )
      {
        Proj->get_TreeCloud(i).set_height();


        pcl::PointXYZI maxp,minp;
        maxp.z=0;
        minp.z=60000;

        for(int j = 0; j < Proj->get_TreeCloud(i).get_Cloud()->points.size();j++)
        {
          pcl::PointXYZI bod = Proj->get_TreeCloud(i).get_Cloud()->points.at(j);

          if(bod.z > maxp.z)
            maxp = bod;

          if(bod.z < minp.z)
            minp = bod;
        }

        std::stringstream name ;
        name << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_height";
        m_vis->addLine(Proj->get_TreeCloud(i).get_pose(),maxp,name.str());
        QString h= QString("%1").arg(Proj->get_TreeCloud(i).get_height());
        m_vis->addText3D(h.toUtf8().constData(),maxp,0.6,0.6,0.5,0);



        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      if(Proj->get_sizeTerainCV() > 0)
        Proj->get_TreeCloud(in->get_inputCloud1()).set_position(Proj->get_TerrainCloud(0));

      Proj->get_TreeCloud(in->get_inputCloud1()).set_height();

      pcl::PointXYZI maxp,minp;
        maxp.z=0;
        minp.z=60000;

        for(int j = 0; j < Proj->get_TreeCloud(in->get_inputCloud1()).get_Cloud()->points.size();j++)
        {
          pcl::PointXYZI bod = Proj->get_TreeCloud(in->get_inputCloud1()).get_Cloud()->points.at(j);

          if(bod.z > maxp.z)
            maxp = bod;

          if(bod.z < minp.z)
            minp = bod;
        }
        std::stringstream name ;
        name << Proj->get_TreeCloud(in->get_inputCloud1()).get_name().toUtf8().constData() << "_height";
        m_vis->addLine(Proj->get_TreeCloud(in->get_inputCloud1()).get_pose(),maxp,name.str());
        QString h= QString("%1").arg(Proj->get_TreeCloud(in->get_inputCloud1()).get_height());
        m_vis->addText3D(h.toUtf8().constData(),maxp,0.6,0.6,0.5,0);

        pBar->setValue(100);
        pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}
void MainWindow::lenght()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute length of the cloud.");
  in->set_path(Proj->get_Path());
  in->set_description("Method compute length of the cloud. It select axis with biggest value range. On this range select extreme points and compute distance between those points. "
                        " This method serve only for display of line connecting those points and estimated value.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 0; i < names.size()-1; i++ )
      {
        Tree *c = new Tree(Proj->get_TreeCloud(i));
        c->set_lenght();

        std::stringstream name ;
        name << c->get_name().toUtf8().constData() << "_lenght";
        m_vis->addLine(c->get_lpoint(true),c->get_lpoint(false),name.str());

        QString h= QString("%1").arg(c->get_lenght());
        m_vis->addText3D(h.toUtf8().constData(),c->get_lpoint(false),0.6,0.6,0.5,0);

        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Tree *c = new Tree(Proj->get_TreeCloud(in->get_inputCloud1()));
      c->set_lenght();

      std::stringstream name ;
      name << c->get_name().toUtf8().constData() << "_lenght";
      m_vis->addLine(c->get_lpoint(true),c->get_lpoint(false),name.str());
      QString h= QString("%1").arg(c->get_lenght());
      m_vis->addText3D(h.toUtf8().constData(),c->get_lpoint(false),0.6,0.6,0.5,0);

      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}
void MainWindow::position()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute tree position using lowest points of tree cloud.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for estimating tree position. Position is computed as a median of points lying in horizontal distance up to 60 cm from lowest point in cloud."
                        " Position is displayed as a sphere with centre at position and with radius 5 cm.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 0; i < names.size()-1; i++ )
      {

        //Tree *c = new Tree(Proj->get_TreeCloud(i));

        pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
        coef->values.push_back((float)Proj->get_TreeCloud(i).get_pose().x); //x
        coef->values.push_back((float)Proj->get_TreeCloud(i).get_pose().y); //y
        coef->values.push_back((float)Proj->get_TreeCloud(i).get_pose().z); //z
        coef->values.push_back((float)0.05);   //r
        std::stringstream name;

        name << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_sphere";
        m_vis->addSphere(*coef,name.str());

        QString h= QString("%1").arg(Proj->get_TreeCloud(i).get_name());
        m_vis->addText3D(h.toUtf8().constData(),Proj->get_TreeCloud(i).get_pose(),0.5,0.6,0.5,0.3);

        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      //Tree *c = new Tree(Proj->get_TreeCloud(in->get_inputCloud1()));

      pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
      coef->values.push_back((float)Proj->get_TreeCloud(in->get_inputCloud1()).get_pose().x); //x
      coef->values.push_back((float)Proj->get_TreeCloud(in->get_inputCloud1()).get_pose().y); //y
      coef->values.push_back((float)Proj->get_TreeCloud(in->get_inputCloud1()).get_pose().z); //z
      coef->values.push_back((float)0.05);   //r
      std::stringstream name;

      name << Proj->get_TreeCloud(in->get_inputCloud1()).get_name().toUtf8().constData() << "_sphere";
      m_vis->addSphere(*coef,name.str());
      QString h= QString("%1").arg(Proj->get_TreeCloud(in->get_inputCloud1()).get_name());
        m_vis->addText3D(h.toUtf8().constData(),Proj->get_TreeCloud(in->get_inputCloud1()).get_pose(),0.5,0.6,0.5,0.3);
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}
void MainWindow::skeleton()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Tree skeleton");
  in->set_path(Proj->get_Path());
  in->set_description("Manual editing of point cloud representing single tree. For editing please press key 'x' and outline rectangle."
                        " Selected points will be deleted. After selection you can choose if you want edit more or you want to close it.");
  in->set_inputCloud1("Input Tree cloud:",names);
  //in->set_inputInt("skeleton radius in cm:", "20");
 // in->set_outputCloud1("Output cloud of deleted points:","tree-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 0; i < names.size()-1; i++ )
      {

        pcl::PointXYZI minp,maxp;
        pcl::getMinMax3D(*Proj->get_TreeCloud(i).get_Cloud(),minp,maxp);
        float dbb = sqrt((minp.x - maxp.x)*(minp.x - maxp.x) + (minp.y - maxp.y)*(minp.y - maxp.y) + (minp.z - maxp.z)*(minp.z - maxp.z));
        float q = pow(Proj->get_TreeCloud(i).get_Cloud()->points.size(),1/3) ;
        float h0=  dbb/q;
        float rad = h0/100;
        float rad0 = rad/2;

        Skeleton *skel = new Skeleton(Proj->get_TreeCloud(i).get_Cloud(),rad);
    //skel->graph();

        for(int j=0; j< 2; j++)
        {
          for(int i=0; i < 10; i++)
          {
            skel->averageTerm();
            skel->repulsionTerm();

            skel->iterate();
            float movement =skel->get_movement_error()*1000;
            if(movement < 0.00001 && i > 5)
              break;
          }
          skel->branching();
          rad +=rad0;
        }
        QString n = Proj->get_TreeCloud(i).get_name().remove(".pcd");
        QString name = QString("%1_skeleton.pcd").arg(n);
        Cloud *cll = new Cloud(skel->get_skeleton(),name);
        saveOstCloud(cll);
        skel->graph();

      // save_skel
        QString newFile = QString("%1\\%2.skel").arg(Proj->get_Path()).arg(n);
        QFile file (newFile);
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);
        out.setRealNumberNotation(QTextStream::FixedNotation);
        out.setRealNumberPrecision(3);
        for(int i = 0; i < skel->get_start()->points.size(); i++)
        {
          pcl::PointXYZI it;
          it = skel->get_start()->points.at(i);

          double x_start = it.x - Proj->get_Xtransform();
          double y_start = it.y - Proj->get_Ytransform();
          double z_start = it.z - Proj->get_Ztransform();

          pcl::PointXYZI it2;
            it2 = skel->get_stop()->points.at(i);
          double x_stop = it2.x - Proj->get_Xtransform();
          double y_stop = it2.y - Proj->get_Ytransform();
          double z_stop = it2.z - Proj->get_Ztransform();

          out << x_start << " " << y_start << " " << z_start << " " << x_stop << " " << y_stop << " " << z_stop<< "\n";
        }
        file.close();

        for(int k = 0; k < skel->get_start()->points.size(); k++)
        {
          std::stringstream namea ;
          namea << name.toUtf8().constData()<< k << "_linie";
          m_vis->addLine(skel->get_start()->points.at(k),skel->get_stop()->points.at(k),namea.str());
        }
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
    //  //spocitat h0 =get minmax
      pcl::PointXYZI minp,maxp;
      pcl::getMinMax3D(*Proj->get_TreeCloud(in->get_inputCloud1()).get_Cloud(),minp,maxp);
      float dbb = sqrt((minp.x - maxp.x)*(minp.x - maxp.x) + (minp.y - maxp.y)*(minp.y - maxp.y) + (minp.z - maxp.z)*(minp.z - maxp.z));
      float q = pow(Proj->get_TreeCloud(in->get_inputCloud1()).get_Cloud()->points.size(),1/3) ;
      float h0= 2* dbb/q;
      float rad = h0/100;
      float rad0 = rad/2;

      Skeleton *skel = new Skeleton(Proj->get_TreeCloud(in->get_inputCloud1()).get_Cloud(),rad);
    //skel->graph();

      for(int j=0; j< 2; j++)
      {
        for(int i=0; i < 10; i++)
        {
          skel->averageTerm();
          skel->repulsionTerm();

          skel->iterate();
          float movement =skel->get_movement_error()*1000;

          if(movement < 0.00001 && i > 5)
            break;
          //pBar->setValue(100*i/(j*i));
        }
        skel->branching();
        rad +=rad0;
      }
      QString n = in->get_inputCloud1().remove(".pcd");
      QString name = QString("%1_skeleton.pcd").arg(n);
      Cloud *cll = new Cloud(skel->get_skeleton(),name);
      saveOstCloud(cll);
      skel->graph();

    // save_skel
    QString newFile = QString("%1\\%2.skel").arg(Proj->get_Path()).arg(n);
      QFile file (newFile);
      file.open(QIODevice::WriteOnly | QIODevice::Text);
      QTextStream out(&file);
      out.setRealNumberNotation(QTextStream::FixedNotation);
      out.setRealNumberPrecision(3);
      for(int i = 0; i < skel->get_start()->points.size(); i++)
      {
        pcl::PointXYZI it;
        it = skel->get_start()->points.at(i);

        double x_start = it.x - Proj->get_Xtransform();
        double y_start = it.y - Proj->get_Ytransform();
        double z_start = it.z - Proj->get_Ztransform();

        pcl::PointXYZI it2;
        it2 = skel->get_stop()->points.at(i);
        double x_stop = it2.x - Proj->get_Xtransform();
        double y_stop = it2.y - Proj->get_Ytransform();
        double z_stop = it2.z - Proj->get_Ztransform();

        out << x_start << " " << y_start << " " << z_start << " " << x_stop << " " << y_stop << " " << z_stop<< "\n";
      }
      file.close();



    // draw lines
      for(int k = 0; k < skel->get_start()->points.size(); k++)
      {
        std::stringstream namea ;
        namea << name.toUtf8().constData()<< k << "_linie";
        m_vis->addLine(skel->get_start()->points.at(k),skel->get_stop()->points.at(k),namea.str());
      }




      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
}

void MainWindow::saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud)
{
  QString name = QInputDialog::getText(this, tr("Name of new tree File"),tr("e.g. strom"));

  if(name.isEmpty())
  {
    QMessageBox::warning(this,tr("error"),tr("You forgot to fill name of new tree cloud"));
    saveTreeCloud(tree_cloud);
  }
  else
  {
    QString name2 =QString("%1.pcd").arg(name);
    QString path = QString("%1\\%2").arg(Proj->get_Path()).arg(name2);
    QFile file(path);
    if(file.exists())
    {
      //do you wish to rewrite existing file?
      QMessageBox::StandardButton rewrite = QMessageBox::question(this,tr("Overwrite file?"),tr("File with given name exist. Do you wish to overwrite file?"),QMessageBox::Yes|QMessageBox::No);
      if(rewrite == QMessageBox::Yes)
      {
        Cloud *c = new Cloud(tree_cloud,name);
        saveTreeCloud(c);
      }
      else
      {
        saveTreeCloud(tree_cloud);
      }
    }
    else
    {
      Cloud *c = new Cloud(tree_cloud,name);
      saveTreeCloud(c);
    }
  }
}
void MainWindow::saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud, QString name, bool overwrt = false)
{
  Cloud *c = new Cloud(tree_cloud,name);
  Proj->set_OstCloud(*c);
  QString pathT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name);
  QFile file(pathT);
  if(overwrt== true)
  {
    Proj->save_Cloud(name,tree_cloud);
    //openTreeFile(pathT);
  }
  else
  {
    if(file.exists())
    {
    //do you wish to rewrite existing file?
      QMessageBox::StandardButton rewrite = QMessageBox::question(this,tr("overwrite file?"),tr("File with given name exist. Do you wish to overwrite file?"),QMessageBox::Yes|QMessageBox::No);
      if(rewrite == QMessageBox::Yes)
      {
        Proj->save_newCloud("strom",name,tree_cloud);
        openTreeFile(pathT);
      }
      else
      {
        saveTreeCloud(tree_cloud);
      }
    }
    else
    {
      Proj->save_newCloud("strom",name,tree_cloud);
      openTreeFile(pathT);
    }
  }
}
void MainWindow::saveTreeCloud(Cloud *s_cloud)
{
  if(s_cloud->get_Cloud()->points.size() > 1)
  {
    Proj->save_newCloud("strom",s_cloud->get_name(),s_cloud->get_Cloud());
    QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
    openTreeFile(path);
  }
}
void MainWindow::saveOstCloud(Cloud *s_cloud)
{
  if(s_cloud->get_Cloud()->points.size() > 1)
  {
    Proj->save_newCloud("ost",s_cloud->get_name(),s_cloud->get_Cloud());
    QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
    openOstFile(path);
  }
}
void MainWindow::saveVegeCloud(Cloud *s_cloud)
{
  if(s_cloud->get_Cloud()->points.size() > 1)
  {
    Proj->save_newCloud("vege",s_cloud->get_name(),s_cloud->get_Cloud());
    QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
    openVegeFile(path);
  }
}
void MainWindow::saveVegeCloud(Cloud *s_cloud, bool overwrt = true)
{
  if(s_cloud->get_Cloud()->points.size() > 1)
  {
    QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
    QFile file(path);
    if(overwrt == true && file.exists())
      Proj->save_Cloud(s_cloud->get_name(),s_cloud->get_Cloud());
    if(overwrt == false && !file.exists())
    {
       Proj->save_newCloud("vege",s_cloud->get_name(),s_cloud->get_Cloud());
      QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
      openVegeFile(path);
    }
  }
}
void MainWindow::saveTerrainCloud(Cloud *s_cloud)
{
  if(s_cloud->get_Cloud()->points.size() > 1)
  {
    Proj->save_newCloud("teren",s_cloud->get_name(),s_cloud->get_Cloud());
    QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
    openTerrainFile(path);
  }
}

void MainWindow::octree(float res, pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output_vege, pcl::PointCloud<pcl::PointXYZI>::Ptr output_ground)
{

//nastaveni zakladnich promenych -rozliseni

  pcl::octree::OctreePointCloud<pcl::PointXYZI> octreeA  (res);
  std::vector<pcl::PointXYZI> voxel;
  pcl::IndicesPtr indexVector (new vector<int>);
  pcl::IndicesPtr indices (new vector<int>);
  pcl::ExtractIndices<pcl::PointXYZI> ex;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_zbytek (new pcl::PointCloud<pcl::PointXYZI>);

  //nastaveni octree a pridani mracna
  octreeA.setInputCloud (input);
  octreeA.addPointsFromInputCloud ();

  //nastavit pole s hodnotou nejnizsiho octreeKey a druheho nejnisiho
  int (*pole)[20000] = new int[20000][20000];// max pocet leafs
  for (int a=0; a<20000;a++)
  {
    for (int b=0; b<20000;b++)
    {
      pole[a][b]=20000;
    }
  }
//hledani nejnizsiho octreeeKey a pak ho zapsat do pole
  pcl::octree::OctreePointCloud<pcl::PointXYZI>::LeafNodeIterator poi;
  poi = octreeA.leaf_begin();
  while (*++poi)
  {
    pcl::octree::OctreeKey *ii = new pcl::octree::OctreeKey (poi.getCurrentOctreeKey());
    if(pole[ii->x][ii->y] > ii->z)
    {
      pole[ii->x][ii->y] = ii->z;
    }
  }
  // zapsat data ze ziskanych leafs
  pcl::octree::OctreePointCloud<pcl::PointXYZI>::LeafNodeIterator poi3;
  poi3 = octreeA.leaf_begin();
  for (poi3; poi3 != octreeA.leaf_end(); ++poi3)
  {
    pcl::octree::OctreeKey *ee = new pcl::octree::OctreeKey (poi3.getCurrentOctreeKey());
 // pokud jsou octrekeys tohoto leaf ty nejspodnejsi z pole nebo o jedno vyssi dostat body ven
    if ((pole[ee->x][ee->y] == ee->z) || (pole[ee->x][ee->y] == (ee->z-1)))
    {
      if(!indexVector->empty())
      {
        indexVector->clear();
      }
      //dostat data pro dany octreekey a jeho leaf
      pcl::octree::OctreeContainerPointIndices *b = new pcl::octree::OctreeContainerPointIndices (poi3.getLeafContainer());
      b->getPointIndices(*indexVector);
      delete b;

      if(!indexVector->empty())
      {
        indices->insert(indices->end(),indexVector->begin(),indexVector->end());
        indexVector->clear();
      }
    }
  }
//extracton of points
  ex.setInputCloud(input);
  ex.setIndices (indices);

//ulozeni bodu ktere nejsou surface
  ex.setNegative(true);
  ex.filter (*output_vege);

//ulozeni spodnich bodu do surface
  ex.setNegative(false);
  ex.filter (*output_ground);

  delete pole;
}
//MISCELLANEOUS
void MainWindow::plusCloud()
{
  QStringList types;
  types << "Base cloud" << "Terrain cloud" << "Vegetation cloud" <<"Tree" << "Other";

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud join");
  in->set_path(Proj->get_Path());
  in->set_description("Join of two clouds which create  new third one.");
  in->set_inputCloud1("1. input cloud:",get_allNames());
  in->set_inputCloud2("2. input cloud:",get_allNames());
  in->set_outputCloud1("Output cloud :","cloud");
  in->set_outputType("Type of the output cloud:", types);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    QString type;
    if(in->get_outputType() == "Base cloud")
      type = "cloud";
    if(in->get_outputType() == "Terrain cloud")
      type = "teren";
    if(in->get_outputType() == "Vegetation cloud")
      type = "vege";
    if(in->get_outputType() == "Tree")
      type = "strom";
    if(in->get_outputType() == "Other")
      type = "ost";

    plusCloud(in->get_inputCloud1(),in->get_inputCloud2(),in->get_outputCloud1(), type );
  //pridat do stromu
    QString filen= QString ("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud1());
    openOstFile(filen);
  }
}
void MainWindow::plusCloud(QString input1, QString input2,QString output, QString typ = "ost")
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cl (new pcl::PointCloud<pcl::PointXYZI>);
  *cl = *Proj->get_Cloud(input1).get_Cloud() + *Proj->get_Cloud(input2).get_Cloud();
  Proj->save_newCloud(typ,output,cl);
}

void MainWindow::minusCloud()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud join");
  in->set_path(Proj->get_Path());
  in->set_description("Remove same points from bigger cloud. The function compare two input clouds. From the bigger one remove all common points and save the rest into a new file. ");
  in->set_inputCloud1("1. input cloud:",get_allNames());
  in->set_inputCloud2("2. input cloud:",get_allNames());
  in->set_outputCloud1("Output cloud:","cloud_minus");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    //porovnat velikost
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_1 = Proj->get_Cloud(in->get_inputCloud1()).get_Cloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_2 = Proj->get_Cloud(in->get_inputCloud2()).get_Cloud();

    Cloud *c1 = new Cloud();
    Cloud *c2 = new Cloud();

    if(cloud_1->points.size() > cloud_2->points.size())
    {
      c1->set_Cloud(cloud_1);
      c1->set_name(in->get_inputCloud1());
      c2->set_Cloud(cloud_2);
      c2->set_name(in->get_inputCloud2());
    }
    else
    {
      c1->set_Cloud(cloud_2);
      c1->set_name(in->get_inputCloud2());
      c2->set_Cloud(cloud_1);
      c2->set_name(in->get_inputCloud1());
    }
    // pro kazdy bod v cloud1
    // pokud neni jejich vzdalenost mensi nez 0,001
    //ulozit do noveho cloudu
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (c2->get_Cloud());

    for(int i=0; i < c1->get_Cloud()->points.size();i++)
    {
      pcl::PointXYZI searchPointV;
      searchPointV=c1->get_Cloud()->points.at(i);
      std::vector<int> pointIDv;
      std::vector<float> pointSDv;

      if(kdtree.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) > 0)
      {
        continue;
      }
      cloud_out->points.push_back(searchPointV);
    }
    cloud_out->width = cloud_out->points.size ();
    cloud_out->is_dense=true;
    cloud_out->height=1;

    Cloud *cV = new Cloud(cloud_out,in->get_outputCloud1());
    saveOstCloud(cV);
  }
}
void MainWindow::voxelize()
{

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud join");
  in->set_path(Proj->get_Path());
  in->set_description("Make voxelized cloud. Resulting cloud has point only in centroids of boxex with resolution where was present at least one point. ");
  in->set_inputCloud1("Input cloud:",get_allNames());
  in->set_outputCloud1("Output cloud name:","voxel");
  in->set_inputInt("Resolution in cm:","10");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    float res =in->get_intValue()/100.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vox;

    vox.setInputCloud (Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    vox.setLeafSize (res, res, res);
    vox.filter (*cloud_filtered);

    Proj->save_newCloud("ost",in->get_outputCloud1(),cloud_filtered);
    QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud1());
    openOstFile(fullnameV);
  }
}

QString MainWindow::get_path()
{
  return Proj->get_Path();
}
void MainWindow::IDW()
{
   //inputDialog
  InputDialog *in = new InputDialog(this);
  in->set_title("Terrain voxel");
  in->set_path(Proj->get_Path());
  in->set_description("Special function only for aligment of shifted data.");
  in->set_inputCloud1("Input reference cloud:",get_allNames());
  in->set_inputCloud2("Input ground cloud:",get_allNames());
  in->set_inputCloud3("Input original cloud:",get_allNames());
  in->set_outputCloud1("Output cloud of ground:","idw-");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
  { return; }

  if(dl == QDialog::Accepted)
  {
      // vypocet vyskoveho rozdilu jeho ulozeni do intensity
      //pro kazdy bod v ground_points najdi nejblissi bod v reference_cloud uloz ho do cloud_vyr
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vyr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_idw (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtreee;
    kdtreee.setInputCloud (Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());

    for(int i=0; i < Proj->get_Cloud(in->get_inputCloud2()).get_Cloud()->points.size();i++)
    {
      std::vector<int> pointIDv;
      std::vector<float> pointSDv;
      pcl::PointXYZI searchPointV;
      searchPointV=Proj->get_Cloud(in->get_inputCloud2()).get_Cloud()->points.at(i);

      if(kdtreee.nearestKSearch(searchPointV,1,pointIDv,pointSDv) >0)
      {
        float z = Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at( pointIDv.at(0) ).z;

        pcl::PointXYZI in;
        in.x = searchPointV.x;
        in.y = searchPointV.y;
        in.z = searchPointV.z;
        in.intensity = searchPointV.z - z;
        //ulozit jako novy cloud
        //#pragma omp critical
        cloud_vyr->points.push_back(in);
      }
    }
    cloud_vyr->width = cloud_vyr->points.size ();
    cloud_vyr->is_dense=true;
    cloud_vyr->height=1;

// tady začíná IDW
    // pro kazdy bod v m_cloud najdi 10 nejblizsich bodu z cloud_vyr:
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    std::vector<int> pointIv;
    std::vector<float> pointSv;
    pcl::PointXYZI searchPointVV;
    kdtree.setInputCloud (cloud_vyr);
    //#pragma omp parallel for
    for(int j=0; j < Proj->get_Cloud(in->get_inputCloud3()).get_Cloud()->points.size();j++)
    {
      searchPointVV=Proj->get_Cloud(in->get_inputCloud3()).get_Cloud()->points.at(j);
      if(kdtree.nearestKSearch(searchPointVV,10,pointIv,pointSv) >0)
      {
        float max_dist = 0;
        float w_sum = 0;
        float idw_sum = 0;
        //max distance
        for(int k =0; k < pointSv.size();k++)
        {
          if(sqrt(pointSv.at(k)) > max_dist)
            max_dist = sqrt(pointSv.at(k));
        }
      //w_sum
        for(int l =0; l < pointSv.size();l++)
        {
          float dist = sqrt(pointSv.at(l));
          float w;
          if(dist == 0)
            { w=0.0000000000001;}
          else
            { w = pow((max_dist - dist)/(max_dist*dist),2);}
          w_sum += w;
        }
      //idw_sum
        for(int m =0; m < pointSv.size();m++)
        {
          float dist = sqrt(pointSv.at(m));
          float w;
          if(dist == 0)
            { w=0.0000000000001;}
          else
            {w = pow((max_dist - dist)/(max_dist*dist),2)/w_sum;}
          float f =   cloud_vyr->points.at(pointIv.at(m)).intensity;
          float idw = f*w;
          idw_sum+=idw;
        }
      pcl::PointXYZI bod;
      bod.x= searchPointVV.x;
      bod.y= searchPointVV.y;
      bod.z= searchPointVV.z - idw_sum;
      //#pragma omp critical
      cloud_idw->points.push_back(bod);
      }
    }
    cloud_idw->width = cloud_idw->points.size ();
    cloud_idw->is_dense=true;
    cloud_idw->height=1;

    // export do pcd
    Proj->save_newCloud("ost",in->get_outputCloud1(),cloud_idw);
    QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud1());
    openVegeFile(fullnameV);
  }
}
void MainWindow::clip()
{
//inputDialog
  InputDialog *in = new InputDialog(this);
  in->set_title("Clip");
  in->set_path(Proj->get_Path());
  in->set_description("Special function only for selectoin of ground points.");
  in->set_inputCloud1("Input reference cloud:",get_allNames());
  in->set_inputCloud2("Input original cloud:",get_allNames());
  in->set_inputInt("set width of strip:","100");
  in->set_outputCloud1("Output cloud of ground:","cliped-ground");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
  { return; }

  if(dl == QDialog::Accepted)
  {
    m_width=0;
    m_res = in->get_intValue();
    //  orginal
    m_cloud->get_Cloud()->clear();
    m_cloud->set_name(in->get_inputCloud1());
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    // reference
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_inputCloud2());
    m_cloud1->set_Cloud(Proj->get_Cloud(in->get_inputCloud2()).get_Cloud());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud2()).get_Psize());

    //ground
    m_cloud2 = new Cloud();
    m_cloud2->get_Cloud()->clear();
    m_cloud2->set_name(in->get_outputCloud1());
    m_cloud2->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());


    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent2, *this );

    //spustit editacni listu
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction("Stop EDIT");
    connect(stopTE,SIGNAL(triggered()),this,SLOT(clipStop()) );
    //QAction *undoAct = editBar->addAction("undo");
    //connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    clipAct = new QAction("clip",this);
    editBar->addAction(clipAct);
    connect(clipAct,SIGNAL(triggered()),this,SLOT(clipped()) );
    undopoint.clear();

    treeWidget->allItemOFF();
    m_vis->removeAllPointClouds();

    clip(Proj->get_Cloud(m_cloud->get_name()),Proj->get_Cloud(m_cloud1->get_name()), m_res);//nastavi pasy
  }
}

void MainWindow::clip(Cloud cl, Cloud cl2, int res)
{
//vyčistit m_cloud a m_cloud2
  m_cloud->get_Cloud()->points.clear(); //  original
  m_cloud1->get_Cloud()->points.clear(); //reference


//zjistit min a max  original cloudu
  pcl::PointXYZI min3D,max3D;
	pcl::getMinMax3D(*cl2.get_Cloud(),min3D,max3D);

  //original cloud strip
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
  for(int i=0; i < cl2.get_Cloud()->points.size();i++)
  {
    if(cl2.get_Cloud()->points.at(i).x > (min3D.x +m_width)&& cl2.get_Cloud()->points.at(i).x < (min3D.x +m_width + res) && cl2.get_Cloud()->points.at(i).y > (min3D.y -10) &&  cl2.get_Cloud()->points.at(i).y < (max3D.y +10))
      cloud1->points.push_back(cl2.get_Cloud()->points.at(i));
  }
  m_cloud->set_Cloud(cloud1);

  //reference terain strip
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
  for(int i=0; i < cl.get_Cloud()->points.size();i++)
  {
    if(cl.get_Cloud()->points.at(i).x > (min3D.x +m_width) && cl.get_Cloud()->points.at(i).x < (min3D.x +m_width + res))
      cloud2->points.push_back(cl.get_Cloud()->points.at(i));
  }
  m_cloud1->set_Cloud(cloud2);


  //dipslay clouds
  m_vis->removeAllPointClouds();
  dispCloud(*m_cloud1,0,220,0); //reference
  dispCloud(*m_cloud,220,220,0); //original

  //m_vis->resetCamera();
  qvtkwidget->update();

  m_width = m_width + res;
  //kontrola posledniho pasu
  if ((min3D.x +m_width)> max3D.x)
  {
    QMessageBox::information(this,"Warning","last strip");
    disconnect(clipAct,SIGNAL(triggered()),this,SLOT(clipped()) );
    editBar->removeAction(clipAct);
  }
}
void MainWindow::clipped()
{
  *m_cloud2->get_Cloud() += *m_cloud->get_Cloud();
  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  clip(Proj->get_Cloud(m_cloud->get_name()),Proj->get_Cloud(m_cloud1->get_name()), m_res);//nastavi pasy
}
void MainWindow::clipStop()
{
  *m_cloud2->get_Cloud() += *m_cloud->get_Cloud();
  m_vis->removeAllPointClouds();
  if(m_cloud2->get_Cloud()->points.size() >1)
  {
  // ulozit m_cloud2 do vystupniho souboru
    Proj->save_newCloud("ost",m_cloud2->get_name(),m_cloud2->get_Cloud());
    QString fullnameT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(m_cloud2->get_name());
    openCloudFile(fullnameT);
  }
  area.disconnect();
  delete editBar;
  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  m_cloud2->get_Cloud()->clear();
  undopoint.clear();
}
//ACTION and MENUS
void MainWindow::createActions()
{
//FILE
  new_projectAct = new QAction( tr("&New Project"), this);
  new_projectAct->setStatusTip(tr("Create new project"));
  connect(new_projectAct, SIGNAL(triggered()), this, SLOT(newProject()));

  open_projectAct = new QAction( tr("&Open Project"), this);
  open_projectAct->setStatusTip(tr("Open project"));
  connect(open_projectAct, SIGNAL(triggered()), this, SLOT(openProject()));

  close_projectAct = new QAction( tr("&Close Project"), this);
  close_projectAct->setStatusTip(tr("close project"));
  connect(close_projectAct, SIGNAL(triggered()), this, SLOT(closeProject()));

  import_projectAct = new QAction( tr("&Import project"), this);
  import_projectAct->setStatusTip(tr("import of existing project or create new"));
  connect(import_projectAct, SIGNAL(triggered()), this, SLOT(importProject()));

  importTXTAct = new QAction( tr("&Import TXT, XYZ"), this);
  importTXTAct->setStatusTip(tr("Import of txt file"));
  connect(importTXTAct, SIGNAL(triggered()), this, SLOT(importtxt()));

  importPTSAct = new QAction( tr("&Import PTS"), this);
  importPTSAct->setStatusTip(tr("Import of pts file"));
  connect(importPTSAct, SIGNAL(triggered()), this, SLOT(importpts()));

  importPTXAct = new QAction( tr("&Import PTX"), this);
  importPTXAct->setStatusTip(tr("Import of ptx file"));
  connect(importPTXAct, SIGNAL(triggered()), this, SLOT(importptx()));

  importLASAct = new QAction( tr("&Import LAS"), this);
  importLASAct->setStatusTip(tr("Import of las file"));
  connect(importLASAct, SIGNAL(triggered()), this, SLOT(importlas()));

  importPCDAct = new QAction(tr("&Import basic cloud (PCD)"), this);
  importPCDAct->setStatusTip(tr("Open an existing file"));
  connect(importPCDAct, SIGNAL(triggered()), this, SLOT(importCloud()));

  importTerenAct = new QAction(tr("&Import Terrain file (PCD)"), this);
  importTerenAct->setStatusTip(tr("Open an existing terrain file"));
  connect(importTerenAct, SIGNAL(triggered()), this, SLOT(importTerrainFile()));

  importVegeAct = new QAction(tr("&Import Vegetation file (PCD)"), this);
  importVegeAct->setStatusTip(tr("Open an existing vegetation file"));
  connect(importVegeAct, SIGNAL(triggered()), this, SLOT(importVegeCloud()));

  importTreeAct = new QAction(tr("&Import Tree file (PCD)"), this);
  importTreeAct->setStatusTip(tr("Open an existing tree file and import into project"));
  connect(importTreeAct, SIGNAL(triggered()), this, SLOT(importTreeCloud()));

  exportTXTAct = new QAction(tr("&Export file (txt)"), this);
  exportTXTAct->setStatusTip(tr("Export cloud into txt file"));
  connect(exportTXTAct, SIGNAL(triggered()), this, SLOT(exportCloud()));

  exportPLYAct = new QAction(tr("&Export file (ply)"), this);
  exportPLYAct->setStatusTip(tr("Export cloud into ply file"));
  connect(exportPLYAct, SIGNAL(triggered()), this, SLOT(plysave()));

  exportPTSAct = new QAction(tr("&Export file (pts)"), this);
  exportPTSAct->setStatusTip(tr("Export cloud into pts file"));
  connect(exportPTSAct, SIGNAL(triggered()), this, SLOT(exportPts()));

  exportCONVEXAct = new QAction(tr("&Export convex contours (txt)"), this);
  exportPTSAct->setStatusTip(tr("Export cloud into txt file"));
  connect(exportCONVEXAct, SIGNAL(triggered()), this, SLOT(exportConvexTxt()));

  exportCONCAVEAct = new QAction(tr("&Export concave contours (txt)"), this);
  exportCONCAVEAct->setStatusTip(tr("Export cloud into txt file"));
  connect(exportCONCAVEAct, SIGNAL(triggered()), this, SLOT(exportConcaveTxt()));

  exitAct = new QAction(tr("E&xit"), this);
  exitAct->setShortcuts(QKeySequence::Quit);
  exitAct->setStatusTip(tr("Exit the application"));
  connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

//TERRAIN
  voxelAct = new QAction(tr("Terrain voxel"), this);
  voxelAct->setStatusTip(tr("voxel method with variable size of voxel"));
 // voxelAct->setEnabled(false);
  connect(voxelAct, SIGNAL(triggered()), this, SLOT(voxelgrid()));

  octreeAct = new QAction(tr("Terrain octree"), this);
  octreeAct->setStatusTip(tr("compute surface point based on octree structure"));
  //octreeAct->setEnabled(false);
  connect(octreeAct, SIGNAL(triggered()), this, SLOT(octreeSlot()));

  manualADAct = new QAction(tr("Manual adjustment"), this);
  manualADAct->setStatusTip(tr("manual adjustment of terrain cloud"));
 // manualADAct->setEnabled(false);
  connect(manualADAct, SIGNAL(triggered()), this, SLOT(manualAdjust()));

//VEGETATION

  manualSelAct = new QAction(tr("Manual selection"), this);
  manualSelAct->setStatusTip(tr("Display best fitted cylinder"));
  connect(manualSelAct, SIGNAL(triggered()), this, SLOT(manualSelect()));


//TREE ATRIBUTES
  tAAct = new QAction(tr("Tree atributes into file"), this);
  tAAct->setStatusTip(tr("Compute atributes for all trees"));
  connect(tAAct, SIGNAL(triggered()), this, SLOT(treeAtributes()));

  dbhHTAct = new QAction(tr("DBH HT"), this);
  dbhHTAct->setStatusTip(tr("compute cylinder using hough transform"));
  //dbhAct->setEnabled(false);
  connect(dbhHTAct, SIGNAL(triggered()), this, SLOT(dbhHT()));

  dbhLSRAct = new QAction(tr("DBH LSR"), this);
  dbhLSRAct->setStatusTip(tr("display best fitted cylinder"));
  //dbhAct->setEnabled(false);
  connect(dbhLSRAct, SIGNAL(triggered()), this, SLOT(dbhLSR()));

  heightAct = new QAction(tr("Height"), this);
  heightAct->setStatusTip(tr("compute height of the tree"));
 // heightAct->setEnabled(false);
  connect(heightAct, SIGNAL(triggered()), this, SLOT(height()));

  posAct = new QAction(tr("Tree position"), this);
  posAct->setStatusTip(tr("display sphere in tree position"));
  connect(posAct, SIGNAL(triggered()), this, SLOT(position()));

  treeEditAct = new QAction(tr("Tree cloud edit"), this);
  treeEditAct->setStatusTip(tr("edit selected tree cloud"));
  connect(treeEditAct, SIGNAL(triggered()), this, SLOT(treeEdit()));

  dbhEditAct = new QAction(tr("Tree DBHcloud edit"), this);
  dbhEditAct->setStatusTip(tr("edit selected tree cloud"));
  connect(dbhEditAct, SIGNAL(triggered()), this, SLOT(dbhCloudEdit()));

  lengAct = new QAction(tr("Cloud lenght"), this);
  lengAct->setStatusTip(tr("display cloud lenght"));
  connect(lengAct, SIGNAL(triggered()), this, SLOT(lenght()));

  skeletonAct = new QAction(tr("Skeleton"), this);
  skeletonAct->setStatusTip(tr("create skeleton cloud"));
  connect(skeletonAct, SIGNAL(triggered()), this, SLOT(skeleton()));

  convexAct = new QAction(tr("Convex hull"), this);
  convexAct->setStatusTip(tr("display boardes"));
  connect(convexAct, SIGNAL(triggered()), this, SLOT(convexhull()));

  concaveAct = new QAction(tr("Concave hull"), this);
  concaveAct->setStatusTip(tr("display boardes"));
  connect(concaveAct, SIGNAL(triggered()), this, SLOT(concavehull()));

//MISC
  plusAct = new QAction(tr("Cloud contencate"), this);
  plusAct->setStatusTip(tr("cloud + cloud"));
  connect(plusAct, SIGNAL(triggered()), this, SLOT(plusCloud()));

  voxAct = new QAction(tr("Voxelize cloud"), this);
  voxAct->setStatusTip(tr("voxels from cloud"));
  connect(voxAct, SIGNAL(triggered()), this, SLOT(voxelize()));

  IDWAct = new QAction(tr("IDW "), this);
  IDWAct->setStatusTip(tr("IDW of clouds. Only for special purpose"));
  connect(IDWAct, SIGNAL(triggered()), this, SLOT(IDW()));

  clipedAct = new QAction(tr("clip "), this);
  clipedAct->setStatusTip(tr("cliped clouds. Only for special purpose"));
  connect(clipedAct, SIGNAL(triggered()), this, SLOT(clip()));

  minusAct = new QAction(tr("Cloud Subtraction"), this);
  minusAct->setStatusTip(tr("delete common points from bigger cloud"));
  connect(minusAct, SIGNAL(triggered()), this, SLOT(minusCloud()));


//ABOUT
  aboutAct = new QAction(tr("&About"), this);
  aboutAct->setStatusTip(tr("Show the application's About box"));
  connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));


//EVENTS treewidget
  connect(treeWidget, SIGNAL(checkedON(QString)), this, SLOT(removeCloud(QString)));
  connect(treeWidget, SIGNAL(checkedOFF(QString)), this, SLOT(dispCloud(QString)));
  connect(treeWidget, SIGNAL(deleteItem(QString)), this, SLOT(deleteCloud(QString)));
  connect(treeWidget, SIGNAL(colorItem(QString)), this, SLOT(colorCloud(QString)));
  connect(treeWidget, SIGNAL(colorItemField(QString)), this, SLOT(colorCloudField(QString)));
  connect(treeWidget, SIGNAL(psize(QString)), this, SLOT(PointSize(QString)));
}
void MainWindow::createMenus()
{
//PROJECT
  fileMenu = menuBar()->addMenu(tr("&Project"));
  fileMenu->addAction(new_projectAct);
  fileMenu->addAction(open_projectAct);
  fileMenu->addAction(close_projectAct);
  fileMenu->addAction(import_projectAct);
  fileMenu->addSeparator();
  importMenu =  fileMenu->addMenu(tr("Import"));
  importMenu->addAction(importTXTAct);
  importMenu->addAction(importLASAct);
  importMenu->addAction(importPTSAct);
  importMenu->addAction(importPTXAct);
  importMenu->addAction(importPCDAct);
  importMenu->addAction(importTerenAct);
  importMenu->addAction(importVegeAct);
  importMenu->addAction(importTreeAct);
  exportMenu = fileMenu->addMenu(tr("Export"));
  exportMenu->addAction(exportTXTAct);
  exportMenu->addAction(exportPLYAct);
  exportMenu->addAction(exportPTSAct);
  exportMenu->addAction(exportCONVEXAct);
  exportMenu->addAction(exportCONCAVEAct);
  fileMenu->addSeparator();
  fileMenu->addAction(exitAct);

//TEREN
  terenMenu = menuBar()->addMenu(tr("&Terrain"));
  terenMenu->addAction(voxelAct);
  terenMenu->addAction(octreeAct);
  terenMenu->addSeparator();
  terenMenu->addAction(manualADAct);

//VEGETATION
  vegeMenu = menuBar()->addMenu(tr("&Vegetation"));
  vegeMenu->addAction(manualSelAct);

//TREE ATRIBUTES
  treeMenu = menuBar()->addMenu(tr("&Trees"));
  treeMenu->addAction(treeEditAct);
  treeMenu->addSeparator();
  treeMenu->addAction(tAAct);
  treeMenu->addSeparator();
  treeMenu->addAction(dbhHTAct);
  treeMenu->addAction(dbhLSRAct);
  treeMenu->addAction(posAct);
  treeMenu->addAction(heightAct);
  treeMenu->addAction(lengAct);
  treeMenu->addAction(dbhEditAct);
  treeMenu->addAction(skeletonAct);
  treeMenu->addAction(convexAct);
  treeMenu->addAction(concaveAct);
//MISC
  miscMenu = menuBar()->addMenu(tr("Other features"));
  miscMenu->addAction(plusAct);
  miscMenu->addAction(minusAct);
  miscMenu->addAction(voxAct);
  miscMenu->addAction(IDWAct);
  miscMenu->addAction(clipedAct);
//ABOUT
  helpMenu = menuBar()->addMenu(tr("&About"));
  helpMenu->addAction(aboutAct);
 }

void MainWindow::createTreeView()
{
  treeWidget->resizeColumnToContents(1);
}
void MainWindow::undo()
{
  if(!undopoint.empty())
  {
     // z posledniho prvku ve ktoru undopoint vzit cislo
    int num = undopoint.at(undopoint.size()-1);

    // num = pocet bodu od konce v m_cloudu1
    //vytvorit novy cloud s temito body
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    for(int y = m_cloud1->get_Cloud()->points.size() - num; y< m_cloud1->get_Cloud()->points.size(); y++)
    {
      pcl::PointXYZI bod;
      bod = m_cloud1->get_Cloud()->points.at(y);
      cloud->points.push_back(bod);
    }

    // kopirovat cloud k m_cloud
    int s = m_cloud->get_Cloud()->points.size();
    *m_cloud->get_Cloud() += *cloud;

    // smazat poslednich "x" bodu z m_cloud1
    *m_cloud1->get_Cloud()->points.erase(m_cloud1->get_Cloud()->points.end() - num, m_cloud1->get_Cloud()->points.end());
    // smazat posledni zaznam v undopoint
    undopoint.pop_back();
    //zobrazit
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
  }
  else
  {
    QMessageBox::information(this,("empty"), ("no more points to put back"));

  return;
  }
}
//QVTKWIDGET

void MainWindow::AreaEvent(const pcl::visualization::AreaPickingEvent& event, void* )
{

  pcl::PointIndices::Ptr inl (new pcl::PointIndices ());

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);


  if(event.getPointsIndices(inl->indices)== false)
  {
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    return;
  }

  if(m_cloud->get_Cloud()->points.size() > 1)
  {
    undopoint.push_back(inl->indices.size());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud (m_cloud->get_Cloud());
    extract.setIndices (inl);
    extract.setNegative (false); // false = vse co je vybrano
    extract.filter (*cloud2);

    extract.setNegative (true); // true = vse co neni vybrano
    extract.filter (*cloud1);

    *m_cloud1->get_Cloud() += *cloud2;
    *m_cloud->get_Cloud() = *cloud1;
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
  }
  return;
}
void MainWindow::AreaEvent2(const pcl::visualization::AreaPickingEvent& event, void* )
{
  pcl::PointIndices::Ptr inl (new pcl::PointIndices ());
  pcl::PointIndices::Ptr in (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);

  if (event.getPointsIndices (inl->indices))
  {
    for(int a = 0; a < inl->indices.size();a++)
    {
      if (inl->indices.at(a) < m_cloud->get_Cloud()->points.size())
          in->indices.push_back(inl->indices.at(a));
    }
    if(m_cloud->get_Cloud()->points.size() > 1)
    {
    //undopoint.push_back(inl->indices.size());
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud (m_cloud->get_Cloud());
      extract.setIndices (in);
      extract.setNegative (true); // true = vse co neni vybrano
      extract.filter (*cloud1);
    // vse co je vybrano patri  znicit
      extract.setNegative (false); //false = vse co je vybrano
      extract.filter (*cloud2);
    // vse co neni vybrano ulozit do m_cloud1
      m_cloud->set_Cloud(cloud1);
      m_vis->removeAllPointClouds();
      dispCloud(*m_cloud1,0,220,0); //reference
      dispCloud(*m_cloud,220,220,0); //original
    }
    return;
  }
  else
  {
    return;
  }
}
void MainWindow::pointEvent(const pcl::visualization::PointPickingEvent& event, void* )
{
  int idx = event.getPointIndex ();
  if (idx == -1)
    return;
        // Get the point that was picked
  pcl::PointXYZI pickp;
  event.getPoint (pickp.x, pickp.y, pickp.z);

  // if point is inbounding box, get cloud name
  for(int i = 0; i < Proj->get_sizeTreeCV(); i ++)
  {
    pcl::PointXYZI maxp, minp;
    pcl::getMinMax3D(*Proj->get_TreeCloud(i).get_Cloud(),minp,maxp);
    if(minp.x < pickp.x && minp.y < pickp.y && minp.z < pickp.z && maxp.x > pickp.x && maxp.y > pickp.y && maxp.z > pickp.z)
    {
      QString a = QString("Name of the tree: %1 \t Number of points: %2 \t"
                         " convex area: %3 m2 \t concave area: %4 m2").arg(Proj->get_TreeCloud(i).get_name())
                                                                      .arg(Proj->get_TreeCloud(i).get_Cloud()->points.size())
                                                                      .arg(Proj->get_TreeCloud(i).get_areaconvex())
                                                                      .arg(Proj->get_TreeCloud(i).get_areaconcave());
      statusBar()->showMessage(a);
    }
  }
  return;
}
//DISPLAY CLOUD
void MainWindow::dispCloud(Cloud cloud, QString field)
{
  QColor col = cloud.get_color();
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud.get_Cloud(), field.toUtf8().constData());

  if(!m_vis->updatePointCloud<pcl::PointXYZI>(cloud.get_Cloud(), intensity_distribution, cloud.get_name().toUtf8().constData()))
    m_vis->addPointCloud<pcl::PointXYZI>(cloud.get_Cloud(), intensity_distribution, cloud.get_name().toUtf8().constData());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.get_Psize(), cloud.get_name().toUtf8().constData());
  qvtkwidget->update();
}
void MainWindow::dispCloud(Cloud cloud)
{
  QColor col = cloud.get_color();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud.get_Cloud(),col.red(),col.green(),col.blue());
  if(!m_vis->updatePointCloud<pcl::PointXYZI>(cloud.get_Cloud(), color, cloud.get_name().toUtf8().constData()))
    m_vis->addPointCloud<pcl::PointXYZI>(cloud.get_Cloud(), color, cloud.get_name().toUtf8().constData());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.get_Psize(), cloud.get_name().toUtf8().constData());
  qvtkwidget->update();
}
void MainWindow::dispCloud(Cloud cloud, int red, int green, int blue)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud.get_Cloud(),red,green,blue);

  if(!m_vis->updatePointCloud<pcl::PointXYZI>(cloud.get_Cloud(),color, cloud.get_name().toUtf8().constData()))
    m_vis->addPointCloud<pcl::PointXYZI>(cloud.get_Cloud(),color, cloud.get_name().toUtf8().constData());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.get_Psize(), cloud.get_name().toUtf8().constData());
  qvtkwidget->update();
}
void MainWindow::removeCloud(QString name)
{
  m_vis->removePointCloud(name.toUtf8().constData());

  qvtkwidget->update();
}
//NAmes
QStringList MainWindow::get_allNames()
{
  QStringList names;
  //basecloud
  for(int i = 0; i< Proj->get_sizebaseCV(); i++)
  {
    names << Proj->get_baseCloud(i).get_name();
  }
  //teren
  for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
  {
    names << Proj->get_TerrainCloud(i).get_name();
  }
  //vege
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  //tree
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  //ost
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
return names;
}
QStringList MainWindow::get_treeNames()
{
  QStringList names;
    //tree
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
return names;
}
QStringList MainWindow::get_terrainNames()
{
  QStringList names;
    //teren
  for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
  {
    names << Proj->get_TerrainCloud(i).get_name();
  }
return names;
}
QStringList MainWindow::get_vegetationNames()
{
  QStringList names;
  //vege
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  return names;
}
QStringList MainWindow::get_ostNames()
{
  QStringList names;
  //ost
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
return names;
}
QStringList MainWindow::get_baseNames()
{
  QStringList names;
  //basecloud
  for(int i = 0; i< Proj->get_sizebaseCV(); i++)
  {
    names << Proj->get_baseCloud(i).get_name();
  }
return names;
}

//TREEWIDGET

void MainWindow::addTreeItem(QString name)
{
  QTreeWidgetItem * item = new QTreeWidgetItem();
  item->setText(1,name);
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable|Qt::ItemIsSelectable);
  item->setCheckState(0,Qt::Checked);
  treeWidget->addTopLevelItem(item);
  treeWidget->resizeColumnToContents(1);
}

void MainWindow::colorCloud(QString name)
{

  QColor color = QColorDialog::getColor(Qt::white,this);
  if(color.isValid())
  {
    Proj->set_color(name, color);
    dispCloud(Proj->get_Cloud(name));
  }
}
void MainWindow::colorCloudField(QString name)
{
  QStringList l;
  l<<"x"<<"y"<<"z"<<"intensity";
  bool ok;
  QString field = QInputDialog::getItem(this,tr("select field"),tr("select field to color cloud:"),l,0,false,&ok);
  if (ok == false)
    return;
  dispCloud(Proj->get_Cloud(name),field);

}
void MainWindow::PointSize(QString name)
{
  int p = QInputDialog::getInt(this,("point size"),("please enter value in range 1 - 64 defining size of point"),1,1,64);

  Proj->set_PointSize(name, p);
  dispCloud(Proj->get_Cloud(name));
}

void MainWindow::deleteCloud(QString name)
{
  Proj->delete_Cloud(name);
  m_vis->removePointCloud(name.toUtf8().constData());
}
void MainWindow::about()
{
QMessageBox::about(this,tr("about 3D Forest application"),tr(" 3D Forest application was developed as a part of my Ph.D. thesis.\n"
                                                             " But continue as a free platform for TLS data processing. \n"
                                                             "  AUTHORS: Jan Trochta j.trochta@gmail.com "));
 }
void MainWindow::dispCloud(QString name)
{
  Cloud *c = new Cloud(Proj->get_Cloud(name));
  dispCloud(*c);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr MainWindow::lowPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr c,float res, bool v)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cld (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cld_v (new pcl::PointCloud<pcl::PointXYZI>);
  for(int i =0; i < c->points.size(); i++)
  {
    bool top= false;
    pcl::PointXYZI bod = c->points.at(i);
    #pragma omp parallel for
    for(int j = 0; j < c->points.size(); j++)
    {
      pcl::PointXYZI por = c->points.at(j);

      if( (por.x - bod.x) < res &&  (bod.x - por.x) < res  &&  (por.y - bod.y) < res &&  (bod.y - por.y) < res  && bod.z > por.z)
      {
        top =true;
      }
    }
    if (top == false)
      cld->points.push_back(bod);
    else
      cld_v->points.push_back(bod);
  }
  if (v == true)
    return cld;
  else
    return cld_v;
}


