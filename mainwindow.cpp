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

#include <vtkTIFFWriter.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

////MainWindow
 MainWindow::MainWindow()
{
  Q_INIT_RESOURCE(3dforest);
  setWindowTitle ( tr("3D Forest - Forest lidar data processing tool") );
  m_cloud1 = new Cloud();
  m_cloud = new Cloud();
  Proj = new Project();

//QVTKwidget - visualizer
  qvtkwidget = new QVTKWidget();
  m_vis  = new Visualizer();
  vtkSmartPointer<vtkRenderWindow> renderWindow = m_vis->getRenderWindow();
  qvtkwidget->SetRenderWindow(renderWindow);
//  m_vis->addOrientationMarkerWidgetAxes();
  m_vis->setShowFPS(false);
  //m_vis->setBackgroundColor(1,1,1);
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
  createToolbars();

  statusBar()->showMessage(tr("3D Forest Ready"));
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
  m_vis->resetCamera();
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
// set connection of treebar to default.
  disconnect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_HideAll()));
  connect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_DisplayAll()));
  disconnect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_HideAll()));
  connect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_DisplayAll()));
  disconnect(heightT,SIGNAL(triggered()),this,SLOT(height_HideAll()));
  connect(heightT,SIGNAL(triggered()),this,SLOT(height_DisplayAll()));
  disconnect(positionT,SIGNAL(triggered()),this,SLOT(position_HideAll()));
  connect(positionT,SIGNAL(triggered()),this,SLOT(position_DisplayAll()));
  convexT->setEnabled(false);
  concaveT->setEnabled(false);

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

pcl::PointCloud<pcl::PointXYZI>::Ptr MainWindow::importTXT(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  QFile fileName (file);
  fileName.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileName);
  while(!in.atEnd())
  {
    QString line = in.readLine();
    QStringList coords = line.split("\t");
    if(coords.size() > 1)
    {
      pcl::PointXYZI data;

      data.x = (float)(coords.at(0).toDouble() + Proj->get_Xtransform());
      data.y = (float)(coords.at(1).toDouble() + Proj->get_Ytransform());
      data.z = coords.at(2).toFloat();
      if(coords.size() > 3)
        data.intensity=coords.at(3).toFloat();
      else
        data.intensity=1;

      cloud->points.push_back(data);
    }
  }
  fileName.close();
      //in.~QTextStream();
  cloud->width = cloud->points.size();
  cloud->is_dense=true;
  cloud->height=1;

  return cloud;
  cloud.reset();
}
pcl::PointCloud<pcl::PointXYZI>::Ptr MainWindow::importPTS(QString file)
{
  QFile fileName (file);
  fileName.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileName);
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
        QString a = QString ("xxx_%1").arg(cloud_number);
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
  fileName.close();
  cloud->width = cloud->points.size();
  cloud->is_dense=true;
  cloud->height=1;
  return cloud;

}
pcl::PointCloud<pcl::PointXYZI>::Ptr MainWindow::importPTX(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  QFile fileName (file);
  fileName.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileName);
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
  fileName.close();
  in.~QTextStream();
  cloud->width = cloud->points.size ();
  cloud->is_dense=true;
  cloud->height=1;
  return cloud;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr MainWindow::importLAS(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  std::ifstream ifs;
  ifs.open(file.toUtf8().constData(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);
//read data into cloud
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
  return cloud;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr MainWindow::importPCD(QString file)
{
  QMessageBox::StandardButton reply;
  reply =QMessageBox::question(0,("Transform cloud?"), "Do you want to apply transform matrix on selected file?",QMessageBox::Yes|QMessageBox::No );

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(file.toUtf8().constData(),*cloud);
  if(reply ==QMessageBox::Yes )
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
    for(int i = 0; i <cloud->points.size();i++)
    {
      pcl::PointXYZI oldp, newp;
      oldp = cloud->points.at(i);
      newp.x = oldp.x + Proj->get_Xtransform();
      newp.y = oldp.y + Proj->get_Ytransform();
      newp.z = oldp.z + Proj->get_Ztransform();
      cloud_->points.push_back(newp);
    }
    cloud = cloud_;
    cloud_.reset();
  }

  cloud->width = cloud->points.size();
  cloud->is_dense=true;
  cloud->height=1;
  return cloud;
}

void MainWindow::importBaseCloud()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select file"),Proj->get_Path(),
                                                 tr("All supported files (*.pcd *.txt *.xyz *.las *.pts *.ptx);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    QStringList typ = fileName.split(".");

    if(typ.at(typ.size()-1) == "pcd")
      cloud = importPCD(fileName);
    if(typ.at(typ.size()-1) == "txt")
      cloud = importTXT(fileName);
    if(typ.at(typ.size()-1) == "xyz")
      cloud = importTXT(fileName);
    if(typ.at(typ.size()-1) == "las")
      cloud = importLAS(fileName);
    if(typ.at(typ.size()-1) == "pts")
      cloud = importPTS(fileName);
    if(typ.at(typ.size()-1) == "ptx")
      cloud = importPTX(fileName);


    QStringList Fname = fileName.split("\\");
    QStringList name = Fname.back().split(".");

    //if file exist in project folder, ask for another name
    QString newFile = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name.at(0));
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = name.at(0);
    }

    Proj->save_newCloud("cloud",newName,cloud);
    QString newFilepath = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(newName);
    openCloudFile(newFilepath);
    cloud.reset();
  }
}
void MainWindow::importTerrainFile()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select file"),Proj->get_Path(),
                                                 tr("All supported files (*.pcd *.txt *.xyz *.las *.pts *.ptx);;;;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    QStringList typ = fileName.split(".");

    if(typ.at(typ.size()-1) == "pcd")
      cloud = importPCD(fileName);
    if(typ.at(typ.size()-1) == "txt")
      cloud = importTXT(fileName);
    if(typ.at(typ.size()-1) == "xyz")
      cloud = importTXT(fileName);
    if(typ.at(typ.size()-1) == "las")
      cloud = importLAS(fileName);
    if(typ.at(typ.size()-1) == "pts")
      cloud = importPTS(fileName);
    if(typ.at(typ.size()-1) == "ptx")
      cloud = importPTX(fileName);


    QStringList Fname = fileName.split("\\");
    QStringList name = Fname.back().split(".");

    //if file exist in project folder, ask for another name
    QString newFile = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name.at(0));
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = name.at(0);
    }

    Proj->save_newCloud("teren",newName,cloud);
    QString newFilepath = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(newName);
    openCloudFile(newFilepath);
  }
}
void MainWindow::importVegeCloud()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select file"),Proj->get_Path(),
                                                 tr("All files (*.*);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    if(selectedFilter == "PCD files (*.pcd)")
      cloud = importPCD(fileName);
    if(selectedFilter == "TXT files (*.txt)")
      cloud = importTXT(fileName);
    if(selectedFilter == "LAS files (*.las)")
      cloud = importLAS(fileName);
    if(selectedFilter == "PTS files (*.pts)")
      cloud = importPTS(fileName);
    if(selectedFilter == "PTX files (*.ptx)")
      cloud = importPTX(fileName);

    QStringList Fname = fileName.split("\\");
    QStringList name = Fname.back().split(".");
    //if file exist in project folder, ask for another name
    QString newFile = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name.at(0));
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = name.at(0);
    }
    Proj->save_newCloud("vege",newName,cloud);
    QString newFilepath = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(newName);
    openCloudFile(newFilepath);
  }
}
void MainWindow::importTreeCloud()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select file"),Proj->get_Path(),
                                                 tr("All files (*.*);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    if(selectedFilter == "PCD files (*.pcd)")
      cloud = importPCD(fileName);
    if(selectedFilter == "TXT files (*.txt)")
      cloud = importTXT(fileName);
    if(selectedFilter == "LAS files (*.las)")
      cloud = importLAS(fileName);
    if(selectedFilter == "PTS files (*.pts)")
      cloud = importPTS(fileName);
    if(selectedFilter == "PTX files (*.ptx)")
      cloud = importPTX(fileName);

    QStringList Fname = fileName.split("\\");
    QStringList name = Fname.back().split(".");
    //if file exist in project folder, ask for another name
    QString newFile = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name.at(0));
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = name.at(0);
    }
    Proj->save_newCloud("strom",newName,cloud);
    QString newFilepath = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(newName);
    openCloudFile(newFilepath);
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
  cloud.reset();
  Proj->set_TerrainCloud(*c);

  //dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  delete c;

}
void MainWindow::openVegeFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);

  Proj->set_VegeCloud(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  delete c;
  cloud.reset();
}
void MainWindow::openTreeFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);
  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c = new Cloud(cloud,coords.back(),col);

  Proj->set_Tree(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());

  m_vis->resetCamera();
  delete c;
  cloud.reset();

}
void MainWindow::openOstFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);

  Proj->set_OstCloud(*c);
  cloud.reset();
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  delete c;
}
void MainWindow::openCloudFile(QString file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  QColor col = QColor(rand() %255,rand() %255,rand() %255);
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_baseCloud(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  cloud.reset();
  delete c;
}
void MainWindow::openTerrainFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  cloud.reset();
  Proj->set_TerrainCloud(*c);
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  delete c;
}
void MainWindow::openVegeFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_VegeCloud(*c);
  dispCloud(*c);
  cloud.reset();
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  delete c;
}
void MainWindow::openOstFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_OstCloud(*c);
  cloud.reset();
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  delete c;
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
  delete c;
  cloud.reset();
}
void MainWindow::openCloudFile(QString file, QColor col)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split("\\");
  Cloud *c =new Cloud(cloud,coords.back(),col);
  Proj->set_baseCloud(*c);
  cloud.reset();
  dispCloud(*c);
  addTreeItem(c->get_name());
  m_vis->resetCamera();
  delete c;
}

//EXPORT method
void MainWindow::exportCloud()
{
//vybrat cloud
  QString cloudName = QInputDialog::getItem(this,("Please select cloud for exporting into text file"),("Name of cloud:"),get_allNames());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  *cloud = *Proj->get_Cloud(cloudName).get_Cloud();

//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("files (*.txt)"));
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
  cloud.reset();
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
  QString cloudName = QInputDialog::getItem(this,("Please select cloud for exporting into PLY file"),("Name of cloud:"),names);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  *cloud = *Proj->get_Cloud(cloudName).get_Cloud();
//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("files (*.ply)"));
  //zapisovat jednotlive radky

  pcl::io::savePLYFileASCII(newFile.toUtf8().constData(),*cloud);
cloud.reset();
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
  QString cloudName = QInputDialog::getItem(this,("Please select cloud for exporting into text file"),("Name of cloud:"),names);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  *cloud = *Proj->get_Cloud(cloudName).get_Cloud();
//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("files (*.pts)"));
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
  cloud.reset();
}
void MainWindow::exportConvexTxt()
{
    //vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("files (*.txt)"));
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
    delete cloud;
  }
  file.close();


}
void MainWindow::exportConcaveTxt()
{
//vybrat jmeno noveho souboru
  QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("files (*.txt)"));
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
        double z = bod.z;
        out << i << ";" << x << ";" << y ;
        bod = cloud->get_Cloud()->points.at(j);
        double xend = bod.x - Proj->get_Xtransform();
        double yend = bod.y - Proj->get_Ytransform();
        out << ";" << xend << ";" << yend << ";" << z << endl;
    }
    delete cloud;
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
  QStringList names;
  names << get_baseNames();
  names << get_terrainNames();
  names << get_ostNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Voxelized terrain");
  in->set_path(Proj->get_Path());
  in->set_description("Automated method for terrain extraction from base cloud. "
                      "Input cloud is voxelized to the voxels based on given resolution and for terrain are selected lowest points which are saved into new file.\n"
                      "This method is excelent for deriving DEM from point cloud since result contains less point of vegetation than other method, but for further vegetation analysis "
                      "do not serve well. ");
  in->set_inputCloud1("Input cloud:",names);
  in->set_outputCloud1("Output cloud of terrain:","voxel-terrain");
  in->set_outputCloud2("Output cloud of non-ground:","voxel-vegetation");
  in->set_inputInt("Resolution in cm:","20");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {

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
    cloud_filtered.reset();
    cloud.reset();
//save vege cloud
      cloud_vege->width = cloud_vege->points.size ();
      cloud_vege->is_dense=true;
      cloud_vege->height=1;

      Proj->save_newCloud("vege",in->get_outputCloud2(),cloud_vege);
      QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud2());
      openVegeFile(fullnameV);
      cloud_vege.reset();
  }
  delete in;
}

void MainWindow::octreeSlot()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Terrain created with OcTree ");
  in->set_path(Proj->get_Path());
  in->set_description("Method divide input cloud into cubes with given resolution and select the lowest ones."
                      " This method use two pass approach for removing points in distant places. "
                      " In first round create cubes with 5 times greater resolution that user input, in the second round in uses given resolution to the rest of the cloud."
                      " Result is cloud with no reduction of points but with more noise. After manual adjustment it can be used for detailed terrain morphology and rest can be used for vegetation segmentation. ");
  in->set_inputCloud1("Input cloud:",get_baseNames());
  in->set_outputCloud1("Output cloud of terrain:","Octree-terrain");
  in->set_outputCloud2("Output cloud of non-ground:","Octree-vegetation");
  in->set_inputInt("Resolution","10");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    float res = in->get_intValue()/100.0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp4(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp5(new pcl::PointCloud<pcl::PointXYZI>);

      //velky cyklus
    octree(res*10,Proj->get_Cloud(in->get_inputCloud1()).get_Cloud(),cloud_tmp4, cloud_tmp);
    octree(res,cloud_tmp,cloud_vege, cloud_tmp2);

    cloud_tmp.reset() ;
    cloud_tmp4->points.clear();
    //maly cyklus
    octree(res,Proj->get_Cloud(in->get_inputCloud1()).get_Cloud(),cloud_tmp4, cloud_tmp3);
    cloud_tmp4->points.clear();
    cloud_tmp4.reset() ;
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


    cloud_tmp2.reset() ;
    cloud_tmp3.reset() ;
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

    cloud_ground.reset();


//SAVE
//Terrain
      Cloud *cT = new Cloud(cloud_tmp5,in->get_outputCloud1());
      saveTerrainCloud(cT);
      delete cT;
      cloud_tmp5.reset() ;
    //vegetation
      Cloud *cV = new Cloud(cloud_vege,in->get_outputCloud2());
      saveVegeCloud(cV);
      delete cV;
      cloud_vege.reset();




  }
delete in;
}
void MainWindow::manualAdjust()
{
  InputDialog *in = new InputDialog(this);
  in->set_path(Proj->get_Path());
  in->set_title("Manual adjustment of terrain cloud");
  in->set_description("Manual editing of point cloud representing terrain. For editing please press key 'x' and draw rectangle with left mouse button. "
                      "For return to normal mouse move press again 'x'."
                        " Selected points will be deleted and saved in output cloud ");
  in->set_inputCloud1("Input terrain cloud:",get_terrainNames());
  in->set_outputCloud1("Output cloud of deleted points:","terrain-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
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
    addToolBarBreak ();
    editBar = new QToolBar(tr("edit bar"),this);
    editBar->setIconSize(QSize(16,16));
    this->addToolBar(editBar);
    QAction *stopE = editBar->addAction(QPixmap(":/images/stopEdit.png"),"Stop EDIT");
    connect(stopE,SIGNAL(triggered()),this,SLOT(manualAdjustStop()));
    QAction *undoAct = editBar->addAction(QPixmap(":/images/undo.png"),"undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );

    undopoint.clear();
    m_vis->removeAllPointClouds();
    treeWidget->allItemOFF();
    dispCloud(*m_cloud,220,220,0);
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
delete in;
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
  in->set_description("Manual editing of point cloud representing vegetation. For editing please press key 'x' and drag and draw rectangle wth left mouse button."
                        " Selected points will be deleted. After selection you can choose if you want edit more or you want to close it.");
  in->set_inputCloud1("Input Vegetation cloud:",names);
  in->set_outputCloud1("Output cloud of deleted points:","vegetation-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    //set input cloud as m_cloud
    m_cloud->get_Cloud()->clear();
    m_cloud->set_name(in->get_inputCloud1());
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //set output into m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //spustit editacni listu
    addToolBarBreak ();
    editBar = new QToolBar(tr("edit bar"),this);
    editBar->setIconSize(QSize(16,16));
    this->addToolBar(editBar);
    QAction *stopEd = editBar->addAction(QPixmap(":/images/stopEdit.png"),"Stop EDIT");
    connect(stopEd,SIGNAL(triggered()),this,SLOT(manualSelectStop()) );

    QAction *undoAct = editBar->addAction(QPixmap(":/images/undo.png"),"undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();

    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->removeAllPointClouds();
    treeWidget->allItemOFF();
    dispCloud(*m_cloud,220,220,0);
    m_vis->resetCamera();
    qvtkwidget->update();
  }
delete in;
}
void MainWindow::manualSelectStop()
{
// save m_cloud as new tree


  saveTreeCloud(m_cloud->get_Cloud());
  //save m_cloud1 into file
  // if exist update file and memory
  QString file = QString("%1.pcd").arg(m_cloud1->get_name());
  saveVegeCloud(m_cloud1, true);

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
    h_cloud.reset();
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
  delete msgBox;
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

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
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
    addToolBarBreak ();
    editBar = new QToolBar(tr("edit bar"),this);
    editBar->setIconSize(QSize(16,16));
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction(QPixmap(":/images/stopEdit.png"),"Stop EDIT");
    connect(stopTE,SIGNAL(triggered()),this,SLOT(treeEditStop()) );
    QAction *undoAct = editBar->addAction(QPixmap(":/images/undo.png"),"undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();

    treeWidget->allItemOFF();
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
delete in;
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

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
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
    addToolBarBreak ();
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    editBar->setIconSize(QSize(16,16));
    QAction *stopTE = editBar->addAction(QPixmap(":/images/stopEdit.png"),"Stop EDIT");
    QAction *undoAct = editBar->addAction(QPixmap(":/images/undo.png"),"undo");
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
delete in;
}
void MainWindow::dbhCloudStopEdit()
{
  // save m_cloud as new tree_dbh -
  QStringList name = m_cloud->get_name().split(".edit");

  Proj->set_dbhCloud(name.at(0),m_cloud->get_Cloud());
  Proj->set_treeDBH_HT(name.at(0));
  Proj->set_treeDBH_LSR(name.at(0));
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
  exdialog->set_description("Method for exporting tree attributes into formatted text file. User can select tree for which want to export attributes, separator of field and attributes."
                            " Method export currently computed values. If you want to change any (position computed with terrain, etc.) use special methods for computing those.");
  exdialog->set_trees(names);
  int dl = exdialog->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
      //progressbar
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
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
      for(int i = 1; i < names.size(); i++ )
      {

        Tree *c = new Tree(Proj->get_TreeCloud(names.at(i)));
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
         c->set_length();
          out << exdialog->get_separator() << c->get_length();
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
        delete c;
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
       c->set_length();
        out << exdialog->get_separator() << c->get_length();
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
      delete c;
    }
    file.close();
  }
  statusBar()->removeWidget(pBar);
  delete exdialog;
}
void MainWindow::convexhull()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute convex hull of tree.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing and display planar projection of given tree."
                      " This method serve for computing point representing convex planar projection of tree and its area.");
  in->set_inputCloud1("Input Tree cloud:",names);

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 1; i < names.size() ; i++ )
      {

        Proj->set_treeConvexCloud(names.at(i));
        convexhullDisplay(names.at(i));
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->set_treeConvexCloud(in->get_inputCloud1());
      convexhullDisplay(in->get_inputCloud1());
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
    convexT->setEnabled(true);
    convexT->setIcon(QPixmap(":/images/treeBar/convex_sel.png"));
  }
  delete in;
}
void MainWindow::convexhullDisplay(QString name)
{
  //addcloud
  QColor col = Proj->get_TreeCloud(name).get_color();
  dispCloud(Proj->get_TreeCloud(name).get_vexhull(),col.red(),col.green(), col.blue());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, Proj->get_TreeCloud(name).get_vexhull().get_name().toUtf8().constData());

 //addAreaText
  pcl::PointXYZI bod;
  bod = Proj->get_TreeCloud(name).get_pose();
  QString h = QString("%1 m2").arg(Proj->get_TreeCloud(name).get_areaconvex());
  std::stringstream name2 ;
  name2 << name.toUtf8().constData() << "_vexText";
  m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.2,0.3,0,name2.str());

  //addpolygon
  std::stringstream Pname;
  Pname << name.toUtf8().constData() << "_convex_polygon";
  m_vis->addPolygon<pcl::PointXYZI>(Proj->get_TreeCloud(name).get_vexhull().get_Cloud(),col.redF(),col.greenF(), col.blueF(), Pname.str());
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, Pname.str() );
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, Pname.str() );
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION , pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE , Pname.str() );

  disconnect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_DisplayAll()));
  connect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_HideAll()));
}
void MainWindow::convexhull_DisplayAll()
{
  QStringList names;
  names << get_treeNames();

  pBar = new QProgressBar(statusBar());
  pBar->setMaximumSize(200,16);
  statusBar()->addWidget(pBar);
  pBar->setValue(0);
  for(int i = 0; i < names.size(); i++)
  {
    convexhullDisplay(names.at(i));
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();
  }
  statusBar()->removeWidget(pBar);
  convexT->setIcon(QPixmap(":/images/treeBar/convex_sel.png"));
}
void MainWindow::convexhull_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove dbhCloud
    std::stringstream Clname ;
    Clname  << names.at(i).toUtf8().constData() << "_convex";
    m_vis->removePointCloud(Clname.str());

    //remove polygon
    std::stringstream Pname ;
    Pname  << names.at(i).toUtf8().constData() << "_convex_polygon";
    m_vis->removeShape (Pname.str());

    //remove text
    std::stringstream Tname ;
    Tname  << names.at(i).toUtf8().constData() << "_vexText";
    m_vis->removeText3D (Tname.str());

  }
  disconnect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_HideAll()));
  connect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_DisplayAll()));
   convexT->setIcon(QPixmap(":/images/treeBar/convex.png"));
  qvtkwidget->update();
}
void MainWindow::concavehull()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute concave planar projection of tree.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing and display planar projection of given tree."
                      " This method serve for computing point representing concave planar projection of tree and its area."
                      " Concave planar projection of a tree is a region enclosing the point cloud in direction of Z axis.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_inputInt("Input Maximal Edge length cm:","150");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 1; i < names.size(); i++ )
      {
        int errors = Proj->set_treeConcaveCloud(names.at(i),(float)in->get_intValue()/100);

        concavehullDisplay(names.at(i));
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
        if(errors>0)
      {
        QString m = QString(" Warning: %1 edge are longer than Maximum Edge Lenght.\n In cloud: %2").arg(errors).arg(Proj->get_TreeCloud(names.at(i)).get_name());
        QMessageBox::information(0,("Warning"),m);
      }
      }
    }
    else
    {
      int errors = Proj->set_treeConcaveCloud(in->get_inputCloud1(),(float)in->get_intValue()/100);

      concavehullDisplay(in->get_inputCloud1());
      pBar->setValue(100);
      pBar->update();

      if(errors>0)
      {
        QString m = QString(" Warning: %1 edge are longer than Maximum Edge Lenght.\n In cloud: %2").arg(errors).arg(Proj->get_TreeCloud(in->get_inputCloud1()).get_name());
        QMessageBox::information(0,("Warning"),m);
      }
    }
    statusBar()->removeWidget(pBar);
    concaveT->setEnabled(true);
    concaveT->setIcon(QPixmap(":/images/treeBar/concave_sel.png"));
  }
  delete in;
}
void MainWindow::concavehullDisplay(QString name)
{
  //addcloud
  QColor col = Proj->get_TreeCloud(name).get_color();
  dispCloud(Proj->get_TreeCloud(name).get_concavehull(),col.red(),col.green(), col.blue());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, Proj->get_TreeCloud(name).get_concavehull().get_name().toUtf8().constData());

  //addAreaText
  pcl::PointXYZI bod;
  bod = Proj->get_TreeCloud(name).get_pose();
  float r = Proj->get_TreeCloud(name).get_areaconcave()*100;

  QString h = QString("%1 m2").arg(std::ceil(r)/100);
  std::stringstream name2 ;
  name2 << name.toUtf8().constData() << "_caveText";
  //m_vis->addText3D(h.toUtf8().constData(),bod,1.3,0.1,0.1,0.1,name2.str());

  //addpolygon
  std::stringstream Pname;
  Pname << name.toUtf8().constData() << "_concave_polygon";
  m_vis->addPolygon<pcl::PointXYZI>(Proj->get_TreeCloud(name).get_concavehull().get_Cloud(),col.redF(),col.greenF(), col.blueF(), Pname.str());
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, Pname.str() );
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, Pname.str() );
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION , pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE , Pname.str() );
  disconnect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_DisplayAll()));
  connect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_HideAll()));
}
void MainWindow::concavehull_DisplayAll()
{
  QStringList names;
  names << get_treeNames();

  pBar = new QProgressBar(statusBar());
  pBar->setMaximumSize(200,16);
  statusBar()->addWidget(pBar);
  pBar->setValue(0);
  for(int i = 0; i < names.size(); i++)
  {
    concavehullDisplay(names.at(i));
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();
  }
  statusBar()->removeWidget(pBar);
   concaveT->setIcon(QPixmap(":/images/treeBar/concave_sel.png"));
}
void MainWindow::concavehull_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove dbhCloud
    std::stringstream Clname ;
    Clname  << names.at(i).toUtf8().constData() << "_concave";
    m_vis->removePointCloud(Clname.str());

    //remove polygon
    std::stringstream Pname ;
    Pname  << names.at(i).toUtf8().constData() << "_concave_polygon";
    m_vis->removeShape (Pname.str());

    //remove text
    std::stringstream Tname ;
    Tname  << names.at(i).toUtf8().constData() << "_caveText";
    m_vis->removeText3D (Tname.str());
  }
  disconnect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_HideAll()));
  connect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_DisplayAll()));
  concaveT->setIcon(QPixmap(":/images/treeBar/concave.png"));
  qvtkwidget->update();
}
void MainWindow::dbhCheck()
{
  // pro each tree
  QStringList trees;
 std::vector<float> hts;
  std::vector<float> lsrs;
  for(int i = 0; i < Proj->get_sizeTreeCV() ; i++ )
  {
    stred lsr;
    lsr = Proj->get_TreeCloud(i).get_dbhLSR();
    stred ht;
    ht = Proj->get_TreeCloud(i).get_dbhHT();
    if(ht.r == -0.5 || lsr.r == -0.5)
      continue;
    //if the difference is greater that 10cm
    if((ht.r - lsr.r) > 8 || (ht.r - lsr.r) < -8)
    {
      //pass name of the tree to the list.
      trees << Proj->get_TreeCloud(i).get_name();
      hts.push_back(ht.r);
      lsrs.push_back(lsr.r);
    }
  }
    // save the list in the text file.
  QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),Proj->get_Path(),tr("files (*.txt)"));
   //zapisovat jednotlive radky
  QFile file (newFile);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream out(&file);

  for(int j = 1; j < trees.size(); j++)
  {
    out << trees.at(j) <<" " << hts.at(j) <<" " <<  lsrs.at(j)<<"\n";
  }
  file.close();
}
void MainWindow::dbhHT()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute/Display tree DBH using Randomized Hough Transform (RHT) for circle detection.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing and display tree DBH (Diameter in Breast Height) for given tree."
                        " This method serve for computing and display estimated cylinder with computed centre, diameter and with cylinder height 10 cm."
                        " The result is saved in tree variable and can be recomputed if you change DBH cloud.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 0; i < Proj->get_sizeTreeCV() ; i++ )
      {
        Proj->get_TreeCloud(i).set_dbhHT();
        dbhHTDisplay(Proj->get_TreeCloud(i).get_name());

        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      //urcit dbh
      Proj->get_TreeCloud(in->get_inputCloud1()).set_dbhHT();
      dbhHTDisplay(Proj->get_TreeCloud(in->get_inputCloud1()).get_name());
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::dbhHTDisplay(QString name)
{
  stred x;
  x = Proj->get_TreeCloud(name).get_dbhHT();
    // zobrazit cylinder a hodnotu
  if(x.r < 5000 && x.r > 0.5)
  {
    QString cl_name = QString ("%1_dbh").arg(name);
    Cloud *cl_ = new Cloud(Proj->get_TreeCloud(name).get_dbhCloud(),cl_name ) ;
    dispCloud(*cl_,255, 0, 0);
    delete cl_;
          //Coeff
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
    coef->values.push_back((float)x.a);
    coef->values.push_back((float)x.b);
    coef->values.push_back((float)x.z-0.05);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0.1);
    coef->values.push_back((float)x.r/100);
    std::stringstream Cname ;
    Cname << name.toUtf8().constData() << "_cylinder_HT";
    m_vis->removeShape(Cname.str());
    m_vis->addCylinder(*coef,Cname.str());

      //addtext3D with R
    pcl::PointXYZ bod;
    bod.x=x.a+(float)x.r/100;
    bod.y=x.b;
    bod.z=x.z+0.1;
    QString h= QString("%1").arg(x.r*2.0);
    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_value_HT";
    m_vis->removeText3D(name2.str());
    m_vis->addText3D(h.toUtf8().constData(),bod,1.25,0.2,0.5,0,name2.str());
    coef.reset();
    }
  else
  {
    QString m = QString("Computed DBH  for tree '%1' is out of range 0 - 50m.\n"
                        "Please check tree DBH Cloud and if needed edit points using DBHCloud Edit tool.").arg(name);
    QMessageBox::information(0,("Warning"),m);
  }
  //disconnect and connect different for treebar
  disconnect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_DisplayAll()));
  connect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_HideAll()));
}
void MainWindow::dbhHT_DisplayAll()
{
  QStringList names;
  names << get_treeNames();

  pBar = new QProgressBar(statusBar());
  pBar->setMaximumSize(200,16);
  statusBar()->addWidget(pBar);
  pBar->setValue(0);
  for(int i = 0; i < names.size(); i++)
  {
    dbhHTDisplay(names.at(i));
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();
  }
  statusBar()->removeWidget(pBar);
  dbhthT->setIcon(QPixmap(":/images/treeBar/dbhHT_sel.png"));
}
void MainWindow::dbhHT_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove cylinder
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_cylinder_HT";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_value_HT";
    m_vis->removeText3D (Tname.str());

    //remove dbhCloud
    std::stringstream Clname ;
    Clname  << names.at(i).toUtf8().constData() << "_dbh";
    m_vis->removePointCloud(Clname.str());
  }
  disconnect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_HideAll()));
  connect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_DisplayAll()));
  dbhthT->setIcon(QPixmap(":/images/treeBar/dbhHT.png"));
  qvtkwidget->update();
}
void MainWindow::dbhLSR()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute/Display tree DBH using Least square regression (LSR) for circle detection.");
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
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees")
    {
      for(int i = 1; i < names.size(); i++ )
      {
        Proj->get_TreeCloud(i).set_dbhLSR();
        dbhLSRDisplay(Proj->get_TreeCloud(i).get_name());
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->get_TreeCloud(in->get_inputCloud1()).set_dbhLSR();
      dbhLSRDisplay(Proj->get_TreeCloud(in->get_inputCloud1()).get_name());
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::dbhLSRDisplay(QString name)
{
  stred x;
  x = Proj->get_TreeCloud(name).get_dbhLSR();
    // zobrazit cylinder a hodnotu
  if(x.r < 5000 && x.r > 0.5)
  {
    QString cl_name = QString ("%1_dbh").arg(name);
    Cloud *cl_ = new Cloud(Proj->get_TreeCloud(name).get_dbhCloud(),cl_name ) ;
    dispCloud(*cl_,255, 0, 0);
          //Coeff
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
    coef->values.push_back((float)x.a);
    coef->values.push_back((float)x.b);
    coef->values.push_back((float)x.z-0.05);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0.1);
    coef->values.push_back((float)x.r/100);
    std::stringstream Cname ;
    Cname << name.toUtf8().constData() << "_cylinder_LSR";
    m_vis->removeShape(Cname.str());
    m_vis->addCylinder(*coef,Cname.str());

      //addtext3D with R
    pcl::PointXYZ bod;
    bod.x=x.a+(float)x.r/100;
    bod.y=x.b;
    bod.z=x.z+0.1;
    QString h= QString("%1").arg(x.r*2.0);
    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_value_LSR";
    m_vis->removeText3D(name2.str());
    m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0,name2.str());
    coef.reset();
  }
  else
  {
    QString m = QString("Computed DBH  for tree '%1' is out of range 0 - 50m.\n"
                        "Please check tree DBH Cloud and if needed edit points using DBHCloud Edit tool.").arg(name);
    QMessageBox::information(0,("Warning"),m);
  }
  //disconnect and connect different for treebar
  disconnect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_DisplayAll()));
  connect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_HideAll()));
}
void MainWindow::dbhLSR_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  pBar = new QProgressBar(statusBar());
  pBar->setMaximumSize(200,16);
  statusBar()->addWidget(pBar);
  pBar->setValue(0);

  for(int i = 0; i < names.size(); i++)
  {
    dbhLSRDisplay(names.at(i));
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();
  }
  statusBar()->removeWidget(pBar);
  dbhlsrT->setIcon(QPixmap(":/images/treeBar/dbhLSR_sel.png"));
}
void MainWindow::dbhLSR_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove cylinder
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_cylinder_LSR";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_value_LSR";
    m_vis->removeText3D (Tname.str());

    //remove dbhCloud
    std::stringstream Clname ;
    Clname  << names.at(i).toUtf8().constData() << "_dbh";
    m_vis->removePointCloud(Clname.str());
  }
  disconnect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_HideAll()));
  connect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_DisplayAll()));
  dbhlsrT->setIcon(QPixmap(":/images/treeBar/dbhLSR.png"));
  qvtkwidget->update();
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
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 1; i < names.size(); i++ )
      {
        Proj->get_TreeCloud(names.at(i)).get_height();
        heightDisplay(names.at(i));

        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->get_TreeCloud(in->get_inputCloud1()).get_height();
      heightDisplay(in->get_inputCloud1());

      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::heightDisplay(QString name)
{
  std::stringstream Hname ;
  Hname << name.toUtf8().constData() << "_heightline";
  pcl::PointXYZI minp,maxp;
  minp = Proj->get_TreeCloud(name).get_pose();
  maxp.x = Proj->get_TreeCloud(name).get_pose().x;
  maxp.y = Proj->get_TreeCloud(name).get_pose().y;
  maxp.z = Proj->get_TreeCloud(name).get_pose().z + Proj->get_TreeCloud(name).get_height();
  m_vis->removeShape(Hname.str());
  m_vis->addLine(minp,maxp,Hname.str());

  std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_heighttext";

  QString h = QString("%1").arg(Proj->get_TreeCloud(name).get_height());
  m_vis->removeText3D(name2.str());
  m_vis->addText3D(h.toUtf8().constData(),maxp,1.3,0.6,0.5,0,name2.str());



  disconnect(heightT,SIGNAL(triggered()),this,SLOT(height_DisplayAll()));
  connect(heightT,SIGNAL(triggered()),this,SLOT(height_HideAll()));
}
void MainWindow::height_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  pBar = new QProgressBar(statusBar());
  pBar->setMaximumSize(200,16);
  statusBar()->addWidget(pBar);
  pBar->setValue(0);
  for(int i = 0; i < names.size(); i++)
  {
    heightDisplay(names.at(i));
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();
  }
  statusBar()->removeWidget(pBar);
  heightT->setIcon(QPixmap(":/images/treeBar/height_sel.png"));
}
void MainWindow::height_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove line
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_heightline";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_heighttext";
    m_vis->removeText3D (Tname.str());

  }
  disconnect(heightT,SIGNAL(triggered()),this,SLOT(height_HideAll()));
  connect(heightT,SIGNAL(triggered()),this,SLOT(height_DisplayAll()));
  heightT->setIcon(QPixmap(":/images/treeBar/height.png"));
  qvtkwidget->update();
}
void MainWindow::length()
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
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 1; i < names.size(); i++ )
      {
        Proj->set_length(names.at(i));

        lengthDisplay(names.at(i));

        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->set_length(in->get_inputCloud1());

      lengthDisplay(in->get_inputCloud1());

      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::lengthDisplay(QString name)
{
  std::stringstream Lname ;
  Lname << name.toUtf8().constData() << "_length";
  m_vis->removeShape(Lname.str());
  m_vis->addLine(Proj->get_TreeCloud(name).get_lpoint(true),Proj->get_TreeCloud(name).get_lpoint(false),Lname.str());
  std::stringstream Lname2 ;
  Lname2 << name.toUtf8().constData() << "_lengthText";
  QString h= QString("%1").arg(Proj->get_TreeCloud(name).get_length());
  m_vis->removeText3D(Lname2.str());
  m_vis->addText3D(h.toUtf8().constData(),Proj->get_TreeCloud(name).get_lpoint(false),0.6,0.6,0.5,0,Lname2.str());

  disconnect(lengthT,SIGNAL(triggered()),this,SLOT(length_DisplayAll()));
  connect(lengthT,SIGNAL(triggered()),this,SLOT(length_HideAll()));
}
void MainWindow::length_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  pBar = new QProgressBar(statusBar());
  pBar->setMaximumSize(200,16);
  statusBar()->addWidget(pBar);
  pBar->setValue(0);
  for(int i = 0; i < names.size(); i++)
  {
    lengthDisplay(names.at(i));
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();
  }
  statusBar()->removeWidget(pBar);
  lengthT->setIcon(QPixmap(":/images/treeBar/length_sel.png"));

}
void MainWindow::length_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove line
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_length";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_lengthText";
    m_vis->removeText3D (Tname.str());

  }
  disconnect(lengthT,SIGNAL(triggered()),this,SLOT(length_HideAll()));
  connect(lengthT,SIGNAL(triggered()),this,SLOT(length_DisplayAll()));
  lengthT->setIcon(QPixmap(":/images/treeBar/length.png"));
  qvtkwidget->update();
}
void MainWindow::position()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  QStringList names2;
  names2 << get_terrainNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute tree position using lowest points of tree cloud.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for estimating tree position. Position is computed as a median of points lying in horizontal distance up to 60 cm from lowest point in cloud."
                        " Position is displayed as a sphere with centre at position and with radius 5 cm.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_inputCloud2("Input Terrain cloud:",names2);
  in->set_inputCheckBox("Recalculate parameters (DBH cloud, Height) based on tree position?");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(160,16);
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 1; i < names.size(); i++ )
      {
        Proj->set_treePosition(names.at(i),Proj->get_TerrainCloud(in->get_inputCloud2()));
        positionDisplay(names.at(i));
        if(in->get_CheckBox()==true)
        {
          Proj->set_treeDBHCloud(names.at(i));
          Proj->set_treeheigth(names.at(i));
        }
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->set_treePosition(in->get_inputCloud1(),Proj->get_TerrainCloud(in->get_inputCloud2()));
      positionDisplay(in->get_inputCloud1());
      if(in->get_CheckBox()==true)
      {
        Proj->set_treeDBHCloud(in->get_inputCloud1());
        Proj->set_treeheigth(in->get_inputCloud1());
      }
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::positionDisplay(QString name)
{
  QColor col = Proj->get_TreeCloud(name).get_color();

  std::stringstream Sname;
  Sname << name.toUtf8().constData() << "_sphere";

  if(!m_vis->updateSphere(Proj->get_TreeCloud(name).get_pose(),0.1,1,1,1,Sname.str()))
    m_vis->addSphere(Proj->get_TreeCloud(name).get_pose(),0.1,1,1,1,Sname.str());

  disconnect(positionT,SIGNAL(triggered()),this,SLOT(position_DisplayAll()));
  connect(positionT,SIGNAL(triggered()),this,SLOT(position_HideAll()));
}
void MainWindow::position_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  pBar = new QProgressBar(statusBar());
  pBar->setMaximumSize(200,16);
  statusBar()->addWidget(pBar);
  pBar->setValue(0);
  for(int i = 0; i < names.size(); i++)
  {
    positionDisplay(names.at(i));
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();
  }
  statusBar()->removeWidget(pBar);
  positionT->setIcon(QPixmap(":/images/treeBar/position_sel.png"));
}
void MainWindow::position_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove sphere
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_sphere";
    m_vis->removeShape (Cname.str());

  }
  disconnect(positionT,SIGNAL(triggered()),this,SLOT(position_HideAll()));
  connect(positionT,SIGNAL(triggered()),this,SLOT(position_DisplayAll()));
  positionT->setIcon(QPixmap(":/images/treeBar/position.png"));
  qvtkwidget->update();
}
void MainWindow::positionHT()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  QStringList names2;
  names2 << get_terrainNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Compute tree position using two estimated ring and terrain.");
  in->set_path(Proj->get_Path());
  in->set_description("Method for estimating tree position. Position is computed as a median of points lying in horizontal distance up to 60 cm from lowest point in cloud."
                        " Position is displayed as a sphere with centre at position and with radius 5 cm.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_inputCloud2("Input Terrain cloud:",names2);
  in->set_inputCheckBox("Recalculate parameters (DBH cloud, Height) based on tree position?");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(160,16);
    // position of progress bar should be extreme right
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 1; i < names.size(); i++ )
      {
        Proj->set_treePositionHT(names.at(i),Proj->get_TerrainCloud(in->get_inputCloud2()));
        positionDisplay(names.at(i));
        if(in->get_CheckBox()==true)
        {
          Proj->set_treeDBHCloud(names.at(i));
          Proj->set_treeheigth(names.at(i));
        }
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->set_treePositionHT(in->get_inputCloud1(),Proj->get_TerrainCloud(in->get_inputCloud2()));
      positionDisplay(in->get_inputCloud1());
      if(in->get_CheckBox()==true)
      {
        Proj->set_treeDBHCloud(in->get_inputCloud1());
        Proj->set_treeheigth(in->get_inputCloud1());
      }
      pBar->setValue(100);
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::skeleton()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Tree skeleton");
  in->set_path(Proj->get_Path());
  in->set_description("Compute tree skeleton with given resolution. Computing is based on L1-median skeleton algorithm.");
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
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 1; i < names.size(); i++ )
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
//compute skeleton
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
        QString n = in->get_inputCloud1().remove(".pcd");
        QString name = QString("%1_skeleton.pcd").arg(n);
        Cloud *cll = new Cloud(skel->get_skeleton(),name);

        Proj->set_skeleton(names.at(i),*cll);

        skel->graph();
      skeletonDisplay(names.at(i));
      // save_skel
        QString newFile = QString("%1\\%2.skel").arg(Proj->get_Path()).arg(n);
        skel->save_skel(newFile);
        //display line
        for(int k = 0; k < skel->get_start()->points.size(); k++)
        {
          std::stringstream namea;
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
delete in;
}
void MainWindow::skeletonDisplay(QString name)
{
  //display cloud
  QColor col = Proj->get_TreeCloud(name).get_color();
  dispCloud(Proj->get_TreeCloud(name).get_skeleton(),col.red(),col.green(), col.blue());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, Proj->get_TreeCloud(name).get_skeleton().get_name().toUtf8().constData());

  //display lines

}
void MainWindow::skeleton_DisplayAll()
{
//skeletonT->setIcon(QPixmap(":/images/treeBar/skeleton_sel.png"));
}
void MainWindow::skeleton_HideAll()
{
//skeletonT->setIcon(QPixmap(":/images/treeBar/skeleton.png"));
}
void MainWindow::stemCurvature()
{
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Tree stem curvature");
  in->set_path(Proj->get_Path());
  in->set_description("Compute and display tree ring at given heights.");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    if(in->get_inputCloud1() == "All_trees" )
    {
     // #pragma omp parallel for
      for(int i = 1; i < names.size(); i++ )
      {
        Proj->set_treeStemCurvature(names.at(i));

        stemCurvatureDisplay(names.at(i));
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      Proj->set_treeStemCurvature(in->get_inputCloud1());

      stemCurvatureDisplay(in->get_inputCloud1());
      pBar->setValue(100);
      pBar->update();
    }
    stemCurveT->setEnabled(true);
  }
  statusBar()->removeWidget(pBar);
  delete in;
}
void MainWindow::stemCurvatureDisplay(QString name)
{
  std::vector<stred> streds = Proj->get_TreeCloud(name).get_stemCurvature();

  for(int i =0; i <streds.size(); i++)
  {
    if(streds.at(i).r < 1 || streds.at(i).r >500 )
      continue;
    //Coeff
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
    coef->values.push_back((float)streds.at(i).a);
    coef->values.push_back((float)streds.at(i).b);
    coef->values.push_back((float)streds.at(i).z-0.5);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0.1);
    coef->values.push_back((float)streds.at(i).r/100);
    std::stringstream Cname ;
    Cname << name.toUtf8().constData() << "_ST_"<< i ;
    m_vis->removeShape(Cname.str());
    m_vis->addCylinder(*coef,Cname.str());

    coef.reset();
  }
  disconnect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_DisplayAll()));
  connect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_HideAll()));
  stemCurveT->setIcon(QPixmap(":/images/treeBar/stemCurve_sel.png"));
}
void MainWindow::stemCurvature_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    stemCurvatureDisplay(names.at(i));
  }
}
void MainWindow::stemCurvature_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int j = 0; j < names.size(); j++)
  {
    std::vector<stred> streds = Proj->get_TreeCloud(names.at(j)).get_stemCurvature();
    for(int i =0; i <streds.size(); i++)
    {
      std::stringstream Cname ;
      Cname << names.at(j).toUtf8().constData() << "_ST_"<< i ;
      m_vis->removeShape(Cname.str());
    }
  }
  disconnect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_HideAll()));
  connect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_DisplayAll()));
  stemCurveT->setIcon(QPixmap(":/images/treeBar/stemCurve.png"));
  qvtkwidget->update();
}
void MainWindow::stemCurvatureExport()
{
// for given tree
// write all ring into file in format: tree_name first row diameter second row X third Y fourth Z
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Tree stem curvature Export");
  in->set_path(Proj->get_Path());
  in->set_description("Exports computed ");
  in->set_inputCloud1("Input Tree cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    //vybrat jmeno noveho souboru
    QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("files (*.txt)"));
   //zapisovat jednotlive radky
    QFile file (newFile);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 1; i < names.size(); i++)
      {

        std::vector<stred> streds = Proj->get_TreeCloud(names.at(i)).get_stemCurvature();
        out << names.at(i);
        for(int j = 0; j <streds.size(); j++)
        {
          out << " " << streds.at(j).r *2;
        }
        out << "\n";
        out << names.at(i);
        for(int j = 0; j <streds.size(); j++)
        {
          if (streds.at(j).a == -1)
            out << " " << streds.at(j).a;
          else
          out << " " << streds.at(j).a - Proj->get_Xtransform();
        }
        out << "\n";
        out << names.at(i);
        for(int j = 0; j <streds.size(); j++)
        {
          if (streds.at(j).a == -1)
            out << " " << streds.at(j).b;
          else
            out << " " << streds.at(j).b - Proj->get_Ytransform();
        }
        out << "\n";
        out << names.at(i);
        for(int j = 0; j < streds.size(); j++)
        {
          if (streds.at(j).a == -1)
            out << " " << streds.at(j).z;
          else
          out << " " << streds.at(j).z - Proj->get_Ztransform();
        }
        out << "\n";

        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      QString dia_line = in->get_inputCloud1();
      QString x_line = in->get_inputCloud1();
      QString y_line = in->get_inputCloud1();
      QString z_line = in->get_inputCloud1();
      std::vector<stred> streds = Proj->get_TreeCloud(in->get_inputCloud1()).get_stemCurvature();
      for(int i =0; i <streds.size(); i++)
      {
        QString dia = QString(" %1").arg(streds.at(i).r *2);
        QString x = QString(" %1").arg(QString::number(streds.at(i).a - Proj->get_Xtransform()));
        QString y = QString(" %1").arg(QString::number(streds.at(i).b - Proj->get_Ytransform()));
        QString z = QString(" %1").arg(QString::number(streds.at(i).z - Proj->get_Ztransform()));
        dia_line.append(dia);
        x_line.append(x);
        y_line.append(y);
        z_line.append(z);
      }
      out << dia_line << "\n" << x_line << "\n" << y_line << "\n" << z_line << "\n" ;
      pBar->setValue(100);
      pBar->update();
    }
    file.close();
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud)
{
  QString name = QInputDialog::getText(this, tr("Name of new tree File"),tr("e.g. tree_id"));

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
        delete c;
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
      delete c;
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
  delete c;
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
    {
      //delete exist file
      QFile::remove(path);
      // write file
      Proj->save_Cloud(s_cloud->get_name(),s_cloud->get_Cloud());
      // assing new cloud to the old one
      Proj->set_VegeCloud(s_cloud->get_name(), s_cloud->get_Cloud());
    }
    if(overwrt == true && !file.exists())
    {
      Proj->save_newCloud("vege",s_cloud->get_name(),s_cloud->get_Cloud());
      QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
      openVegeFile(path);
    }
    if(overwrt == false && !file.exists())
    {
      Proj->save_newCloud("vege",s_cloud->get_name(),s_cloud->get_Cloud());
      QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(s_cloud->get_name());
      openVegeFile(path);
    }
    if(overwrt == false && file.exists())
    {
      QString name = QInputDialog::getText(this, tr("File exist"),tr("File exist in the project.\nPlease enter new name for the file."));
      Proj->save_newCloud("vege",name,s_cloud->get_Cloud());
      QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name);
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
    delete ii;
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
    delete ee;
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
  cloud_zbytek.reset();
  cloud_vege.reset();
  indexVector.reset();
  indices.reset();
}
//MISCELLANEOUS
void MainWindow::plusCloud()
{
  QStringList types;
  types << "Base cloud" << "Terrain cloud" << "Vegetation cloud" <<"Tree" << "Other";

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud merge");
  in->set_path(Proj->get_Path());
  in->set_description("Merge two clouds into new one.");
  in->set_inputCloud1("1. input cloud:",get_allNames());
  in->set_inputCloud2("2. input cloud:",get_allNames());
  in->set_outputCloud1("Output cloud :","cloud");
  in->set_outputType("Type of the output cloud:", types);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
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
  delete in;
}
void MainWindow::plusCloud(QString input1, QString input2,QString output, QString typ = "ost")
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cl (new pcl::PointCloud<pcl::PointXYZI>);
  *cl = *Proj->get_Cloud(input1).get_Cloud() + *Proj->get_Cloud(input2).get_Cloud();
  Proj->save_newCloud(typ,output,cl);
  cl.reset();
}

void MainWindow::minusCloud()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud subtraction");
  in->set_path(Proj->get_Path());
  in->set_description("Remove same points from bigger cloud. The function compare two input clouds. From the bigger one remove all common points and save the rest into a new file. ");
  in->set_inputCloud1("1. input cloud:",get_allNames());
  in->set_inputCloud2("2. input cloud:",get_allNames());
  in->set_outputCloud1("Output cloud:","cloud_subtraction");
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

    cloud_1.reset();
    cloud_2.reset();
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

    cloud_out.reset();
    delete c1;
    delete c2;
    delete cV;
  }
  delete in;
}
void MainWindow::voxelize()
{

  InputDialog *in = new InputDialog(this);
  in->set_title("Voxelize input cloud");
  in->set_path(Proj->get_Path());
  in->set_description("Make voxelized cloud. Resulting cloud has points only in centroids of boxes with resolution where was present at least one point.");
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

    cloud_filtered.reset();
  }
  delete in;
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
  delete in;
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
  delete in;
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
  cloud1.reset();
  cloud2.reset();
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

void MainWindow::set_ConcaveCloud()
{
QStringList names;
  names << get_allNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud boundaries");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing concave hull of given tree."
                        " This method serve only for display estimated polygon of concave hull.");
  in->set_inputCloud1("Input cloud:",get_allNames());
  in->set_outputCloud1("Output cloud name:","hull");
  in->set_inputInt("Input Maximal Edge length cm:","150");
  in->set_stretch();

  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    QString name = Proj->get_Cloud(in->get_inputCloud1()).get_name();
    QColor color = Proj->get_Cloud(in->get_inputCloud1()).get_color();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud = Proj->get_Cloud(in->get_inputCloud1()).get_Cloud();
    Cloud *cl = new Cloud (Proj->set_ConcaveCloud(cloud,(float)in->get_intValue()/100,name,color));

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudHull(new pcl::PointCloud<pcl::PointXYZI>);
    cloudHull = cl->get_Cloud();

    Proj->save_newCloud("ost",in->get_outputCloud1(),cloudHull);
    QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud1());
    openOstFile(fullnameV);

    pBar->setValue(100);
    pBar->update();
    cloud.reset();
    cloudHull.reset();

  }
  statusBar()->removeWidget(pBar);
  concaveT->setEnabled(true);
}
void MainWindow::set_ConvexCloud()
{
QStringList names;
  names << get_allNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud boundaries");
  in->set_path(Proj->get_Path());
  in->set_description("Method for computing convex hull of given tree."
                        " This method serve only for display estimated polygon of convex hull. ");
  in->set_inputCloud1("Input cloud:",get_allNames());
  in->set_outputCloud1("Output cloud name:","hull");
  in->set_stretch();

  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    QString name = Proj->get_Cloud(in->get_inputCloud1()).get_name();
    QColor color = Proj->get_Cloud(in->get_inputCloud1()).get_color();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud = Proj->get_Cloud(in->get_inputCloud1()).get_Cloud();
        Cloud *cl = new Cloud (Proj->set_ConvexCloud(cloud,name,color));

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudHull(new pcl::PointCloud<pcl::PointXYZI>);
        cloudHull = cl->get_Cloud();

        Proj->save_newCloud("ost",in->get_outputCloud1(),cloudHull);
        QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(in->get_outputCloud1());
        openOstFile(fullnameV);

      pBar->setValue(100);
      pBar->update();
      cloud.reset();
      cloudHull.reset();
    }
    statusBar()->removeWidget(pBar);
    concaveT->setEnabled(true);
}
void MainWindow::save_tiff()
{
  QString filename = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("TIFF files (*.tiff)"));
  if(!filename.isEmpty())
  {
    vtkSmartPointer<vtkRenderWindow> renderWindow = m_vis->getRenderWindow();
    renderWindow->SetOffScreenRendering(1);


    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
    vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->SetMagnification(3);

    vtkSmartPointer<vtkTIFFWriter> tiffWriter =
    vtkSmartPointer<vtkTIFFWriter>::New();
    tiffWriter->SetFileName(filename.toUtf8().constData());
    tiffWriter->SetCompressionToDeflate ();
    tiffWriter->SetInputConnection(windowToImageFilter->GetOutputPort());
    tiffWriter->Write();

    renderWindow->SetOffScreenRendering(0);
    qvtkwidget->update();

    //windowToImageFilter->Delete();
  //  tiffWriter->Delete();
  }
}
void MainWindow::bgColor()
{
  QColor c = QColorDialog::getColor(Qt::white,this);
  if(c.isValid())
  {
    m_vis->setBackgroundColor(c.redF(),c.greenF(),c.blueF()); // Background color
  }
}
//ACTION and MENUS
void MainWindow::createActions()
{
//FILE
  new_projectAct = new QAction(QPixmap(":/images/projectBar/new.png"), tr("New Project"), this);
  new_projectAct->setStatusTip(tr("Create new project."));
  connect(new_projectAct, SIGNAL(triggered()), this, SLOT(newProject()));

  open_projectAct = new QAction(QPixmap(":/images/projectBar/open.png"), tr("Open Project"), this);
  open_projectAct->setStatusTip(tr("Open existing project."));
  connect(open_projectAct, SIGNAL(triggered()), this, SLOT(openProject()));

  close_projectAct = new QAction(QPixmap(":/images/projectBar/close.png"), tr("Close Project"), this);
  close_projectAct->setStatusTip(tr("Close project."));
  connect(close_projectAct, SIGNAL(triggered()), this, SLOT(closeProject()));

  import_projectAct = new QAction(QPixmap(":/images/projectBar/import.png"), tr("Import project"), this);
  import_projectAct->setStatusTip(tr("Import of existing project into new folder."));
  connect(import_projectAct, SIGNAL(triggered()), this, SLOT(importProject()));

  importBaseAct = new QAction(tr("Import Base cloud"), this);
  importBaseAct->setStatusTip(tr("Import new Base cloud into project. Various formats are available."));
  connect(importBaseAct, SIGNAL(triggered()), this, SLOT(importBaseCloud()));

  importTerenAct = new QAction(tr("Import Terrain cloud"), this);
  importTerenAct->setStatusTip(tr("Import new Terrain cloud into project. Various formats are available."));
  connect(importTerenAct, SIGNAL(triggered()), this, SLOT(importTerrainFile()));

  importVegeAct = new QAction(tr("Import Vegetation cloud"), this);
  importVegeAct->setStatusTip(tr("Import new VEgetation cloud into project. Various formats are available."));
  connect(importVegeAct, SIGNAL(triggered()), this, SLOT(importVegeCloud()));

  importTreeAct = new QAction(tr("Import Tree cloud"), this);
  importTreeAct->setStatusTip(tr("Import new Tree cloud into project. Various formats are available."));
  connect(importTreeAct, SIGNAL(triggered()), this, SLOT(importTreeCloud()));

  exportTXTAct = new QAction(tr("Export cloud (txt)"), this);
  exportTXTAct->setStatusTip(tr("Export selected cloud into TXT file."));
  connect(exportTXTAct, SIGNAL(triggered()), this, SLOT(exportCloud()));

  exportPLYAct = new QAction(tr("Export cloud (ply)"), this);
  exportPLYAct->setStatusTip(tr("Export selected cloud into PLY file."));
  connect(exportPLYAct, SIGNAL(triggered()), this, SLOT(plysave()));

  exportPTSAct = new QAction(tr("Export cloud (pts)"), this);
  exportPTSAct->setStatusTip(tr("Export selected cloud into PTS file."));
  connect(exportPTSAct, SIGNAL(triggered()), this, SLOT(exportPts()));

  exportCONVEXAct = new QAction(tr("Export Tree convex projection (txt)"), this);
  exportPTSAct->setStatusTip(tr("Export convex planar projection of selected Tree."));
  connect(exportCONVEXAct, SIGNAL(triggered()), this, SLOT(exportConvexTxt()));

  exportCONCAVEAct = new QAction(tr("Export Tree concave projection (txt)"), this);
  exportCONCAVEAct->setStatusTip(tr("Export concae planar projection of selected Tree."));
  connect(exportCONCAVEAct, SIGNAL(triggered()), this, SLOT(exportConcaveTxt()));

  exitAct = new QAction(tr("Exit"), this);
  exitAct->setShortcuts(QKeySequence::Quit);
  exitAct->setStatusTip(tr("Terminate the application."));
  connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

//TERRAIN
  voxelAct = new QAction(tr("Terrain from voxels"), this);
  voxelAct->setStatusTip(tr("Automatic Terrain extraction method using voxelized derivate of input cloud. Result is voxelized."));
 // voxelAct->setEnabled(false);
  connect(voxelAct, SIGNAL(triggered()), this, SLOT(voxelgrid()));

  octreeAct = new QAction(tr("Terrain from OcTree"), this);
  octreeAct->setStatusTip(tr("Automatic Terrain extraction method using searching for lowest point in OcTree search. Result is origal points from input, but can be noisy and can be adjusted by maual adjustment."));
  //octreeAct->setEnabled(false);
  connect(octreeAct, SIGNAL(triggered()), this, SLOT(octreeSlot()));

  manualADAct = new QAction(tr("Manual adjustment"), this);
  manualADAct->setStatusTip(tr("Manual adjustment of selected terrain cloud. serves for deletion of non-ground points from cloud."));
 // manualADAct->setEnabled(false);
  connect(manualADAct, SIGNAL(triggered()), this, SLOT(manualAdjust()));

//VEGETATION

  manualSelAct = new QAction(tr("Manual tree selection"), this);
  manualSelAct->setStatusTip(tr("Manual selection of trees from vegetation cloud. User iteratively delete points that do not belong to the tree."));
  connect(manualSelAct, SIGNAL(triggered()), this, SLOT(manualSelect()));


//TREE ATRIBUTES
  tAAct = new QAction(tr("Export tree attributes"), this);
  tAAct->setStatusTip(tr("Export Tree attributes into new file. User can selected which attributes want export and how they are separated."));
  connect(tAAct, SIGNAL(triggered()), this, SLOT(treeAtributes()));

  dbhHTAct = new QAction(tr("DBH HT"), this);
  dbhHTAct->setStatusTip(tr("Compute and display DBH using Randomized Hough Transform method. It display cloud representing point for computing DBH and estimated cylinder with DBH value."));
  //dbhAct->setEnabled(false);
  connect(dbhHTAct, SIGNAL(triggered()), this, SLOT(dbhHT()));

  dbhCheckAct = new QAction(tr("DBH Check"), this);
  dbhCheckAct->setStatusTip(tr("Check the consistensy of computed DBH by both methods. and if they differ for more than 10 cm put them into list to check."));
  //dbhAct->setEnabled(false);
  connect(dbhCheckAct, SIGNAL(triggered()), this, SLOT(dbhCheck()));

  dbhLSRAct = new QAction(tr("DBH LSR"), this);
  dbhLSRAct->setStatusTip(tr("Compute and display DBH using Least Square Regression method. It display cloud representing point for computing DBH and estimated cylinder with DBH value."));
  //dbhAct->setEnabled(false);
  connect(dbhLSRAct, SIGNAL(triggered()), this, SLOT(dbhLSR()));

  heightAct = new QAction(tr("Height"), this);
  heightAct->setStatusTip(tr("Compute and display height of the tree. Displays line starting at tree position and follows Z axis to the height of highest point of tree."));
 // heightAct->setEnabled(false);
  connect(heightAct, SIGNAL(triggered()), this, SLOT(height()));

  posAct = new QAction(tr("Position"), this);
  posAct->setStatusTip(tr("Compute and display position of the tree. Displays sphere with diameter 10 cm and centre at tree position. If terrain is presented user can recalculate position according to terrain."));
  connect(posAct, SIGNAL(triggered()), this, SLOT(position()));

  posHTAct = new QAction(tr("Position HT"), this);
  posHTAct->setStatusTip(tr("Compute and display position of the tree. Displays sphere with diameter 10 cm and centre at tree position. If terrain is presented user can recalculate position according to terrain."));
  connect(posHTAct, SIGNAL(triggered()), this, SLOT(positionHT()));

  treeEditAct = new QAction(tr("Tree cloud edit"), this);
  treeEditAct->setStatusTip(tr("Edit tree cloud and save deleted parts in new file."));
  connect(treeEditAct, SIGNAL(triggered()), this, SLOT(treeEdit()));

  dbhEditAct = new QAction(tr("DBH cloud edit"), this);
  dbhEditAct->setStatusTip(tr("Edit tree cloud udsed for computing DBH."));
  connect(dbhEditAct, SIGNAL(triggered()), this, SLOT(dbhCloudEdit()));

  lengAct = new QAction(tr("Length"), this);
  lengAct->setStatusTip(tr("Compute and display length of the tree. Display line between two points with the greatest distance in cloud "));
  connect(lengAct, SIGNAL(triggered()), this, SLOT(length()));

  skeletonAct = new QAction(tr("Skeleton"), this);
  skeletonAct->setStatusTip(tr("Compute and display skeleton of tree. Dispaly connected parts of tree as lines. "));
  skeletonAct->setEnabled(false);
  connect(skeletonAct, SIGNAL(triggered()), this, SLOT(skeleton()));

  convexAct = new QAction(tr("Convex planar projection"), this);
  convexAct->setStatusTip(tr("Compute and display convex planar projection of tree. Displays polygon in color of tree with 50% opacity and with value of polygon area."));
  connect(convexAct, SIGNAL(triggered()), this, SLOT(convexhull()));

  concaveAct = new QAction(tr("Concave planar projection"), this);
  concaveAct->setStatusTip(tr("Compute and display concave planar projection of tree. Displays polygon in color of tree with 50% opacity and with value of polygon area."));
  connect(concaveAct, SIGNAL(triggered()), this, SLOT(concavehull()));

  stemCurvatureAct = new QAction(tr("Stem Curvature"), this);
  stemCurvatureAct->setStatusTip(tr("stem curvature"));
  connect(stemCurvatureAct, SIGNAL(triggered()), this, SLOT(stemCurvature()));

  exportStemCurvAct = new QAction(tr("Export Stem Curvature"), this);
  exportStemCurvAct->setStatusTip(tr("Export stem curvature rings into text file"));
  connect(exportStemCurvAct, SIGNAL(triggered()), this, SLOT(stemCurvatureExport()));

//MISC
  plusAct = new QAction(tr("Cloud merge"), this);
  plusAct->setStatusTip(tr("Merge two clouds into single one and save result as a new cloud with desired type of cloud."));
  connect(plusAct, SIGNAL(triggered()), this, SLOT(plusCloud()));

  voxAct = new QAction(tr("Voxelize cloud"), this);
  voxAct->setStatusTip(tr("Create voxelized cloudfrom selected cloud with given resolution and save it into new file."));
  connect(voxAct, SIGNAL(triggered()), this, SLOT(voxelize()));

  IDWAct = new QAction(tr("IDW"), this);
  IDWAct->setStatusTip(tr("Method only for special purpose! Serve as a computation of Inverse Distance Weight. Weight are computed from diff of reference terrain and ground point and apply to cloud. "));
  connect(IDWAct, SIGNAL(triggered()), this, SLOT(IDW()));

  clipedAct = new QAction(tr("Clip"), this);
  clipedAct->setStatusTip(tr("Method only for special purpose! Serve as selection from cloud strip when displayed two cloud. BUGGY!  "));
  connect(clipedAct, SIGNAL(triggered()), this, SLOT(clip()));

  minusAct = new QAction(tr("Cloud Subtraction"), this);
  minusAct->setStatusTip(tr("Extract common points from bigger cloud and save. "));
  connect(minusAct, SIGNAL(triggered()), this, SLOT(minusCloud()));

  convexCloudAct = new QAction(tr("Create convex hull"), this);
  minusAct->setStatusTip(tr("Compute and display concave planar projection of selected cloud. Saved as new cloud."));
  connect(convexCloudAct, SIGNAL(triggered()), this, SLOT(set_ConvexCloud()));

  concaveCloudAct = new QAction(tr("Create concave hull"), this);
  minusAct->setStatusTip(tr("Compute and display concave planar projection of selected cloud. Saved as new cloud."));
  connect(concaveCloudAct, SIGNAL(triggered()), this, SLOT(set_ConcaveCloud()));

  tiffAct = new QAction(tr("Save into tiff"), this);
  minusAct->setStatusTip(tr("Save visualized clouds into tiff file."));
  connect(tiffAct, SIGNAL(triggered()), this, SLOT(save_tiff()));

  bgcolorAct = new QAction(tr("Change bg color"), this);
  minusAct->setStatusTip(tr("background color."));
  connect(bgcolorAct, SIGNAL(triggered()), this, SLOT(bgColor()));
//ABOUT
  aboutAct = new QAction(tr("&About"), this);
  aboutAct->setStatusTip(tr("Show information about 3D Forest application."));
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
  importMenu->addAction(importBaseAct);
  importMenu->addAction(importTerenAct);
  importMenu->addAction(importVegeAct);
  importMenu->addAction(importTreeAct);
  exportMenu = fileMenu->addMenu(tr("Export"));
  exportMenu->addAction(exportTXTAct);
  exportMenu->addAction(exportPLYAct);
  exportMenu->addAction(exportPTSAct);
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
  treeMenu->addAction(dbhEditAct);
  treeMenu->addAction(dbhCheckAct);
  treeMenu->addSeparator();
  treeMenu->addAction(posAct);
  treeMenu->addAction(posHTAct);
  treeMenu->addAction(dbhHTAct);
  treeMenu->addAction(dbhLSRAct);
  treeMenu->addAction(heightAct);
  treeMenu->addAction(lengAct);
  treeMenu->addAction(convexAct);
  treeMenu->addAction(concaveAct);
  treeMenu->addAction(stemCurvatureAct);
  treeMenu->addAction(skeletonAct);
  treeMenu->addSeparator();
  treeMenu->addAction(tAAct);
  treeMenu->addAction(exportCONVEXAct);
  treeMenu->addAction(exportCONCAVEAct);
  treeMenu->addAction(exportStemCurvAct);
//MISC
  miscMenu = menuBar()->addMenu(tr("Other features"));
  miscMenu->addAction(plusAct);
  miscMenu->addAction(minusAct);
  miscMenu->addAction(voxAct);
  //miscMenu->addAction(IDWAct);
  //miscMenu->addAction(clipedAct);
  miscMenu->addAction(convexCloudAct);
  miscMenu->addAction(concaveCloudAct);
  miscMenu->addSeparator();
  miscMenu->addAction(tiffAct);
  miscMenu->addAction(bgcolorAct);

//ABOUT
  helpMenu = menuBar()->addMenu(tr("&About"));
  helpMenu->addAction(aboutAct);
 }
void MainWindow::createToolbars()
{
//Project toolbar
  QToolBar *Projectbar = addToolBar("Project toolbar");
  Projectbar->setMaximumHeight(24);
  Projectbar->resize(20,80);
  Projectbar->setIconSize(QSize(16,16));
  QAction *newProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/new.png"),"New Project");
  connect(newProjectT,SIGNAL(triggered()),this,SLOT(newProject()));
  QAction *openProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/open.png"),"Open Project");
  connect(openProjectT,SIGNAL(triggered()),this,SLOT(openProject()));
  QAction *closeProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/close.png"),"Close Project");
  connect(closeProjectT,SIGNAL(triggered()),this,SLOT(closeProject()));
  QAction *importProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/import.png"),"Import Project");
  connect(importProjectT,SIGNAL(triggered()),this,SLOT(importProject()));


// Tree toolbar
  treeBar = addToolBar("Tree toolbar");
  treeBar->setMaximumHeight(24);
  treeBar->setIconSize(QSize(16,16));

  positionT  = new QAction(QPixmap(":/images/treeBar/position.png"),"Display/Hide Tree position",0);
  treeBar->addAction(positionT);
  connect(positionT,SIGNAL(triggered()),this,SLOT(position_DisplayAll()));

  dbhthT  = new QAction(QPixmap(":/images/treeBar/dbhHT.png"),"Display/Hide DBH Hough Transform",0);
  treeBar->addAction(dbhthT);
  connect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_DisplayAll()));

  dbhlsrT  = new QAction(QPixmap(":/images/treeBar/dbhLSR.png"),"Display/Hide DBH Least Square Regression",0);
  treeBar->addAction(dbhlsrT);
  connect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_DisplayAll()));

  heightT  = new QAction(QPixmap(":/images/treeBar/height.png"),"Display/Hide Tree height",0);
  treeBar->addAction(heightT);
  connect(heightT,SIGNAL(triggered()),this,SLOT(height_DisplayAll()));

  lengthT  = new QAction(QPixmap(":/images/treeBar/length.png"),"Display/Hide Tree length",0);
  treeBar->addAction(lengthT);
  connect(lengthT,SIGNAL(triggered()),this,SLOT(length_DisplayAll()));

  convexT  = new QAction(QPixmap(":/images/treeBar/convex.png"),"Display/Hide convex projection",0);
  treeBar->addAction(convexT);
  convexT->setEnabled(false);
  connect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_DisplayAll()));

  concaveT  = new QAction(QPixmap(":/images/treeBar/concave.png"),"Display/Hide concave projection",0);
  treeBar->addAction(concaveT);
  concaveT->setEnabled(false);
  connect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_DisplayAll()));

  stemCurveT  = new QAction(QPixmap(":/images/treeBar/stemCurve.png"),"Display/Hide tree skeleton",0);
  treeBar->addAction(stemCurveT);
  stemCurveT->setEnabled(false);
  connect(stemCurveT,SIGNAL(triggered()),this,SLOT(skeleton_DisplayAll()));

  skeletonT  = new QAction(QPixmap(":/images/treeBar/skeleton.png"),"Display/Hide tree skeleton",0);
  treeBar->addAction(skeletonT);
  skeletonT->setEnabled(false);
  connect(skeletonT,SIGNAL(triggered()),this,SLOT(skeleton_DisplayAll()));
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
   // int s = m_cloud->get_Cloud()->points.size();
    *m_cloud->get_Cloud() += *cloud;

    // smazat poslednich "x" bodu z m_cloud1
    *m_cloud1->get_Cloud()->points.erase(m_cloud1->get_Cloud()->points.end() - num, m_cloud1->get_Cloud()->points.end());
    // smazat posledni zaznam v undopoint
    undopoint.pop_back();
    //zobrazit
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    cloud.reset();
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

  if(event.getPointsIndices(inl->indices)== false || inl->indices.size() == 0 )
  {
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    return;
  }

 if(m_cloud->get_Cloud()->points.size() > 0 && event.getPointsIndices(inl->indices)== true)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
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
    cloud1.reset();
    cloud2.reset();
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
  }
  inl.reset();
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
QString MainWindow::name_Exists()
{
  QString newName = QInputDialog::getText(this, tr("Name of new Cloud file"),tr("Please enter name for new cloud (without spaces)"));
  if(newName.isEmpty())
  {
    QMessageBox::warning(this,tr("Error"),tr("You forgot to fill name of new tree cloud"));
    name_Exists();
  }
  else
  {
    QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(newName);
    QFile file(path);
    if(file.exists())
    {
      //do you wish to rewrite existing file?
      QMessageBox::StandardButton rewrite = QMessageBox::question(this,tr("Overwrite file?"),tr("File with given name exist. Do you wish to overwrite file?"),QMessageBox::Yes|QMessageBox::No);
      if(rewrite == QMessageBox::Yes)
      {
        return newName;
      }
      else
      {
        return name_Exists();
      }
    }
    else
    {
      return newName;
    }
  }
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
  removeCloud(name);
  Proj->delete_Cloud(name);
}
void MainWindow::about()
{


QMessageBox::about(this,tr("about 3D Forest application"),tr(" 3D Forest application started as a part of Ph.D. thesis.\n"
                                                             " But now continue as a free platform for TLS data processing. \n\n"
                                                             "  AUTHORS:\n "
                                                             "\tJan Trochta j.trochta@gmail.com \n"
                                                             "\tMartin Krucek krucek.martin@gmail.com\n"
                                                             "\tKamil Kral kamil.kral@vukoz.cz"));
 }
void MainWindow::dispCloud(QString name)
{
  Cloud *c = new Cloud(Proj->get_Cloud(name));
  dispCloud(*c);
  delete c;
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


