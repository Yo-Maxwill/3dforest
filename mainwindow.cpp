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

//QT
#include <QtGui/QtGui>
#include <QVTKWidget.h>
#include <QtGui/QMessageBox>
#include <QtGui/QInputDialog>
#include <QtGui/QTreeWidget>
#include <QtGui/QColorDialog>
#include <QtCore/QSignalMapper>
#include <QtGui/QCheckBox>
#include <string>
#include <iostream>
#include <fstream>

#include "mainwindow.h"
#include <limits>
#include <cmath>
#include <cstdlib>
#include <omp.h>

#include <liblas/liblas.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>

//include BASE
#include <string>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//include INPUT OUTPUT
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <iostream>
//include VISUALIZATION
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/mouse_event.h>
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

//include SAMPLE CONSENSUS
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/sample_consensus/sac_model_cylinder.h>
//include SEGMENTATION
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/region_growing.h>
//#include <pcl/segmentation/conditional_euclidean_clustering.h>
//include FILTERS
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
//include FEATURES
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/features/vfh.h>
//#include <pcl/features/pfh.h>
//include SURFACE
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/mls.h>
//#include <pcl/surface/convex_hull.h>
//#include <pcl/surface/concave_hull.h>
//#include <pcl/surface/qhull.h>

#include <vtkWin32OpenGLRenderWindow.h>
////Visualizer
Visualizer::Visualizer(QString name)
{
  PCLVisualizer (name.toUtf8().constData());
}





////MAin Window

 MainWindow::MainWindow()
 : m_cloud(new Cloud()),
 m_cloud1(new Cloud()),
 m_vis (new Visualizer ("3D Viewer")),
 Proj (new Project())
{
//QVTK widget - visualizer
  qvtkwidget = new QVTKWidget;
  vtkSmartPointer<vtkRenderWindow> renderWindow = m_vis->getRenderWindow();
  qvtkwidget->SetRenderWindow(renderWindow);
//  m_vis->addOrientationMarkerWidgetAxes();
  m_vis->setShowFPS(false);
  //m_vis->setBackgroundColor(10,10,10);
  setCentralWidget(qvtkwidget);
  qvtkwidget->show();



// Tree widget - bocni lista
  treeWidget = new MyTree;
  QDockWidget *dockWidget = new QDockWidget(tr("Pointcloud Layers"), this);
  dockWidget->setAllowedAreas(Qt::LeftDockWidgetArea);
  dockWidget->setMaximumWidth(500);
  dockWidget->setWidget(treeWidget);
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget);

  QSettings settings("3D FOREST - phd thesis", "Application Example");
  QPoint pos = settings.value("pos", QPoint(50, 50)).toPoint();
  QSize size = settings.value("size", QSize(1024, 800)).toSize();
  resize(size);
  move(pos);

  createActions();
  createMenus();
  createToolBars();
  createStatusBar();

  m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
}

//PROJECT methods
void MainWindow::newProject()
{
//ZADAT JMENO NOVEHO PROJEKTU A ZALOZIT SLOZKU
  QString proj_name = QInputDialog::getText(this, tr("Set New Project Name"),
                                          tr("Name of the project:"), QLineEdit::Normal,
                                          tr("New Project"));
  if(proj_name.isEmpty())
    return;

  QString dirName = QFileDialog::getExistingDirectory(this,tr("CHOOSE BASE DIRECTORY FOR PROJECTS"),tr(""),QFileDialog::ShowDirsOnly);
  QString pathDir = QString("%1\\%2").arg(dirName).arg(proj_name);
  QDir myDir(pathDir);

  if(!myDir.exists())
    myDir.mkpath(".");

  else
  {
    QMessageBox *msgBox =  new QMessageBox(this);
    msgBox->setText("project exist");
    msgBox->setInformativeText("directory with the same name exist already. Do you wish to rewrite it?");
    msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox->setDefaultButton(QMessageBox::Yes);
    if(msgBox->exec() != QMessageBox::Yes)
    {
      QMessageBox::about(this,tr("do not ovewrite"),tr("project creation arborted"));
      return;
    }
  }

//NACIST TRANSFORMACNI MATICE
  std::vector<item> seznam;
  QFile fileproj ("projects.txt");
  fileproj.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileproj);
  while(!in.atEnd())
  {
    QString line = in.readLine();
    QStringList coords = line.split(" ");
    item data;

    data.name = coords.at(0);
    data.x = coords.at(1).toDouble();
    data.y = coords.at(2).toDouble();
    seznam.push_back(data);
  }
  fileproj.close();

//VYBRAT TRANSFORMACNI MATICE
  bool ok;
  QStringList items;
  for (int i=0;i <seznam.size();i++)
  {
    QString a = QString("%1").arg(seznam.at(i).name);
    items<<(a);
  }
  QString name = QInputDialog::getItem(this, tr("select your forest"),tr("Select transform matrix of project:\n or CANCEl for creating new:"), items, 0, false, &ok);
//NEBO DEFINOVAT NOVOU
  if(!ok)
  {
    name = QInputDialog::getText(this,tr("name of project"),tr("enter name of project:"));
    QMessageBox::information(this,"","coordinats are not set up, please select file with coordinates which will be used in transformation matrix");
    QString fileName = QFileDialog::getOpenFileName(this,tr("open file"),"",tr("files (*.txt *.xyz)"));

    QFile file (fileName);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&file);
    QString line = in.readLine();
    QStringList coords = line.split(" ");
    file.close();

  //zapat novy project do souboru projects!!!
    fileproj.open(QIODevice::Append | QIODevice::Text);
    QTextStream out(&fileproj);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(0);
    if(coords.at(0).toDouble()< 100 && coords.at(0).toDouble()> -100)
    {
      out << " \n"  <<name<< " "<< 0 <<" " << 0;
      Proj = new Project(0, 0, name.toUtf8().constData());
    }
    else
    {
      out <<" \n"  <<name<< " "<< ceilf(coords.at(0).toDouble() / 1000) * 1000 <<" " << ceilf(coords.at(1).toDouble() / 1000) * 1000;
      Proj = new Project(ceilf(coords.at(0).toDouble() / 1000) * 1000, ceilf(coords.at(1).toDouble() / 1000) * 1000, name.toUtf8().constData());
    }
    fileproj.close();
  }
  else
  {
    Proj = new Project(seznam.at(items.indexOf(name)).x,seznam.at(items.indexOf(name)).y,proj_name.toUtf8().constData());
  }
//ZALOZENI PROJEKTU

  QString fileN =QString("%1/proj.3df").arg(pathDir);
  Proj->set_path(pathDir);
  QFile file (fileN);
  file.open(QIODevice::WriteOnly);
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(0);
  out << proj_name <<" "<< Proj->get_Xtransform() << " " << Proj->get_Ytransform()<< " "<<  pathDir << "\n" ;
  file.close();
  QString a = QString("3DForest - %1 ").arg(proj_name);
  setWindowTitle(a);
}
void MainWindow::openProject()
{
//SET 3DF FILE
  QString fileName = QFileDialog::getOpenFileName(this,tr("open file"),"",tr("files (*.3df)"));
  if (fileName.isEmpty())
    return;

  closeProject();

  QFile file (fileName);
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
      Proj = new Project(coords.at(1).toDouble(),coords.at(2).toDouble(),coords.at(0).toUtf8().constData());
      Proj->set_path(coords.at(3).toUtf8().constData());
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
  qvtkwidget->update();
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
//IMPORT methods
void MainWindow::importtxt2()
{
  QStringList ls =QFileDialog::getOpenFileNames(this,tr("open file"),"",tr("files (*.txt *xyz)"));
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No file selected");
    return;
  }

  std::vector<QString> names;
  for(int i=0;i<ls.size(); i++)
  {
    QString fileName = ls.at(i);

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

        data.x = coords.at(0).toDouble()+Proj->get_Xtransform();
        data.y = coords.at(1).toDouble()+Proj->get_Ytransform();
        data.z = coords.at(2).toFloat();
        data.intensity=coords.at(3).toFloat();
        cloud->points.push_back(data);
      }
    }
    file.close();
    in.~QTextStream();
    cloud->width = cloud->points.size();
    cloud->is_dense=true;
    cloud->height=1;

    QStringList Fname = fileName.split("\\");
    QStringList name = Fname.back().split(".");

    Cloud *c =new Cloud(cloud,name.at(0));

    Proj->save_newCloud("cloud",name.at(0),cloud);

    QString fullname = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name.at(0));
    names.push_back(fullname);
    QMessageBox::information(this,("tt"),fullname);
  }
  for(int j; j < ls.size();j++)
  {
    //open new file
    openCloudFile(names.at(j));
  }
  QMessageBox::information(this,("tt"),("konec"));
}
void MainWindow::importtxt()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  QString fileName = QFileDialog::getOpenFileName(this,tr("open file"),"",tr("files (*.txt *xyz)"));
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
    while(!in.atEnd())
    {
      QString line = in.readLine();
      QStringList coords = line.split(" ");
      pcl::PointXYZI data;
      if(coords.size() >1)
      {
        data.x = coords.at(0).toDouble()+Proj->get_Xtransform();
        data.y = coords.at(1).toDouble()+Proj->get_Ytransform();
        data.z = coords.at(2).toFloat();
        data.intensity=0;
        cloud->points.push_back(data);
      }
    }
    file.close();
    in.~QTextStream();
    cloud->width = cloud->points.size ();
    cloud->is_dense=true;
    cloud->height=1;

    QStringList Fname = fileName.split("/");
    QStringList name = Fname.back().split(".");

    Cloud *c =new Cloud(cloud,name.at(0));
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

//EXPORT method
void MainWindow::exportCloud()
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
//EXIT method
void MainWindow::closeEvent(QCloseEvent *event)
{
  event->accept();
}

//TEREN methods
void MainWindow::voxelgrid()
{
 QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizebaseCV(); i++)
  {
    names << Proj->get_baseCloud(i).get_name();
  }
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select base cloud"), tr("cloud name:"), names, 0, false, &ok);

  if (ok && !item.isEmpty())
  {
    float res = QInputDialog::getDouble(this, tr("Voxel resolution"), tr("Enter size of voxel in m: "), 0.5,0,1000,5);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vox;

    vox.setInputCloud (Proj->get_Cloud(item).get_Cloud());
    vox.setLeafSize (res, res, res/2);
    vox.filter (*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  //vybrat jen spodni voxely
    cloud = lowPoints(cloud_filtered,res/2, true);



//save dem cloud
    QString nameT = QInputDialog::getText(this, tr("Name of Terrain File"),tr("e.g. terrain-vox"));
    if(!nameT.isEmpty())
    {
      Proj->save_newCloud("teren",nameT,cloud);
      QString fullnameT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(nameT);
      openTerrainFile(fullnameT);
    }
    else
    {
      QMessageBox::warning(this, tr("warning"),tr("terrain points will not be saved into file"));
    }




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
    QString nameV = QInputDialog::getText(this, tr("Name of Vegetation File"),tr("vege-vox"));
    if(!nameV.isEmpty())
    {
      cloud_vege->width = cloud_vege->points.size ();
      cloud_vege->is_dense=true;
      cloud_vege->height=1;

      Proj->save_newCloud("vege",nameV,cloud_vege);
      QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(nameV);
      openVegeFile(fullnameV);
    }
    else
    {
      QMessageBox::warning(this, tr("warning"),tr("Vegetation points will not be saved into file"));
    }
  }
}
void MainWindow::voxelstat()
{

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
void MainWindow::octreeSlot()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizebaseCV(); i++)
  {
    names << Proj->get_baseCloud(i).get_name();
  }
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select base cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty())
  {
    float res = QInputDialog::getDouble(this, tr("zadat rozliseni"), tr("velikost rozliseni octree v m:"), 0.1,0,1000000,3);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp4(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp5(new pcl::PointCloud<pcl::PointXYZI>);

//velky cyklus
    octree(res*10,Proj->get_Cloud(item).get_Cloud(),cloud_vege, cloud_tmp);
    octree(res/2,cloud_tmp,cloud_vege, cloud_tmp2);

//maly cyklus
    octree(res,Proj->get_Cloud(item).get_Cloud(),cloud_vege, cloud_tmp3);

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

    for(int i=0; i < Proj->get_Cloud(item).get_Cloud()->points.size();i++)
    {
      pcl::PointXYZI searchPointV;
      searchPointV=Proj->get_Cloud(item).get_Cloud()->points.at(i);
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


// SAVE CLOUDS
    QString nameT = QInputDialog::getText(this, tr("Name of Terrain File"),tr("e.g. terrain"));
    if(!nameT.isEmpty())
    {
      Proj->save_newCloud("teren",nameT,cloud_tmp5);
      QString fullnameT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(nameT);
      openTerrainFile(fullnameT);
    }
    else
    {
      QMessageBox::warning(this, tr("warning"),tr("terrain points will not be saved into file"));
    }

    QString nameV = QInputDialog::getText(this, tr("Name of Vegetation File"),tr("vege.pcd"));
    if(!nameV.isEmpty())
    {
      Cloud *cV = new Cloud(cloud_vege,nameV);
      Proj->set_VegeCloud(*cV);
      cloud_vege->width = cloud_vege->points.size ();
      cloud_vege->is_dense=true;
      cloud_vege->height=1;
      Proj->save_newCloud("vege",nameV,cloud_vege);
      QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(nameV);
      openVegeFile(fullnameV);
    }
    else
    {
      QMessageBox::warning(this, tr("warning"),tr("Vegetation points will not be saved into file"));
    }
  }
}
void MainWindow::manualAdjust()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
  {
    names << Proj->get_TerrainCloud(i).get_name();
  }
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select terrain cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty())
  {
    m_cloud->get_Cloud()->clear();
    //zkopirovat do m_cloud
    Cloud *c = new Cloud(Proj->get_Cloud(item).get_Cloud(),Proj->get_Cloud(item).get_name());
    m_cloud = c;
    m_cloud->set_Psize(Proj->get_Cloud(item).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    h_cloud->clear();
    Cloud *cl = new Cloud(h_cloud,Proj->get_Cloud(item).get_name());
    m_cloud1 = cl;
    m_cloud1->set_Psize(Proj->get_Cloud(item).get_Psize());

    //spustit editacni listu
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopE = editBar->addAction("Stop EDIT");
    connect(stopE,SIGNAL(triggered()),this,SLOT(manualAdjustStop()) );

    //connect areapicking event
   // m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->removeAllPointClouds();
    treeWidget->allItemOFF();
    dispCloud(*m_cloud,220,220,0);
  }
}
void MainWindow::manualAdjustStop()
{
   //save m_cloud1 as vege or add to existing cloud
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("Saving deleted parts");
	msgBox->setInformativeText("Do you want to save deleted parts as new file?");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

  m_vis->removeAllPointClouds();

	 if(msgBox->exec() == QMessageBox::Yes)
    {
      QString name = QInputDialog::getText(this, tr("Name of new File"),tr("e.g. other_cloud"));
      if(!name.isEmpty())
      {
        Cloud *c = new Cloud();
        *m_cloud1 = *c;
        Proj->set_OstCloud(*c);
        Proj->save_newCloud("ost",name,m_cloud1->get_Cloud());
        QString fullnameT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name);
        openCloudFile(fullnameT);
        delete editBar;
        m_cloud->get_Cloud()->clear();
        m_cloud1->get_Cloud()->clear();
      }
      else
      {
        QMessageBox::warning(this,tr("error"),tr("You forget to fill name of new cloud."));
        manualAdjustStop();
        return;
      }
    }
    else
    {
  // get names of all clouds into QStringList and make selection from it
      QStringList names;
      //vegeCloud
      for(int i = 0; i< Proj->get_sizevegeCV(); i++)
      {
        names << Proj->get_VegeCloud(i).get_name();
      }
      for(int i = 0; i< Proj->get_sizebaseCV(); i++)
      {
        names << Proj->get_baseCloud(i).get_name();
      }
      //terrainCloud
      for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
      {
        names << Proj->get_TerrainCloud(i).get_name();
      }

      //treeCloud
       for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
      {
        names << Proj->get_TreeCloud(i).get_name();
      }
      //ostCloud
       for(int i = 0; i< Proj->get_sizeostCV(); i++)
      {
        names << Proj->get_ostCloud(i).get_name();
      }

      bool ok;
      QString item = QInputDialog::getItem(this, tr("Select cloud for join"), tr("cloud name:"), names, 0, false, &ok);
      if (ok && !item.isEmpty())
      {
        // contencate cloud
        Cloud *c = new Cloud(Proj->get_Cloud(item));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());
        *cloud = *c->get_Cloud() + *m_cloud1->get_Cloud();
        c->set_Cloud(cloud);

        //delete item save new file open file
        QString path_out = QString ("%1\\%2").arg(Proj->get_Path()).arg(c->get_name());
        pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *c->get_Cloud());
        treeWidget->itemdelete(c->get_name());
      //opencloud
        openCloudFile(path_out);
      }
   //save m_cloud into old file
    QString path_out = QString ("%1\\%2").arg(Proj->get_Path()).arg(m_cloud->get_name());
    pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *m_cloud->get_Cloud());
    treeWidget->itemdelete(m_cloud->get_name());
  //opencloud
    openCloudFile(path_out);
    delete editBar;
    m_cloud->get_Cloud()->clear();
    m_cloud1->get_Cloud()->clear();
  }
}

//VEGETATION
void MainWindow::manualSelect()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select vegetation cloud for tree extraction"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty())
  {
    //zkopirovat do m_cloud
    Cloud *c = new Cloud(Proj->get_Cloud(item).get_Cloud(),Proj->get_Cloud(item).get_name());
    m_cloud = c;
    m_cloud->set_Psize(Proj->get_Cloud(item).get_Psize());

    //zkopirovat do m_cloud1
    pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    Cloud *cl = new Cloud(h_cloud,Proj->get_Cloud(item).get_name());
    m_cloud1 = cl;
    m_cloud1->set_Psize(Proj->get_Cloud(item).get_Psize());

    //spustit editacni listu

    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopEd = editBar->addAction("Stop EDIT");

    connect(stopEd,SIGNAL(triggered()),this,SLOT(manualSelectStop()) );

    //connect areapicking event
   // m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );

    m_vis->removeAllPointClouds();
// naradit za displcloud
    treeWidget->allItemOFF();
    dispCloud(*m_cloud,220,220,0);
  }
}
void MainWindow::manualSelectStop()
{
// save m_cloud as new tree
  saveTreeCloud(m_cloud->get_Cloud());

  //tmp file
  QString pathTMP = QString("%1/cloud_tmp.pcd").arg(Proj->get_Path());
  pcl::io::savePCDFileBinaryCompressed(pathTMP.toUtf8().constData(),*m_cloud1->get_Cloud());
  //treeWidget->allItemOFF();

//ask if continue
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("Continue editting?");
	msgBox->setInformativeText("Do you want to continue editting the cloud?");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox->setDefaultButton(QMessageBox::Yes);

  if(msgBox->exec() == QMessageBox::Yes)
  {
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

    m_vis->removeAllPointClouds();
    saveOstCloud(m_cloud1->get_Cloud());
    m_cloud->get_Cloud()->clear();
    m_cloud1->get_Cloud()->clear();
    delete editBar;
  }
}
void MainWindow::seg_dist()
{
//vybrat cloud
  QStringList names;
  // vybrat cloud vegetace
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select vegetation cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty())
  {
    float res = QInputDialog::getDouble(this, tr("resolution"), tr("enter resolution in cm:"),10,0,10000,1);
    QString prefix = QInputDialog::getText(this,tr("prefix of files"),tr("please enter prefix (e.g ID ) of created file:"));
    //zkopirovat do m_cloud
    Cloud *cl = new Cloud(Proj->get_Cloud(item));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_main (cl->get_Cloud());

//vybrat jen body do vysky 1,5 m a zbytek ulozit bokem
    //vybrat teren
    QStringList tereny;
    for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
    {
      tereny << Proj->get_TerrainCloud(i).get_name();
    }
    bool okk;
    QString teren = QInputDialog::getItem(this, tr("select terrain"), tr("cloud name:"), tereny, 0, false, &okk);
    if (teren.isEmpty())
    {
      QMessageBox::warning(this, tr("error"), tr("You do not select terrain operation canceled"));
      return;
    }
    Cloud *clt = new Cloud(Proj->get_Cloud(teren));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_teren (clt->get_Cloud());
    // vybrat body
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vyrez (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rest (new pcl::PointCloud<pcl::PointXYZI>);
    kdtree.setInputCloud (cloud_teren);

    #pragma omp parallel for
    for(int i=0; i < cloud_main->points.size();i++)
    {
      pcl::PointXYZI searchPointV;
      searchPointV=cloud_main->points.at(i);
      std::vector<int> pointIDv(1);
      std::vector<float> pointSDv(1);
  //#pragma omp critical
      if(kdtree.nearestKSearch(searchPointV,1,pointIDv,pointSDv) > 0 && (searchPointV.z - cloud_teren->points[ pointIDv.at(0) ].z) < 1.3)
      {
        pcl::PointXYZI in;
        in.x = searchPointV.x;
        in.y = searchPointV.y;
        in.z = searchPointV.z;
        #pragma omp critical
        cloud_vyrez->points.push_back(in);
      }
      else
      {
        pcl::PointXYZI in;
        in.x = searchPointV.x;
        in.y = searchPointV.y;
        in.z = searchPointV.z;
        #pragma omp critical
        cloud_rest->points.push_back(in);
      }
    }
    cloud_vyrez->width = cloud_vyrez->points.size ();
    cloud_vyrez->is_dense=true;
    cloud_vyrez->height=1;
//segmentace na jednotlive objekty
  // zacit segmentaci
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_vyrez);
    // samotna segmentace
    ec.setClusterTolerance (res/100); // m
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (10000000);// max velikost clusteru
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_vyrez);
    ec.extract (cluster_indices); //uklada se do vectoru
    //#pragma omp parallel for
    int c =0;
    for( c ; c< cluster_indices.size(); c++)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointIndices it = cluster_indices.at(c);
      //ulozeni do cloud_clusteru
      for (int t=0; t < it.indices.size(); t++) //vem kazdou indice co obsahuje pole indices
      {
        cloud_cluster->points.push_back (cloud_vyrez->points.at(it.indices.at(t))); // prirad ji do cloudu
        inliers->indices.push_back(it.indices.at(t));
      }
      //ulozeni do souboru
      QString name =QString("%1_%2").arg(prefix).arg(c);
      saveTreeCloud(cloud_cluster,name,false);
    }
  //prida zbytky do cloud_rest
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_zbytek (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> ext;
    // Extract the inliers
    ext.setInputCloud (cloud_vyrez);
    ext.setIndices (inliers);
    ext.setNegative (true);
    ext.filter (*cloud_zbytek);
    *cloud_rest= *cloud_rest + *cloud_zbytek;

// pro kazdy bod ve zbytku spocitat dva nejblizsi objekty

   //pro kazdy bod
   #pragma omp parallel for
   for(int i=0; i < cloud_rest->points.size();i++)
   {
      pcl::PointXYZI searchPointV;
      searchPointV=cloud_rest->points.at(i);
     // pro kazdy object
      int n = -1;
      for(int j=0; j < Proj->get_sizeTreeCV(); j++)
      {
        Cloud *tree = new Cloud (Proj->get_TreeCloud(j));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tree (tree->get_Cloud());
        pcl::KdTreeFLANN<pcl::PointXYZI> kdt;
        kdt.setInputCloud (cloud_tree);// object cloud
        float dist = 1000;// distance to object

        std::vector<int> pointIDv(1);
        std::vector<float> pointSDv(1);
        // najde nejbizsi bod a urci jeho vzdalenost
        if(kdtree.nearestKSearch(searchPointV,1,pointIDv,pointSDv))
        {
          if(pointSDv.at(0) < dist)
          {
            dist = pointSDv.at(0);
            n = j;
          }
        }
      }
      if(n>-1)
      {
        //priradit bod k danemu objektu
        Cloud *tree = new Cloud (Proj->get_TreeCloud(n));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object(tree->get_Cloud());
        QString nameCloud = tree->get_name();
        #pragma omp critical
        cloud_object->points.push_back(searchPointV);
        Cloud *tree_new = new Cloud(cloud_object,nameCloud);
        Proj->set_TreeCloudat(n,*tree_new);
      }
    }
    //ulozit kazdy object
    for(int j=0; j < Proj->get_sizeTreeCV(); j++)
    {
      Cloud *tree = new Cloud (Proj->get_TreeCloud(j));
      saveTreeCloud(tree->get_Cloud(),tree->get_name(),true);
    }
  }
}

//TREE ATRIBUTES methods

void MainWindow::dbhCloudEdit()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty())
  {
    int i = names.indexOf(item);
    //zkopirovat do m_cloud
    Cloud *c = new Cloud(Proj->get_TreeCloud(i).get_dbhCloud().get_Cloud(), Proj->get_TreeCloud(i).get_name());
    m_cloud = c;
    m_cloud->set_Psize(Proj->get_TreeCloud(i).get_Psize());

    //zkopirovat do m_cloud1
    pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    Cloud *cl = new Cloud(h_cloud,Proj->get_TreeCloud(i).get_name());
    m_cloud1 = cl;
    m_cloud1->set_Psize(Proj->get_TreeCloud(i).get_Psize());


    //spustit editacni listu
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction("Stop EDIT");

    connect(stopTE,SIGNAL(triggered()),this,SLOT(dbhCloudStopEdit()) );
    //connect areapicking event
    //m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->removeAllPointClouds();
    m_vis->removeAllShapes();
    dispCloud(*m_cloud,220,220,0);
  }
}
void MainWindow::dbhCloudStopEdit()
{
  //get the Strom
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_dbhCloud().get_name();
  }

  int j;
  for(int i = 0; i< names.size(); i++)
  {
    if( names.at(i) == m_cloud->get_name())
      j = i;
  }
  // save m_cloud as old tree
  stred x = Proj->get_TreeCloud(j).set_dbhLSRALG(m_cloud->get_Cloud());
  stred xx = Proj->get_TreeCloud(j).set_dbhLSRGEOM(x,m_cloud->get_Cloud());
  Proj->set_treedbh(j,xx);

  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  delete editBar;
}
void MainWindow::treeAtributes()
{

//TODO: volit ktere parametry budou zapsany
  //prograssbar
  pBar = new QProgressBar(statusBar());
  // position of progress bar should be extreme right
  statusBar()->addWidget(pBar);
  pBar->setValue(0);


  //vytvorit soubor s vysledkama
  QString fileN =QString("%1/TREE_ATRIBUTES.txt").arg(Proj->get_Path());
  QFile file (fileN);
  file.open(QIODevice::Append | QIODevice::Text);

  QTextStream header(&file);
  header << "Cloud_name\tHeight\tX_coord_meter\tY_coord_meter\tZ_coord_meter\tX_coord_LSR\tY_coord_LSR\tDBH_LSR\tLenght\n";
  file.close();



  for(int i =0;i < Proj->get_sizeTreeCV(); i++)
  {
    Tree *c = new Tree(Proj->get_TreeCloud(i));

    if(c->get_dbh().r < 1)
      c->set_dbhLSR();
    //addcylinder
    c->set_lenght();

    float h = c->get_height();
    float z = c->get_pose().z;
    float len = c->get_lenght();

    double x,y,dbhLSRx,dbhLSRy;

    x = (c->get_pose().x - Proj->get_Xtransform());
    y = (c->get_pose().y - Proj->get_Ytransform());
        //DBH_LSR
    dbhLSRx = (c->get_dbh().a - Proj->get_Xtransform());
    dbhLSRy = (c->get_dbh().b - Proj->get_Ytransform());
    float dbhLSR = (c->get_dbh().r*2);
    //height
   // c->set_position();

    //file record
    file.open(QIODevice::Append | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);
    //header << "Cloud_name\tHeight\tX_coord_meter\tY_coord_meter\tZ_coord_meter\tX_coord_LSR\tY_coord_LSR\tDBH_LSR\n";

    if(Proj->get_Xtransform() < 0) //Velka ples
    {
       x *=-1;
       y *=-1;
       dbhLSRx *=-1;
       dbhLSRy *=-1;
    }
    out << Proj->get_TreeCloud(i).get_name() << "\t" << h <<  "\t"  << x <<  "\t" << y << "\t" << z <<"\t" << dbhLSRx <<"\t" << dbhLSRy<<"\t" << dbhLSR<<"\n";
    file.close();
    //progressbar
    pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
    pBar->update();

  }
}
void MainWindow::treeAtributesRead()
{
  Proj->readAtrs();

  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    stred x = Proj->get_TreeCloud(i).get_dbh();
    if (2000 > x.r >0 )
    {
// DBH
      //displ dbhCloud
      dispCloud(Proj->get_TreeCloud(i).get_dbhCloud(),255, 0, 0);

      //Coeff
      pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
      coef->values.push_back((float)x.a);
      coef->values.push_back((float)x.b);
      coef->values.push_back((float)Proj->get_TreeCloud(i).get_pose().z+1.25);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0.1);
      coef->values.push_back((float)x.r/100);
      std::stringstream name_dbh;
      name_dbh << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "cylinder";
      m_vis->addCylinder(*coef,name_dbh.str());

      //addtext3D with R
      pcl::PointXYZ bod;
      bod.x=x.a+(float)x.r/100;
      bod.y=x.b;
      bod.z=x.z+0.1;
      QString hdbh= QString("%1").arg(x.r*2);
      m_vis->addText3D(hdbh.toUtf8().constData(),bod,0.6,0.6,0.5,0);

//HEIGHT
      pcl::PointXYZI maxp,minp;
      maxp.z=0;
      minp.z=60000;

      for(int j = 0; j < Proj->get_TreeCloud(i).get_Cloud()->points.size();j++)
      {
        pcl::PointXYZI bod = Proj->get_TreeCloud(i).get_Cloud()->points.at(j);
        if(bod.z > maxp.z)
          maxp=bod;
        if(bod.z < minp.z)
          minp= bod;
      }
      std::stringstream name_h;
      name_h << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "line";
      m_vis->addLine(Proj->get_TreeCloud(i).get_pose(),maxp,name_h.str());
      QString h= QString("%1").arg(Proj->get_TreeCloud(i).get_height());
      m_vis->addText3D(h.toUtf8().constData(),maxp,0.6,0.6,0.5,0);
//POSITION
      pcl::ModelCoefficients::Ptr coefp (new pcl::ModelCoefficients ());
      coefp->values.push_back((float)Proj->get_TreeCloud(i).get_pose().x); //x
      coefp->values.push_back((float)Proj->get_TreeCloud(i).get_pose().y); //y
      coefp->values.push_back((float)Proj->get_TreeCloud(i).get_pose().z); //z
      coefp->values.push_back((float)0.05);   //r

      std::stringstream namep;
      namep << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_sphere";
      m_vis->addSphere(*coefp,namep.str());
    }
  }
}
void MainWindow::cylinderSeg()
{

  //set tree cloud names and select one
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
    names <<"all";
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty()&&item!="all")
  {
    int i = names.indexOf(item);
  // set as strom
    Tree *c = new Tree(Proj->get_TreeCloud(i));
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
    //coef = c->get_dbhC();

    pcl::PointXYZ bod;
    bod.x=coef->values[0]+0.5;
    bod.y=coef->values[1]+0.5;
    bod.z=coef->values[2];

    QString h= QString("%1").arg(ceil(coef->values[6]*200));
    m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0);

    std::stringstream name ;
    name << c->get_name().toUtf8().constData() << "cyl";
    m_vis->addCylinder(*coef,name.str());
    dispCloud(c->get_dbhCloud(),255,0,0);
  }
  if(ok && item == "all")
  {
    for(int j=0; j < names.size()-1; j++)
    {
    // set as strom
      Tree *c = new Tree(Proj->get_TreeCloud(j));
      pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
      //coef = Proj->get_TreeCloud(j).get_dbhC();

      pcl::PointXYZ bod;
      bod.x=coef->values[0]+0.5;
      bod.y=coef->values[1]+0.5;
      bod.z=coef->values[2];

      QString h= QString("%1").arg(ceil(coef->values[6]*200));
      m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0);

      std::stringstream name ;
      name << c->get_name().toUtf8().constData() << "cyl";
      m_vis->addCylinder(*coef,name.str());
      dispCloud(c->get_dbhCloud(),255,0,0);
    }
  }
}
int MainWindow::dbh (pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{

//MRACNO V 1,25-1,35 CM OD ZEME = H_CLOUD, proredit na nejnizsi body, bud voxel, nebo neco takoveho...
  pcl::PointCloud<pcl::PointXYZI>::Ptr dbh_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointXYZI minp,maxp;
  pcl::getMinMax3D(*input,minp,maxp);

  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = input->begin (); ith != input->end (); ith++)
  {
    if (ith->z > (minp.z + 1.27) && ith->z < (minp.z + 1.33) )
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      h_cloud->points.push_back(bod);
    }
  }

//VYBRA JEN NEJNIZSI BODY +- 1 CM  -mela by to byt funkce a ne tady vlozeno, mozna voxelizace pred?
#pragma omp parallel for
for(int i=0; i < h_cloud->points.size();i++)
 {
  pcl::PointXYZI ith = h_cloud->points.at(i);

  for(int j=0; j < h_cloud->points.size();j++)
   {
     pcl::PointXYZI ith2 = h_cloud->points.at(j);

     if(ith.x > ith2.x-0.005 &&  ith.x < ith2.x+0.005 && ith.y > ith2.y-0.005 &&  ith.y < ith2.y+0.005 && ith.z > ith2.z) //1 cm
      {
        pcl::PointXYZI bod;
        bod.x = ith.x;
        bod.y = ith.y;
        bod.z = ith.z;
        bod.intensity =1;
        #pragma omp critical
        dbh_cloud->points.push_back(bod);
        break;
      }
   }
 }


//URCIT HOUGH TRANSFORM A NEJVETSI R
  std::vector<stred> maxima;

  for(int r=50; r > 2; r--)
  {
    std::vector<stred> stredy;
    std::vector<stred> acc;

 //pocitani moznych stredu ulozeni do vectoru stredy
    for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = dbh_cloud->begin (); ith != dbh_cloud->end (); ith++)
    {
      for(float uhel =0; uhel <360; uhel ++)
      {
        float A,B;
        A = ith->x - ((float)r/100) * cos(uhel*M_PI/180);
        B = ith->y - ((float)r/100) * sin(uhel*M_PI/180);

        float AA = ceilf(A * 1000) / 1000; //zaokrouhleni
        float BB = ceilf(B * 1000) / 1000; //zaokrouhleni
        int s = 0;
        float z = ith->z;
        stred T={AA,BB,z,s,r};
        stredy.push_back(T);
      }
    }

  //akumulator hledani nejcastejsiho stredu
    acc.push_back(stredy.at(0));
    for(std::vector<stred>::iterator q = stredy.begin(); q!= stredy.end(); ++q)
    {
      int used = 0;
      #pragma omp parallel for
      for(int it = 0; it < acc.size(); it++)
      {
        if ((acc.at(it).a - 0.01) < q->a &&  q->a < (acc.at(it).a+0.01) && (acc.at(it).b - 0.01) < q->b && q->b < (acc.at(it).b+0.01)&& q->i ==0 )
        {
          acc.at(it).i++;
          q->i=1;
          used = 1;
        }
      }// konec acc iteratoru
      if (used == 0)
			{
				acc.push_back(*q);
			}
    }
    std::sort(acc.begin(),acc.end());
    maxima.push_back(acc.back());
  }

  for(int i=0; i< maxima.size();i++)
  {
    maxima.at(i).r;
    maxima.at(i).i;
  }
  std::sort(maxima.begin(),maxima.end());
int r;
  std::ofstream stre ("out_stredy_maxima.txt"); //,ofstream::app
  for(std::vector<stred>::const_iterator e= maxima.begin(); e!= maxima.end(); ++e)
	{
		stre.setf( std::ios::fixed, std:: ios::floatfield );
		stre.precision(3);
		stre << e->a << " " << e->b << " " << e->i <<" " << e->r <<"\n";
    r=e->r;
	}
  stre.close();
  return r;
  //return e.r;
}
void MainWindow::dbh()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
   names <<"all";
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty()&&item!="all")
  {
    int i = names.indexOf(item);
  // set c as Strom
    Tree *c = new Tree(Proj->get_TreeCloud(i));

    //set dbh()
    c->set_dbh();

    //displ dbhCloud
    dispCloud(c->get_dbhCloud(),255, 0, 0);

    //addcylinder
    stred x = c->get_dbh();
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
    name << c->get_name().toUtf8().constData() << "cylinder";
    m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
    pcl::PointXYZ bod;
    bod.x=x.a+(float)x.r/100;
    bod.y=x.b;
    bod.z=x.z+0.15;
    QString h= QString("%1").arg(c->get_dbh().r*2);
    m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0);
  }
  if(ok && item == "all")
  {
    for(int j=0; j < names.size()-1; j++)
    {
      // set as strom
      Tree *c = new Tree(Proj->get_TreeCloud(j));

      c->set_dbh();
      //c->set_position();
    //displ dbhCloud
      dispCloud(c->get_dbhCloud(),255, 0, 0);

    //addcylinder
      stred x = c->get_dbh();
        //Coeff
      pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
      coef->values.push_back((float)x.a);
      coef->values.push_back((float)x.b);
      coef->values.push_back((float)c->get_pose().z+1.25);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0);
      coef->values.push_back((float)0.1);
      coef->values.push_back((float)x.r/100);
      std::stringstream name ;
      name << c->get_name().toUtf8().constData() << "cylinder";
      m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
      pcl::PointXYZ bod;
      bod.x=x.a+(float)x.r/100;
      bod.y=x.b;
      bod.z=x.z+0.1;
      QString h= QString("%1").arg(c->get_dbh().r*2);
      m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0);
    }
  }
}
void MainWindow::dbhLSR()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
   names <<"all";
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty()&&item!="all")
  {
    int i = names.indexOf(item);
  // set c as Strom
    Tree *c = new Tree(Proj->get_TreeCloud(i));
        //set dbh()
    if(c->get_dbh().r < 1)
      c->set_dbhLSR();
    //addcylinder
    stred x = c->get_dbh();
    if(x.r > 150)
    {
      QString a = QString("Warning,\n tree  '%1'  has DBh %2 cm.\n please edit dbh_cloud").arg(c->get_name()).arg(x.r*2);
      QMessageBox::information(this,tr("info"),a );
      return;
    }
    else
    {
      //displ dbhCloud
      dispCloud(c->get_dbhCloud(),255, 0, 0);
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
      name << c->get_name().toUtf8().constData() << "cylinder_LSR";
      m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
      pcl::PointXYZ bod;
      bod.x=x.a;
      bod.y=x.b+(float)x.r/100;
      bod.z=x.z+0.1;
      QString h= QString("%1").arg(c->get_dbh().r*2.0);
      m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0);
    }
  }
  if(ok && item == "all")
  {
    for(int j=0; j < names.size()-1; j++)
    {
      // set as strom
      Tree *c = new Tree(Proj->get_TreeCloud(j));

        //set dbh()
      if(c->get_dbh().r < 1)
        c->set_dbhLSR();

    //addcylinder
      stred x = c->get_dbh();
      if(x.r > 150)
      {
        QString a = QString("Warning,\n tree  '%1'  has DBH: %2 cm.\n please edit dbh_cloud").arg(c->get_name()).arg(x.r*2);
        QMessageBox::information(this,tr("info"),a );
      }
      else
      {
        dispCloud(c->get_dbhCloud(),255, 0, 0);
        //Coeff
        pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
        coef->values.push_back((float)x.a);
        coef->values.push_back((float)x.b);
        coef->values.push_back((float)c->get_pose().z+1.25);
        coef->values.push_back((float)0);
        coef->values.push_back((float)0);
        coef->values.push_back((float)0.1);
        coef->values.push_back((float)x.r/100);
        std::stringstream name ;
        name << c->get_name().toUtf8().constData() << "cylinder_LSR";
        m_vis->addCylinder(*coef,name.str());

    //addtext3D with R
        pcl::PointXYZ bod;
        bod.x=x.a;
        bod.y=x.b+(float)x.r/100;
      bod.z=x.z-0.15;
        QString h= QString("%1").arg(c->get_dbh().r*2.0);
        m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.6,0.5,0);
      }
    }
  }
}
void MainWindow::height()
{
// pro vybrany strom
//set tree cloud names and select one
  QStringList names;

  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
   names <<"all";
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty()&&item!="all")
  {
    int i = names.indexOf(item);
  // set as strom
    Tree *c = new Tree(Proj->get_TreeCloud(i));
    pcl::PointXYZI maxp,minp;
    maxp.z=0;
    minp.z=60000;

    for(int i = 0; i < c->get_Cloud()->points.size();i++)
    {
      pcl::PointXYZI bod = c->get_Cloud()->points.at(i);

      if(bod.z > maxp.z)
        maxp=bod;

      if(bod.z < minp.z)
        minp= bod;
    }
    std::stringstream name ;
    name << c->get_name().toUtf8().constData() << "line";
    m_vis->addLine(c->get_pose(),maxp,name.str());

    QString h= QString("%1").arg(c->get_height());

    m_vis->addText3D(h.toUtf8().constData(),maxp,0.6,0.6,0.5,0);
  }
  if(ok && item == "all")
  {
    for(int j=0; j < names.size()-1; j++)
    {
      // set as strom
    Tree *c = new Tree(Proj->get_TreeCloud(j));
    pcl::PointXYZI maxp,minp;
    maxp.z=0;
    minp.z=60000;

    for(int i = 0; i < c->get_Cloud()->points.size();i++)
    {
      pcl::PointXYZI bod = c->get_Cloud()->points.at(i);

      if(bod.z > maxp.z)
        maxp=bod;

      if(bod.z < minp.z)
        minp= bod;
    }
    std::stringstream name;
    name << c->get_name().toUtf8().constData() << "line";
    m_vis->addLine(c->get_pose(),maxp,name.str());
    QString h= QString("%1").arg(c->get_height());
    m_vis->addText3D(h.toUtf8().constData(),maxp,0.6,0.6,0.5,0);
    }
  }
}
void MainWindow::lenght()
{
  QStringList names;
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
   names <<"all";
  bool ok;

  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty()&&item!="all")
  {
    int i = names.indexOf(item);
  // set c as Strom
    Tree *c = new Tree(Proj->get_TreeCloud(i));
    c->set_lenght();

    std::stringstream name ;
    name << c->get_name().toUtf8().constData() << "lenght";
    m_vis->addLine(c->get_lpoint(true),c->get_lpoint(false),name.str());

    QString h= QString("%1").arg(c->get_lenght());
    m_vis->addText3D(h.toUtf8().constData(),c->get_lpoint(false),0.6,0.6,0.5,0);

  }

  if(ok && item == "all")
  {
    for(int j=0; j < names.size()-1; j++)
    {
      Tree *c = new Tree(Proj->get_TreeCloud(j));
      c->set_lenght();

      std::stringstream name ;
      name << c->get_name().toUtf8().constData() << "lenght";
      m_vis->addLine(c->get_lpoint(true),c->get_lpoint(false),name.str());

      QString h= QString("%1").arg(c->get_lenght());
      m_vis->addText3D(h.toUtf8().constData(),c->get_lpoint(false),0.6,0.6,0.5,0);
    }
  }
}
void MainWindow::position()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  names <<"all";
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty()&&item!="all")
  {
    int i = names.indexOf(item);
    Tree *c = new Tree(Proj->get_TreeCloud(i));
     // sphere coef and visualize
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
    coef->values.push_back((float)c->get_pose().x); //x
    coef->values.push_back((float)c->get_pose().y); //y
    coef->values.push_back((float)c->get_pose().z); //z
    coef->values.push_back((float)0.05);   //r
    std::stringstream name;

    name << c->get_name().toUtf8().constData() << "_sphere";
    m_vis->addSphere(*coef,name.str());
  }
  if(ok && item == "all")
  {
    for(int j=0; j < names.size()-1; j++)
    {
      // set as strom
      Tree *c = new Tree(Proj->get_TreeCloud(j));

      // sphere coef and visualize
      pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
      coef->values.push_back((float)c->get_pose().x); //x
      coef->values.push_back((float)c->get_pose().y); //y
      coef->values.push_back((float)c->get_pose().z); //z
      coef->values.push_back((float)0.05);   //r

      std::stringstream name;
      name << c->get_name().toUtf8().constData() << "_sphere";
      m_vis->addSphere(*coef,name.str());
    }
  }
}
void MainWindow::treeEdit()
{
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty())
  {
    int i = names.indexOf(item);
    //zkopirovat do m_cloud
    Cloud *c = new Cloud(Proj->get_TreeCloud(i).get_Cloud(),Proj->get_TreeCloud(i).get_name());
    m_cloud = c;
    m_cloud->set_Psize(Proj->get_TreeCloud(i).get_Psize());

    //zkopirovat do m_cloud1
    pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    Cloud *cl = new Cloud(h_cloud,Proj->get_TreeCloud(i).get_name());
    m_cloud1 = cl;
    m_cloud1->set_Psize(Proj->get_TreeCloud(i).get_Psize());


    //spustit editacni listu
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction("Stop EDIT");

    connect(stopTE,SIGNAL(triggered()),this,SLOT(treeEditStop()) );
    //connect areapicking event
    //m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
  }
}
void MainWindow::treeEditStop()
{
  // save m_cloud as old tree
  Cloud *c = new Cloud(m_cloud->get_Cloud(),m_cloud->get_name());

  //get the Strom
  QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }

  int j;
  for(int i = 0; i< names.size(); i++)
  {
    if( names.at(i) == m_cloud->get_name())
      j= i;
  }
  //set as a cloud
  Proj->get_TreeCloud(j).set_Cloud(c->get_Cloud());

  //Save into file
  QString pathT = QString("%1\\%2").arg(Proj->get_Path()).arg(m_cloud->get_name());
  pcl::io::savePCDFileBinaryCompressed(pathT.toUtf8().constData(),*m_cloud->get_Cloud());


  //save the rest
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("Save the deleted parts");
	msgBox->setInformativeText("Do you want to save deleted parts as new file?");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

  if(msgBox->exec() == QMessageBox::Yes)
  {

    saveOstCloud(m_cloud1->get_Cloud());
  }
  else
  {
    //save and contencate with existing file
    QStringList names;
      //terrainCloud
      for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
      {
        names << Proj->get_TerrainCloud(i).get_name();
      }
      //vegeCloud
       for(int i = 0; i< Proj->get_sizevegeCV(); i++)
      {
        names << Proj->get_VegeCloud(i).get_name();
      }
      //treeCloud
       for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
      {
        names << Proj->get_TreeCloud(i).get_name();
      }
      //ostCloud
       for(int i = 0; i< Proj->get_sizeostCV(); i++)
      {
        names << Proj->get_ostCloud(i).get_name();
      }

      bool ok;
      QString item = QInputDialog::getItem(this, tr("select cloud for join"), tr("cloud name:"), names, 0, false, &ok);
      if (ok && !item.isEmpty())
      {
        // contencate clouds
        Cloud *c = new Cloud(Proj->get_Cloud(item));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());
        cloud = c->get_Cloud();
        *cloud += *m_cloud1->get_Cloud();
        Proj->get_Cloud(item).set_Cloud(cloud);
      }

  }
  m_cloud->get_Cloud()->points.erase(m_cloud->get_Cloud()->points.begin(),m_cloud->get_Cloud()->points.end());
  m_cloud1->get_Cloud()->points.erase(m_cloud1->get_Cloud()->points.begin(),m_cloud1->get_Cloud()->points.end());
  m_vis->removeAllPointClouds();
  delete editBar;
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
    Cloud *c = new Cloud(tree_cloud,name);
    Proj->set_OstCloud(*c);
    QString pathT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name);
    QFile file(pathT);
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
void MainWindow::saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud, QString name, bool overwrt = false)
{
  Cloud *c = new Cloud(tree_cloud,name);
  Proj->set_OstCloud(*c);
  QString pathT = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name);
  QFile file(pathT);
  if(overwrt== true)
  {
    Proj->save_newCloud("strom",name,tree_cloud);
    openTreeFile(pathT);
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
void MainWindow::saveOstCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr s_cloud)
{
  QString name = QInputDialog::getText(this, tr("Name of new File"),tr("e.g. rest"));
    if(name.isEmpty())
    {
      QMessageBox::warning(this,tr("error"),tr("You forget fill name of new cloud."));
      saveOstCloud(s_cloud);
      return;
    }
    else
    {
      Cloud *cR = new Cloud(s_cloud,name);
      Proj->set_OstCloud(*cR);
      Proj->save_newCloud("ost",name,s_cloud);
      QString path = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(name);
      openOstFile(path);
    }
}

void MainWindow::skeleton()
{
QStringList names;
  // vybrat cloud terenu
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
   names <<"all";
  bool ok;
  QString item = QInputDialog::getItem(this, tr("select tree cloud"), tr("cloud name:"), names, 0, false, &ok);
  if (ok && !item.isEmpty()&&item!="all")
  {
    int i = names.indexOf(item);
  // set c as Strom
    Tree *c = new Tree(Proj->get_TreeCloud(i));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_ = c->skeleton();
    QString name = QString("skelet");
    Cloud *cl = new Cloud(cloud_,name);
    dispCloud(*cl,200,200,50);

//    matrix3x3 m,tmp;
//    for(int j=0;j<cloud_->points.size();j++)
//    {
//      pcl::PointXYZI med = cloud_->points.at(j);
//      m=c->covariance(med,cloud_);
//      for(int i=0;i<10;i++)
//      {
//        tmp = c->jacobi(m);
//        m=tmp;
//      }
//      cloud_->points.at(j).intensity = (m.e)/(m.a+m.e+m.i);
//    }
//
//  QString a = QString("intensity");
//        //displ dbhCloud
//  Cloud *cl = new Cloud(cloud_,a);
//  dispCloud(*cl,a);



  }
  if(ok && item == "all")
  {
    for(int j=0; j < names.size()-1; j++)
    {
      // set c as Strom
      Tree *c = new Tree(Proj->get_TreeCloud(j));

    //displ dbhCloud
     // dispCloud(c->skeleton(),255, 200, 0);


      matrix3x3 m,tmp;
    m.a=1;
    m.b=0;
    m.c=2;
    m.d=0;
    m.e=2;
    m.f=1;
    m.g=2;
    m.h=1;
    m.i=1;
    for(int i=0;i<10;i++)
    {
      QString a =QString("matice:\n %1\t%2\t%3\n%4\t%5\t%6\n%7\t%8\t%9").arg(m.a).arg(m.b).arg(m.c).arg(m.d).arg(m.e).arg(m.f).arg(m.g).arg(m.h).arg(m.i);
      QMessageBox::information(this,("matice"),a);
      tmp = c->jacobi(m);
      m=tmp;
    }
    }
  }
}

//MISCELLANEOUS
void MainWindow::plusCloud()
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
  //ost
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
  //tree
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }

  PlusDialog *plusD = new PlusDialog( names,this);
  plusD->exec();
  if(plusD->get_names().isEmpty())
    return;
  plusCloud(plusD->get_names(), "strom");
  //pridat do stromu
  QString filen= QString ("%1\\%2.pcd").arg(Proj->get_Path()).arg(plusD->get_names().at(2));
  openOstFile(filen);
}
void MainWindow::plusCloud(QStringList names, QString typ = "ost")
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cl (new pcl::PointCloud<pcl::PointXYZI>);
  *cl = *Proj->get_Cloud(names.at(0)).get_Cloud() + *Proj->get_Cloud(names.at(1)).get_Cloud();
//  m_cloud->set_Cloud(cl);
  Proj->save_newCloud(typ,names.at(2),cl);
}
void MainWindow::voxelize()
{
  QStringList names;
  //basecloud
  //names<< Proj->get_baseCloud().get_name();
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
  //ost
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
  //tree
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  bool ok;
  QString cl = QInputDialog::getItem(this,("Select cloud for voxelization"),("name of cloud:"),names,0, false, &ok);
  if(ok== true)
  {
    float res =QInputDialog::getDouble(this,("insert resolution"),("resolution in m:"),0.1,0.01,100000,3);

    pcl::VoxelGrid<pcl::PointXYZI> vox;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    vox.setInputCloud (Proj->get_Cloud(cl).get_Cloud());
    vox.setLeafSize (res, res, res);
    vox.filter (*cloud_filtered);

    QStringList clo = cl.split(".");
    int res_cm = std::ceil(res*100);

    QString it = QString("voxel_%1cm_%2").arg(res_cm).arg(clo.at(0));
    Proj->save_newCloud("ost",it,cloud_filtered);
    QString fullnameV = QString("%1\\%2.pcd").arg(Proj->get_Path()).arg(it);
    openOstFile(fullnameV);
  }
  else
  {
    return;
  }
}
void MainWindow::backgroundColor()
{
  QColor color = QColorDialog::getColor(Qt::white,this);
  if(color.isValid())
  {
    qreal *r,*g,*b;
    color.getRgbF (  r, g,  b);
    double rr,gg,bb;
    rr = *r;
    gg = *g;
    bb = *b;
    m_vis->setBackgroundColor(rr,gg,bb);

    qvtkwidget->show();
  }
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

  importTXTAct = new QAction( tr("&Import TXT, XYZ"), this);
  importTXTAct->setStatusTip(tr("Import of txt file"));
  connect(importTXTAct, SIGNAL(triggered()), this, SLOT(importtxt2()));

  importLASAct = new QAction( tr("&Import LAS"), this);
  importTXTAct->setStatusTip(tr("Import of las file"));
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

  exitAct = new QAction(tr("E&xit"), this);
  exitAct->setShortcuts(QKeySequence::Quit);
  exitAct->setStatusTip(tr("Exit the application"));
  connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

//TERRAIN
  voxelAct = new QAction(tr("Terrain voxel"), this);
  voxelAct->setStatusTip(tr("voxel method with variable size of voxel"));
 // voxelAct->setEnabled(false);
  connect(voxelAct, SIGNAL(triggered()), this, SLOT(voxelgrid()));

  voxelStatAct = new QAction(tr("Voxel statistic "), this);
  voxelStatAct->setStatusTip(tr("voxel method with statistically computed terrain"));
 // voxelStatAct->setEnabled(false);
  connect(voxelStatAct, SIGNAL(triggered()), this, SLOT(voxelstat()));

  octreeAct = new QAction(tr("Terrain octree"), this);
  octreeAct->setStatusTip(tr("compute surface point based on octree structure"));
  //octreeAct->setEnabled(false);
  connect(octreeAct, SIGNAL(triggered()), this, SLOT(octreeSlot()));

  manualADAct = new QAction(tr("Manual adjustment"), this);
  manualADAct->setStatusTip(tr("manual adjustment of terrain cloud"));
 // manualADAct->setEnabled(false);
  connect(manualADAct, SIGNAL(triggered()), this, SLOT(manualAdjust()));

//TREE ATRIBUTES
  segDistAct = new QAction(tr("Automatic segmentation"), this);
  segDistAct->setStatusTip(tr("compute atributes for all trees"));
  connect(segDistAct, SIGNAL(triggered()), this, SLOT(seg_dist()));

  tAAct = new QAction(tr("Tree atributes into file"), this);
  tAAct->setStatusTip(tr("compute atributes for all trees"));
  //tAAct->setEnabled(false);
  connect(tAAct, SIGNAL(triggered()), this, SLOT(treeAtributes()));

  tAReadAct= new QAction(tr("Load tree atributes"), this);
  tAReadAct->setStatusTip(tr("compute atributes for all trees"));
  //tAReadAct->setEnabled(false);
  connect(tAReadAct, SIGNAL(triggered()), this, SLOT(treeAtributesRead()));


  dbhAct = new QAction(tr("DBH"), this);
  dbhAct->setStatusTip(tr("display best fitted cylinder"));
  //dbhAct->setEnabled(false);
  connect(dbhAct, SIGNAL(triggered()), this, SLOT(dbh()));

  dbhLSRAct = new QAction(tr("DBH LSR"), this);
  dbhLSRAct->setStatusTip(tr("display best fitted cylinder"));
  //dbhAct->setEnabled(false);
  connect(dbhLSRAct, SIGNAL(triggered()), this, SLOT(dbhLSR()));

  heightAct = new QAction(tr("Height"), this);
  heightAct->setStatusTip(tr("display best fitted cylinder"));
 // heightAct->setEnabled(false);
  connect(heightAct, SIGNAL(triggered()), this, SLOT(height()));

  posAct = new QAction(tr("Tree position"), this);
  posAct->setStatusTip(tr("displaysphere in tree position"));
 // posAct->setEnabled(false);
  connect(posAct, SIGNAL(triggered()), this, SLOT(position()));

  manualSelAct = new QAction(tr("Manual selection"), this);
  manualSelAct->setStatusTip(tr("display best fitted cylinder"));
 // manualSelAct->setEnabled(false);
  connect(manualSelAct, SIGNAL(triggered()), this, SLOT(manualSelect()));

  treeEditAct = new QAction(tr("Tree cloud edit"), this);
  treeEditAct->setStatusTip(tr("edit selected tree cloud"));
  //treeEditAct->setEnabled(false);
  connect(treeEditAct, SIGNAL(triggered()), this, SLOT(treeEdit()));

  dbhEditAct = new QAction(tr("Tree DBHcloud edit"), this);
  dbhEditAct->setStatusTip(tr("edit selected tree cloud"));
  //treeEditAct->setEnabled(false);
  connect(dbhEditAct, SIGNAL(triggered()), this, SLOT(dbhCloudEdit()));

  lengAct = new QAction(tr("Cloud lenght"), this);
  lengAct->setStatusTip(tr("display cloud lenght"));
  connect(lengAct, SIGNAL(triggered()), this, SLOT(lenght()));


  skeletonAct = new QAction(tr("Skeleton"), this);
  skeletonAct->setStatusTip(tr("create skeleton cloud"));
  connect(skeletonAct, SIGNAL(triggered()), this, SLOT(skeleton()));
//MISC
  plusAct = new QAction(tr("Cloud contencate"), this);
  plusAct->setStatusTip(tr("cloud + cloud"));
  connect(plusAct, SIGNAL(triggered()), this, SLOT(plusCloud()));

  voxAct = new QAction(tr("Voxelize cloud"), this);
  voxAct->setStatusTip(tr("voxels from cloud"));
  connect(voxAct, SIGNAL(triggered()), this, SLOT(voxelize()));


  backgrdColAct = new QAction(tr("Color backgroung"), this);
  backgrdColAct->setStatusTip(tr("background color"));
  connect(backgrdColAct, SIGNAL(triggered()), this, SLOT(backgroundColor()));

//ABOUT
  aboutAct = new QAction(tr("&About"), this);
  aboutAct->setStatusTip(tr("Show the application's About box"));
  connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

  aboutQtAct = new QAction(tr("About &Qt"), this);
  aboutQtAct->setStatusTip(tr("Show the Qt library's About box"));
  connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));

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
  fileMenu->addSeparator();
  importMenu =  fileMenu->addMenu(tr("Import"));
  importMenu->addAction(importTXTAct);
  importMenu->addAction(importLASAct);
  importMenu->addAction(importPCDAct);
  importMenu->addAction(importTerenAct);
  importMenu->addAction(importVegeAct);
  importMenu->addAction(importTreeAct);
  fileMenu->addSeparator();
  fileMenu->addAction(exportTXTAct);
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
  vegeMenu->addAction(segDistAct);

//TREE ATRIBUTES
  treeMenu = menuBar()->addMenu(tr("&Trees"));
  treeMenu->addAction(treeEditAct);
  treeMenu->addSeparator();
  treeMenu->addAction(tAAct);
  treeMenu->addAction(tAReadAct);
  treeMenu->addSeparator();
  //treeMenu->addAction(cylinderAct);
 // treeMenu->addAction(dbhAct);
  treeMenu->addAction(posAct);
  treeMenu->addAction(heightAct);
  treeMenu->addAction(dbhLSRAct);
  treeMenu->addAction(lengAct);
  treeMenu->addAction(dbhEditAct);
  //treeMenu->addAction(skeletonAct);
//MISC
  miscMenu = menuBar()->addMenu(tr("Other features"));
  miscMenu->addAction(plusAct);
  miscMenu->addAction(voxAct);backgrdColAct;
 // miscMenu->addAction(backgrdColAct);
//ABOUT
  helpMenu = menuBar()->addMenu(tr("&About"));
  helpMenu->addAction(aboutAct);
  helpMenu->addAction(aboutQtAct);
 }
void MainWindow::createToolBars()
{

}
void MainWindow::createStatusBar()
{
     statusBar()->showMessage(tr("3D FOREST Ready"));
 }
void MainWindow::createTreeView()
{
  treeWidget->resizeColumnToContents(1);
}

//QVTKWIDGET
void MainWindow::ShowContextMenu(const QPoint& pos)
{

}
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

void MainWindow::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* )
{
    if (event.keyDown () && event.getKeyCode() == '+')
  {
	// for all cloud displayed chanze Psize+1
	QMessageBox::information(0,("tt"),("dd"));
  }

  if (event.keyDown () && event.getKeyCode() == '-')
  {
	// for all cloud displayed chanze Psize-1

  }

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
//  std::stringstream namep,name_dbh,name_h,name_dbhcloud;
//  name_dbh <<name.toUtf8().constData() << "cylinder";
//
//  m_vis->removeShape(name_dbh.str());
//  name_h << name.toUtf8().constData() << "line";
//  m_vis->removeShape(name_h.str());
//
//  namep << name.toUtf8().constData() << "_sphere";
//  m_vis->removeShape(namep.str());
//  name_dbhcloud << name.toUtf8().constData() << "_dbh";
//  m_vis->removePointCloud(name_dbhcloud.str());

  qvtkwidget->update();
}
void MainWindow::removeCloud()
{
  m_vis->removePointCloud(treeWidget->name.toUtf8().constData());
}

//TREEWIDGET
void MainWindow::treeWid()
{

  //signals
  //connect(treeWidget, SIGNAL(itemClicked(QTreeWidgetItem*,int)), this, SLOT(onItemChange(QTreeWidgetItem*,int)));
  //connect(treeWidget, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(ShowTreeContextMenu(const QPoint&)));
}
void MainWindow::ShowTreeContextMenu(const QPoint& pos)
{
  //QMessageBox::information(this,tr("nn"),tr("context menu"));



  QPoint globalPos = treeWidget->mapToGlobal(pos);

  QMenu *myMenu = new QMenu;
  QAction *actCustom = new QAction("about", myMenu);
  myMenu->exec(globalPos);


//  QModelIndex index = treeWidget->currentIndex();
//
//
//
//if (!index.isValid())
//  return;
//
//  QString fileName = treeWidget->model()->data(treeWidget->model()->index(index.row(), 0),0).toString();
//
//
//
//// create the context menu
//     // QSignalMapper *signalMapper = new QSignalMapper(this);
//
//
//         QAction *actCustom = new QAction("about", myMenu);
//         // myMenu->addAction(actCustom);
//         // connect(actCustom, SIGNAL(triggered(void)), this, SLOT(about()));
//
//          QAction *selectColorAct = new QAction("color", myMenu);
//          myMenu->addAction(selectColorAct);

//          QSignalMapper *signalMapper = new QSignalMapper(this);
//          connect(selectColorAct, SIGNAL(triggered(void)), signalMapper, SLOT(map()));
//          signalMapper->setMapping(selectColorAct, fileName);
//          connect(signalMapper, SIGNAL(mapped(QString)),this, SLOT(colorCloud(QString)));
//
//          QAction *hideCloudAct = new QAction("hide", myMenu);
//          myMenu->addAction(hideCloudAct);
//          QSignalMapper *signalMapper1 = new QSignalMapper(this);
//          connect(hideCloudAct, SIGNAL(triggered(void)), signalMapper1, SLOT(map()));
//          signalMapper1->setMapping(hideCloudAct, fileName);
//          connect(signalMapper1, SIGNAL(mapped(QString)),this, SLOT(hideCloud(QString)));
//
//          QAction *showCloudAct = new QAction("show", myMenu);
//          myMenu->addAction(showCloudAct);
//          QSignalMapper *signalMapper2 = new QSignalMapper(this);
//          connect(showCloudAct, SIGNAL(triggered(void)), signalMapper2, SLOT(map()));
//          signalMapper2->setMapping(showCloudAct, fileName);
//          connect(signalMapper2, SIGNAL(mapped(QString)),this, SLOT(showCLoud(QString)));
//
//          QAction *startEditAct = new QAction("Start Edit", myMenu);
//          myMenu->addAction(startEditAct);
//          QSignalMapper *signalMapper3 = new QSignalMapper(this);
//          connect(startEditAct, SIGNAL(triggered(void)), signalMapper3, SLOT(map()));
//          signalMapper3->setMapping(startEditAct, fileName);
//          connect(signalMapper3, SIGNAL(mapped(QString)),this, SLOT(startEdit(QString)));
//
//
//
}
void MainWindow::addTreeItem(QString i)
{
  QTreeWidgetItem * item = new QTreeWidgetItem();
  item->setText(1,i);
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable|Qt::ItemIsSelectable);
  item->setCheckState(0,Qt::Checked);
  treeWidget->addTopLevelItem(item);
  treeWidget->resizeColumnToContents(1);
}
void MainWindow::onItemChange(QString name, bool st)
{
  if(st==false)
  {
    //removeCloud(*name);
  }
  else
    {
     //dispCloud()
    }

  //QString a = QString ("%1").arg(item->text(1));
  //QMessageBox::information(this,tr("nn"),a);
  //QModelIndex index = treeWidget->currentIndex();
  //QString fileName = treeWidget->model()->data(treeWidget->model()->index(index.row(), 0),0).toString();
  //removeCloud(fileName);
}
void MainWindow::startEdit(QString name)
{
//COPY cloud into m_cloud
  QStringList index = name.split(" ");
  int cloud_ID = index.at(0).toInt();
  QString cloud_name = index.at(1);

//  m_cloud = Proj->cloudy.at(cloud_ID);


//removeall clouds and display only m_cloud
  m_vis->removeAllPointClouds();

  dispCloud(*m_cloud,220,220,0);
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_edit(m_cloud,220,220,0);
//  if(!m_vis->updatePointCloud<pcl::PointXYZI>(m_cloud, color_edit, "m_cloud"))
//    m_vis->addPointCloud<pcl::PointXYZI>(m_cloud, color_edit, "m_cloud");

//connect areapicking event
  m_vis->registerAreaPickingCallback  (&MainWindow::AreaEvent, *this );
// create klicking button stop edit
     fileToolBar = addToolBar(tr("EDIT"));
    QAction *stopEditAct = new QAction( tr("&STOP EDIT"), this);
     fileToolBar->addAction(stopEditAct);

     connect(stopEditAct, SIGNAL(triggered()),this, SLOT(stopEdit()));
}
void MainWindow::stopEdit()
{
// ULOZIT M_CLOUD JAKO
  QString fileName = QFileDialog::getSaveFileName(this,tr("SAVE SELECTED POINTS AS..."),"",tr("files (*.pcd)"));
  if (fileName.isEmpty())
  {   QMessageBox::warning(this,"ERROR","no name filled");  }
  else
  {
   //saveCloud(m_cloud,fileName);
   //open(fileName);
  }

  QString backup = QString ("%1/cloud_tmp.pcd").arg(Proj->get_Path());
  pcl::io::savePCDFileBinaryCompressed(backup.toUtf8().constData(), *m_cloud1->get_Cloud());

// BUDU POKRACOVAT V EDITACI?
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("EDIT..");
	msgBox->setInformativeText("DO YOU WANT TO CONTINUE EDITING THE CLOUD?");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

	if(msgBox->exec() == QMessageBox::Yes)
  {
    m_cloud = m_cloud1;
    m_cloud1->get_Cloud()->clear();
  }
//ULOZIT M_CLOUD2 JAKO
  else
  {
    QString a = QString ("%1/cloud_rest.pcd").arg(Proj->get_Path());
    //saveCloud(m_cloud2,a.toUtf8().constData());
    //open(a);
  }
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
void MainWindow::showCLoud(QString name)
{
  QMessageBox::information(this,tr(""),name);

  Cloud *c = new Cloud(Proj->get_Cloud(name));

  dispCloud(*c);

}
void MainWindow::PointSize(QString name)
{
  int p = QInputDialog::getInt(this,("point size"),("please enter value in range 1 - 64 defining size of point"),1,1,64);

  Proj->set_PointSize(name, p);
  dispCloud(Proj->get_Cloud(name));
}

void MainWindow::hideCloud(QString name)
{
  QStringList index = name.split(" ");

  int cloud_ID = index.at(0).toInt();
  QString cloud_name = index.at(1);

  m_vis->removePointCloud( cloud_name.toUtf8().constData());
}
void MainWindow::deleteCloud(QString name)
{
  Proj->delete_Cloud(name);
  m_vis->removePointCloud(name.toUtf8().constData());
}
void MainWindow::about()
{
QMessageBox::about(this,tr("about 3D Forest application"),tr(" 3D forest application was developed as a part of my Ph.D. thesis.\n But continue as a free platform for TLS data processing. \n  AUTHOR: Jan Trochta j.trochta@gmail.com "));
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

////PLUSDIALOG

PlusDialog::PlusDialog(QStringList items, QWidget *parent)
 : QDialog(parent)
{
  combo1 = new QComboBox(this);
  combo2 = new QComboBox(this);
  combo1->addItems(items);
  combo2->addItems(items);

  name = new QLineEdit(this);
  name->setMaxLength(15);

  label1 = new QLabel(this);
  label1->setText("Cloud 1  \t+");

  label2 = new QLabel(this);
  label2->setText("Cloud 2  \t=");

  label3 = new QLabel(this);
  label3->setText("new Cloud");

  QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  QGridLayout *layout = new QGridLayout;
  layout->setColumnStretch(1, 1);
  layout->setColumnMinimumWidth(0, 20);
  layout->setColumnMinimumWidth(1, 20);
  layout->setColumnMinimumWidth(2, 20);

     //header
  layout->addWidget(label1, 0, 0);
  layout->addWidget(label2, 0, 1);
  layout->addWidget(label3, 0, 2);
  layout->addWidget(combo1, 1, 0);
  layout->addWidget(combo2, 1, 1);
  layout->addWidget(name, 1, 2);
  layout->addWidget(buttonBox, 2, 1);

  setLayout(layout);

  connect(combo1, SIGNAL(currentIndexChanged(QString)), this, SLOT(setCloud1(QString)));
  connect(combo2, SIGNAL(currentIndexChanged(QString)), this, SLOT(setCloud2(QString)));
  connect(name, SIGNAL(textChanged(QString)), this, SLOT(setCloud3(QString)));
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}
void PlusDialog::reject()
{
  emit rejecte();
  this->hide();
}
void PlusDialog::accept()
{
  result << cloud1 << cloud2 << cloud3;

  emit pluscloud(result);
  this->close();
}
void PlusDialog::setCloud1(QString name)
{
cloud1 = name;
}
void PlusDialog::setCloud2(QString name)
{
cloud2 = name;
}
void PlusDialog::setCloud3(QString name)
{
cloud3 = name;
}
QStringList PlusDialog::get_names()
{
return result;
}
  ////MYTREE
MyTree::MyTree(QWidget *parent)
  : QTreeWidget(parent)
{
    setContextMenuPolicy(Qt::CustomContextMenu);
    setColumnCount(2);
    //resizeColumnToContents(1);
    header()->resizeSection(0, 50);
    QStringList q;
    q << QString (" visible") ;
    q << QString ("     name");
    setHeaderLabels(q);
    header()->resizeSection(1, 120);

    //SLOTS
    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(showContextMenu(const QPoint&)));
    connect(this, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(onItemChange(QTreeWidgetItem*,int)));

  // set as strom
    //Tree *c = new Tree(Proj->get_TreeCloud(i));
    //signals
}
void MyTree::itemdelete(QString name)
{
  QTreeWidgetItemIterator it(this);
  while (*it)
  {
    if( (*it)->text(1) == name)
    {
      QTreeWidgetItem *currentItem = (*it);
      delete currentItem;
    }
    ++it;
  }
}
void MyTree::showContextMenu(const QPoint &pos)
{
  QMenu *menu = new QMenu;

  QModelIndex index = this->currentIndex();
  if(!index.isValid())
    return;

  QString it = this->model()->data(this->model()->index(index.row(), 1),0).toString();

  QAction *deleteACT = new QAction("delete",menu);
  menu->addAction(deleteACT);

  QAction *colorACT = new QAction("color",menu);
  menu->addAction(colorACT);

  QAction *colorFieldACT = new QAction("Color by field",menu);
  menu->addAction(colorFieldACT);

  QAction *PsizeACT = new QAction("Point size",menu);
  menu->addAction(PsizeACT);

  QAction *allONACT = new QAction("all Clouds ON",menu);
  menu->addAction(allONACT);
  connect(allONACT, SIGNAL(triggered()), this, SLOT(allON()));

  QAction *allOFFACT = new QAction("all Clouds OFF",menu);
  menu->addAction(allOFFACT);
  connect(allOFFACT, SIGNAL(triggered()), this, SLOT(allOFF()));

  QSignalMapper *signalMapper = new QSignalMapper(this);
  connect(deleteACT, SIGNAL(triggered()), signalMapper, SLOT(map()));
  signalMapper->setMapping(deleteACT, it);
  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(onDeleteItem(QString)));

  QSignalMapper *signalMapperC = new QSignalMapper(this);
  connect(colorACT, SIGNAL(triggered()), signalMapperC, SLOT(map()));
  signalMapperC->setMapping(colorACT, it);
  connect(signalMapperC, SIGNAL(mapped(QString)), this, SLOT(onColor(QString)));

  QSignalMapper *signalMapperCF = new QSignalMapper(this);
  connect(colorFieldACT, SIGNAL(triggered()), signalMapperCF, SLOT(map()));
  signalMapperCF->setMapping(colorFieldACT, it);
  connect(signalMapperCF, SIGNAL(mapped(QString)), this, SLOT(onColorField(QString)));

  QSignalMapper *signalMapperPsize = new QSignalMapper(this);
  connect(PsizeACT, SIGNAL(triggered()), signalMapperPsize, SLOT(map()));
  signalMapperPsize->setMapping(PsizeACT, it);
  connect(signalMapperPsize, SIGNAL(mapped(QString)), this, SLOT(onPsize(QString)));

  menu->exec(QCursor::pos());
}
void MyTree::onItemChange(QTreeWidgetItem *item,int i)
{
  name = item->text(1);

  if(!item->checkState(0))
  {
    emit checkedON(name);
  }
 if(item->checkState(0))
  {
    emit checkedOFF(name);
  }
}
void MyTree::onDeleteItem(QString name)
{
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("DELETE");
	QString a = QString("DO YOU WANT TO DELETE CLOUD -- %1 -- FROM PROJECT?").arg(name);
	msgBox->setInformativeText(a);
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

	if(msgBox->exec() == QMessageBox::Yes)
  {
    itemdelete(name);
    emit deleteItem(name);
  }
}
void MyTree::onColor(QString name)
{
  emit colorItem(name);
}
void MyTree::onColorField(QString name)
{
  emit colorItemField(name);
}
void MyTree::onPsize(QString name)
{
  emit psize(name);
}
void MyTree::cleanAll()
{
  QTreeWidgetItemIterator it(this);
  int index = 0;
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
     if(!item)return;
     int x = this->indexOfTopLevelItem(item);
     if(x >= 0 && x < this->topLevelItemCount())
     {
       item = this->takeTopLevelItem(x);
       if(item)
         delete item;
     }
  }
}
void MyTree::allON()
{
  QTreeWidgetItemIterator it(this);
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    name = item->text(1);
    item->setCheckState(0,Qt::Checked);
    emit checkedOFF(name);
    it++;
  }
}
void MyTree::allOFF()
{
  allItemOFF();
}
void MyTree::allItemOFF()
{
  QTreeWidgetItemIterator it(this);
  //int index = 0;
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    name = item->text(1);
    item->setCheckState(0,Qt::Unchecked);
    emit checkedON(name);
    it++;
  }

}
