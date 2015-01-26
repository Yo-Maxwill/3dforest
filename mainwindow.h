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
#ifndef MAINWINDOW_H_INCLUDED
#define MAINWINDOW_H_INCLUDED

#include <string>
 #include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <pcl/octree/octree_pointcloud.h>
#include <QVTKWidget.h>
#include <QtGui/QMainWindow>
#include <QtGui/QTreeWidget>
#include <QtGui/QProgressBar>
#include <QtGui/QDialog>
#include <omp.h>
#include <project.h>

class QAction;
class QMenu;
class QCheckBox;
class QInputDialog;
class QLabel;
class QComboBox;

class Visualizer : public pcl::visualization::PCLVisualizer
{
  public:
    Visualizer(QString name);
};

class InputDialog :public QDialog
{
  Q_OBJECT
  public:
    InputDialog(QWidget *parent = 0 );
    void DialogSize();
    void DialogLayout();

    void set_inputCloud1(QString, QStringList);
    void set_inputCloud2(QString, QStringList);
    void set_inputCloud3(QString, QStringList);
    void set_outputCloud1(QString,QString);
    void set_outputCloud2(QString,QString);
    void set_inputInt(QString, QString);
    void set_description(QString);
    void set_title(QString);
    void set_stretch();
    void set_path(QString);

    QString get_inputCloud1();
    QString get_inputCloud2();
    QString get_inputCloud3();
    QString get_outputCloud1();
    QString get_outputCloud2();

    int get_intValue();
    bool accet();
  private slots:
    void ok();
    void validate(QString);
    void validateInt(QString);
    void validateOutput1(QString);
    void validateOutput2(QString);

  private:
    QString input_cloud1;
    QString input_cloud2;
    QString input_cloud3;
    QString output_cloud1;
    QString output_cloud2;

    float float_value1;
    float float_value2;
    int int_value1;
    int int_value2;
    QString path;
    QComboBox * inputCloud1;
    bool isIC1;
    QComboBox * inputCloud2;
    bool isIC2;
    QComboBox * inputCloud3;
    bool isIC3;
    QLineEdit *outputCloud1;
    bool output1Bool;
    bool isOC1;
    QLineEdit *outputCloud2;
    bool isOC2;
    bool output2Bool;
    QLineEdit *intInput;
    bool intInputBool;
    bool isII1;

    QDialogButtonBox *buttonBox;

    QHBoxLayout *buttontLayout;
    QVBoxLayout *InputLayout;
    QHBoxLayout *inputareaLayout;
    QVBoxLayout *mainLayout;
};


class MyTree :public QTreeWidget
{
  Q_OBJECT
public:
    MyTree(QWidget *parent = 0);
    QString name;
    void cleanAll();
    void itemdelete(QString name);
    void allItemOFF();
    void itemON(QString);

private slots:
    void showContextMenu(const QPoint &pos);
    void onItemChange(QTreeWidgetItem *item,int i);
    void onDeleteItem(QString name);
    void onColor(QString name);
    void onColorField(QString name);
    void onPsize(QString name);
    void allON();
    void allOFF();

 signals:
    void checkedON(QString);
    void checkedOFF(QString);
    void deleteItem(QString);
    void colorItem(QString);
    void colorItemField(QString);
    void psize(QString);
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

  public:
    MainWindow();
    QString get_path();

 private slots:

////PROJECT
  //PROJECT
  void newProject();
  void openProject();
  void closeProject();
  //OPEN
  void openTerrainFile(QString file);
  void openVegeFile(QString file);
  void openOstFile(QString file);
  void openTreeFile(QString file);
  void openCloudFile(QString file);

  void openTerrainFile(QString file, QColor col);
  void openVegeFile(QString file, QColor col);
  void openOstFile(QString file, QColor col);
  void openTreeFile(QString file, QColor col);
  void openCloudFile(QString file, QColor col);
  //IMPORT
  void importtxt();
  void importtxt2();
  void importlas();
  void importTerrainFile();
  void importCloud();
  void importVegeCloud();
  void importTreeCloud();
  //EXPORT
  void exportCloud();
  //EXIT
  void closeEvent(QCloseEvent *event);

////TEREN
  void voxelgrid();     // regular voxel of cloud and selecting of lowest points
  void voxelstat();     //voxels of cloud and computed terrain accorning accumulated sum of points
  void octree(float res, pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output_vege, pcl::PointCloud<pcl::PointXYZI>::Ptr output_ground);// toto asi nebude potreba
  void octreeSlot();    //octree segmentation of cloud and selecting of lowest nodes and it points
  void manualAdjust();  // manual adjustment of terrain file
  void manualAdjustStop(); //stop manual adjustment and save result file
  void undo();

//// TREE ATRIBUTES
  void treeAtributes(); //save tree atributes into file
  void treeAtributesRead(); //read tree atributes from file
  void cylinderSeg();   //show best selected cylinders on each tree
  int dbh (pcl::PointCloud<pcl::PointXYZI>::Ptr input);
  void dbh();           //show cylinder computed by Hough Transform with graph of accumulated sum
  void dbhHT();
  void dbhLSR();        //least sqweare regression for circle
  void height();        //show line connecting lowest and highest point in tree cloud
  void position();      //show sphere at tree position
  void manualSelect();  //manually secting of trees
  void manualSelectStop();//stop manual tree selection
  void treeEdit();      //manual editing of tree cloud
  void treeEditStop();  //stop of manual editing of tree cloud
  void lenght();        //lenght of cloud
  void seg_dist();      //euclidian segmentation
  void skeleton();
  void dbhCloudEdit();
  void dbhCloudStopEdit();
  void plysave();

////MISC
  void plusCloud();     //contencate cloud
  void plusCloud(QString, QString, QString, QString );
  void voxelize();  //voxels of various size on selected cloud
  void backgroundColor();
  void IDW();
  void clip();
  void clip(Cloud cl, Cloud cl2, int res);
  void clipped();
  void clipStop();
//// ABOUT
  void about();         //about 3D FOREST application
////TREE SLOTS
  void removeCloud();
  void dispCloud(QString name);
  void showCLoud(QString name);
  void removeCloud(QString name);
  void deleteCloud(QString name);
  void colorCloud(QString name);
  void colorCloudField(QString name);
  void PointSize(QString name);

//// ACTION and MENUS
 private:
  void createActions();
  void createMenus();
  void createToolBars();
  void createStatusBar();
  void createTreeView();
  void saveOstCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr s_cloud);
  void saveOstCloud(Cloud *s_cloud);
  void saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud);
  void saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud, QString filename,bool overwrt);
  void saveTreeCloud(Cloud *s_cloud);
  void saveVegeCloud(Cloud *s_cloud);
  void saveTerrainCloud(Cloud *s_cloud);

  QStringList get_allNames();
  QStringList get_treeNames();
  QStringList get_terrainNames();
  QStringList get_vegetationNames();
  QStringList get_ostNames();
  QStringList get_baseNames();
  //MENUS
    QMenu *fileMenu;
    QMenu *importMenu;
    QMenu *terenMenu;
    QMenu *vegeMenu;
    QMenu *treeMenu;
    QMenu *helpMenu;
    QMenu *miscMenu;
  //TOOLBARS
    QToolBar *editBar;
  //PROJECT ACTIONS
    QAction *new_projectAct;
    QAction *open_projectAct;
    QAction *close_projectAct;
    QAction *importTXTAct;
    QAction *importLASAct;
    QAction *importPCDAct;
    QAction *importTerenAct;
    QAction *importVegeAct;
    QAction *importTreeAct;
    QAction *exportTXTAct;
    QAction *exportPLYAct;
    QAction *exitAct;
  //TEREN ACTIONS
    QAction *voxelAct;
    QAction *voxelStatAct;
    QAction *octreeAct;
    QAction *manualADAct;
  //TREE ATRIBUTES ACTIONS
    QAction *tAAct;
    QAction *tAReadAct;
    QAction *cylinderAct;
    QAction *dbhAct;
    QAction *dbhHTAct;
    QAction *dbhLSRAct;
    QAction *heightAct;
    QAction *posAct;
    QAction *manualSelAct;
    QAction *treeEditAct;
    QAction *dbhEditAct;
    QAction *lengAct;
    QAction *segDistAct;
    QAction *skeletonAct;
  //MISC ACTIONS
    QAction *plusAct;
    QAction *voxAct;
    QAction *backgrdColAct;
    QAction *clipAct;
    QAction *clipedAct;
  //ABOUT ACTIONS
    QAction *aboutAct;
    QAction *aboutQtAct;
    QAction *IDWAct;

////TREEWIDGET
  MyTree *treeWidget;
  QList<QTreeWidgetItem *> tree_items;
  void addTreeItem(QString i);
  void onItemChange(QString name, bool st);
  void treeWid();
  void delete_cloud(QString name);

  //CONTEXT MENU

  void ShowTreeContextMenu(const QPoint& pos);

  void hideCloud(QString name);
  void startEdit(QString name);
  void stopEdit();

//// QVTKWIDGET
  QVTKWidget *qvtkwidget;
  boost::shared_ptr<Visualizer> m_vis;
  boost::signals2::connection area;

  void AreaEvent(const pcl::visualization::AreaPickingEvent& event, void* );
  void AreaEvent2(const pcl::visualization::AreaPickingEvent& event, void* );

  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* );
  void ShowContextMenu(const QPoint& pos);

  pcl::PointCloud<pcl::PointXYZI>::Ptr lowPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr c,float res, bool v);


  Project *Proj;
  Cloud *m_cloud ;
  Cloud *m_cloud1;
  Cloud *m_cloud2;

  void dispCloud(Cloud cloud);
  void dispCloud(Cloud cloud, QString i);
  void dispCloud(Cloud cloud,int red, int green, int blue);

  QToolBar *fileToolBar;
  QToolBar *editToolBar;
  QProgressBar *pBar;
  std::vector<int> undopoint;

  int m_width;
  int m_res;
 };

#endif // MAINWINDOW_H_INCLUDED
