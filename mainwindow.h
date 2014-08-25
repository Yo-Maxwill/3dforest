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

class PlusDialog :public QDialog
{
  Q_OBJECT
public:

    PlusDialog(QStringList items, QWidget *parent = 0);
    QStringList get_names();
signals:
    void pluscloud(QStringList);
    void rejecte();

private slots:
   void accept();
   void reject();
   void setCloud1(QString);
   void setCloud2(QString);
   void setCloud3(QString);

private:
    QLabel *label1;
    QLabel *label2;
    QLabel *label3;
    QComboBox *combo1;
    QComboBox *combo2;
    QLineEdit *name;
    QStringList result;
    QString cloud1;
    QString cloud2;
    QString cloud3;
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

private slots:
    void showContextMenu(const QPoint &pos);
    void onItemChange(QTreeWidgetItem *item,int i);
    void onDeleteItem(QString name);
    void onColor(QString name);
    void allON();
    void allOFF();

 signals:
    void checkedON(QString);
    void checkedOFF(QString);
    void deleteItem(QString);
    void colorItem(QString);
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

  public:
    MainWindow();

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

//// TREE ATRIBUTES
  void treeAtributes(); //save tree atributes into file
  void treeAtributesRead(); //read tree atributes from file
  void cylinderSeg();   //show best selected cylinders on each tree
  int dbh (pcl::PointCloud<pcl::PointXYZI>::Ptr input);
  void dbh();           //show cylinder computed by Hough Transform with graph of accumulated sum
  void height();        //show line connecting lowest and highest point in tree cloud
  void position();      //show sphere at tree position
  void manualSelect();  //manually secting of trees
  void manualSelectStop();//stop manual tree selection
  void treeEdit();      //manual editing of tree cloud
  void treeEditStop();  //stop of manual editing of tree cloud
  void lenght();        //lenght of cloud
  void lenghtExport();
  void seg_dist();      //euclidian segmentation

////MISC
  void plusCloud();     //contencate cloud
  void plusCloud(QStringList names);
  void voxelize();  //voxels of various size on selected cloud
  void backgroundColor();
//// ABOUT
  void about();         //about 3D FOREST application
////TREE SLOTS
  void removeCloud();
  void dispCloud(QString name);
  void showCLoud(QString name);
  void removeCloud(QString name);
  void deleteCloud(QString name);
  void colorCloud(QString name);
//// ACTION and MENUS
 private:
  void createActions();
  void createMenus();
  void createToolBars();
  void createStatusBar();
  void createTreeView();
  void saveOstCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr s_cloud);
  void saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud);
  void saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud, QString filename,bool overwrt);
  //MENUS
    QMenu *fileMenu;
    QMenu *terenMenu;
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
    QAction *importPCDAct;
    QAction *importTerenAct;
    QAction *importVegeAct;
    QAction *importTreeAct;
    QAction *exportTXTAct;
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
    QAction *heightAct;
    QAction *posAct;
    QAction *manualSelAct;
    QAction *treeEditAct;
    QAction *lengAct;
    QAction *lengExAct;
    QAction *segDistAct;
  //MISC ACTIONS
    QAction *plusAct;
    QAction *voxAct;
    QAction *backgrdColAct;
  //ABOUT ACTIONS
    QAction *aboutAct;
    QAction *aboutQtAct;

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
  boost::shared_ptr<pcl::visualization::PCLVisualizer> m_vis;
  void AreaEvent(const pcl::visualization::AreaPickingEvent& event, void* );
  void ShowContextMenu(const QPoint& pos);

  pcl::PointCloud<pcl::PointXYZI>::Ptr lowPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr c,float res, bool v);


  Project *Proj;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud ;
  QString m_cloud_name;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud1;
  QString m_cloud1_name;
  void dispCloud(Cloud cloud);
  void dispCloud(Cloud cloud,int red, int green, int blue);

  QToolBar *fileToolBar;
  QToolBar *editToolBar;
  QProgressBar *pBar;
 };

#endif // MAINWINDOW_H_INCLUDED
