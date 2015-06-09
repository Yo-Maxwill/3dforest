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
#ifndef MAINWINDOW_H_INCLUDED
#define MAINWINDOW_H_INCLUDED

#include <QVTKWidget.h>
#include <QtGui/QMainWindow>
#include <gui.h>
#include <project.h>
#include "hull.h"

//!  Main application window.
/*!
  Main class for application window. Contains slots, method actions and signals for
  project functions.
*/
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
    //! Constructor.
    /*! Main application constructor. */
    MainWindow();
    //! path of project.
    /*! Return path of current project.
    \return QString path to the directory of project. */
    QString get_path();

private slots:

//PROJECT
  //project manager
    //! New project.
    /*! Create new project to store cloud data. */
  void newProject();
    //! Open existing project.
    /*! Slot for openning existing project. */
  void openProject();
    //! Close project.
    /*! Close open project, delete all visible shapes, clouds and restore project vectors to 0. */
  void closeProject();
    //! Import project.
    /*! Import existing project to new path and correct all path in proj file. */
  void importProject();
  //file import/open

    //! Import PCD file.
    /*! Import existing file into project as a terrain cloud.  */
  void importTerrainFile();
    //! Import Basic cloud file.
    /*! Import existing file into project as a base cloud.  */
  void importBaseCloud();
    //! Import PCD file.
    /*! Import existing file into project as a vegetation cloud.  */
  void importVegeCloud();
    //! Import PCD file.
    /*! Import existing file into project as a tree cloud.  */
  void importTreeCloud();
  //file export
    //! Export cloud.
    /*! Export cloud into text file  */
  void exportCloud();
  //! Export cloud into ply file.
    /*! Export selected cloud into ply file  */
  void plysave();
    //! Export cloud to pts file.
    /*! Export cloud into pts file  */
  void exportPts();
    //! Export convex hull.
    /*! Export convex hull of tree into text file. */
  void exportConvexTxt();
    //! Export concave hull.
    /*! Export concave hull of tree into text file. */
  void exportConcaveTxt();
  //exit
    //! Close event.
    /*! Close main window.
        \param event is called by application signal Close event */
  void closeEvent(QCloseEvent *event);

//TERAIN
    //! Terrain voxel grid.
    /*! voxelization of input cloud and selecting lowest point as a terrain cloud and others as vegetation cloud. */
  void voxelgrid();
     //! Octree Terrain.
    /*! select lowest part of input cloud using octree search with smallest resolution as a input value */
  void octreeSlot();
    //! Manual adjustment of terrain cloud.
    /*! Manual delete of point falsely classified as a ground */
  void manualAdjust();
  //! Save manually adjusted terrain cloud.
    /*! Save adjusted terrain cloud into old file and deleted point into new file */
  void manualAdjustStop();

//VEGETATION
    //! Manual selection of trees from vegetation cloud.
    /*! Choose vegetation cloud for tree selection. Delete points that do not belong to the tree*/
  void manualSelect();
    //! Save manually selected tree cloud.
    /*! Save selected tree cloud into new file and if user do not want to continue save  rest into output file. */
  void manualSelectStop();
// TREE ATRIBUTES
    //! Manual editing of tree cloud.
    /*! Choose tree cloud for manual editing. */
  void treeEdit();
    //! Save manually edited tree cloud.
    /*! Save edited tree cloud into old file. Deleted points are saved into new file */
  void treeEditStop();
    //! Compute tree attributes for all trees and save them into file.
    /*! Compute tree attributes for all trees and save them into file. User can select file name and path for the file.  */
  void treeAtributes();
    //! Compute DBH using Randomized Hough Transform.
    /*! Compute DBH for given tree with method of Randomized Hough Transform for circle detection. User can select single tree or all_trees.
        Result is displayed as a cylinder in 1,3 m from position with green value of estimated DBH. */
  void dbhHT();
    //! Display DBH computed by Randomized Hough Transform.
    //!  For given cloud name show dbh_cloud and cylinder with text of DBH value
    /*! \param name name of tree cloud.*/
  void dbhHTDisplay(QString name);
    //! Display DBH computed by Randomized Hough Transform for all trees.
  void dbhHT_DisplayAll();
    //! Hide cylinder and DBH_cloud fro all trees.
  void dbhHT_HideAll();
    //! Compute and display DBH using Least Square Regression.
    /*! Compute DBH for given tree with method of Least Square Regression for circle fitting. User can select single tree or all_trees.
        Result is displayed as a cylinder in 1,3 m from position with beige value of estimated DBH.*/
  void dbhLSR();
  //! Compare DBH computed by two methods and if they are different more that 1 cm  put them in the list .
    /*! Compute DBH for given tree with method of Least Square Regression for circle fitting. User can select single tree or all_trees.
        Result is displayed as a cylinder in 1,3 m from position with beige value of estimated DBH.*/
  void dbhCheck();
    //! Display DBH computed using Least Square Regression.
    /*! For given cloud name show dbh_cloud and cylinder with text of DBH value*/
    /*! \param name name of tree cloud.*/
  void dbhLSRDisplay(QString name);
    //! Display DBH computed by Least Square Regression for all trees.
  void dbhLSR_DisplayAll();
    //! Hide cylinder and DBH_cloud fro all trees.
  void dbhLSR_HideAll();
    //! Compute tree height.
    /*! Compute and display height for given tree as diff of Z coordinate between highest point of tree and Z coordinate of input terrain point closest to the tree position.
        Lenght is displayed as a line connection those two points and beige value of height. */
  void height();
    //! Display height for given tree.
    /*! For given cloud name show line starting at tree position and follows Z axis up to the height of the highest pint of tree. at the end is dispalyed test with value.*/
    /*! \param name name of tree cloud.*/
  void heightDisplay(QString name);
    //! Display Height for all trees.
  void height_DisplayAll();
    //! Hide line and text of height for all trees.
  void height_HideAll();
    //! Compute tree position.
    /*! Compute tree position using median coordinate of all point that are up to given height above lowest point of cloud.
        Position is displayed as a sphere with centre at position and radius of 10 cm. */
  void position();
  //! Compute tree position using hough transform.
    /*! Compute tree position using median coordinate of all point that are up to given height above lowest point of cloud.
        Position is displayed as a sphere with centre at position and radius of 10 cm. */
  void positionHT();
    //! Display position as s sphere for given tree.
    /*! For given cloud name show sphere at tree position.*/
    /*! \param name name of tree cloud.*/
  void positionDisplay(QString name);
    //! Display sphere at position for all trees.
  void position_DisplayAll();
    //! Hide sphere at position for all trees.
  void position_HideAll();
    //! Compute cloud lenght.
    /*! Compute hlenght for given tree as as diff of coordinates of two selected points.
        it select axis with biggest value range. On this range select extreme points and compute distance between those points.
        Lenght is displayed as a line connection those two points and green value of lenght.  */
  void length();
    //! Display length for given tree.
    /*! For given cloud name show line of two points with greatest distance between in cloud and text with distance at starting point.*/
    /*! \param name name of tree cloud.*/
  void lengthDisplay(QString name);
    //! Display cloud length for all trees.
  void length_DisplayAll();
    //! Hide line and text of cloud length for all trees.
  void length_HideAll();
    //! Compute and display tree skeleton.
    /*! Slot for selecting tree cloud and compute skeleton. skeleton is saved in project with ext skel. and it is loaded automatically with tree. */
  void skeleton();
    //! Display tree skeleton.
    /*! Display lines connected in tree skeleton. */
    /*! \param name name of tree cloud.*/
  void skeletonDisplay(QString name);
    //! Display tree skeleton for all trees.
    /*! Display lines connected in tree skeleton for all trees. */
  void skeleton_DisplayAll();
    //! Hide skeleton of all trees.
  void skeleton_HideAll();
    //! Compute and display convex planar projection of tree.
    /*! Compute convex planar projection of tree and display.  */
  void convexhull();
    //! Display convex planar projection of tree.
    /*! Display convex planar projection of tree and display it as a polygon with 50% opacity and with text at tree position with value of polygon area.  */
    /*! \param name name of tree cloud.*/
  void convexhullDisplay(QString name);
    //! Display convex planar projection of all trees.
  void convexhull_DisplayAll();
    //! Hide convex planar projection of all trees.
  void convexhull_HideAll();
    //! Compute and display concave planar projection of tree.
    /*! Compute concave planar projection of tree and display.  */
  void concavehull();
    //! Display concave planar projection of tree.
    /*! Display concave planar projection of tree and display it as a polygon with 50% opacity and with text at tree position with value of polygon area.  */
    /*! \param name name of tree cloud.*/
  void concavehullDisplay(QString name);
    //! Display concave planar projection of all trees.
  void concavehull_DisplayAll();
    //! Hide concave planar projection of all trees.
  void concavehull_HideAll();
    //! Compute and display stem curve.
    /*! Compute and display stem curve rings */
  void stemCurvature();
    //! Display stem curve.
    /*! Display stem curve rings */
    /*! \param name name of tree cloud.*/
  void stemCurvatureDisplay(QString name);
    //! Displaystem curven for all trees.
    /*! Display rings in 1 meter range for whole stem.  */
  void stemCurvature_DisplayAll();
    //! Hide stem curve rings of all trees.
  void stemCurvature_HideAll();
  //! Export stem curve into text file.
  void stemCurvatureExport();
    //! Manual editing of tree DBH cloud.
    /*! Choose tree cloud for manual editing of cloud representing points for estimating DBH. */
  void dbhCloudEdit();
    //! Stop manual editing of tree DBH cloud.
    /*! Save edited DBH cloud of given tree. */
  void dbhCloudStopEdit();


//MISC
    //! Join two cloud into one.
    /*! join two input clouds into new one.  Slot only displays input dialog for entering clouds and new name.  */
  void plusCloud();
    //! subtract two clouds.
    /*! Choose two input clouds and from the bigger one subtract all point that are common with the second one. Result save into new file.  */
  void minusCloud();
    //! Voxelize cloud.
    /*! For given cloud make new voxelized cloud with given resolution of voxel. */
  void voxelize();
    //! IDW for given clouds - special purpose function only.
    /*! input cloud are: reference terrain cloud, selected ground points for computing idw, original cloud for computing new height.  */
  void IDW();
    //! Clip of two cloud - special purpose function only.
    /*! input clouds: original cloud, reference cloud. serve for editing clouds in strips of given width. work only in special angle of viewing clouds.  */
  void clip();
    //! Clip of two cloud - special purpose function only.
    /*! if one strip is ready, you can switch to next one  */
  void clipped();
    //! Clip of two cloud - special purpose function only.
    /*! finish editing clipping */
  void clipStop();
  //! Create concave hull of cloud.
    /*! output is new ost cloud*/
  void set_ConcaveCloud();
    //! Create convex hull of cloud.
    /*! output is new ost cloud*/
  void set_ConvexCloud();
    //! save vtkwidget into tiff file.
    /*! output actual visualizatin in vtkcanvas into file*/
  void save_tiff();
    //! Change background color of vTK widget.
    /*! select color for background*/
  void bgColor();

// ABOUT
    //! Information about application
    /*! Basic information about application purpose and authors.  */
  void about();

//TREEVIEW SLOTS
    //! Display cloud
    /*! Display cloud in m_vis.
    \param name name of cloud */
  void dispCloud(QString name);
    //! Remove displayed cloud
    /*! Remove displayed cloud from m_vis.
        \param name name of cloud */
  void removeCloud(QString name);
    //! Delete cloud
    /*! Delete given cloud from project and optionally from disc.
    \param name name of cloud */
  void deleteCloud(QString name);
    //! Color cloud
    /*! Color cloud based on selection of color.
    \param name name of cloud */
  void colorCloud(QString name);
    //! Color cloud
    /*! Color cloud based on selection of point field (x,y,z, intensity) .
    \param name name of cloud */
  void colorCloudField(QString name);
  //! Pint size of cloud
    /*! Display points of cloud with given size
    \param name name of cloud */
  void PointSize(QString name);
  //! Restore deleted point during area picking event
    /*! Restore points removed during area picking event. Function is connected to undo button. */
  void undo();

private:
  //application widgets and actions
    //! Create actions
    /*! Create main window actions.  */
  void createActions();
    //! Create menus
    /*! Create main window menu.  */
  void createMenus();
    //! Create treeview
    /*! Create main window widget Treeview on left side of application window. */
  void createTreeView();
  //! Create toolbars
    /*! Create toolbars for display/hide tree parameters.  */
  void createToolbars();
//import various formats
    //! Import text file.
    /*! Import existing text file into project as a base cloud.  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr importTXT(QString file);
    //! Import LAS file.
    /*! Import existing las file into project as a base cloud.  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr importLAS(QString file);
    //! Import PCD file.
    /*! Import existing text file into project as a base cloud.  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr importPCD(QString file);
    //! Import PTS Leica file.
    /*! Import existing pts file into project as a base cloud. Fields of this file is: X Y Z intensity  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr importPTS(QString file);
    //! Import PTX Leica file.
    /*! Import existing ptx file into project as a base cloud.  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr importPTX(QString file);
//opening methods
    //! Open project file
    /*! Open project file load transformation matrix, all clouds and display it.
    \param path  Path to the proj file */
  void openProject(QString path);
    //! Open terrain file
    /*! Open terrain file with randomly selected color of cloud and save it into project terrain cloud vector.
    \param file path to the file */
  void openTerrainFile(QString file);
    //! Open vegetation file
    /*! Open vegetation file with randomly selected color of cloud and save it into project vegetation cloud vector.
    \param file path to the file */
  void openVegeFile(QString file);
    //! Open ost file
    /*! Open ost  file with randomly selected color of cloud and save it into project ost cloud vector.
    \param file path to the file */
  void openOstFile(QString file);
    //! Open tree file
    /*! Open tree file with randomly selected color of cloud and save it into project tree cloud vector.
    \param file path to the file */
  void openTreeFile(QString file);
    //! Open base cloud file
    /*! Open tree file with randomly selected color of cloud and save it into project base cloud vector.
    \param file path to the file */
  void openCloudFile(QString file);
    //! Open terrain file
    /*! Open terrain file with selected color of cloud and save it into project terrain cloud vector.
    \param file path to the file \param col QColor defining cloud color */
  void openTerrainFile(QString file, QColor col);
    //! Open vegetation file
    /*! Open vegetation file with selected color of cloud and save it into project vegetation cloud vector.
    \param file path to the file \param col QColor defining cloud color */
  void openVegeFile(QString file, QColor col);
    //! Open ost file
    /*! Open ost file with selected color of cloud and save it into project tree cloud vector.
    \param file path to the file \param col QColor defining cloud color */
  void openOstFile(QString file, QColor col);
    //! Open tree file
    /*! Open tree file with selected color of cloud and save it into project tree cloud vector.
    \param file  path to the file \param col QColor defining cloud color */
  void openTreeFile(QString file, QColor col);
    //! Open base cloud file
    /*! Open tree file with selected color of cloud and save it into project base cloud vector.
    \param file  path to the file \param col QColor defining cloud color */
  void openCloudFile(QString file, QColor col);

//QVTKWIDGET - display and hide clouds
  QVTKWidget *qvtkwidget;               /**< Define QVTKWidget */
  Visualizer *m_vis;                    /**< Visualizer */
  boost::signals2::connection area;     /**< Boost signal for connecting area picking events */
  boost::signals2::connection point_ev; /**< Boost signal for connecting point picking events */
    //! Display cloud
    /*! Display cloud in m_vis.
    \param cloud  cloud for displaying */
  void dispCloud(Cloud cloud);
    //! Display cloud
    /*! Display cloud in m_vis.
        \param cloud  Displaying cloud \param field of Point field to color cloud based on value of this field */
  void dispCloud(Cloud cloud, QString field);
    //! Display cloud
    /*! Display cloud in m_vis.
        \param cloud Displaying cloud \param red red part of color in range 0 - 255 \param green green part of color in range 0 - 255
        \param blue part of color in range 0 - 255 */
  void dispCloud(Cloud cloud,int red, int green, int blue);
    //! Area picking event
    /*! Area picking event for cloud editing, adjustment and selection
        \param event pcl::visualization::AreaPickingEvent  */
  void AreaEvent(const pcl::visualization::AreaPickingEvent& event, void* );
    //! Area picking event
    /*! Area picking event fonly for clipping functions
        \param event pcl::visualization::AreaPickingEvent  */
  void AreaEvent2(const pcl::visualization::AreaPickingEvent& event, void* );
  //! Point picking event
    /*! If selected point displays information about cloud containing this point
        \param event pcl::visualization::AreaPickingEvent  */
  void pointEvent(const pcl::visualization::PointPickingEvent& event, void*);

//TREEWIDGET
  MyTree *treeWidget; /**< Define MyTree Widget */
  QList<QTreeWidgetItem *> tree_items; /**< Define tree items */
    //! Add tree item into treeWidget
    /*! Adding tree item into treeWidget
        \param name  cloud name */
  void addTreeItem(QString name);

// EDIT ToolBar
  QToolBar *editBar;/**< Define editBar Widget */
  std::vector<int> undopoint;/**< Vector of points selected during area picking event */


// save functions
    //! Save cloud as a ost type.
    /*! Save selected cloud as a "ost" type in project.
        \param s_cloud  input cloud */
  void saveOstCloud(Cloud *s_cloud);
    //! Save cloud as a tree type.
    /*! Save selected pointcloud as a new tree in project.
        \param tree_cloud input pointcloud */
  void saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud);
    //! Save cloud as a tree type.
    /*! Save selected pointcloud as a new tree in project .
        \param tree_cloud input pointcloud \param name name of the cloud \param overwrt overwrite existing file */
  void saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud, QString name,bool overwrt);
    //! Save cloud as a tree type.
    /*! Save selected cloud as a new tree in project.
        \param s_cloud Input Cloud */
  void saveTreeCloud(Cloud *s_cloud);
    //! Save cloud as a vegetation type.
    /*! Save selected cloud as a new vegetation cloud in project.
        \param s_cloud Input Cloud */
  void saveVegeCloud(Cloud *s_cloud);
    //! Save cloud as a vegetation type.
    /*! Save selected cloud as a new vegetation cloud in project.
        \param s_cloud Input Cloud \param overwrt overwrite existing file  */
  void saveVegeCloud(Cloud *s_cloud, bool overwrt );
    //! Save cloud as a terrain type.
    /*! Save selected cloud as a new terrain cloud in project.
        \param s_cloud Input Cloud */
  void saveTerrainCloud(Cloud *s_cloud);

    //! Segment input_cloud into vegetation and ground.
    /*! Method for segmenting input cloud into vegetation and ground cloud with given resolution.
        \param res resolution of smallest octree searchbox \param input Input Cloud \param output_vege output cloud of vegetation
        \param output_ground output cloud of terrain */
  void octree(float res, pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output_vege, pcl::PointCloud<pcl::PointXYZI>::Ptr output_ground);
    //! Select lowest/highest point of cloud.
    /*! From given pointCloud select all points that have in given resolution the lowest/highest z coordinate.
        \param c Input pointCloud \param res Resolution \param v if true select lowest point else highest
        \return  pointCloud */
  pcl::PointCloud<pcl::PointXYZI>::Ptr lowPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr c,float res, bool v);
    //! Join two cloud into one.
    /*! join two input clouds into new one.
    \param  input1 first input cloud \param  input2 second input cloud \param  output name of output cloud
    \param  typ type of new cloud, default is "ost". */
  void plusCloud(QString, QString, QString, QString );
    //! Make strip of two clouds based on strip widht.
    /*! Create to strips from two clouds. Both strip starts with the same X coordiante and is widht according to resolution.
        \param cl reference terrain cloud \param cl2 original cloud \param res widht of strip  */
  void clip(Cloud cl, Cloud cl2, int res);
  int m_width;  /**< width of clipped strip*/
  int m_res;    /**< resolution of clipped strip */

//cloud names
    //! Names of all clouds in project.
    /*! Return names of all clouds in project
        \return QStringList of all cloud names */
  QStringList get_allNames();
    //! Names of all tree clouds in project.
    /*! Return names of all tree clouds in project
        \return QStringList of all tree cloud names */
  QStringList get_treeNames();
    //! Names of all terrain clouds in project.
    /*! Return names of all terrain clouds in project
        \return QStringList of all terrain cloud names */
  QStringList get_terrainNames();
  //! Names of all vegetation clouds in project.
    /*! Return names of all vegetation clouds in project
        \return QStringList of all vegetation cloud names */
  QStringList get_vegetationNames();
  //! Names of all tost clouds in project.
    /*! Return names of all ost clouds in project
        \return QStringList of all ost cloud names */
  QStringList get_ostNames();
  //! Names of all base clouds in project.
    /*! Return names of all base clouds in project
        \return QStringList of all base cloud names */
  QStringList get_baseNames();
  //! Check if given name eist in project.
    /*!
        \return QString ogiven new name*/
  QString name_Exists();

  QProgressBar *pBar;          /**< ProgressBar definition */
  //MENUS
  QMenu *fileMenu;            /**< Project menu */
  QMenu *importMenu;          /**< Import menu */
  QMenu *exportMenu;          /**< Import menu */
  QMenu *terenMenu;           /**< Terrain menu */
  QMenu *vegeMenu;            /**< Vegetation menu */
  QMenu *treeMenu;            /**< Tree attributes menu */
  QMenu *helpMenu;            /**< About menu */
  QMenu *miscMenu;             /**< Other features menu */

//PROJECT ACTIONS
  QAction *new_projectAct;    /**< New project Act */
  QAction *open_projectAct;   /**< Open project Act */
  QAction *close_projectAct;  /**< Close project Act */
  QAction *import_projectAct; /**< Import Project Act */
  QAction *importBaseAct;     /**< Import files as base cloud Act */
  QAction *importTerenAct;    /**< Import files as terrain cloud Act */
  QAction *importVegeAct;     /**< Import files as vegetation cloud Act */
  QAction *importTreeAct;     /**< Import files as tree cloud Act */
  QAction *exportTXTAct;      /**< Export cloud as text file Act */
  QAction *exportPLYAct;      /**< Export cloud as PLY file Act */
  QAction *exportPTSAct;      /**< Export cloud as PTS file Act */
  QAction *exportCONVEXAct;   /**< Export tree convex hull Act */
  QAction *exportCONCAVEAct;  /**< Export tree concave hull Act */
  QAction *exitAct;           /**< Close application Act */
  //TEREN ACTIONS
  QAction *voxelAct;          /**< Voxelized terrain Act */
  QAction *octreeAct;         /**< Terrain by octree Act */
  QAction *manualADAct;       /**< manula terrain adjustment Act */
  //TREE ATRIBUTES ACTIONS
  QAction *tAAct;             /**< Save tree attributes into file Act */
  QAction *dbhHTAct;          /**< Compute DBH using RHT and display Act */
  QAction *dbhLSRAct;         /**< Compute DBH using LSR and display Act */
  QAction *heightAct;         /**< Compute Height and display Act */
  QAction *posAct;            /**< Compute Position and display Act */
  QAction *posHTAct;            /**< Compute Position and display Act */
  QAction *manualSelAct;      /**< Manual selection of trees Act */
  QAction *treeEditAct;       /**< Manual editing of tree cloud Act */
  QAction *dbhEditAct;        /**< Manual editing of tree DBH cloud Act */
  QAction *dbhCheckAct;       /**< Check DBH computation of both methods  */
  QAction *lengAct;           /**< Compute cloud length and display Act */
  QAction *skeletonAct;       /**< Compute tree skeleton and display Act */
  QAction *convexAct;         /**< Compute convex hull Act */
  QAction *concaveAct;        /**< Compute concave hull Act */
  QAction *stemCurvatureAct;  /**< Compute stem curve Act */
  QAction *exportStemCurvAct; /**< Export given tree stem curve Act*/

  //MISC ACTIONS
  QAction *plusAct;           /**< Merge two cloud into single one Act */
  QAction *minusAct;          /**< Subtract two cloud Act */
  QAction *voxAct;            /**< Voxelize cloud Act */
  QAction *clipAct;           /**< Clipping Act */
  QAction *clipedAct;         /**< Clipping Act 2 */
  QAction *convexCloudAct;    /**< ConvexCloud Act */
  QAction *concaveCloudAct;   /**< ConcaveCloud Act */
  QAction *tiffAct;           /**< save vtkwidget into tiff file Act */
  QAction *bgcolorAct;         /**< Change background color Act */


  //ABOUT ACTIONS
  QAction *aboutAct;          /**< About application Act */
  QAction *aboutQtAct;        /**< About Qt Act */
  QAction *IDWAct;            /**< IDW Act */

  //Treebar actions
  QToolBar *treeBar;          /**< Toolbar with display/hide tree attributes */
  QAction *dbhthT;            /**< display/hide cylinders and text associated to dbhHT. Used in treeBar */
  QAction *dbhlsrT;           /**< display/hide cylinders and text associated to dbhLSR. Used in treeBar */
  QAction *heightT;           /**< display/hide  line and text associated to height. Used in treeBar */
  QAction *positionT;         /**< display/hide  sphere in tree position. Used in treeBar */
  QAction *lengthT;           /**< display/hide  line connecting the most remote points. Used in treeBar */
  QAction *convexT;           /**< display/hide  points representing convex projection of tree. Used in treeBar */
  QAction *concaveT;          /**< display/hide  points representing concave projection of tree. Used in treeBar */
  QAction *skeletonT;         /**< display/hide  lines connected in tree skeleton. Used in treeBar */
  QAction *stemCurveT;         /**< display/hide  tree rings representing stem curve*/
  Project *Proj;              /**< Project definition */
  Cloud *m_cloud;             /**< temporary cloud serves mainly in editing mode */
  Cloud *m_cloud1;            /**< temporary cloud serves mainly in editing mode */
  Cloud *m_cloud2;            /**< temporary cloud serves mainly in editing mode*/

 };

#endif // MAINWINDOW_H_INCLUDED
