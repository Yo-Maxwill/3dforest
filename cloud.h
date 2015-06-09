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

#ifndef CLOUD_H_INCLUDED
#define CLOUD_H_INCLUDED

#include <QtGui/QtGui>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

  //! Struct consist for hold tree centre and diameter.
  /*! Struct of holding information about computed tree DBH with coordinates of centre (a,b,z) radius (r) and voting value when computing (i) . */
struct stred {
				float a;    /**< the X-coordinate of the center of the fitting circle */
				float b;    /**< the Y-coordinate of the center of the fitting circle */
				float z;    /**< the Z-coordinate of the center of the fitting circle */
        float i;    /**< the total number of outer iterations, can be removed */
        float r;    /**< the radius of the fitting circle */
   bool operator < (const stred& str) const
    {return (i < str.i);}
				};



  //! Basic data class of pointcloud representation.
  /*! Class represents points cloud with color, point size and name.  */
class Cloud
{
protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_Cloud;   /**< pcl::pointcloud for points */
  QString m_name;                                 /**< cloud name */
  QColor m_color;                                 /**< cloud color */
  int m_PointSize;                                /**< size of point for display */

public:
    //! Constructor.
    /*! Costructor with all parameters \param cloud pointCloud itself \param name cloud name \param col color of cloud. */
  Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col);
    //! Constructor.
    /*! Costructor with parameters. color is randomly selected. \param cloud pointCloud itself \param name cloud name  */
  Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name);
    //! Constructor.
    /*! Empty costructor  */
  Cloud();
    //! Destructor.
    /*! Empty destructor  */
  ~Cloud();
    //! Copy Constructor.
    /*! Copy contructor */
  Cloud operator=(Cloud &kopie);

    //! Set new cloud.
    /*! Set new pointCloud for cloud. \param cloud reference to the pointCloud. */
  void set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Set new name.
    /*! Set new name for cloud. \param name name of the cloud. */
  void set_name(QString name);
    //! Set new color of pointCloud.
    /*! Set new color of pointCloud. \param col QColor value of color */
  void set_color (QColor col);
    //! Set point size.
    /*! Set point size for displaing in vizualizer. \param p size of point. */
  void set_Psize (int p);

    //! Get cloud.
    /*! Get cloud pointCloud. \return reference to the pointCloud of Cloud. */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_Cloud();
    //! Get name.
    /*! Get cloud name. \return name of the Cloud. */
  QString get_name();
    //! Get color.
    /*! Get cloud color. \return color of the Cloud. */
  QColor get_color();
    //! Get point size.
    /*! Get cloud point size. \return value of point size. */
  int get_Psize();
};
  //! Class for tree representation.
  /*! Class for holding information about single tree with all parameters. */
class Tree : public Cloud
{
  Cloud *m_dbhCloud;                  /**< pcl::pointcloud for points representing dbh_cloud */
  Cloud *m_convexhull;                /**< cloud of points representing convex hull of tree */
  Cloud *m_concavehull;               /**< cloud of points representing concave hull of tree */
  Cloud *m_skeleton;                  /**< cloud of points representing skeleton of tree */
  stred m_dbh_HT;                     /**< stred of the DBH with radius computed by HT */
  stred m_dbh_LSR;                    /**< stred of the DBH with radius computed by LSR*/
  float m_height;                     /**< tree height */
  float m_lenght;                     /**< tree length */
  float m_areaconvex;                 /**< Area of tree projection from top view */
  float m_areaconcave;                /**< Area of tree projection from top view */
  pcl::PointXYZI m_pose;              /**< tree position */
  pcl::PointXYZI m_minp;              /**< tree point with lowest coordinates, corner of boundary box */
  pcl::PointXYZI m_maxp;              /**< tree point with highest coordinates, corner of boundary box  */
  pcl::PointXYZI m_lmax;              /**< lowest point of tree for the longest axis */
  pcl::PointXYZI m_lmin;              /**< highest point of tree for the longest axis */
  std::vector<stred> m_stemCurvature; /**< vector of all rings presenting stem curve */

public:
    //! Constructor.
    /*! Costructor of tree. \param cloud tree pointCloud \param name of the tree \param col color of pointCloud  */
  Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col);
    //! Constructor.
    /*! Costructor of tree. \param cloud Cloud */
  Tree (Cloud cloud);
    //! Constructor.
    /*! Copy Costructor of tree. \param kopie tree copy */
  Tree operator=(Tree &kopie);
    //! Set dbh cloud.
    /*! Set cloud representing point for computing DBH  from tree pointCloud*/
  void set_dbhCloud();
    //! Set dbh cloud.
    /*! Set cloud representing point for computing DBH  \param cloud pointCloud representing points for computing DBH*/
  void set_dbhCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Set dbh value.
    /*! Compute DBh value and centre of the circle using Randomized Hough Transform. */
  void set_dbhHT();
    //! Get dbh value.
    /*! Compute DBH value and centre of the circle using Randomized Hough Transform. \return stred structure consist of DBH, centre */
  stred get_dbhHT();
  //! Get dbh value.
    /*! Compute DBH value and centre of the circle using Least Square Regression. \return stred structure consist of DBH, centre */
  stred get_dbhLSR();
    //! Set dbh cloud using LSR.
    /*! Compute DBH using Least Square Regression computing. and save it into m_dbh */
  void set_dbhLSR();
    //! Set tree height.
    /*! Compute and set tree height. */
  void set_height();
    //! Set tree position.
    /*! Compute and set tree position. */
  void set_position();
  //! Set tree position.
    /*! Compute and set tree position. */
  void set_positionRHT();
    //! Set tree position.
    /*! Compute and set tree position based on digital terrain model. \param terrain cloud representing DMT */
  void set_position(Cloud terrain);
  //! Set tree position.
    /*! Compute and set tree position. */
  void set_positionHT(Cloud terrain);
    //! Set cloud length.
    /*! Compute and set tree cloud length. */
  void set_length();

    //! Get tree dbh_pointCloud.
    /*! Get tree DBH_pointCLoud representing points for compute DBH \return pointCloud for DBh estimate. */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_dbhCloud();
    //! Get tree height.
    /*! get tree height \return value of computed tree height. */
  float get_height();
    //! Get tree position.
    /*! Ger tree position. \return pcl::point representing position of tree */
  pcl::PointXYZI get_pose();
    //! Get cloud length.
    /*! Get tree cloud length.  \return value of tree cloud length*/
  float get_length();
    //! Get lowest/highest point of tree.
    /*! \param low if true get lowest point else highest \return pcl::point representing lowest/highest point of tree cloud.*/
  pcl::PointXYZI get_lpoint(bool);

    //! Set convex planar projection.
    /*! Compute points representing convex planar projection and save them into m_convexHull*/
  void set_convexhull();
    //! Set convex planar projection.
    /*! Compute points representing convex planar projection and save them into m_convexHull \param c cloud of points representing convex hull*/
  void set_convexhull(Cloud c);
    //! Set concave planar projection.
    /*! Compute points representing concave planar projection and save them into m_concaveHull \param maxEdgeLenght cmaximal length of edge*/
  int set_concavehull(float maxEdgeLenght);
    //! Set concave planar projection.
    /*! Compute points representing concave planar projection and save them into m_concaveHull \param c cloud of points representing concave hull*/
  void set_concavehull(Cloud c);
    //! Get convex planar projection area.
    /*! \return float area of polygon representing convex planar projection in square meters*/
  float get_areaconvex();
    //! Get concave planar projection area.
    /*! \return float area of polygon representing concave planar projection in square meters*/
  float get_areaconcave();
    //! Get convex planar projection cloud.
    /*! \return cloud representing convex planar projection */
  Cloud get_vexhull();
    //! Get concave planar projection cloud.
    /*! \return cloud representing concave planar projection */
  Cloud get_concavehull();
    //! Set skeleton for tree.
    /*! Compute tree skeleton based and save it onto m_skeleton*/
  void set_skeleton();
    //! Set skeleton for tree.
    /*! Compute tree skeleton based and save it onto m_skeleton \param c cloud of points representing skeleton*/
  void set_skeleton(Cloud c);
    //! Get skeleton of tree.
    /*! \return cloud representing tree skeleton */
  Cloud get_skeleton();
    //! Get stem curve rings.
    /*! \return vector of stem curve rings */
  std::vector<stred> get_stemCurvature();
    //! Set stem curve rings.
  void set_stemCurvature();
};

#endif // CLOUD_H_INCLUDED
