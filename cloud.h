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

  //! Struct consist for hold tree centre and diameter.
  /*! Struct of holding information about computed tree DBH with coordinates of centre (a,b) radius (r) and parameters of LSR computing . */
struct stredLSR {
				float a;  /**< the X-coordinate of the center of the fitting circle */
				float b;  /**< the Y-coordinate of the center of the fitting circle */
				float r;  /**< the radius of the fitting circle */
        float s;  /**< the root mean square error (the estimate of sigma) */
        float i;  /**< the total number of outer iterations (updating the parameters) */
        float j;  /**< the total number of inner iterations (adjusting lambda) */
        float g;  /**<  */
   bool operator < (const stred& str) const
    {return (i < str.i);}
				};


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
  Cloud *m_dbhCloud;          /**< pcl::pointcloud for points representing dbh_cloud */
  Cloud *m_convexhull;        /**< cloud of points representing convex hull of tree */
  Cloud *m_concavehull;       /**< cloud of points representing concave hull of tree */
  Cloud *m_skeleton;          /**< cloud of points representing skeleton of tree */
  stred m_dbh_HT;             /**< stred of the DBH with radius computed by HT */
  stred m_dbh_LSR;            /**< stred of the DBH with radius computed by LSR*/
  float m_height;             /**< tree height */
  float m_lenght;             /**< tree length */
  float m_areaconvex;         /**< Area of tree projection from top view */
  float m_areaconcave;        /**< Area of tree projection from top view */
  pcl::PointXYZI m_pose;      /**< tree position */
  pcl::PointXYZI m_minp;      /**< tree point with lowest coordinates, corner of boundary box */
  pcl::PointXYZI m_maxp;      /**< tree point with highest coordinates, corner of boundary box  */
  pcl::PointXYZI m_lmax;      /**< lowest point of tree for the longest axis */
  pcl::PointXYZI m_lmin;      /**< highest point of tree for the longest axis */


public:
    //! Constructor.
    /*! Costructor of tree. \param cloud tree pointCloud \param name of the tree \param col color of pointCloud \param s DBH value with estimated centre */
  Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col, stred s);
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
    //! Compute algebraic circle fit.
    /*! Compute algebraic circle fit for given pointCloud. \param cloud pointCloud representing DBH pointCloud. \return stred of computed fit */
  stred set_dbhLSRALG(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Compute geometric circle fit.
    /*! Compute geometricc circle fit for given pointCloud.
        \param circ input algebraic circle copmputed from set_dbh_LSRALG \param cloud pointCloud representing DBH pointCloud. \return stred of computed fit */
  stred set_dbhLSRGEOM(stred circ, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Compute  the root mean square error - sigma
    /*! Compute the square root of the average square of the distance for given pointCloud and computed circle.
    \param cloud pointCloud representing DBH pointCloud. \param circle computd stred \return value of sigma */
  float Sigma(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, stredLSR circle);
    //! Set tree height.
    /*! Compute and set tree height. */
  void set_height();
    //! Set tree position.
    /*! Compute and set tree position. */
  void set_position();
    //! Set tree position.
    /*! Compute and set tree position based on digital terrain model. \param teren cloud representing DMT */
  void set_position(Cloud teren);
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

    void set_convexhull();
    void set_convexhull(Cloud c);
    void set_concavehull(float maxEdgeLenght);
    void set_concavehull(Cloud c);
    void set_areavex(Cloud c);
    void set_areacave(Cloud c);
    float get_areaconvex();
    float get_areaconcave();
    Cloud get_vexhull();
    Cloud get_concavehull();

    void set_skeleton();
    void set_skeleton(Cloud c);
    Cloud get_skeleton();
};

#endif // CLOUD_H_INCLUDED