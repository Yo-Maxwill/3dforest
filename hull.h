//    This file is part of 3DFOREST  www.3dforest.eu
//
//    3DFOREST is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    3DFOREST is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with 3DFOREST.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////

#ifndef HULL_H_INCLUDED
#define HULL_H_INCLUDED

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "cloud.h"
  //! Class for computing convex/concave hull of given cloud.
  /*! Algorithms for computing convex hull, concave hull in 2D or 3D space. */
class Hull : public Cloud
{
    Hull *m_hull;                      /**<  */
    Cloud *vexhull;                    /**< Cloud representing convex hull */
    Cloud *concavehull;                /**< Cloud representing concave hull */
    float m_areaconvex;                /**< Area of convex hull */
    float m_areaconcave;               /**< Area of concave hull */
    float maxEdgeLenght;               /**< Maximal length of edge */
    float vertical;                    /**< Vertical angle  */
    float direction;                   /**< Horizontal angle */

public:
      //! Default constructor
      /*! Constructor for creating Hull \param cloud pointcloud \param name name of the cloud \param col color of the cloud */
    Hull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col);
      //! Constructor
      /*! Constructor for creating Hull \param cloud pointcloud \param name name of the cloud */
    Hull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name);
      //! Constructor
      /*! Constructor for creating Hull \param Cloud of points*/
    Hull (Cloud cloud);
      //! Set convex hull for cloud
      /*! For m_cloud create convex hull and save it onto m_convexhull*/
    void set_convexhull();
      //! Set convex hull for cloud
      /*! \param c cloud representing hull points*/
    void set_convexhull(Cloud c);
    void set_concavehull(float maxEdgeLenght);
    int set_concaveZkracovanim(float maxEdgeLenght);
    void set_concavehull(Cloud c);
    bool if_intersect(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float x, float y,int &q);
    void set_areavex(Cloud c);
    void set_areacave(Cloud c);
    float get_areaconvex();
    float get_areaconcave();

    Cloud get_convexhull();
    Cloud get_concavehull();
    Hull get_Hull();
    void set_Hull(Hull hull);
    void returnConvexhull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudb);
    float returnDistance(pcl::PointXYZI boda, pcl::PointXYZI bodb);
    float return_clockwiseAngle(pcl::PointXYZI boda, pcl::PointXYZI bodc, pcl::PointXYZI bodb);
    void erasePointFromCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI bod);
    float returnPreponaDist (float a, float b);
    void PointToHull (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointXYZI boda, pcl::PointXYZI bodb, pcl::PointXYZI bodx);
};





#endif // HULL_H_INCLUDED
