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

#ifndef HULL_H_INCLUDED
#define HULL_H_INCLUDED

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "cloud.h"

//! Class for convex hull cumputing.
/*! Class for computinh convex polygon and holding its attributes. */
class ConvexHull
{
protected:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;    /**< input point cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_convexhull;    /**< cloud representing convex hull */
    float m_polygonArea;    /**< area of convex hull */

public:
  //! Constructor.
    /*! Costructor of ConvexHull  */
    ConvexHull();
    //! Constructor.
    /*! Costructor of ConvexHull \param pcl point cloud \param name */
    ConvexHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Destructor.
    /*! Destructor of ConvexHull. */
    ~ConvexHull();

    void set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! execute computation.
    /*! Compute convex hull and area */
    void compute();
    //! Get polygon.
    /*! Get cloud cloud containing points in convex polygon \param pointer to cloud */
    void getPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Get polygon area.
    /*! Get convex polygon area in square meters \return float */
    float getPolygonArea();

private:
    //! Create convex hull.
    /*! Compute convex hull \param pcl point cloud with points \param pcl pointcloud for resulting polygon */
    void createConvexHull (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudForHull);
    //! Return point lowest YZ.
    /*! Return pcl point with xy coordinates equal to point with lowes y coordinate in cloud, z coordinate is equal to lowest z value in cloud
     \param pcl point cloud with points \return pcl point */
    pcl::PointXYZI returnPointLowestYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Return second point to polygon.
    /*! Return second point to polygon line \param pcl point cloud with points \param pcl point \return pcl point */
    pcl::PointXYZI returnSecondPointToPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI firstPoint);
    //! Return next point to hull.
    /*! Return next point into polygon \param pcl point cloud  \param pcl point \param pcl point \return pcl point */
    pcl::PointXYZI returnNextPointToHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointXYZI pointA, pcl::PointXYZI pointB);
    void createConvexIfOnlyFourPointsInCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};
//! Class for concave hull cumputing.
/*! Class for computinh concave polygon and holding its attributes. */
class ConcaveHull
{
protected:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;    /**< input point cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_concavehull;       /**< cloud representing concave hull */
    float m_polygonArea;        /**< area of concave hull */
    float m_searchingDistance;  /**<Start searching distance to find edge breaking point */

public:
    //! Constructor.
    /*! Costructor of ConcaveHull \param name \param pcl pointcloud \param float */
    ConcaveHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float searchDist=1.0);
    //! Destructor.
    /*! Destructor of ConcaveHull. */
    ~ConcaveHull();
    //! Get polygon.
    /*! Get cloud cloud containing points in convex polygon \param pointer to cloud */
    void getPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Get polygon area.
    /*! Get concave polygon area in square meters \return float */
    float getPolygonArea();
    //! Get polygon swapped ZI.
    /*! Get cloud cloud containing points in concave polygon with swapped z and intensity values \return pointer to cloud */
    void getPolygonSwappedZI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Get triangle at.
    /*! Get get triangle at iterator \return pcl::point cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getTriangleAt(int i);
    //! Get triangulated polygon.
    /*! Get pointer to triangulated polygon \return TriangulatedPolygon */
    float getSearchDist();
    //! Compute Attributes.
    /*! Compute concave hull and area */
    void compute();

private:
    //! Compute concave hull.
    /*! Compute concave hull */
    void computeConcaveHull();
    //! Edges breaking.
    /*! Walk areound polygon and break edges. \param pcl poin cloud \param pcl poin cloud \param float */
    void edgesBreaking (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud,float maxEdgeLenght);
    //! Return edge breaking point.
    /*! Find best point to break edge, if any point suit for conditions. \param pcl poin cloud \param pcl point \param pcl point \param float \return pcl point */
    pcl::PointXYZI returnEdgeBreakingPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointXYZI boda, pcl::PointXYZI bodb,float maxEdgeLenght);
};

#endif // HULL_H_INCLUDED
