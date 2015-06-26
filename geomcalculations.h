#ifndef GEOMCALCULATIONS_H_INCLUDED
#define GEOMCALCULATIONS_H_INCLUDED

#include <QtGui/QtGui>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

    //! Struct hold cloud highest and lowest Z value.
    /*! Struct holding highest and lowest value of the cloud */
struct cloudHighestAndLowestZValue{
float Highest;
float Lowest;
};

    //! Struct hold points with longest distance.
    /*! Struct holding points with longest distance between them */
struct pointsWithLongestDist{
pcl::PointXYZI pointA;
pcl::PointXYZI pointB;
};

    //! Class with basic geometric functions.
    /*! Class containing basic functions for geometric calculations with PCL points and clouds. */
class GeomCalc
{
public:
    //! Find cloud highest and lowest Z value.
    /*! Find cloud highest and lowest Z value. \param pcl point cloud \return struct */
    static cloudHighestAndLowestZValue findHighestAndLowestPoints (pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI);
    //! Points with longest distance.
    /*! Find points with longest distance between them in the cloud. \param pcl point cloud \return struct */
    static pointsWithLongestDist findPointsWithLongestDistance (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Calculate distance in XY plane.
    /*! Calculate distance in XY plane between two points. \param pcl point \param pcl point \return float */
    static float computeDistance2Dxy(pcl::PointXYZI boda, pcl::PointXYZI bodb);
    //! Calculate distance.
    /*! Calculate distance between two points. \param pcl point \param pcl point \return float */
    static float computeDistance3D(pcl::PointXYZI boda, pcl::PointXYZI bodb);
    //! Compute clockwise angle.
    /*! Compute clockwise angle between vectors AB and BC. \param pcl point A \param pcl point B \param pcl point C \return float */
    static float computeClockwiseAngle(pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC);
    //! Find longest perpendicular distance.
    /*! Find longest perpendicular distance to vector AB in point cloud . \param pcl point cloud \param pcl point A \param pcl point B \return float */
    static float findLongestPerpendicularDistance (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI &pointA, pcl::PointXYZI &pointB);
    //! Calculate perpendicular distance.
    /*! Calculate perpendicular distance to vector AB from point C . \param pcl point A \param pcl point B \param pcl point C \return float */
    static float computePerpendicularDistanceFromPointC (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC);
    //! Calculate triangle perimeter.
    /*! Calculate triangle perimeter. \param pcl point A \param pcl point B \param pcl point C \return float */
    static float computeTrianglePerimeter (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC);
    //! Calculate triangle area.
    /*! Calculate triangle area using Herons formula. \param pcl point A \param pcl point B \param pcl point C \return float */
    static float computeTriangleArea (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC);
    //! Compute polygon area.
    /*! Compute planar polygon area. \param pcl point cloud \return float */
    static float computePolygonArea (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Is line intersection.
    /*! Check if two line segments are intersect. \param pcl point \param pcl point \param pcl point \param pcl point \return bool */
    static bool isLineIntersection(pcl::PointXYZI a1, pcl::PointXYZI a2,pcl::PointXYZI b1,pcl::PointXYZI b2);
    //! Is any edge intersect.
    /*! Check if edge intersect any adge from the list of edges (polygon order). \param pcl point cloud \param pcl point \param pcl point \return bool */
    static bool isAnyEdgeIntersect(pcl::PointCloud<pcl::PointXYZI>::Ptr listOfEdges, pcl::PointXYZI newEdgePtA, pcl::PointXYZI newEdgePtB);
    //! Is any point in triangle.
    /*! Check if any point lies inside the given triangle. \param pcl point cloud \param pcl point \param pcl point \param pcl point \return bool */
    static bool isAnyPointInTriangle(pcl::PointCloud<pcl::PointXYZI>::Ptr points,pcl::PointXYZI a,pcl::PointXYZI b,pcl::PointXYZI c);
     //! Is point in triangle.
    /*! Check if point lies inside the given triangle. \param pcl point \param pcl point \param pcl point \param pcl point \return bool */
    static bool isPointInTriangle(pcl::PointXYZI testedPoint,pcl::PointXYZI a,pcl::PointXYZI b,pcl::PointXYZI c);
    //! Is XY equal.
    /*! Check if XY coordinates of given points are equal. \param pcl point \param pcl point \return bool */
    static bool isXYequal (pcl::PointXYZI pointA, pcl::PointXYZI pointB);
};

    //! Class for cloud operations
    /*! Class containing basic functions for cloud operations. */
class CloudOperations
{
public:
    //! Cloud XYZI to XYZ.
    /*! Copy XYZ vylue from XYZI cloud to XYZ point cloud type. \param pcl point cloud XYZI \param pcl point cloud XYZ */
    static void cloudXYZItoXYZ (pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ);
    //! Transform cloud to plane.
    /*! Transform given cloud to plane at lowest point level by horizontal direction and vertical angle. \param pcl point cloud
    \param float horizontal direction \param float vertical angle */
    static void transformCloudToPlane (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float direction, float vertical);
    //! Erase point from cloud .
    /*! Erase all point with equal XY coordinates from cloud. \param pcl point cloud \param pcl point */
    static void erasePointFromCloudXY (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI bod);
    //! Copy pcl::point cloud .
    /*! Copy points from given pcl point cloud ino new one. \param pcl point cloud \return pcl point cloud*/
    static pcl::PointCloud<pcl::PointXYZI>::Ptr getCloudCopy (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! If first and last point are equal erase one .
    /*! If first and last point are equal erase the first point in polygon. \param pcl point cloud */
    static void ifFirstLastAreEqualEraseOne(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon);
};

    //! Class for polygon triangulation
    /*! Class for represtntation of triangulated polygon. */
class TriangulatedPolygon
{
protected:
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_triangles;  /**< Vector of resulting triangles */

public:
    //! Constructor.
    /*! Parametric constructor of class. \param pcl point cloud */
    TriangulatedPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon);
    //! Get triangle at.
    /*! Return triangle at int. \param int \return pcl point cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getTriangleAt(int i);
    //! Get triangle size.
    /*! Return amount of triangles in vector. \return int */
    int getTrianglesSize ();

private:
    //! Polygon triangulation.
    /*! Triangulate given polygon using ear clipping algorithm. \param pcl point cloud */
    void polygonTriangulation ( pcl::PointCloud<pcl::PointXYZI>::Ptr polygon);
    //! Add new triangle.
    /*! Add new triangle to m_triangles. \param pcl point cloud \param int iterator */
    void addNewTriangle(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon,int iter);
};
#endif // GEOMCALCULATIONS_H_INCLUDED


