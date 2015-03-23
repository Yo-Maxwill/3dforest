

#ifndef HULL_H_INCLUDED
#define HULL_H_INCLUDED

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "cloud.h"

class Hull : public Cloud
{
    Hull *m_hull;
    Cloud *vexhull;// mracno hranicnich bodu
    Cloud *concavehull;
    float m_areaconvex;               /**< Area of tree projection from top view */
    float m_areaconcave;               /**< Area of tree projection from top view */
    float maxEdgeLenght;
    float vertical;
    float direction;
public:
    Hull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col);
    Hull (Cloud cloud);
    void set_convexhull();
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
};





#endif // HULL_H_INCLUDED
