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

#include "hull.h"

//CLASS CONVEXHULL*/
ConvexHull::ConvexHull (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    m_cloud = cloud;
}
ConvexHull::~ConvexHull()
{
    m_convexhull.reset();
    m_cloud.reset();
}
//COMPUTE
void ConvexHull::compute()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_convexhull = cloud;
  cloud.reset();

  int cloudsize = m_cloud->points.size();
  if(cloudsize < 3)
  {
    QString a = QString("Could not create convexhull of pointcloud with less than 3 points !");
    QMessageBox::information(0,("WARNING"),a);
    return;
  }

  if(cloudsize == 3)
  {
    m_convexhull = m_cloud;
    m_polygonArea = GeomCalc::computeTriangleArea(m_cloud->points.at(0),m_cloud->points.at(1),m_cloud->points.at(2));
  }
  else if(cloudsize == 4)
  {
    createConvexIfOnlyFourPointsInCloud(m_cloud);
  }
  else
  {
    createConvexHull(m_cloud,m_convexhull);
    m_polygonArea = GeomCalc::computePolygonArea(m_convexhull);

  }
}
void ConvexHull::set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
   m_cloud = cloud;
}

void ConvexHull::createConvexHull (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud)
{
    //Find point with lowest y value, set its z value to lowest z value and set it as first polygon point
    pcl::PointXYZI pointLowest = returnPointLowestYZ(cloud);
    hullCloud->points.push_back(pointLowest);
    //Erase first equal points from source point cloud
    CloudOperations::erasePointFromCloudXY(cloud,pointLowest);
    //Add second point to polygon and erase xy equal points from cloud
    hullCloud->points.push_back(returnSecondPointToPolygon(cloud,pointLowest));
    CloudOperations::erasePointFromCloudXY(cloud,hullCloud->points.at(1));
    //Add next point into polygon until first point is not equal last point and polygon is closed
     for (int k=1; k<100; k++)
     {
        pcl::PointXYZI pointA = hullCloud->points.at(k-1);
        pcl::PointXYZI pointB = hullCloud->points.at(k);
        //Select next point to polygon
        pcl::PointXYZI point = returnNextPointToHull(cloud,pointA,pointB);
        //Add chosen point to polygon
        hullCloud->points.push_back(point);
        //erase equal points from source point cloud
        CloudOperations::erasePointFromCloudXY(cloud,point);
        //Return first point in polygon into source cloud
        if (k==3)
          {cloud->points.push_back(pointLowest);}
        //Condition for stoping the loops, polygon is closed forst, point == last point
        if (k > 3 && point.x == pointLowest.x && point.y == pointLowest.y)
        {break;}
    }
    //erase first point in polygon from source cloud, because its z value is equal to lowest z value in the cloud
    int s = cloud->points.size();
    cloud->points.erase(cloud->points.begin()+s);
}
pcl::PointXYZI ConvexHull::returnSecondPointToPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI firstPoint)
{
    pcl::PointXYZI secondPoint;
    pcl::PointXYZI backVector = firstPoint;
    backVector.x -=1;
    backVector.y -=1;
    float Abigger = 0;
    //iterate over cloud points and select that one with biggest clockwise angle
    for(int m=0; m < cloud->points.size();m++)
    {
        pcl::PointXYZI point = cloud->points.at(m);
        float A = GeomCalc::computeClockwiseAngle(backVector,firstPoint,point);
        if (A > Abigger)
        {
            Abigger = A;
            secondPoint = point;
        }
    }
    // align z coordinate
    secondPoint.z = firstPoint.z;
    return secondPoint;
}
pcl::PointXYZI ConvexHull::returnPointLowestYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointXYZI bodYmin = cloud->points.at(0);
    for (int i=0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI bod =cloud->points.at(i);
        if (bod.y < bodYmin.y)
        {
            bodYmin.y = bod.y;
            bodYmin.x = bod.x;
        }
        if (bod.z < bodYmin.z){bodYmin.z =bod.z;}
    }
    return bodYmin;
}
pcl::PointXYZI ConvexHull::returnNextPointToHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointXYZI pointA, pcl::PointXYZI pointB)
{
    float Abigger = 0;
    pcl::PointXYZI nextPoint;
    //iterate over cloud points and select that one with biigest clockwise angle
    for(int m=0; m < cloud->points.size();m++)
    {
        pcl::PointXYZI point = cloud->points.at(m);
        float A = GeomCalc::computeClockwiseAngle(pointA,pointB,point);
        if (A > Abigger)
        {
            if(GeomCalc::computeDistance2Dxy(pointB,point)>0)
            {
                Abigger = A;
                nextPoint = point;
            }
        }
    }
    // align z coordinate
    nextPoint.z = pointA.z;
    return nextPoint;
}
void ConvexHull::createConvexIfOnlyFourPointsInCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud(new pcl::PointCloud<pcl::PointXYZI>);
     pcl::PointXYZI first = returnPointLowestYZ(cloud);
     pcl::PointXYZI second = returnSecondPointToPolygon(cloud,first);
     pcl::PointXYZI third = returnNextPointToHull(cloud,first,second);

    CloudOperations::erasePointFromCloudXY(cloud,first);
    CloudOperations::erasePointFromCloudXY(cloud,second);
    CloudOperations::erasePointFromCloudXY(cloud,third);

    hullCloud->points.push_back(first);
    hullCloud->points.push_back(second);
    hullCloud->points.push_back(third);

    pcl::PointXYZI last = cloud->points.at(0);
    if (GeomCalc::isPointInTriangle(last,first,second,third) != true)
    {
       hullCloud->points.push_back(last);
    }
    m_convexhull = hullCloud;
    m_polygonArea = GeomCalc::computePolygonArea(hullCloud);
}
//GET
void ConvexHull::getPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    *cloud = *m_convexhull;
}
float ConvexHull::getPolygonArea()
{
    return m_polygonArea;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/* CLASS CONCAVEHULL */
ConcaveHull::ConcaveHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float searchDist)
{
  m_cloud = cloud;
  m_searchingDistance = searchDist;

}
ConcaveHull::~ConcaveHull()
{
    m_concavehull.reset();
}
//COMPUTE
void ConcaveHull::compute()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_concavehull = cloud;
  cloud.reset();

    computeConcaveHull();
    m_polygonArea = GeomCalc::computePolygonArea(m_concavehull);
}
void ConcaveHull::computeConcaveHull()
{
    //Copy cloud and create convex hull
   // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(m_Cloud);
  int sizeOfCloud = m_cloud->points.size();

  ConvexHull *c = new ConvexHull(m_cloud);
  c->compute();
  pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud (new pcl::PointCloud<pcl::PointXYZI>);
  c->getPolygon(hullCloud);
  delete c;

  if(sizeOfCloud <= hullCloud->points.size() || m_cloud->points.size() == 1)
  {
    m_concavehull = hullCloud;
    hullCloud.reset();
    return;
  }
  //Edges breaking with incremental max edge lenght, start at 0.5m
  float searchDist = m_searchingDistance;
  int polygonSize = hullCloud->points.size();
  for (int i=0; i<11; i++)
    //co to je za cislo?
  {
    edgesBreaking(m_cloud,hullCloud,searchDist);
            //If all edges are shorter than searchDist stop the loop
    if(polygonSize == hullCloud->points.size())
      {break;}
    polygonSize = hullCloud->points.size();
            //Increase serching distance
    searchDist +=0.1;
  }
  m_concavehull = hullCloud;
  hullCloud.reset();
}
void ConcaveHull::edgesBreaking(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud,float searchDist)
{
    float n = (float) hullCloud->points.size();
    int a =0;
    do{
        int j = a+1;
        pcl::PointXYZI boda =hullCloud->points.at(a);
        pcl::PointXYZI bodb =hullCloud->points.at(j);
        pcl::PointXYZI bodx;
        float distAB =GeomCalc::computeDistance2Dxy(boda,bodb);
        if (distAB > searchDist)
        {
            bodx = returnEdgeBreakingPoint(cloud,boda,bodb,searchDist);
            if(bodx.z != 0 and bodx.x != 0)
            {
                hullCloud->points.insert(hullCloud->points.begin()+j,bodx);
                CloudOperations::erasePointFromCloudXY(cloud,bodx);
                n++;
                a--;
            }
        }
    a++;
    }while (a < n-1);
}
pcl::PointXYZI ConcaveHull::returnEdgeBreakingPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointXYZI boda, pcl::PointXYZI bodb,float searchDist)
{
    float Amax = 9999;
    pcl::PointXYZI bodx;
    float distAB =GeomCalc::computeDistance2Dxy(boda,bodb);
    for (int g=0; g<cloud->points.size(); g++)
    {
        pcl::PointXYZI bodc =cloud->points.at(g);
        if(GeomCalc::isXYequal(boda,bodc)!=true || GeomCalc::isXYequal(bodb,bodc)!=true)
        {
            if ((GeomCalc::computeDistance2Dxy(boda,bodc) < searchDist and GeomCalc::computeDistance2Dxy(bodb,bodc) < distAB) or
                (GeomCalc::computeDistance2Dxy(bodb,bodc) < searchDist and GeomCalc::computeDistance2Dxy(boda,bodc) < distAB))
            {
                float A =GeomCalc::computeClockwiseAngle(boda,bodc,bodb);
                if (A < Amax)
                {
                    Amax = A;
                    bodx =bodc;
                    bodx.z =boda.z;
                }
            }
        }
    }
    return bodx;
}

//GET
void  ConcaveHull::getPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    *cloud = *m_concavehull;
}
float ConcaveHull::getPolygonArea()
{
    return m_polygonArea;
}
void ConcaveHull::getPolygonSwappedZI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
//     pcl::PointCloud<pcl::PointXYZI>::Ptr c (new pcl::PointCloud<pcl::PointXYZI>);
//    c = m_concavehull->get_Cloud();
//     for(pcl::PointCloud<pcl::PointXYZI>::iterator it = c->begin(); it != c->end(); it++)
//    {
//        float z = it->z;
//        it->z = it->intensity;
//        it->intensity = z;
//    }
//    QString a = QString("%1_swapppedConcavehull").arg(m_name);
//    Cloud *cloud = new Cloud(c, a);
//    return *cloud;
}
float ConcaveHull::getSearchDist()
{
    return m_searchingDistance;
}





