#include "geomcalculations.h"

// GEOMETRIC CALCULATIONS
float GeomCalc::computeDistance2Dxy(pcl::PointXYZI boda, pcl::PointXYZI bodb)
{
    float xv =boda.x-bodb.x;
    float yv =boda.y-bodb.y;
    float dist =sqrt(xv*xv+yv*yv);

    return dist;
}
float GeomCalc::computeDistance3D(pcl::PointXYZI boda, pcl::PointXYZI bodb)
{
float xv =boda.x-bodb.x;
float yv =boda.y-bodb.y;
float zv =boda.z-bodb.z;
float dist =sqrt(xv*xv+yv*yv+zv*zv);

return dist;
}
float GeomCalc::computeClockwiseAngle(pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    //vectors
    float xnext = pointB.x - pointC.x;
    float ynext = pointB.y - pointC.y;
    float xback = pointB.x - pointA.x;
    float yback = pointB.y - pointA.y;
    //angle in degrees
    float A = (((atan2(xback*ynext-xnext*yback,xback*xnext+yback*ynext)))*180/3.14159265359);
            if (A > 0){A -= 360;}
            if (A < 0){A = A * (-1);}
            //if (A == 0){A = 180;}
    return A;
}
cloudHighestAndLowestZValue GeomCalc::findHighestAndLowestPoints (pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI)
{
    float zmin = 9999;
    float zmax =-9999;
    for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloudXYZI->begin(); it != cloudXYZI->end(); it++)
       {
           if (it->z < zmin){zmin = it->z;}
           if (it->z > zmax){zmax = it->z;}
           it->intensity = it->z;
       }
    cloudHighestAndLowestZValue HL;
    HL.Highest = zmax;
    HL.Lowest = zmin;
   return HL;
}
pointsWithLongestDist GeomCalc::findPointsWithLongestDistance (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pointsWithLongestDist PT;
    float maxDist = 0;
    for (int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI pointI = cloud->points.at(i);
        for (int j=0; j<cloud->points.size(); j++)
        {
            pcl::PointXYZI pointJ = cloud->points.at(j);
            float Dist = GeomCalc::computeDistance2Dxy(pointI,pointJ);
            if (Dist > maxDist)
            {
                maxDist = Dist;
                PT.pointA = pointI;
                PT.pointB = pointJ;
            }
        }
    }
    return PT;
}
float GeomCalc::computePerpendicularDistanceFromPointC (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    float AB = GeomCalc::computeDistance2Dxy(pointA,pointB);
    float S = GeomCalc::computeTriangleArea(pointA,pointB,pointC);
    return (2*S)/AB;
}
float GeomCalc::computeTrianglePerimeter (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    float perimeter = GeomCalc::computeDistance2Dxy(pointA,pointB);
    perimeter += GeomCalc::computeDistance2Dxy(pointB,pointC);
    perimeter += GeomCalc::computeDistance2Dxy(pointA,pointC);
    return perimeter;
}
float GeomCalc::findLongestPerpendicularDistance (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI &pointA, pcl::PointXYZI &pointB)
{
    float distMax =0;
    for (int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI pointC = cloud->points.at(i);
        float dist = GeomCalc::computePerpendicularDistanceFromPointC(pointA,pointB,pointC);
        if (dist > distMax) {distMax = dist;}
    }
    return distMax;
}
float GeomCalc::computeTriangleArea (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    float AB = GeomCalc::computeDistance2Dxy(pointA,pointB);
    float BC = GeomCalc::computeDistance2Dxy(pointB,pointC);
    float AC = GeomCalc::computeDistance2Dxy(pointA,pointC);
    float s = (AB+BC+AC)/2;
    return sqrt(s*(s-AB)*(s-BC)*(s-AC));
}
float GeomCalc::computePolygonArea (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // for each point
    float sum;
    for(int i = 1; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI A,B;
        A = cloud->points.at(i);
        B = cloud->points.at(i-1);

        sum+= (A.x*B.y - B.x*A.y);
    }
    pcl::PointXYZI A,B;
    A = cloud->points.at(0);
    B = cloud->points.at(cloud->points.size()-1);
    sum+= (A.x*B.y - B.x*A.y);
    return std::fabs(sum/2);
}
bool GeomCalc::isLineIntersection(pcl::PointXYZI a1, pcl::PointXYZI a2,pcl::PointXYZI b1,pcl::PointXYZI b2)
{
    float s1x, s1y, s2x, s2y;
    s1x = a2.x-a1.x;
    s1y = a2.y-a1.y;
    s2x = b2.x-b1.x;
    s2y = b2.y-b1.y;

    float s = (-s1y*(a1.x-b1.x)+s1x*(a1.y-b1.y))/(-s2x*s1y+s1x*s2y);
    float t = (-s2x*(a1.y-b1.y)-s2y*(a1.x-b2.x))/(-s2x*s1y+s1x*s2y);
    if(s>0 && s<1 && t>-1 && t<0)
    {return true;
    }else return false;
}
bool GeomCalc::isXYequal (pcl::PointXYZI pointA, pcl::PointXYZI pointB)
{
    if(pointA.x == pointB.x && pointA.y == pointB.y){
    return true;
    }else return false;
}
bool GeomCalc::isAnyEdgeIntersect(pcl::PointCloud<pcl::PointXYZI>::Ptr listOfEdges, pcl::PointXYZI newEdgePtA, pcl::PointXYZI newEdgePtB)
{
    for (int i = 1; i<listOfEdges->points.size();i++)
    {
        pcl::PointXYZI a = listOfEdges->points.at(i-1);
        pcl::PointXYZI b = listOfEdges->points.at(i);
        if(GeomCalc::isLineIntersection(newEdgePtA,newEdgePtB,a,b)== true)
        {
            return true;
        }
    }
    return false;
}
bool GeomCalc::isAnyPointInTriangle(pcl::PointCloud<pcl::PointXYZI>::Ptr points,pcl::PointXYZI a,pcl::PointXYZI b,pcl::PointXYZI c)
{
    for(int i=0;i<points->points.size();i++)
    {
        pcl::PointXYZI testedPoint = points->points.at(i);
        if (GeomCalc::isPointInTriangle(testedPoint,a,b,c) == true)
        {
            return true;
        }else continue;
    }
    return false;
}
bool GeomCalc::isPointInTriangle(pcl::PointXYZI testedPoint,pcl::PointXYZI a,pcl::PointXYZI b,pcl::PointXYZI c)
{
    if (GeomCalc::isXYequal(testedPoint,a) != true && GeomCalc::isXYequal(testedPoint,b) != true && GeomCalc::isXYequal(testedPoint,c) != true)
    {
        float alfa = GeomCalc::computeClockwiseAngle(a,b,testedPoint);
        float beta = GeomCalc::computeClockwiseAngle(b,c,testedPoint);
        float gama = GeomCalc::computeClockwiseAngle(c,a,testedPoint);
        if(alfa <180 && beta < 180 && gama < 180)
        {
            return true;
        }
    }
    return false;
}

//CLOUD OPERATIONS
void CloudOperations::cloudXYZItoXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ)
{

    cloudXYZ->points.resize(cloudXYZI->points.size());
    for (size_t i = 0; i < cloudXYZI->points.size(); i++)
        {
            cloudXYZ->points[i].x = cloudXYZI->points[i].x;
            cloudXYZ->points[i].y = cloudXYZI->points[i].y;
            cloudXYZ->points[i].z = cloudXYZI->points[i].z;
        }
}
void CloudOperations::transformCloudToPlane (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float direction, float vertical)
{
    //transform angles from degree to rad
    vertical /= (180/3.14159265359);
    direction /= (180/3.14159265359);
    //find lowest point of the tree
    cloudHighestAndLowestZValue HL = GeomCalc::findHighestAndLowestPoints(cloud);
    //cloud transformation
    for (int i =0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI bod = cloud->points.at(0);
        float a = bod.z - HL.Lowest;
        float prepona = a/(sin(vertical));
        float c = prepona*(cos(vertical));
        bod.x = bod.x +(c*sin(direction));
        bod.y = bod.y +(c*cos(direction));
        bod.z = HL.Lowest;
        cloud->points.push_back(bod);
        cloud->points.erase(cloud->points.begin());
    }
}
void CloudOperations::erasePointFromCloudXY (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI bod)
{
                float q = (float) cloud->points.size();
                for (int h=0; h<q; h++)
                {
                    pcl::PointXYZI bodx =cloud->points.at(h);
                    if (bodx.x == bod.x && bodx.y == bod.y)
                    {
                        cloud->points.erase(cloud->points.begin()+h);
                        q--;
                    }
                }
}
pcl::PointCloud<pcl::PointXYZI>::Ptr CloudOperations::getCloudCopy (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr copyCloud (new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < cloud->points.size(); i++)
    {
      pcl::PointXYZI point = cloud->points.at(i);
      copyCloud->points.push_back(point);
    }
    return copyCloud;
}
void CloudOperations::ifFirstLastAreEqualEraseOne(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    pcl::PointXYZI first = polygon->points.at(0);
    pcl::PointXYZI last = polygon->points.at(polygon->points.size()-1);
    if(first.x == last.x && first.y == last.y)
    {
        polygon->points.erase(polygon->points.begin());
    }
}

// TRIANGULATED POLYGON
TriangulatedPolygon::TriangulatedPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    polygonTriangulation(polygon);
}
//GET
pcl::PointCloud<pcl::PointXYZI>::Ptr TriangulatedPolygon::getTriangleAt(int i)
{
    return m_triangles.at(i);
}
int TriangulatedPolygon::getTrianglesSize()
{
    return m_triangles.size();
}
//COMPUTE
void TriangulatedPolygon::polygonTriangulation (pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    CloudOperations::ifFirstLastAreEqualEraseOne(polygon);
    int polsize = polygon->points.size();
    do{
        for(int i=0; i<polygon->points.size()-1;i++)
        {
            // Add new triangle into m_triangles if fulfill the conditions
            addNewTriangle(polygon,i);
        }
        if (polsize == polygon->points.size()) {break;}
            polsize = polygon->points.size();
    }while(polygon->points.size()>3);
    //add last remaining triangle to point cloud vector
    if(polygon->points.size()==3){
        m_triangles.push_back(polygon);
    }
}
void TriangulatedPolygon::addNewTriangle(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon,int iter)
{
    pcl::PointXYZI a;
    if(iter==0){
    a = polygon->points.at(polygon->points.size()-1);
    }else {a = polygon->points.at(iter-1);}
    pcl::PointXYZI b = polygon->points.at(iter);
    pcl::PointXYZI c = polygon->points.at(iter+1);

    if(GeomCalc::computeClockwiseAngle(a,b,c) <= 180 && GeomCalc::isAnyPointInTriangle(polygon,a,b,c)== false)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr triangle (new pcl::PointCloud<pcl::PointXYZI>);
        //push points to triangle cloud and push that cloud to point cloud vector
        triangle->points.push_back(a);
        triangle->points.push_back(b);
        triangle->points.push_back(c);
        m_triangles.push_back(triangle);
        //erase ear central point from polygon
        polygon->points.erase(polygon->points.begin()+iter);
    }
}

























