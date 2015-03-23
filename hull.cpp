
#include "hull.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <iostream>
#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>


Hull::Hull (Cloud cloud)
: Cloud(cloud),
vexhull(new Cloud()),
concavehull(new Cloud())
{

}
Hull::Hull (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col)
: Cloud(cloud, name, col),
vexhull(new Cloud()),
concavehull(new Cloud())
{

}

void Hull::set_Hull(Hull hull)
{
    *m_hull = hull;
}
Hull Hull::get_Hull()
{
    return *m_hull;
}
void Hull::set_convexhull()
{
  //nacteni do vekteru
  double ymin = 9999;
  double zmin = 9999;
  double xminy, intens, xx, yy;
  int i = 0;
  int it = 0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
      i++;
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;
        if (ymin > ith->y)
            {
            ymin = ith->y;
            xminy = ith->x;
            intens = ith->intensity;
            it = i;
            }
        if (zmin > ith->z) {zmin = ith->z;}
      cloud->points.push_back(bod);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudb (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointXYZI bod;
      bod.x =xminy;
      bod.y =ymin;
      bod.z =zmin;
      bod.intensity =intens;
      cloudb->points.push_back(bod);
      float n = (float) cloud->points.size();

      for(int i=0; i < n;i++)
        {
            double x,y;
            pcl::PointXYZI ith = cloud->points.at(i);
            x =ith.x;
            y =ith.y;
            if (x==xminy && y==ymin)
            {
              cloud->points.erase(cloud->points.begin()+i);
              n--;
            }
        }

      int k = 0;
     do
     {
            double xback, yback, xnext, ynext, xB, yB, xb, yb;
            double Abigger = 0;
            if (k == 0)
            {xback = -1;
             yback = -1;
             xx =xminy;
             yy =ymin;
             }
            else
            {
             pcl::PointXYZI ith = cloudb->points.at(k-1);
             xB =ith.x;
             yB =ith.y;
             xback =xx - xB;
             yback =yy - yB;
            }
         float n = (float) cloud->points.size();

         for(int m=0; m < n;m++)
        {
                pcl::PointXYZI ith = cloud->points.at(m);
                double xfor, yfor, A;
                xfor = ith.x;
                yfor = ith.y;

            xnext = xfor - xx;
            ynext = yfor - yy;
            double dist =sqrt(xnext*xnext+ynext*ynext);
            A = (((atan2(xback*ynext-xnext*yback,xback*xnext+yback*ynext)))*180/3.14159265359);
            if (A > 0){A -= 360;}
            if (A < 0){A = A * (-1);}
            if (A == 0){A = 180;}
            if (A > Abigger && dist > 0)
            {
                Abigger = A;
                xb = xfor;
                yb = yfor;
            }
        }

        xx =xb;
        yy =yb;

        pcl::PointXYZI bod;
        bod.x =xx;
        bod.y =yy;
        bod.z =zmin;
        bod.intensity =intens;
        cloudb->points.push_back(bod);

       for(int i=0; i < n;i++)  // Prohledava vektor a maze vybrane body
        {
            double x,y;
            pcl::PointXYZI ith = cloud->points.at(i);
            x =ith.x;
            y =ith.y;
            if (x==xx && y==yy)
            {
              cloud->points.erase(cloud->points.begin()+i);
              n--;
            }
        }
        if (k==3) // Vrati prvni bod do prohledavaneho vektoru
            {
                pcl::PointXYZI bod;
                bod.x =xminy;
                bod.y =ymin;
                bod.z =zmin;
                bod.intensity =intens;
                cloud->points.push_back(bod);
            }
        //Podminka
        if (k > 10 && xx == xminy && yy == ymin)
        {
            break;
        }
        k++;
        }while (k<100);

        QString a = QString("%1_vex").arg(m_name);
        QColor col = QColor(255,0,0);
        Cloud *cl = new Cloud(cloudb, a);
        vexhull = cl;
        set_areavex(*cl);
}
void Hull::set_convexhull(Cloud c)
{
  vexhull->set_Cloud(c.get_Cloud());
}
Cloud Hull::get_convexhull()
{
  return *vexhull;
}
void Hull::set_concavehull(Cloud c)
{
  concavehull->set_Cloud(c.get_Cloud());
}
Cloud Hull::get_concavehull()
{
  return *concavehull;
}
void Hull::set_concavehull(float maxEdgeLenght = 150)
{
  //nacteni do vekteru
  double ymin = 9999;
  double zmin = 9999;
  double xminy, intens, xx, yy;
  int i = 0;
  int it = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtrans (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;
        if (ymin > ith->y)
            {
            ymin = ith->y;
            xminy = ith->x;
            intens = ith->intensity;
            it = i;
            }
        if (zmin > ith->z) {zmin = ith->z;}
      cloud->points.push_back(bod);
      i++;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudb (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointXYZI bod;
      bod.x =xminy;
      bod.y =ymin;
      bod.z =zmin;
      bod.intensity =intens;
      cloudb->points.push_back(bod);
      float n = (float) cloud->points.size();

      for(int i=0; i < n;i++)
        {
            double x,y;
            pcl::PointXYZI ith = cloud->points.at(i);
            x =ith.x;
            y =ith.y;
            if (x==xminy && y==ymin)
            {
              cloud->points.erase(cloud->points.begin()+i);
              n--;
            }
        }

    int k = 0;
     do
     {
            double xback, yback, xnext, ynext, xB, yB, xb, yb;
            double Abigger = 0;
            if (k == 0)
            {xback = -1;
             yback = -1;
             xx =xminy;
             yy =ymin;
             }
            else
            {
             pcl::PointXYZI ith = cloudb->points.at(k-1);
             xB =ith.x;
             yB =ith.y;
             xback =xB-xx;
             yback =yB-yy;
            }
         float n = (float) cloud->points.size();

         for(int m=0; m < n;m++)
        {
                pcl::PointXYZI ith = cloud->points.at(m);
                double xfor, yfor, A;
                xfor = ith.x;
                yfor = ith.y;

            xnext = xfor - xx;
            ynext = yfor - yy;
            double dist =sqrt(xnext*xnext+ynext*ynext);
            if (dist > 0 && dist < maxEdgeLenght)
            {
                A = (((atan2(xback*ynext-xnext*yback,xback*xnext+yback*ynext)))*180/3.14159265359);
                if (A > 0){A -= 360;}
                if (A < 0){A = A * (-1);}
                if (A > Abigger && A < 270)
                    {
                    Abigger = A;
                    xb = xfor;
                    yb = yfor;
                    }
            }
        }

           //Podminka
        if (k > 10 && xx == xminy && yy == ymin)
        {
            break;
        }

        xx =xb;
        yy =yb;

        pcl::PointXYZI bod;
        bod.x =xx;
        bod.y =yy;
        bod.z =zmin;
        bod.intensity =intens;
        cloudb->points.push_back(bod);

       for(int i=0; i < n;i++)  // Prohledava vektor a maze vybrane body
        {
            double x,y;
            pcl::PointXYZI ith = cloud->points.at(i);
            x =ith.x;
            y =ith.y;
            if (x==xx && y==yy)
            {
              cloud->points.erase(cloud->points.begin()+i);
              n--;
            }
        }
        if (k==10) // Vrati prvni bod do prohledavaneho vektoru
            {
                pcl::PointXYZI bod;
                bod.x =xminy;
                bod.y =ymin;
                bod.z =zmin;
                bod.intensity =intens;
                cloud->points.push_back(bod);
            }
        k++;
        }while (k<500);

        QString a = QString("%1_vex").arg(m_name);
        QColor col = QColor(255,0,0);
        Cloud *cl = new Cloud(cloudb, a);
        concavehull = cl;
        set_areacave(*cl);
}
int Hull::set_concaveZkracovanim(float maxEdgeLenght = 150)
{
    int errors = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;
      cloud->points.push_back(bod);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudb (new pcl::PointCloud<pcl::PointXYZI>);
  returnConvexhull(cloud, cloudb);
    float n = (float) cloudb->points.size();

  for (int a=0; a<n-1; a++)
  {
      int i =a;
      int j = a+1;
      float ax, ay, bx, by, xx, yy;
      pcl::PointXYZI boda =cloudb->points.at(i);
      float z =boda.z;
      pcl::PointXYZI bodb =cloudb->points.at(j);
      pcl::PointXYZI bodx;
      bodx.x =0;
      bodx.y =0;

      float distAB =returnDistance(boda,bodb);
      float Amax = 9999;
      bool BC = false;

        if (distAB > maxEdgeLenght)
        {
                for (int g=0; g<cloud->points.size(); g++)
                {
                    pcl::PointXYZI bodc =cloud->points.at(g);
                    float distAC =returnDistance(boda,bodc);
                    float distBC =returnDistance(bodb,bodc);
                    if (distAC < maxEdgeLenght && distBC < distAB)
                    {
                        float A =return_clockwiseAngle(boda,bodc,bodb);
                        if (A < Amax)
                        {
                            Amax = A;
                            bodx =bodc;
                            bodx.z =z;
                        }
                    }
                }

            if(bodx.z != z)
            {
               float Amax =9999;
               for (int g=0; g< cloud->points.size(); g++)
                {
                    pcl::PointXYZI bodc =cloud->points.at(g);
                    float distBC =returnDistance(bodb,bodc);
                    float distAC =returnDistance(boda,bodc);
                    if (distBC < maxEdgeLenght && distAC < distAB)
                    {
                        float A =return_clockwiseAngle(boda,bodc,bodb);
                        if (A < Amax)
                        {
                            Amax = A;
                            bodx =bodc;
                            bodx.z =z;
                            BC = true;
                        }
                    }
                }
            }
                if(bodx.z ==z)
                {
                    cloudb->points.insert(cloudb->points.begin()+j,bodx);
                    n++;

                    float q = (float) cloud->points.size();
                    for (int h=0; h<n; h++)
                    {
                        pcl::PointXYZI bod =cloud->points.at(h);
                        if (bod.x == bodx.x && bod.y == bodx.y)
                        {
                            cloud->points.erase(cloud->points.begin()+h);
                            q--;
                        }
                    }
                }

            if(bodx.z != z){errors++;}
            if(BC == true) {a--;}
        }
  }

    QString a = QString("%1_vex").arg(m_name);
    QColor col = QColor(255,0,0);
    Cloud *cl = new Cloud(cloudb, a);
    concavehull = cl;
    concavehull->set_Psize(3);
    set_areacave(*cl);

    return errors;
}

float Hull::returnDistance(pcl::PointXYZI boda, pcl::PointXYZI bodb)
{
float xv =boda.x-bodb.x;
float yv =boda.y-bodb.y;
float dist =sqrt(xv*xv+yv*yv);

return dist;
}

float Hull::return_clockwiseAngle(pcl::PointXYZI boda, pcl::PointXYZI bodc, pcl::PointXYZI bodb)
{
    float xback =boda.x-bodc.x;
    float yback =boda.y-bodc.y;
    float xnext =bodb.x-bodc.x;
    float ynext =bodb.y-bodc.y;

    float A = (((atan2(xback*ynext-xnext*yback,xback*xnext+yback*ynext)))*180/3.14159265359);
            if (A > 0){A -= 360;}
            if (A < 0){A = A * (-1);}

    return A;
}

void Hull::set_areavex(Cloud c)
{
  // for each point
  float sum;
  for(int i = 1; i < c.get_Cloud()->points.size(); i++)
  {
    pcl::PointXYZI A,B;
    A = c.get_Cloud()->points.at(i);
    B = c.get_Cloud()->points.at(i-1);

    sum+= (A.x*B.y - B.x*A.y);
  }
  pcl::PointXYZI A,B;
  A = c.get_Cloud()->points.at(0);
  B = c.get_Cloud()->points.at(c.get_Cloud()->points.size()-1);
  sum+= (A.x*B.y - B.x*A.y);
  m_areaconvex = std::fabs(sum/2);
}
void Hull::set_areacave(Cloud c)
{
  // for each point
  float sum;
  for(int i = 1; i < c.get_Cloud()->points.size(); i++)
  {
    pcl::PointXYZI A,B;
    A = c.get_Cloud()->points.at(i);
    B = c.get_Cloud()->points.at(i-1);

    sum+= (A.x*B.y - B.x*A.y);
  }
  pcl::PointXYZI A,B;
  A = c.get_Cloud()->points.at(0);
  B = c.get_Cloud()->points.at(c.get_Cloud()->points.size()-1);
  sum+= (A.x*B.y - B.x*A.y);
  m_areaconcave = std::fabs(sum/2);
}
float Hull::get_areaconvex()
{
  return m_areaconvex;
}
float Hull::get_areaconcave()
{
  return m_areaconcave;
}
void Hull::returnConvexhull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr cloudb)
{
    float xmin =0;
    float ymin =99999;
    float zmin =99999;
    for (int i=0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI bod =cloud->points.at(i);
        if (bod.y < ymin)
        {
            xmin =bod.x;
            ymin =bod.y;
        }
        if (bod.z < zmin)
        {
            zmin =bod.z;
        }
    }
    pcl::PointXYZI bod;
      bod.x =xmin;
      bod.y =ymin;
      bod.z =zmin;
      bod.intensity =100;
      cloudb->points.push_back(bod);
      float n = (float) cloud->points.size();

      for(int i=0; i < n;i++)
        {
            float x,y;
            pcl::PointXYZI ith = cloud->points.at(i);
            x =ith.x;
            y =ith.y;
            if (x==xmin && y==ymin)
            {
              cloud->points.erase(cloud->points.begin()+i);
              n--;
            }
        }

      int k = 0;
     do
     {
            float xback, yback, xnext, ynext, xB, yB, xb, yb, xx, yy;
            float Abigger = 0;
            if (k == 0)
            {xback = -1;
             yback = -1;
             xx =xmin;
             yy =ymin;
             }
            else
            {
             pcl::PointXYZI ith = cloudb->points.at(k-1);
             xB =ith.x;
             yB =ith.y;
             xback =xx - xB;
             yback =yy - yB;
            }
         float n = (float) cloud->points.size();

         for(int m=0; m < n;m++)
        {
                pcl::PointXYZI ith = cloud->points.at(m);
                double xfor, yfor, A;
                xfor = ith.x;
                yfor = ith.y;

            xnext = xfor - xx;
            ynext = yfor - yy;
            double dist =sqrt(xnext*xnext+ynext*ynext);
            A = (((atan2(xback*ynext-xnext*yback,xback*xnext+yback*ynext)))*180/3.14159265359);
            if (A > 0){A -= 360;}
            if (A < 0){A = A * (-1);}
            if (A == 0){A = 180;}
            if (A > Abigger && dist > 0)
            {
                Abigger = A;
                xb = xfor;
                yb = yfor;
            }
        }

        xx =xb;
        yy =yb;

        pcl::PointXYZI bod;
        bod.x =xx;
        bod.y =yy;
        bod.z =zmin;
        bod.intensity =100;
        cloudb->points.push_back(bod);

       for(int i=0; i < n;i++)  // Prohledava vektor a maze vybrane body
        {
            double x,y;
            pcl::PointXYZI ith = cloud->points.at(i);
            x =ith.x;
            y =ith.y;
            if (x==xx && y==yy)
            {
              cloud->points.erase(cloud->points.begin()+i);
              n--;
            }
        }
        if (k==3) // Vrati prvni bod do prohledavaneho vektoru
            {
                pcl::PointXYZI bod;
                bod.x =xmin;
                bod.y =ymin;
                bod.z =zmin;
                bod.intensity =100;
                cloud->points.push_back(bod);
            }
        //Podminka
        if (k > 10 && xx == xmin && yy == ymin)
        {
            break;
        }
        k++;
        }while (k<100);
}





