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
#ifndef SKELETON_H_INCLUDED
#define SKELETON_H_INCLUDED

#include <QtGui/QtGui>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
  //! Struct presenting 3x3 matrix.
  /*! Base struct for matrix computing. */
struct matrix3x3{
  float a;  /**< first row, first column */
  float b;  /**< first row, second column */
  float c;  /**< first row, third column */
  float d;  /**< second row, first column */
  float e;  /**< second row, second column */
  float f;  /**< second row, third column */
  float g;  /**< third row, first column */
  float h;  /**< third row, second column */
  float i;  /**< third row, third column */
    //! operator +
    /*! matrix addition */
  inline matrix3x3 operator+(const matrix3x3& r) const {
        matrix3x3 res {a+r.a,b+r.b,c+r.c,d+r.d,e+r.e,f+r.f,g+r.g,h+r.h,i+r.i};
        return res;
    }
    //! operator +=
    /*! matrix addition */
  inline matrix3x3& operator+=(const matrix3x3& r)  {
        a = a + r.a;
        b = b + r.b;
        c = c + r.c;
        d = d + r.d;
        e = e + r.e;
        f = f + r.f;
        g = g + r.g;
        h = h + r.h;
        i = i + r.i;
        return *this;
    }
    //! operator *
    /*! matrix multiplication */
  inline matrix3x3 operator*(const matrix3x3& r) const {
        matrix3x3 res {
    a*r.a + b*r.d + c*r.g,
    a*r.b + b*r.e + c*r.h,
    a*r.c + b*r.f + c*r.i,
    d*r.a + e*r.d + f*r.g,
    d*r.b + e*r.e + f*r.h,
    d*r.c + e*r.f + f*r.i,
    g*r.a + h*r.d + i*r.g,
    g*r.b + h*r.e + i*r.h,
    g*r.c + h*r.f + i*r.i,
          };
        return res;
    }
};
  //! Class for computing and diplsaying skeleton of cloud .
  /*! Class for computing skeleton of given pointcloud. Class uses computing of L1- medial skeleton with improvements.
      Results display as graph connecting lowest and hightest point and the res is connected as minimum spannig tree.  */
class Skeleton
{
public:
    //! Constructor.
    /*! Empty constructor */
  Skeleton();
    //! Constructor.
    /*! Constructor with input pointCloud and radius. */
  Skeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float radius);
    //! Compute average term .
    /*! Compute average position of median based on original points. */
  void averageTerm();
    //! Compute repulsion term .
    /*! Compute repulsion based on previously computed positon of medians */
  void repulsionTerm();
    //! Iteration .
    /*! for each iteration it compute new average and repulsion term  and sigma values of all points */
  void iterate();
    //! Sample input pointCloud.
    /*! Create samples of point for computing medians. */
  void sample();
    //! The first iteration.
    /*! the first iteration and computing of medians. */
  void init();
    //! Minimum spannig tree for resulting points.
    /*! Create minimum spannig tree for resulting point saved in m_sekeleton */
  void graph();
    //! Minimum distance path between lowest and highest point.
    /*! Minimum distance path between lowest and highest point of the resultant pointCloud.*/
  void graph_min_max();
    //! get sample pointCloud.
    /*! \return pointCloud of samples */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_sample();
    //! get average pointCloud.
    /*! \return pointCloud of computed average points */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_average();
    //! get result of skeleton pointCloud.
    /*! \return pointCloud of computed skeletal points */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_skeleton();
    //! get points where lines of skeleton starts.
    /*! \return pointCloud of points where start skeleton lines. */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_start();
    //! get points where lines of skeleton stops.
    /*! \return pointCloud of points where stop skeleton lines. */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_stop();
    //! get movement error for all points in teration.
    /*! \return value of movement error. */
  float get_movement_error();
    //! Compute covariation matrix for given point.
    /*! \param bod point for computing covariation matrix \param w theta of given point \return value of movement error. */
  matrix3x3 cov(pcl::PointXYZI bod, float w);
    //! Compute eigen values of input matrix.
    /*! \param in  input matrix to estimate eigenvalues \return matrix with eigenvalues at a,e,i variables. */
  matrix3x3 jacobi(matrix3x3 in);
    //! Compute sigma value from eigenvalues of matrix.
    /*! \param in input matrix whit computed eigenvalues.  \return value of sigma alignment. */
  float sigma( matrix3x3 in);
    //! Compute skeleton points.
    /*! Select point to be part of skeleton based on sigma value and smoothing of neighbour points */
  void branching();
    //! remove too close points.
    /*! remove point in skeleton, that are too close to each other. */
  void removeTooClosePoints();
    //! Compute cos of angle of three points.
    /*! Compute cos of angle between ab and ac.  */
  float cosAngle(pcl::PointXYZI a, pcl::PointXYZI b, pcl::PointXYZI c);
  void save_skel(QString File);

protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;     /**< Original pointCloud */
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_sample;    /**< PointCloud with samples */
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_skeleton;  /**< Skeleton pointCloud */
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_start;     /**< PointCloud with starting points */
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_stop;      /**< PointCloud with stop points */
  pcl::PointCloud<pcl::PointXYZI>::Ptr repulsion;   /**< PointCloud repulsion force for each point */
	pcl::PointCloud<pcl::PointXYZI>::Ptr average;     /**< PointCloud average median for each point */


	float m_radius;                                   /**< radius of search */
	float m_mju;                                      /**< mju value defaul is 0.35 */
	float movement_error;                             /**< Movement error*/
	std::vector<int> m_nodes_stem;                    /**< indices of point in minimun distance path from lowest to highest point in skeleton*/
};

#endif // SKELETON_H_INCLUDED