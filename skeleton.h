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
#ifndef SKELETON_H_INCLUDED
#define SKELETON_H_INCLUDED

#include <QtGui/QtGui>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

//BOOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
//#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/visitors.hpp>

//  //! Struct presenting 3x3 matrix.
//  /*! Base struct for matrix computing. */
//struct matrix3x3{
//  float a;  /**< first row, first column */
//  float b;  /**< first row, second column */
//  float c;  /**< first row, third column */
//  float d;  /**< second row, first column */
//  float e;  /**< second row, second column */
//  float f;  /**< second row, third column */
//  float g;  /**< third row, first column */
//  float h;  /**< third row, second column */
//  float i;  /**< third row, third column */
//    //! operator +
//    /*! matrix addition */
//  inline matrix3x3 operator+(const matrix3x3& r) const {
//        matrix3x3 res {a+r.a,b+r.b,c+r.c,d+r.d,e+r.e,f+r.f,g+r.g,h+r.h,i+r.i};
//        return res;
//    }
//    //! operator +=
//    /*! matrix addition */
//  inline matrix3x3& operator+=(const matrix3x3& r)  {
//        a = a + r.a;
//        b = b + r.b;
//        c = c + r.c;
//        d = d + r.d;
//        e = e + r.e;
//        f = f + r.f;
//        g = g + r.g;
//        h = h + r.h;
//        i = i + r.i;
//        return *this;
//    }
//    //! operator *
//    /*! matrix multiplication */
//  inline matrix3x3 operator*(const matrix3x3& r) const {
//        matrix3x3 res {
//    a*r.a + b*r.d + c*r.g,
//    a*r.b + b*r.e + c*r.h,
//    a*r.c + b*r.f + c*r.i,
//    d*r.a + e*r.d + f*r.g,
//    d*r.b + e*r.e + f*r.h,
//    d*r.c + e*r.f + f*r.i,
//    g*r.a + h*r.d + i*r.g,
//    g*r.b + h*r.e + i*r.h,
//    g*r.c + h*r.f + i*r.i,
//          };
//        return res;
//    }
//};


typedef float Weight;
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::property<boost::vertex_name_t, std::string> NameProperty;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::undirectedS, NameProperty, EdgeWeightProperty > mygraph;

typedef boost::property_map< mygraph, boost::vertex_index_t>::type VertexIndexMap;
typedef boost::graph_traits <mygraph >::edge_descriptor Edges;
typedef boost::graph_traits <mygraph >::vertex_descriptor Vertex;
typedef boost::property_map < mygraph, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map < mygraph, boost::vertex_name_t >::type NameMap;
typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;
typedef std::vector<mygraph::edge_descriptor> PathType;

  //! Class for computing and diplsaying skeleton of cloud .
  /*! Class for computing skeleton of given pointcloud. Class uses computing of L1- medial skeleton with improvements.
      Results display as graph connecting lowest and hightest point and the res is connected as minimum spannig tree.  */
class Skeleton
{
protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;     /**< Original pointCloud */
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_output;     /**< Original pointCloud */
  mygraph m_graph;                                  /**< graph structure */
  std::vector<Vertex> m_vertexNames;
  int z_max;

	float m_radius;                                   /**< radius of search */
	float m_mju;                                      /**< mju value defaul is 0.35 */
	float movement_error;                             /**< Movement error*/
	std::vector<int> m_nodes_stem;                    /**< indices of point in minimun distance path from lowest to highest point in skeleton*/
	std::vector<std::pair<int, float> > m_dist;                    /**< indices of point in minimun distance path from lowest to highest point in skeleton*/
	  //////tato cast by se asi nemusela opakovat        // Create things for Dijkstra
  std::vector<Vertex> predecessors; // To store parents
  std::vector<Weight> distances; // To store distances

  IndexMap indexMap;
  PredecessorMap m_predecessorMap;
  DistanceMap m_distanceMap;



public:
    //! Constructor.
    /*! Empty constructor */
  Skeleton();
  //Skeleton(Tree strom);
    //! Constructor.
    /*! Constructor with input pointCloud and radius. */
  Skeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

  void stemSkeleton();
  void branchskeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  void branchskeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI point);
  void branchskeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int indice);
  void get_Output(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  void set_Vertex();
  void set_skeleton();
  void setVertexToZero(std::vector<int> nodes);
    //! Compute average term .
    /*! Compute average position of median based on original points. */
//  void averageTerm();
//    //! Compute repulsion term .
//    /*! Compute repulsion based on previously computed positon of medians */
//  void repulsionTerm();
//    //! Iteration .
//    /*! for each iteration it compute new average and repulsion term  and sigma values of all points */
//  void iterate();
//    //! Sample input pointCloud.
//    /*! Create samples of point for computing medians. */
//  void sample();
//    //! The first iteration.
//    /*! the first iteration and computing of medians. */
//  void init();
//    //! Minimum spannig tree for resulting points.
//    /*! Create minimum spannig tree for resulting point saved in m_sekeleton */
//  void graph(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI pose, pcl::PointCloud<pcl::PointXYZI>::Ptr output);
//    //! Minimum distance path between lowest and highest point.
//    /*! Minimum distance path between lowest and highest point of the resultant pointCloud.*/
//  void graph_min_max(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI pose, pcl::PointCloud<pcl::PointXYZI>::Ptr output);
//    //! get sample pointCloud.
//    /*! \return pointCloud of samples */
//  pcl::PointCloud<pcl::PointXYZI>::Ptr get_sample();
//    //! get average pointCloud.
//    /*! \return pointCloud of computed average points */
//  pcl::PointCloud<pcl::PointXYZI>::Ptr get_average();
//    //! get result of skeleton pointCloud.
//    /*! \return pointCloud of computed skeletal points */
//  pcl::PointCloud<pcl::PointXYZI>::Ptr get_skeleton();
//    //! get points where lines of skeleton starts.
//    /*! \return pointCloud of points where start skeleton lines. */
//  pcl::PointCloud<pcl::PointXYZI>::Ptr get_start();
//    //! get points where lines of skeleton stops.
//    /*! \return pointCloud of points where stop skeleton lines. */
//  pcl::PointCloud<pcl::PointXYZI>::Ptr get_stop();
//    //! get movement error for all points in teration.
//    /*! \return value of movement error. */
//  float get_movement_error();
//    //! Compute covariation matrix for given point.
//    /*! \param bod point for computing covariation matrix \param w theta of given point \return value of movement error. */
////  matrix3x3 cov(pcl::PointXYZI bod, float w);
////    //! Compute eigen values of input matrix.
////    /*! \param in  input matrix to estimate eigenvalues \return matrix with eigenvalues at a,e,i variables. */
////  matrix3x3 jacobi(matrix3x3 in);
////    //! Compute sigma value from eigenvalues of matrix.
////    /*! \param in input matrix whit computed eigenvalues.  \return value of sigma alignment. */
////  float sigma( matrix3x3 in);
//    //! Compute skeleton points.
//    /*! Select point to be part of skeleton based on sigma value and smoothing of neighbour points */
//  void branching();

//    //! remove too close points.
//    /*! remove point in skeleton, that are too close to each other. */
//  void removeTooClosePoints();
//    //! Compute cos of angle of three points.
//    /*! Compute cos of angle between ab and ac.  */
//  float cosAngle(pcl::PointXYZI a, pcl::PointXYZI b, pcl::PointXYZI c);
//  void save_skel(QString File);


};

#endif // SKELETON_H_INCLUDED
