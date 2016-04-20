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
#include "skeleton.h"

#include <pcl/kdtree/kdtree_flann.h>
//BOOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
//#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/array.hpp>

Skeleton::Skeleton()
{
  m_radius = 0.5;
  m_mju = 0.35;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_cloud = cloud;


}
Skeleton::Skeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_cloud = cloud;
 // m_cloud->points.push_back(pose);
}
void Skeleton::set_Vertex()
{
//  z_max = -1;
//  float z_look_heit = -99999999999999;
//    //pro kazdy bod v skeleton
//  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//  kdtree.setInputCloud (m_cloud);
//  for(int i = 0; i < m_cloud->points.size(); i++)
//  {
//    pcl::PointXYZI x = m_cloud->points.at(i);
//
//    std::stringstream s;
//    s << i;
//    m_vertexNames.push_back(i);
//
//    boost::add_vertex(s.str(), m_graph);
//
//    if( z_look_heit < x.z)
//    {
//      z_max = i;
//      z_look_heit = x.z;
//    }
//    std::vector<int> pointIDv;
//    std::vector<float> pointSDv;
//    int num = 5;
//    float radius = 0.07;
//    // search num nearest
//    if( kdtree.radiusSearch(x,radius,pointIDv,pointSDv) > 3)
//    {
//      for(int j = 1; j < pointIDv.size(); j++)
//      {
//        pcl::PointXYZI bod;
//        bod = m_cloud->points.at(pointIDv.at(j));
//        float w = sqrt(pointSDv.at(j));
//
//        boost::add_edge(i, pointIDv.at(j), w, m_graph);
//      }
//    }
//    else
//    {
//      if( kdtree.nearestKSearch (x,num,pointIDv,pointSDv) > 0)
//      {
//        for(int j = 1; j < pointIDv.size(); j++)
//        {
//          pcl::PointXYZI bod;
//          bod = m_cloud->points.at(pointIDv.at(j));
//          float w = sqrt(pointSDv.at(j));
//pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//  kdtree.setInputCloud (m_cloud);
//  for(int i = 0; i < m_cloud->points.size(); i++)
//  {
//    pcl::PointXYZI x = m_cloud->points.at(i);
//
//    std::stringstream s;
//    s << i;
//    m_vertexNames.push_back(i);
//
//    boost::add_vertex(s.str(), m_graph);
//
//    if( z_look_heit < x.z)
//    {
//      z_max = i;
//      z_look_heit = x.z;
//    }
//    std::vector<int> pointIDv;
//    std::vector<float> pointSDv;
//    int num = 5;
//    float radius = 0.07;
//    // search num nearest
//    if( kdtree.radiusSearch(x,radius,pointIDv,pointSDv) > 3)
//    {
//      for(int j = 1; j < pointIDv.size(); j++)
//      {
//        pcl::PointXYZI bod;
//        bod = m_cloud->points.at(pointIDv.at(j));
//        float w = sqrt(pointSDv.at(j));
//
//        boost::add_edge(i, pointIDv.at(j), w, m_graph);
//      }
//    }
//    else
//    {
//      if( kdtree.nearestKSearch (x,num,pointIDv,pointSDv) > 0)
//      {
//          boost::add_edge(i, pointIDv.at(j), w, m_graph);
//        }
//      }
//    }
//  }
//  set_skeleton();
}
void Skeleton::stemSkeleton()
{
//
//   // Create things for Dijkstra
//  std::vector<Vertex> predecessors(boost::num_vertices(m_graph)); // To store parents
//  std::vector<Weight> distances(boost::num_vertices(m_graph)); // To store distances
//
//  IndexMap indexMap = boost::get(boost::vertex_index, m_graph);
//  PredecessorMap predecessorMap(&predecessors[0], indexMap);
//  DistanceMap distanceMap(&distances[0], indexMap);
//
//  // Compute shortest paths from v0 to all vertices, and store the output in predecessors and distances
//  // boost::dijkstra_shortest_paths(g, v0, boost::predecessor_map(predecessorMap).distance_map(distanceMap));
//  // This is exactly the same as the above line - it is the idea of "named parameters" - you can pass the
//  // prdecessor map and the distance map in any order.
//  boost::dijkstra_shortest_paths(m_graph,m_vertexNames.at(m_cloud->points.size()-1), boost::distance_map(distanceMap).predecessor_map(predecessorMap));

  PathType path;
  Vertex v = m_vertexNames.at(0);
  for(Vertex u = m_predecessorMap[v]; u != v;  v = u, u = m_predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
  {
    std::pair<mygraph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);
    mygraph::edge_descriptor edge = edgePair.first;
    path.push_back( edge );
  }

  int nod2;
  //NameMap nameMap = boost::get(boost::vertex_name, g);
  for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    m_nodes_stem.push_back(source(*pathIterator,m_graph));
    nod2 = target(*pathIterator,m_graph);
    // remove  edge: remove_edge(u, v, g) a pka ji tam vratit s novou hodnotou...!!!
  }
  m_nodes_stem.push_back(nod2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for(int p = 0; p < m_nodes_stem.size();p++)
  {
    cloud->points.push_back(m_cloud->points.at(m_nodes_stem.at(p)));
  }
 // setVertexToZero(m_nodes_stem);
  m_output = cloud;
}
void Skeleton::get_Output(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  *cloud = *m_output;
}
void Skeleton::branchskeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
 // do{
        // Create things for Dijkstra
      std::vector<Vertex> predecessors(boost::num_vertices(m_graph)); // To store parents
      std::vector<Weight> distances(boost::num_vertices(m_graph)); // To store distances

      IndexMap indexMap = boost::get(boost::vertex_index, m_graph);
      PredecessorMap predecessorMap(&predecessors[0], indexMap);
      DistanceMap distanceMap(&distances[0], indexMap);
      boost::dijkstra_shortest_paths(m_graph, m_vertexNames.at(m_cloud->points.size()-1), boost::distance_map(distanceMap).predecessor_map(predecessorMap));
    //for every point compute shortest path
      int max_i = 0;
      float max_dist = 0;
      BGL_FORALL_VERTICES( vv,  m_graph, mygraph )
      {
        int i = 0;
        if(distanceMap[vv] > max_dist)
        {
          max_dist = distanceMap[vv];
          max_i = i;
        }
        i++;
      }

      // save the longest path  as stem points
      PathType path;
      Vertex v = m_vertexNames.at(max_i);
      std::vector<int> branch_points;
      for(Vertex u = predecessorMap[v]; u != v;  v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
      {
        std::pair<mygraph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);
        mygraph::edge_descriptor edge = edgePair.first;
        path.push_back( edge );
      }
      int nod2;
      NameMap nameMap = boost::get(boost::vertex_name, m_graph);
      for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
      {
        branch_points.push_back(source(*pathIterator,m_graph));
        nod2 = target(*pathIterator,m_graph);
      }
      branch_points.push_back(nod2);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
      for(int p = 0; p < branch_points.size();p++)
      {
        cloud_->points.push_back(m_cloud->points.at(branch_points.at(p)));
        if(p < branch_points.size()-1)
        {
          boost::remove_edge(branch_points.at(p),branch_points.at(p+1),m_graph);
          boost::add_edge(branch_points.at(p),branch_points.at(p+1), 0.00001, m_graph);
        }
      }
      QString a = QString ("velikost cloud_: %1 vzdalenost: %2").arg(cloud_->points.size()).arg(max_dist);
      QMessageBox::information(0,(""),a);
      *cloud+= *cloud_;
  //}while (max_dist > 0.5);
}
void Skeleton::branchskeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI point)
{
 int i_sel = 0;
 for(int i = 0; i < m_cloud->points.size(); i++)
 {
   if(m_cloud->points.at(i).x == point.x && m_cloud->points.at(i).y == point.y && m_cloud->points.at(i).z == point.z)
    i_sel=i;
 }
        // Create things for Dijkstra
  std::vector<Vertex> predecessors(boost::num_vertices(m_graph)); // To store parents
  std::vector<Weight> distances(boost::num_vertices(m_graph)); // To store distances

  IndexMap indexMap = boost::get(boost::vertex_index, m_graph);
  PredecessorMap predecessorMap(&predecessors[0], indexMap);
  DistanceMap distanceMap(&distances[0], indexMap);
  boost::dijkstra_shortest_paths(m_graph, m_vertexNames.at(m_cloud->points.size()-1), boost::distance_map(distanceMap).predecessor_map(predecessorMap));
    //for every point compute shortest path
  int max_i = 0;
  float max_dist = 0;

      // save the longest path  s stem points
      PathType path;
      Vertex v = m_vertexNames.at(i_sel);
      std::vector<int> branch_points;
      for(Vertex u = predecessorMap[v]; u != v;  v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
      {
        std::pair<mygraph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);
        mygraph::edge_descriptor edge = edgePair.first;
        path.push_back( edge );
      }
      int nod2;
      NameMap nameMap = boost::get(boost::vertex_name, m_graph);
      for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
      {
        branch_points.push_back(source(*pathIterator,m_graph));
        nod2 = target(*pathIterator,m_graph);
      }
      branch_points.push_back(nod2);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
      for(int p = 0; p < branch_points.size();p++)
      {
        cloud_->points.push_back(m_cloud->points.at(branch_points.at(p)));
      }
      *cloud+= *cloud_;
  //}while (max_dist > 0.5);
}
void Skeleton::branchskeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int indice)
{
      // save the longest path  s stem points
  PathType path;
  Vertex v = m_vertexNames.at(indice);
  std::vector<int> branch_points;
  float dist=0;
//QMessageBox::information(0,(""),("hledani cesty"));
  for(Vertex u = m_predecessorMap[v]; u != v;  v = u, u = m_predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
  {
    std::pair<mygraph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);
    mygraph::edge_descriptor edge = edgePair.first;
    path.push_back( edge );

  }

//QMessageBox::information(0,(""),("iterator"));
  int nod2;
  NameMap nameMap = boost::get(boost::vertex_name, m_graph);
  for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    branch_points.push_back(source(*pathIterator,m_graph));
    nod2 = target(*pathIterator,m_graph);
   dist+= m_distanceMap[target(*pathIterator,m_graph)];
  }
  //dist= m_distanceMap[v];
  std::cout << "vzdalenost: " << dist <<"\n";

//QMessageBox::information(0,(""),("prirazeni do cloudu"));
  branch_points.push_back(nod2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);

  for(int p = 0; p < branch_points.size();p++)
  {
    cloud->points.push_back(m_cloud->points.at(branch_points.at(p)));
  }
  //setVertexToZero(branch_points);
}
void Skeleton::set_skeleton()
{
  indexMap = boost::get(boost::vertex_index, m_graph);
  predecessors.resize(boost::num_vertices(m_graph)); // To store parents
  distances.resize(boost::num_vertices(m_graph)); // To store distances
  PredecessorMap predecessorMap(&predecessors[0], indexMap);
  DistanceMap distanceMap(&distances[0], indexMap);

  boost::dijkstra_shortest_paths(m_graph, m_vertexNames.at(m_cloud->points.size()-1),
                                 boost::distance_map(distanceMap).predecessor_map(predecessorMap));

      m_predecessorMap = predecessorMap;
      m_distanceMap = distanceMap;
///////

}
void Skeleton::setVertexToZero(std::vector<int> nodes)
{
  for(int p = 0; p < nodes.size()-1; p++)
  {
    boost::remove_edge(nodes.at(p),nodes.at(p+1),m_graph);
    boost::add_edge(nodes.at(p),nodes.at(p+1), 0.001, m_graph);
  }
  set_skeleton();
}
