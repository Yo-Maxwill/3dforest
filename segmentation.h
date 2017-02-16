#ifndef SEGMENTATION_H_INCLUDED
#define SEGMENTATION_H_INCLUDED

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "cloud.h"
#include <QtCore/QObject>


class Segmentation : public QObject
{
  Q_OBJECT

public:
  Segmentation();
  Segmentation( pcl::PointCloud<pcl::PointXYZI>::Ptr vegetation, pcl::PointCloud<pcl::PointXYZI>::Ptr terrain);
  ~Segmentation();
public slots:
  // sets
  void setDistance(float i);

  void setMinimalPoint(int i);
  void setVegetation(Cloud input);
  void setTerrain(Cloud input);
  void setRestCloudName(QString a);
  void setTreePrefix(QString a);


  void vegeSegmentation(float h);
  int euclSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr input,std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters, int min_pt);
  void cluster();
  void stumpSegments();
  void stumps();
  void segmentsClean();
  bool crowns(std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters);
  bool crowns(float distance);
  void restCloud();
  void pointsAdd(float dist);
  void execute();
  float minClusterDistance(pcl::PointCloud<pcl::PointXYZI>::Ptr inputA, pcl::PointCloud<pcl::PointXYZI>::Ptr inputB);
  bool isStump(int i);
  void angleAdd();

  //gets
  void getRestcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output);
  void getTree(int i, pcl::PointCloud<pcl::PointXYZI>::Ptr output);
  int get_treeSize();
  void getData();
  void ready();

signals:
  void started();
  void finished();
  void sendingTree( Cloud *);
  void sendingRest( Cloud *);
  void sendingCentr( Cloud *);
  void hotovo();
  void percentage(int);

private:
  Cloud *m_vegetation;
  Cloud *m_terrain;
  Cloud * m_restCloud;
  Cloud * m_centroids;
  std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > m_segments;
  std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > m_clusters;
  std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > m_stems;
  std::vector< int > m_centrNeighbors;
  std::vector< int > m_pointscentroid;
  std::vector< float > m_centrNeighborsDist;
  std::vector<bool> m_usedCluster;
  std::vector<bool> m_used_rest;
  std::vector<bool> m_finishedStem;
  std::vector<Eigen::Vector3f> m_eigenValues;
  std::vector<int> m_stumps; // vsechny segmenty, ktere maji centroidy  jsou do urcite vysky od terenu
  std::vector<int> m_crowns; // vsechny segmenty, ktere maji centroidy  jsou do urcite vysky od terenu
  int neighborsize;
  int m_minimal_pnt;
  float m_cm;
  int recursive_count;
  QString m_treeName;
  int percent;

};

#endif // SEGMENTATION_H_INCLUDED
