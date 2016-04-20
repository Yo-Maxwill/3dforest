#ifndef TERRAIN_H_INCLUDED
#define TERRAIN_H_INCLUDED

#include "cloud.h"
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <QtCore/QObject>


class OctreeTerrain : public QObject
{
  Q_OBJECT

public:
  OctreeTerrain();
  OctreeTerrain( Cloud input, float resolution);
  ~OctreeTerrain();

public slots:
  //sets
  void setResolution(float res);
  void setBaseCloud(Cloud input);
  void setVegetationName(QString name);
  void setTerrainName(QString name);
  void execute();
  void octree(float res, pcl::PointCloud<pcl::PointXYZI>::Ptr input,pcl::PointCloud<pcl::PointXYZI>::Ptr output_ground, pcl::PointCloud<pcl::PointXYZI>::Ptr output_vege);
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingTerrain( Cloud *);
  void sendingVegetation( Cloud *);

private:
  float m_resolution;
  Cloud *m_baseCloud;
  Cloud *m_terrain;
  Cloud *m_vegetation;
};

class VoxelTerrain : public QObject
{
  Q_OBJECT
public:
    VoxelTerrain();
    ~VoxelTerrain();
public slots:
  void setResolution(float res);
  void setBaseCloud(Cloud input);
  void setVegetationName(QString name);
  void setTerrainName(QString name);
  void execute();
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingTerrain( Cloud *);
  void sendingVegetation( Cloud *);

private:
  float m_resolution;
  Cloud *m_baseCloud;
  Cloud *m_terrain;
  Cloud *m_vegetation;

};

class IDW : public QObject
{
   Q_OBJECT
public:
    IDW();
    ~IDW();
public slots:
  void setResolution(float res);
  void setPointNumber(float num);
  void setBaseCloud(Cloud input);
  void setOutputName(QString name);

  void execute();
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingoutput( Cloud *);


private:
  float m_resolution;
  int m_pointsnum;
  Cloud *m_baseCloud;
  Cloud *m_output;
};

class StatOutlierRemoval : public QObject
{
  Q_OBJECT
public:
    StatOutlierRemoval();
    ~StatOutlierRemoval();

public slots:
  void setMeanDistance(float dist);
  void setNeighborhood(int num);
  void setBaseCloud(Cloud input);
  void setOutputName(QString name);

  void execute();
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingoutput( Cloud *);

private:
  float m_mDist;
  int m_neighbors;
  Cloud *m_baseCloud;
  Cloud *m_output;
};

class RadiusOutlierRemoval : public QObject
{
  Q_OBJECT
public:
    RadiusOutlierRemoval();
    ~RadiusOutlierRemoval();
public slots:
  void setRadius(float radius);
  void setNeighborhood(int num);
  void setBaseCloud(Cloud input);
  void setOutputName(QString name);

  void execute();
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingoutput( Cloud *);


private:
  float m_radius;
  int m_neighbors;
  Cloud *m_baseCloud;
  Cloud *m_output;
};

#endif // TERRAIN_H_INCLUDED
