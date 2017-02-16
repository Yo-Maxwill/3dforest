#include "segmentation.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <QtCore/QTime>

Segmentation::Segmentation()
{
  m_vegetation = new Cloud();
  m_terrain = new Cloud();
  m_centroids = new Cloud();
  m_restCloud = new Cloud();

  recursive_count=1;
  m_cm=0.05;
  neighborsize = 20;
  m_minimal_pnt = 10;
  m_treeName = "ID";
  emit started();
  percent=0;
  m_centroids->set_name("centroids");

}
Segmentation::Segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr vegetation, pcl::PointCloud<pcl::PointXYZI>::Ptr terrain)
{
  m_vegetation = new Cloud();
  m_vegetation->set_Cloud(vegetation);
  m_terrain = new Cloud();
  m_terrain->set_Cloud(terrain);
  m_centroids = new Cloud();
  m_restCloud = new Cloud();
  m_pointscentroid.resize(vegetation->points.size());
  std::fill(m_pointscentroid.begin(), m_pointscentroid.end(), -1);

  recursive_count=1;
  percent=0;
  m_cm=0.05;
  neighborsize = 20;
  m_minimal_pnt = 10;
  m_treeName = "ID";
  m_centroids->set_name("centroids");
  emit started();
}
Segmentation::~Segmentation()
{
  delete m_vegetation;
  delete m_terrain;
  delete m_restCloud;
  delete m_centroids;

  m_segments.clear();
  m_clusters.clear();
  m_stems.clear();

  m_usedCluster.clear();
  m_stumps.clear();
  m_crowns.clear();

}
void Segmentation::ready()
{
  emit started();
}
void Segmentation::setDistance(float i)
{
  m_cm = i;
}
void Segmentation::setRestCloudName(QString a)
{
  m_restCloud->set_name(a);
}
void Segmentation::setMinimalPoint(int i)
{
  m_minimal_pnt = i;
}
void Segmentation::setVegetation(Cloud input)
{
  *m_vegetation= input;
  m_pointscentroid.resize(m_vegetation->get_Cloud()->points.size());
  std::fill(m_pointscentroid.begin(), m_pointscentroid.end(), -1);
}
void Segmentation::setTerrain(Cloud input)
{
  *m_terrain= input;
}
void Segmentation:: setTreePrefix(QString a)
{
  m_treeName = a;
}

void Segmentation:: vegeSegmentation(float h)
{

QTime time;
time.start();

  pcl::PointXYZI minp, maxp; // body ohranicujici vegetaci
  pcl::getMinMax3D(*m_vegetation->get_Cloud(),minp, maxp);

  float cut = minp.z;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> os (h);
  os.setInputCloud(m_vegetation->get_Cloud());
  os.addPointsFromInputCloud();
  // pokud neni dosazeno vysky vegetace

  while(cut <= maxp.z)
  {
    std::vector <int> pID;
    std::vector< float > dist;
    Eigen::Vector3f minb (minp.x,minp.y, cut);
    Eigen::Vector3f maxb (maxp.x,maxp.y, cut+h);
    pcl::PointCloud<pcl::PointXYZI>::Ptr segment (new pcl::PointCloud<pcl::PointXYZI>);

    if(os.boxSearch(minb,maxb,pID)> 0)
    {
      for(int j=0; j < pID.size(); j++)
      {
        pcl::PointXYZI bod = m_vegetation->get_Cloud()->points.at(pID.at(j));
        segment->points.push_back(bod);
      }
      segment->width = segment->points.size ();
      segment->height = 1;
      segment->is_dense = true;
      m_segments.push_back(segment);
    }
    cut+=h;
  }

  for(int i=0; i < m_segments.size(); i++)
  {
    m_segments.at(i)->width = m_segments.at(i)->points.size ();
    m_segments.at(i)->height = 1;
    m_segments.at(i)->is_dense = true;
  }
  int difference = time.elapsed();
qWarning()<< "vegeSegmentation() stop time: " << difference/1000 << "s";
}
void Segmentation:: execute()
{
  QTime time;
  time.start();

  vegeSegmentation(m_cm);
  emit percentage( percent+=3);

  cluster();
  emit percentage( percent+=2);


  stumpSegments();
  emit percentage( percent+=3);

  stumps();
  emit percentage( percent+=5);


  //angleAdd();

  segmentsClean();
  emit percentage( percent+=7);

  crowns(m_stems);
  emit percentage( percent+=2);
int di = time.elapsed();
qWarning()<< "crown() stop time: " << di/1000 << "s" ;

  restCloud();
  emit percentage( 93);

  for(int u=5; u < 10; u++)
    pointsAdd(m_cm+(m_cm*u/10));
  pointsAdd(m_cm*2.5);
emit percentage(99);
    int difference = time.elapsed()/1000;
qWarning()<< "execution() stop time: " << difference << "s";

emit finished();
}
void Segmentation::cluster()
{
QTime time;
time.start();
  pcl::PointXYZI minp, maxp; // body ohranicujici vegetaci
  pcl::getMinMax3D(*m_terrain->get_Cloud(),minp, maxp);
  pcl::PointXYZI minpv, maxpv; // body ohranicujici vegetaci
  pcl::getMinMax3D(*m_vegetation->get_Cloud(),minpv, maxpv);
  float h= maxpv.z/m_cm;//(maxp.z + 1.3 - minpv.z)/m_cm;

// rozdeleni segmentu na clustery
  for(int q=0; q < m_segments.size(); q++)
  {
    if(q > h)
      break;
    int a = euclSegmentation(m_segments.at(q), m_clusters, m_minimal_pnt);
  }

  //pro kazdy segm spocitat centroid a ulozit je do společného mracna, eigen hodnoty
  pcl::PointCloud<pcl::PointXYZI>::Ptr centroidCloud (new pcl::PointCloud<pcl::PointXYZI>);// mracno s centroidy vsech clusterů
  for(int q = 0; q < m_clusters.size(); q++)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(	*m_clusters.at(q),	centroid );
    pcl::PointXYZI bod;
    bod.x =centroid.x();
    bod.y =centroid.y();
    bod.z =centroid.z();

    //eigenvalues

    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(m_clusters.at(q));

    Eigen::Matrix3f pcaEVects = pca.getEigenVectors ();
    Eigen::Vector4f pcaMean = pca.getMean();
    Eigen::Vector3f pcaEValues = pca.getEigenValues();


   // std::cout <<"eigen vector pro m_cluster " << q << " : =(" << pcaEVects(0,0)<< ", "<< pcaEVects(1,0)<< ", "<< pcaEVects(2,0)<< ") "<< std::endl;


    //float angle = std::acos (pcaEVects(2,0)/std::sqrt(pcaEVects(1,0)*pcaEVects(1,0)+pcaEVects(2,0)*pcaEVects(2,0)+ pcaEVects(0,0)*pcaEVects(0,0))) * 180.0 / M_PI;
    float SFFIx= (pcaEValues(0)- pcaEValues(1))/(pcaEValues(0)- pcaEValues(2));
    float SFFIy= pcaEValues(2)/pcaEValues(0);

    if(SFFIx >=0 && SFFIx <0.35 && SFFIy >=0 && SFFIy <0.2 )
      bod.intensity =1;
    else if(SFFIx >=0 && SFFIx <0.35 && SFFIy >=0.2 && SFFIy <0.5 )
      bod.intensity =2;
    else if(SFFIx >=0 && SFFIx <0.35 && SFFIy >=0.5 && SFFIy <0.8)
      bod.intensity =3;
    else if(SFFIx >=0.35 && SFFIx <0.65 && SFFIy >=0.5 && SFFIy <0.8 )
      bod.intensity =4;
    else if(SFFIx >=0.35 && SFFIx <0.65 && SFFIy >=0.2 && SFFIy <0.5 )
      bod.intensity =5;
    else if(SFFIx >=0.35 && SFFIx <0.65 && SFFIy >=0.5 && SFFIy <0.8 )
      bod.intensity =6;
    else if(SFFIx >=0.65 && SFFIx <1 && SFFIy >=0 && SFFIy <0.2 )
      bod.intensity =7;
    else if(SFFIx >=0.65 && SFFIx <1 && SFFIy >=0.2 && SFFIy <0.5 )
      bod.intensity =8;
    else if(SFFIx >=0.65 && SFFIx <1 && SFFIy >=0.5 && SFFIy <0.8 )
      bod.intensity =9;
    else
      bod.intensity =0;


    centroidCloud->points.push_back(bod);
    m_usedCluster.push_back(false);
  }
  m_centroids->set_Cloud(centroidCloud);
  centroidCloud.reset();

// spocitat sousedni centroidy
  pcl::KdTreeFLANN<pcl::PointXYZI> kdt;
  kdt.setInputCloud (m_centroids->get_Cloud());
  for(int k=0; k <m_centroids->get_Cloud()->points.size(); k++)
  {
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    pcl::PointXYZI u = m_centroids->get_Cloud()->points.at(k);
    if(kdt.nearestKSearch(u,neighborsize+1,pointIDv,pointSDv) > neighborsize)
    {
      for(int h =1; h < neighborsize+1; h++)
      {
        m_centrNeighbors.push_back(pointIDv.at(h));
      }
    }
  }
int difference = time.elapsed();
qWarning()<< "cluster stop time: " << difference/1000 << " s";
}
int Segmentation::euclSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters, int min_pt )
{
  if(input->points.size()==0)
    return -1;
  pcl::search::Octree<pcl::PointXYZI>::Ptr tree (new pcl::search::Octree<pcl::PointXYZI>(m_cm));
  tree->setInputCloud (input);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

  ec.setClusterTolerance (m_cm); // 5cm
  ec.setMinClusterSize (min_pt );
  ec.setMaxClusterSize (25000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input);
  ec.extract (cluster_indices);


  //m_cluster_indices =  new (std::vector<pcl::PointIndices> (cluster_indices));

  //m_clusters.resize(cluster_indices.size());

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (input->points[*pit]);
      m_pointscentroid[*pit] = clusters.size();
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    clusters[j]->width = cloud_cluster->points.size ();
    clusters[j]->height = 1;
    clusters[j]->is_dense = true;
    j++;
  }
  //clusters = clu;
  return j;
}
void Segmentation:: stumpSegments()
{

  QTime time;
  time.start();
  float dist = 1; // distance from terrain
  // vybrat jen ty clustery, ktere jsou do vysky dist m od terenu
  pcl::PointCloud<pcl::PointXYZI>::Ptr centroidCloud (new pcl::PointCloud<pcl::PointXYZI>);// mracno s centroidy vsech clusterů
  for(int q = 0; q < m_clusters.size(); q++)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(	*m_clusters.at(q),	centroid );
    pcl::PointXYZI bod;
    bod.x =centroid.x();
    bod.y =centroid.y();
    bod.z =centroid.z();
    bod.intensity =1;
    centroidCloud->points.push_back(bod);
    m_usedCluster.push_back(false);
  }

    //zjisti vzdálenost od terenu
  pcl::KdTreeFLANN<pcl::PointXYZI> kdt;
  kdt.setInputCloud (m_terrain->get_Cloud());

  for(int r = 0; r < centroidCloud->points.size(); r++ )
  {
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    pcl::PointXYZI searchp = centroidCloud->points.at(r);

    if(kdt.radiusSearch(searchp,dist,pointIDv,pointSDv)>0)
      m_stumps.push_back(r);

    else
      m_crowns.push_back(r);
  }
  int difference = time.elapsed();
  qWarning() << "stump segment stop time: " << difference/1000 << "s";
}
bool Segmentation::isStump(int i)
{
  for(int q=0; q < m_stumps.size(); q++)
  {
    if(m_stumps.at(q) == i)
      return true;
  }
  return false;
}
void Segmentation:: stumps()// chtelo by to zrychlit
{
QTime time;
time.start();
  // pro kazdy segm ulozeny v m_stumps
  for(int a=0; a< m_stumps.size(); a++)
  {
    std::vector<int> cluster_used; // soupis vsech pouzitych clusteru
    // m_clusters.at(m_stumps.at(a) == hledany cluster
    if (m_usedCluster.at(m_stumps.at(a)) == true)
      continue;

    cluster_used.push_back(m_stumps.at(a));

    for(int g = 0; g < cluster_used.size(); g++)
    {
      if(m_usedCluster.at(cluster_used.at(g)) == true) // pokud byl už daný centroid pouzit preskoc
        continue;
      m_usedCluster.at(cluster_used.at(g)) = true;
      // zjisit sousedy a udelat z nich frontu n1
      for(int k=0; k < neighborsize; k++)
      {
        int n1 = m_centrNeighbors.at((neighborsize*cluster_used.at(g))+k);
        if(!isStump(n1)) //pokud neni v m_stumps;
          continue;

        pcl::PointXYZI nn1 = m_centroids->get_Cloud()->points.at(n1);
        //pro kazdeho souseda zjistit sousedy a ulozit do druhe fronty n2
        for(int j=0; j < neighborsize; j++)
        {
          int n2 = m_centrNeighbors.at((neighborsize*n1)+j);
          if(!isStump(n2)) //pokud neni v m_stumps;
            continue;
          if(n2 != cluster_used.at(g))
            continue;

          pcl::PointXYZI nn2 = m_centroids->get_Cloud()->points.at(n2);
          float dist = GeomCalc::computeDistance3D(nn1,nn2);// vzdalenost centroidu - MISTO TOHOTO SPOCITAT SKUTECFNOU VZDALENOST BODU!!!!!!!
          float dist2 = minClusterDistance(m_clusters.at(n2), m_clusters.at(n1) );
          //float angle =
          //vzdalenost nejblizsich bodů
          // pokud je n2 rovno bodu v centr_used
          if(dist < m_cm*2 || dist2 <= m_cm) // pokud jsou centroidy do určité vzdálenosti nebo pokud je uhel centroidu podobny tomu dosavadnimu
          //nebo pokud na sebe primo navazuji!!!!!!!!!!!!!!!!!!!!!!
            cluster_used.push_back(n1);
        }
      }
    }
    if(cluster_used.size() < 5 )
    {
      for(int q=0; q < cluster_used.size(); q++)
      {
        m_usedCluster.at(cluster_used.at(q)) = false;
      }
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustr (new pcl::PointCloud<pcl::PointXYZI>);
    for(int d=0; d< cluster_used.size(); d++)
    {
      *clustr += *m_clusters.at(cluster_used.at(d));
      clustr->width = clustr->points.size ();
      clustr->height = 1;
      clustr->is_dense = true;
    }

    // pridat tady pripojovani podle uhlu
    // pro kazdy m_clusters.at(cluster_used.at(d));
    for(int c=0; c< cluster_used.size(); c++)
    {
        // zjistit eigenvecotr clusteru
      pcl::PCA<pcl::PointXYZI> pca;
      pca.setInputCloud(clustr);
      Eigen::Matrix3f pcaEVects = pca.getEigenVectors ();
      pcl::PointXYZI p = m_centroids->get_Cloud()->points.at(cluster_used.at(c));

      for(int k=0; k < neighborsize; k++)
      {
        int n1 = m_centrNeighbors.at((neighborsize*cluster_used.at(c))+k);
        pcl::PointXYZI nn1 = m_centroids->get_Cloud()->points.at(n1);
        float dist =GeomCalc::computeDistance3D(p,nn1);
// chce to vybirat jen z stumps!!!!!!!!!!!!!
        if(m_usedCluster.at(n1) == true || dist > 4*m_cm) // pokud byl už daný centroid pouzit preskoc chce
          continue;

        float ux= p.x - nn1.x;
        float uy= p.y - nn1.y;
        float uz= p.z - nn1.z;

        float angle = std::acos ((ux*pcaEVects(0,0) + uy*pcaEVects(1,0) + uz*pcaEVects(2,0))/std::sqrt(ux*ux+uy*uy+ uz*uz)* std::sqrt(pcaEVects(1,0)*pcaEVects(1,0)+pcaEVects(2,0)*pcaEVects(2,0)+ pcaEVects(0,0)*pcaEVects(0,0))) * 180.0 / M_PI;
       // std::cout << "angle: "<< angle << std::endl;
        if(angle < 5 || angle > 175)
        {
          *clustr += *m_clusters.at(n1);
          m_usedCluster.at(n1) = true;
          cluster_used.push_back(n1);
          //pridat cluster k clustru
          // oznacit centroid jako used
         // pridat centroid k centroid_used

        }
      }
    }
    clustr->width = clustr->points.size ();
    clustr->height = 1;
    clustr->is_dense = true;

    pcl::PointXYZI minp,maxp;
    pcl::getMinMax3D(*clustr, minp,maxp);
    float veliksot = GeomCalc::computeDistance3D(minp,maxp);
    if(veliksot < 1 )
    {
      for(int q=0; q < cluster_used.size(); q++)
      {
        m_usedCluster.at(cluster_used.at(q)) = false;
      }
      clustr.reset();
      continue;
    }
//qWarning()<<"clustery spojeny ";

    m_stems.push_back(clustr);
    clustr.reset();
  }
qWarning()<<"pocet nalezenych stromu: " << m_stems.size() ;
  for(int t =0; t< m_stems.size();t++)
  {
    m_stems.at(t)->width = m_stems.at(t)->points.size ();
    m_stems.at(t)->height = 1;
    m_stems.at(t)->is_dense = true;
    m_finishedStem.push_back(false);
  }
  int difference = time.elapsed();
qWarning() << "stumps hotovo time: " << difference/1000 << "s";
}
void Segmentation::angleAdd()
{
  qWarning()<<"angle add";
  // ke kazdemu stems pridat centroidy, ktere splnuji že mají podobný úhel jako  stems centroids a jso do urcite vzdálenosti od stems.
    for(int c=0; c < 5; c++)
    {

std::cout << c << std::endl;
    for(int t =0; t< m_stems.size();t++)
    {
      // zjistit ktere clustery ho tvoři a ziskat jejich ID
      // potom pro kazdy najit sousedy a pokud nejsou ještě použití, tak zjisti uhel a pokud je v rozsahu ta přidat

      pcl::PCA<pcl::PointXYZI> pca;
      pca.setInputCloud(m_stems.at(t));
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(	*m_stems.at(t),	centroid );
      pcl::PointXYZI bod;
      bod.x =centroid.x();
      bod.y =centroid.y();
      bod.z =centroid.z();

      Eigen::Matrix3f pcaEVects = pca.getEigenVectors ();
      Eigen::Vector4f pcaMean = pca.getMean();
      Eigen::Vector3f pcaEValues = pca.getEigenValues();

  //std::cout <<"eigen vector pro strom " << t << " : =(" << pcaEVects(0,0)<< ", "<< pcaEVects(1,0)<< ", "<< pcaEVects(2,0)<< ") "<< std::endl;

      #pragma omp parallel for
      for(int r=0; r <m_centroids->get_Cloud()->points.size(); r++)
      {
        if(m_usedCluster.at(r) == true) // pokud byl už daný centroid pouzit preskoc
          continue;
        //pokud je centroid do urcite vzdalenosti a zaroven má podobný ůhel pridat k m_stems
        float distance;
        // spocitat sousedni centroidy
        pcl::KdTreeFLANN<pcl::PointXYZI> kdt;
        kdt.setInputCloud (m_stems.at(t));

        std::vector<int> pointIDv;
        std::vector<float> pointSDv;
        pcl::PointXYZI p = m_centroids->get_Cloud()->points.at(r);
        if(kdt.radiusSearch(p,3*m_cm,pointIDv,pointSDv) > 0)
        {
          float ux= p.x -centroid.x();
          float uy= p.y -centroid.y();
          float uz= p.z -centroid.z();

          float angle = std::acos ((ux*pcaEVects(0,0) + uy*pcaEVects(1,0) + uz*pcaEVects(2,0))/std::sqrt(ux*ux+uy*uy+ uz*uz)* std::sqrt(pcaEVects(1,0)*pcaEVects(1,0)+pcaEVects(2,0)*pcaEVects(2,0)+ pcaEVects(0,0)*pcaEVects(0,0))) * 180.0 / M_PI;
          std::cout << "angle: "<< angle << std::endl;
          if(angle < 10 || angle > 170)
          {
            *m_stems.at(t) += *m_clusters.at(r);
            m_usedCluster.at(r) = true;
          }
        }
      }
    }
  }

  for(int t =0; t< m_stems.size();t++)
  {
    m_stems.at(t)->width = m_stems.at(t)->points.size ();
    m_stems.at(t)->height = 1;
    m_stems.at(t)->is_dense = true;
  }
}
void Segmentation:: segmentsClean()
{
  QTime time;
time.start();
  pcl::PointXYZI minp, maxp; // body ohranicujici vegetaci
  pcl::getMinMax3D(*m_terrain->get_Cloud(),minp, maxp);
  pcl::PointXYZI minpv, maxpv; // body ohranicujici vegetaci
  pcl::getMinMax3D(*m_vegetation->get_Cloud(),minpv, maxpv);
  float h= (maxp.z + 1.3 - minpv.z)/m_cm;
  // pro kazdy segment
  for(int a =0; a < m_segments.size(); a++)
  {
    if(h > a)
      break;

    std::vector<int> used_pt;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> os (m_cm);
    os.setInputCloud(m_segments.at(a));
    os.addPointsFromInputCloud();

    #pragma omp parallel
    {
      std::vector<int> vec_private;
      #pragma omp for nowait //fill vec_private in parallel
      for(int s = 0; s < m_stems.size(); s++)
      {
        for(int d =0; d < m_stems.at(s)->points.size(); d++)
        {
          pcl::PointXYZI searchPointV = m_stems.at(s)->points.at(d);

          std::vector<int> pointIDv;
          std::vector<float> pointSDv;
          if(os.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) > 0) // poku najde urci vzdalenost a ID segmentu
          {
            for(int f=0; f< pointIDv.size(); f++)
            {
              vec_private.push_back(pointIDv.at(f));
            }
          }
        }
      }

      #pragma omp critical
      used_pt.insert(used_pt.end(), vec_private.begin(), vec_private.end());
     //

    }
    if(used_pt.size()<1)
      continue;

      pcl::PointCloud<pcl::PointXYZI>::Ptr rest (new pcl::PointCloud<pcl::PointXYZI>);
    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (used_pt));
    pcl::ExtractIndices<pcl::PointXYZI> extract;
     // Extract the inliers
    extract.setInputCloud (m_segments.at(a));
    extract.setIndices (indicesptr);
    extract.setNegative (true);
    extract.filter (*rest);
    *m_segments.at(a) = *rest;
  }
  int difference = time.elapsed();
qWarning()<< "segment clean hotovo, time: " << difference/1000 << "s";

}
float Segmentation:: minClusterDistance(pcl::PointCloud<pcl::PointXYZI>::Ptr inputA, pcl::PointCloud<pcl::PointXYZI>::Ptr inputB)
{
  float distance = 1000000;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  if(inputA->points.size() <inputB->points.size())
  {
    kdtree.setInputCloud ( inputB);
    *cloud = *inputA;
  }
  else
  {
    kdtree.setInputCloud ( inputA);
    *cloud = *inputB;
  }

  for(int c=0; c < cloud->points.size(); c++)
  {
    pcl::PointXYZI searchPointV;
    searchPointV=cloud->points.at(c);
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;

    if(kdtree.nearestKSearch(searchPointV,2,pointIDv,pointSDv) >0)
    {
      if(pointSDv.at(0) < distance*distance )
        distance = std::sqrt(pointSDv.at(0));
    }
  }
  return distance;
}
bool Segmentation::crowns(float distance)
{
QTime time;
time.start();
  bool change = false;
  for(int a =0; a < m_segments.size(); a++)
  {
//qWarning()<<"segment: " << a << " z " << m_segments.size();
    std::vector<int> used_pt;
//tady zacne blok
#pragma omp parallel
    {
      std::vector<int> vec_private;
      #pragma omp for nowait //fill vec_private in parallel
      for(int s = 0; s < m_stems.size(); s++)
      {
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> os (distance);
        os.setInputCloud(m_stems.at(s));
        os.addPointsFromInputCloud();
        std::vector<int> points;

        for(int d =0; d < m_segments.at(a)->points.size(); d++)
        {
          pcl::PointXYZI searchPointV = m_segments.at(a)->points.at(d);
          std::vector<int> pointIDv;
          std::vector<float> pointSDv;
          if(os.radiusSearch(searchPointV,distance*1.01,pointIDv,pointSDv) > 0) // poku najde urci vzdalenost a ID segmentu
          {
            points.push_back(d);
            vec_private.push_back(d);
            change = true;
          }
        }
        for(int h=0; h <points.size();h++)
        {
          m_stems.at(s)->points.push_back(m_segments.at(a)->points.at(points.at(h)));
        }
      }
      // tady zacne critical
      #pragma omp critical
      used_pt.insert(used_pt.end(), vec_private.begin(), vec_private.end());
    }
      // tady skonci
    if(used_pt.size()<1)
      continue;

//qWarning()<<"sorting";
      std::sort(used_pt.begin(), used_pt.end()); // srovnat podle velikosti
    int r = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr rest (new pcl::PointCloud<pcl::PointXYZI>);
    for(int g=0; g < m_segments.at(a)->points.size(); g++)
    {
      if( r < used_pt.size()-1 && used_pt.at(r) == g )
      {
        while( used_pt.at(r) == used_pt.at( r+1) && r < used_pt.size()-2)
        {
          r++;
        }
        r++;
        continue;
      }
      rest->points.push_back(m_segments.at(a)->points.at(g));
    }
    *m_segments.at(a) = *rest;
    rest.reset();

  }

  for(int t =0; t< m_stems.size();t++)
  {
    m_stems.at(t)->width = m_stems.at(t)->points.size ();
    m_stems.at(t)->height = 1;
    m_stems.at(t)->is_dense = true;
  }
  int difference = time.elapsed();
 qWarning()<< "crown hotovo time: " << difference;
 return change;
}
bool Segmentation::crowns(std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters)
{

  bool change = false;
  std::vector<int> pointID;
  std::vector<int> clusterID;
  std::vector<int> segmentID;

  #pragma omp parallel
  {
    std::vector<int> segment_private;
    std::vector<int> point_private;
    std::vector<int> cluster_private;
    #pragma omp for nowait //fill vec_private in parallel
    for(int a =0; a < m_segments.size(); a++)
    {
      for(int s = 0; s < clusters.size(); s++)
      {
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> os (m_cm);
        os.setInputCloud(clusters.at(s));
        os.addPointsFromInputCloud();

        for(int d =0; d < m_segments.at(a)->points.size(); d++)
        {
          pcl::PointXYZI searchPointV = m_segments.at(a)->points.at(d);
          std::vector<int> pointIDv;
          std::vector<float> pointSDv;
          if(os.radiusSearch(searchPointV,m_cm*1.001,pointIDv,pointSDv) > 0) // poku najde urci vzdalenost a ID segmentu
          {
            point_private.push_back(d);
            cluster_private.push_back(s);
            segment_private.push_back(a);
            change = true;
          }
        }
      }
    }

    #pragma omp critical
    {
      if(point_private.size() > 0)
      {
        pointID.insert(pointID.end(), point_private.begin(), point_private.end());
        clusterID.insert(clusterID.end(), cluster_private.begin(), cluster_private.end());
        segmentID.insert(segmentID.end(), segment_private.begin(), segment_private.end());
      }
    }
  }
  // naplneni noveho vectoru mracen a vymazani bodu z m_segments
  std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > clusters2;
std::vector< std::vector<int> > segments(m_segments.size());

  for(int f=0; f< clusters.size();f ++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr rest (new pcl::PointCloud<pcl::PointXYZI>);
    for(int q=0; q <pointID.size(); q++)
    {
      if(clusterID.at(q) == f)
      {
        rest->points.push_back(m_segments.at(segmentID.at(q))->points.at(pointID.at(q)));
        segments.at(segmentID.at(q)).push_back(pointID.at(q));
      }
    }
    clusters2.push_back(rest);
  }

  for(int rr=0; rr <segments.size(); rr++)
  {
    if(segments.at(rr).size() >0 && m_segments.at(rr)->points.size() >0 )
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr rest (new pcl::PointCloud<pcl::PointXYZI>);
      boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (segments.at(rr)));
      pcl::ExtractIndices<pcl::PointXYZI> extract;
       // Extract the inliers
      extract.setInputCloud (m_segments.at(rr));
      extract.setIndices (indicesptr);
      extract.setNegative (true);
      extract.filter (*rest);
      *m_segments.at(rr) = *rest;
      m_segments.at(rr)->width = m_segments.at(rr)->points.size ();
      m_segments.at(rr)->height = 1;
      m_segments.at(rr)->is_dense = true;
      rest.reset();
    }
  }

  if(recursive_count < 500 && change == true )
  {
    recursive_count++;
    if(recursive_count%50 == true)
      emit percentage(percent+=6);
    crowns(clusters2);
  }

  for(int e=0; e < clusters.size(); e++)
  {
    if(clusters2.at(e)->points.size() ==0)
    {
      m_finishedStem.at(e) = true;
      continue;
    }

    *clusters.at(e) += *clusters2.at(e);

    clusters.at(e)->width = clusters.at(e)->points.size ();
    clusters.at(e)->height = 1;
    clusters.at(e)->is_dense = true;
  }
 return change;
}

void Segmentation:: restCloud()
{
QTime time;
time.start();
pcl::PointCloud<pcl::PointXYZI>::Ptr rest (new pcl::PointCloud<pcl::PointXYZI>);

  for(int a = 0; a < m_segments.size(); a++)
  {
    if(m_segments.at(a) ->points.size() >0)
      *rest += *m_segments.at(a);
  }
  rest->width = rest->points.size();
  rest->is_dense=true;
  rest->height=1;
  m_restCloud->set_Cloud(rest);


  for(int i=0; i < m_restCloud->get_Cloud()->points.size(); i++)
  {
    m_used_rest.push_back(false);
  }
  rest.reset();
int difference = time.elapsed();
qWarning()<<"rest stop time: " << difference/1000 << "s";
}
void Segmentation:: pointsAdd(float dist)
{

QTime time;
time.start();
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> os (dist);
  os.setInputCloud(m_restCloud->get_Cloud());
  os.addPointsFromInputCloud();
  for(int w =0; w < m_stems.size(); w++)
  {

    for(int q = 0; q < m_stems.at(w)->points.size(); q++)
    {
      pcl::PointXYZI searchPointV = m_stems.at(w)->points.at(q);

      std::vector<int> pointIDv;
      std::vector<float> pointSDv;
      if(os.radiusSearch(searchPointV,dist,pointIDv,pointSDv) > 0)
      {
        for(int e=0; e < pointIDv.size(); e++)
        {
          if(m_used_rest.at(pointIDv.at(e)) == false )
          {
            m_stems.at(w)->points.push_back( m_restCloud->get_Cloud()->points.at(pointIDv.at(e)));
            m_used_rest.at(pointIDv.at(e)) = true;
          }
        }
      }
    }
    m_stems.at(w)->width = m_stems.at(w)->points.size ();
    m_stems.at(w)->height = 1;
    m_stems.at(w)->is_dense = true;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for(int r=0; r <m_restCloud->get_Cloud()->points.size();r++)
  {
    if(m_used_rest.at(r) == false)
      cloud->points.push_back(m_restCloud->get_Cloud()->points.at(r));
  }
  m_used_rest.clear();
  for(int t=0; t< cloud->points.size(); t++)
  {
    m_used_rest.push_back(false);
  }
  cloud->width = cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;
  m_restCloud->set_Cloud(cloud);
  int difference = time.elapsed();
  qWarning()<<"points add stop time: " <<difference/1000 << "s";
}
void Segmentation::getRestcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
  *output = *m_restCloud->get_Cloud();
}
int Segmentation::get_treeSize()
{
  return m_stems.size();
}
void Segmentation::getTree(int i, pcl::PointCloud<pcl::PointXYZI>::Ptr  output)
{
  if(i < m_stems.size())
    *output = *m_stems.at(i);
}

void Segmentation::getData()
{
  emit sendingRest( m_restCloud);
   qWarning()<<"rest odeslany";
  for(int i=0; i < m_stems.size(); i++)
  {

    QString name = QString("%1_%2").arg(m_treeName).arg(i);
    Cloud *c = new Cloud(m_stems.at(i), name);
    emit sendingTree( c);
  }
  emit sendingCentr( m_centroids);

  qWarning()<<"stromy odeslany";

  emit hotovo();
}



