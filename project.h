#ifndef PROJECT_H_INCLUDED
#define PROJECT_H_INCLUDED

#include <QtGui/QtGui>
//include BASE
#include <string>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

struct stred {
				float a;
				float b;
				float z;
					int i;
					int r;
   bool operator < (const stred& str) const
    {return (i < str.i);}
				};

struct vert{
		double x;
		double y;
		double z;
		float intensity;
		};

struct item{
    QString name;
    double x;
    double y;
    };

struct coef{
    float x;
    float y;
    float z;
    float r;
    };


class Cloud
{
protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_Cloud;
  QString m_name;
  QColor m_color;

public:
    Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col);
    Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name);
    Cloud();

    void set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void set_name(QString name);
    void set_color (QColor col);


    pcl::PointCloud<pcl::PointXYZI>::Ptr get_Cloud();
    QString get_name();
    QColor get_color();

};

class Tree : public Cloud
{
  Cloud m_dbhCloud; //cloud s doby v dbh bude mít jenom pár bodů
  stred m_dbh; //dbh
  float m_height; //tree height
  float m_lenght;
  pcl::PointXYZI m_minp,m_maxp, m_pose, m_lmax, m_lmin; //boundary box, tree position

public:
    Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col, stred s);
    Tree (Cloud cloud);

    void set_dbhCloud();
    void set_dbh(); //Hough transform
    void set_dbh(stred s);
    void set_height();
    void set_position(Cloud teren);
    void set_lenght();

    Cloud get_dbhCloud();
    float get_height();
    stred get_dbh();
    pcl::PointXYZI get_pose();
    float get_lenght();
    pcl::PointXYZI get_lpoint(bool);
};

class Project
{
    double m_x,m_y; //coordinate system transform matrix
    QString m_projectName; //project name
    QString m_path;

    std::vector<Cloud> m_baseCloud;// clouds containing all
    std::vector<Cloud> m_terrainCloud; //only terrain cloud
    std::vector<Cloud> m_vegeCloud; // only vegetation cloud
    std::vector<Cloud> m_ostCloud; // only the rest
    std::vector<Tree> m_stromy;// vector of trees

  public:
    Project();
    ~Project();
    Project( QString name);
    Project(double x, double y, QString name);

//SET
    void set_xTransform(double x);
    void set_yTransform(double y);
    void set_path(QString path);

    void save_newCloud(QString type, QString name);
    void save_newCloud(QString type, QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c );
    QString save_Cloud(QString path);
    QString save_Cloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c);
    void save_color(QString, QColor);
    void set_color(QString name, QColor col);
    void cleanAll();
    void readAtrs();
    void delete_Cloud(QString name);
    //GET
    double get_Xtransform();
    double get_Ytransform();
    QString get_Jmeno_Projektu();
    QString get_Path();
    Cloud get_Cloud(QString name);

//BASECLOUD
    void set_baseCloud(Cloud cloud);
    Cloud get_baseCloud(int i);
    int get_sizebaseCV();

//TERRAINCLOUD
    void set_TerrainCloud(Cloud cloud);
    void set_TerrainCloudat(int,Cloud);
    void set_TreeCloudat(int,Cloud);
    Cloud get_TerrainCloud(int i);
    int get_sizeTerainCV();

//VEGECLOUD
    void set_VegeCloud(Cloud cloud);
    Cloud get_VegeCloud(int i);
    int get_sizevegeCV();

//TREECLOUD
    void set_Tree(Cloud cloud);
    Tree get_TreeCloud(int i);
    int get_sizeTreeCV();

//OSTCLOUD
    void set_OstCloud(Cloud cloud);
    Cloud get_ostCloud(int i);
    int get_sizeostCV();

  private:
    void remove_file(QString name);

  //jeste pridat mazani ze souboru proj.3df a podobne funkce

};


#endif // PROJECT_H_INCLUDED
