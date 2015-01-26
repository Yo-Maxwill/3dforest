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
        float i;
        float r;
   bool operator < (const stred& str) const
    {return (i < str.i);}
				};
struct stredLSR {
				float a;
				float b;
				float r;
        float s;
        float i;
        float j;
        float g;
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
struct matrix3x3{
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float g;
    float h;
    float i;
    };

struct median{
    float x;
    float y;
    float z;
    std::vector<int> pi;
    float alfa;
    float alf_sum;
    };


class Cloud
{
protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_Cloud;
  QString m_name;
  QColor m_color;
  int m_PointSize;

public:
    Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col);
    Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name);
    Cloud();
    Cloud operator=(Cloud &kopie);

    void set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void set_name(QString name);
    void set_color (QColor col);
    void set_Psize (int p);


    pcl::PointCloud<pcl::PointXYZI>::Ptr get_Cloud();
    QString get_name();
    QColor get_color();
    int get_Psize();
};

class Tree : public Cloud
{
  Cloud *m_dbhCloud; //cloud s doby v dbh bude mít jenom pár bodů
  stred m_dbh; //dbh
  float m_height; //tree height
  float m_lenght;
  pcl::PointXYZI m_minp,m_maxp, m_pose, m_lmax, m_lmin; //boundary box, tree position

public:
    Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col, stred s);
    Tree (Cloud cloud);
    Tree operator=(Tree &kopie);
    void set_dbhCloud();
    void set_dbhCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    stred get_dbhHT(); //Hough transform
    void set_dbhLSR(); //Least Square Regression
    stred set_dbhLSRALG(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    stred set_dbhLSRGEOM(stred circ, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    float Sigma(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, stredLSR circle);
    void set_dbh(stred s);
    void set_height();
    void set_position(Cloud teren);
    void set_lenght();


    //skeletizace
    pcl::PointCloud<pcl::PointXYZI>::Ptr skeleton();
    median median_(pcl::PointXYZI input,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    median wlopInit(median med,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );
    std::vector<median> iterate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<median> med);
    pcl::PointCloud<pcl::PointXYZI>::Ptr skeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double h);
    matrix3x3 jacobi(matrix3x3 in);
    matrix3x3 covariance(pcl::PointXYZI input,pcl::PointCloud<pcl::PointXYZI>::Ptr c);
    float alfa (float a, float b, float h=0.2);
    float beta (float a, float b, float h=0.2);
    float sigma(matrix3x3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr get_dbhCloud();
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
    void set_PointSize(QString name, int p);
    void cleanAll();
    void readAtrs();
    void delete_Cloud(QString name);
    void set_Psize(QString name, int p);


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
    Tree get_TreeCloud(QString name);
    int get_sizeTreeCV();
    void set_treedbh(int i, stred x);
    void set_dbhCloud(QString name,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

//OSTCLOUD
    void set_OstCloud(Cloud cloud);
    Cloud get_ostCloud(int i);
    int get_sizeostCV();

  private:
    void remove_file(QString name);

  //jeste pridat mazani ze souboru proj.3df a podobne funkce

};


#endif // PROJECT_H_INCLUDED
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
        float i;
        float r;
   bool operator < (const stred& str) const
    {return (i < str.i);}
				};
struct stredLSR {
				float a;
				float b;
				float r;
        float s;
        float i;
        float j;
        float g;
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
struct matrix3x3{
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float g;
    float h;
    float i;
    };

struct median{
    float x;
    float y;
    float z;
    std::vector<int> pi;
    float alfa;
    float alf_sum;
    };


class Cloud
{
protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_Cloud;
  QString m_name;
  QColor m_color;
  int m_PointSize;

public:
    Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col);
    Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name);
    Cloud();
    Cloud operator=(Cloud &kopie);

    void set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void set_name(QString name);
    void set_color (QColor col);
    void set_Psize (int p);


    pcl::PointCloud<pcl::PointXYZI>::Ptr get_Cloud();
    QString get_name();
    QColor get_color();
    int get_Psize();
};

class Tree : public Cloud
{
  Cloud *m_dbhCloud; //cloud s doby v dbh bude mít jenom pár bodů
  stred m_dbh; //dbh
  float m_height; //tree height
  float m_lenght;
  pcl::PointXYZI m_minp,m_maxp, m_pose, m_lmax, m_lmin; //boundary box, tree position

public:
    Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col, stred s);
    Tree (Cloud cloud);
    Tree operator=(Tree &kopie);
    void set_dbhCloud();
    void set_dbhCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void set_dbhHT(); //Hough transform
    void set_dbhLSR(); //Least Square Regression
    stred set_dbhLSRALG(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    stred set_dbhLSRGEOM(stred circ, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    float Sigma(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, stredLSR circle);
    void set_dbh(stred s);
    void set_height();
    void set_position(Cloud teren);
    void set_lenght();


    //skeletizace
    pcl::PointCloud<pcl::PointXYZI>::Ptr skeleton();
    median median_(pcl::PointXYZI input,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    median wlopInit(median med,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );
    std::vector<median> iterate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<median> med);
    pcl::PointCloud<pcl::PointXYZI>::Ptr skeleton(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double h);
    matrix3x3 jacobi(matrix3x3 in);
    matrix3x3 covariance(pcl::PointXYZI input,pcl::PointCloud<pcl::PointXYZI>::Ptr c);
    float alfa (float a, float b, float h=0.2);
    float beta (float a, float b, float h=0.2);
    float sigma(matrix3x3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr get_dbhCloud();
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
    void set_PointSize(QString name, int p);
    void cleanAll();
    void readAtrs();
    void delete_Cloud(QString name);
    void set_Psize(QString name, int p);


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
    Tree get_TreeCloud(QString name);
    int get_sizeTreeCV();
    void set_treedbh(int i, stred x);
    void set_dbhCloud(QString name,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

//OSTCLOUD
    void set_OstCloud(Cloud cloud);
    Cloud get_ostCloud(int i);
    int get_sizeostCV();

  private:
    void remove_file(QString name);

  //jeste pridat mazani ze souboru proj.3df a podobne funkce

};


#endif // PROJECT_H_INCLUDED
