#include <QtGui/QApplication>
//#include <omp.h>
 #include "mainwindow.h"

 int main(int argc, char *argv[])
{

  QApplication app(argc, argv);
  Q_INIT_RESOURCE(3dforest);
  app.setOrganizationName("VUKOZ v.v.i.");
  app.setApplicationName("3D Forest - Forest lidar data processing tool");
  app.setWindowIcon(QIcon(":/images/icon.png"));

  MainWindow mainWin;
  mainWin.show();
  return app.exec();
 }
