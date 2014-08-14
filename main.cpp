#include <QtGui/QApplication>
#include <omp.h>
 #include "mainwindow.h"

 int main(int argc, char *argv[])
 {
     //Q_INIT_RESOURCE(application);

     QApplication app(argc, argv);
     app.setOrganizationName("VUKOZ");
     app.setApplicationName("3D Forest TOOL");

     MainWindow mainWin;

     mainWin.show();

     return app.exec();
 }
