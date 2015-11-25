#include "MainWindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication app(argc,argv);
  app.setOrganizationName("JustAStartup");
  app.setApplicationName("BodyOfYours");
  MainWindow mw;
  mw.show();
  return app.exec();
}
