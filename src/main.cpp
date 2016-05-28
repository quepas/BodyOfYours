#include "MainWindow.h"

#include <QApplication>
#include <QTranslator>

int main(int argc, char *argv[])
{
  QApplication app(argc,argv);
  QTranslator qtTranslator;
  qtTranslator.load("bodyofyours_pl", "lang");
  app.installTranslator(&qtTranslator);
  app.setOrganizationName("JustAStartup");
  app.setApplicationName("BodyOfYours");
  MainWindow mw;
  mw.show();
  return app.exec();
}
