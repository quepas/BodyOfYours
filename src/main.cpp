#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  MainWindow main_window;
  main_window.setWindowTitle("BodyOfYours - 3d scanning app");
  main_window.show();

  return app.exec();
}
