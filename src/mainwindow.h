#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "scans_viewer.h"

#include <QMainWindow>
#include <QModelIndex>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void on_scansTree_doubleClicked(const QModelIndex& index);

private:
  Ui::MainWindow *ui;
  ScansViewer* scans_viewer_;

  void SetupScansTree();
};

#endif // MAINWINDOW_H
