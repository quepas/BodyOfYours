#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "scans_viewer.h"

#include <QFileSystemModel>
#include <QDebug>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <QVTKWidget.h>

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::io::loadPLYFile;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  SetupScansTree();
  scans_viewer_ = new ScansViewer(ui->scansViewer);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::SetupScansTree()
{
  QFileSystemModel* model = new QFileSystemModel;
  QString data_root_path = QDir::currentPath() + "";
  model->setRootPath(data_root_path);
  QTreeView* tree = ui->scansTree;
  tree->setModel(model);
  tree->setRootIndex(model->index(data_root_path));
  tree->setColumnWidth(0, 150);
  tree->setColumnWidth(1, 50);
  tree->setColumnWidth(2, 70);
  tree->setColumnWidth(3, 100);
}

void MainWindow::on_scansTree_doubleClicked(const QModelIndex &index)
{
  PointCloud<PointXYZ>::Ptr ply_cloud (new PointCloud<PointXYZ>);
  QFileSystemModel* model = (QFileSystemModel*) ui->scansTree->model();
  loadPLYFile(model->fileName(index).toStdString(), *ply_cloud);
  scans_viewer_->ShowPointCloud(ply_cloud);
}
